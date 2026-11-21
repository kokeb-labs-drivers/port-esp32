/**
 * @file spi_dma.hpp
 * @brief `kb::port::esp32::SpiDmaChannel<Region>`: zero-copy, asynchronous SPI DMA transfers of a
 * typed `kb::dma::Buffer` on one SpiDevice, through a `kb::dma::Lease` (Start, Poll, Cancel,
 * Recover). The synchronous `SpiDevice::Transfer` copies through the host's staging buffer; this is
 * the only path on which the DMA reads or writes memory the caller owns, and that memory is then a
 * `kb::dma::Buffer` lent to the port.
 *
 * One transfer at a time, one direction per transfer: `ToDevice` clocks the buffer out (no MISO
 * phase), `FromDevice` clocks the bus with no MOSI phase and receives into the buffer. The
 * transfer holds the bus lock from Start() until it is settled (Poll() reports it, Cancel() or the
 * deadline ends it), so the lease's scope is inside the bus hold. Start, Poll, Cancel and Recover
 * are called from one task (the owner of the channel).
 *
 * | Call | Result |
 * | --- | --- |
 * | `Start` | `Ok`; `OutOfRange` (above the host's ceiling); `Unsupported` (the host has no DMA);
 * `InvalidArgument` (a receive not in whole cache lines on the ESP32-P4, or the lend refused);
 * `Busy` (a transfer is out, or the bus lock was not had); `Fault` (the host is failed, or the
 * buffer is quarantined); `NotReady` (the device is not up) |
 * | `Poll` | `Pending`; `Done` once with `Ok`, `Fault` (DMA overflow or underflow, the flags in
 * `flags`) or `Timeout` (the deadline passed: the port waited once more for the end, see below);
 * `Idle` when nothing is out |
 * | `Cancel` | `Ok` once the hardware has stopped (ESP-IDF reported the transaction finished);
 * `Fault` when it did not within the frame's wire time plus the device timeout: the buffer is
 * quarantined and the host failed |
 * | `Recover` | `Ok` when the stop is now proven (the transaction finished meanwhile, or the host
 * was reset); `Fault` otherwise (on ESP-IDF v5.5 a never-ending transaction needs a restart) |
 *
 * **No abort on ESP-IDF.** A started SPI transaction cannot be stopped: a cancel or an overdue
 * transfer waits once more, for the frame's wire time W plus the device timeout T, for ESP-IDF to
 * report the end. Only that report returns the memory (`kb::dma::StopEvidence::Confirmed`); without
 * it the lease ends quarantined and the buffer stays unusable until Recover() proves the stop.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_SPI_DMA_HPP
#define KB_PORT_ESP32_SPI_DMA_HPP

#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/dma/buffer.hpp"
#include "kb/dma/capabilities.hpp"
#include "kb/dma/completion.hpp"
#include "kb/dma/region.hpp"
#include "kb/port/esp32/dma.hpp"
#include "kb/port/esp32/spi.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"

namespace kb::port::esp32 {

/** @brief Where a zero-copy transfer stands. */
enum class SpiDmaState : uint8_t {
  Idle = 0,    ///< Nothing out, nothing to report.
  Pending = 1, ///< In flight.
  Done = 2,    ///< Finished; reported once.
};

/** @brief One answer of `SpiDmaChannel::Poll`. */
struct SpiDmaResult {
  SpiDmaState state{SpiDmaState::Idle};
  kb::Status status{kb::Status::Ok}; ///< Valid when `Done`.
  std::size_t count{0U};             ///< Bytes of the transfer (valid when `Done`).
  uint32_t flags{0U}; ///< The raw error flags of a failed transfer (`spi_dma_flags`).
};

/** @brief The raw error flags `SpiDmaResult::flags` carries. */
namespace spi_dma_flags {
inline constexpr uint32_t kComplete = 1U;      ///< The transaction ran to its end.
inline constexpr uint32_t kTransferError = 2U; ///< ESP-IDF reported a DMA overflow or underflow.
inline constexpr uint32_t kFifoError = 4U;     ///< The same event, as a FIFO error.
} // namespace spi_dma_flags

/** @brief The counters of one channel. */
struct SpiDmaStats {
  uint32_t starts{0U};          ///< Transfers started.
  uint32_t completions{0U};     ///< Transfers that ended Ok.
  uint32_t transfer_errors{0U}; ///< Transfers that ended with a DMA error.
  uint32_t fifo_errors{0U};     ///< FIFO errors among them.
  uint32_t timeouts{0U};        ///< Transfers reported Timeout.
  uint32_t cancels{0U};         ///< Cancel() calls that found a transfer.
  uint32_t quarantines{0U};     ///< Leases ended quarantined.
};

/**
 * @brief The non-template part of SpiDmaChannel: the SDK calls, the bus lock and the host's state.
 * Use SpiDmaChannel.
 */
class SpiDmaCore {
public:
  /** @brief Serves `device`, which must outlive the core. */
  explicit SpiDmaCore(SpiDevice& device) noexcept : device_(device) {}
  SpiDmaCore(const SpiDmaCore&) = delete;
  SpiDmaCore& operator=(const SpiDmaCore&) = delete;
  SpiDmaCore(SpiDmaCore&&) = delete;
  SpiDmaCore& operator=(SpiDmaCore&&) = delete;
  ~SpiDmaCore() = default;

  /** @brief Whether a transfer of `used` bytes in `dir` may start (see the Start results). */
  [[nodiscard]] kb::Status Admit(std::size_t used, kb::dma::Direction dir) const noexcept;
  /** @brief Takes the bus lock within the device's timeout. */
  [[nodiscard]] bool Lock() noexcept;
  /** @brief Gives the bus lock back. */
  void Unlock() noexcept;
  /** @brief Queues the DMA transaction over `memory` (the leased buffer); the esp_err_t. */
  [[nodiscard]] int32_t Begin(std::span<std::byte> memory, kb::dma::Direction dir,
                              std::size_t used) noexcept;
  /** @brief The transaction's end, waiting at most `wait_ms`; the esp_err_t (ESP_ERR_TIMEOUT: in
   * flight), the raw flags in `flags`. */
  [[nodiscard]] int32_t Result(uint32_t wait_ms, uint32_t& flags) const noexcept;
  /** @brief W + T of a frame of `used` bytes, in milliseconds. */
  [[nodiscard]] uint32_t EndWaitMs(std::size_t used) const noexcept;
  /** @brief Marks the host failed after a stop that was not confirmed (counted). */
  void MarkFailed() noexcept;
  /** @brief Counts a transfer that ended with a DMA error. */
  void NoteError() noexcept;
  /** @brief Proof that a quarantined transfer stopped: the transaction finished meanwhile, or the
   * host was reset. Clears the failed state when proven. */
  [[nodiscard]] kb::dma::StopEvidence Settle() noexcept;
  /** @brief The host's capabilities. */
  [[nodiscard]] kb::dma::Capabilities Capabilities() const noexcept;
  /** @brief The host's DMA health. */
  [[nodiscard]] kb::dma::DmaHealth Health() const noexcept;

private:
  SpiDevice& device_;
};

/**
 * @brief Zero-copy SPI DMA of `kb::dma::Buffer`s in region `R` on one device (see the file note).
 *
 * @tparam R The buffers' region: `InternalRam`, or `InternalRamL1` on the ESP32-P4 (the port's
 * cache hooks run at every lease transition of a `Maintained` region).
 */
template <kb::dma::Region R>
class SpiDmaChannel {
  static_assert(R::kAlignBytes >= 4U, "SPI DMA buffers are padded to whole words at least (the "
                                      "receive descriptor rounds up to whole words)");

public:
  /// The buffers' region.
  using RegionType = R;

  /** @brief A channel on `device`, which must outlive it. */
  explicit SpiDmaChannel(SpiDevice& device) noexcept : core_(device) {}
  SpiDmaChannel(const SpiDmaChannel&) = delete;
  SpiDmaChannel& operator=(const SpiDmaChannel&) = delete;
  SpiDmaChannel(SpiDmaChannel&&) = delete;
  SpiDmaChannel& operator=(SpiDmaChannel&&) = delete;
  /** @brief Cancels a transfer still out (a stop that is not confirmed quarantines the buffer). */
  ~SpiDmaChannel() noexcept {
    static_cast<void>(Cancel());
  }

  /**
   * @brief Lends `buffer` for a transfer of `used` bytes in `dir` and starts it; `deadline` bounds
   * it (see Poll).
   */
  template <std::size_t Bytes>
  [[nodiscard]] kb::Status Start(kb::dma::Buffer<R, Bytes>& buffer, kb::dma::Direction dir,
                                 std::size_t used, kb::utils::Deadline deadline) noexcept {
    kb::Status status = lease_.Active() ? kb::Status::Busy : core_.Admit(used, dir);
    if ((status == kb::Status::Ok) && !core_.Lock()) {
      status = kb::Status::Busy;
    }
    if (status == kb::Status::Ok) {
      status = buffer.Lend(key(), dir, used, ops_, lease_);
      if (status == kb::Status::Ok) {
        const int32_t err = core_.Begin(lease_.DeviceView(key()), dir, used);
        if (err == esp_err::kOk) {
          deadline_ = deadline;
          used_ = used;
          ++stats_.starts;
        } else {
          // Refused before the hardware saw it: the memory never left the port's hands.
          static_cast<void>(lease_.Abort(key(), ops_, kb::dma::StopEvidence::Confirmed));
          status = (err == esp_err::kInvalidArg) ? kb::Status::InvalidArgument : MapEspErr(err);
        }
      }
      if (status != kb::Status::Ok) {
        core_.Unlock();
      }
    }
    return status;
  }

  /** @brief Reports a finished transfer once; ends an overdue one as `Timeout`. */
  [[nodiscard]] SpiDmaResult Poll(kb::Instant now) noexcept {
    SpiDmaResult out{};
    if (lease_.Active()) {
      uint32_t flags = 0U;
      const int32_t err = core_.Result(0U, flags);
      if (err != esp_err::kTimeout) {
        out = finish(err, flags, kb::Status::Ok);
      } else if (deadline_.Expired(now)) {
        out = overdue();
      } else {
        out.state = SpiDmaState::Pending;
      }
    }
    return out;
  }

  /** @brief Ends the transfer out: `Ok` only once the hardware has stopped (see the file note). */
  [[nodiscard]] kb::Status Cancel() noexcept {
    kb::Status status = kb::Status::Ok;
    if (lease_.Active()) {
      ++stats_.cancels;
      uint32_t flags = 0U;
      const int32_t err = core_.Result(core_.EndWaitMs(used_), flags);
      if (err == esp_err::kTimeout) {
        status = quarantine();
      } else {
        status = lease_.Abort(key(), ops_, kb::dma::StopEvidence::Confirmed);
        core_.Unlock();
      }
    }
    return status;
  }

  /** @brief Returns a quarantined `buffer` once the stop is proven (see the file note). */
  template <std::size_t Bytes>
  [[nodiscard]] kb::Status Recover(kb::dma::Buffer<R, Bytes>& buffer) noexcept {
    kb::Status status = kb::Status::InvalidArgument;
    if (buffer.Current() == kb::dma::BufferState::Quarantined) {
      status = buffer.Recover(key(), ops_, core_.Settle());
      if (status == kb::Status::Ok) {
        faulted_ = false;
      }
    }
    return status;
  }

  /** @brief What the channel offers now (the host's mode and ceiling). */
  [[nodiscard]] kb::dma::Capabilities Capabilities() const noexcept {
    return core_.Capabilities();
  }
  /** @brief The DMA part of a handler's health record. */
  [[nodiscard]] kb::dma::DmaHealth Health() const noexcept {
    kb::dma::DmaHealth h = core_.Health();
    h.faulted = h.faulted || faulted_;
    return h;
  }
  /** @brief The counters. */
  [[nodiscard]] SpiDmaStats Stats() const noexcept {
    return stats_;
  }
  /** @brief The cache hooks' refused syncs (a `Maintained` region only). */
  [[nodiscard]] uint32_t CacheFailures() const noexcept {
    return ops_.Failures();
  }

private:
  [[nodiscard]] kb::dma::PortKey<SpiDmaChannel> key() const noexcept {
    return kb::dma::PortKey<SpiDmaChannel>{};
  }

  [[nodiscard]] SpiDmaResult finish(int32_t err, uint32_t flags, kb::Status late) noexcept {
    const bool error = (err != esp_err::kOk) || ((flags & spi_dma_flags::kTransferError) != 0U);
    static_cast<void>(
        lease_.Complete(key(), ops_, error ? kb::dma::Outcome::Error : kb::dma::Outcome::Ok));
    core_.Unlock();
    SpiDmaResult out{};
    out.state = SpiDmaState::Done;
    out.count = used_;
    out.flags = error ? flags : 0U;
    out.status = error ? kb::Status::Fault : late;
    if (error) {
      ++stats_.transfer_errors;
      if ((flags & spi_dma_flags::kFifoError) != 0U) {
        ++stats_.fifo_errors;
      }
      core_.NoteError();
    } else if (late == kb::Status::Ok) {
      ++stats_.completions;
    } else {
      ++stats_.timeouts;
    }
    return out;
  }

  // The deadline passed: wait once more for the end (the SDK cannot abort), then settle.
  [[nodiscard]] SpiDmaResult overdue() noexcept {
    uint32_t flags = 0U;
    const int32_t err = core_.Result(core_.EndWaitMs(used_), flags);
    SpiDmaResult out{};
    if (err == esp_err::kTimeout) {
      static_cast<void>(quarantine());
      ++stats_.timeouts;
      out.state = SpiDmaState::Done;
      out.status = kb::Status::Timeout;
      out.count = used_;
    } else {
      out = finish(err, flags, kb::Status::Timeout);
    }
    return out;
  }

  [[nodiscard]] kb::Status quarantine() noexcept {
    const kb::Status status = lease_.Abort(key(), ops_, kb::dma::StopEvidence::NotConfirmed);
    core_.MarkFailed();
    core_.Unlock();
    faulted_ = true;
    ++stats_.quarantines;
    return status;
  }

  SpiDmaCore core_;
  EspCacheOps ops_{};
  kb::dma::Lease<R> lease_{};
  kb::utils::Deadline deadline_;
  std::size_t used_{0U};
  SpiDmaStats stats_{};
  bool faulted_{false};
};

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_SPI_DMA_HPP
