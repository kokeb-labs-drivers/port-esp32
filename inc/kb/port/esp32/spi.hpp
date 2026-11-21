/**
 * @file spi.hpp
 * @brief `kb::port::esp32::SpiBus` (one SPI host: pins, DMA, the bus lock) and
 * `kb::port::esp32::SpiDevice` (one chip select on it), which satisfies `kb::iface::SpiDevice` and
 * `kb::iface::SpiClockControl`. ESP-IDF v5.5 `spi_master`, polling transactions.
 *
 * A Transfer() takes the bus lock (at most `SpiDeviceConfig::timeout`, T), clocks the frame as
 * one transaction with chip select held throughout, and returns. A frame is at most one
 * transaction: kSpiMaxFrameBytes (64) without DMA, and with `SpiBusConfig::dma` the host's DMA
 * buffer, `dma_buffer_bytes` (at most kSpiMaxDmaFrameBytes, 4092). SpiBus::MaxFrameBytes() gives
 * the limit; a longer frame is refused with `InvalidArgument` (never split into several
 * transactions).
 *
 * DMA (docs/dma.md). Each DMA host has one port-owned staging buffer (a transmit and a receive
 * half): the board's static `SpiStaging` given in `SpiBusConfig::staging` (the default the examples
 * use), or, without one, a block from `heap_caps_aligned_alloc` taken once by
 * SpiBus::EnsureReady(). EnsureReady() checks that it is DMA-capable and aligned. Every DMA frame
 * is copied in and out of it under the bus lock, so any caller buffer works (unaligned, odd length,
 * not DMA-capable), the DMA's rounding of receive lengths to whole words lands in port memory, and
 * after a failed host (below) the hardware can only ever write port memory: the staging stays held
 * (quarantined) until the host is torn down. Every DMA transaction carries
 * `SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL`, so ESP-IDF refuses instead of allocating a bounce buffer:
 * nothing is allocated after start. On the ESP32-P4 (internal RAM behind a 64-byte cache) a DMA
 * receive must be whole lines in length; a shorter one is refused with `InvalidArgument`
 * (SpiBusLimits() says so at build time). Without DMA the CPU copies through the FIFO. `tx` and
 * `rx` must not overlap. Zero-copy transfers of a typed `kb::dma::Buffer` go through
 * `SpiDmaChannel` (spi_dma.hpp).
 *
 * Fallback. When the chip has no free DMA channel (`ESP_ERR_NOT_FOUND`, the GDMA pairs of the
 * ESP32-C6 are three), EnsureReady() fails, unless `SpiBusConfig::dma_fallback` allows the FIFO
 * path: then the bus runs without DMA, Capabilities() and Health() say `Polled` with the 64-byte
 * ceiling, and LastError() keeps `ESP_ERR_NOT_FOUND`. The fallback is never silent.
 *
 * Bounded time. The port waits for the end of the transaction for its wire time W (the frame's
 * bits at the clock the SDK actually set, plus the chip-select setup and hold cycles) plus T. When
 * it has not ended by then the port waits once more, W plus T: the master drives SCLK, so the
 * hardware finishes, and the frame is reported `Timeout` (the data is complete, the deadline was
 * missed; a retry repeats the frame). When it still has not ended the host is marked failed: the
 * transaction stays with ESP-IDF, and every later call on any device of that host returns `Fault`
 * at once without touching the SDK (Release() still asks ESP-IDF, which refuses the stuck device),
 * until the application restarts. A Transfer() returns within T + 2 x (W + T) plus up to five RTOS
 * ticks; a frame for which that exceeds kMaxTimeout is refused, and EnsureReady() refuses a T for
 * which even a one-byte frame would (T up to about 660 ms at a 1 kHz tick). The waits for the end
 * spin the CPU (ESP-IDF's polling mode); keep T short for high-priority tasks.
 *
 * ESP-IDF v5.5 takes only "wait for ever" for the bus lock of a polling transaction. The port's own
 * bus lock serialises every device of the host, the port initialises the host itself
 * (EnsureReady() fails when another component already did), and a host with an unfinished
 * transaction is never used again, so that SDK lock is always free when the port asks for it.
 * Requirement: no other component may add a device to a host the port owns. The port never calls
 * `spi_device_acquire_bus`.
 *
 * Results: `Ok` (clocked); `InvalidArgument` (bad spans, a frame longer than MaxFrameBytes() or
 * beyond kMaxTimeout, a bad configuration); `Busy` (the bus lock not had within T, or a pin, host
 * or device slot taken; nothing clocked); `Timeout` (ended later than W + T: complete, but late);
 * `Fault` (the host is failed, or no memory for the staging buffer); `NotReady` (before
 * EnsureReady()); `Unsupported` from EnsureReady() when no DMA channel is free and `dma_fallback`
 * is off (`ESP_ERR_NOT_FOUND`).
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_SPI_HPP
#define KB_PORT_ESP32_SPI_HPP

// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/dma/capabilities.hpp"
#include "kb/interfaces/spi.hpp"
#include "kb/port/esp32/dma.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/rtos/mutex.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"

namespace kb::port::esp32 {

/** @brief How an SPI host is wired. */
struct SpiBusConfig {
  uint8_t host{1U};     ///< `spi_host_device_t`: 1 is SPI2_HOST, 2 is SPI3_HOST (where it exists).
  int16_t mosi_pin{-1}; ///< GPIO of MOSI, or -1 when unused.
  int16_t miso_pin{-1}; ///< GPIO of MISO, or -1 when unused.
  int16_t sclk_pin{-1}; ///< GPIO of SCLK.
  bool dma{false};      ///< Use a DMA channel and the staging buffer (see the file comment).
  /// With `dma` and no `staging`: the longest frame of the host, kSpiMinDmaBufferBytes to
  /// kSpiMaxDmaFrameBytes. The port allocates twice this (transmit and receive), DMA-capable and
  /// aligned, once in EnsureReady(). Ignored when `staging` is given (its frame is the limit).
  uint16_t dma_buffer_bytes{static_cast<uint16_t>(kSpiMaxDmaFrameBytes)};
  /// With `dma`: the board's static staging buffer (`SpiStaging<N>::Ref()`); empty: allocate once.
  SpiStagingRef staging{};
  /// The interrupt level of the host: 0 for the ESP-IDF default, 1 to 3 (`InterruptLevelOf` of the
  /// board's `kb::dma::StreamAssignment`). ESP-IDF allocates exactly that level or fails.
  uint8_t interrupt_level{0U};
  /// With `dma`: run without DMA (reported, see the file comment) when no DMA channel is free.
  bool dma_fallback{false};
};

/** @brief One device on an SpiBus. */
struct SpiDeviceConfig {
  int16_t cs_pin{-1}; ///< GPIO of the chip select (driven by the peripheral), or -1.
  uint8_t mode{0U};   ///< SPI mode 0 to 3 (CPOL, CPHA).
  uint32_t clock_hz{
      1000000U}; ///< SCLK frequency, up to 80 MHz (IO MUX pins) or 26 MHz (GPIO matrix).
  /// Bus lock wait, and the margin over the frame's wire time: 1 ms to about 660 ms (3 T plus two
  /// one-byte wire times and the tick rounding within kMaxTimeout).
  kb::Milliseconds timeout{kb::Milliseconds{20U}};
  uint8_t cs_setup_cycles{0U}; ///< SPI clock cycles between CS assert and the first edge (0 to 16).
  uint8_t cs_hold_cycles{0U}; ///< SPI clock cycles CS stays asserted after the last edge (0 to 16).
};

/** @brief Counters of an SPI device. */
struct SpiStats {
  uint32_t transfers{0U};     ///< Frames started.
  uint32_t failures{0U};      ///< Frames that failed.
  uint32_t late{0U};          ///< Frames that ended after W + T (reported Timeout).
  uint32_t lock_timeouts{0U}; ///< Calls refused with Busy.
};

/** @brief One SPI host: initialises the bus and serialises its devices. */
class SpiBus {
public:
  /** @brief Keeps `config`. */
  explicit SpiBus(const SpiBusConfig& config) noexcept : config_(config) {}
  SpiBus(const SpiBus&) = delete;
  SpiBus& operator=(const SpiBus&) = delete;
  SpiBus(SpiBus&&) = delete;
  SpiBus& operator=(SpiBus&&) = delete;
  /** @brief Frees the bus if it is up. */
  ~SpiBus() noexcept;

  /** @brief Claims the pins and calls `spi_bus_initialize` once; idempotent. Call once before the
   * object is shared. */
  [[nodiscard]] kb::Status EnsureReady() noexcept;
  /** @brief `spi_bus_free` under the bus lock; every device must be released first. */
  [[nodiscard]] kb::Status Release() noexcept;
  /** @brief The calling task's last call: status and its esp_err_t in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return error_.Get();
  }
  /** @brief The configuration. */
  [[nodiscard]] const SpiBusConfig& Config() const noexcept {
    return config_;
  }
  /** @brief True after EnsureReady() succeeded. */
  [[nodiscard]] bool IsReady() const noexcept {
    return ready_.load();
  }
  /** @brief True when a transaction on the host never ended: every device call returns Fault. */
  [[nodiscard]] bool IsFailed() const noexcept {
    return failed_.load();
  }
  /** @brief Marks the host failed (called by a device, holding the bus lock). */
  void MarkFailed() noexcept {
    failed_.store(true);
  }
  /** @brief The longest frame of a device on this host: kSpiMaxFrameBytes, or with DMA the
   * staging buffer's frame. */
  [[nodiscard]] std::size_t MaxFrameBytes() const noexcept {
    return dma_active_ ? frame_bytes_ : kSpiMaxFrameBytes;
  }
  /** @brief True when the host moves frames by DMA (false before EnsureReady() and after a
   * fallback). */
  [[nodiscard]] bool DmaActive() const noexcept {
    return dma_active_;
  }
  /** @brief What the host offers now: mode, ceiling, staging size, alignment unit. */
  [[nodiscard]] kb::dma::Capabilities Capabilities() const noexcept;
  /** @brief The DMA part of a handler's health record: mode, failed host, held staging. */
  [[nodiscard]] kb::dma::DmaHealth Health() const noexcept;

  /** @brief Takes the bus lock, waiting at most `timeout` (devices hold it for one frame). */
  [[nodiscard]] bool TryLockFor(kb::Milliseconds timeout) noexcept {
    return mutex_.TryLockFor(timeout);
  }
  /** @brief Gives the bus lock back. */
  void Unlock() noexcept {
    mutex_.Unlock();
  }

private:
  friend class SpiDevice;  // uses the staging buffer under the bus lock
  friend class SpiDmaCore; // counts zero-copy errors and quarantines under the bus lock
  [[nodiscard]] bool configValid() const noexcept;
  [[nodiscard]] int32_t open() noexcept;
  [[nodiscard]] int32_t openStaging() noexcept;
  void closeStaging() noexcept;

  SpiBusConfig config_;
  kb::rtos::Mutex mutex_;
  CallerRecord error_;
  std::span<uint8_t> dma_tx_; // the staging buffer, transmit half (empty without DMA)
  std::span<uint8_t> dma_rx_; // receive half
  std::size_t frame_bytes_{kSpiMaxFrameBytes};
  bool dma_active_{false};   // written by EnsureReady() before ready_, read after it
  bool heap_staging_{false}; // the staging came from the heap (freed by Release())
  bool staging_held_{false}; // a transaction that never ended still owns the staging
  Counter quarantines_;
  Counter dma_errors_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> failed_{false};
};

/** @brief One chip select on an SpiBus; satisfies kb::iface::SpiDevice and SpiClockControl. */
class SpiDevice {
public:
  /** @brief Refers to `bus`, which must outlive the device. */
  SpiDevice(SpiBus& bus, const SpiDeviceConfig& config) noexcept : bus_(bus), config_(config) {}
  SpiDevice(const SpiDevice&) = delete;
  SpiDevice& operator=(const SpiDevice&) = delete;
  SpiDevice(SpiDevice&&) = delete;
  SpiDevice& operator=(SpiDevice&&) = delete;
  /** @brief Removes the device from the bus if it was added. */
  ~SpiDevice() noexcept;

  /** @brief Brings the bus up, claims the chip-select pin and a device slot, adds the device;
   * idempotent. */
  [[nodiscard]] kb::Status EnsureReady() noexcept;
  /** @brief One full-duplex frame with chip select held (see the file comment). */
  [[nodiscard]] kb::Status Transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) noexcept;
  /** @brief Changes SCLK: the device is removed and added again with the new clock. */
  [[nodiscard]] kb::Status SetClockHz(uint32_t hz) noexcept;
  /** @brief Removes the device from the bus (under the bus lock). */
  [[nodiscard]] kb::Status Release() noexcept;
  /** @brief The calling task's last call: status and its esp_err_t in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return error_.Get();
  }
  /** @brief The counters. */
  [[nodiscard]] SpiStats Stats() const noexcept {
    return SpiStats{.transfers = transfers_.Get(),
                    .failures = failures_.Get(),
                    .late = late_.Get(),
                    .lock_timeouts = lock_timeouts_.Get()};
  }
  /** @brief The SCLK frequency the SDK set (at most the asked one), 0 before EnsureReady(). */
  [[nodiscard]] uint32_t ActualClockHz() const noexcept {
    return actual_hz_;
  }
  /** @brief The configuration (with the current clock). */
  [[nodiscard]] const SpiDeviceConfig& Config() const noexcept {
    return config_;
  }

private:
  friend class SpiDmaCore; // zero-copy transactions of this device (spi_dma.hpp)
  [[nodiscard]] kb::Status addLocked(uint32_t hz) noexcept;
  [[nodiscard]] kb::Status removeLocked() noexcept;
  [[nodiscard]] kb::Status frameLocked(std::span<const uint8_t> tx, std::span<uint8_t> rx) noexcept;
  [[nodiscard]] int32_t endLocked(uint32_t wait, bool& failed) noexcept;
  [[nodiscard]] kb::Status checkFrame(std::span<const uint8_t> tx,
                                      std::span<uint8_t> rx) const noexcept;
  [[nodiscard]] uint32_t waitMs(std::size_t bytes) const noexcept;
  [[nodiscard]] bool configValid() const noexcept;

  SpiBus& bus_;
  SpiDeviceConfig config_;
  CallerRecord error_;
  Counter transfers_;
  Counter failures_;
  Counter late_;
  Counter lock_timeouts_;
  uint32_t actual_hz_{0U};
  uint8_t slot_{0U};
  bool slot_claimed_{false};
  std::atomic<bool> added_{false};
};

static_assert(kb::iface::SpiDevice<SpiDevice>, "SpiDevice satisfies kb::iface::SpiDevice");
static_assert(kb::iface::SpiClockControl<SpiDevice>, "SpiDevice satisfies SpiClockControl");

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_SPI_HPP
