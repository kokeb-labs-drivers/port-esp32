/**
 * @file i2c.hpp
 * @brief `kb::port::esp32::I2cController`: one ESP32 I2C controller (ESP-IDF v5.5 `i2c_master`,
 * synchronous mode only) that satisfies `kb::iface::I2cBus`, with bus recovery, statistics and a
 * lock per controller.
 *
 * The controller owns the SDK bus handle and one device handle whose 7-bit address follows the
 * transfer (`i2c_master_device_change_address`), so any address can be used per call. The bus is
 * created for synchronous transfers only (`trans_queue_depth` 0): ESP-IDF refuses a mix of
 * synchronous and asynchronous operations on one bus handle.
 *
 * Results: `Ok` (done; `LastFailure()` is `None`); `NoDevice` (`AddressNack`: a probe, or a NACK
 * split by an allowed probe); `Nack` (`DataNack`: a NACK, unsplit by default); `Timeout` (a
 * failure later than the wire time plus kI2cNackSlackUs: a stretch past the SCL limit or the
 * timeout); `BusError` (`Bus`: a failure between the two time bands without a probe to settle it,
 * or another bus failure; `BusStuck`: a line held low); `Busy` (the controller lock or ESP-IDF's
 * own bus lock not had; nothing sent); `InvalidArgument`; `NotReady`.
 *
 * ESP-IDF v5.5 reports a NACK, a clock-stretch or software timeout and lost arbitration of a
 * transfer with one code (`ESP_ERR_INVALID_STATE`; v6 uses `ESP_ERR_INVALID_RESPONSE`). The
 * controller classifies by time, in three bands. A timeout cannot end before the shortest limit
 * that applies: the SCL limit the hardware sets (`stretch_limit_us` or the SDK default, capped as
 * the chip caps it) or the call's timeout. A NACK cannot end after the wire time at `clock_hz` plus
 * kI2cNackSlackUs. So a failure before the earliest timeout is a NACK, one after the wire time plus
 * the slack is a timeout, and one between is ambiguous: `Bus` (unsplit), never `DataNack`, unless
 * the probe is allowed for the address. The probe (an address-only frame) is off by default and
 * enabled per address with AllowNackProbe(), because some devices NACK every poll while busy and
 * must not receive extra frames. After a timeout or bus error SDA and SCL are read; when either is
 * held low the failure is `BusStuck` and, with `I2cConfig::auto_recover`, RecoverBus() runs at
 * once.
 *
 * Time: the lock wait is at most the timeout T. ESP-IDF then waits at most T for its own bus lock
 * and at most T for each hardware command batch (up to one FIFO, 32 bytes on most chips), so a
 * transfer of n bytes on a bus that stops raising interrupts can take T x (2 + ceil(n / 32)); a
 * stretched or stuck bus ends at the SCL limit first. An opted-in probe adds at most T plus 20 ms,
 * a recovery at most 50 ms. T is at most kMaxTimeout. Every millisecond value handed to the SDK is
 * rounded up to whole RTOS ticks plus one, so a non-zero timeout never becomes a zero-tick poll.
 *
 * LastFailure() and LastError() report the calling task's own last call (CallerRecord).
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_I2C_HPP
#define KB_PORT_ESP32_I2C_HPP

#include <array>
// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstdint>
#include <span>

#include "kb/interfaces/i2c.hpp"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/rtos/mutex.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"

namespace kb::port::esp32 {

/** @brief How an I2C controller is wired and clocked. */
struct I2cConfig {
  uint8_t port{0U};           ///< Controller number, 0 to SOC_HP_I2C_NUM - 1.
  int16_t sda_pin{-1};        ///< GPIO of SDA.
  int16_t scl_pin{-1};        ///< GPIO of SCL.
  uint32_t clock_hz{100000U}; ///< SCL frequency, 1 Hz to 1 MHz (chip limits apply).
  kb::Milliseconds timeout{
      kb::Milliseconds{50U}};       ///< Lock wait and transfer limit, 1 ms to kMaxTimeout.
  bool internal_pullups{false};     ///< Weak internal pull-ups (use external 2.2 to 4.7 kOhm).
  uint8_t glitch_filter_cycles{7U}; ///< Glitch filter, in controller clock cycles.
  bool auto_recover{true};          ///< Run RecoverBus() when a stuck line is found.
  /// Longest SCL low phase (clock stretching) before the hardware ends the transfer, in
  /// microseconds; 0 keeps the SDK default (about 2 ms). Raise it for a device that stretches long
  /// (some secure elements stretch for hundreds of milliseconds); the chip caps it (ESP32: about
  /// 13 ms). It applies to every device on the controller.
  uint32_t stretch_limit_us{0U};
};

/** @brief Counters of an I2C controller since EnsureReady(). */
struct I2cStats {
  uint32_t transfers{0U};     ///< Transfers started.
  uint32_t failures{0U};      ///< Transfers that failed.
  uint32_t recoveries{0U};    ///< RecoverBus() runs.
  uint32_t lock_timeouts{0U}; ///< Calls refused with Busy.
};

/** @brief The lowest and highest 7-bit address a transfer accepts. */
inline constexpr uint8_t kI2cFirstAddress = 0x08U;
inline constexpr uint8_t kI2cLastAddress = 0x77U; ///< See kI2cFirstAddress.
/** @brief Time a failed transfer may exceed its wire time and still count as a NACK. */
inline constexpr uint32_t kI2cNackSlackUs = 1000U;

/** @brief One ESP32 I2C controller; satisfies kb::iface::I2cBus. */
class I2cController {
public:
  /** @brief Keeps `config`; nothing is touched until EnsureReady(). */
  explicit I2cController(const I2cConfig& config) noexcept : config_(config) {}
  I2cController(const I2cController&) = delete;
  I2cController& operator=(const I2cController&) = delete;
  I2cController(I2cController&&) = delete;
  I2cController& operator=(I2cController&&) = delete;
  /** @brief Releases the controller if it is up. */
  ~I2cController() noexcept;

  /**
   * @brief Creates the SDK bus once, then resets the bus (nine SCL clocks and a STOP) so a transfer
   * cut by a warm reset cannot leave a device holding SDA; idempotent. Call once before the object
   * is shared.
   */
  [[nodiscard]] kb::Status EnsureReady() noexcept;
  /** @brief START, address, `tx`, STOP. An empty `tx` sends the address only (a probe). */
  [[nodiscard]] kb::Status Write(uint8_t address, std::span<const uint8_t> tx) noexcept;
  /** @brief Write() with its own timeout (capped at kMaxTimeout) instead of the controller's. */
  [[nodiscard]] kb::Status Write(uint8_t address, std::span<const uint8_t> tx,
                                 kb::Milliseconds timeout) noexcept;
  /** @brief Writes `tx`, repeated START, reads `rx`. Both spans non-empty. */
  [[nodiscard]] kb::Status WriteRead(uint8_t address, std::span<const uint8_t> tx,
                                     std::span<uint8_t> rx) noexcept;
  /** @brief WriteRead() with its own timeout (capped at kMaxTimeout). */
  [[nodiscard]] kb::Status WriteRead(uint8_t address, std::span<const uint8_t> tx,
                                     std::span<uint8_t> rx, kb::Milliseconds timeout) noexcept;
  /** @brief START, address, reads `rx` (non-empty), STOP. */
  [[nodiscard]] kb::Status Read(uint8_t address, std::span<uint8_t> rx) noexcept;
  /** @brief Read() with its own timeout (capped at kMaxTimeout). */
  [[nodiscard]] kb::Status Read(uint8_t address, std::span<uint8_t> rx,
                                kb::Milliseconds timeout) noexcept;
  /** @brief Why the calling task's last transfer failed; `None` after a success. */
  [[nodiscard]] kb::iface::I2cFailure LastFailure() const noexcept {
    return static_cast<kb::iface::I2cFailure>(error_.Aux());
  }
  /**
   * @brief Lets the controller probe `address` once after a NACK, to split an address NACK
   * (`NoDevice`) from a data NACK (`Nack`). Off for every address by default.
   */
  [[nodiscard]] kb::Status AllowNackProbe(uint8_t address, bool allow) noexcept;
  /** @brief Nine SCL clocks and a STOP (`i2c_master_bus_reset`), then a check of both lines. */
  [[nodiscard]] kb::Status RecoverBus() noexcept;
  /** @brief Deletes the SDK bus under the lock (Busy when not had in time); EnsureReady() may
   * bring it up again. */
  [[nodiscard]] kb::Status Release() noexcept;
  /** @brief The calling task's last call: status and its esp_err_t in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return error_.Get();
  }
  /** @brief The counters. */
  [[nodiscard]] I2cStats Stats() const noexcept {
    return I2cStats{.transfers = transfers_.Get(),
                    .failures = failures_.Get(),
                    .recoveries = recoveries_.Get(),
                    .lock_timeouts = lock_timeouts_.Get()};
  }
  /** @brief The configuration. */
  [[nodiscard]] const I2cConfig& Config() const noexcept {
    return config_;
  }

private:
  enum class Op : uint8_t { Write, WriteRead, Read };
  /** One transfer as the classification sees it. */
  struct Call {
    Op op;
    uint8_t address;
    std::span<const uint8_t> tx;
    std::span<uint8_t> rx;
    uint32_t ms;
  };
  [[nodiscard]] kb::Status transfer(const Call& call) noexcept;
  [[nodiscard]] int32_t run(const Call& call) noexcept;
  [[nodiscard]] kb::Status classify(const Call& call, int32_t err, uint64_t elapsed_us) noexcept;
  [[nodiscard]] kb::iface::I2cFailure failureOf(const Call& call, int32_t err,
                                                uint64_t elapsed_us) const noexcept;
  [[nodiscard]] kb::iface::I2cFailure nackOf(const Call& call, bool ambiguous) const noexcept;
  [[nodiscard]] uint64_t wireUs(const Call& call) const noexcept;
  [[nodiscard]] bool probeAllowed(uint8_t address) const noexcept;
  [[nodiscard]] static kb::Status statusOf(kb::iface::I2cFailure failure) noexcept;
  [[nodiscard]] bool linesIdle() const noexcept;
  [[nodiscard]] kb::iface::I2cFailure recoverLocked(int32_t& err) noexcept;
  [[nodiscard]] bool configValid() const noexcept;

  I2cConfig config_;
  kb::rtos::Mutex mutex_;
  CallerRecord error_;
  Counter transfers_;
  Counter failures_;
  Counter recoveries_;
  Counter lock_timeouts_;
  std::array<std::atomic<uint32_t>, 4> probe_mask_{};
  uint8_t device_address_{0U}; // under mutex_
  uint32_t stretch_us_{0U};    // the SCL limit the hardware applies (set by EnsureReady)
  std::atomic<bool> ready_{false};
};

static_assert(kb::iface::I2cBus<I2cController>, "I2cController satisfies kb::iface::I2cBus");

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_I2C_HPP
