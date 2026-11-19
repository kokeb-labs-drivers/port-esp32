/**
 * @file can.hpp
 * @brief `kb::port::esp32::CanController`: one TWAI controller (ESP-IDF v5.5 `twai` driver,
 * handle API `twai_*_v2`), which satisfies `kb::iface::CanBus`. Classic CAN 2.0A/B frames.
 *
 * Send() waits at most its timeout for a slot in the SDK transmit queue; Receive() waits at most
 * its timeout for a frame in the receive queue (every timeout capped at kMaxTimeout). The SDK
 * queues are safe to use from several tasks, so Send() and Receive() take no lock of their own;
 * they are counted while they run, and Release() first refuses new calls, then waits (at most
 * kMaxTimeout plus 100 ms) for the running ones to return before it uninstalls the driver, and
 * returns `Busy` (leaving the controller up) when they do not. A bus-off controller refuses Send()
 * with `BusError`; Recover() starts the bus-off recovery (128 x 11 recessive bits) and restarts it.
 * LastError() reports the calling task's own last call.
 *
 * | Result | Meaning |
 * | --- | --- |
 * | `Ok` | Queued (Send) or taken (Receive) |
 * | `Timeout` | No transmit slot or no frame within the timeout |
 * | `BusError` | Bus-off or the controller is not running. Detail: the esp_err_t |
 * | `InvalidArgument` | dlc above 8, an identifier too wide, or a bit rate without a timing |
 * | `Unsupported` | The chip has no TWAI controller (or not this one) |
 * | `NotReady` | EnsureReady() has not succeeded |
 *
 * Supported bit rates: 25k, 50k, 100k, 125k, 250k, 500k, 800k (where the chip can) and 1M bit/s.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_CAN_HPP
#define KB_PORT_ESP32_CAN_HPP

// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstdint>

#include "kb/interfaces/can.hpp"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/rtos/mutex.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"

namespace kb::port::esp32 {

/** @brief Operating mode of a TWAI controller. */
enum class CanMode : uint8_t {
  Normal = 0,     ///< Sends, receives and acknowledges.
  NoAck = 1,      ///< Sends without needing an acknowledgement (self test with one node).
  ListenOnly = 2, ///< Receives only; never drives the bus.
};

/** @brief The state of a CAN controller. */
enum class CanState : uint8_t {
  Stopped = 0,    ///< Installed, not on the bus.
  Running = 1,    ///< Error active or error passive, on the bus.
  BusOff = 2,     ///< Off the bus after too many errors; Recover() needed.
  Recovering = 3, ///< Bus-off recovery in progress.
};

/** @brief Error counters and state of a CAN controller. */
struct CanStatus {
  CanState state{CanState::Stopped}; ///< State.
  uint32_t tx_error_counter{0U};     ///< TEC.
  uint32_t rx_error_counter{0U};     ///< REC.
  uint32_t tx_failed{0U};            ///< Frames the SDK could not send.
  uint32_t rx_missed{0U};            ///< Frames lost because the receive queue was full.
  uint32_t bus_errors{0U};           ///< Bus errors seen.
};

/** @brief How a TWAI controller is wired and timed. */
struct CanConfig {
  uint8_t controller{0U};        ///< TWAI controller, 0 to SOC_TWAI_CONTROLLER_NUM - 1.
  int16_t tx_pin{-1};            ///< GPIO to the transceiver's TXD.
  int16_t rx_pin{-1};            ///< GPIO from the transceiver's RXD.
  uint32_t bitrate{500000U};     ///< Bits per second (see the file comment).
  CanMode mode{CanMode::Normal}; ///< Mode.
  uint8_t tx_queue{8U};          ///< SDK transmit queue length.
  uint8_t rx_queue{16U};         ///< SDK receive queue length.
};

/** @brief One TWAI controller; satisfies kb::iface::CanBus. */
class CanController {
public:
  /** @brief Keeps `config`. */
  explicit CanController(const CanConfig& config) noexcept : config_(config) {}
  CanController(const CanController&) = delete;
  CanController& operator=(const CanController&) = delete;
  CanController(CanController&&) = delete;
  CanController& operator=(CanController&&) = delete;
  /** @brief Stops and uninstalls the controller if it is up. */
  ~CanController() noexcept;

  /** @brief Installs and starts the controller once; idempotent. */
  [[nodiscard]] kb::Status EnsureReady() noexcept;
  /** @brief Queues `frame`, waiting at most `timeout` for a slot. */
  [[nodiscard]] kb::Status Send(const kb::iface::CanFrame& frame,
                                kb::Milliseconds timeout) noexcept;
  /** @brief Takes the oldest received frame, waiting at most `timeout`. */
  [[nodiscard]] kb::Status Receive(kb::iface::CanFrame& frame, kb::Milliseconds timeout) noexcept;
  /** @brief Counters and state from the SDK. */
  [[nodiscard]] kb::Result<CanStatus> Status() noexcept;
  /** @brief Back on the bus: from bus-off through the recovery sequence, from stopped by a start; a
   * running controller is left alone. Waits at most `timeout` for the recovery to finish. */
  [[nodiscard]] kb::Status Recover(kb::Milliseconds timeout) noexcept;
  /** @brief Stops and uninstalls the controller (see the file comment). */
  [[nodiscard]] kb::Status Release() noexcept;
  /** @brief The calling task's last call: status and its esp_err_t in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return error_.Get();
  }

private:
  [[nodiscard]] kb::Status sendResult(int32_t err) noexcept;
  [[nodiscard]] int32_t recoverLocked(kb::Milliseconds timeout) const noexcept;

  [[nodiscard]] bool enter() noexcept;
  void leave() noexcept;

  CanConfig config_;
  kb::rtos::Mutex mutex_;
  CallerRecord error_;
  std::atomic<uint32_t> active_{0U};
  std::atomic<bool> ready_{false};
};

/** @brief True when `frame` is a frame CAN 2.0 can carry (dlc, identifier width). */
[[nodiscard]] constexpr bool CanFrameValid(const kb::iface::CanFrame& frame) noexcept {
  const uint32_t limit = frame.extended ? 0x1FFFFFFFU : 0x7FFU;
  return (frame.dlc <= kb::iface::kCanMaxDlc) && (frame.id <= limit);
}

static_assert(kb::iface::CanBus<CanController>, "CanController satisfies kb::iface::CanBus");

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_CAN_HPP
