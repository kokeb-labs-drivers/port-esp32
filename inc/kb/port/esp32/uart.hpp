/**
 * @file uart.hpp
 * @brief `kb::port::esp32::Uart`: one ESP32 UART (ESP-IDF v5.5 `uart` driver) that satisfies
 * `kb::iface::UartPort`, `UartBaudControl` and `UartTxDrain`, with optional RS-485 half-duplex.
 *
 * Reception runs in the SDK's interrupt into its ring buffer of `UartConfig::rx_buffer_bytes`;
 * transmission writes the hardware FIFO directly (no TX ring buffer), so Write() returns when the
 * last byte is in the FIFO, never later than `UartConfig::write_timeout`. Read() waits at most its
 * timeout for the first byte, then takes whatever else has arrived. Reader and writer have separate
 * locks, so one task may read while another writes; WaitTxDone() takes the writer's lock (bounded
 * by its timeout), so a Write() never waits behind it longer than its own lock wait. Every wait is
 * capped at kMaxTimeout. LastError() reports the calling task's own last call.
 *
 * | Result | Meaning |
 * | --- | --- |
 * | `Ok` | Done (Read: one byte or more) |
 * | `Timeout` | Read: nothing in time. Write: the FIFO did not drain in time |
 * | `Overflow` | Read: FIFO or ring buffer overflow, or receive events lost, since the last Read
 * (Detail: flags) | | `BusError` | Read: framing, parity or break since the last Read (Detail:
 * flags) | | `Busy` | The reader or writer lock was not had in time | | `InvalidArgument` | Empty
 * span, baud 0, or a bad configuration | | `NotReady` | EnsureReady() has not succeeded |
 *
 * An error is reported once, by the next Read(), and the bytes received stay readable. ESP-IDF
 * reports receive errors through an event queue shared with the data events, and drops events when
 * that queue is full. The port checks the queue before it drains it: a full queue means events may
 * have been lost, errors included, and Read() reports `Overflow` with `kEventsLost` (the error
 * state of the received bytes is unknown). Size `UartConfig::event_queue` for the data bursts that
 * arrive between two Read() calls (one event per FIFO-full or receive-timeout interrupt). In RS-485
 * mode the RTS pin drives the transceiver's DE input in hardware (`UART_MODE_RS485_HALF_DUPLEX`).
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_UART_HPP
#define KB_PORT_ESP32_UART_HPP

// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/interfaces/uart.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/rtos/mutex.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"

namespace kb::port::esp32 {

/** @brief Parity of a UART frame. */
enum class UartParity : uint8_t {
  None = 0, ///< No parity bit.
  Even = 1, ///< Even parity.
  Odd = 2,  ///< Odd parity.
};

/** @brief Stop bits of a UART frame. */
enum class UartStopBits : uint8_t {
  One = 1, ///< One stop bit.
  Two = 2, ///< Two stop bits.
};

/** @brief The receive errors latched until the next Read() (UartStats and Read's Detail()). */
namespace uart_flags {
inline constexpr uint32_t kFifoOverflow = 1U << 0U; ///< The hardware FIFO overflowed.
inline constexpr uint32_t kBufferFull = 1U << 1U;   ///< The SDK ring buffer was full.
inline constexpr uint32_t kFrameError = 1U << 2U;   ///< A stop bit was missing.
inline constexpr uint32_t kParityError = 1U << 3U;  ///< A parity bit was wrong.
inline constexpr uint32_t kBreak = 1U << 4U;        ///< A break was received.
inline constexpr uint32_t kEventsLost = 1U << 5U;   ///< The event queue was full: events lost.
} // namespace uart_flags

/** @brief How a UART is wired and framed. */
struct UartConfig {
  uint8_t port{1U};       ///< UART number, 0 to SOC_UART_HP_NUM - 1 (0 is often the console).
  int16_t tx_pin{-1};     ///< GPIO of TXD.
  int16_t rx_pin{-1};     ///< GPIO of RXD.
  int16_t rts_pin{-1};    ///< GPIO of RTS (or RS-485 DE), -1 when unused.
  int16_t cts_pin{-1};    ///< GPIO of CTS, -1 when unused (enables CTS flow control).
  uint32_t baud{115200U}; ///< Bits per second.
  uint8_t data_bits{8U};  ///< 5 to 8.
  UartParity parity{UartParity::None};       ///< Parity.
  UartStopBits stop_bits{UartStopBits::One}; ///< Stop bits.
  bool rs485{false};                         ///< RS-485 half-duplex: RTS drives DE while sending.
  uint16_t rx_buffer_bytes{
      512U}; ///< SDK receive ring buffer, more than the FIFO (SOC_UART_FIFO_LEN).
  kb::Milliseconds write_timeout{
      kb::Milliseconds{100U}}; ///< Write() and lock waits (to kMaxTimeout).
  uint8_t event_queue{32U};    ///< SDK event queue length, 4 or more (see the file comment).
};

/** @brief Counters of a UART since EnsureReady(). */
struct UartStats {
  uint32_t bytes_written{0U}; ///< Bytes put in the FIFO.
  uint32_t bytes_read{0U};    ///< Bytes returned by Read().
  uint32_t errors{0U};        ///< Receive errors reported.
  uint32_t lock_timeouts{0U}; ///< Calls refused with Busy.
};

/** @brief One ESP32 UART; satisfies UartPort, UartBaudControl and UartTxDrain. */
class Uart {
public:
  /** @brief Keeps `config`. */
  explicit Uart(const UartConfig& config) noexcept : config_(config) {}
  Uart(const Uart&) = delete;
  Uart& operator=(const Uart&) = delete;
  Uart(Uart&&) = delete;
  Uart& operator=(Uart&&) = delete;
  /** @brief Deletes the SDK driver if it is installed. */
  ~Uart() noexcept;

  /** @brief Installs and configures the SDK driver once; idempotent. */
  [[nodiscard]] kb::Status EnsureReady() noexcept;
  /** @brief Puts every byte of `tx` in the FIFO, at most `write_timeout`. */
  [[nodiscard]] kb::Status Write(std::span<const uint8_t> tx) noexcept;
  /** @brief Waits at most `timeout` for the first byte, then takes what has arrived. */
  [[nodiscard]] kb::Result<std::size_t> Read(std::span<uint8_t> rx,
                                             kb::Milliseconds timeout) noexcept;
  /** @brief Drops everything received and not yet read, and the latched errors. */
  [[nodiscard]] kb::Status FlushInput() noexcept;
  /** @brief Changes the baud rate (both directions idle first). */
  [[nodiscard]] kb::Status SetBaud(uint32_t baud) noexcept;
  /** @brief Waits at most `timeout` until the last byte has left the shift register. */
  [[nodiscard]] kb::Status WaitTxDone(kb::Milliseconds timeout) noexcept;
  /** @brief Deletes the SDK driver, holding both locks (Busy when either is not had in time). */
  [[nodiscard]] kb::Status Release() noexcept;
  /** @brief The calling task's last call: status and its esp_err_t (or error flags) in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return error_.Get();
  }
  /** @brief The counters. */
  [[nodiscard]] UartStats Stats() const noexcept {
    return UartStats{.bytes_written = bytes_written_.Get(),
                     .bytes_read = bytes_read_.Get(),
                     .errors = errors_.Get(),
                     .lock_timeouts = lock_timeouts_.Get()};
  }
  /** @brief The configuration (with the current baud rate). */
  [[nodiscard]] const UartConfig& Config() const noexcept {
    return config_;
  }

private:
  [[nodiscard]] bool configValid() const noexcept;
  [[nodiscard]] uint32_t charTimeUs() const noexcept;
  [[nodiscard]] kb::Status writeLocked(std::span<const uint8_t> tx) noexcept;
  [[nodiscard]] kb::Result<std::size_t> readLocked(std::span<uint8_t> rx,
                                                   kb::Milliseconds timeout) noexcept;

  UartConfig config_;
  kb::rtos::Mutex tx_mutex_;
  kb::rtos::Mutex rx_mutex_;
  CallerRecord error_;
  Counter bytes_written_;
  Counter bytes_read_;
  Counter errors_;
  Counter lock_timeouts_;
  std::atomic<bool> ready_{false};
};

static_assert(kb::iface::UartPort<Uart>, "Uart satisfies kb::iface::UartPort");
static_assert(kb::iface::UartBaudControl<Uart>, "Uart satisfies UartBaudControl");
static_assert(kb::iface::UartTxDrain<Uart>, "Uart satisfies UartTxDrain");

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_UART_HPP
