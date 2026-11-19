// Uart: argument checks, the reader and writer locks, a FIFO writer bounded by the write timeout,
// a reader that waits for the first byte only, and latched receive errors. Platform-free; every SDK
// call goes through sdk.hpp (src/boundary/uart.cpp).
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include "claims.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/port/esp32/uart.hpp"
#include "kb/rtos/mutex.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

namespace {
constexpr uint8_t kMinDataBits = 5U;
constexpr uint8_t kMaxDataBits = 8U;
constexpr uint32_t kMaxBaud = 5000000U;
constexpr uint32_t kUsPerSecond = 1000000U;
constexpr uint32_t kRefillChars = 8U;  // wait for this many characters to leave before refilling
constexpr uint32_t kBitsOverhead = 2U; // start and stop bit
constexpr uint32_t kOverflowFlags =
    uart_flags::kFifoOverflow | uart_flags::kBufferFull | uart_flags::kEventsLost;
constexpr uint8_t kMinEvents = 4U;

[[nodiscard]] kb::Result<std::size_t> Fail(const CallerRecord& error, kb::Status status,
                                           int32_t err) noexcept {
  static_cast<void>(error.Set(status, err));
  return kb::Result<std::size_t>::Failure(status, VendorCode(err));
}
} // namespace

Uart::~Uart() noexcept {
  static_cast<void>(Release());
}

bool Uart::configValid() const noexcept {
  return (config_.port < sdk::UartPortCount()) && sdk::GpioValidOutput(config_.tx_pin) &&
         sdk::GpioValid(config_.rx_pin) && (config_.baud > 0U) && (config_.baud <= kMaxBaud) &&
         (config_.data_bits >= kMinDataBits) && (config_.data_bits <= kMaxDataBits) &&
         (!config_.rs485 || sdk::GpioValidOutput(config_.rts_pin)) &&
         (config_.write_timeout.Count() > 0U) && (config_.event_queue >= kMinEvents);
}

uint32_t Uart::charTimeUs() const noexcept {
  const uint32_t bits = uint32_t{config_.data_bits} + kBitsOverhead +
                        ((config_.parity == UartParity::None) ? 0U : 1U) +
                        ((config_.stop_bits == UartStopBits::Two) ? 1U : 0U);
  return ((bits * kUsPerSecond) + config_.baud - 1U) / config_.baud;
}

kb::Status Uart::EnsureReady() noexcept {
  kb::Status status = kb::Status::Ok;
  if (!ready_.load()) {
    if (!configValid()) {
      status = error_.Set(kb::Status::InvalidArgument, esp_err::kInvalidArg);
    } else if ((!tx_mutex_.IsCreated() && !tx_mutex_.Create().IsOk()) ||
               (!rx_mutex_.IsCreated() && !rx_mutex_.Create().IsOk())) {
      status = error_.Set(kb::Status::Fault, esp_err::kNoMem); // no RTOS object for a lock
    } else if (!detail::ClaimHardware(detail::Hardware::Uart, config_.port)) {
      status = error_.Set(kb::Status::Busy, esp_err::kNotAllowed); // another Uart object
    } else if (!detail::ClaimPins(config_.tx_pin, config_.rx_pin, config_.rts_pin,
                                  config_.cts_pin)) {
      detail::UnclaimHardware(detail::Hardware::Uart, config_.port);
      status = error_.Set(kb::Status::Busy, esp_err::kNotAllowed); // a pin has another owner
    } else {
      const int32_t err = sdk::UartOpen(config_);
      status = error_.Set(MapEspErr(err), err);
      if (status == kb::Status::Ok) {
        bytes_written_.Reset();
        bytes_read_.Reset();
        errors_.Reset();
        lock_timeouts_.Reset();
        ready_.store(true);
      } else {
        detail::UnclaimPins(config_.tx_pin, config_.rx_pin, config_.rts_pin, config_.cts_pin);
        detail::UnclaimHardware(detail::Hardware::Uart, config_.port);
      }
    }
  }
  return status;
}

kb::Status Uart::Release() noexcept {
  kb::Status status = kb::Status::Ok;
  if (ready_.load()) {
    const kb::Milliseconds wait{BoundedMs(config_.write_timeout)};
    const kb::rtos::LockGuard tx_lock{tx_mutex_, wait};
    const kb::rtos::LockGuard rx_lock{rx_mutex_, wait};
    if (!tx_lock.OwnsLock() || !rx_lock.OwnsLock()) {
      lock_timeouts_.Add();
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else if (ready_.load()) {
      ready_.store(false);
      const int32_t err = sdk::UartClose(config_.port);
      status = error_.Set(MapEspErr(err), err);
      detail::UnclaimPins(config_.tx_pin, config_.rx_pin, config_.rts_pin, config_.cts_pin);
      detail::UnclaimHardware(detail::Hardware::Uart, config_.port);
    } else {
      status = kb::Status::Ok;
    }
  }
  return status;
}

kb::Status Uart::writeLocked(std::span<const uint8_t> tx) noexcept {
  const kb::Instant start{sdk::NowUs()};
  const kb::utils::Deadline deadline =
      kb::utils::Deadline::After(start, kb::Milliseconds{BoundedMs(config_.write_timeout)});
  const uint32_t refill_us = charTimeUs() * kRefillChars;
  kb::Status status = kb::Status::Ok;
  std::size_t done = 0U;
  while ((status == kb::Status::Ok) && (done < tx.size())) {
    std::size_t written = 0U;
    const int32_t err = sdk::UartTxFifo(config_.port, tx.subspan(done), written);
    done += written;
    bytes_written_.Add(static_cast<uint32_t>(written));
    if (err != esp_err::kOk) {
      status = error_.Set(MapEspErr(err), err);
    } else if (done < tx.size()) {
      if (deadline.Expired(kb::Instant{sdk::NowUs()})) {
        status = error_.Set(kb::Status::Timeout, esp_err::kTimeout);
      } else if (refill_us >= (kUsPerSecond / 1000U)) {
        sdk::SleepMs(1U);
      } else {
        sdk::DelayUs(refill_us);
      }
    } else {
      status = error_.Clear();
    }
  }
  return status;
}

kb::Status Uart::Write(std::span<const uint8_t> tx) noexcept {
  kb::Status status = kb::Status::InvalidArgument;
  if (tx.empty()) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidArg));
  } else if (!ready_.load()) {
    status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState);
  } else {
    const kb::rtos::LockGuard lock{tx_mutex_, kb::Milliseconds{BoundedMs(config_.write_timeout)}};
    if (!lock.OwnsLock()) {
      lock_timeouts_.Add();
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else if (!ready_.load()) {
      status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState);
    } else {
      status = writeLocked(tx);
    }
  }
  return status;
}

kb::Result<std::size_t> Uart::readLocked(std::span<uint8_t> rx, kb::Milliseconds timeout) noexcept {
  uint32_t flags = 0U;
  int32_t err = sdk::UartTakeErrors(config_.port, flags);
  kb::Result<std::size_t> result = kb::Result<std::size_t>::Failure(kb::Status::Fault);
  if (err != esp_err::kOk) {
    result = Fail(error_, MapEspErr(err), err);
  } else if (flags != 0U) {
    errors_.Add();
    const kb::Status status =
        ((flags & kOverflowFlags) != 0U) ? kb::Status::Overflow : kb::Status::BusError;
    result = Fail(error_, status, static_cast<int32_t>(flags));
  } else {
    std::size_t got = 0U;
    err = sdk::UartRead(config_.port, rx.first(1U), BoundedMs(timeout), got);
    std::size_t more = 0U;
    if ((err == esp_err::kOk) && (got == 1U) && (rx.size() > 1U)) {
      err = sdk::UartBuffered(config_.port, more);
      more = std::min(more, rx.size() - 1U);
      if ((err == esp_err::kOk) && (more > 0U)) {
        err = sdk::UartRead(config_.port, rx.subspan(1U, more), 0U, more);
      }
    }
    if (err != esp_err::kOk) {
      result = Fail(error_, MapEspErr(err), err);
    } else if (got == 0U) {
      result = Fail(error_, kb::Status::Timeout, esp_err::kTimeout);
    } else {
      bytes_read_.Add(static_cast<uint32_t>(got + more));
      static_cast<void>(error_.Clear());
      result = kb::Result<std::size_t>::Success(got + more);
    }
  }
  return result;
}

kb::Result<std::size_t> Uart::Read(std::span<uint8_t> rx, kb::Milliseconds timeout) noexcept {
  kb::Result<std::size_t> result = kb::Result<std::size_t>::Failure(kb::Status::Fault);
  if (rx.empty()) {
    result = Fail(error_, kb::Status::InvalidArgument, esp_err::kInvalidArg);
  } else if (!ready_.load()) {
    result = Fail(error_, kb::Status::NotReady, esp_err::kInvalidState);
  } else {
    const kb::rtos::LockGuard lock{rx_mutex_, kb::Milliseconds{BoundedMs(timeout)}};
    if (!lock.OwnsLock()) {
      lock_timeouts_.Add();
      result = Fail(error_, kb::Status::Busy, esp_err::kTimeout);
    } else if (!ready_.load()) {
      result = Fail(error_, kb::Status::NotReady, esp_err::kInvalidState);
    } else {
      result = readLocked(rx, timeout);
    }
  }
  return result;
}

kb::Status Uart::FlushInput() noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_.load()) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState));
  } else {
    const kb::rtos::LockGuard lock{rx_mutex_, kb::Milliseconds{BoundedMs(config_.write_timeout)}};
    if (!lock.OwnsLock()) {
      lock_timeouts_.Add();
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else if (!ready_.load()) {
      status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState);
    } else {
      uint32_t flags = 0U;
      int32_t err = sdk::UartFlushInput(config_.port);
      if (err == esp_err::kOk) {
        err = sdk::UartTakeErrors(config_.port, flags); // dropped with the data
      }
      status = error_.Set(MapEspErr(err), err);
    }
  }
  return status;
}

kb::Status Uart::SetBaud(uint32_t baud) noexcept {
  kb::Status status = kb::Status::InvalidArgument;
  if ((baud == 0U) || (baud > kMaxBaud)) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidArg));
  } else if (!ready_.load()) {
    config_.baud = baud; // used by EnsureReady()
    status = error_.Clear();
  } else {
    const kb::Milliseconds wait{BoundedMs(config_.write_timeout)};
    const kb::rtos::LockGuard tx_lock{tx_mutex_, wait};
    const kb::rtos::LockGuard rx_lock{rx_mutex_, wait};
    if (!tx_lock.OwnsLock() || !rx_lock.OwnsLock()) {
      lock_timeouts_.Add();
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else if (!ready_.load()) {
      status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState);
    } else {
      const int32_t err = sdk::UartSetBaud(config_.port, baud);
      status = error_.Set(MapEspErr(err), err);
      if (status == kb::Status::Ok) {
        config_.baud = baud;
      }
    }
  }
  return status;
}

kb::Status Uart::WaitTxDone(kb::Milliseconds timeout) noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_.load()) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState));
  } else {
    // Under the writer's lock: uart_wait_tx_done holds the SDK's TX mutex for its whole wait, and
    // uart_tx_chars waits for that mutex without a limit, so a Write() must not run meanwhile.
    const uint32_t ms = BoundedMs(timeout);
    const kb::rtos::LockGuard lock{tx_mutex_, kb::Milliseconds{ms}};
    if (!lock.OwnsLock()) {
      lock_timeouts_.Add();
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else if (!ready_.load()) {
      status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState);
    } else {
      const int32_t err = sdk::UartWaitTxDone(config_.port, ms);
      status = error_.Set(MapEspErr(err), err);
    }
  }
  return status;
}

} // namespace kb::port::esp32
