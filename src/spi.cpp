// SpiBus and SpiDevice: argument checks, pin and slot claims, the bus lock, one polling transaction
// per frame with a wait sized to the frame's wire time, and the failed-host state when a
// transaction never ends. Platform-free; every SDK call goes through sdk.hpp
// (src/boundary/spi.cpp).
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include "claims.hpp"
#include "kb/interfaces/spi.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/port/esp32/spi.hpp"
#include "kb/rtos/mutex.hpp"
#include "kb/rtos/time.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

namespace {
constexpr uint32_t kMaxClockHz = 80000000U;
constexpr uint32_t kMinClockHz = 1000U; // ESP-IDF reports the clock it set in kHz
constexpr uint8_t kMaxMode = 3U;
constexpr uint8_t kMaxCsCycles = 16U;
constexpr uint8_t kFirstHost = 1U; // SPI1 is the flash bus
constexpr uint64_t kUsPerSecond = 1000000U;
constexpr uint64_t kUsPerMs = 1000U;
constexpr uint64_t kBitsPerByte = 8U;

// The end wait of one transaction: its wire time at `hz` plus the timeout, in whole milliseconds.
[[nodiscard]] uint32_t EndWaitMs(std::size_t bytes, uint32_t hz,
                                 const SpiDeviceConfig& config) noexcept {
  const uint64_t cycles = (uint64_t{bytes} * kBitsPerByte) + uint64_t{config.cs_setup_cycles} +
                          uint64_t{config.cs_hold_cycles};
  const uint64_t wire_us = ((cycles * kUsPerSecond) + hz - 1U) / hz;
  const uint64_t wire_ms = (wire_us + kUsPerMs - 1U) / kUsPerMs;
  return static_cast<uint32_t>(wire_ms) + BoundedMs(config.timeout);
}

// The worst case of one Transfer() in milliseconds, tick rounding included: the lock wait (ticks
// rounded up, plus one) and two end waits (ticks rounded up, plus one, plus one more because
// polling_end compares the elapsed ticks with ">").
[[nodiscard]] uint64_t WorstMs(uint32_t lock_ms, uint32_t end_ms) noexcept {
  constexpr uint64_t kMsPerSecond = 1000U;
  const uint64_t hz = std::max<uint64_t>(kb::rtos::TickRateHz(), 1U);
  const auto ticks = [hz](uint64_t ms) noexcept -> uint64_t {
    return (((ms * hz) + kMsPerSecond - 1U) / kMsPerSecond) + 1U;
  };
  const uint64_t all = ticks(lock_ms) + (2U * (ticks(end_ms) + 1U));
  return ((all * kMsPerSecond) + hz - 1U) / hz;
}

// True when a one-byte frame at `hz` fits kMaxTimeout: otherwise every frame would be refused.
[[nodiscard]] bool TimeoutFits(const SpiDeviceConfig& config, uint32_t hz) noexcept {
  return WorstMs(BoundedMs(config.timeout), EndWaitMs(1U, hz, config)) <=
         uint64_t{kMaxTimeout.Count()};
}

detail::BitClaims<1U>& SpiSlots() noexcept {
  // codeql::misra_deviation_next_line(DEV-003)
  static detail::BitClaims<1U> slots{};
  return slots;
}
} // namespace

// --- SpiBus
// ---------------------------------------------------------------------------------------

SpiBus::~SpiBus() noexcept {
  static_cast<void>(Release());
}

bool SpiBus::configValid() const noexcept {
  return (config_.host >= kFirstHost) && (config_.host <= sdk::SpiHostLast()) &&
         sdk::GpioValidOutput(config_.sclk_pin) &&
         ((config_.mosi_pin >= 0) || (config_.miso_pin >= 0)) &&
         (!config_.dma || ((config_.dma_buffer_bytes >= kSpiMinDmaBufferBytes) &&
                           (config_.dma_buffer_bytes <= kSpiMaxDmaFrameBytes)));
}

int32_t SpiBus::open() noexcept {
  int32_t err = sdk::SpiBusOpen(config_);
  if ((err == esp_err::kOk) && config_.dma) {
    err = sdk::SpiDmaBufferOpen(config_.host, config_.dma_buffer_bytes, dma_tx_, dma_rx_);
    if (err != esp_err::kOk) {
      static_cast<void>(sdk::SpiBusClose(config_.host));
    }
  }
  return err;
}

kb::Status SpiBus::EnsureReady() noexcept {
  kb::Status status = kb::Status::Ok;
  if (!ready_.load()) {
    if (!configValid()) {
      status = error_.Set(kb::Status::InvalidArgument, esp_err::kInvalidArg);
    } else if (!mutex_.IsCreated() && !mutex_.Create().IsOk()) {
      status = error_.Set(kb::Status::Fault, esp_err::kNoMem);
    } else if (!detail::ClaimHardware(detail::Hardware::SpiHost, config_.host)) {
      status = error_.Set(kb::Status::Busy, esp_err::kNotAllowed); // another bus has the host
    } else if (!detail::ClaimPins(config_.sclk_pin, config_.mosi_pin, config_.miso_pin)) {
      detail::UnclaimHardware(detail::Hardware::SpiHost, config_.host);
      status = error_.Set(kb::Status::Busy, esp_err::kNotAllowed); // a pin has another owner
    } else {
      const int32_t err = open();
      status = error_.Set(MapEspErr(err), err);
      if (status == kb::Status::Ok) {
        ready_.store(true);
      } else {
        detail::UnclaimPins(config_.sclk_pin, config_.mosi_pin, config_.miso_pin);
        detail::UnclaimHardware(detail::Hardware::SpiHost, config_.host);
      }
    }
  }
  return status;
}

kb::Status SpiBus::Release() noexcept {
  kb::Status status = kb::Status::Ok;
  if (ready_.load()) {
    const kb::rtos::LockGuard lock{mutex_, kb::Milliseconds{kMaxTimeout}};
    if (!lock.OwnsLock()) {
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else {
      // ESP-IDF refuses while a device is attached (ESP_ERR_INVALID_STATE), so the DMA buffer is
      // freed only when no transaction can still use it.
      const int32_t err = sdk::SpiBusClose(config_.host);
      status = error_.Set(MapEspErr(err), err);
      if (status == kb::Status::Ok) {
        ready_.store(false);
        if (config_.dma) {
          sdk::SpiDmaBufferClose(config_.host);
          dma_tx_ = std::span<uint8_t>{};
          dma_rx_ = std::span<uint8_t>{};
        }
        detail::UnclaimPins(config_.sclk_pin, config_.mosi_pin, config_.miso_pin);
        detail::UnclaimHardware(detail::Hardware::SpiHost, config_.host);
      }
    }
  }
  return status;
}

// --- SpiDevice
// ------------------------------------------------------------------------------------

SpiDevice::~SpiDevice() noexcept {
  static_cast<void>(Release());
}

bool SpiDevice::configValid() const noexcept {
  return (config_.mode <= kMaxMode) && (config_.clock_hz >= kMinClockHz) &&
         (config_.clock_hz <= kMaxClockHz) && (config_.timeout.Count() > 0U) &&
         (config_.cs_setup_cycles <= kMaxCsCycles) && (config_.cs_hold_cycles <= kMaxCsCycles) &&
         ((config_.cs_pin < 0) || sdk::GpioValidOutput(config_.cs_pin)) &&
         TimeoutFits(config_, config_.clock_hz);
}

kb::Status SpiDevice::addLocked(uint32_t hz) noexcept {
  uint32_t actual = 0U;
  int32_t err = sdk::SpiDeviceAdd(bus_.Config().host, config_, hz, slot_, actual);
  if ((err == esp_err::kOk) && (actual == 0U)) {
    static_cast<void>(sdk::SpiDeviceRemove(slot_)); // below 1 kHz: no wire time to bound a wait by
    err = esp_err::kInvalidArg;
  }
  const kb::Status status = error_.Set(MapEspErr(err), err);
  if (status == kb::Status::Ok) {
    actual_hz_ = (actual <= hz) ? actual : hz;
    added_.store(true);
  }
  return status;
}

kb::Status SpiDevice::removeLocked() noexcept {
  kb::Status status = kb::Status::Ok;
  if (added_.load()) {
    const int32_t err = sdk::SpiDeviceRemove(slot_);
    status = error_.Set(MapEspErr(err), err);
    if (status == kb::Status::Ok) {
      added_.store(false);
    }
  }
  return status;
}

kb::Status SpiDevice::EnsureReady() noexcept {
  kb::Status status = kb::Status::Ok;
  if (!added_.load()) {
    if (!configValid()) {
      status = error_.Set(kb::Status::InvalidArgument, esp_err::kInvalidArg);
    } else {
      status = bus_.EnsureReady();
      if (status != kb::Status::Ok) {
        static_cast<void>(error_.Set(status, esp_err::kInvalidState));
      } else if (!slot_claimed_) {
        const uint32_t slot = SpiSlots().ClaimFirst(sdk::SpiSlotCount());
        if (slot >= sdk::SpiSlotCount()) {
          status = error_.Set(kb::Status::Busy, esp_err::kNoMem); // every device slot is in use
        } else if (!detail::ClaimPins(config_.cs_pin)) {
          SpiSlots().Unclaim(slot);
          status = error_.Set(kb::Status::Busy, esp_err::kNotAllowed);
        } else {
          slot_ = static_cast<uint8_t>(slot);
          slot_claimed_ = true;
        }
      } else {
        status = kb::Status::Ok;
      }
      if (status != kb::Status::Ok) {
        // refused above: no slot or the chip-select pin is taken
      } else if (!bus_.TryLockFor(kb::Milliseconds{BoundedMs(config_.timeout)})) {
        lock_timeouts_.Add();
        status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
      } else {
        status = bus_.IsFailed() ? error_.Set(kb::Status::Fault, esp_err::kInvalidState)
                                 : addLocked(config_.clock_hz);
        bus_.Unlock();
      }
    }
  }
  return status;
}

kb::Status SpiDevice::Release() noexcept {
  kb::Status status = kb::Status::Ok;
  if (added_.load()) {
    if (!bus_.TryLockFor(kb::Milliseconds{BoundedMs(config_.timeout)})) {
      lock_timeouts_.Add();
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else {
      status = removeLocked(); // ESP-IDF refuses a device whose transaction never ended
      bus_.Unlock();
    }
  }
  if (!added_.load() && slot_claimed_) {
    SpiSlots().Unclaim(slot_);
    detail::UnclaimPins(config_.cs_pin);
    slot_claimed_ = false;
  }
  return status;
}

uint32_t SpiDevice::waitMs(std::size_t bytes) const noexcept {
  return EndWaitMs(bytes, actual_hz_, config_);
}

kb::Status SpiDevice::checkFrame(std::span<const uint8_t> tx,
                                 std::span<uint8_t> rx) const noexcept {
  const std::size_t total = std::max(tx.size(), rx.size());
  const std::size_t limit = bus_.MaxFrameBytes();
  const std::size_t single =
      (tx.empty() && !bus_.Config().dma) ? std::min(limit, sdk::SpiZeroBytes()) : limit;
  bool ok = kb::iface::SpiSpansValid(tx, rx) && (total > 0U) && (total <= single);
  if (ok && (actual_hz_ > 0U)) {
    // T + 2 x (W + T) at most kMaxTimeout: the documented bound of one call holds.
    ok = WorstMs(BoundedMs(config_.timeout), waitMs(total)) <= uint64_t{kMaxTimeout.Count()};
  }
  return ok ? kb::Status::Ok : kb::Status::InvalidArgument;
}

int32_t SpiDevice::endLocked(uint32_t wait, bool& failed) noexcept {
  int32_t err = sdk::SpiEnd(slot_, wait);
  if (err == esp_err::kTimeout) {
    // Still in flight: the master clocks it to the end, so wait once more before giving up.
    late_.Add();
    const int32_t again = sdk::SpiEnd(slot_, wait);
    if (again == esp_err::kOk) {
      err = esp_err::kTimeout; // complete, but past the deadline
    } else if (again == esp_err::kTimeout) {
      bus_.MarkFailed(); // never ended: the host is not used again
      err = esp_err::kInvalidState;
      failed = true;
    } else {
      err = again;
    }
  }
  return err;
}

kb::Status SpiDevice::frameLocked(std::span<const uint8_t> tx, std::span<uint8_t> rx) noexcept {
  transfers_.Add();
  std::fill(rx.begin(), rx.end(), uint8_t{0U});
  const std::size_t n = std::max(tx.size(), rx.size());
  const uint32_t wait = waitMs(n);
  const bool dma = bus_.Config().dma;
  std::span<const uint8_t> t = tx;
  std::span<uint8_t> r = rx;
  if (dma) {
    // Through the port's DMA buffer: the hardware never reads or writes the caller's memory.
    const std::span<uint8_t> out = bus_.dma_tx_.first(n);
    if (tx.empty()) {
      std::fill(out.begin(), out.end(), uint8_t{0U});
    } else {
      static_cast<void>(std::copy(tx.begin(), tx.end(), out.begin()));
    }
    t = out;
    r = rx.empty() ? std::span<uint8_t>{} : bus_.dma_rx_.first(n);
  }
  int32_t err = sdk::SpiStart(slot_, t, r);
  bool failed = false;
  if (err == esp_err::kOk) {
    err = endLocked(wait, failed);
  }
  const bool ended = (err == esp_err::kOk) || ((err == esp_err::kTimeout) && !failed);
  if (dma && ended && !rx.empty()) {
    static_cast<void>(std::copy(r.begin(), r.end(), rx.begin()));
  }
  kb::Status status = failed ? kb::Status::Fault : MapEspErr(err);
  if (err == esp_err::kNoMem) {
    status = kb::Status::Busy; // ESP-IDF found no heap for a bounce buffer: transient, try again
  }
  if (status != kb::Status::Ok) {
    failures_.Add();
  }
  return error_.Set(status, err);
}

kb::Status SpiDevice::Transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) noexcept {
  kb::Status status = checkFrame(tx, rx);
  if (status != kb::Status::Ok) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidArg));
  } else if (!added_.load()) {
    status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState);
  } else if (bus_.IsFailed()) {
    status = error_.Set(kb::Status::Fault, esp_err::kInvalidState);
  } else if (!bus_.TryLockFor(kb::Milliseconds{BoundedMs(config_.timeout)})) {
    lock_timeouts_.Add();
    status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
  } else {
    if (bus_.IsFailed()) {
      status = error_.Set(kb::Status::Fault, esp_err::kInvalidState);
    } else if (!added_.load()) {
      status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState);
    } else {
      status = frameLocked(tx, rx);
    }
    bus_.Unlock();
  }
  return status;
}

kb::Status SpiDevice::SetClockHz(uint32_t hz) noexcept {
  kb::Status status = kb::Status::InvalidArgument;
  if ((hz < kMinClockHz) || (hz > kMaxClockHz) || !TimeoutFits(config_, hz)) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidArg));
  } else if (!added_.load()) {
    config_.clock_hz = hz; // used by EnsureReady()
    status = error_.Clear();
  } else if (!bus_.TryLockFor(kb::Milliseconds{BoundedMs(config_.timeout)})) {
    lock_timeouts_.Add();
    status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
  } else {
    if (bus_.IsFailed()) {
      status = error_.Set(kb::Status::Fault, esp_err::kInvalidState);
    } else {
      status = removeLocked();
      if (status == kb::Status::Ok) {
        status = addLocked(hz);
        if (status == kb::Status::Ok) {
          config_.clock_hz = hz;
        } else if (addLocked(config_.clock_hz) != kb::Status::Ok) {
          status = error_.Set(kb::Status::Fault, esp_err::kInvalidState); // lost the device
        } else {
          status = error_.Set(kb::Status::InvalidArgument, esp_err::kInvalidArg);
        }
      }
    }
    bus_.Unlock();
  }
  return status;
}

} // namespace kb::port::esp32
