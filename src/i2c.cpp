// I2cController: argument checks, the controller lock, failure classification (by time, and by an
// opted-in address probe) and bus recovery. Platform-free; every SDK call goes through sdk.hpp
// (src/boundary/i2c.cpp).
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <algorithm>
// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

#include "claims.hpp"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/port/esp32/i2c.hpp"
#include "kb/rtos/mutex.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

namespace {
constexpr uint32_t kMaxClockHz = 1000000U;
constexpr uint64_t kUsPerSecond = 1000000U;
constexpr uint64_t kUsPerMs = 1000U;
constexpr uint64_t kBitsPerByte = 9U; // eight data bits and the acknowledge
constexpr uint64_t kFrameBits = 4U;   // START, STOP and the bus free time, rounded up
constexpr uint32_t kMaskBits = 32U;
using kb::iface::I2cFailure;

[[nodiscard]] bool AddressValid(uint8_t address) noexcept {
  return (address >= kI2cFirstAddress) && (address <= kI2cLastAddress);
}

[[nodiscard]] uint8_t Aux(I2cFailure failure) noexcept {
  return static_cast<uint8_t>(failure);
}
} // namespace

I2cController::~I2cController() noexcept {
  static_cast<void>(Release());
}

bool I2cController::configValid() const noexcept {
  return (config_.port < sdk::I2cPortCount()) && sdk::GpioValidOutput(config_.sda_pin) &&
         sdk::GpioValidOutput(config_.scl_pin) && (config_.sda_pin != config_.scl_pin) &&
         (config_.clock_hz > 0U) && (config_.clock_hz <= kMaxClockHz) &&
         (config_.timeout.Count() > 0U) &&
         (config_.stretch_limit_us <=
          (kMaxTimeout.Count() * 1000U)); // within every chip's register
}

kb::Status I2cController::EnsureReady() noexcept {
  kb::Status status = kb::Status::Ok;
  if (!ready_.load()) {
    if (!configValid()) {
      status = error_.Set(kb::Status::InvalidArgument, esp_err::kInvalidArg);
    } else if (!mutex_.IsCreated() && !mutex_.Create().IsOk()) {
      status = error_.Set(kb::Status::Fault, esp_err::kNoMem); // no RTOS object for the lock
    } else if (!detail::ClaimHardware(detail::Hardware::I2c, config_.port)) {
      status = error_.Set(kb::Status::Busy, esp_err::kNotAllowed); // another controller object
    } else if (!detail::ClaimPins(config_.sda_pin, config_.scl_pin)) {
      detail::UnclaimHardware(detail::Hardware::I2c, config_.port);
      status = error_.Set(kb::Status::Busy, esp_err::kNotAllowed); // a pin has another owner
    } else {
      const int32_t err = sdk::I2cOpen(config_);
      status = error_.Set(MapEspErr(err), err);
      stretch_us_ = sdk::I2cStretchLimitUs(config_.stretch_limit_us);
      if (status != kb::Status::Ok) {
        detail::UnclaimPins(config_.sda_pin, config_.scl_pin);
        detail::UnclaimHardware(detail::Hardware::I2c, config_.port);
      } else {
        // A bus left mid-transfer by a warm reset: clock it free. ESP-IDF reports an error when no
        // line was held low, which is not a failure here.
        static_cast<void>(sdk::I2cReset(config_.port));
        device_address_ = 0U;
        transfers_.Reset();
        failures_.Reset();
        recoveries_.Reset();
        lock_timeouts_.Reset();
        ready_.store(true);
      }
    }
  }
  return status;
}

kb::Status I2cController::Release() noexcept {
  kb::Status status = kb::Status::Ok;
  if (ready_.load()) {
    const kb::rtos::LockGuard lock{mutex_, kb::Milliseconds{BoundedMs(config_.timeout)}};
    if (!lock.OwnsLock()) {
      lock_timeouts_.Add();
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else if (ready_.load()) {
      ready_.store(false); // callers that already passed the check find it false under the lock
      const int32_t err = sdk::I2cClose(config_.port);
      detail::UnclaimPins(config_.sda_pin, config_.scl_pin);
      detail::UnclaimHardware(detail::Hardware::I2c, config_.port);
      status = error_.Set(MapEspErr(err), err);
    } else {
      status = kb::Status::Ok;
    }
  }
  return status;
}

kb::Status I2cController::AllowNackProbe(uint8_t address, bool allow) noexcept {
  kb::Status status = kb::Status::InvalidArgument;
  if (!AddressValid(address)) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidArg));
  } else {
    const uint32_t bit = uint32_t{1U} << (uint32_t{address} % kMaskBits);
    std::atomic<uint32_t>& word = probe_mask_[uint32_t{address} / kMaskBits];
    if (allow) {
      static_cast<void>(word.fetch_or(bit));
    } else {
      static_cast<void>(word.fetch_and(~bit));
    }
    status = error_.Clear();
  }
  return status;
}

bool I2cController::probeAllowed(uint8_t address) const noexcept {
  const uint32_t bit = uint32_t{1U} << (uint32_t{address} % kMaskBits);
  return (probe_mask_[uint32_t{address} / kMaskBits].load() & bit) != 0U;
}

kb::Status I2cController::Write(uint8_t address, std::span<const uint8_t> tx) noexcept {
  return Write(address, tx, config_.timeout);
}

kb::Status I2cController::Write(uint8_t address, std::span<const uint8_t> tx,
                                kb::Milliseconds timeout) noexcept {
  return transfer(Call{.op = Op::Write,
                       .address = address,
                       .tx = tx,
                       .rx = std::span<uint8_t>{},
                       .ms = BoundedMs(timeout)});
}

kb::Status I2cController::WriteRead(uint8_t address, std::span<const uint8_t> tx,
                                    std::span<uint8_t> rx) noexcept {
  return WriteRead(address, tx, rx, config_.timeout);
}

kb::Status I2cController::WriteRead(uint8_t address, std::span<const uint8_t> tx,
                                    std::span<uint8_t> rx, kb::Milliseconds timeout) noexcept {
  kb::Status status = kb::Status::InvalidArgument;
  if (tx.empty() || rx.empty() || (timeout.Count() == 0U)) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidArg, Aux(I2cFailure::Bus)));
  } else {
    status = transfer(Call{
        .op = Op::WriteRead, .address = address, .tx = tx, .rx = rx, .ms = BoundedMs(timeout)});
  }
  return status;
}

kb::Status I2cController::Read(uint8_t address, std::span<uint8_t> rx) noexcept {
  return Read(address, rx, config_.timeout);
}

kb::Status I2cController::Read(uint8_t address, std::span<uint8_t> rx,
                               kb::Milliseconds timeout) noexcept {
  kb::Status status = kb::Status::InvalidArgument;
  if (rx.empty() || (timeout.Count() == 0U)) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidArg, Aux(I2cFailure::Bus)));
  } else {
    status = transfer(Call{.op = Op::Read,
                           .address = address,
                           .tx = std::span<const uint8_t>{},
                           .rx = rx,
                           .ms = BoundedMs(timeout)});
  }
  return status;
}

int32_t I2cController::run(const Call& call) noexcept {
  int32_t err = esp_err::kOk;
  if ((call.op == Op::Write) && call.tx.empty()) {
    err = sdk::I2cProbe(config_.port, call.address, call.ms);
  } else {
    if (call.address != device_address_) {
      err = sdk::I2cSetAddress(config_.port, call.address, call.ms);
      device_address_ = (err == esp_err::kOk) ? call.address : 0U;
    }
    if (err == esp_err::kOk) {
      if (call.op == Op::Write) {
        err = sdk::I2cTransmit(config_.port, call.tx, call.ms);
      } else if (call.op == Op::WriteRead) {
        err = sdk::I2cTransmitReceive(config_.port, call.tx, call.rx, call.ms);
      } else {
        err = sdk::I2cReceive(config_.port, call.rx, call.ms);
      }
    }
  }
  return err;
}

kb::Status I2cController::transfer(const Call& call) noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_.load()) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState, Aux(I2cFailure::Bus)));
  } else if (!AddressValid(call.address) || (call.ms == 0U)) {
    status = error_.Set(kb::Status::InvalidArgument, esp_err::kInvalidArg, Aux(I2cFailure::Bus));
  } else {
    const kb::rtos::LockGuard lock{mutex_, kb::Milliseconds{call.ms}};
    if (!lock.OwnsLock()) {
      lock_timeouts_.Add();
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout, Aux(I2cFailure::Timeout));
    } else if (!ready_.load()) {
      status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState, Aux(I2cFailure::Bus));
    } else {
      transfers_.Add();
      const uint64_t start = sdk::NowUs();
      const int32_t err = run(call);
      const uint64_t elapsed = sdk::NowUs() - start;
      status = classify(call, err, elapsed);
      if (status != kb::Status::Ok) {
        failures_.Add();
      }
    }
  }
  return status;
}

bool I2cController::linesIdle() const noexcept {
  return sdk::GpioGet(config_.sda_pin) && sdk::GpioGet(config_.scl_pin);
}

uint64_t I2cController::wireUs(const Call& call) const noexcept {
  // Every byte of the transfer and the address byte (twice with a repeated START), at the clock.
  const uint64_t addresses = (call.op == Op::WriteRead) ? 2U : 1U;
  const uint64_t bits =
      ((uint64_t{call.tx.size()} + uint64_t{call.rx.size()} + addresses) * kBitsPerByte) +
      (kFrameBits * addresses);
  return ((bits * kUsPerSecond) + config_.clock_hz - 1U) / config_.clock_hz;
}

kb::iface::I2cFailure I2cController::nackOf(const Call& call, bool ambiguous) const noexcept {
  // Fast (a NACK): unsplit DataNack without the probe. Ambiguous (a NACK or an early stretch
  // timeout): Bus without the probe, never DataNack.
  I2cFailure failure = ambiguous ? I2cFailure::Bus : I2cFailure::DataNack;
  if (probeAllowed(call.address)) {
    const int32_t probe = sdk::I2cProbe(config_.port, call.address, call.ms);
    if (probe == esp_err::kNotFound) {
      failure = I2cFailure::AddressNack;
    } else if (probe == esp_err::kOk) {
      failure = ambiguous ? I2cFailure::Bus : I2cFailure::DataNack; // the device answers now
    } else if (probe == esp_err::kTimeout) {
      failure = linesIdle() ? I2cFailure::Timeout : I2cFailure::BusStuck;
    } else {
      failure = linesIdle() ? I2cFailure::Bus : I2cFailure::BusStuck;
    }
  }
  if ((failure == I2cFailure::Bus) && !linesIdle()) {
    failure = I2cFailure::BusStuck;
  }
  return failure;
}

kb::iface::I2cFailure I2cController::failureOf(const Call& call, int32_t err,
                                               uint64_t elapsed_us) const noexcept {
  I2cFailure failure = I2cFailure::Bus;
  const bool probe = (call.op == Op::Write) && call.tx.empty();
  if (err == esp_err::kOk) {
    failure = I2cFailure::None;
  } else if (probe && (err == esp_err::kNotFound)) {
    failure = I2cFailure::AddressNack; // i2c_master_probe: the address was not acknowledged
  } else if (probe && (err == esp_err::kTimeout)) {
    failure = linesIdle() ? I2cFailure::Timeout : I2cFailure::BusStuck; // stretched or locked
  } else if (err == sdk::I2cTransferFailedCode()) {
    // A NACK, a stretch or software timeout, or lost arbitration (one code). A timeout cannot end
    // before the shortest timeout that applies (the SCL limit or the transfer's own timeout); a
    // NACK cannot end after the wire time plus the slack. Three bands follow.
    const uint64_t earliest_timeout_us =
        std::min<uint64_t>(uint64_t{stretch_us_}, uint64_t{call.ms} * kUsPerMs);
    if (elapsed_us < earliest_timeout_us) {
      failure = nackOf(call, false);
    } else if (elapsed_us > (wireUs(call) + kI2cNackSlackUs)) {
      failure = linesIdle() ? I2cFailure::Timeout : I2cFailure::BusStuck;
    } else {
      failure = nackOf(call, true);
    }
  } else {
    failure = linesIdle() ? I2cFailure::Bus : I2cFailure::BusStuck;
  }
  return failure;
}

kb::Status I2cController::statusOf(kb::iface::I2cFailure failure) noexcept {
  kb::Status status = kb::Status::BusError;
  switch (failure) {
  case I2cFailure::None:
    status = kb::Status::Ok;
    break;
  case I2cFailure::AddressNack:
    status = kb::Status::NoDevice;
    break;
  case I2cFailure::DataNack:
    status = kb::Status::Nack;
    break;
  case I2cFailure::Timeout:
    status = kb::Status::Timeout;
    break;
  case I2cFailure::Bus:
  case I2cFailure::BusStuck:
  default:
    status = kb::Status::BusError;
    break;
  }
  return status;
}

kb::Status I2cController::classify(const Call& call, int32_t err, uint64_t elapsed_us) noexcept {
  kb::Status status = kb::Status::Ok;
  I2cFailure failure = I2cFailure::None;
  const bool probe = (call.op == Op::Write) && call.tx.empty();
  if (err == esp_err::kInvalidArg) {
    status = kb::Status::InvalidArgument; // the SDK refused the arguments; nothing reached the bus
    failure = I2cFailure::Bus;
  } else if (!probe && (err == esp_err::kTimeout)) {
    status = kb::Status::Busy; // ESP-IDF's own bus lock was not had in time; nothing was sent
    failure = I2cFailure::Timeout;
  } else {
    failure = failureOf(call, err, elapsed_us);
    status = statusOf(failure);
    if ((failure == I2cFailure::BusStuck) && config_.auto_recover) {
      int32_t reset = esp_err::kOk;
      static_cast<void>(recoverLocked(reset)); // the transfer failed even if the bus is free now
    }
  }
  return error_.Set(status, err, Aux(failure));
}

kb::iface::I2cFailure I2cController::recoverLocked(int32_t& err) noexcept {
  recoveries_.Add();
  err = sdk::I2cReset(config_.port);
  device_address_ = 0U;
  return ((err == esp_err::kOk) && linesIdle()) ? I2cFailure::None : I2cFailure::BusStuck;
}

kb::Status I2cController::RecoverBus() noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_.load()) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState, Aux(I2cFailure::Bus)));
  } else {
    const kb::rtos::LockGuard lock{mutex_, kb::Milliseconds{BoundedMs(config_.timeout)}};
    if (!lock.OwnsLock()) {
      lock_timeouts_.Add();
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout, Aux(I2cFailure::Timeout));
    } else {
      int32_t err = esp_err::kOk;
      const I2cFailure failure = recoverLocked(err);
      kb::Status s = MapEspErr(err);
      if ((s == kb::Status::Ok) && (failure != I2cFailure::None)) {
        s = kb::Status::BusError; // a device still holds a line low
      }
      status = error_.Set(s, err, Aux(failure));
    }
  }
  return status;
}

} // namespace kb::port::esp32
