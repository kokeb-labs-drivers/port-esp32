// AdcChannel: argument checks, pin and channel-slot claims, the shared ADC unit (users and lock),
// calibration state and the raw and millivolt reads. Platform-free; every SDK call goes through
// sdk.hpp (the ADC part, implemented in src/boundary/adc.cpp).
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <algorithm>
#include <array>
// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstdint>

#include "claims.hpp"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/adc.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/port/esp32/pwm.hpp"
#include "kb/rtos/mutex.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

namespace {
constexpr uint8_t kMaxAdcUnits = 2U;
constexpr uint8_t kLockNone = 0U;
constexpr uint8_t kLockCreating = 1U;
constexpr uint8_t kLockReady = 2U;

// ESP-IDF refuses a read while another task reads the same unit (adc_lock_try_acquire); the port
// serialises the channels of a unit with the unit's lock. `users` changes only under the lock.
struct AdcUnitState {
  kb::rtos::Mutex lock;
  std::atomic<uint8_t> lock_state{kLockNone};
  uint32_t users{0U};
};

std::array<AdcUnitState, kMaxAdcUnits>& AdcUnits() noexcept {
  // codeql::misra_deviation_next_line(DEV-003)
  static std::array<AdcUnitState, kMaxAdcUnits> units{};
  return units;
}

detail::BitClaims<1U>& AdcSlots() noexcept {
  // codeql::misra_deviation_next_line(DEV-003)
  static detail::BitClaims<1U> slots{};
  return slots;
}

// The unit's lock, created by the first channel that needs it; false while another task creates it
// or when the RTOS has no room for it.
[[nodiscard]] bool UnitLockReady(AdcUnitState& unit) noexcept {
  uint8_t expected = kLockNone;
  if (unit.lock_state.compare_exchange_strong(expected, kLockCreating)) {
    unit.lock_state.store(unit.lock.Create().IsOk() ? kLockReady : kLockNone);
  }
  return unit.lock_state.load() == kLockReady;
}
} // namespace

// --- AdcChannel
// -----------------------------------------------------------------------------------

AdcChannel::~AdcChannel() noexcept {
  static_cast<void>(Release());
}

kb::Status AdcChannel::openLocked() noexcept {
  AdcUnitState& unit = AdcUnits()[unit_];
  int32_t err = (unit.users == 0U) ? sdk::AdcUnitOpen(unit_) : esp_err::kOk;
  if (err == esp_err::kOk) {
    ++unit.users;
    err = sdk::AdcChannelOpen(slot_, unit_, channel_, config_.attenuation);
    if (err != esp_err::kOk) {
      --unit.users;
      if (unit.users == 0U) {
        static_cast<void>(sdk::AdcUnitClose(unit_));
      }
    }
  }
  const kb::Status status = error_.Set(MapEspErr(err), err);
  if (status == kb::Status::Ok) {
    full_scale_ = (uint32_t{1U} << sdk::AdcBits()) - 1U;
    calibrated_.store(sdk::AdcCalibrate(slot_, config_.default_vref_mv) == esp_err::kOk);
  }
  return status;
}

kb::Status AdcChannel::EnsureReady() noexcept {
  kb::Status status = kb::Status::Ok;
  if (!ready_.load()) {
    int32_t err = sdk::AdcPinChannel(config_.pin, unit_, channel_);
    if ((err == esp_err::kOk) && (unit_ >= std::min(sdk::AdcUnitCount(), kMaxAdcUnits))) {
      err = esp_err::kNotSupported;
    }
    if (err != esp_err::kOk) {
      status = error_.Set(MapEspErr(err), err);
    } else if (!UnitLockReady(AdcUnits()[unit_])) {
      status = error_.Set(kb::Status::Busy, esp_err::kNoMem);
    } else if (!detail::ClaimPins(config_.pin)) {
      status = error_.Set(kb::Status::Busy, esp_err::kNotAllowed);
    } else {
      const uint32_t slot = AdcSlots().ClaimFirst(sdk::AdcSlotCount());
      if (slot >= sdk::AdcSlotCount()) {
        status = error_.Set(kb::Status::Busy, esp_err::kNoMem); // every channel slot is in use
      } else {
        slot_ = static_cast<uint8_t>(slot);
        const kb::rtos::LockGuard lock{AdcUnits()[unit_].lock,
                                       kb::Milliseconds{BoundedMs(config_.lock_timeout)}};
        status = lock.OwnsLock() ? openLocked() : error_.Set(kb::Status::Busy, esp_err::kTimeout);
        if (status != kb::Status::Ok) {
          AdcSlots().Unclaim(slot);
        }
      }
      if (status == kb::Status::Ok) {
        ready_.store(true);
      } else {
        detail::UnclaimPins(config_.pin);
      }
    }
  }
  return status;
}

kb::Result<uint32_t> AdcChannel::ReadRaw() noexcept {
  kb::Result<uint32_t> result =
      kb::Result<uint32_t>::Failure(kb::Status::NotReady, VendorCode(esp_err::kInvalidState));
  if (!ready_.load()) {
    static_cast<void>(error_.Set(kb::Status::NotReady, esp_err::kInvalidState));
  } else {
    int32_t raw = 0;
    int32_t err = esp_err::kNotFinished; // the unit's lock was not had in time
    {
      const kb::rtos::LockGuard lock{AdcUnits()[unit_].lock,
                                     kb::Milliseconds{BoundedMs(config_.lock_timeout)}};
      if (lock.OwnsLock()) {
        err = ready_.load() ? sdk::AdcRead(slot_, raw) : esp_err::kInvalidState;
      }
    }
    const kb::Status status = error_.Set(MapEspErr(err), err);
    result = (status == kb::Status::Ok) ? kb::Result<uint32_t>::Success(static_cast<uint32_t>(raw))
                                        : kb::Result<uint32_t>::Failure(status, VendorCode(err));
  }
  return result;
}

kb::Result<int32_t> AdcChannel::ReadMillivolts() noexcept {
  kb::Result<int32_t> result =
      kb::Result<int32_t>::Failure(kb::Status::Unsupported, VendorCode(esp_err::kNotSupported));
  if (ready_.load() && !calibrated_.load()) {
    static_cast<void>(error_.Set(kb::Status::Unsupported, esp_err::kNotSupported));
  } else {
    const kb::Result<uint32_t> raw = ReadRaw();
    if (!raw.IsOk()) {
      result = kb::Result<int32_t>::Failure(raw.Error(), raw.Detail());
    } else {
      int32_t mv = 0;
      const int32_t err = sdk::AdcToMillivolts(slot_, static_cast<int32_t>(raw.Value()), mv);
      const kb::Status status = error_.Set(MapEspErr(err), err);
      result = (status == kb::Status::Ok) ? kb::Result<int32_t>::Success(mv)
                                          : kb::Result<int32_t>::Failure(status, VendorCode(err));
    }
  }
  return result;
}

kb::Status AdcChannel::Release() noexcept {
  kb::Status status = kb::Status::Ok;
  if (ready_.load()) {
    AdcUnitState& unit = AdcUnits()[unit_];
    const kb::rtos::LockGuard lock{unit.lock, kb::Milliseconds{BoundedMs(config_.lock_timeout)}};
    if (!lock.OwnsLock()) {
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else {
      ready_.store(false);
      calibrated_.store(false);
      const int32_t cali = sdk::AdcChannelClose(slot_);
      int32_t err = cali;
      if (unit.users > 0U) {
        --unit.users;
      }
      if (unit.users == 0U) {
        const int32_t closed = sdk::AdcUnitClose(unit_);
        err = (cali == esp_err::kOk) ? closed : cali;
      }
      AdcSlots().Unclaim(slot_);
      detail::UnclaimPins(config_.pin);
      status = error_.Set(MapEspErr(err), err);
    }
  }
  return status;
}

} // namespace kb::port::esp32
