// PwmChannel: argument checks, pin and LEDC channel claims, the LEDC timer records (frequency,
// resolution, users), the duty resolution search and duty rounding and clamping. Platform-free;
// every SDK call goes through sdk.hpp (the LEDC part, implemented in src/boundary/pwm.cpp).
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
constexpr uint8_t kLedcTimers = 4U;
constexpr uint32_t kGuardTries = 10U;
// The fastest LEDC source clock of the ESP32 family (APB or PLL divided to 80 MHz); slower clocks
// are found by the search, which steps the resolution down until the SDK accepts the timer.
constexpr uint64_t kLedcFastestClockHz = 80000000U;

// One record per LEDC timer, shared by the channels on it. `hz` and `users` change only with
// `busy` held; `bits` is read without it.
struct TimerUse {
  std::atomic<bool> busy{false};
  std::atomic<uint8_t> bits{0U};
  uint32_t hz{0U};
  uint32_t users{0U};
};

std::array<TimerUse, kLedcTimers>& Timers() noexcept {
  // codeql::misra_deviation_next_line(DEV-003)
  static std::array<TimerUse, kLedcTimers> timers{};
  return timers;
}

// Holds a timer record for one scope: a few tries 1 ms apart, then gives up (Busy).
class TimerGuard {
public:
  explicit TimerGuard(TimerUse& timer) noexcept : timer_(timer) {
    for (uint32_t i = 0U; (i < kGuardTries) && !owns_; ++i) {
      owns_ = !timer_.busy.exchange(true);
      if (!owns_) {
        sdk::SleepMs(1U);
      }
    }
  }
  TimerGuard(const TimerGuard&) = delete;
  TimerGuard& operator=(const TimerGuard&) = delete;
  TimerGuard(TimerGuard&&) = delete;
  TimerGuard& operator=(TimerGuard&&) = delete;
  ~TimerGuard() {
    if (owns_) {
      timer_.busy.store(false);
    }
  }
  [[nodiscard]] bool OwnsTimer() const noexcept {
    return owns_;
  }

private:
  TimerUse& timer_;
  bool owns_{false};
};

[[nodiscard]] uint8_t StartBits(uint32_t hz, uint8_t limit) noexcept {
  uint8_t bits = 0U;
  while ((bits < limit) && ((uint64_t{hz} << (bits + 1U)) <= kLedcFastestClockHz)) {
    ++bits;
  }
  return bits;
}

// Configures `timer` at `hz` with the finest resolution the SDK accepts; 0 bits when none.
[[nodiscard]] uint8_t ConfigureTimer(uint8_t timer, uint32_t hz, int32_t& err) noexcept {
  err = esp_err::kInvalidArg;
  uint8_t bits = StartBits(hz, sdk::LedcTimerBits());
  while ((bits > 0U) && (err != esp_err::kOk)) {
    err = sdk::LedcTimer(timer, hz, bits);
    bits = (err == esp_err::kOk) ? bits : static_cast<uint8_t>(bits - 1U);
  }
  return bits;
}
} // namespace

// --- PwmChannel
// -----------------------------------------------------------------------------------

PwmChannel::~PwmChannel() noexcept {
  static_cast<void>(Release());
}

uint8_t PwmChannel::ResolutionBits() const noexcept {
  return ready_ ? Timers()[config_.timer].bits.load() : uint8_t{0U};
}

kb::Status PwmChannel::applyDuty(uint8_t bits) noexcept {
  kb::Status status = kb::Status::Ok;
  if (enabled_) {
    const uint32_t full = uint32_t{1U} << bits;
    const uint32_t high = std::min(DutyCounts(duty_, bits), full);    // never past the period
    const uint32_t counts = config_.idle_high ? (full - high) : high; // the output is inverted
    const int32_t err = sdk::LedcDuty(config_.channel, counts);
    status = error_.Set(MapEspErr(err), err);
  }
  return status;
}

// Takes the timer (joining) or retimes it for this channel; the caller has checked the frequency.
kb::Status PwmChannel::retime(uint32_t hz, bool joining) noexcept {
  TimerUse& timer = Timers()[config_.timer];
  const TimerGuard guard{timer};
  kb::Status status = kb::Status::Ok;
  if (!guard.OwnsTimer()) {
    status = error_.Set(kb::Status::Busy, esp_err::kTimeout); // another channel is changing it
  } else if (joining && (timer.users > 0U) && (timer.hz == hz)) {
    ++timer.users; // the timer already runs at this frequency
    status = error_.Clear();
  } else if ((joining && (timer.users > 0U)) ||
             (!joining && (timer.users > 1U) && (timer.hz != hz))) {
    status =
        error_.Set(kb::Status::Busy, esp_err::kNotAllowed); // another channel needs its frequency
  } else {
    int32_t err = esp_err::kOk;
    const uint8_t bits = ConfigureTimer(config_.timer, hz, err);
    status = error_.Set((err == esp_err::kOk) ? kb::Status::Ok : kb::Status::InvalidArgument, err);
    if (status == kb::Status::Ok) {
      timer.hz = hz;
      timer.bits.store(bits);
      timer.users = joining ? 1U : timer.users;
      status = applyDuty(bits); // every user of the timer is this channel
    }
  }
  return status;
}

kb::Status PwmChannel::SetFrequencyHz(uint32_t hz) noexcept {
  kb::Status status = kb::Status::InvalidArgument;
  if ((hz == 0U) || (StartBits(hz, sdk::LedcTimerBits()) == 0U)) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidArg));
  } else if (!ready_) {
    config_.frequency_hz = hz; // used by EnsureReady()
    status = error_.Clear();
  } else {
    status = retime(hz, false);
    if (status == kb::Status::Ok) {
      config_.frequency_hz = hz;
    }
  }
  return status;
}

kb::Status PwmChannel::EnsureReady() noexcept {
  kb::Status status = kb::Status::Ok;
  if (!ready_) {
    if ((config_.channel >= sdk::LedcChannelCount()) || (config_.timer >= kLedcTimers) ||
        !sdk::GpioValidOutput(config_.pin) || (config_.frequency_hz == 0U) ||
        (StartBits(config_.frequency_hz, sdk::LedcTimerBits()) == 0U)) {
      status = error_.Set(kb::Status::InvalidArgument, esp_err::kInvalidArg);
    } else if (!detail::ClaimPins(config_.pin)) {
      status = error_.Set(kb::Status::Busy, esp_err::kNotAllowed);
    } else {
      enabled_ = false;
      status = retime(config_.frequency_hz, true);
      if (status == kb::Status::Ok) {
        int32_t err = sdk::LedcChannel(config_);
        if (err == esp_err::kOk) {
          err = sdk::LedcStop(config_.channel);
        }
        status = error_.Set(MapEspErr(err), err);
        if (status != kb::Status::Ok) {
          TimerUse& timer = Timers()[config_.timer];
          const TimerGuard guard{timer};
          timer.users =
              (guard.OwnsTimer() && (timer.users > 0U)) ? (timer.users - 1U) : timer.users;
        }
      }
      ready_ = status == kb::Status::Ok;
      if (!ready_) {
        detail::UnclaimPins(config_.pin);
      }
    }
  }
  return status;
}

kb::Status PwmChannel::SetDuty(kb::iface::Duty duty) noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState));
  } else {
    duty_ = duty;
    const TimerGuard guard{Timers()[config_.timer]};
    status = guard.OwnsTimer() ? applyDuty(Timers()[config_.timer].bits.load())
                               : error_.Set(kb::Status::Busy, esp_err::kTimeout);
  }
  return status;
}

kb::Status PwmChannel::Enable(bool on) noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState));
  } else if (on) {
    enabled_ = true;
    const TimerGuard guard{Timers()[config_.timer]};
    // ledc_update_duty also turns the output back on
    status = guard.OwnsTimer() ? applyDuty(Timers()[config_.timer].bits.load())
                               : error_.Set(kb::Status::Busy, esp_err::kTimeout);
  } else {
    enabled_ = false;
    const int32_t err = sdk::LedcStop(config_.channel);
    status = error_.Set(MapEspErr(err), err);
  }
  return status;
}

kb::Status PwmChannel::Release() noexcept {
  kb::Status status = kb::Status::Ok;
  if (ready_) {
    const int32_t err = sdk::LedcStop(config_.channel); // the idle level first
    status = error_.Set(MapEspErr(err), err);
    TimerUse& timer = Timers()[config_.timer];
    const TimerGuard guard{timer};
    if (!guard.OwnsTimer()) {
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout); // stopped, but still on the timer
    } else {
      timer.users = (timer.users > 0U) ? (timer.users - 1U) : 0U;
      ready_ = false;
      enabled_ = false;
      detail::UnclaimPins(config_.pin);
    }
  }
  return status;
}

} // namespace kb::port::esp32
