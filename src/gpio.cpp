// OutputPin and InputPin: pin claims, level conversion and the interrupt hook. Platform-free; every
// SDK call goes through sdk.hpp (src/boundary/gpio.cpp).
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstdint>

#include "claims.hpp"
#include "kb/interfaces/gpio.hpp"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/port/esp32/gpio.hpp"
#include "kb/utils/status.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

namespace {
[[nodiscard]] bool IsHigh(kb::iface::Level level) noexcept {
  return level == kb::iface::Level::High;
}

// Claims `pin`, then runs `configure`; the claim is given back when configuring fails.
template <typename Config, typename Fn>
[[nodiscard]] int32_t ClaimAndConfigure(const Config& config, Fn configure) noexcept {
  int32_t err = esp_err::kNotAllowed; // another port object has the pin
  if (detail::ClaimPins(config.pin)) {
    err = configure(config);
    if (err != esp_err::kOk) {
      detail::UnclaimPins(config.pin);
    }
  }
  return err;
}

[[nodiscard]] bool SafeHigh(const OutputConfig& config) noexcept {
  bool high = false;
  if (config.safe == SafeLevel::High) {
    high = true;
  } else if (config.safe == SafeLevel::Initial) {
    high = IsHigh(config.initial);
  } else {
    high = false; // SafeLevel::Low
  }
  return high;
}

// True once the port installed the GPIO ISR service itself (without ESP_INTR_FLAG_IRAM).
std::atomic<bool>& PortInstalledIsrService() noexcept {
  // codeql::misra_deviation_next_line(DEV-003)
  static std::atomic<bool> installed{false};
  return installed;
}
} // namespace

// --- OutputPin
// ------------------------------------------------------------------------------------

OutputPin::~OutputPin() noexcept {
  static_cast<void>(Release());
}

kb::Status OutputPin::EnsureReady() noexcept {
  kb::Status status = kb::Status::Ok;
  if (!ready_) {
    if (!sdk::GpioValidOutput(config_.pin)) {
      status = error_.Set(kb::Status::InvalidArgument, esp_err::kInvalidArg);
    } else {
      const int32_t err = ClaimAndConfigure(
          config_, [](const OutputConfig& c) noexcept { return sdk::GpioConfigOutput(c); });
      status = error_.Set((err == esp_err::kNotAllowed) ? kb::Status::Busy : MapEspErr(err), err);
      ready_ = status == kb::Status::Ok;
    }
  }
  return status;
}

kb::Status OutputPin::Set(kb::iface::Level level) noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState));
  } else {
    const int32_t err = sdk::GpioSet(config_.pin, IsHigh(level));
    status = error_.Set(MapEspErr(err), err);
  }
  return status;
}

kb::Status OutputPin::Toggle() noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState));
  } else {
    const int32_t err = sdk::GpioSet(config_.pin, !sdk::GpioGet(config_.pin));
    status = error_.Set(MapEspErr(err), err);
  }
  return status;
}

kb::Status OutputPin::Release() noexcept {
  kb::Status status = kb::Status::Ok;
  if (ready_) {
    // Never gpio_reset_pin: it disables the output and enables the pull-up.
    const int32_t err = sdk::GpioSet(config_.pin, SafeHigh(config_));
    detail::UnclaimPins(config_.pin);
    status = error_.Set(MapEspErr(err), err);
    ready_ = false;
  }
  return status;
}

// --- InputPin
// -------------------------------------------------------------------------------------

InputPin::~InputPin() noexcept {
  static_cast<void>(Release());
}

kb::Status InputPin::EnsureReady() noexcept {
  kb::Status status = kb::Status::Ok;
  if (!ready_) {
    if (!sdk::GpioValid(config_.pin)) {
      status = error_.Set(kb::Status::InvalidArgument, esp_err::kInvalidArg);
    } else {
      const int32_t err = ClaimAndConfigure(
          config_, [](const InputConfig& c) noexcept { return sdk::GpioConfigInput(c); });
      status = error_.Set((err == esp_err::kNotAllowed) ? kb::Status::Busy : MapEspErr(err), err);
      ready_ = status == kb::Status::Ok;
    }
  }
  return status;
}

kb::Status InputPin::Get(kb::iface::Level& out) noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState));
  } else {
    out = sdk::GpioGet(config_.pin) ? kb::iface::Level::High : kb::iface::Level::Low;
    status = error_.Clear();
  }
  return status;
}

kb::Status InputPin::EnableInterrupt(kb::iface::Edge edge, kb::iface::GpioCallback fn,
                                     void* context) noexcept {
  kb::Status status = kb::Status::InvalidArgument;
  if (fn == nullptr) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidArg));
  } else if (!ready_) {
    status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState);
  } else {
    int32_t err = sdk::GpioIsrService();
    if (err == esp_err::kOk) {
      PortInstalledIsrService().store(true);
    } else if ((err == esp_err::kInvalidState) && PortInstalledIsrService().load()) {
      err = esp_err::kOk; // installed by the port before
    } else if (err == esp_err::kInvalidState) {
      // Installed by the application with flags the port cannot read.
      err = (config_.foreign_isr_service_ok || sdk::GpioCallbackInIram(fn))
                ? esp_err::kOk
                : esp_err::kNotSupported;
    } else {
      // the installation failed: err is the SDK's code
    }
    if (err == esp_err::kOk) {
      err = sdk::GpioIrqEnable(config_.pin, edge, fn, context);
    }
    status = error_.Set(MapEspErr(err), err);
    irq_ = status == kb::Status::Ok;
  }
  return status;
}

kb::Status InputPin::DisableInterrupt() noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState));
  } else {
    const int32_t err = irq_ ? sdk::GpioIrqDisable(config_.pin) : esp_err::kOk;
    status = error_.Set(MapEspErr(err), err);
    irq_ = irq_ && (status != kb::Status::Ok);
  }
  return status;
}

kb::Status InputPin::Release() noexcept {
  kb::Status status = kb::Status::Ok;
  if (ready_) {
    status = DisableInterrupt(); // the pin stays an input as configured
    detail::UnclaimPins(config_.pin);
    ready_ = false;
  }
  return status;
}

} // namespace kb::port::esp32
