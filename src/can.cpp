// CanController: frame checks, bus-off handling and recovery. Platform-free; every SDK call goes
// through sdk.hpp (the TWAI part, implemented in src/boundary/can.cpp or, on chips without a TWAI
// controller, src/boundary/no_can.cpp).
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <cstddef>
#include <cstdint>

#include "claims.hpp"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/can.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/rtos/mutex.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

namespace {
constexpr uint32_t kRecoverPollMs = 1U;
constexpr uint32_t kCanLockWaitMs = 100U;
} // namespace

// --- CanController
// --------------------------------------------------------------------------------

CanController::~CanController() noexcept {
  static_cast<void>(Release());
}

kb::Status CanController::EnsureReady() noexcept {
  kb::Status status = kb::Status::Ok;
  if (!ready_.load()) {
    int32_t err = esp_err::kOk;
    if (sdk::TwaiCount() == 0U) {
      err = esp_err::kNotSupported;
    } else if ((config_.controller >= sdk::TwaiCount()) || !sdk::GpioValidOutput(config_.tx_pin) ||
               !sdk::GpioValid(config_.rx_pin) || (config_.tx_queue == 0U) ||
               (config_.rx_queue == 0U)) {
      err = esp_err::kInvalidArg;
    } else if (!mutex_.IsCreated() && !mutex_.Create().IsOk()) {
      err = esp_err::kNoMem; // no RTOS object for the lock
    } else if (!detail::ClaimHardware(detail::Hardware::Twai, config_.controller)) {
      err = esp_err::kNotAllowed; // another CanController object
    } else if (!detail::ClaimPins(config_.tx_pin, config_.rx_pin)) {
      detail::UnclaimHardware(detail::Hardware::Twai, config_.controller);
      err = esp_err::kNotAllowed; // a pin has another owner
    } else {
      err = sdk::TwaiOpen(config_);
      if (err != esp_err::kOk) {
        detail::UnclaimPins(config_.tx_pin, config_.rx_pin);
        detail::UnclaimHardware(detail::Hardware::Twai, config_.controller);
      }
    }
    status = error_.Set(MapEspErr(err), err);
    ready_.store(status == kb::Status::Ok);
  }
  return status;
}

bool CanController::enter() noexcept {
  static_cast<void>(active_.fetch_add(1U));
  const bool up = ready_.load();
  if (!up) {
    leave();
  }
  return up;
}

void CanController::leave() noexcept {
  static_cast<void>(active_.fetch_sub(1U));
}

kb::Status CanController::sendResult(int32_t err) noexcept {
  kb::Status status = MapEspErr(err);
  if ((status == kb::Status::NotReady) || (err == esp_err::kFail)) {
    status = kb::Status::BusError; // not running (bus-off or recovering), or the frame was refused
  }
  return error_.Set(status, err);
}

kb::Status CanController::Send(const kb::iface::CanFrame& frame,
                               kb::Milliseconds timeout) noexcept {
  kb::Status status = kb::Status::InvalidArgument;
  if (!CanFrameValid(frame)) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidArg));
  } else if (!enter()) {
    status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState);
  } else {
    status = sendResult(sdk::TwaiSend(config_.controller, frame, BoundedMs(timeout)));
    leave();
  }
  return status;
}

kb::Status CanController::Receive(kb::iface::CanFrame& frame, kb::Milliseconds timeout) noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!enter()) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState));
  } else {
    const int32_t err = sdk::TwaiReceive(config_.controller, frame, BoundedMs(timeout));
    leave();
    status = MapEspErr(err);
    status = error_.Set((status == kb::Status::NotReady) ? kb::Status::BusError : status, err);
  }
  return status;
}

kb::Result<CanStatus> CanController::Status() noexcept {
  kb::Result<CanStatus> result =
      kb::Result<CanStatus>::Failure(kb::Status::NotReady, VendorCode(esp_err::kInvalidState));
  if (!enter()) {
    static_cast<void>(error_.Set(kb::Status::NotReady, esp_err::kInvalidState));
  } else {
    CanStatus status{};
    const int32_t err = sdk::TwaiStatus(config_.controller, status);
    leave();
    const kb::Status s = error_.Set(MapEspErr(err), err);
    result = (s == kb::Status::Ok) ? kb::Result<CanStatus>::Success(status)
                                   : kb::Result<CanStatus>::Failure(s, VendorCode(err));
  }
  return result;
}

int32_t CanController::recoverLocked(kb::Milliseconds timeout) const noexcept {
  const kb::utils::Deadline deadline =
      kb::utils::Deadline::After(kb::Instant{sdk::NowUs()}, kb::Milliseconds{BoundedMs(timeout)});
  CanStatus now{};
  int32_t err = sdk::TwaiStatus(config_.controller, now);
  if ((err == esp_err::kOk) && (now.state == CanState::BusOff)) {
    err = sdk::TwaiRecover(config_.controller);
    now.state = CanState::Recovering;
  }
  while ((err == esp_err::kOk) && (now.state == CanState::Recovering)) {
    if (deadline.Expired(kb::Instant{sdk::NowUs()})) {
      err = esp_err::kTimeout;
    } else {
      sdk::SleepMs(kRecoverPollMs);
      err = sdk::TwaiStatus(config_.controller, now);
    }
  }
  if ((err == esp_err::kOk) && (now.state == CanState::Stopped)) {
    err = sdk::TwaiStart(config_.controller);
  }
  return err;
}

kb::Status CanController::Recover(kb::Milliseconds timeout) noexcept {
  kb::Status status = kb::Status::NotReady;
  if (!ready_.load()) {
    static_cast<void>(error_.Set(status, esp_err::kInvalidState));
  } else {
    const kb::rtos::LockGuard lock{mutex_, kb::Milliseconds{kCanLockWaitMs}};
    if (!lock.OwnsLock()) {
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else if (!ready_.load()) {
      status = error_.Set(kb::Status::NotReady, esp_err::kInvalidState);
    } else {
      const int32_t err = recoverLocked(timeout);
      status = error_.Set(MapEspErr(err), err);
    }
  }
  return status;
}

kb::Status CanController::Release() noexcept {
  kb::Status status = kb::Status::Ok;
  if (ready_.load()) {
    const kb::rtos::LockGuard lock{mutex_, kb::Milliseconds{kCanLockWaitMs}};
    if (!lock.OwnsLock()) {
      status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
    } else if (ready_.load()) {
      ready_.store(false); // new calls are refused from here
      // Running Send/Receive calls end within their timeout (at most kMaxTimeout).
      const kb::utils::Deadline deadline = kb::utils::Deadline::After(
          kb::Instant{sdk::NowUs()}, kb::Milliseconds{kMaxTimeout.Count() + kCanLockWaitMs});
      while ((active_.load() != 0U) && !deadline.Expired(kb::Instant{sdk::NowUs()})) {
        sdk::SleepMs(kRecoverPollMs);
      }
      if (active_.load() != 0U) {
        ready_.store(true);
        status = error_.Set(kb::Status::Busy, esp_err::kTimeout);
      } else {
        const int32_t err = sdk::TwaiClose(config_.controller);
        detail::UnclaimPins(config_.tx_pin, config_.rx_pin);
        detail::UnclaimHardware(detail::Hardware::Twai, config_.controller);
        status = error_.Set(MapEspErr(err), err);
      }
    } else {
      status = kb::Status::Ok;
    }
  }
  return status;
}

} // namespace kb::port::esp32
