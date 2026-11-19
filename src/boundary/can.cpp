// Boundary file (deviations/boundary-files.yml): the TWAI (CAN) calls of ESP-IDF v5.5 behind
// sdk.hpp, for chips with SOC_TWAI_SUPPORTED (CMakeLists.txt picks no_can.cpp for the others). Each
// function converts its arguments, calls the SDK and returns its esp_err_t; the handles live in the
// table below.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <array>
#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/can.hpp"
#include "sdk.hpp"
#include "soc/soc_caps.h"

namespace kb::port::esp32::sdk {

namespace {
// A pin number as the SDK's gpio_num_t (-1 is GPIO_NUM_NC).
[[nodiscard]] gpio_num_t Pin(int16_t pin) noexcept {
  return static_cast<gpio_num_t>(pin);
}

constexpr uint64_t kMsPerSecond = 1000U;
// The port caps every wait at kMaxTimeout, so no conversion below reaches portMAX_DELAY.
static_assert((configTICK_RATE_HZ >= 1U) && (configTICK_RATE_HZ <= kMsPerSecond),
              "the tick conversions assume a tick of 1 ms or longer");
static_assert(((uint64_t{kMaxTimeout.Count()} + 1U) * configTICK_RATE_HZ) < portMAX_DELAY);

// Milliseconds to FreeRTOS ticks, rounded up, so a wait is never shorter than asked.
[[nodiscard]] TickType_t TicksCeil(uint32_t ms) noexcept {
  const uint64_t ticks =
      ((uint64_t{ms} * uint64_t{configTICK_RATE_HZ}) + (kMsPerSecond - 1U)) / kMsPerSecond;
  return static_cast<TickType_t>(ticks);
}

std::array<twai_handle_t, SOC_TWAI_CONTROLLER_NUM>& TwaiHandles() noexcept {
  // codeql::misra_deviation_next_line(DEV-001)
  static std::array<twai_handle_t, SOC_TWAI_CONTROLLER_NUM> handles{};
  return handles;
}

[[nodiscard]] bool TimingFor(uint32_t bitrate, twai_timing_config_t& timing) noexcept {
  bool known = true;
  if (bitrate == 25000U) {
    timing = TWAI_TIMING_CONFIG_25KBITS();
  } else if (bitrate == 50000U) {
    timing = TWAI_TIMING_CONFIG_50KBITS();
  } else if (bitrate == 100000U) {
    timing = TWAI_TIMING_CONFIG_100KBITS();
  } else if (bitrate == 125000U) {
    timing = TWAI_TIMING_CONFIG_125KBITS();
  } else if (bitrate == 250000U) {
    timing = TWAI_TIMING_CONFIG_250KBITS();
  } else if (bitrate == 500000U) {
    timing = TWAI_TIMING_CONFIG_500KBITS();
  } else if (bitrate == 800000U) {
    timing = TWAI_TIMING_CONFIG_800KBITS();
  } else if (bitrate == 1000000U) {
    timing = TWAI_TIMING_CONFIG_1MBITS();
  } else {
    known = false;
  }
  return known;
}

[[nodiscard]] twai_mode_t ModeOf(CanMode mode) noexcept {
  twai_mode_t out = TWAI_MODE_NORMAL;
  if (mode == CanMode::NoAck) {
    out = TWAI_MODE_NO_ACK;
  } else if (mode == CanMode::ListenOnly) {
    out = TWAI_MODE_LISTEN_ONLY;
  } else {
    out = TWAI_MODE_NORMAL;
  }
  return out;
}

[[nodiscard]] CanState StateOf(twai_state_t state) noexcept {
  CanState out = CanState::Stopped;
  if (state == TWAI_STATE_RUNNING) {
    out = CanState::Running;
  } else if (state == TWAI_STATE_BUS_OFF) {
    out = CanState::BusOff;
  } else if (state == TWAI_STATE_RECOVERING) {
    out = CanState::Recovering;
  } else {
    out = CanState::Stopped;
  }
  return out;
}
} // namespace

uint8_t TwaiCount() noexcept {
  return static_cast<uint8_t>(SOC_TWAI_CONTROLLER_NUM);
}

int32_t TwaiOpen(const CanConfig& config) noexcept {
  twai_handle_t& handle = TwaiHandles()[config.controller];
  twai_timing_config_t timing{};
  esp_err_t err = ESP_ERR_NOT_ALLOWED; // the controller is owned by another object
  if (!TimingFor(config.bitrate, timing)) {
    err = ESP_ERR_INVALID_ARG;
  } else if (handle == nullptr) {
    twai_general_config_t general{};
    general.controller_id = static_cast<int>(config.controller);
    general.mode = ModeOf(config.mode);
    general.tx_io = Pin(config.tx_pin);
    general.rx_io = Pin(config.rx_pin);
    general.clkout_io = TWAI_IO_UNUSED;
    general.bus_off_io = TWAI_IO_UNUSED;
    general.tx_queue_len = config.tx_queue;
    general.rx_queue_len = config.rx_queue;
    general.alerts_enabled = TWAI_ALERT_NONE;
    general.clkout_divider = 0U;
    general.intr_flags = ESP_INTR_FLAG_LEVEL1;
    const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    err = twai_driver_install_v2(&general, &timing, &filter, &handle);
    if (err == ESP_OK) {
      err = twai_start_v2(handle);
      if (err != ESP_OK) {
        static_cast<void>(twai_driver_uninstall_v2(handle));
      }
    }
    if (err != ESP_OK) {
      handle = nullptr;
    }
  } else {
    err = ESP_ERR_NOT_ALLOWED;
  }
  return err;
}

int32_t TwaiClose(uint8_t controller) noexcept {
  twai_handle_t& handle = TwaiHandles()[controller];
  esp_err_t err = ESP_OK;
  if (handle != nullptr) {
    static_cast<void>(twai_stop_v2(handle)); // ESP_ERR_INVALID_STATE when already stopped
    err = twai_driver_uninstall_v2(handle);
    handle = nullptr;
  }
  return err;
}

int32_t TwaiSend(uint8_t controller, const kb::iface::CanFrame& frame,
                 uint32_t timeout_ms) noexcept {
  twai_message_t message{};
  message.identifier = frame.id;
  message.data_length_code = frame.dlc;
  message.flags =
      (frame.extended ? TWAI_MSG_FLAG_EXTD : 0U) | (frame.remote ? TWAI_MSG_FLAG_RTR : 0U);
  for (std::size_t i = 0U; i < frame.dlc; ++i) {
    message.data[i] = frame.data[i];
  }
  return twai_transmit_v2(TwaiHandles()[controller], &message, TicksCeil(timeout_ms));
}

int32_t TwaiReceive(uint8_t controller, kb::iface::CanFrame& frame, uint32_t timeout_ms) noexcept {
  twai_message_t message{};
  const esp_err_t err = twai_receive_v2(TwaiHandles()[controller], &message, TicksCeil(timeout_ms));
  if (err == ESP_OK) {
    frame = kb::iface::CanFrame{};
    frame.id = message.identifier;
    frame.extended = (message.flags & TWAI_MSG_FLAG_EXTD) != 0U;
    frame.remote = (message.flags & TWAI_MSG_FLAG_RTR) != 0U;
    frame.dlc = (message.data_length_code > kb::iface::kCanMaxDlc) ? kb::iface::kCanMaxDlc
                                                                   : message.data_length_code;
    for (std::size_t i = 0U; i < frame.dlc; ++i) {
      frame.data[i] = message.data[i];
    }
  }
  return err;
}

int32_t TwaiStatus(uint8_t controller, CanStatus& status) noexcept {
  twai_status_info_t info{};
  const esp_err_t err = twai_get_status_info_v2(TwaiHandles()[controller], &info);
  if (err == ESP_OK) {
    status.state = StateOf(info.state);
    status.tx_error_counter = info.tx_error_counter;
    status.rx_error_counter = info.rx_error_counter;
    status.tx_failed = info.tx_failed_count;
    status.rx_missed = info.rx_missed_count;
    status.bus_errors = info.bus_error_count;
  }
  return err;
}

int32_t TwaiRecover(uint8_t controller) noexcept {
  return twai_initiate_recovery_v2(TwaiHandles()[controller]);
}

int32_t TwaiStart(uint8_t controller) noexcept {
  return twai_start_v2(TwaiHandles()[controller]);
}

} // namespace kb::port::esp32::sdk
