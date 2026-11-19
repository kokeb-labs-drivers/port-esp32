// Boundary file (deviations/boundary-files.yml): the LEDC (PWM) calls of ESP-IDF v5.5 behind
// sdk.hpp. Each function converts its arguments, calls the SDK and returns its esp_err_t.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <cstdint>

#include "driver/ledc.h"
#include "esp_err.h"
#include "kb/port/esp32/pwm.hpp"
#include "sdk.hpp"
#include "soc/soc_caps.h"

namespace kb::port::esp32::sdk {

namespace {
constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;

[[nodiscard]] ledc_channel_t LedcCh(uint8_t channel) noexcept {
  return static_cast<ledc_channel_t>(channel);
}

[[nodiscard]] ledc_timer_t LedcTm(uint8_t timer) noexcept {
  return static_cast<ledc_timer_t>(timer);
}
} // namespace

uint8_t LedcChannelCount() noexcept {
  return static_cast<uint8_t>(SOC_LEDC_CHANNEL_NUM);
}
uint8_t LedcTimerBits() noexcept {
  return static_cast<uint8_t>(SOC_LEDC_TIMER_BIT_WIDTH);
}

int32_t LedcTimer(uint8_t timer, uint32_t hz, uint8_t bits) noexcept {
  ledc_timer_config_t config{};
  config.speed_mode = kLedcMode;
  config.duty_resolution = static_cast<ledc_timer_bit_t>(bits);
  config.timer_num = LedcTm(timer);
  config.freq_hz = hz;
  config.clk_cfg = LEDC_AUTO_CLK;
  return ledc_timer_config(&config);
}

int32_t LedcChannel(const PwmConfig& config) noexcept {
  ledc_channel_config_t channel{};
  channel.gpio_num = config.pin;
  channel.speed_mode = kLedcMode;
  channel.channel = LedcCh(config.channel);
  channel.timer_sel = LedcTm(config.timer);
  channel.duty = 0U;
  channel.hpoint = 0;
  // Inverted for an idle-high output: duty 0 then shows high from the moment the pin is routed.
  channel.flags.output_invert = config.idle_high ? 1U : 0U;
  return ledc_channel_config(&channel);
}

int32_t LedcDuty(uint8_t channel, uint32_t counts) noexcept {
  esp_err_t err = ledc_set_duty(kLedcMode, LedcCh(channel), counts);
  if (err == ESP_OK) {
    err = ledc_update_duty(kLedcMode, LedcCh(channel));
  }
  return err;
}

int32_t LedcStop(uint8_t channel) noexcept {
  return ledc_stop(kLedcMode, LedcCh(channel), 0U); // idle 0, shown inverted for idle-high
}

} // namespace kb::port::esp32::sdk
