// Boundary file (deviations/boundary-files.yml): the clock, busy-wait and sleep calls of ESP-IDF
// v5.5 behind sdk.hpp. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include <cstdint>

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdk.hpp"

namespace kb::port::esp32::sdk {

namespace {
// Milliseconds to FreeRTOS ticks, rounded up, so a wait is never shorter than asked.
[[nodiscard]] TickType_t TicksCeil(uint32_t ms) noexcept {
  constexpr uint64_t kMsPerSecond = 1000U;
  const uint64_t ticks =
      ((uint64_t{ms} * uint64_t{configTICK_RATE_HZ}) + (kMsPerSecond - 1U)) / kMsPerSecond;
  return static_cast<TickType_t>(ticks);
}
} // namespace

uint64_t NowUs() noexcept {
  return static_cast<uint64_t>(esp_timer_get_time());
}

void DelayUs(uint32_t us) noexcept {
  esp_rom_delay_us(us);
}

void SleepMs(uint32_t ms) noexcept {
  vTaskDelay(TicksCeil(ms));
}

} // namespace kb::port::esp32::sdk
