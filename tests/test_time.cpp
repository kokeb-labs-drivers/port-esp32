// Clock and BusyWait over the fake esp_timer and esp_rom. Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "support.hpp"

KB_TEST("clock: Now is esp_timer_get_time in microseconds and never goes back") {
  Fresh();
  fake::SetTimeUs(5000000000ULL); // past 32 bits
  fake::SetTimeStepUs(3U);
  const esp::Clock clock{};
  const kb::Instant a = clock.Now();
  const kb::Instant b = clock.Now();
  KB_CHECK(a.Us() == 5000000000ULL);
  KB_CHECK(b.Us() == 5000000003ULL);
  KB_CHECK(kb::utils::ElapsedUs(b, a) == 3U);
}

KB_TEST("busy wait: WaitUs spins through esp_rom_delay_us") {
  Fresh();
  esp::BusyWait wait{};
  wait.WaitUs(kb::Microseconds{250U});
  KB_CHECK_EQ(fake::TimeUs(), 250U);
  KB_CHECK_EQ(fake::Calls("esp_rom_delay_us"), 1U);
}
