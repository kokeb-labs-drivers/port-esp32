// PwmChannel over the fake ledc: the resolution search, duty rounding, enabling, shared timers and
// every refusal. Test code. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include "support.hpp"

namespace {
using kb::iface::Duty;

esp::PwmConfig Pwm() {
  esp::PwmConfig c{};
  c.pin = 2;
  c.channel = 1U;
  c.timer = 0U;
  c.frequency_hz = 1000U;
  return c;
}
} // namespace

KB_TEST("pwm: duty counts round to nearest at the resolution") {
  static_assert(esp::PwmChannel::DutyCounts(Duty{0U}, 10U) == 0U);
  static_assert(esp::PwmChannel::DutyCounts(Duty{10000U}, 10U) == 1024U);
  static_assert(esp::PwmChannel::DutyCounts(Duty{5000U}, 1U) == 1U);
  static_assert(esp::PwmChannel::DutyCounts(Duty{2500U}, 14U) == 4096U);
  static_assert(esp::PwmChannel::DutyCounts(Duty{1U}, 14U) == 2U); // 1.6384 rounds to 2
  KB_CHECK(esp::PwmChannel::DutyCounts(Duty{12000U}, 8U) == 256U); // Duty clamps to 100 %
}

KB_TEST("pwm: EnsureReady finds the finest resolution and stops at the idle level") {
  Fresh();
  esp::PwmChannel pwm{Pwm()};
  KB_CHECK_EQ(pwm.SetDuty(Duty{5000U}), Status::NotReady);
  KB_CHECK_EQ(pwm.Enable(true), Status::NotReady);
  KB_REQUIRE(pwm.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(pwm.EnsureReady(), Status::Ok);
  // 80 MHz / 1 kHz allows 16 bits, the timer has 14; the fake's 40 MHz clock accepts 14 at 1 kHz.
  KB_CHECK_EQ(pwm.ResolutionBits(), 14U);
  KB_CHECK_EQ(fake::LedcBits(), 14U);
  KB_CHECK(!fake::LedcRunning(1));
  KB_CHECK_EQ(fake::LedcIdle(1), 0U);
  KB_CHECK_EQ(pwm.SetDuty(Duty{2500U}), Status::Ok);
  KB_CHECK_EQ(fake::LedcDuty(1), 0U); // not applied while disabled
  KB_CHECK_EQ(pwm.Enable(true), Status::Ok);
  KB_CHECK(fake::LedcRunning(1));
  KB_CHECK_EQ(fake::LedcDuty(1), 4096U);
  KB_CHECK_EQ(pwm.SetDuty(Duty{10000U}), Status::Ok);
  KB_CHECK_EQ(fake::LedcDuty(1), 16384U);
  KB_CHECK_EQ(pwm.Enable(false), Status::Ok);
  KB_CHECK(!fake::LedcRunning(1));
  KB_CHECK(pwm.LastError().IsOk());
}

KB_TEST("pwm: SetFrequencyHz steps the resolution down until the clock allows it") {
  Fresh();
  esp::PwmChannel pwm{Pwm()};
  KB_REQUIRE(pwm.EnsureReady() == Status::Ok);
  KB_REQUIRE(pwm.Enable(true) == Status::Ok);
  KB_REQUIRE(pwm.SetDuty(Duty{5000U}) == Status::Ok);
  KB_CHECK_EQ(pwm.SetFrequencyHz(1000000U), Status::Ok);
  // 80 MHz allows 6 bits at 1 MHz; the fake's 40 MHz only 5.
  KB_CHECK_EQ(pwm.ResolutionBits(), 5U);
  KB_CHECK_EQ(fake::LedcFrequency(), 1000000U);
  KB_CHECK_EQ(fake::LedcDuty(1), 16U); // the duty is kept as a fraction
  KB_CHECK_EQ(pwm.SetFrequencyHz(0U), Status::InvalidArgument);
  KB_CHECK_EQ(pwm.SetFrequencyHz(40000001U), Status::InvalidArgument); // not even 1 bit
  fake::LedcSourceHz(1U);
  KB_CHECK_EQ(pwm.SetFrequencyHz(1000U),
              Status::InvalidArgument); // the SDK refuses every resolution
  KB_CHECK(FailedWith(pwm.LastError(), Status::InvalidArgument, ESP_FAIL));
  KB_CHECK_EQ(pwm.ResolutionBits(), 5U);
}

KB_TEST("pwm: settings before EnsureReady, bad channels and SDK failures") {
  Fresh();
  esp::PwmChannel early{Pwm()};
  KB_CHECK_EQ(early.SetFrequencyHz(2000U), Status::Ok);
  KB_REQUIRE(early.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::LedcFrequency(), 2000U);
  esp::PwmConfig c = Pwm();
  c.channel = 8U;
  esp::PwmChannel bad_channel{c};
  KB_CHECK_EQ(bad_channel.EnsureReady(), Status::InvalidArgument);
  c = Pwm();
  c.timer = 4U;
  esp::PwmChannel bad_timer{c};
  KB_CHECK_EQ(bad_timer.EnsureReady(), Status::InvalidArgument);
  c = Pwm();
  c.pin = 36;
  esp::PwmChannel bad_pin{c};
  KB_CHECK_EQ(bad_pin.EnsureReady(), Status::InvalidArgument);
  c = Pwm();
  c.channel = 2U;
  c.timer = 1U; // timer 0 runs `early` at another frequency
  c.pin = 3;    // `early` owns pin 2
  c.idle_high = true;
  esp::PwmChannel pwm{c};
  fake::Fail("ledc_channel_config", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(pwm.EnsureReady(), Status::InvalidArgument);
  fake::Fail("ledc_stop", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(pwm.EnsureReady(), Status::NotReady);
  fake::LedcSourceHz(1U);
  KB_CHECK_EQ(pwm.EnsureReady(), Status::InvalidArgument);
  fake::LedcSourceHz(40000000U);
  KB_REQUIRE(pwm.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::LedcIdle(2), 0U); // idle 0 on an inverted output: the pin is high
  KB_CHECK(fake::LedcInverted(2));
  KB_REQUIRE(pwm.Enable(true) == Status::Ok);
  fake::Fail("ledc_set_duty", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(pwm.SetDuty(Duty{100U}), Status::InvalidArgument);
  fake::Fail("ledc_update_duty", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(pwm.SetDuty(Duty{100U}), Status::NotReady);
  fake::Fail("ledc_stop", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(pwm.Enable(false), Status::InvalidArgument);
  fake::Fail("ledc_stop", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(pwm.Release(), Status::InvalidArgument);
  KB_CHECK_EQ(pwm.Release(), Status::Ok);
}

KB_TEST("pwm: channels share a timer only at one frequency; the duty never exceeds the period") {
  // Two channels on one timer: a frequency change by one would change the resolution under the
  // other, whose compare value would then exceed the period (output stuck high).
  Fresh();
  esp::PwmConfig a = Pwm();
  esp::PwmConfig b = Pwm();
  b.channel = 4U;
  b.pin = 4;
  esp::PwmChannel first{a};
  esp::PwmChannel second{b};
  KB_REQUIRE(first.EnsureReady() == Status::Ok);
  KB_REQUIRE(second.EnsureReady() == Status::Ok); // same timer, same frequency: shared
  KB_REQUIRE(first.Enable(true) == Status::Ok);
  KB_REQUIRE(first.SetDuty(Duty{5000U}) == Status::Ok);
  KB_CHECK_EQ(fake::LedcDuty(1), 8192U);
  KB_CHECK_EQ(second.SetFrequencyHz(20000U), Status::Busy); // would retime `first`
  KB_CHECK_EQ(fake::LedcTimerFrequency(0), 1000U);
  KB_CHECK_EQ(first.SetDuty(Duty{5000U}), Status::Ok);
  KB_CHECK(fake::LedcDuty(1) <= (1U << fake::LedcTimerBits(0)));
  esp::PwmConfig c = Pwm();
  c.channel = 5U;
  c.pin = 5;
  c.frequency_hz = 20000U;
  esp::PwmChannel other{c};
  KB_CHECK_EQ(other.EnsureReady(), Status::Busy); // timer 0 runs at 1 kHz
  KB_REQUIRE(second.Release() == Status::Ok);
  KB_CHECK_EQ(first.SetFrequencyHz(20000U), Status::Ok); // now the only user
  KB_CHECK_EQ(first.ResolutionBits(), fake::LedcTimerBits(0));
  KB_CHECK(fake::LedcDuty(1) <= (1U << fake::LedcTimerBits(0)));
  KB_CHECK_EQ(fake::LedcDuty(1), 1U << (fake::LedcTimerBits(0) - 1U)); // still 50 %
  KB_CHECK_EQ(other.EnsureReady(), Status::Ok);                        // joins at 20 kHz
  KB_CHECK_EQ(other.ResolutionBits(), first.ResolutionBits());
}

KB_TEST("pwm: an idle-high channel is inverted from the start; the duty keeps its meaning") {
  Fresh();
  esp::PwmConfig c = Pwm();
  c.idle_high = true;
  esp::PwmChannel pwm{c};
  KB_REQUIRE(pwm.EnsureReady() == Status::Ok);
  KB_CHECK(fake::LedcInverted(1)); // duty 0 inverted: high the moment LEDC takes the pin
  KB_CHECK_EQ(fake::LedcIdle(1), 0U);
  KB_REQUIRE(pwm.Enable(true) == Status::Ok);
  KB_REQUIRE(pwm.SetDuty(Duty{2500U}) == Status::Ok); // 25 % high
  KB_CHECK_EQ(fake::LedcDuty(1), 16384U - 4096U);     // 75 % low before the inversion
  KB_REQUIRE(pwm.SetDuty(Duty{10000U}) == Status::Ok);
  KB_CHECK_EQ(fake::LedcDuty(1), 0U);
  esp::PwmConfig d = Pwm();
  d.channel = 6U;
  d.pin = 6;
  d.timer = 2U;
  esp::PwmChannel plain{d};
  KB_REQUIRE(plain.EnsureReady() == Status::Ok);
  KB_CHECK(!fake::LedcInverted(6));
}
