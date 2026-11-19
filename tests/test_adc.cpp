// AdcChannel over the fake adc_oneshot and adc_cali: shared units, calibration, the unit's lock and
// every refusal. Test code. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include "support.hpp"

namespace {
void ReenterRead(void* context) {
  const kb::Result<uint32_t> r = static_cast<esp::AdcChannel*>(context)->ReadRaw();
  KB_CHECK(!r.IsOk() && (r.Error() == Status::Busy));
}
} // namespace

KB_TEST("adc: a channel reads raw counts and calibrated millivolts") {
  Fresh();
  esp::AdcConfig c{};
  c.pin = 3;
  {
    esp::AdcChannel ch{c};
    KB_CHECK(FailedWith(ch.ReadRaw(), Status::NotReady, ESP_ERR_INVALID_STATE));
    KB_REQUIRE(ch.EnsureReady() == Status::Ok);
    KB_CHECK_EQ(ch.EnsureReady(), Status::Ok);
    KB_CHECK_EQ(ch.FullScale(), 4095U);
    KB_CHECK(ch.Calibrated());
    fake::AdcRaw(4095);
    KB_CHECK(ch.ReadRaw().ValueOr(0U) == 4095U);
    KB_CHECK(ch.ReadMillivolts().ValueOr(0) == 3100);
    KB_CHECK(ch.LastError().IsOk());
    KB_CHECK_EQ(fake::AdcUnitsAlive(), 1);
    KB_CHECK_EQ(fake::AdcCalisAlive(), 1);
  }
  KB_CHECK_EQ(fake::AdcUnitsAlive(), 0);
  KB_CHECK_EQ(fake::AdcCalisAlive(), 0);
}

KB_TEST("adc: channels on one unit share it; the unit closes with the last") {
  Fresh();
  esp::AdcConfig a{};
  a.pin = 1;
  a.attenuation = esp::AdcAttenuation::Db0;
  esp::AdcConfig b{};
  b.pin = 2;
  b.attenuation = esp::AdcAttenuation::Db2p5;
  esp::AdcConfig d{};
  d.pin = 12; // ADC2
  d.attenuation = esp::AdcAttenuation::Db6;
  esp::AdcChannel ch_a{a};
  esp::AdcChannel ch_b{b};
  esp::AdcChannel ch_d{d};
  KB_REQUIRE(ch_a.EnsureReady() == Status::Ok);
  KB_REQUIRE(ch_b.EnsureReady() == Status::Ok);
  KB_REQUIRE(ch_d.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::AdcUnitsAlive(), 2);
  KB_CHECK_EQ(ch_a.Release(), Status::Ok);
  KB_CHECK_EQ(fake::AdcUnitsAlive(), 2);
  KB_CHECK_EQ(ch_b.Release(), Status::Ok);
  KB_CHECK_EQ(fake::AdcUnitsAlive(), 1);
  KB_CHECK_EQ(ch_b.Release(), Status::Ok);
}

KB_TEST("adc: refusals and SDK failures") {
  Fresh();
  esp::AdcConfig c{};
  c.pin = 30; // not an ADC pin in the fake
  esp::AdcChannel not_adc{c};
  KB_CHECK_EQ(not_adc.EnsureReady(), Status::NoDevice);
  c.pin = -1;
  esp::AdcChannel none{c};
  KB_CHECK_EQ(none.EnsureReady(), Status::InvalidArgument);
  c.pin = 4;
  esp::AdcChannel ch{c};
  fake::Fail("adc_oneshot_new_unit", ESP_ERR_NO_MEM);
  KB_CHECK_EQ(ch.EnsureReady(), Status::Fault);
  fake::Fail("adc_oneshot_config_channel", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(ch.EnsureReady(), Status::InvalidArgument);
  KB_CHECK_EQ(fake::AdcUnitsAlive(), 0);
  fake::Fail("adc_cali_create_scheme_curve_fitting", ESP_ERR_NOT_SUPPORTED);
  KB_REQUIRE(ch.EnsureReady() == Status::Ok);
  KB_CHECK(!ch.Calibrated());
  KB_CHECK(FailedWith(ch.ReadMillivolts(), Status::Unsupported, ESP_ERR_NOT_SUPPORTED));
  KB_CHECK(ch.ReadRaw().IsOk());
  fake::Fail("adc_oneshot_read", ESP_ERR_TIMEOUT);
  KB_CHECK(FailedWith(ch.ReadRaw(), Status::Timeout, ESP_ERR_TIMEOUT));
  fake::Fail("adc_oneshot_del_unit", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(ch.Release(), Status::NotReady);
}

KB_TEST("adc: conversion and calibration failures of ReadMillivolts") {
  Fresh();
  esp::AdcConfig c{};
  c.pin = 5;
  esp::AdcChannel ch{c};
  KB_CHECK(FailedWith(ch.ReadMillivolts(), Status::NotReady, ESP_ERR_INVALID_STATE));
  KB_REQUIRE(ch.EnsureReady() == Status::Ok);
  fake::Fail("adc_oneshot_read", ESP_ERR_TIMEOUT);
  KB_CHECK(FailedWith(ch.ReadMillivolts(), Status::Timeout, ESP_ERR_TIMEOUT));
  fake::Fail("adc_cali_raw_to_voltage", ESP_ERR_INVALID_ARG);
  KB_CHECK(FailedWith(ch.ReadMillivolts(), Status::InvalidArgument, ESP_ERR_INVALID_ARG));
  fake::Fail("adc_cali_delete_scheme_curve_fitting", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(ch.Release(), Status::InvalidArgument);
}

KB_TEST("adc: a read while the unit is held is Busy") {
  Fresh();
  esp::AdcConfig c{};
  c.pin = 6;
  esp::AdcChannel ch{c};
  KB_REQUIRE(ch.EnsureReady() == Status::Ok);
  fake::Hook("adc_oneshot_read", ReenterRead, &ch);
  KB_CHECK(ch.ReadRaw().IsOk());
}

KB_TEST("adc: Release closes the unit under its lock; the pin is claimed") {
  Fresh();
  esp::AdcConfig c{};
  c.pin = 3;
  esp::AdcChannel ch{c};
  KB_REQUIRE(ch.EnsureReady() == Status::Ok);
  esp::OutputConfig o{};
  o.pin = 3;
  esp::OutputPin same{o};
  KB_CHECK_EQ(same.EnsureReady(), Status::Busy);
  fake::Hook(
      "adc_oneshot_read",
      [](void* context) {
        KB_CHECK_EQ(static_cast<esp::AdcChannel*>(context)->Release(), Status::Busy);
      },
      &ch);
  KB_CHECK(ch.ReadRaw().IsOk());
  KB_CHECK_EQ(fake::AdcUnitsAlive(), 1);
  KB_CHECK_EQ(ch.Release(), Status::Ok);
  KB_CHECK_EQ(fake::AdcUnitsAlive(), 0);
  KB_CHECK_EQ(same.EnsureReady(), Status::Ok);
}
