// The boundary files built for a chip without TWAI and with the line-fitting ADC calibration (the
// ESP32 and ESP32-S2 scheme): CAN is Unsupported everywhere, the ADC calibrates by line fitting.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "sdk.hpp"
#include "support.hpp"

#ifdef SOC_TWAI_SUPPORTED
#error "the minimal variant is built without TWAI"
#endif

KB_TEST("minimal: a chip without TWAI reports Unsupported and touches nothing") {
  Fresh();
  esp::CanConfig c{};
  c.tx_pin = 1;
  c.rx_pin = 2;
  esp::CanController can{c};
  KB_CHECK_EQ(can.EnsureReady(), Status::Unsupported);
  KB_CHECK(FailedWith(can.LastError(), Status::Unsupported, ESP_ERR_NOT_SUPPORTED));
  KB_CHECK_EQ(can.Send(kb::iface::CanFrame{}, kb::Milliseconds{1U}), Status::NotReady);
  KB_CHECK_EQ(fake::Calls("twai_driver_install_v2"), 0U);
  // The SDK seam itself, as a port object would see it after a forced start.
  kb::iface::CanFrame f{};
  esp::CanStatus s{};
  namespace sdk = kb::port::esp32::sdk;
  KB_CHECK_EQ(sdk::TwaiCount(), 0U);
  KB_CHECK_EQ(sdk::TwaiOpen(c), ESP_ERR_NOT_SUPPORTED);
  KB_CHECK_EQ(sdk::TwaiClose(0U), ESP_ERR_NOT_SUPPORTED);
  KB_CHECK_EQ(sdk::TwaiSend(0U, f, 1U), ESP_ERR_NOT_SUPPORTED);
  KB_CHECK_EQ(sdk::TwaiReceive(0U, f, 1U), ESP_ERR_NOT_SUPPORTED);
  KB_CHECK_EQ(sdk::TwaiStatus(0U, s), ESP_ERR_NOT_SUPPORTED);
  KB_CHECK_EQ(sdk::TwaiRecover(0U), ESP_ERR_NOT_SUPPORTED);
  KB_CHECK_EQ(sdk::TwaiStart(0U), ESP_ERR_NOT_SUPPORTED);
}

KB_TEST("minimal: the ADC calibrates by line fitting, with the default Vref when given") {
  Fresh();
  esp::AdcConfig c{};
  c.pin = 3;
  c.default_vref_mv = 1100U;
  {
    esp::AdcChannel ch{c};
    KB_REQUIRE(ch.EnsureReady() == Status::Ok);
    KB_CHECK(ch.Calibrated());
    KB_CHECK_EQ(fake::Calls("adc_cali_create_scheme_line_fitting"), 1U);
    KB_CHECK_EQ(fake::Calls("adc_cali_create_scheme_curve_fitting"), 0U);
    fake::AdcRaw(0);
    KB_CHECK(ch.ReadMillivolts().ValueOr(-1) == 0);
  }
  KB_CHECK_EQ(fake::Calls("adc_cali_delete_scheme_line_fitting"), 1U);
  KB_CHECK_EQ(fake::AdcCalisAlive(), 0);
  KB_CHECK_EQ(fake::AdcDefaultVref(), 1100U);
}

KB_TEST("minimal: ESP-IDF v6 reports a failed I2C transfer as ESP_ERR_INVALID_RESPONSE") {
  Fresh();
  esp::I2cConfig c{};
  c.sda_pin = 8;
  c.scl_pin = 9;
  esp::I2cController bus{c};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  KB_REQUIRE(bus.AllowNackProbe(0x20U, true) == Status::Ok);
  const std::array<uint8_t, 1> tx{0x00U};
  KB_CHECK_EQ(bus.Write(0x20U, tx), Status::NoDevice);
  KB_CHECK(FailedWith(bus.LastError(), Status::NoDevice, ESP_ERR_INVALID_RESPONSE));
  fake::I2cPresent(0x20U, true);
  fake::I2cStretchUs(30000U);
  KB_CHECK_EQ(bus.Write(0x20U, tx), Status::Timeout);
}

KB_TEST("minimal: at a 100 Hz tick a short timeout reaches the SDK as whole ticks plus one") {
  Fresh();
  esp::I2cConfig c{};
  c.sda_pin = 8;
  c.scl_pin = 9;
  c.timeout = kb::Milliseconds{5U}; // 0 ticks after ESP-IDF's pdMS_TO_TICKS without rounding
  esp::I2cController bus{c};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  fake::I2cPresent(0x20U, true);
  const std::array<uint8_t, 1> tx{0x00U};
  KB_CHECK_EQ(bus.Write(0x20U, tx), Status::Ok);
  KB_CHECK_EQ(fake::I2cLastTimeoutMs(), 20); // 1 tick rounded up, plus 1: 2 ticks of 10 ms
}

namespace {
void Count(void* context) noexcept {
  ++*static_cast<int*>(context);
}
} // namespace

// In this executable no test installed the GPIO ISR service before: the application does it here.
KB_TEST("gpio: an ISR service installed by the application takes only IRAM callbacks") {
  Fresh();
  int hits = 0;
  KB_REQUIRE(gpio_install_isr_service(0) == ESP_OK); // the application, flags unknown to the port
  esp::InputConfig c{};
  c.pin = 33;
  esp::InputPin pin{c};
  KB_REQUIRE(pin.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(pin.EnableInterrupt(kb::iface::Edge::Rising, Count, &hits), Status::Unsupported);
  KB_CHECK(!fake::IrqEnabled(33));
  fake::IramAll(true); // the callback is in IRAM
  KB_CHECK_EQ(pin.EnableInterrupt(kb::iface::Edge::Rising, Count, &hits), Status::Ok);
  fake::IramAll(false);
  KB_REQUIRE(pin.Release() == Status::Ok);
  c.foreign_isr_service_ok = true; // the application says: installed without ESP_INTR_FLAG_IRAM
  esp::InputPin stated{c};
  KB_REQUIRE(stated.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(stated.EnableInterrupt(kb::iface::Edge::Falling, Count, &hits), Status::Ok);
}
