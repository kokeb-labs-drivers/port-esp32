// Every port type satisfies its kb::iface concepts (compile time) and works through the contract's
// type-erased *Ref (run time, over the fakes), the way a HAL chooses an implementation at run time.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "support.hpp"

namespace i = kb::iface;

static_assert(i::I2cBus<esp::I2cController>);
static_assert(i::SpiDevice<esp::SpiDevice>);
static_assert(i::SpiClockControl<esp::SpiDevice>);
static_assert(i::UartPort<esp::Uart>);
static_assert(i::UartBaudControl<esp::Uart>);
static_assert(i::UartTxDrain<esp::Uart>);
static_assert(i::GpioOutput<esp::OutputPin>);
static_assert(i::GpioInput<esp::InputPin>);
static_assert(i::GpioInterrupt<esp::InputPin>);
static_assert(i::AdcChannel<esp::AdcChannel>);
static_assert(i::PwmOutput<esp::PwmChannel>);
static_assert(i::CanBus<esp::CanController>);
static_assert(i::MonotonicClock<esp::Clock>);
static_assert(i::BusyWait<esp::BusyWait>);
static_assert(i::LogSink<esp::LogSink>);
// What each type is not: the concepts keep the roles apart.
static_assert(!i::GpioInput<esp::OutputPin>);
static_assert(!i::GpioOutput<esp::InputPin>);
static_assert(!i::SpiDevice<esp::SpiBus>);
static_assert(!i::UartPort<esp::I2cController>);
// Ports own SDK resources and locks: never copied or moved.
static_assert(!std::is_copy_constructible_v<esp::I2cController>);
static_assert(!std::is_move_constructible_v<esp::Uart>);
static_assert(!std::is_copy_assignable_v<esp::SpiDevice>);

KB_TEST("contracts: I2cBusRef, SpiDeviceRef and UartPortRef reach the port") {
  Fresh();
  fake::I2cPresent(0x40U, true);
  esp::I2cConfig ic{};
  ic.sda_pin = 1;
  ic.scl_pin = 2;
  esp::I2cController i2c{ic};
  i::I2cBusRef bus{i2c};
  const std::array<uint8_t, 1> one{0x01U};
  std::array<uint8_t, 1> rx{};
  KB_CHECK(bus.EnsureReady() == Status::Ok);
  KB_CHECK(bus.Write(0x40U, one) == Status::Ok);
  KB_CHECK(bus.WriteRead(0x40U, one, rx) == Status::Ok);
  KB_CHECK(bus.Read(0x41U, rx) == Status::Nack); // unsplit: no probe unless allowed
  KB_CHECK(bus.LastFailure() == i::I2cFailure::DataNack);

  esp::SpiBusConfig sc{};
  sc.mosi_pin = 3;
  sc.sclk_pin = 4;
  esp::SpiBus spi_bus{sc};
  esp::SpiDevice spi{spi_bus, esp::SpiDeviceConfig{}};
  i::SpiDeviceRef dev{spi};
  KB_CHECK(dev.EnsureReady() == Status::Ok);
  KB_CHECK(dev.Transfer(one, rx) == Status::Ok);

  esp::UartConfig uc{};
  uc.tx_pin = 5;
  uc.rx_pin = 6;
  esp::Uart uart{uc};
  i::UartPortRef port{uart};
  KB_CHECK(port.EnsureReady() == Status::Ok);
  KB_CHECK(port.Write(one) == Status::Ok);
  fake::UartFeed(one);
  KB_CHECK(port.Read(rx, kb::Milliseconds{1U}).ValueOr(0U) == 1U);
  KB_CHECK(port.FlushInput() == Status::Ok);
}

KB_TEST("contracts: pin, ADC, PWM and CAN references reach the port") {
  Fresh();
  esp::OutputConfig oc{};
  oc.pin = 7;
  esp::OutputPin out{oc};
  KB_REQUIRE(out.EnsureReady() == Status::Ok);
  i::GpioOutputRef out_ref{out};
  KB_CHECK(out_ref.Set(i::Level::High) == Status::Ok);
  KB_CHECK(out_ref.Toggle() == Status::Ok);
  KB_CHECK_EQ(fake::DrivenLevel(7), 0);

  esp::InputConfig inc{};
  inc.pin = 8;
  esp::InputPin in{inc};
  KB_REQUIRE(in.EnsureReady() == Status::Ok);
  i::GpioInputRef in_ref{in};
  i::Level level = i::Level::Low;
  KB_CHECK(in_ref.Get(level) == Status::Ok);
  KB_CHECK(level == i::Level::High);

  esp::AdcConfig ac{};
  ac.pin = 9;
  esp::AdcChannel adc{ac};
  KB_REQUIRE(adc.EnsureReady() == Status::Ok);
  i::AdcChannelRef adc_ref{adc};
  fake::AdcRaw(100);
  KB_CHECK(adc_ref.ReadRaw().ValueOr(0U) == 100U);
  KB_CHECK(adc_ref.ReadMillivolts().IsOk());
  KB_CHECK_EQ(adc_ref.FullScale(), 4095U);

  esp::PwmConfig pc{};
  pc.pin = 10;
  esp::PwmChannel pwm{pc};
  KB_REQUIRE(pwm.EnsureReady() == Status::Ok);
  i::PwmOutputRef pwm_ref{pwm};
  KB_CHECK(pwm_ref.SetFrequencyHz(500U) == Status::Ok);
  KB_CHECK(pwm_ref.SetDuty(i::Duty{5000U}) == Status::Ok);
  KB_CHECK(pwm_ref.Enable(true) == Status::Ok);

  esp::CanConfig cc{};
  cc.tx_pin = 11;
  cc.rx_pin = 12;
  esp::CanController can{cc};
  i::CanBusRef can_ref{can};
  KB_CHECK(can_ref.EnsureReady() == Status::Ok);
  KB_CHECK(can_ref.Send(i::CanFrame{}, kb::Milliseconds{1U}) == Status::Ok);
  i::CanFrame f{};
  KB_CHECK(can_ref.Receive(f, kb::Milliseconds{1U}) == Status::Timeout);
}

KB_TEST("contracts: clock, busy-wait and log references reach the port") {
  Fresh();
  fake::SetTimeUs(42U);
  const esp::Clock clock{};
  const i::MonotonicClockRef clock_ref{clock};
  KB_CHECK(clock_ref.Now().Us() == 42U);
  esp::BusyWait wait{};
  i::BusyWaitRef wait_ref{wait};
  wait_ref.WaitUs(kb::Microseconds{10U});
  KB_CHECK_EQ(fake::Calls("esp_rom_delay_us"), 1U);
  esp::LogSink log{"t"};
  i::LogSinkRef log_ref{log};
  log_ref.Write(i::LogLevel::Info, "hello");
  KB_CHECK(fake::LastLog().find("t: hello") != std::string::npos);
}
