// OutputPin and InputPin over the fake gpio driver: levels, toggling, claims, pulls and interrupts.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "support.hpp"

namespace {
using kb::iface::Edge;
using kb::iface::Level;

void Count(void* context) noexcept {
  ++*static_cast<int*>(context);
}
} // namespace

KB_TEST("gpio: an output drives its initial level first, then sets and toggles") {
  Fresh();
  esp::OutputConfig c{};
  c.pin = 4;
  c.initial = Level::High;
  esp::OutputPin pin{c};
  KB_CHECK_EQ(pin.Set(Level::Low), Status::NotReady);
  KB_CHECK_EQ(pin.Toggle(), Status::NotReady);
  KB_CHECK(FailedWith(pin.LastError(), Status::NotReady, ESP_ERR_INVALID_STATE));
  fake::ForceLevel(4, -1);
  KB_REQUIRE(pin.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(pin.EnsureReady(), Status::Ok);
  KB_CHECK_EQ(fake::DrivenLevel(4), 1);
  KB_CHECK(fake::Mode(4) == GPIO_MODE_INPUT_OUTPUT);
  KB_CHECK_EQ(pin.Set(Level::Low), Status::Ok);
  KB_CHECK_EQ(fake::DrivenLevel(4), 0);
  KB_CHECK_EQ(pin.Toggle(), Status::Ok);
  KB_CHECK_EQ(fake::DrivenLevel(4), 1);
  KB_CHECK_EQ(pin.Toggle(), Status::Ok);
  KB_CHECK_EQ(fake::DrivenLevel(4), 0);
  fake::Fail("gpio_set_level", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(pin.Set(Level::High), Status::InvalidArgument);
  fake::Fail("gpio_set_level", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(pin.Toggle(), Status::InvalidArgument);
  KB_CHECK_EQ(pin.Release(), Status::Ok);
  // Released at the safe level (the initial High), still driven: never gpio_reset_pin, which turns
  // the output off and the pull-up on.
  KB_CHECK(fake::Mode(4) == GPIO_MODE_INPUT_OUTPUT);
  KB_CHECK_EQ(fake::DrivenLevel(4), 1);
  KB_CHECK_EQ(fake::Calls("gpio_reset_pin"), 0U);
  KB_CHECK_EQ(pin.Release(), Status::Ok);
}

KB_TEST("gpio: Release and the destructor drive the configured safe level") {
  Fresh();
  esp::OutputConfig c{};
  c.pin = 21;
  c.initial = Level::Low;
  {
    esp::OutputPin enable{c};
    KB_REQUIRE(enable.EnsureReady() == Status::Ok);
    KB_REQUIRE(enable.Set(Level::High) == Status::Ok); // the load on
  }
  KB_CHECK_EQ(fake::DrivenLevel(21), 0); // off again at scope exit, still an output
  KB_CHECK(fake::Mode(21) == GPIO_MODE_INPUT_OUTPUT);
  c.safe = esp::SafeLevel::High;
  esp::OutputPin active_low{c};
  KB_REQUIRE(active_low.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::DrivenLevel(21), 0);
  KB_CHECK_EQ(active_low.Release(), Status::Ok);
  KB_CHECK_EQ(fake::DrivenLevel(21), 1);
  c.safe = esp::SafeLevel::Low;
  c.initial = Level::High;
  esp::OutputPin low{c};
  KB_REQUIRE(low.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(low.Release(), Status::Ok);
  KB_CHECK_EQ(fake::DrivenLevel(21), 0);
  KB_CHECK_EQ(fake::Calls("gpio_reset_pin"), 0U);
}

KB_TEST("gpio: open drain with a pull-up; low initial level") {
  Fresh();
  esp::OutputConfig c{};
  c.pin = 5;
  c.open_drain = true;
  c.pull = esp::Pull::Up;
  esp::OutputPin pin{c};
  KB_REQUIRE(pin.EnsureReady() == Status::Ok);
  KB_CHECK(fake::Mode(5) == GPIO_MODE_INPUT_OUTPUT_OD);
  KB_CHECK_EQ(fake::DrivenLevel(5), 0);
  esp::OutputConfig d{};
  d.pin = 6;
  d.pull = esp::Pull::Down;
  esp::OutputPin down{d};
  KB_CHECK_EQ(down.EnsureReady(), Status::Ok);
}

KB_TEST("gpio: invalid pins, a second owner and SDK failures are refused") {
  Fresh();
  esp::OutputConfig c{};
  c.pin = 35; // input only
  esp::OutputPin input_only{c};
  KB_CHECK_EQ(input_only.EnsureReady(), Status::InvalidArgument);
  c.pin = 7;
  esp::OutputPin first{c};
  esp::OutputPin second{c};
  KB_REQUIRE(first.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(second.EnsureReady(), Status::Busy);
  KB_CHECK(FailedWith(second.LastError(), Status::Busy, ESP_ERR_NOT_ALLOWED));
  esp::InputConfig i{};
  i.pin = 7;
  esp::InputPin same{i};
  KB_CHECK_EQ(same.EnsureReady(), Status::Busy);
  i.pin = 60;
  esp::InputPin bad{i};
  KB_CHECK_EQ(bad.EnsureReady(), Status::InvalidArgument);
  c.pin = 8;
  esp::OutputPin failing{c};
  fake::Fail("gpio_config", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(failing.EnsureReady(), Status::InvalidArgument);
  KB_CHECK_EQ(failing.EnsureReady(), Status::Ok); // the claim was given back
  fake::Fail("gpio_set_level", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(failing.Release(), Status::InvalidArgument);
}

KB_TEST("gpio: an input reads the pin with its pull") {
  Fresh();
  esp::InputConfig c{};
  c.pin = 34;
  c.pull = esp::Pull::Up;
  esp::InputPin pin{c};
  Level level = Level::Low;
  KB_CHECK_EQ(pin.Get(level), Status::NotReady);
  KB_REQUIRE(pin.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(pin.EnsureReady(), Status::Ok);
  KB_CHECK(fake::Mode(34) == GPIO_MODE_INPUT);
  fake::ForceLevel(34, 0);
  KB_CHECK_EQ(pin.Get(level), Status::Ok);
  KB_CHECK(level == Level::Low);
  fake::ForceLevel(34, 1);
  KB_CHECK_EQ(pin.Get(level), Status::Ok);
  KB_CHECK(level == Level::High);
  KB_CHECK(pin.LastError().IsOk());
  esp::InputConfig d{};
  d.pin = 33;
  d.pull = esp::Pull::Down;
  esp::InputPin down{d};
  fake::Fail("gpio_set_pull_mode", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(down.EnsureReady(), Status::InvalidArgument);
  KB_CHECK_EQ(down.EnsureReady(), Status::Ok);
}

KB_TEST("gpio: interrupts call the callback on the chosen edge and stop when disabled") {
  Fresh();
  esp::InputConfig c{};
  c.pin = 15;
  esp::InputPin pin{c};
  int hits = 0;
  KB_CHECK_EQ(pin.EnableInterrupt(Edge::Falling, Count, &hits), Status::NotReady);
  KB_CHECK_EQ(pin.DisableInterrupt(), Status::NotReady);
  KB_REQUIRE(pin.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(pin.EnableInterrupt(Edge::Falling, nullptr, &hits), Status::InvalidArgument);
  KB_CHECK_EQ(pin.DisableInterrupt(), Status::Ok); // nothing enabled yet
  KB_CHECK_EQ(pin.EnableInterrupt(Edge::Falling, Count, &hits), Status::Ok);
  KB_CHECK(fake::IrqType(15) == GPIO_INTR_NEGEDGE);
  fake::FireIrq(15);
  KB_CHECK_EQ(hits, 1);
  KB_CHECK_EQ(pin.EnableInterrupt(Edge::Rising, Count, &hits),
              Status::Ok); // replaced, service kept
  KB_CHECK(fake::IrqType(15) == GPIO_INTR_POSEDGE);
  KB_CHECK_EQ(pin.EnableInterrupt(Edge::Both, Count, &hits), Status::Ok);
  KB_CHECK(fake::IrqType(15) == GPIO_INTR_ANYEDGE);
  fake::FireIrq(15);
  KB_CHECK_EQ(hits, 2);
  KB_CHECK_EQ(pin.DisableInterrupt(), Status::Ok);
  fake::FireIrq(15);
  KB_CHECK_EQ(hits, 2);
  KB_CHECK(!fake::IrqEnabled(15));
}

KB_TEST("gpio: interrupt SDK failures are reported") {
  Fresh();
  esp::InputConfig c{};
  c.pin = 16;
  esp::InputPin pin{c};
  KB_REQUIRE(pin.EnsureReady() == Status::Ok);
  int hits = 0;
  fake::Fail("gpio_install_isr_service", ESP_ERR_NO_MEM);
  KB_CHECK_EQ(pin.EnableInterrupt(Edge::Rising, Count, &hits), Status::Fault);
  fake::Fail("gpio_set_intr_type", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(pin.EnableInterrupt(Edge::Rising, Count, &hits), Status::InvalidArgument);
  fake::Fail("gpio_isr_handler_add", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(pin.EnableInterrupt(Edge::Rising, Count, &hits), Status::NotReady);
  fake::Fail("gpio_intr_enable", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(pin.EnableInterrupt(Edge::Rising, Count, &hits), Status::InvalidArgument);
  KB_CHECK_EQ(pin.EnableInterrupt(Edge::Rising, Count, &hits), Status::Ok);
  fake::Fail("gpio_intr_disable", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(pin.DisableInterrupt(), Status::InvalidArgument);
  fake::Fail("gpio_isr_handler_remove", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(pin.DisableInterrupt(), Status::NotReady);
  fake::Fail("gpio_intr_disable", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(pin.Release(), Status::InvalidArgument); // reported, the pin is released anyway
  KB_CHECK_EQ(pin.Release(), Status::Ok);
  KB_CHECK(fake::Mode(16) == GPIO_MODE_INPUT); // stays an input, no reset
}
