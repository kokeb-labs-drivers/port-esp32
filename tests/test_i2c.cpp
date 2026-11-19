// I2cController over the fake i2c_master: every member, every result and the bus recovery.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <condition_variable>
#include <mutex>
#include <thread>

#include "support.hpp"

namespace {
using kb::iface::I2cFailure;
constexpr int kSda = 8;
constexpr int kScl = 9;
constexpr uint8_t kDev = 0x20U;

esp::I2cConfig Config() {
  esp::I2cConfig c{};
  c.port = 0U;
  c.sda_pin = kSda;
  c.scl_pin = kScl;
  c.clock_hz = 400000U;
  return c;
}

void ReenterWrite(void* context) {
  auto* bus = static_cast<esp::I2cController*>(context);
  const std::array<uint8_t, 1> b{0x00U};
  KB_CHECK_EQ(bus->Write(kDev, b), Status::Busy);
}
} // namespace

KB_TEST("i2c: a bad configuration is refused and nothing is created") {
  Fresh();
  const auto refused = [](esp::I2cConfig c) {
    esp::I2cController bus{c};
    return bus.EnsureReady() == Status::InvalidArgument;
  };
  esp::I2cConfig c = Config();
  c.port = 2U;
  KB_CHECK(refused(c));
  c = Config();
  c.sda_pin = 35; // input only
  KB_CHECK(refused(c));
  c = Config();
  c.scl_pin = -1;
  KB_CHECK(refused(c));
  c = Config();
  c.scl_pin = kSda;
  KB_CHECK(refused(c));
  c = Config();
  c.clock_hz = 0U;
  KB_CHECK(refused(c));
  c = Config();
  c.clock_hz = 1000001U;
  KB_CHECK(refused(c));
  c = Config();
  c.timeout = kb::Milliseconds{0U};
  KB_CHECK(refused(c));
  KB_CHECK_EQ(fake::Calls("i2c_new_master_bus"), 0U);
}

KB_TEST("i2c: every transfer before EnsureReady is NotReady") {
  Fresh();
  esp::I2cController bus{Config()};
  const std::array<uint8_t, 1> tx{0x01U};
  std::array<uint8_t, 1> rx{};
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::NotReady);
  KB_CHECK_EQ(bus.WriteRead(kDev, tx, rx), Status::NotReady);
  KB_CHECK_EQ(bus.Read(kDev, rx), Status::NotReady);
  KB_CHECK_EQ(bus.RecoverBus(), Status::NotReady);
  KB_CHECK(FailedWith(bus.LastError(), Status::NotReady, ESP_ERR_INVALID_STATE));
  KB_CHECK_EQ(bus.Release(), Status::Ok); // nothing to release
}

KB_TEST("i2c: EnsureReady is idempotent and one port has one owner") {
  Fresh();
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(bus.EnsureReady(), Status::Ok);
  KB_CHECK_EQ(fake::Calls("i2c_new_master_bus"), 1U);
  KB_CHECK_EQ(fake::I2cClockHz(), 400000U);
  KB_CHECK(bus.Config().clock_hz == 400000U);
  esp::I2cController other{Config()};
  KB_CHECK_EQ(other.EnsureReady(), Status::Busy);
  KB_CHECK(FailedWith(other.LastError(), Status::Busy, ESP_ERR_NOT_ALLOWED));
  KB_CHECK_EQ(bus.Release(), Status::Ok);
  KB_CHECK_EQ(other.EnsureReady(), Status::Ok); // free again
}

KB_TEST("i2c: SDK failures while creating the bus are mapped and cleaned up") {
  Fresh();
  {
    esp::I2cController bus{Config()};
    fake::Fail("i2c_new_master_bus", ESP_ERR_NO_MEM);
    KB_CHECK_EQ(bus.EnsureReady(), Status::Fault);
    KB_CHECK(FailedWith(bus.LastError(), Status::Fault, ESP_ERR_NO_MEM));
    fake::Fail("i2c_master_bus_add_device", ESP_ERR_INVALID_ARG);
    KB_CHECK_EQ(bus.EnsureReady(), Status::InvalidArgument);
    KB_CHECK_EQ(fake::Calls("i2c_del_master_bus"), 1U);
    KB_CHECK_EQ(bus.EnsureReady(), Status::Ok); // the port is not left owned
  }
  KB_CHECK_EQ(fake::Calls("i2c_master_bus_rm_device"), 1U);
  KB_CHECK_EQ(fake::Calls("i2c_del_master_bus"), 2U);
}

KB_TEST("i2c: Release reports the SDK's failures") {
  Fresh();
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  fake::Fail("i2c_master_bus_rm_device", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(bus.Release(), Status::NotReady);
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  fake::Fail("i2c_del_master_bus", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(bus.Release(), Status::NotReady);
}

KB_TEST("i2c: write, write-read and read reach the device at its address") {
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 2> tx{0x06U, 0xA5U};
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Ok);
  KB_CHECK_EQ(fake::I2cAddress(), kDev);
  KB_CHECK(fake::I2cWritten().size() == 2U);
  KB_CHECK_EQ(fake::I2cWritten()[1], 0xA5U);
  const std::array<uint8_t, 2> reply{0x12U, 0x34U};
  fake::I2cReply(reply);
  std::array<uint8_t, 2> rx{};
  KB_CHECK_EQ(bus.WriteRead(kDev, std::span<const uint8_t>{tx}.first(1), rx), Status::Ok);
  KB_CHECK_EQ(rx[1], 0x34U);
  rx = {};
  KB_CHECK_EQ(bus.Read(kDev, rx), Status::Ok);
  KB_CHECK_EQ(rx[0], 0x12U);
  KB_CHECK_EQ(fake::Calls("i2c_master_device_change_address"), 1U); // the address is kept
  KB_CHECK(bus.LastFailure() == I2cFailure::None);
  KB_CHECK(bus.LastError().IsOk());
  KB_CHECK_EQ(bus.Stats().transfers, 3U);
  KB_CHECK_EQ(bus.Stats().failures, 0U);
}

KB_TEST("i2c: arguments outside the contract are refused before the bus") {
  Fresh();
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 1> tx{0x00U};
  std::array<uint8_t, 1> rx{};
  KB_CHECK_EQ(bus.Write(0x07U, tx), Status::InvalidArgument);
  KB_CHECK_EQ(bus.Write(0x78U, tx), Status::InvalidArgument);
  KB_CHECK_EQ(bus.WriteRead(kDev, std::span<const uint8_t>{}, rx), Status::InvalidArgument);
  KB_CHECK_EQ(bus.WriteRead(kDev, tx, std::span<uint8_t>{}), Status::InvalidArgument);
  KB_CHECK_EQ(bus.Read(kDev, std::span<uint8_t>{}), Status::InvalidArgument);
  KB_CHECK_EQ(bus.Stats().transfers, 0U);
  KB_CHECK(FailedWith(bus.LastError(), Status::InvalidArgument, ESP_ERR_INVALID_ARG));
}

KB_TEST("i2c: an empty write is an address probe") {
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(bus.Write(kDev, std::span<const uint8_t>{}), Status::Ok);
  KB_CHECK_EQ(bus.Write(0x21U, std::span<const uint8_t>{}), Status::NoDevice);
  KB_CHECK(bus.LastFailure() == I2cFailure::AddressNack);
  KB_CHECK(FailedWith(bus.LastError(), Status::NoDevice, ESP_ERR_NOT_FOUND));
  KB_CHECK_EQ(fake::Calls("i2c_master_transmit"), 0U);
}

KB_TEST("i2c: a NACK is unsplit by default: no probe is sent") {
  Fresh();
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 1> tx{0x00U};
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Nack); // nobody at the address
  KB_CHECK(bus.LastFailure() == I2cFailure::DataNack);
  KB_CHECK(FailedWith(bus.LastError(), Status::Nack, ESP_ERR_INVALID_STATE));
  KB_CHECK_EQ(fake::Calls("i2c_master_probe"), 0U); // some devices must not see extra frames
  KB_CHECK_EQ(bus.AllowNackProbe(0x78U, true), Status::InvalidArgument);
}

KB_TEST("i2c: with the probe allowed, address NACK and data NACK are told apart") {
  Fresh();
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  KB_REQUIRE(bus.AllowNackProbe(kDev, true) == Status::Ok);
  const std::array<uint8_t, 1> tx{0x00U};
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::NoDevice);
  KB_CHECK(bus.LastFailure() == I2cFailure::AddressNack);
  fake::I2cPresent(kDev, true);
  fake::I2cDataNack(true);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Nack);
  KB_CHECK(bus.LastFailure() == I2cFailure::DataNack);
  KB_CHECK(FailedWith(bus.LastError(), Status::Nack, ESP_ERR_INVALID_STATE));
  KB_CHECK_EQ(bus.Stats().failures, 2U);
  fake::I2cDataNack(false);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Ok);
  KB_CHECK(bus.LastFailure() == I2cFailure::None);
  KB_REQUIRE(bus.AllowNackProbe(kDev, false) == Status::Ok);
  fake::I2cDataNack(true);
  const uint32_t probes = fake::Calls("i2c_master_probe");
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Nack);
  KB_CHECK_EQ(fake::Calls("i2c_master_probe"), probes);
}

KB_TEST("i2c: a stretch past the limit is a Timeout even when the device ACKs afterwards") {
  // ESP-IDF v5.5 returns ESP_ERR_INVALID_STATE for a NACK and for a stretch timeout alike; the
  // failure is told apart by its time against the transfer's wire time.
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  KB_REQUIRE(bus.AllowNackProbe(kDev, true) == Status::Ok);
  const std::array<uint8_t, 1> tx{0x00U};
  fake::I2cStretchUs(30000U);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Timeout);
  KB_CHECK(bus.LastFailure() == I2cFailure::Timeout);
  KB_CHECK(FailedWith(bus.LastError(), Status::Timeout, ESP_ERR_INVALID_STATE));
  KB_CHECK_EQ(fake::Calls("i2c_master_probe"), 0U); // a timeout needs no probe
  fake::I2cStretchUs(30000U);
  fake::ForceLevel(kScl, 0);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::BusError);
  KB_CHECK(bus.LastFailure() == I2cFailure::BusStuck);
  KB_CHECK_EQ(fake::Calls("i2c_master_bus_reset"), 2U); // start-up, then auto_recover
  fake::ForceLevel(kScl, -1);
}

KB_TEST("i2c: a stretch timeout inside a long transfer is never reported as a NACK") {
  // 200 bytes at 400 kHz take 4.5 ms on the wire; the SCL limit (2 ms) ends a stretch on the first
  // bytes after about 2 ms: earlier than the wire time, later than any NACK could end. Ambiguous.
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  std::array<uint8_t, 200> tx{};
  fake::I2cStretchUs(2000U);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::BusError);
  KB_CHECK(bus.LastFailure() == I2cFailure::Bus); // unsplit, not DataNack
  KB_CHECK_EQ(fake::Calls("i2c_master_probe"), 0U);
  KB_REQUIRE(bus.AllowNackProbe(kDev, true) == Status::Ok);
  fake::I2cStretchUs(2000U);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::BusError); // the device ACKs its address afterwards
  KB_CHECK(bus.LastFailure() == I2cFailure::Bus);
  KB_CHECK_EQ(fake::Calls("i2c_master_probe"), 1U);
  fake::I2cPresent(kDev, false);
  fake::I2cStretchUs(2000U);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::NoDevice); // the probe settles it
  fake::I2cPresent(kDev, true);
  fake::I2cStretchUs(2000U);
  fake::ForceLevel(kSda, 0);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::BusError);
  KB_CHECK(bus.LastFailure() == I2cFailure::BusStuck);
  fake::ForceLevel(kSda, -1);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Ok);
}

KB_TEST("i2c: the earliest timeout is the SCL limit or the call's timeout, capped by the chip") {
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cConfig c = Config();
  c.stretch_limit_us = 500000U; // above the chip's cap (the fake's: 104 ms)
  c.timeout = kb::Milliseconds{200U};
  esp::I2cController bus{c};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 1> tx{0x00U};
  fake::I2cStretchUs(110000U); // after the capped limit: a timeout, even though below 500 ms
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Timeout);
  fake::I2cStretchUs(20000U); // below every limit: only a NACK ends this fast...
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Nack); // ...even when the wait was long (preempted)
  KB_CHECK_EQ(bus.Write(kDev, tx, kb::Milliseconds{10U}), Status::Ok);
  fake::I2cStretchUs(20000U); // past this call's own 10 ms timeout: a timeout
  KB_CHECK_EQ(bus.Write(kDev, tx, kb::Milliseconds{10U}), Status::Timeout);
}

KB_TEST("i2c: a failed probe after a NACK is a timeout, a bus error, or a stuck bus") {
  Fresh();
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  KB_REQUIRE(bus.AllowNackProbe(kDev, true) == Status::Ok);
  const std::array<uint8_t, 1> tx{0x00U};
  fake::Fail("i2c_master_probe", ESP_ERR_TIMEOUT);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Timeout);
  KB_CHECK(bus.LastFailure() == I2cFailure::Timeout);
  fake::Fail("i2c_master_probe", ESP_FAIL);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::BusError);
  KB_CHECK(bus.LastFailure() == I2cFailure::Bus);
  fake::Fail("i2c_master_probe", ESP_FAIL);
  fake::ForceLevel(kSda, 0);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::BusError);
  KB_CHECK(bus.LastFailure() == I2cFailure::BusStuck);
  KB_CHECK_EQ(bus.Stats().recoveries, 1U); // auto_recover
  fake::Fail("i2c_master_probe", ESP_ERR_TIMEOUT);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::BusError);
  KB_CHECK(bus.LastFailure() == I2cFailure::BusStuck);
}

KB_TEST("i2c: ESP_ERR_TIMEOUT of a transfer is ESP-IDF's bus lock: Busy, nothing sent") {
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 1> tx{0x00U};
  fake::Fail("i2c_master_transmit", ESP_ERR_TIMEOUT);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Busy);
  KB_CHECK(bus.LastFailure() == I2cFailure::Timeout);
  KB_CHECK(FailedWith(bus.LastError(), Status::Busy, ESP_ERR_TIMEOUT));
  fake::Fail("i2c_master_probe", ESP_ERR_TIMEOUT); // an empty write: lock or stretch
  KB_CHECK_EQ(bus.Write(kDev, std::span<const uint8_t>{}), Status::Timeout);
  fake::Fail("i2c_master_probe", ESP_ERR_TIMEOUT);
  fake::ForceLevel(kScl, 0);
  KB_CHECK_EQ(bus.Write(kDev, std::span<const uint8_t>{}), Status::BusError);
  KB_CHECK(bus.LastFailure() == I2cFailure::BusStuck);
}

KB_TEST("i2c: without auto_recover a stuck bus waits for RecoverBus") {
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cConfig c = Config();
  c.auto_recover = false;
  esp::I2cController bus{c};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::Calls("i2c_master_bus_reset"), 1U); // the start-up reset
  std::array<uint8_t, 1> rx{};
  fake::Fail("i2c_master_receive", ESP_FAIL);
  fake::ForceLevel(kSda, 0);
  KB_CHECK_EQ(bus.Read(kDev, rx), Status::BusError);
  KB_CHECK(bus.LastFailure() == I2cFailure::BusStuck);
  KB_CHECK_EQ(fake::Calls("i2c_master_bus_reset"), 1U);
  KB_CHECK_EQ(bus.RecoverBus(), Status::BusError); // still held low
  KB_CHECK(bus.LastFailure() == I2cFailure::BusStuck);
  fake::ForceLevel(kSda, -1);
  KB_CHECK_EQ(bus.RecoverBus(), Status::Ok);
  KB_CHECK(bus.LastFailure() == I2cFailure::None);
  KB_CHECK_EQ(bus.Stats().recoveries, 2U);
  fake::Fail("i2c_master_bus_reset", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(bus.RecoverBus(), Status::NotReady);
  fake::Fail("i2c_master_receive", ESP_FAIL);
  KB_CHECK_EQ(bus.Read(kDev, rx), Status::BusError);
  KB_CHECK(bus.LastFailure() == I2cFailure::Bus);
}

KB_TEST("i2c: SDK argument and address errors are reported as they are") {
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 1> tx{0x00U};
  std::array<uint8_t, 1> rx{};
  fake::Fail("i2c_master_transmit_receive", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(bus.WriteRead(kDev, tx, rx), Status::InvalidArgument);
  fake::Fail("i2c_master_device_change_address", ESP_ERR_TIMEOUT);
  KB_CHECK_EQ(bus.Write(0x30U, tx), Status::Busy); // ESP-IDF's bus lock
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Ok);    // the address is set again after the failure
}

KB_TEST("i2c: timeouts are capped, rounded up to ticks, and may be given per call") {
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cConfig c = Config();
  c.timeout = kb::kMaxMilliseconds; // "no limit"
  c.stretch_limit_us = 500000U;
  esp::I2cController bus{c};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::I2cStretchLimitUs(), 500000U);
  const std::array<uint8_t, 1> tx{0x00U};
  std::array<uint8_t, 2> rx{};
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Ok);
  KB_CHECK_EQ(fake::I2cLastTimeoutMs(), 2001); // kMaxTimeout plus the tick in progress, never -1
  KB_CHECK_EQ(bus.Write(kDev, tx, kb::Milliseconds{5U}), Status::Ok);
  KB_CHECK_EQ(fake::I2cLastTimeoutMs(), 6);
  KB_CHECK_EQ(bus.WriteRead(kDev, tx, rx, kb::Milliseconds{500U}), Status::Ok);
  KB_CHECK_EQ(fake::I2cLastTimeoutMs(), 501);
  KB_CHECK_EQ(bus.Read(kDev, rx, kb::Milliseconds{7U}), Status::Ok);
  KB_CHECK_EQ(fake::I2cLastTimeoutMs(), 8);
  KB_CHECK_EQ(bus.Read(kDev, rx, kb::Milliseconds{0U}), Status::InvalidArgument);
  KB_CHECK_EQ(bus.Write(kDev, tx, kb::Milliseconds{0U}), Status::InvalidArgument);
}

KB_TEST("i2c: each task reads its own last failure on a shared controller") {
  // Two tasks take turns on one controller: each fails or succeeds, lets the other call, then reads
  // LastFailure() and LastError(), which must be its own call's, not the other task's.
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  std::mutex m;
  std::condition_variable cv;
  int turn = 0; // 0: A calls, 1: B calls, 2: A reads, 3: B reads
  int wrong = 0;
  const auto wait_for = [&](std::unique_lock<std::mutex>& lock, int t) {
    cv.wait(lock, [&] { return turn == t; });
  };
  const std::array<uint8_t, 1> tx{0x00U};
  std::thread a{[&] {
    std::unique_lock<std::mutex> lock{m};
    wait_for(lock, 0);
    static_cast<void>(bus.Write(0x21U, tx)); // nobody there
    turn = 1;
    cv.notify_all();
    wait_for(lock, 2);
    wrong += (bus.LastFailure() == I2cFailure::DataNack) ? 0 : 1;
    wrong += (bus.LastError().Error() == Status::Nack) ? 0 : 1;
    turn = 3;
    cv.notify_all();
  }};
  std::thread b{[&] {
    std::unique_lock<std::mutex> lock{m};
    wait_for(lock, 1);
    static_cast<void>(bus.Write(kDev, tx)); // succeeds after A's failure
    turn = 2;
    cv.notify_all();
    wait_for(lock, 3);
    wrong += (bus.LastFailure() == I2cFailure::None) ? 0 : 1;
    wrong += bus.LastError().IsOk() ? 0 : 1;
  }};
  a.join();
  b.join();
  KB_CHECK_EQ(wrong, 0);
  KB_CHECK(bus.LastFailure() == I2cFailure::None); // this task made no call
  KB_CHECK_EQ(bus.Stats().transfers, 2U);
}

KB_TEST("i2c: a call while the controller is held is Busy and sends nothing") {
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 1> tx{0x00U};
  fake::Hook("i2c_master_transmit", ReenterWrite, &bus);
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Ok);
  KB_CHECK_EQ(bus.Stats().lock_timeouts, 1U);
  fake::Hook(
      "i2c_master_bus_reset",
      [](void* context) {
        KB_CHECK_EQ(static_cast<esp::I2cController*>(context)->RecoverBus(), Status::Busy);
      },
      &bus);
  KB_CHECK_EQ(bus.RecoverBus(), Status::Ok);
  KB_CHECK_EQ(bus.Stats().lock_timeouts, 2U);
}

KB_TEST("i2c: Release while a transfer holds the lock is Busy and deletes nothing") {
  Fresh();
  fake::I2cPresent(kDev, true);
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  fake::Hook(
      "i2c_master_transmit",
      [](void* context) {
        KB_CHECK_EQ(static_cast<esp::I2cController*>(context)->Release(), Status::Busy);
      },
      &bus);
  const std::array<uint8_t, 1> tx{0x00U};
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::Ok);
  KB_CHECK_EQ(fake::Calls("i2c_del_master_bus"), 0U);
  KB_CHECK_EQ(bus.Release(), Status::Ok);
  KB_CHECK_EQ(fake::Calls("i2c_del_master_bus"), 1U);
}

KB_TEST("i2c: the port number and the pins have one owner") {
  Fresh();
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  esp::I2cConfig c = Config();
  c.sda_pin = 4; // another pair, the same port
  c.scl_pin = 5;
  esp::I2cController twin{c};
  KB_CHECK_EQ(twin.EnsureReady(), Status::Busy);
  KB_CHECK_EQ(fake::Calls("i2c_new_master_bus"), 1U); // refused before the SDK
  esp::OutputConfig o{};
  o.pin = kSda;
  esp::OutputPin sda{o};
  KB_CHECK_EQ(sda.EnsureReady(), Status::Busy);
  KB_REQUIRE(bus.Release() == Status::Ok);
  KB_CHECK_EQ(twin.EnsureReady(), Status::Ok);
  KB_CHECK_EQ(sda.EnsureReady(), Status::Ok);
}

KB_TEST("i2c: a stretch limit above kMaxTimeout is refused") {
  Fresh();
  esp::I2cConfig c = Config();
  c.stretch_limit_us = 2000001U;
  esp::I2cController bus{c};
  KB_CHECK_EQ(bus.EnsureReady(), Status::InvalidArgument);
}

KB_TEST("i2c: without the clock of the controller every slow-enough failure is ambiguous") {
  Fresh();
  fake::I2cPresent(kDev, true);
  fake::Fail("esp_clk_tree_src_get_freq_hz", ESP_FAIL);
  esp::I2cController bus{Config()};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  fake::I2cDataNack(true);
  const std::array<uint8_t, 1> tx{0x00U};
  KB_CHECK_EQ(bus.Write(kDev, tx), Status::BusError); // no lower bound: never a NACK
  KB_CHECK(bus.LastFailure() == I2cFailure::Bus);
}
