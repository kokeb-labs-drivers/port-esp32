// CanController over the fake twai driver: frames both ways, bit rates, bus-off recovery, status
// and every refusal. Test code. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32
// authors
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "support.hpp"

namespace {
using kb::iface::CanFrame;

esp::CanConfig Config() {
  esp::CanConfig c{};
  c.controller = 0U;
  c.tx_pin = 21;
  c.rx_pin = 22;
  c.bitrate = 500000U;
  return c;
}

void ReenterRecover(void* context) {
  KB_CHECK_EQ(static_cast<esp::CanController*>(context)->Recover(kb::Milliseconds{1U}),
              Status::Busy);
}
} // namespace

KB_TEST("can: frame validity follows CAN 2.0") {
  CanFrame f{};
  f.id = 0x7FFU;
  f.dlc = 8U;
  KB_CHECK(esp::CanFrameValid(f));
  f.id = 0x800U;
  KB_CHECK(!esp::CanFrameValid(f));
  f.extended = true;
  KB_CHECK(esp::CanFrameValid(f));
  f.id = 0x20000000U;
  KB_CHECK(!esp::CanFrameValid(f));
  f.id = 1U;
  f.dlc = 9U;
  KB_CHECK(!esp::CanFrameValid(f));
}

KB_TEST("can: EnsureReady installs and starts once; every bit rate has a timing") {
  Fresh();
  const std::array<uint32_t, 8> rates{25000U,  50000U,  100000U, 125000U,
                                      250000U, 500000U, 800000U, 1000000U};
  const std::array<uint32_t, 8> quanta{500000U,  1000000U,  2000000U,  2000000U,
                                       4000000U, 10000000U, 20000000U, 20000000U};
  for (std::size_t i = 0U; i < rates.size(); ++i) {
    esp::CanConfig c = Config();
    c.bitrate = rates[i];
    c.mode = (i == 0U) ? esp::CanMode::NoAck
                       : ((i == 1U) ? esp::CanMode::ListenOnly : esp::CanMode::Normal);
    esp::CanController can{c};
    KB_REQUIRE(can.EnsureReady() == Status::Ok);
    KB_CHECK_EQ(can.EnsureReady(), Status::Ok);
    KB_CHECK_EQ(fake::TwaiBitrateQuanta(), quanta[i]);
  }
  KB_CHECK_EQ(fake::TwaiHandlesAlive(), 0);
  KB_CHECK_EQ(fake::Calls("twai_driver_install_v2"), 8U);
  esp::CanConfig c = Config();
  c.bitrate = 333333U;
  esp::CanController odd{c};
  KB_CHECK_EQ(odd.EnsureReady(), Status::InvalidArgument);
}

KB_TEST("can: bad configurations, a second owner and SDK failures are refused") {
  Fresh();
  const auto status = [](esp::CanConfig c) {
    esp::CanController can{c};
    return can.EnsureReady();
  };
  esp::CanConfig c = Config();
  c.controller = 2U;
  KB_CHECK_EQ(status(c), Status::InvalidArgument);
  c = Config();
  c.tx_pin = 35;
  KB_CHECK_EQ(status(c), Status::InvalidArgument);
  c = Config();
  c.rx_pin = -1;
  KB_CHECK_EQ(status(c), Status::InvalidArgument);
  c = Config();
  c.tx_queue = 0U;
  KB_CHECK_EQ(status(c), Status::InvalidArgument);
  c = Config();
  c.rx_queue = 0U;
  KB_CHECK_EQ(status(c), Status::InvalidArgument);
  esp::CanController can{Config()};
  KB_REQUIRE(can.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(status(Config()), Status::Busy);
  KB_CHECK_EQ(can.Release(), Status::Ok);
  fake::Fail("twai_driver_install_v2", ESP_ERR_NO_MEM);
  KB_CHECK_EQ(can.EnsureReady(), Status::Fault);
  fake::Fail("twai_start_v2", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(can.EnsureReady(), Status::NotReady);
  KB_CHECK_EQ(fake::TwaiHandlesAlive(), 0);
  KB_CHECK_EQ(can.EnsureReady(), Status::Ok);
  fake::Fail("twai_driver_uninstall_v2", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(can.Release(), Status::NotReady);
}

KB_TEST("can: Send and Receive carry the identifier, flags and data") {
  Fresh();
  esp::CanController can{Config()};
  CanFrame f{};
  KB_CHECK_EQ(can.Send(f, kb::Milliseconds{1U}), Status::NotReady);
  KB_CHECK_EQ(can.Receive(f, kb::Milliseconds{1U}), Status::NotReady);
  KB_REQUIRE(can.EnsureReady() == Status::Ok);
  f.id = 0x18FEF100U;
  f.extended = true;
  f.remote = true;
  f.dlc = 3U;
  f.data = {1U, 2U, 3U, 0U, 0U, 0U, 0U, 0U};
  KB_CHECK_EQ(can.Send(f, kb::Milliseconds{5U}), Status::Ok);
  KB_REQUIRE(fake::TwaiSent().size() == 1U);
  KB_CHECK_EQ(fake::TwaiSent()[0].identifier, 0x18FEF100U);
  KB_CHECK_EQ(fake::TwaiSent()[0].flags, TWAI_MSG_FLAG_EXTD | TWAI_MSG_FLAG_RTR);
  KB_CHECK_EQ(fake::TwaiSent()[0].data[2], 3U);
  f.dlc = 9U;
  KB_CHECK_EQ(can.Send(f, kb::Milliseconds{5U}), Status::InvalidArgument);
  twai_message_t m{};
  m.identifier = 0x123U;
  m.data_length_code = 12U; // a non-compliant DLC is cut to 8
  m.data[7] = 0x77U;
  fake::TwaiFeed(m);
  CanFrame got{};
  KB_CHECK_EQ(can.Receive(got, kb::Milliseconds{5U}), Status::Ok);
  KB_CHECK_EQ(got.id, 0x123U);
  KB_CHECK(!got.extended && !got.remote);
  KB_CHECK_EQ(got.dlc, 8U);
  KB_CHECK_EQ(got.data[7], 0x77U);
  KB_CHECK_EQ(can.Receive(got, kb::Milliseconds{5U}), Status::Timeout);
  KB_CHECK(FailedWith(can.LastError(), Status::Timeout, ESP_ERR_TIMEOUT));
}

KB_TEST("can: send failures: full queue, bus-off, refusal") {
  Fresh();
  esp::CanController can{Config()};
  KB_REQUIRE(can.EnsureReady() == Status::Ok);
  CanFrame f{};
  f.id = 1U;
  fake::Fail("twai_transmit_v2", ESP_ERR_TIMEOUT);
  KB_CHECK_EQ(can.Send(f, kb::Milliseconds{1U}), Status::Timeout);
  fake::Fail("twai_transmit_v2", ESP_FAIL);
  KB_CHECK_EQ(can.Send(f, kb::Milliseconds{1U}), Status::BusError);
  fake::TwaiState(TWAI_STATE_BUS_OFF);
  KB_CHECK_EQ(can.Send(f, kb::Milliseconds{1U}), Status::BusError);
  KB_CHECK(FailedWith(can.LastError(), Status::BusError, ESP_ERR_INVALID_STATE));
  fake::Fail("twai_transmit_v2", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(can.Send(f, kb::Milliseconds{1U}), Status::InvalidArgument);
  fake::Fail("twai_receive_v2", ESP_ERR_INVALID_STATE);
  CanFrame got{};
  KB_CHECK_EQ(can.Receive(got, kb::Milliseconds{1U}), Status::BusError);
}

KB_TEST("can: Status reports state and counters") {
  Fresh();
  esp::CanController can{Config()};
  KB_CHECK(FailedWith(can.Status(), Status::NotReady, ESP_ERR_INVALID_STATE));
  KB_REQUIRE(can.EnsureReady() == Status::Ok);
  const kb::Result<esp::CanStatus> s = can.Status();
  KB_REQUIRE(s.IsOk());
  KB_CHECK(s.Value().state == esp::CanState::Running);
  KB_CHECK_EQ(s.Value().tx_error_counter, 7U);
  KB_CHECK_EQ(s.Value().rx_error_counter, 3U);
  KB_CHECK_EQ(s.Value().bus_errors, 1U);
  fake::TwaiState(TWAI_STATE_BUS_OFF);
  KB_CHECK(can.Status().Value().state == esp::CanState::BusOff);
  fake::TwaiState(TWAI_STATE_STOPPED);
  KB_CHECK(can.Status().Value().state == esp::CanState::Stopped);
  fake::TwaiState(TWAI_STATE_RECOVERING);
  KB_CHECK(can.Status().Value().state == esp::CanState::Recovering);
  fake::Fail("twai_get_status_info_v2", ESP_ERR_INVALID_ARG);
  KB_CHECK(FailedWith(can.Status(), Status::InvalidArgument, ESP_ERR_INVALID_ARG));
}

KB_TEST("can: Recover brings a bus-off controller back; a stopped one is started") {
  Fresh();
  esp::CanController can{Config()};
  KB_CHECK_EQ(can.Recover(kb::Milliseconds{10U}), Status::NotReady);
  KB_REQUIRE(can.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(can.Recover(kb::Milliseconds{10U}), Status::Ok); // running: nothing to do
  KB_CHECK_EQ(fake::Calls("twai_initiate_recovery_v2"), 0U);
  fake::TwaiState(TWAI_STATE_BUS_OFF);
  fake::TwaiRecoveryReads(3U);
  KB_CHECK_EQ(can.Recover(kb::Milliseconds{10U}), Status::Ok);
  KB_CHECK_EQ(fake::Calls("twai_initiate_recovery_v2"), 1U);
  KB_CHECK(can.Status().Value().state == esp::CanState::Running);
  fake::TwaiState(TWAI_STATE_STOPPED);
  KB_CHECK_EQ(can.Recover(kb::Milliseconds{10U}), Status::Ok);
  KB_CHECK(can.Status().Value().state == esp::CanState::Running);
}

KB_TEST("can: Recover is bounded and reports SDK failures") {
  Fresh();
  esp::CanController can{Config()};
  KB_REQUIRE(can.EnsureReady() == Status::Ok);
  fake::TwaiState(TWAI_STATE_BUS_OFF);
  fake::TwaiRecoveryReads(1000000U);
  KB_CHECK_EQ(can.Recover(kb::Milliseconds{5U}), Status::Timeout);
  fake::TwaiState(TWAI_STATE_BUS_OFF);
  fake::Fail("twai_initiate_recovery_v2", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(can.Recover(kb::Milliseconds{5U}), Status::NotReady);
  fake::Fail("twai_get_status_info_v2", ESP_FAIL);
  KB_CHECK_EQ(can.Recover(kb::Milliseconds{5U}), Status::Fault);
  fake::TwaiState(TWAI_STATE_STOPPED);
  fake::Fail("twai_start_v2", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(can.Recover(kb::Milliseconds{5U}), Status::NotReady);
  fake::TwaiState(TWAI_STATE_STOPPED);
  fake::Hook("twai_get_status_info_v2", ReenterRecover, &can);
  KB_CHECK_EQ(can.Recover(kb::Milliseconds{5U}), Status::Ok);
}

KB_TEST("can: Release waits for running calls; timeouts are capped at kMaxTimeout") {
  Fresh();
  esp::CanController can{Config()};
  KB_REQUIRE(can.EnsureReady() == Status::Ok);
  fake::Hook(
      "twai_receive_v2",
      [](void* context) {
        // A Release from another task while Receive runs: refused, nothing uninstalled.
        KB_CHECK_EQ(static_cast<esp::CanController*>(context)->Release(), Status::Busy);
      },
      &can);
  CanFrame frame{};
  KB_CHECK_EQ(can.Receive(frame, kb::kMaxMilliseconds), Status::Timeout);
  KB_CHECK_EQ(fake::TwaiHandlesAlive(), 1);
  KB_CHECK_EQ(can.Release(), Status::Ok);
  KB_CHECK_EQ(fake::TwaiHandlesAlive(), 0);
  KB_CHECK_EQ(can.Send(frame, kb::Milliseconds{1U}), Status::NotReady);
}

KB_TEST("can: Release waits for a running call and succeeds when it returns") {
  // Deterministic: the receiving thread is held inside twai_receive_v2 until Release() has begun
  // its wait; Release's first sleep lets it go and waits until the call has returned. The two
  // threads never touch the fake at the same time.
  Fresh();
  esp::CanController can{Config()};
  KB_REQUIRE(can.EnsureReady() == Status::Ok);
  struct Sync {
    std::mutex m;
    std::condition_variable cv;
    int step{0}; // 1: in the call, 2: go, 3: returned
  } sync;
  const auto wait_step = [&sync](std::unique_lock<std::mutex>& lock, int step) {
    sync.cv.wait(lock, [&sync, step] { return sync.step >= step; });
  };
  fake::Hook(
      "twai_receive_v2",
      [](void* context) {
        auto* s = static_cast<Sync*>(context);
        std::unique_lock<std::mutex> lock{s->m};
        s->step = 1;
        s->cv.notify_all();
        // Bounded, so a Release() that does not wait fails the test instead of hanging it.
        static_cast<void>(
            s->cv.wait_for(lock, std::chrono::seconds{5}, [s] { return s->step >= 2; }));
      },
      &sync);
  fake::Hook(
      "vTaskDelay",
      [](void* context) {
        auto* s = static_cast<Sync*>(context);
        std::unique_lock<std::mutex> lock{s->m};
        s->step = 2;
        s->cv.notify_all();
        s->cv.wait(lock, [s] { return s->step >= 3; });
      },
      &sync);
  std::thread receiver{[&can, &sync] {
    CanFrame frame{};
    static_cast<void>(can.Receive(frame, kb::Milliseconds{1U}));
    const std::lock_guard<std::mutex> lock{sync.m};
    sync.step = 3;
    sync.cv.notify_all();
  }};
  {
    std::unique_lock<std::mutex> lock{sync.m};
    wait_step(lock, 1);
  }
  KB_CHECK_EQ(can.Release(), Status::Ok); // waited, then uninstalled
  receiver.join();
  KB_CHECK_EQ(fake::TwaiHandlesAlive(), 0);
}
