// Uart over the fake uart driver: framing, the bounded FIFO writer, the first-byte reader, latched
// errors, baud changes, RS-485 and every refusal.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <vector>

#include "support.hpp"

namespace {
esp::UartConfig Config() {
  esp::UartConfig c{};
  c.port = 1U;
  c.tx_pin = 17;
  c.rx_pin = 18;
  c.baud = 115200U;
  return c;
}

void ReenterRead(void* context) {
  std::array<uint8_t, 1> rx{};
  const kb::Result<std::size_t> r =
      static_cast<esp::Uart*>(context)->Read(rx, kb::Milliseconds{1U});
  KB_CHECK(!r.IsOk() && (r.Error() == Status::Busy));
}

void ReenterWrite(void* context) {
  const std::array<uint8_t, 1> tx{1U};
  KB_CHECK_EQ(static_cast<esp::Uart*>(context)->Write(tx), Status::Busy);
}
} // namespace

KB_TEST("uart: a bad configuration is refused") {
  Fresh();
  const auto refused = [](esp::UartConfig c) {
    esp::Uart u{c};
    return u.EnsureReady() == Status::InvalidArgument;
  };
  esp::UartConfig c = Config();
  c.port = 3U;
  KB_CHECK(refused(c));
  c = Config();
  c.tx_pin = 34;
  KB_CHECK(refused(c));
  c = Config();
  c.rx_pin = -1;
  KB_CHECK(refused(c));
  c = Config();
  c.baud = 0U;
  KB_CHECK(refused(c));
  c = Config();
  c.baud = 5000001U;
  KB_CHECK(refused(c));
  c = Config();
  c.data_bits = 4U;
  KB_CHECK(refused(c));
  c = Config();
  c.data_bits = 9U;
  KB_CHECK(refused(c));
  c = Config();
  c.rs485 = true; // needs an RTS pin
  KB_CHECK(refused(c));
  c = Config();
  c.write_timeout = kb::Milliseconds{0U};
  KB_CHECK(refused(c));
}

KB_TEST("uart: the frame reaches the SDK; RS-485 selects half duplex") {
  Fresh();
  esp::UartConfig c = Config();
  c.data_bits = 7U;
  c.parity = esp::UartParity::Even;
  c.stop_bits = esp::UartStopBits::Two;
  c.cts_pin = 20;
  {
    esp::Uart u{c};
    KB_REQUIRE(u.EnsureReady() == Status::Ok);
    KB_CHECK_EQ(u.EnsureReady(), Status::Ok);
    KB_CHECK(fake::UartLastConfig().parity == UART_PARITY_EVEN);
    KB_CHECK(fake::UartLastConfig().stop_bits == UART_STOP_BITS_2);
    KB_CHECK(fake::UartLastConfig().flow_ctrl == UART_HW_FLOWCTRL_CTS);
    KB_CHECK_EQ(static_cast<int>(fake::UartLastConfig().data_bits), 2);
    KB_CHECK(fake::UartMode() == UART_MODE_UART);
  }
  c = Config();
  c.parity = esp::UartParity::Odd;
  c.rs485 = true;
  c.rts_pin = 21;
  esp::Uart u{c};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  KB_CHECK(fake::UartLastConfig().parity == UART_PARITY_ODD);
  KB_CHECK(fake::UartMode() == UART_MODE_RS485_HALF_DUPLEX);
  KB_CHECK(u.Config().rs485);
}

KB_TEST("uart: SDK failures while installing are mapped and the driver removed") {
  Fresh();
  esp::Uart u{Config()};
  fake::Fail("uart_driver_install", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(u.EnsureReady(), Status::InvalidArgument);
  fake::Fail("uart_param_config", ESP_FAIL);
  KB_CHECK_EQ(u.EnsureReady(), Status::Fault);
  fake::Fail("uart_set_pin", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(u.EnsureReady(), Status::InvalidArgument);
  esp::UartConfig c = Config();
  c.rs485 = true;
  c.rts_pin = 21;
  esp::Uart rs{c};
  fake::Fail("uart_set_mode", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(rs.EnsureReady(), Status::NotReady);
  KB_CHECK_EQ(fake::Calls("uart_driver_delete"), 3U);
}

KB_TEST("uart: calls before EnsureReady are NotReady; empty spans are refused") {
  Fresh();
  esp::Uart u{Config()};
  const std::array<uint8_t, 1> tx{1U};
  std::array<uint8_t, 1> rx{};
  KB_CHECK_EQ(u.Write(tx), Status::NotReady);
  KB_CHECK(FailedWith(u.Read(rx, kb::Milliseconds{1U}), Status::NotReady, ESP_ERR_INVALID_STATE));
  KB_CHECK_EQ(u.FlushInput(), Status::NotReady);
  KB_CHECK_EQ(u.WaitTxDone(kb::Milliseconds{1U}), Status::NotReady);
  KB_CHECK_EQ(u.Write(std::span<const uint8_t>{}), Status::InvalidArgument);
  KB_CHECK(FailedWith(u.Read(std::span<uint8_t>{}, kb::Milliseconds{1U}), Status::InvalidArgument,
                      ESP_ERR_INVALID_ARG));
  KB_CHECK_EQ(u.Release(), Status::Ok);
}

KB_TEST("uart: Write fills the FIFO in pieces until every byte is in") {
  Fresh();
  esp::Uart u{Config()};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  fake::UartFifoSpace(16U);
  std::vector<uint8_t> tx(40U, 0x7EU);
  KB_CHECK_EQ(u.Write(tx), Status::Ok);
  KB_CHECK_EQ(fake::UartWritten().size(), 40U);
  KB_CHECK_EQ(fake::Calls("uart_tx_chars"), 3U);
  KB_CHECK_EQ(fake::Calls("esp_rom_delay_us"), 2U); // 16 characters at 115200 baud is below 1 ms
  KB_CHECK_EQ(u.Stats().bytes_written, 40U);
}

KB_TEST("uart: at a slow baud rate Write sleeps between pieces") {
  Fresh();
  esp::UartConfig c = Config();
  c.baud = 9600U;
  esp::Uart u{c};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  fake::UartFifoSpace(8U);
  std::vector<uint8_t> tx(20U, 0x01U);
  KB_CHECK_EQ(u.Write(tx), Status::Ok);
  KB_CHECK_EQ(fake::Calls("vTaskDelay"), 2U);
}

KB_TEST("uart: Write gives up after write_timeout when the FIFO does not drain") {
  Fresh();
  esp::Uart u{Config()};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  fake::UartFifoSpace(0U); // CTS held: nothing leaves
  fake::SetTimeStepUs(10000U);
  const std::array<uint8_t, 4> tx{1U, 2U, 3U, 4U};
  KB_CHECK_EQ(u.Write(tx), Status::Timeout);
  KB_CHECK(FailedWith(u.LastError(), Status::Timeout, ESP_ERR_TIMEOUT));
  fake::Fail("uart_tx_chars", ESP_FAIL);
  KB_CHECK_EQ(u.Write(tx), Status::Fault);
}

KB_TEST("uart: Read waits for the first byte only, then takes what is there") {
  Fresh();
  esp::Uart u{Config()};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 3> in{0x10U, 0x20U, 0x30U};
  fake::UartFeed(in);
  std::array<uint8_t, 8> rx{};
  const kb::Result<std::size_t> r = u.Read(rx, kb::Milliseconds{25U});
  KB_REQUIRE(r.IsOk());
  KB_CHECK_EQ(r.Value(), 3U);
  KB_CHECK_EQ(rx[2], 0x30U);
  KB_CHECK_EQ(fake::UartLastReadTicks(), 0U); // the rest without waiting
  KB_CHECK_EQ(u.Stats().bytes_read, 3U);
  fake::UartFeed(std::span<const uint8_t>{in}.first(1));
  std::array<uint8_t, 1> one{};
  KB_CHECK(u.Read(one, kb::Milliseconds{5U}).ValueOr(0U) == 1U);
  KB_CHECK_EQ(fake::UartLastReadTicks(), 5U);
  KB_CHECK(FailedWith(u.Read(rx, kb::Milliseconds{7U}), Status::Timeout, ESP_ERR_TIMEOUT));
  KB_CHECK_EQ(fake::UartLastReadTicks(), 7U);
}

KB_TEST("uart: SDK read failures are reported") {
  Fresh();
  esp::Uart u{Config()};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  std::array<uint8_t, 4> rx{};
  fake::Fail("uart_read_bytes", ESP_FAIL);
  KB_CHECK(FailedWith(u.Read(rx, kb::Milliseconds{1U}), Status::Fault, ESP_FAIL));
  const std::array<uint8_t, 2> in{1U, 2U};
  fake::UartFeed(in);
  fake::Fail("uart_get_buffered_data_len", ESP_ERR_INVALID_ARG);
  KB_CHECK(
      FailedWith(u.Read(rx, kb::Milliseconds{1U}), Status::InvalidArgument, ESP_ERR_INVALID_ARG));
}

KB_TEST("uart: receive errors are reported once by the next Read, data stays") {
  Fresh();
  esp::Uart u{Config()};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 2> in{0xAAU, 0xBBU};
  fake::UartFeed(in);
  fake::UartEvent(UART_DATA);
  fake::UartEvent(UART_FIFO_OVF);
  std::array<uint8_t, 4> rx{};
  KB_CHECK(FailedWith(u.Read(rx, kb::Milliseconds{1U}), Status::Overflow,
                      static_cast<int>(esp::uart_flags::kFifoOverflow)));
  KB_CHECK(u.Read(rx, kb::Milliseconds{1U}).ValueOr(0U) == 2U);
  fake::UartEvent(UART_FRAME_ERR);
  fake::UartEvent(UART_PARITY_ERR);
  fake::UartEvent(UART_BREAK);
  KB_CHECK(FailedWith(u.Read(rx, kb::Milliseconds{1U}), Status::BusError,
                      static_cast<int>(esp::uart_flags::kFrameError |
                                       esp::uart_flags::kParityError | esp::uart_flags::kBreak)));
  fake::UartEvent(UART_BUFFER_FULL);
  KB_CHECK(FailedWith(u.Read(rx, kb::Milliseconds{1U}), Status::Overflow,
                      static_cast<int>(esp::uart_flags::kBufferFull)));
  KB_CHECK_EQ(u.Stats().errors, 3U);
}

KB_TEST("uart: FlushInput drops data and latched errors") {
  Fresh();
  esp::Uart u{Config()};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 2> in{1U, 2U};
  fake::UartFeed(in);
  fake::UartEvent(UART_FRAME_ERR);
  KB_CHECK_EQ(u.FlushInput(), Status::Ok);
  std::array<uint8_t, 4> rx{};
  KB_CHECK(FailedWith(u.Read(rx, kb::Milliseconds{1U}), Status::Timeout, ESP_ERR_TIMEOUT));
  fake::Fail("uart_flush_input", ESP_FAIL);
  KB_CHECK_EQ(u.FlushInput(), Status::Fault);
}

KB_TEST("uart: SetBaud before and after EnsureReady; WaitTxDone and Release") {
  Fresh();
  esp::Uart u{Config()};
  KB_CHECK_EQ(u.SetBaud(0U), Status::InvalidArgument);
  KB_CHECK_EQ(u.SetBaud(6000000U), Status::InvalidArgument);
  KB_CHECK_EQ(u.SetBaud(57600U), Status::Ok);
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::UartBaud(), 57600U);
  KB_CHECK_EQ(u.SetBaud(230400U), Status::Ok);
  KB_CHECK_EQ(fake::UartBaud(), 230400U);
  KB_CHECK(u.Config().baud == 230400U);
  fake::Fail("uart_set_baudrate", ESP_FAIL);
  KB_CHECK_EQ(u.SetBaud(9600U), Status::Fault);
  KB_CHECK(u.Config().baud == 230400U);
  KB_CHECK_EQ(u.WaitTxDone(kb::Milliseconds{10U}), Status::Ok);
  fake::Fail("uart_wait_tx_done", ESP_ERR_TIMEOUT);
  KB_CHECK_EQ(u.WaitTxDone(kb::Milliseconds{10U}), Status::Timeout);
  fake::Fail("uart_driver_delete", ESP_FAIL);
  KB_CHECK_EQ(u.Release(), Status::Fault);
}

KB_TEST("uart: reader and writer locks are separate and bounded") {
  Fresh();
  esp::Uart u{Config()};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 1> in{9U};
  fake::UartFeed(in);
  fake::Hook("uart_read_bytes", ReenterRead, &u);
  std::array<uint8_t, 1> rx{};
  KB_CHECK(u.Read(rx, kb::Milliseconds{1U}).IsOk());
  fake::Hook("uart_tx_chars", ReenterWrite, &u);
  const std::array<uint8_t, 1> tx{1U};
  KB_CHECK_EQ(u.Write(tx), Status::Ok);
  fake::Hook(
      "uart_tx_chars",
      [](void* context) {
        auto* uart = static_cast<esp::Uart*>(context);
        KB_CHECK_EQ(uart->SetBaud(9600U), Status::Busy);
        KB_CHECK_EQ(uart->FlushInput(), Status::Ok); // the reader lock is free
      },
      &u);
  KB_CHECK_EQ(u.Write(tx), Status::Ok);
  fake::Hook(
      "uart_read_bytes",
      [](void* context) {
        KB_CHECK_EQ(static_cast<esp::Uart*>(context)->FlushInput(), Status::Busy);
      },
      &u);
  fake::UartFeed(in);
  KB_CHECK(u.Read(rx, kb::Milliseconds{1U}).IsOk());
  KB_CHECK_EQ(u.Stats().lock_timeouts, 4U);
}

KB_TEST("uart: a full event queue is reported as Overflow (events lost), never as Ok") {
  Fresh();
  esp::UartConfig c = Config();
  c.event_queue = 4U;
  esp::Uart u{c};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 2> in{1U, 2U};
  fake::UartFeed(in);
  for (int i = 0; i < 4; ++i) {
    fake::UartEvent(UART_DATA); // a reader slower than the data bursts
  }
  fake::UartEvent(UART_FRAME_ERR); // dropped by the SDK: the queue is full
  KB_CHECK_EQ(fake::UartEventsDropped(), 1U);
  std::array<uint8_t, 4> rx{};
  KB_CHECK(FailedWith(u.Read(rx, kb::Milliseconds{1U}), Status::Overflow,
                      static_cast<int>(esp::uart_flags::kEventsLost)));
  KB_CHECK(u.Read(rx, kb::Milliseconds{1U}).ValueOr(0U) == 2U); // the bytes stay readable
  esp::UartConfig small = Config();
  small.port = 2U;
  small.tx_pin = 25;
  small.rx_pin = 26;
  small.event_queue = 3U;
  esp::Uart refused{small};
  KB_CHECK_EQ(refused.EnsureReady(), Status::InvalidArgument);
}

KB_TEST("uart: WaitTxDone holds the writer lock; waits are capped at kMaxTimeout") {
  Fresh();
  esp::Uart u{Config()};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  fake::Hook(
      "uart_wait_tx_done",
      [](void* context) {
        const std::array<uint8_t, 1> tx{1U};
        KB_CHECK_EQ(static_cast<esp::Uart*>(context)->Write(tx), Status::Busy);
      },
      &u);
  KB_CHECK_EQ(u.WaitTxDone(kb::Milliseconds{10U}), Status::Ok);
  KB_CHECK_EQ(u.Stats().lock_timeouts, 1U);
  std::array<uint8_t, 1> rx{};
  KB_CHECK(!u.Read(rx, kb::kMaxMilliseconds).IsOk()); // nothing arrives
  KB_CHECK_EQ(fake::UartLastReadTicks(), 2000U);      // 2 s at 1 kHz, never portMAX_DELAY
}

KB_TEST("uart: Release takes both locks and the pins are claimed") {
  Fresh();
  esp::Uart u{Config()};
  KB_REQUIRE(u.EnsureReady() == Status::Ok);
  esp::OutputConfig pin{};
  pin.pin = 17; // TXD
  esp::OutputPin tx_pin{pin};
  KB_CHECK_EQ(tx_pin.EnsureReady(), Status::Busy);
  fake::Hook(
      "uart_read_bytes",
      [](void* context) {
        KB_CHECK_EQ(static_cast<esp::Uart*>(context)->Release(), Status::Busy); // a Read runs
      },
      &u);
  const std::array<uint8_t, 1> in{9U};
  fake::UartFeed(in);
  std::array<uint8_t, 1> rx{};
  KB_CHECK(u.Read(rx, kb::Milliseconds{1U}).IsOk());
  KB_CHECK_EQ(fake::Calls("uart_driver_delete"), 0U);
  KB_CHECK_EQ(u.Release(), Status::Ok);
  KB_CHECK_EQ(tx_pin.EnsureReady(), Status::Ok);
}

KB_TEST("uart: a second object on the port is refused and leaves the first one's check intact") {
  Fresh();
  esp::UartConfig a = Config();
  a.event_queue = 4U;
  esp::Uart first{a};
  KB_REQUIRE(first.EnsureReady() == Status::Ok);
  esp::UartConfig b = Config();
  b.tx_pin = 25;
  b.rx_pin = 26;
  b.event_queue = 32U;
  esp::Uart second{b};
  KB_CHECK_EQ(second.EnsureReady(), Status::Busy);
  KB_CHECK_EQ(fake::Calls("uart_driver_install"), 1U);
  for (int i = 0; i < 4; ++i) {
    fake::UartEvent(UART_DATA);
  }
  std::array<uint8_t, 1> rx{};
  KB_CHECK(FailedWith(first.Read(rx, kb::Milliseconds{1U}), Status::Overflow,
                      static_cast<int>(esp::uart_flags::kEventsLost)));
}
