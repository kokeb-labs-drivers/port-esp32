// SpiBus and SpiDevice over the fake spi_master (which models ESP-IDF v5.5: polling_start takes
// only portMAX_DELAY and the SDK bus lock, polling_end gives it back only when the transaction
// ended, remove_device refuses a device with a transaction in flight): frames, waits sized to the
// wire time, late and never-ending transactions, clock changes and every refusal.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <vector>

#include "support.hpp"

namespace {
esp::SpiBusConfig BusConfig() {
  esp::SpiBusConfig c{};
  c.host = 1U;
  c.mosi_pin = 11;
  c.miso_pin = 13;
  c.sclk_pin = 12;
  return c;
}

esp::SpiDeviceConfig DevConfig() {
  esp::SpiDeviceConfig c{};
  c.cs_pin = 10;
  c.mode = 3U;
  c.clock_hz = 4000000U;
  return c;
}
} // namespace

KB_TEST("spi: bad bus and device configurations are refused") {
  Fresh();
  esp::SpiBusConfig b = BusConfig();
  b.host = 0U; // SPI1 is the flash bus
  esp::SpiBus flash{b};
  KB_CHECK_EQ(flash.EnsureReady(), Status::InvalidArgument);
  b = BusConfig();
  b.host = 3U;
  esp::SpiBus none{b};
  KB_CHECK_EQ(none.EnsureReady(), Status::InvalidArgument);
  b = BusConfig();
  b.mosi_pin = -1;
  b.miso_pin = -1;
  esp::SpiBus no_data{b};
  KB_CHECK_EQ(no_data.EnsureReady(), Status::InvalidArgument);
  esp::SpiBus bus{BusConfig()};
  const auto refused = [&bus](esp::SpiDeviceConfig c) {
    esp::SpiDevice dev{bus, c};
    return dev.EnsureReady() == Status::InvalidArgument;
  };
  esp::SpiDeviceConfig d = DevConfig();
  d.mode = 4U;
  KB_CHECK(refused(d));
  d = DevConfig();
  d.clock_hz = 0U;
  KB_CHECK(refused(d));
  d = DevConfig();
  d.clock_hz = 80000001U;
  KB_CHECK(refused(d));
  d = DevConfig();
  d.timeout = kb::Milliseconds{0U};
  KB_CHECK(refused(d));
  d = DevConfig();
  d.cs_setup_cycles = 17U;
  KB_CHECK(refused(d));
  d = DevConfig();
  d.cs_hold_cycles = 17U;
  KB_CHECK(refused(d));
  d = DevConfig();
  d.cs_pin = 36;
  KB_CHECK(refused(d));
  KB_CHECK_EQ(fake::Calls("spi_bus_initialize"), 0U);
}

KB_TEST("spi: a device brings its bus up, transfers full duplex and releases") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  {
    esp::SpiDevice dev{bus, DevConfig()};
    const std::array<uint8_t, 5> tx{0x80U, 1U, 2U, 3U, 4U};
    std::array<uint8_t, 5> rx{};
    KB_CHECK_EQ(dev.Transfer(tx, rx), Status::NotReady);
    KB_REQUIRE(dev.EnsureReady() == Status::Ok);
    KB_CHECK(bus.IsReady());
    KB_CHECK_EQ(dev.EnsureReady(), Status::Ok);
    KB_CHECK_EQ(fake::Calls("spi_bus_add_device"), 1U);
    KB_CHECK(fake::SpiDma() == SPI_DMA_DISABLED);
    const std::array<uint8_t, 5> reply{0x0FU, 0x11U, 0x22U, 0x33U, 0x44U};
    fake::SpiReply(reply);
    KB_CHECK_EQ(dev.Transfer(tx, rx), Status::Ok);
    KB_CHECK_EQ(rx[4], 0x44U);
    KB_CHECK_EQ(fake::SpiSent()[0], 0x80U);
    KB_CHECK_EQ(fake::SpiTransactions(), 1U);
    KB_CHECK_EQ(fake::SpiKeepCs(), 0U);
    KB_CHECK_EQ(dev.Stats().transfers, 1U);
    KB_CHECK(dev.LastError().IsOk());
    KB_CHECK(dev.Config().clock_hz == 4000000U);
    KB_CHECK_EQ(bus.Release(),
                Status::Ok); // the fake frees it even with a device (real SDK refuses)
  }
  KB_CHECK_EQ(fake::Calls("spi_bus_remove_device"), 1U);
  KB_CHECK(bus.LastError().IsOk());
  KB_CHECK(bus.Config().host == 1U);
}

KB_TEST("spi: spans must be both empty-or-equal and not both empty") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDevice dev{bus, DevConfig()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 2> tx{};
  std::array<uint8_t, 3> rx{};
  KB_CHECK_EQ(dev.Transfer(tx, rx), Status::InvalidArgument);
  KB_CHECK_EQ(dev.Transfer(std::span<const uint8_t>{}, std::span<uint8_t>{}),
              Status::InvalidArgument);
  KB_CHECK_EQ(fake::SpiTransactions(), 0U);
}

KB_TEST("spi: write-only and read-only frames; a read clocks zeros") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDevice dev{bus, DevConfig()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 3> tx{0xAAU, 0xBBU, 0xCCU};
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Ok);
  std::array<uint8_t, 3> rx{0x55U, 0x55U, 0x55U};
  KB_CHECK_EQ(dev.Transfer(std::span<const uint8_t>{}, rx), Status::Ok);
  KB_CHECK_EQ(fake::SpiSent()[3], 0x00U); // zeros clocked out for the read
  KB_CHECK_EQ(fake::SpiSent().size(), 6U);
}

KB_TEST("spi: a frame is one transaction; longer frames are refused, never chunked") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDevice dev{bus, DevConfig()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  std::vector<uint8_t> tx(64U, 0x5AU);
  std::vector<uint8_t> rx(64U, 0U);
  KB_CHECK_EQ(dev.Transfer(tx, rx), Status::Ok);
  KB_CHECK_EQ(fake::SpiTransactions(), 1U);
  std::vector<uint8_t> long_tx(65U, 0x5AU);
  KB_CHECK_EQ(dev.Transfer(long_tx, std::span<uint8_t>{}), Status::InvalidArgument);
  std::vector<uint8_t> long_read(65U, 0U);
  KB_CHECK_EQ(dev.Transfer(std::span<const uint8_t>{}, long_read), Status::InvalidArgument);
  KB_CHECK_EQ(fake::SpiTransactions(), 1U);
  KB_CHECK_EQ(fake::Calls("spi_device_acquire_bus"), 0U); // never: it waits for ever
  KB_CHECK_EQ(fake::SpiKeepCs(), 0U);
}

KB_TEST("spi: with DMA every frame goes through the port's buffer, allocated once") {
  Fresh();
  esp::SpiBusConfig b = BusConfig();
  b.dma = true;
  b.dma_buffer_bytes = 1028U;
  fake::CacheAlign(64U); // internal RAM behind a cache (ESP32-P4): whole lines
  esp::SpiBus bus{b};
  esp::SpiDevice dev{bus, DevConfig()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  KB_CHECK(fake::SpiDma() == SPI_DMA_CH_AUTO);
  KB_CHECK_EQ(fake::HeapBlocks(), 1);
  KB_CHECK_EQ(fake::HeapLastAlign(), 64U);
  KB_CHECK_EQ(bus.MaxFrameBytes(), 1028U);
  // An odd-sized, unaligned caller buffer: the DMA rounds receive lengths up to whole words, so
  // passed directly it would write past the caller's bytes (the fake does as the ESP32 does).
  struct Guarded {
    std::array<uint8_t, 8> raw{};
    std::array<uint8_t, 4> guard{0xA5U, 0xA5U, 0xA5U, 0xA5U};
  } g{};
  const std::array<uint8_t, 5> reply{1U, 2U, 3U, 4U, 5U};
  fake::SpiReply(reply);
  const std::array<uint8_t, 5> tx{9U, 8U, 7U, 6U, 5U};
  const std::span<uint8_t> rx = std::span<uint8_t>{g.raw}.subspan(1U, 5U);
  KB_CHECK_EQ(dev.Transfer(tx, rx), Status::Ok);
  KB_CHECK_EQ(rx[0], 1U);
  KB_CHECK_EQ(rx[4], 5U);
  KB_CHECK_EQ(g.raw[6], 0U); // nothing written after the frame
  KB_CHECK_EQ(g.guard[0], 0xA5U);
  KB_CHECK_EQ(fake::SpiSent()[0], 9U);
  std::vector<uint8_t> read(1028U, 0U);
  KB_CHECK_EQ(dev.Transfer(std::span<const uint8_t>{}, read), Status::Ok); // zeros, one frame
  KB_CHECK_EQ(fake::SpiTransactions(), 2U);
  std::vector<uint8_t> too_long(1029U, 0U);
  KB_CHECK_EQ(dev.Transfer(too_long, std::span<uint8_t>{}), Status::InvalidArgument);
  KB_CHECK_EQ(fake::HeapBlocks(), 1); // the port allocates nothing per transfer
  // On a cached chip ESP-IDF itself bounces a receive whose length is not whole cache lines (the
  // 5-byte and the 1028-byte frames above); a 64-byte frame needs no bounce.
  KB_CHECK_EQ(fake::SdkBounces(), 2U);
  std::array<uint8_t, 64> line_tx{};
  std::array<uint8_t, 64> line_rx{};
  KB_CHECK_EQ(dev.Transfer(line_tx, line_rx), Status::Ok);
  KB_CHECK_EQ(fake::SdkBounces(), 2U);
  fake::Fail("spi_sdk_bounce_alloc", ESP_ERR_NO_MEM); // a transient lack of heap in ESP-IDF
  KB_CHECK_EQ(dev.Transfer(tx, rx), Status::Busy);
  KB_CHECK(FailedWith(dev.LastError(), Status::Busy, ESP_ERR_NO_MEM));
  KB_CHECK(!bus.IsFailed());
  fake::CacheAlign(0U);
  KB_CHECK_EQ(dev.Release(), Status::Ok);
  KB_CHECK_EQ(bus.Release(), Status::Ok);
  KB_CHECK_EQ(fake::HeapBlocks(), 0);
}

KB_TEST("spi: DMA buffer sizes are checked and an allocation failure fails EnsureReady") {
  Fresh();
  esp::SpiBusConfig b = BusConfig();
  b.dma = true;
  b.dma_buffer_bytes = 3U;
  esp::SpiBus small{b};
  KB_CHECK_EQ(small.EnsureReady(), Status::InvalidArgument);
  b.dma_buffer_bytes = 4093U;
  esp::SpiBus large{b};
  KB_CHECK_EQ(large.EnsureReady(), Status::InvalidArgument);
  b.dma_buffer_bytes = 64U;
  esp::SpiBus bus{b};
  fake::Fail("heap_caps_aligned_alloc", ESP_ERR_NO_MEM);
  KB_CHECK_EQ(bus.EnsureReady(), Status::Fault);
  KB_CHECK_EQ(fake::Calls("spi_bus_free"), 1U); // the bus is closed again
  KB_CHECK_EQ(bus.EnsureReady(), Status::Ok);
  KB_CHECK_EQ(bus.LastError().Detail(), 0U);
  static_assert(esp::kSpiMaxFrameBytes == 64U);
  static_assert(esp::kSpiMaxDmaFrameBytes == 4092U);
}

KB_TEST("spi: a frame that could outlast kMaxTimeout is refused; clocks below 1 kHz too") {
  Fresh();
  esp::SpiBusConfig b = BusConfig();
  b.dma = true;
  esp::SpiBus bus{b};
  esp::SpiDeviceConfig d = DevConfig();
  d.clock_hz = 10000U; // 4092 bytes take 3.3 s at 10 kHz
  esp::SpiDevice slow{bus, d};
  KB_REQUIRE(slow.EnsureReady() == Status::Ok);
  std::vector<uint8_t> frame(4092U, 0U);
  KB_CHECK_EQ(slow.Transfer(frame, std::span<uint8_t>{}), Status::InvalidArgument);
  std::vector<uint8_t> shorter(1000U, 0U); // 0.8 s: T + 2 x (W + T) below 2 s
  KB_CHECK_EQ(slow.Transfer(shorter, std::span<uint8_t>{}), Status::Ok);
  KB_CHECK_EQ(slow.SetClockHz(999U), Status::InvalidArgument);
  KB_CHECK_EQ(fake::SpiTransactions(), 1U);
}

KB_TEST("spi: the defaults clock a full DMA frame at 1 MHz (the wait covers the wire time)") {
  Fresh();
  esp::SpiBusConfig b = BusConfig();
  b.dma = true;
  esp::SpiBus bus{b};
  esp::SpiDeviceConfig d{}; // 1 MHz, 20 ms: a 4092-byte frame takes 32.7 ms on the wire
  d.cs_pin = 10;
  esp::SpiDevice dev{bus, d};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(dev.ActualClockHz(), 1000000U);
  alignas(4) static std::array<uint8_t, 4092> frame{};
  KB_CHECK_EQ(dev.Transfer(frame, std::span<uint8_t>{}), Status::Ok);
  KB_CHECK_EQ(dev.Stats().late, 0U);
  KB_CHECK(!fake::SpiInFlight(1));
  KB_CHECK_EQ(fake::SpiWouldBlock(), 0U);
}

KB_TEST("spi: a late transaction is finished, reported Timeout, and the host stays usable") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDevice dev{bus, DevConfig()};
  esp::SpiDeviceConfig o = DevConfig();
  o.cs_pin = 9;
  esp::SpiDevice other{bus, o};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  KB_REQUIRE(other.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 2> tx{1U, 2U};
  fake::SpiSlow(6000U); // the hardware takes far longer than wire time plus timeout, once
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Timeout);
  KB_CHECK(FailedWith(dev.LastError(), Status::Timeout, ESP_ERR_TIMEOUT));
  KB_CHECK_EQ(dev.Stats().late, 1U);
  KB_CHECK_EQ(dev.Stats().failures, 1U);
  KB_CHECK(!fake::SpiInFlight(1)); // ended: ESP-IDF gave its bus lock back
  KB_CHECK_EQ(fake::Calls("spi_device_polling_end"), 2U);
  fake::SpiSlow(1U);
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Ok);
  KB_CHECK_EQ(other.Transfer(tx, std::span<uint8_t>{}), Status::Ok);
  KB_CHECK_EQ(fake::SpiWouldBlock(), 0U);
  KB_CHECK_EQ(dev.Release(), Status::Ok);
  KB_CHECK_EQ(other.Release(), Status::Ok);
}

KB_TEST("spi: a transaction that never ends marks the host failed; nothing waits for ever") {
  Fresh();
  esp::SpiBusConfig b = BusConfig();
  b.host = 2U; // its own host and chip select: the device cannot be removed afterwards
  b.sclk_pin = 40;
  b.mosi_pin = 41;
  b.miso_pin = 42;
  esp::SpiBus bus{b};
  esp::SpiDeviceConfig d = DevConfig();
  d.cs_pin = 43;
  esp::SpiDevice dev{bus, d};
  esp::SpiDeviceConfig o = DevConfig();
  o.cs_pin = 44;
  esp::SpiDevice other{bus, o};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  KB_REQUIRE(other.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 2> tx{1U, 2U};
  fake::SpiStuck(true);
  const uint64_t before = fake::TimeUs();
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Fault);
  // T + 2 x (W + T): two waits of 1 ms (rounded up) + 20 ms, each rounded up to ticks plus one.
  KB_CHECK(fake::TimeUs() - before <= 50000U);
  KB_CHECK(bus.IsFailed());
  const uint32_t starts = fake::Calls("spi_device_polling_start");
  KB_CHECK_EQ(other.Transfer(tx, std::span<uint8_t>{}), Status::Fault);
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Fault);
  KB_CHECK_EQ(other.SetClockHz(2000000U), Status::Fault);
  KB_CHECK_EQ(fake::Calls("spi_device_polling_start"), starts); // the SDK is not touched again
  KB_CHECK_EQ(fake::SpiWouldBlock(), 0U);
  KB_CHECK_EQ(dev.Release(), Status::NotReady); // ESP-IDF refuses: transaction in flight
  KB_CHECK_EQ(other.Release(), Status::Ok);
  KB_CHECK_EQ(fake::Calls("spi_device_acquire_bus"), 0U);
  fake::SpiStuck(false);
}

KB_TEST("spi: start and end failures, and the old finite-timeout start, are reported") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDevice dev{bus, DevConfig()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 2> tx{1U, 2U};
  fake::Fail("spi_device_polling_start", ESP_ERR_NO_MEM); // transient: no heap for a bounce
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Busy);
  fake::Fail("spi_device_polling_start", ESP_FAIL);
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Fault);
  fake::Fail("spi_device_polling_end", ESP_ERR_INVALID_STATE); // a DMA error at the end
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::NotReady);
  fake::Fail("spi_device_polling_end", ESP_ERR_TIMEOUT); // the first wait only
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Timeout);
  KB_CHECK(!fake::SpiInFlight(1));
  KB_CHECK_EQ(dev.Stats().failures, 4U);
}

KB_TEST("spi: SetClockHz re-adds the device; before EnsureReady it is kept") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDevice dev{bus, DevConfig()};
  KB_CHECK_EQ(dev.SetClockHz(0U), Status::InvalidArgument);
  KB_CHECK_EQ(dev.SetClockHz(90000000U), Status::InvalidArgument);
  KB_CHECK_EQ(dev.SetClockHz(2000000U), Status::Ok);
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::SpiClockHz(), 2000000);
  KB_CHECK_EQ(dev.SetClockHz(8000000U), Status::Ok);
  KB_CHECK_EQ(fake::SpiClockHz(), 8000000);
  KB_CHECK(dev.Config().clock_hz == 8000000U);
  fake::Fail("spi_bus_remove_device", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(dev.SetClockHz(1000000U), Status::NotReady);
  KB_CHECK(dev.Config().clock_hz == 8000000U);
  fake::Fail("spi_bus_add_device", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(dev.SetClockHz(1000000U), Status::InvalidArgument); // added back at the old clock
  KB_CHECK(dev.Config().clock_hz == 8000000U);
  KB_CHECK_EQ(fake::SpiClockHz(), 8000000);
  fake::Fail("spi_bus_add_device", ESP_ERR_INVALID_ARG, 2U);
  KB_CHECK_EQ(dev.SetClockHz(1000000U), Status::Fault); // lost
  const std::array<uint8_t, 1> tx{1U};
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::NotReady);
}

KB_TEST("spi: device slots and chip-select pins are claimed") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDevice dev{bus, DevConfig()};
  esp::SpiDevice twin{bus, DevConfig()}; // the same chip select
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(twin.EnsureReady(), Status::Busy);
  esp::OutputConfig pin{};
  pin.pin = 12; // the bus's SCLK
  esp::OutputPin sclk{pin};
  KB_CHECK_EQ(sclk.EnsureReady(), Status::Busy);
  KB_CHECK_EQ(dev.Release(), Status::Ok);
  KB_CHECK_EQ(twin.EnsureReady(), Status::Ok);
}

KB_TEST("spi: bus failures reach the device") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDevice dev{bus, DevConfig()};
  fake::Fail("spi_bus_initialize", ESP_ERR_INVALID_ARG);
  KB_CHECK_EQ(dev.EnsureReady(), Status::InvalidArgument);
  KB_CHECK(FailedWith(bus.LastError(), Status::InvalidArgument, ESP_ERR_INVALID_ARG));
  KB_CHECK(FailedWith(dev.LastError(), Status::InvalidArgument, ESP_ERR_INVALID_STATE));
  fake::Fail("spi_bus_add_device", ESP_ERR_NOT_FOUND);
  KB_CHECK_EQ(dev.EnsureReady(), Status::NoDevice);
  KB_CHECK_EQ(dev.Release(), Status::Ok); // nothing added
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  fake::Fail("spi_bus_free", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(bus.Release(), Status::NotReady);
  KB_CHECK(bus.IsReady());
}

KB_TEST("spi: a held bus makes every device call Busy") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDevice dev{bus, DevConfig()};
  esp::SpiDevice other{bus, DevConfig()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  KB_REQUIRE(bus.TryLockFor(kb::Milliseconds{1U}));
  const std::array<uint8_t, 1> tx{1U};
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Busy);
  KB_CHECK_EQ(dev.SetClockHz(1000000U), Status::Busy);
  KB_CHECK_EQ(other.EnsureReady(), Status::Busy);
  KB_CHECK_EQ(dev.Release(), Status::Busy);
  KB_CHECK_EQ(dev.Stats().lock_timeouts, 3U);
  bus.Unlock();
  KB_CHECK_EQ(dev.Release(), Status::Ok);
  fake::Fail("spi_bus_remove_device", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(other.EnsureReady(), Status::Ok);
  KB_CHECK_EQ(other.Release(), Status::NotReady);
  KB_CHECK_EQ(other.Release(), Status::Ok);
}

KB_TEST("spi: the DMA buffer outlives a refused spi_bus_free; reads clock zeros from it") {
  Fresh();
  esp::SpiBusConfig b = BusConfig();
  b.dma = true;
  b.dma_buffer_bytes = 16U;
  esp::SpiBus bus{b};
  esp::SpiDevice dev{bus, DevConfig()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 4> cmd{0xAAU, 0xBBU, 0xCCU, 0xDDU};
  KB_REQUIRE(dev.Transfer(cmd, std::span<uint8_t>{}) == Status::Ok);
  std::array<uint8_t, 4> rx{};
  KB_REQUIRE(dev.Transfer(std::span<const uint8_t>{}, rx) == Status::Ok);
  KB_CHECK_EQ(fake::SpiSent()[4], 0x00U); // not the previous command still in the buffer
  KB_CHECK_EQ(fake::SpiSent()[7], 0x00U);
  fake::Fail("spi_bus_free", ESP_ERR_INVALID_STATE); // a device still attached
  KB_CHECK_EQ(bus.Release(), Status::NotReady);
  KB_CHECK_EQ(fake::HeapBlocks(), 1); // kept: a transaction may still use it
  KB_REQUIRE(dev.Release() == Status::Ok);
  KB_CHECK_EQ(bus.Release(), Status::Ok);
  KB_CHECK_EQ(fake::HeapBlocks(), 0);
}

KB_TEST("spi: after a failed DMA host nothing is copied out to the caller") {
  Fresh();
  esp::SpiBusConfig b = BusConfig();
  b.host = 2U;
  b.sclk_pin = 45;
  b.mosi_pin = 46;
  b.miso_pin = 47;
  b.dma = true;
  b.dma_buffer_bytes = 16U;
  esp::SpiBus bus{b};
  esp::SpiDeviceConfig d = DevConfig();
  d.cs_pin = 48;
  esp::SpiDevice dev{bus, d};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 4> reply{1U, 2U, 3U, 4U};
  fake::SpiReply(reply);
  const std::array<uint8_t, 4> tx{9U, 9U, 9U, 9U};
  std::array<uint8_t, 4> rx{7U, 7U, 7U, 7U};
  fake::SpiStuck(true);
  KB_CHECK_EQ(dev.Transfer(tx, rx), Status::Fault);
  KB_CHECK(rx == (std::array<uint8_t, 4>{})); // zeros, not the port buffer's contents
  fake::SpiStuck(false);
}

KB_TEST("spi: a clock ESP-IDF reports as 0 kHz is refused and the device removed") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDevice dev{bus, DevConfig()};
  fake::SpiReportKhz(0);
  KB_CHECK_EQ(dev.EnsureReady(), Status::InvalidArgument);
  KB_CHECK_EQ(fake::Calls("spi_bus_remove_device"), 1U);
  fake::SpiReportKhz(-1);
  KB_CHECK_EQ(dev.EnsureReady(), Status::Ok);
}

KB_TEST("spi: a timeout no frame could meet within kMaxTimeout is refused at EnsureReady") {
  Fresh();
  esp::SpiBus bus{BusConfig()};
  esp::SpiDeviceConfig d = DevConfig();
  d.timeout = kb::Milliseconds{1000U}; // 3 x T alone is above 2 s
  esp::SpiDevice dev{bus, d};
  KB_CHECK_EQ(dev.EnsureReady(), Status::InvalidArgument);
  d.timeout = kb::Milliseconds{600U};
  esp::SpiDevice ok{bus, d};
  KB_REQUIRE(ok.EnsureReady() == Status::Ok);
  const std::array<uint8_t, 1> tx{1U};
  KB_CHECK_EQ(ok.Transfer(tx, std::span<uint8_t>{}), Status::Ok);
  KB_CHECK_EQ(ok.SetClockHz(1000U), Status::Ok); // 8 ms on the wire still fits at 600 ms
}
