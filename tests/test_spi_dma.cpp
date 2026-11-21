// SpiDmaChannel over the ESP-IDF fakes (queued transactions, manual alignment): zero-copy transfers
// of kb::dma::Buffer leases, the exactly-once report, the deadline (wait once more, then Timeout or
// quarantine), Cancel only with a confirmed end, the DMA error flags, Recover, the admission rules
// and the cache hooks of a Maintained region.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/dma/buffer.hpp"
#include "kb/port/esp32/spi_dma.hpp"
#include "support.hpp"

namespace {
using esp::SpiDmaState;
using kb::dma::BufferState;
using kb::dma::Direction;

kb::dma::Buffer<esp::InternalRam, 64U> g_buf{};
kb::dma::Buffer<esp::InternalRam, 64U> g_other{};
kb::dma::Buffer<esp::InternalRamL1, 100U> g_l1{};
esp::SpiStaging<128U> g_staging{};

esp::SpiBusConfig Bus(bool dma) {
  esp::SpiBusConfig c{};
  c.host = 1U;
  c.mosi_pin = 11;
  c.miso_pin = 13;
  c.sclk_pin = 12;
  c.dma = dma;
  c.staging = g_staging.Ref();
  return c;
}
esp::SpiDeviceConfig Dev() {
  esp::SpiDeviceConfig c{};
  c.cs_pin = 10;
  c.clock_hz = 1000000U;
  return c;
}
kb::utils::Deadline Far() {
  return kb::utils::Deadline::After(kb::Instant{fake::TimeUs()}, kb::Milliseconds{1000U});
}
} // namespace

KB_TEST("spi dma: a zero-copy transmit and receive, reported once, the lease back to the CPU") {
  Fresh();
  esp::SpiBus bus{Bus(true)};
  esp::SpiDevice dev{bus, Dev()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  esp::SpiDmaChannel<esp::InternalRam> ch{dev};
  std::span<std::byte> view = g_buf.CpuView();
  for (std::size_t i = 0U; i < view.size(); ++i) {
    view[i] = static_cast<std::byte>(i);
  }
  KB_REQUIRE(ch.Start(g_buf, Direction::ToDevice, 10U, Far()) == Status::Ok);
  KB_CHECK(g_buf.Current() == BufferState::Device);
  KB_CHECK(g_buf.CpuView().empty()); // no CPU view while leased
  KB_CHECK((fake::SpiLastFlags() & SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL) != 0U);
  KB_CHECK_EQ(ch.Start(g_other, Direction::ToDevice, 4U, Far()), Status::Busy); // one at a time
  KB_CHECK(ch.Poll(kb::Instant{fake::TimeUs()}).state == SpiDmaState::Pending);
  fake::SpiElapse(1000U);
  const esp::SpiDmaResult done = ch.Poll(kb::Instant{fake::TimeUs()});
  KB_CHECK(done.state == SpiDmaState::Done);
  KB_CHECK_EQ(done.status, Status::Ok);
  KB_CHECK_EQ(done.count, 10U);
  KB_CHECK(ch.Poll(kb::Instant{fake::TimeUs()}).state == SpiDmaState::Idle); // reported once
  KB_CHECK(g_buf.Current() == BufferState::Cpu);
  KB_CHECK_EQ(fake::SpiSent().size(), 10U);
  KB_CHECK_EQ(fake::SpiSent()[9], 9U);
  // The fake answers byte k of the whole session with reply[k]: 10 bytes went out already.
  std::array<uint8_t, 16> reply{};
  reply[15] = 0x66U;
  fake::SpiReply(reply);
  KB_REQUIRE(ch.Start(g_buf, Direction::FromDevice, 6U, Far()) == Status::Ok);
  fake::SpiElapse(1000U);
  KB_CHECK_EQ(ch.Poll(kb::Instant{fake::TimeUs()}).status, Status::Ok);
  KB_CHECK(g_buf.CpuView()[5] == std::byte{0x66U});
  const esp::SpiDmaStats st = ch.Stats();
  KB_CHECK_EQ(st.starts, 2U);
  KB_CHECK_EQ(st.completions, 2U);
  KB_CHECK_EQ(fake::HeapBlocks(), 0);
  KB_CHECK(ch.Capabilities().zero_copy);
  KB_CHECK(!ch.Health().faulted);
}

KB_TEST("spi dma: a cancel returns Ok only once ESP-IDF reported the end") {
  Fresh();
  esp::SpiBus bus{Bus(true)};
  esp::SpiDevice dev{bus, Dev()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  esp::SpiDmaChannel<esp::InternalRam> ch{dev};
  KB_CHECK_EQ(ch.Cancel(), Status::Ok); // nothing out
  KB_REQUIRE(ch.Start(g_buf, Direction::FromDevice, 32U, Far()) == Status::Ok);
  KB_CHECK_EQ(ch.Cancel(), Status::Ok); // waited for the end (W + T)
  KB_CHECK(g_buf.Current() == BufferState::Cpu);
  KB_CHECK(ch.Poll(kb::Instant{fake::TimeUs()}).state == SpiDmaState::Idle);
  KB_CHECK_EQ(ch.Stats().cancels, 1U);
  // A transaction that never ends: the lease ends quarantined, the host failed.
  KB_REQUIRE(ch.Start(g_buf, Direction::FromDevice, 32U, Far()) == Status::Ok);
  fake::SpiStuck(true);
  KB_CHECK_EQ(ch.Cancel(), Status::Fault);
  KB_CHECK(g_buf.Current() == BufferState::Quarantined);
  KB_CHECK(ch.Health().faulted);
  KB_CHECK(bus.IsFailed());
  KB_CHECK_EQ(ch.Stats().quarantines, 1U);
  KB_CHECK_EQ(ch.Start(g_other, Direction::ToDevice, 4U, Far()), Status::Fault);
  const std::array<uint8_t, 2> tx{1U, 2U};
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Fault);
  // ESP-IDF v5.5 cannot stop it: Recover stays Fault until the SDK reports the end.
  KB_CHECK_EQ(ch.Recover(g_buf), Status::Fault);
  KB_CHECK(g_buf.Current() == BufferState::Quarantined);
  fake::SpiStuck(false);
  fake::SpiElapse(10000U);
  KB_CHECK_EQ(ch.Recover(g_buf), Status::Ok); // the end is reported now: proven
  KB_CHECK(g_buf.Current() == BufferState::Cpu);
  KB_CHECK(!ch.Health().faulted);
  KB_CHECK(!bus.IsFailed());
  KB_CHECK_EQ(ch.Recover(g_buf), Status::InvalidArgument); // not quarantined
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Ok);
}

KB_TEST("spi dma: an overdue transfer waits once more, then reports Timeout or quarantines") {
  Fresh();
  esp::SpiBus bus{Bus(true)};
  esp::SpiDevice dev{bus, Dev()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  esp::SpiDmaChannel<esp::InternalRam> ch{dev};
  const kb::Instant t0{fake::TimeUs()};
  KB_REQUIRE(ch.Start(g_buf, Direction::ToDevice, 16U,
                      kb::utils::Deadline::After(t0, kb::Milliseconds{1U})) == Status::Ok);
  const esp::SpiDmaResult late = ch.Poll(kb::Instant{t0.Us() + 5000U});
  KB_CHECK(late.state == SpiDmaState::Done);
  KB_CHECK_EQ(late.status, Status::Timeout); // complete, but after the deadline
  KB_CHECK(g_buf.Current() == BufferState::Cpu);
  KB_CHECK_EQ(ch.Stats().timeouts, 1U);
  fake::SpiStuck(true);
  const kb::Instant t1{fake::TimeUs()};
  KB_REQUIRE(ch.Start(g_buf, Direction::FromDevice, 16U,
                      kb::utils::Deadline::After(t1, kb::Milliseconds{1U})) == Status::Ok);
  const esp::SpiDmaResult lost = ch.Poll(kb::Instant{t1.Us() + 5000U});
  KB_CHECK_EQ(lost.status, Status::Timeout);
  KB_CHECK(g_buf.Current() == BufferState::Quarantined); // the DMA may still write it
  KB_CHECK(bus.IsFailed());
  KB_CHECK_EQ(bus.Health().quarantined, 1U);
  fake::SpiStuck(false);
  fake::SpiElapse(10000U);
  KB_CHECK_EQ(ch.Recover(g_buf), Status::Ok); // the end came: the host may be released
}

KB_TEST("spi dma: a DMA overflow is a Fault with its flags, counted, the buffer settled") {
  Fresh();
  esp::SpiBus bus{Bus(true)};
  esp::SpiDevice dev{bus, Dev()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  esp::SpiDmaChannel<esp::InternalRam> ch{dev};
  fake::SpiDmaLost(true);
  KB_REQUIRE(ch.Start(g_buf, Direction::FromDevice, 8U, Far()) == Status::Ok);
  fake::SpiElapse(1000U);
  const esp::SpiDmaResult r = ch.Poll(kb::Instant{fake::TimeUs()});
  KB_CHECK_EQ(r.status, Status::Fault);
  KB_CHECK((r.flags & esp::spi_dma_flags::kTransferError) != 0U);
  KB_CHECK_EQ(ch.Stats().transfer_errors, 1U);
  KB_CHECK_EQ(ch.Stats().fifo_errors, 1U);
  KB_CHECK_EQ(bus.Health().errors, 1U);
  KB_CHECK(g_buf.Current() == BufferState::Cpu);
  KB_CHECK(!bus.IsFailed()); // the transaction ended: the host is fine
}

KB_TEST("spi dma: admission: ceiling, no DMA, not ready, cached receive lines, SDK refusal") {
  Fresh();
  esp::SpiBus nodma{Bus(false)};
  esp::SpiDevice fifo{nodma, Dev()};
  esp::SpiDmaChannel<esp::InternalRam> off{fifo};
  KB_CHECK_EQ(off.Start(g_buf, Direction::ToDevice, 4U, Far()), Status::NotReady);
  KB_REQUIRE(fifo.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(off.Start(g_buf, Direction::ToDevice, 65U, Far()), Status::OutOfRange);
  KB_CHECK_EQ(off.Start(g_buf, Direction::ToDevice, 4U, Far()), Status::Unsupported);
  KB_CHECK(off.Capabilities().mode == kb::dma::TransferMode::Polled);
  KB_CHECK_EQ(off.Capabilities().max_frame_bytes, 64U);
  KB_CHECK_EQ(fifo.Release(), Status::Ok);
  KB_CHECK_EQ(nodma.Release(), Status::Ok);

  Fresh();
  esp::SpiBus bus{Bus(true)};
  esp::SpiDevice dev{bus, Dev()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  esp::SpiDmaChannel<esp::InternalRam> ch{dev};
  KB_CHECK_EQ(ch.Start(g_buf, Direction::ToDevice, 0U, Far()), Status::OutOfRange);
  KB_CHECK_EQ(ch.Start(g_buf, Direction::ToDevice, 65U, Far()), Status::InvalidArgument); // > Bytes
  fake::Fail("spi_device_queue_trans", ESP_ERR_INVALID_ARG); // manual alignment refused
  KB_CHECK_EQ(ch.Start(g_buf, Direction::ToDevice, 8U, Far()), Status::InvalidArgument);
  KB_CHECK(g_buf.Current() == BufferState::Cpu); // never lent to the hardware
  fake::Fail("spi_device_queue_trans", ESP_ERR_NO_MEM);
  KB_CHECK_EQ(ch.Start(g_buf, Direction::ToDevice, 8U, Far()), Status::Fault);
  fake::CacheAlign(64U); // ESP32-P4: a DMA receive in whole lines only
  KB_CHECK_EQ(ch.Start(g_buf, Direction::FromDevice, 8U, Far()), Status::InvalidArgument);
  KB_CHECK_EQ(ch.Start(g_buf, Direction::FromDevice, 64U, Far()), Status::Ok);
  fake::CacheAlign(0U);
  KB_CHECK_EQ(ch.Cancel(), Status::Ok);
  // The bus lock is held by a frame of another task: Busy, nothing lent.
  KB_REQUIRE(bus.TryLockFor(kb::Milliseconds{1U}));
  KB_CHECK_EQ(ch.Start(g_buf, Direction::ToDevice, 8U, Far()), Status::Busy);
  bus.Unlock();
  KB_CHECK_EQ(dev.Stats().lock_timeouts, 1U);
}

KB_TEST("spi dma: a Maintained region is cleaned and invalidated at the lease transitions") {
  Fresh();
  fake::CacheAlign(64U);
  esp::SpiBus bus{Bus(true)};
  esp::SpiDevice dev{bus, Dev()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  {
    esp::SpiDmaChannel<esp::InternalRamL1> ch{dev};
    KB_REQUIRE(ch.Start(g_l1, Direction::ToDevice, 40U, Far()) == Status::Ok);
    KB_CHECK_EQ(fake::CacheSyncs(), 1U); // clean of the used lines
    fake::SpiElapse(1000U);
    KB_CHECK_EQ(ch.Poll(kb::Instant{fake::TimeUs()}).status, Status::Ok);
    KB_REQUIRE(ch.Start(g_l1, Direction::FromDevice, 64U, Far()) == Status::Ok);
    KB_CHECK_EQ(fake::CacheSyncs(), 3U); // clean and invalidate of the whole capacity
    fake::SpiElapse(1000U);
    KB_CHECK_EQ(ch.Poll(kb::Instant{fake::TimeUs()}).status, Status::Ok);
    KB_CHECK_EQ(fake::CacheSyncs(), 4U); // invalidate again before the CPU reads
    KB_CHECK_EQ(ch.CacheFailures(), 0U);
    KB_REQUIRE(ch.Start(g_l1, Direction::FromDevice, 64U, Far()) == Status::Ok);
  } // the channel's destructor cancels the transfer still out
  KB_CHECK(g_l1.Current() == BufferState::Cpu);
  fake::CacheAlign(0U);
}
