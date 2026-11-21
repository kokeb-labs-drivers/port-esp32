// The port's DMA vocabulary (dma.hpp) and the DMA set-up of an SPI host over the ESP-IDF fakes: the
// per-target limits, the interrupt tiers (B32), the static staging buffer and its start-time check,
// the reported fallback when no DMA channel is free, the cache hooks, and the staging held after a
// transaction that never ended.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/dma/assignment.hpp"
#include "kb/dma/capabilities.hpp"
#include "support.hpp"

namespace {
using kb::dma::TransferMode;
using T = esp::Target;

// Compile-time: a driver frame against the bus, the interrupt tier against ESP-IDF's limits.
using Spi2C6 = esp::SpiLimits<T::Esp32C6, true, 1028U>;
using Spi2NoDma = esp::SpiLimits<T::Esp32S3, false>;
constexpr bool kBno08xFits = (kb::dma::RequireBusCarries<Spi2C6, 1028U>(), true);
constexpr bool kTmc5160Fits = (kb::dma::RequireBusCarries<Spi2NoDma, 5U>(), true);
static_assert(kBno08xFits && kTmc5160Fits);
struct SpiTask { // a stream that notifies a task: level 3 = kb priority 4, the RTOS limit
  static constexpr kb::dma::StreamAssignment kAssignment{.engine = 0U,
                                                         .stream = 0U,
                                                         .request = 0U,
                                                         .priority = kb::dma::ChannelPriority::High,
                                                         .irq_priority = 4U,
                                                         .notify = kb::dma::Notify::Task,
                                                         .irq = kb::dma::IrqUse::All,
                                                         .mode = kb::dma::ChannelMode::Normal};
};
constexpr bool kTierOk = (kb::dma::RequireInterruptTier<SpiTask, esp::EspRtosLimits>(), true);
static_assert(kTierOk);
constexpr bool kClassicFullDuplex =
    (esp::RequireSpiHalfDuplexPhases<T::Esp32, true, true, false>(), true);
static_assert(kClassicFullDuplex);

esp::SpiBusConfig DmaBus() {
  esp::SpiBusConfig c{};
  c.host = 1U;
  c.mosi_pin = 11;
  c.miso_pin = 13;
  c.sclk_pin = 12;
  c.dma = true;
  return c;
}
esp::SpiDeviceConfig Dev() {
  esp::SpiDeviceConfig c{};
  c.cs_pin = 10;
  c.clock_hz = 4000000U;
  return c;
}

// Static staging buffers are namespace-scope objects (.bss: internal RAM on target).
esp::SpiStaging<100U> g_staging{};
esp::SpiStaging<5U> g_tiny{};
} // namespace

KB_TEST("dma: SPI limits per target as kb::dma::BusLimits") {
  constexpr kb::dma::BusLimits c6 = esp::SpiBusLimits(T::Esp32C6, true, 1028U);
  static_assert(c6.dma && (c6.MaxFrame() == 1028U) && (c6.rx_length_multiple == 4U));
  constexpr kb::dma::BusLimits p4 = esp::SpiBusLimits(T::Esp32P4, true, 9000U);
  static_assert((p4.MaxFrame() == esp::kSpiMaxDmaFrameBytes) && (p4.rx_length_multiple == 64U) &&
                (p4.align_bytes == 64U));
  constexpr kb::dma::BusLimits none = esp::SpiBusLimits(T::Esp32, false, 4092U);
  static_assert(!none.dma && (none.MaxFrame() == esp::kSpiMaxFrameBytes));
  KB_CHECK_EQ(esp::SpiDmaUnitBytes(T::Esp32S3), 4U);
  KB_CHECK_EQ(Spi2C6::kBus.max_frame_dma, 1028U);
  KB_CHECK_EQ(esp::SpiStaging<100U>::kHalf, 128U);
  const esp::SpiStagingRef ref = g_staging.Ref();
  KB_CHECK_EQ(ref.half, 128U);
  KB_CHECK_EQ(ref.frame, 100U);
  KB_CHECK((kb::dma::AddressOf(reinterpret_cast<const std::byte*>(ref.base)) % 64U) == 0U);
}

KB_TEST("dma: interrupt levels map kb priorities onto ESP-IDF levels 1 to 3 (B32)") {
  KB_CHECK_EQ(esp::InterruptLevelOf(4U), 3U);
  KB_CHECK_EQ(esp::InterruptLevelOf(6U), 1U);
  KB_CHECK_EQ(esp::InterruptLevelOf(0U), 0U);
  KB_CHECK_EQ(esp::InterruptLevelOf(8U), 0U);
  KB_CHECK(esp::InterruptLevelUsable(0U));
  KB_CHECK(esp::InterruptLevelUsable(3U));
  KB_CHECK(!esp::InterruptLevelUsable(4U));
  KB_CHECK(kb::dma::InterruptTierAllowed(4U, kb::dma::Notify::Task, esp::kInterruptLimits));
  KB_CHECK(!kb::dma::InterruptTierAllowed(3U, kb::dma::Notify::Task, esp::kInterruptLimits));
  // The board's level is what ESP-IDF is asked for, exactly (ESP-IDF allocates that level or
  // fails); a level a C handler cannot run at is refused before the SDK.
  Fresh();
  esp::SpiBusConfig b = DmaBus();
  b.interrupt_level = esp::InterruptLevelOf(SpiTask::kAssignment.irq_priority);
  esp::SpiBus bus{b};
  KB_REQUIRE(bus.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::SpiIntrFlags(), ESP_INTR_FLAG_LEVEL3);
  KB_CHECK_EQ(bus.Release(), Status::Ok);
  for (const uint8_t level : {uint8_t{1U}, uint8_t{2U}}) {
    Fresh();
    b.interrupt_level = level;
    esp::SpiBus other{b};
    KB_REQUIRE(other.EnsureReady() == Status::Ok);
    KB_CHECK_EQ(fake::SpiIntrFlags(), (level == 1U) ? ESP_INTR_FLAG_LEVEL1 : ESP_INTR_FLAG_LEVEL2);
    KB_CHECK_EQ(other.Release(), Status::Ok);
  }
  Fresh();
  b.interrupt_level = 0U;
  esp::SpiBus deflt{b};
  KB_REQUIRE(deflt.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::SpiIntrFlags(), 0);
  KB_CHECK_EQ(deflt.Release(), Status::Ok);
  Fresh();
  b.interrupt_level = 4U;
  esp::SpiBus high{b};
  KB_CHECK_EQ(high.EnsureReady(), Status::InvalidArgument);
  KB_CHECK_EQ(fake::Calls("spi_bus_initialize"), 0U);
}

KB_TEST("dma: a static staging buffer is checked at start and used for every DMA frame") {
  Fresh();
  esp::SpiBusConfig b = DmaBus();
  b.staging = g_staging.Ref();
  esp::SpiBus bus{b};
  esp::SpiDevice dev{bus, Dev()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(fake::HeapBlocks(), 0); // nothing allocated at all
  KB_CHECK_EQ(bus.MaxFrameBytes(), 100U);
  const kb::dma::Capabilities caps = bus.Capabilities();
  KB_CHECK(caps.mode == TransferMode::Dma);
  KB_CHECK(caps.zero_copy && caps.confirmed_abort);
  KB_CHECK_EQ(caps.max_frame_bytes, 100U);
  KB_CHECK_EQ(caps.staging_bytes, 100U);
  const std::array<uint8_t, 3> reply{7U, 8U, 9U};
  fake::SpiReply(reply);
  std::array<uint8_t, 4> raw{0U, 0U, 0U, 0xC3U};
  const std::span<uint8_t> rx = std::span<uint8_t>{raw}.first(3U);
  const std::array<uint8_t, 3> tx{1U, 2U, 3U};
  KB_CHECK_EQ(dev.Transfer(tx, rx), Status::Ok);
  KB_CHECK_EQ(raw[2], 9U);
  KB_CHECK_EQ(raw[3], 0xC3U); // the word rounding stayed in the staging buffer
  std::array<uint8_t, 101> big{};
  KB_CHECK_EQ(dev.Transfer(big, std::span<uint8_t>{}), Status::InvalidArgument);
  KB_CHECK_EQ(dev.Release(), Status::Ok);
  KB_CHECK_EQ(bus.Release(), Status::Ok);
  KB_CHECK(bus.Capabilities().mode == TransferMode::Polled); // down
}

KB_TEST("dma: a staging buffer that is not DMA-capable or badly shaped is refused") {
  Fresh();
  fake::DmaCapable(false); // for example a buffer in PSRAM or flash
  esp::SpiBusConfig b = DmaBus();
  b.staging = g_staging.Ref();
  esp::SpiBus bus{b};
  KB_CHECK_EQ(bus.EnsureReady(), Status::InvalidArgument);
  KB_CHECK_EQ(fake::Calls("spi_bus_free"), 1U); // the bus is closed again
  KB_CHECK(!bus.IsReady());
  fake::DmaCapable(true);
  esp::SpiStagingRef odd = g_staging.Ref();
  odd.half = 100U; // not whole 64-byte units
  b.staging = odd;
  esp::SpiBus shaped{b};
  KB_CHECK_EQ(shaped.EnsureReady(), Status::InvalidArgument);
  esp::SpiStagingRef over = g_tiny.Ref();
  over.frame = 65U; // more than the half holds
  b.staging = over;
  esp::SpiBus overflow{b};
  KB_CHECK_EQ(overflow.EnsureReady(), Status::InvalidArgument);
  KB_CHECK_EQ(fake::Calls("spi_bus_initialize"), 1U);
}

KB_TEST("dma: no free DMA channel fails, or with the fallback runs without DMA and says so") {
  Fresh();
  fake::SpiDmaExhausted(true); // the ESP32-C6 has three GDMA pairs
  esp::SpiBus strict{DmaBus()};
  KB_CHECK_EQ(strict.EnsureReady(), Status::Unsupported);
  KB_CHECK(FailedWith(strict.LastError(), Status::Unsupported, ESP_ERR_NOT_FOUND));
  esp::SpiBusConfig b = DmaBus();
  b.dma_fallback = true;
  esp::SpiBus bus{b};
  esp::SpiDevice dev{bus, Dev()};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  KB_CHECK(FailedWith(bus.LastError(), Status::Unsupported, ESP_ERR_NOT_FOUND));
  KB_CHECK(!bus.DmaActive());
  KB_CHECK(fake::SpiDma() == SPI_DMA_DISABLED);
  const kb::dma::Capabilities caps = bus.Capabilities();
  KB_CHECK(caps.mode == TransferMode::Polled);
  KB_CHECK(!caps.zero_copy);
  KB_CHECK_EQ(caps.max_frame_bytes, esp::kSpiMaxFrameBytes);
  KB_CHECK_EQ(caps.staging_bytes, 0U);
  KB_CHECK(bus.Health().mode == TransferMode::Polled);
  KB_CHECK_EQ(fake::HeapBlocks(), 0);
  const std::array<uint8_t, 2> tx{1U, 2U};
  KB_CHECK_EQ(dev.Transfer(tx, std::span<uint8_t>{}), Status::Ok);
  KB_CHECK_EQ(fake::SpiLastFlags() & SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL, 0U); // FIFO: no flag
  std::array<uint8_t, 65> big{};
  KB_CHECK_EQ(dev.Transfer(big, std::span<uint8_t>{}), Status::InvalidArgument);
  KB_CHECK_EQ(dev.Release(), Status::Ok);
  KB_CHECK_EQ(bus.Release(), Status::Ok);
  // Another error of spi_bus_initialize is not a fallback.
  Fresh();
  fake::Fail("spi_bus_initialize", ESP_ERR_INVALID_ARG);
  esp::SpiBus refused{b};
  KB_CHECK_EQ(refused.EnsureReady(), Status::InvalidArgument);
}

KB_TEST("dma: the cache hooks sync whole lines through esp_cache_msync and count refusals") {
  Fresh();
  fake::CacheAlign(64U);
  alignas(64) std::array<std::byte, 128> lines{};
  esp::EspCacheOps ops{};
  ops.Clean(std::span<const std::byte>(lines));
  KB_CHECK_EQ(fake::CacheLastFlags(), ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  ops.Invalidate(std::span<std::byte>(lines));
  KB_CHECK_EQ(fake::CacheLastFlags(), ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  ops.Barrier();
  ops.Clean(std::span<const std::byte>{}); // nothing to do: no call
  KB_CHECK_EQ(fake::CacheSyncs(), 2U);
  KB_CHECK_EQ(ops.Failures(), 0U);
  ops.Invalidate(std::span<std::byte>(lines).first(5U)); // not whole lines: ESP-IDF refuses
  ops.Clean(std::span<const std::byte>(lines).first(5U));
  KB_CHECK_EQ(ops.Failures(), 2U);
  KB_CHECK(esp::DmaMemoryUsable(std::span<const uint8_t>(g_staging.Ref().base, 128U)));
  KB_CHECK(!esp::DmaMemoryUsable(std::span<const uint8_t>{}));
  fake::DmaCapable(false);
  KB_CHECK(!esp::DmaMemoryUsable(std::span<const uint8_t>(g_staging.Ref().base, 128U)));
}

KB_TEST("dma: after a transaction that never ended the staging stays held and the host failed") {
  Fresh();
  // A failed host keeps its claims until the application restarts: this file runs last
  // (tests/CMakeLists.txt) and this case last in it.
  esp::SpiBusConfig b = DmaBus();
  b.mosi_pin = 31;
  b.miso_pin = 32;
  b.sclk_pin = 33;
  b.staging = g_staging.Ref();
  esp::SpiBus bus{b};
  esp::SpiDeviceConfig d = Dev();
  d.cs_pin = 30;
  esp::SpiDevice dev{bus, d};
  KB_REQUIRE(dev.EnsureReady() == Status::Ok);
  fake::SpiStuck(true);
  std::array<uint8_t, 8> rx{};
  const std::array<uint8_t, 8> tx{};
  KB_CHECK_EQ(dev.Transfer(tx, rx), Status::Fault);
  const kb::dma::DmaHealth h = bus.Health();
  KB_CHECK(h.faulted);
  KB_CHECK_EQ(h.quarantined, 1U);
  KB_CHECK(h.mode == TransferMode::Dma);
  KB_CHECK_EQ(dev.Transfer(tx, rx), Status::Fault); // the SDK is not touched again
  KB_CHECK_EQ(dev.Release(), Status::NotReady);     // ESP-IDF refuses the stuck device
}
