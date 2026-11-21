// The SPI DMA bench app (docs/assurance/bench/procedure-esp32.md): on a devkit with a MOSI-MISO
// jumper it runs every SPI DMA bench item of the port once and prints one line per item:
//   KB-BENCH <item> PASS|FAIL key=value ...
// Items: SPI-5B (T8 on the ESP32: 5-byte receive, canaries after it), SPI-1028, SPI-UNALIGNED,
// SPI-PSRAM (ESP32-S3 with PSRAM), SPI-ZEROCOPY, SPI-LATE (a transfer that ends after its
// deadline), SPI-HEAP (no allocation after start), SPI-GDMA (no free DMA channel: the reported
// fallback). Bench code. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "bench.hpp"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kb/port/esp32/port.hpp"
#include "soc/soc_caps.h"
#if SOC_AHB_GDMA_SUPPORTED
#include "esp_async_memcpy.h"
#endif

namespace {
namespace esp = kb::port::esp32;
using kb::Status;

constexpr std::size_t kFrame = 2048U;
constexpr uint8_t kCanary = 0xC3U;
constexpr std::size_t kCanaryBytes = 8U;

esp::SpiStaging<kFrame> g_staging{};                     // the host's static staging buffer
kb::dma::Buffer<esp::InternalRam, kFrame> g_zero_copy{}; // a handler's typed DMA buffer

struct Guarded {
  std::array<uint8_t, kFrame + kCanaryBytes + 1U> bytes{};
  void Arm(std::size_t offset, std::size_t n) {
    bytes.fill(0U);
    std::fill_n(bytes.begin() + static_cast<std::ptrdiff_t>(offset + n), kCanaryBytes, kCanary);
  }
  [[nodiscard]] bool Intact(std::size_t offset, std::size_t n) const {
    return std::all_of(bytes.begin() + static_cast<std::ptrdiff_t>(offset + n),
                       bytes.begin() + static_cast<std::ptrdiff_t>(offset + n + kCanaryBytes),
                       [](uint8_t b) { return b == kCanary; });
  }
};
Guarded g_rx{};
std::array<uint8_t, kFrame> g_tx{};

kb::utils::Deadline Far() {
  return kb::utils::Deadline::After(kb::Instant{static_cast<uint64_t>(esp_timer_get_time())},
                                    kb::Milliseconds{500U});
}

esp::SpiBusConfig BusConfig(bool fallback) {
  esp::SpiBusConfig c{};
  c.host = bench::kSpiHost;
  c.sclk_pin = bench::kSclk;
  c.mosi_pin = bench::kMosi;
  c.miso_pin = bench::kMiso;
  c.dma = true;
  c.staging = g_staging.Ref();
  c.dma_fallback = fallback;
  return c;
}

esp::SpiDeviceConfig DevConfig(uint32_t hz) {
  esp::SpiDeviceConfig c{};
  c.cs_pin = bench::kCs;
  c.clock_hz = hz;
  c.timeout = kb::Milliseconds{100U};
  return c;
}

// One loopback frame of `n` bytes into the caller's buffer at `offset`: the data comes back, the
// canaries stay.
bool Loopback(esp::SpiDevice& dev, std::size_t offset, std::size_t n, uint32_t& mismatches) {
  for (std::size_t i = 0U; i < n; ++i) {
    g_tx[i] = static_cast<uint8_t>((i * 7U) + 3U);
  }
  g_rx.Arm(offset, n);
  const Status s = dev.Transfer(std::span<const uint8_t>(g_tx.data(), n),
                                std::span<uint8_t>(g_rx.bytes).subspan(offset, n));
  mismatches = 0U;
  for (std::size_t i = 0U; i < n; ++i) {
    mismatches += (g_rx.bytes[offset + i] == g_tx[i]) ? 0U : 1U;
  }
  return (s == Status::Ok) && (mismatches == 0U) && g_rx.Intact(offset, n);
}

void Frames(esp::SpiDevice& dev) {
  uint32_t bad = 0U;
  const bool five = Loopback(dev, 0U, 5U, bad);
  bench::Result("SPI-5B", five, "bytes=5 mismatches=%" PRIu32 " canary=%s", bad,
                g_rx.Intact(0U, 5U) ? "intact" : "hit");
#if CONFIG_IDF_TARGET_ESP32
  // T8 on the chip whose DMA rounds a receive up to whole words: the rounding must land in the
  // staging buffer.
  bench::Result("T8", five, "rx=5 rounding_to=4 canary=%s", g_rx.Intact(0U, 5U) ? "intact" : "hit");
#endif
  const bool big = Loopback(dev, 0U, 1028U, bad);
  bench::Result("SPI-1028", big, "bytes=1028 mismatches=%" PRIu32, bad);
  const bool odd = Loopback(dev, 1U, 37U, bad);
  bench::Result("SPI-UNALIGNED", odd, "offset=1 bytes=37 mismatches=%" PRIu32, bad);
}

void Psram(esp::SpiDevice& dev, uint8_t* psram) {
  if (psram == nullptr) {
    bench::Skip("SPI-PSRAM", "no-psram");
    return;
  }
  for (std::size_t i = 0U; i < 512U; ++i) {
    g_tx[i] = static_cast<uint8_t>(i);
  }
  std::memset(psram, 0, 512U);
  const Status s =
      dev.Transfer(std::span<const uint8_t>(g_tx.data(), 512U), std::span<uint8_t>(psram, 512U));
  const bool same = std::memcmp(psram, g_tx.data(), 512U) == 0;
  bench::Result("SPI-PSRAM", (s == Status::Ok) && same, "bytes=512 status=%s same=%d",
                kb::ToString(s), same ? 1 : 0);
}

void ZeroCopy(esp::SpiDevice& dev) {
  esp::SpiDmaChannel<esp::InternalRam> ch{dev};
  std::span<std::byte> view = g_zero_copy.CpuView();
  for (std::size_t i = 0U; i < view.size(); ++i) {
    view[i] = static_cast<std::byte>(i);
  }
  Status s = ch.Start(g_zero_copy, kb::dma::Direction::ToDevice, 1024U, Far());
  esp::SpiDmaResult r{};
  for (int i = 0; (s == Status::Ok) && (i < 200) && (r.state != esp::SpiDmaState::Done); ++i) {
    r = ch.Poll(kb::Instant{static_cast<uint64_t>(esp_timer_get_time())});
    vTaskDelay(1);
  }
  const bool tx_ok = (s == Status::Ok) && (r.status == Status::Ok) &&
                     (g_zero_copy.Current() == kb::dma::BufferState::Cpu);
  s = ch.Start(g_zero_copy, kb::dma::Direction::FromDevice, 1024U, Far());
  r = esp::SpiDmaResult{};
  for (int i = 0; (s == Status::Ok) && (i < 200) && (r.state != esp::SpiDmaState::Done); ++i) {
    r = ch.Poll(kb::Instant{static_cast<uint64_t>(esp_timer_get_time())});
    vTaskDelay(1);
  }
  const bool rx_ok = (s == Status::Ok) && (r.status == Status::Ok) &&
                     (g_zero_copy.Current() == kb::dma::BufferState::Cpu);
  const bool once = ch.Poll(kb::Instant{static_cast<uint64_t>(esp_timer_get_time())}).state ==
                    esp::SpiDmaState::Idle;
  bench::Result("SPI-ZEROCOPY", tx_ok && rx_ok && once,
                "tx=%d rx=%d reported_once=%d starts=%" PRIu32, tx_ok ? 1 : 0, rx_ok ? 1 : 0,
                once ? 1 : 0, ch.Stats().starts);
}

void Late(esp::SpiBus& bus) {
  // A slow device: 2048 bytes at 200 kHz take about 82 ms, the deadline is 1 ms.
  esp::SpiDeviceConfig cfg = DevConfig(200000U);
  cfg.cs_pin = -1; // the bench device keeps the chip select
  esp::SpiDevice slow{bus, cfg};
  if (slow.EnsureReady() != Status::Ok) {
    bench::Result("SPI-LATE", false, "ensure_ready=%s", kb::ToString(slow.LastError().Error()));
    return;
  }
  esp::SpiDmaChannel<esp::InternalRam> ch{slow};
  const kb::Instant t0{static_cast<uint64_t>(esp_timer_get_time())};
  Status s = ch.Start(g_zero_copy, kb::dma::Direction::FromDevice, kFrame,
                      kb::utils::Deadline::After(t0, kb::Milliseconds{1U}));
  vTaskDelay(pdMS_TO_TICKS(2));
  const esp::SpiDmaResult r = ch.Poll(kb::Instant{static_cast<uint64_t>(esp_timer_get_time())});
  const uint64_t took = static_cast<uint64_t>(esp_timer_get_time()) - t0.Us();
  const bool pass = (s == Status::Ok) && (r.status == Status::Timeout) &&
                    (g_zero_copy.Current() == kb::dma::BufferState::Cpu) && !bus.IsFailed();
  bench::Result("SPI-LATE", pass, "status=%s buffer=%s took_us=%" PRIu64 " host_failed=%d",
                kb::ToString(r.status),
                (g_zero_copy.Current() == kb::dma::BufferState::Cpu) ? "cpu" : "not-cpu", took,
                bus.IsFailed() ? 1 : 0);
  s = slow.Release();
  static_cast<void>(s);
}

void Gdma() {
#if SOC_AHB_GDMA_SUPPORTED
  // Take every GDMA pair with a public driver (async memcpy), then ask for an SPI host with DMA.
  std::array<async_memcpy_handle_t, 8> taken{};
  std::size_t n = 0U;
  async_memcpy_config_t config{};
  config.backlog = 8U;
  config.dma_burst_size = 16U;
  while ((n < taken.size()) && (esp_async_memcpy_install(&config, &taken[n]) == ESP_OK)) {
    ++n;
  }
  esp::SpiBus strict{BusConfig(false)};
  const Status refused = strict.EnsureReady();
  esp::SpiBus fallback{BusConfig(true)};
  const Status up = fallback.EnsureReady();
  const kb::dma::Capabilities caps = fallback.Capabilities();
  const bool pass = (refused == Status::Unsupported) && (up == Status::Ok) &&
                    (caps.mode == kb::dma::TransferMode::Polled) &&
                    (fallback.LastError().Detail() == static_cast<uint32_t>(ESP_ERR_NOT_FOUND));
  bench::Result("SPI-GDMA", pass, "pairs_taken=%u strict=%s fallback=%s mode=%s ceiling=%u",
                static_cast<unsigned>(n), kb::ToString(refused), kb::ToString(up),
                kb::dma::ToString(caps.mode), static_cast<unsigned>(caps.max_frame_bytes));
  static_cast<void>(fallback.Release());
  for (std::size_t i = 0U; i < n; ++i) {
    static_cast<void>(esp_async_memcpy_uninstall(taken[i]));
  }
#else
  bench::Skip("SPI-GDMA", "no-gdma");
#endif
}

void Run() {
  bench::Header("dma_spi_bench");
  uint8_t* psram = nullptr;
#if CONFIG_SPIRAM
  psram = static_cast<uint8_t*>(
      heap_caps_malloc(512U, MALLOC_CAP_SPIRAM)); // before start: not counted below
#endif
  esp::SpiBus bus{BusConfig(false)};
  esp::SpiDevice dev{bus, DevConfig(8000000U)};
  const Status up = dev.EnsureReady();
  bench::Result("SPI-START", up == Status::Ok, "status=%s mode=%s staging=%u", kb::ToString(up),
                kb::dma::ToString(bus.Capabilities().mode),
                static_cast<unsigned>(bus.Capabilities().staging_bytes));
  if (up == Status::Ok) {
    const bench::Heap before = bench::HeapNow();
    Frames(dev);
    Psram(dev, psram);
    ZeroCopy(dev);
    uint32_t bad = 0U;
    uint32_t failures = 0U;
    for (int i = 0; i < 1000; ++i) {
      failures += Loopback(dev, static_cast<std::size_t>(i % 3),
                           5U + static_cast<std::size_t>(i % 200), bad)
                      ? 0U
                      : 1U;
    }
    const bench::Heap after = bench::HeapNow();
    bench::Result(
        "SPI-HEAP",
        (after.blocks == before.blocks) && (after.bytes == before.bytes) && (failures == 0U),
        "frames=1000 failures=%" PRIu32
        " blocks_before=%u blocks_after=%u bytes_before=%u bytes_after=%u",
        failures, static_cast<unsigned>(before.blocks), static_cast<unsigned>(after.blocks),
        static_cast<unsigned>(before.bytes), static_cast<unsigned>(after.bytes));
    Late(bus);
    static_cast<void>(dev.Release());
  }
  static_cast<void>(bus.Release());
  Gdma();
  bench::Summary("dma_spi_bench");
}
} // namespace

extern "C" void app_main(void) {
  Run();
}
