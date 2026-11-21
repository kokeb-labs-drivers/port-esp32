// The ADC continuous bench app (docs/assurance/bench/procedure-esp32.md): three ADC1 inputs on a
// resistor divider, 1 kHz per channel into the kb stream, one line per item:
//   KB-BENCH <item> PASS|FAIL key=value ...
// Items: ADC-START, ADC-RATE (rate and jitter of the interrupt stamps), ADC-DATA (channels in
// pattern order, values in range), ADC-ISR (cost of the port's interrupt path in cycles),
// ADC-BLOCKED (consumer held: ring overrun and pool overflow counted separately, the next frame
// flagged), ADC-NVS (NVS writes while streaming, IRAM-safe), ADC-STALL (the conversions stopped:
// the supervisor sees a stall and asks for a restart), ADC-HEAP (no allocation after start),
// ADC-SHAPE (a board shape that differs: DMA refused, the fallback reported). Bench code.
// SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>

#include "bench.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kb/port/esp32/port.hpp"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"

namespace {
namespace esp = kb::port::esp32;
using kb::Status;
using Stream = esp::AdcStreamDefault;

constexpr std::size_t kResultBytes = SOC_ADC_DIGI_RESULT_BYTES;
constexpr std::size_t kResults = esp::kAdcDefaultFrameBytes / kResultBytes;
constexpr uint32_t kRate =
    (SOC_ADC_SAMPLE_FREQ_THRES_LOW > 3000) ? SOC_ADC_SAMPLE_FREQ_THRES_LOW : 3000U;
constexpr uint64_t kPeriodUs = (uint64_t{kResults} * 1000000U) / kRate;

kb::Instant Now() {
  return kb::Instant{static_cast<uint64_t>(esp_timer_get_time())};
}

esp::AdcStreamConfig Config(std::size_t elements) {
  esp::AdcStreamConfig c{};
  c.pins = {bench::kAdc0, bench::kAdc1, bench::kAdc2, -1, -1, -1, -1, -1};
  c.sample_hz = kRate;
  c.pool_frames = 4U;
  c.shape = kb::dma::StreamShape{
      .frames = esp::kAdcDefaultFrames, .element_bytes = kResultBytes, .elements = elements};
  c.supervision.frame_period = kb::Microseconds{static_cast<uint32_t>(kPeriodUs)};
  c.supervision.stall_periods = 10U;
  c.iram_safe = true; // the app writes NVS while streaming (sdkconfig.adc)
  return c;
}

Stream g_stream{Config(kResults)}; // static: internal RAM, as the IRAM-safe check requires

struct Run {
  uint32_t frames{0U};
  uint32_t gaps{0U};
  uint32_t torn{0U};
  uint32_t bad_order{0U};
  uint32_t out_of_range{0U};
  uint64_t min_dt{UINT64_MAX};
  uint64_t max_dt{0U};
  uint64_t last_at{0U};
};

// The owner task's pass: take every frame, check it, give it back (bounded: at most the ring per
// pass).
void Drain(Run& run) {
  Stream::FrameView view{};
  for (std::size_t n = 0U; (n < esp::kAdcDefaultFrames) && (g_stream.TryTake(view) == Status::Ok);
       ++n) {
    ++run.frames;
    run.gaps += view.Gap() ? 1U : 0U;
    run.torn += view.Torn() ? 1U : 0U;
    if (run.last_at != 0U) {
      const uint64_t dt = view.At().Us() - run.last_at;
      run.min_dt = std::min(run.min_dt, dt);
      run.max_dt = std::max(run.max_dt, dt);
    }
    run.last_at = view.At().Us();
    esp::AdcSample first{};
    for (std::size_t i = 0U; i < kResults; ++i) {
      esp::AdcSample s{};
      if (Stream::Decode(view.Data(), i, s) != Status::Ok) {
        ++run.out_of_range;
      } else if (i == 0U) {
        first = s;
      } else if (s.raw > 4095U) {
        ++run.out_of_range;
      } else {
        // Channels repeat in pattern order: three apart, the same channel.
        run.bad_order += ((i % 3U) == 0U) && (s.channel != first.channel) ? 1U : 0U;
      }
    }
    g_stream.Give(std::move(view));
  }
}

Run Stream_for(uint32_t ms) {
  Run run{};
  const uint64_t end = Now().Us() + (uint64_t{ms} * 1000U);
  while (Now().Us() < end) {
    Drain(run);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return run;
}

void RateAndData() {
  const bench::Heap before = bench::HeapNow();
  const Run run = Stream_for(2000U);
  const bench::Heap after = bench::HeapNow();
  const double rate = run.frames / 2.0;
  const double expected = 1000000.0 / static_cast<double>(kPeriodUs);
  const bool rate_ok = (rate > expected * 0.95) && (rate < expected * 1.05) && (run.gaps == 0U);
  bench::Result("ADC-RATE", rate_ok,
                "frames_per_s=%.1f expected=%.1f gaps=%" PRIu32 " torn=%" PRIu32
                " dt_min_us=%" PRIu64 " dt_max_us=%" PRIu64 " period_us=%" PRIu64,
                rate, expected, run.gaps, run.torn, run.min_dt, run.max_dt, kPeriodUs);
  bench::Result("ADC-DATA", (run.bad_order == 0U) && (run.out_of_range == 0U),
                "bad_order=%" PRIu32 " invalid=%" PRIu32, run.bad_order, run.out_of_range);
  const uint32_t cycles = g_stream.Stats().max_isr_cycles;
  bench::Result("ADC-ISR", (cycles > 0U) && (cycles < 50000U), "max_isr_cycles=%" PRIu32, cycles);
  bench::Result("ADC-HEAP", (after.blocks == before.blocks) && (after.bytes == before.bytes),
                "blocks_before=%u blocks_after=%u bytes_before=%u bytes_after=%u",
                static_cast<unsigned>(before.blocks), static_cast<unsigned>(after.blocks),
                static_cast<unsigned>(before.bytes), static_cast<unsigned>(after.bytes));
}

void Blocked() {
  const kb::dma::StreamStats s0 = g_stream.Stats();
  vTaskDelay(pdMS_TO_TICKS(1000)); // the consumer does not run for 1 s
  const kb::dma::StreamStats s1 = g_stream.Stats();
  const Run run = Stream_for(200U);
  const uint32_t overruns = s1.overruns - s0.overruns;
  const uint32_t pool = s1.hw_overruns - s0.hw_overruns;
  bench::Result("ADC-BLOCKED", (overruns > 0U) && (pool > 0U) && (run.gaps > 0U),
                "ring_overruns=%" PRIu32 " pool_overflows=%" PRIu32 " gaps_after=%" PRIu32,
                overruns, pool, run.gaps);
}

void Nvs() {
  esp_err_t err = nvs_flash_init();
  if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
    static_cast<void>(nvs_flash_erase());
    err = nvs_flash_init();
  }
  nvs_handle_t h = 0U;
  if (err == ESP_OK) {
    err = nvs_open("kbbench", NVS_READWRITE, &h);
  }
  Run run{};
  uint32_t writes = 0U;
  std::array<uint8_t, 512> blob{};
  const uint64_t end = Now().Us() + 2000000U;
  while ((err == ESP_OK) && (Now().Us() < end)) {
    blob[0] = static_cast<uint8_t>(writes);
    err = nvs_set_blob(h, "blob", blob.data(), blob.size());
    err = (err == ESP_OK) ? nvs_commit(h) : err;
    ++writes;
    Drain(run);
  }
  if (h != 0U) {
    nvs_close(h);
  }
  const double rate = run.frames / 2.0;
  const double expected = 1000000.0 / static_cast<double>(kPeriodUs);
  bench::Result("ADC-NVS", (err == ESP_OK) && (writes > 0U) && (rate > expected * 0.8),
                "nvs_writes=%" PRIu32 " frames_per_s=%.1f expected=%.1f torn=%" PRIu32 " err=0x%x",
                writes, rate, expected, run.torn, static_cast<unsigned>(err));
}

void Stall() {
  // Supervision with real interrupt stamps: the owner reads its clock, then the interrupt may stamp
  // a later frame (the signed-time race of an earlier implementation). No false stall in 2 s of
  // supervised streaming.
  Run run{};
  uint32_t false_stalls = 0U;
  const uint64_t end = Now().Us() + 2000000U;
  while (Now().Us() < end) {
    const kb::Instant before = Now();
    Drain(run);
    false_stalls += g_stream.CheckStall(before) ? 1U : 0U;
    vTaskDelay(pdMS_TO_TICKS(3));
  }
  // The restart path: Stop, Start (idempotent while running), NoteRestart; frames come back.
  const Status stopped = g_stream.Stop();
  const Status started = g_stream.Start(Now());
  const Status twice = g_stream.Start(Now());
  g_stream.NoteRestart(Now());
  const Run after = Stream_for(300U);
  bench::Result("ADC-STALL",
                (false_stalls == 0U) && (stopped == Status::Ok) && (started == Status::Ok) &&
                    (twice == Status::Ok) && (after.frames > 0U),
                "false_stalls=%" PRIu32 " restarts=%" PRIu32 " frames_after_restart=%" PRIu32
                " budget_us=%" PRIu64,
                false_stalls, g_stream.Stats().restarts, after.frames, kPeriodUs * 10U);
}

void Shape() {
  static_cast<void>(g_stream.Release());
  Stream wrong{Config(kResults - 3U)};
  const Status s = wrong.EnsureReady();
  const kb::dma::Capabilities caps = wrong.Capabilities();
  bench::Result("ADC-SHAPE",
                (s == Status::NotReady) && (caps.mode == kb::dma::TransferMode::Polled),
                "ensure_ready=%s mode=%s", kb::ToString(s), kb::dma::ToString(caps.mode));
}

void RunAll() {
  bench::Header("dma_adc_bench");
  Status s = g_stream.EnsureReady();
  s = (s == Status::Ok) ? g_stream.Start(Now()) : s;
  bench::Result("ADC-START", s == Status::Ok,
                "status=%s detail=0x%" PRIx32 " mode=%s rate_hz=%" PRIu32 " frame_bytes=%u",
                kb::ToString(s), g_stream.LastError().Detail(),
                kb::dma::ToString(g_stream.Capabilities().mode), kRate,
                static_cast<unsigned>(esp::kAdcDefaultFrameBytes));
  if (s == Status::Ok) {
    RateAndData();
    Blocked();
    Nvs();
    Stall();
  }
  Shape();
  bench::Summary("dma_adc_bench");
}
} // namespace

extern "C" void app_main(void) {
  RunAll();
}
