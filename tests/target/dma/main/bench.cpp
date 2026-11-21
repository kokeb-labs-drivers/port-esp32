// Shared helpers of the DMA bench apps (bench.hpp).
// Bench code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "bench.hpp"

#include <cinttypes>
#include <cstdarg>
#include <cstdio>

#include "esp_chip_info.h"
#include "esp_clk_tree.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"

namespace bench {
namespace {
unsigned g_pass = 0U;
unsigned g_fail = 0U;
} // namespace

void Result(const char* item, bool pass, const char* format, ...) {
  std::printf("KB-BENCH %s %s ", item, pass ? "PASS" : "FAIL");
  va_list args;
  va_start(args, format);
  std::vprintf(format, args);
  va_end(args);
  std::printf("\n");
  (pass ? g_pass : g_fail) += 1U;
}

void Skip(const char* item, const char* reason) {
  std::printf("KB-BENCH %s SKIP reason=%s\n", item, reason);
}

void Header(const char* app) {
  esp_chip_info_t chip{};
  esp_chip_info(&chip);
  uint32_t cpu_hz = 0U;
  static_cast<void>(esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU,
                                                 ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &cpu_hz));
  std::printf(
      "KB-BENCH-RUN app=%s target=%s chip_model=%d revision=%u.%u cores=%u idf=%s cpu_hz=%" PRIu32
      " tick_hz=%u\n",
      app, CONFIG_IDF_TARGET, static_cast<int>(chip.model), chip.revision / 100U,
      chip.revision % 100U, static_cast<unsigned>(chip.cores), esp_get_idf_version(), cpu_hz,
      static_cast<unsigned>(configTICK_RATE_HZ));
}

void Summary(const char* app) {
  std::printf("KB-BENCH SUMMARY app=%s pass=%u fail=%u\n", app, g_pass, g_fail);
}

Heap HeapNow() {
  multi_heap_info_t info{};
  heap_caps_get_info(&info, MALLOC_CAP_8BIT);
  return Heap{.blocks = info.allocated_blocks, .bytes = info.total_allocated_bytes};
}

} // namespace bench
