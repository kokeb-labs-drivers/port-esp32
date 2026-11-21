// Boundary file (deviations/boundary-files.yml): the DMA memory and cache calls of ESP-IDF v5.5
// (esp_mm esp_cache_msync, esp_hw_support esp_ptr_dma_capable and
// esp_dma_is_buffer_alignment_satisfied) behind sdk.hpp. Each function converts its arguments,
// calls the SDK and returns its result.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <cstddef>
#include <cstdint>
#include <span>

#include "esp_cache.h"
#include "esp_dma_utils.h"
#include "esp_err.h"
#include "esp_memory_utils.h"
#include "sdk.hpp"

namespace kb::port::esp32::sdk {

bool DmaMemoryUsable(std::span<const uint8_t> memory) noexcept {
  esp_dma_mem_info_t info{};
  info.extra_heap_caps = 0;
  info.dma_alignment_bytes = 4U; // the GDMA and SPI DMA descriptor rule; the cache line is added
  const void* const base = memory.data();
  return esp_ptr_dma_capable(base) &&
         esp_dma_is_buffer_alignment_satisfied(base, memory.size(), info);
}

int32_t CacheWriteBack(std::span<const std::byte> range) noexcept {
  // esp_cache_msync takes `void*` for both directions; C2M only writes cache lines back to memory.
  // codeql::misra_deviation_next_line(DEV-006)
  void* const base =
      const_cast<std::byte*>(range.data()); // NOLINT(cppcoreguidelines-pro-type-const-cast)
  return esp_cache_msync(base, range.size(), ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

int32_t CacheInvalidate(std::span<std::byte> range) noexcept {
  return esp_cache_msync(range.data(), range.size(), ESP_CACHE_MSYNC_FLAG_DIR_M2C);
}

void MemoryBarrier() noexcept {
  __atomic_thread_fence(__ATOMIC_SEQ_CST); // the compiler builtin: no <atomic> in the boundary
}

} // namespace kb::port::esp32::sdk
