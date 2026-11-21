// EspCacheOps and the start-time placement check of DMA memory. Platform-free; the cache sync, the
// barrier and the memory queries go through sdk.hpp (src/boundary/dma.cpp).
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "kb/port/esp32/dma.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/port/esp32/error.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

void EspCacheOps::Clean(std::span<const std::byte> range) noexcept {
  if (!range.empty() && (sdk::CacheWriteBack(range) != esp_err::kOk)) {
    ++failures_;
  }
}

void EspCacheOps::Invalidate(std::span<std::byte> range) noexcept {
  if (!range.empty() && (sdk::CacheInvalidate(range) != esp_err::kOk)) {
    ++failures_;
  }
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static): a kb::dma::CacheOps member
void EspCacheOps::Barrier() noexcept {
  sdk::MemoryBarrier();
}

bool DmaMemoryUsable(std::span<const uint8_t> memory) noexcept {
  return !memory.empty() && sdk::DmaMemoryUsable(memory);
}

} // namespace kb::port::esp32
