// Zero-copy SPI DMA buffers are padded to whole words at least.
// EXPECT: padded to whole words
// Test code. SPDX-License-Identifier: MIT
#include "kb/port/esp32/spi_dma.hpp"

struct ByteRegion {
  static constexpr kb::dma::CachePolicy kPolicy = kb::dma::CachePolicy::Coherent;
  static constexpr std::size_t kLineBytes = 1U;
  static constexpr std::size_t kAlignBytes = 1U;
  static constexpr kb::dma::MasterSet kMasters = kb::port::esp32::kMasterGdma;
};
void Use(kb::port::esp32::SpiDevice& dev) {
  kb::port::esp32::SpiDmaChannel<ByteRegion> ch{dev};
  static_cast<void>(ch);
}
