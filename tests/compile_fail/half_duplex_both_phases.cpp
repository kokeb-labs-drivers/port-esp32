// Half-duplex SPI DMA with both phases on the ESP32 is refused at build time.
// EXPECT: half-duplex SPI DMA with both a write and a read phase
// Test code. SPDX-License-Identifier: MIT
#include "kb/port/esp32/dma.hpp"

namespace esp = kb::port::esp32;
constexpr bool kRefused =
    (esp::RequireSpiHalfDuplexPhases<esp::Target::Esp32, true, true, true>(), true);
