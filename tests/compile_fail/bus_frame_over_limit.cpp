// A 97-byte frame on an SPI bus without DMA fails the build.
// EXPECT: exceeds what this bus carries
// Test code. SPDX-License-Identifier: MIT
#include "kb/dma/assignment.hpp"
#include "kb/port/esp32/dma.hpp"

namespace esp = kb::port::esp32;
using Bus = esp::SpiLimits<esp::Target::Esp32C6, false>;
constexpr bool kRefused = (kb::dma::RequireBusCarries<Bus, 97U>(), true);
