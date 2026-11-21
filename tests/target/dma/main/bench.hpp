// Shared helpers of the DMA bench apps: the result line, the heap snapshot and the bench pins per
// target. Bench code (built in CI, run by hand). SPDX-License-Identifier: MIT Copyright (c) 2026
// The port-esp32 authors
#ifndef KB_PORT_ESP32_BENCH_HPP
#define KB_PORT_ESP32_BENCH_HPP

#include <cstddef>
#include <cstdint>

#include "sdkconfig.h"

namespace bench {

/// Prints `KB-BENCH <item> PASS|FAIL <details>` (details: printf format) and counts it.
void Result(const char* item, bool pass, const char* format, ...)
    __attribute__((format(printf, 3, 4)));
/// Prints `KB-BENCH <item> SKIP reason=<reason>` (the item needs hardware this board lacks).
void Skip(const char* item, const char* reason);
/// Prints `KB-BENCH SUMMARY pass=<n> fail=<n>` and the build facts the evidence form asks for.
void Summary(const char* app);
/// Prints the run header: chip, revision, ESP-IDF version, CPU clock, tick rate.
void Header(const char* app);

/// Allocated blocks and bytes of every heap now.
struct Heap {
  std::size_t blocks{0U};
  std::size_t bytes{0U};
};
[[nodiscard]] Heap HeapNow();

// The bench wiring (docs/assurance/bench/procedure-esp32.md): SPI2 (or VSPI on the ESP32) with a
// MOSI-MISO jumper, three ADC1 inputs on a divider.
#if CONFIG_IDF_TARGET_ESP32
inline constexpr uint8_t kSpiHost = 2U; // SPI3_HOST (VSPI): no strapping pin on MISO
inline constexpr int16_t kSclk = 18;
inline constexpr int16_t kMosi = 23;
inline constexpr int16_t kMiso = 19;
inline constexpr int16_t kCs = 5;
inline constexpr int16_t kAdc0 = 36;
inline constexpr int16_t kAdc1 = 39;
inline constexpr int16_t kAdc2 = 34;
#elif CONFIG_IDF_TARGET_ESP32S3
inline constexpr uint8_t kSpiHost = 1U; // SPI2_HOST on its IO MUX pins
inline constexpr int16_t kSclk = 12;
inline constexpr int16_t kMosi = 11;
inline constexpr int16_t kMiso = 13;
inline constexpr int16_t kCs = 10;
inline constexpr int16_t kAdc0 = 1;
inline constexpr int16_t kAdc1 = 2;
inline constexpr int16_t kAdc2 = 3;
#else // ESP32-C6 and other RISC-V targets
inline constexpr uint8_t kSpiHost = 1U; // SPI2_HOST
inline constexpr int16_t kSclk = 19;
inline constexpr int16_t kMosi = 18;
inline constexpr int16_t kMiso = 20;
inline constexpr int16_t kCs = 21;
inline constexpr int16_t kAdc0 = 0;
inline constexpr int16_t kAdc1 = 1;
inline constexpr int16_t kAdc2 = 4;
#endif

} // namespace bench

#endif
