// The control of the model boundary (model_boundary.cpp): the port's DMA logic (spi.cpp,
// spi_dma.cpp, dma.cpp, adc_stream.cpp) linked against kb-core's host DMA-and-cache model instead
// of ESP-IDF. The model plays the SDK: queued and polling SPI transactions run on a
// kb::dma::host::DmaEngine, cache syncs act on a kb::dma::host::Machine, and the ADC continuous
// callbacks are driven by the rig.
//
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#ifndef KB_PORT_ESP32_TEST_DMA_MODEL_HPP
#define KB_PORT_ESP32_TEST_DMA_MODEL_HPP

#include <cstddef>
#include <cstdint>

#include "kb/dma/host/engine.hpp"
#include "kb/dma/host/machine.hpp"
#include "kb/port/esp32/adc_stream.hpp"

namespace model {

/** @brief Points the SDK model at `machine` and `engine` (one rig at a time); resets the rest. */
void Attach(kb::dma::host::Machine& machine, kb::dma::host::DmaEngine& engine);
/** @brief The next DMA receive writes `seed + i` (the engine's receive pattern). */
void NextSeed(uint8_t seed);
/** @brief While on, a started transaction never ends: the ESP32 has no stop request, so a stop that
 * is never confirmed is a transaction ESP-IDF never reports finished. SpiHostReset clears it. */
void Stall(bool on);
[[nodiscard]] bool Stalled();
/** @brief Engine ticks per millisecond of an SDK wait. */
void TicksPerMs(std::size_t ticks);
/** @brief The unit of a DMA receive length (the P4 cache line; 1 elsewhere). */
void RxUnit(std::size_t unit);
/** @brief The sink the port registered with AdcStreamOpen (null when none). */
[[nodiscard]] kb::port::esp32::AdcStreamSink* Sink();
/** @brief ADC continuous starts / stops seen. */
[[nodiscard]] uint32_t AdcStarts();
/** @brief SpiHostReset calls. */
[[nodiscard]] uint32_t HostResets();

} // namespace model

#endif
