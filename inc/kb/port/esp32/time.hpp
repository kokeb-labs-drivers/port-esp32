/**
 * @file time.hpp
 * @brief `kb::port::esp32::Clock` (satisfies `kb::iface::MonotonicClock`, from `esp_timer`) and
 * `kb::port::esp32::BusyWait` (satisfies `kb::iface::BusyWait`, from `esp_rom_delay_us`).
 *
 * `esp_timer_get_time()` counts microseconds since boot in 64 bits from a hardware timer: it never
 * goes back and does not wrap in practice (584 000 years), works before the scheduler starts and
 * from an interrupt. Both types hold no state; any number of them may exist.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_TIME_HPP
#define KB_PORT_ESP32_TIME_HPP

#include "kb/interfaces/time.hpp"
#include "kb/utils/time.hpp"

namespace kb::port::esp32 {

/** @brief The 64-bit microsecond clock; satisfies kb::iface::MonotonicClock. */
class Clock {
public:
  /** @brief Microseconds since boot. */
  [[nodiscard]] kb::Instant Now() const noexcept;
};

/** @brief A spin delay; satisfies kb::iface::BusyWait. */
class BusyWait {
public:
  /** @brief Spins for at least `us` (any value; waits of 1 ms or more should sleep instead). */
  void WaitUs(kb::Microseconds us) noexcept;
};

static_assert(kb::iface::MonotonicClock<Clock>, "Clock satisfies kb::iface::MonotonicClock");
static_assert(kb::iface::BusyWait<BusyWait>, "BusyWait satisfies kb::iface::BusyWait");

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_TIME_HPP
