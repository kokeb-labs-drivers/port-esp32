/**
 * @file claims.hpp
 * @brief Lock-free claim tables shared by the port objects: GPIO pins (one owner per pin, across
 * every peripheral), controller numbers (I2C, UART, SPI host, TWAI), SPI device slots and ADC
 * channel slots. Private to the port; platform-free. Used by the sources of gpio, i2c, uart, spi,
 * adc, pwm and can (src/<header>.cpp); implemented in claims.cpp.
 *
 * A claim is one atomic compare-and-swap on a bit, so two tasks bringing objects up at once can
 * never take the same pin or slot, and no critical section or RTOS call is needed.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_SRC_CLAIMS_HPP
#define KB_PORT_ESP32_SRC_CLAIMS_HPP

#include <array>
// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

namespace kb::port::esp32::detail {

/** @brief `Words` x 32 claimable bits. */
template <std::size_t Words>
class BitClaims {
public:
  static constexpr uint32_t kBits = static_cast<uint32_t>(Words * 32U);

  /** @brief Takes `bit`; false when it is out of range or already taken. */
  [[nodiscard]] bool Claim(uint32_t bit) noexcept {
    bool done = false;
    if (bit < kBits) {
      const uint32_t mask = uint32_t{1U} << (bit % 32U);
      const uint32_t old = words_[bit / 32U].fetch_or(mask);
      done = (old & mask) == 0U;
    }
    return done;
  }
  /** @brief Gives `bit` back. */
  void Unclaim(uint32_t bit) noexcept {
    if (bit < kBits) {
      static_cast<void>(words_[bit / 32U].fetch_and(~(uint32_t{1U} << (bit % 32U))));
    }
  }
  /** @brief Takes the lowest free bit below `limit`; `limit` when none is free. */
  [[nodiscard]] uint32_t ClaimFirst(uint32_t limit) noexcept {
    uint32_t found = limit;
    for (uint32_t bit = 0U; (bit < limit) && (bit < kBits) && (found == limit); ++bit) {
      if (Claim(bit)) {
        found = bit;
      }
    }
    return found;
  }

private:
  std::array<std::atomic<uint32_t>, Words> words_{};
};

/** @brief The peripherals whose hardware numbers are claimed (one owner per controller). */
enum class Hardware : uint8_t { I2c = 0, Uart = 1, SpiHost = 2, Twai = 3 };

/** @brief Takes controller `number` of `kind`; false when another object has it. */
[[nodiscard]] bool ClaimHardware(Hardware kind, uint8_t number) noexcept;
/** @brief Gives controller `number` of `kind` back. */
void UnclaimHardware(Hardware kind, uint8_t number) noexcept;

/** @brief GPIO pins 0 to 63: one owner per pin, whatever the peripheral. */
[[nodiscard]] BitClaims<2U>& PinClaims() noexcept;

/**
 * @brief Claims every pin of `pins` that is not -1, all or none. False (nothing claimed) when one
 * is taken already.
 */
[[nodiscard]] bool ClaimPinList(std::span<const int16_t> pins) noexcept;
/** @brief Gives back every pin of `pins` that is not -1. */
void UnclaimPinList(std::span<const int16_t> pins) noexcept;

/** @brief ClaimPinList() of the pins given. */
template <typename... Pins>
[[nodiscard]] bool ClaimPins(Pins... pins) noexcept {
  const std::array<int16_t, sizeof...(Pins)> list{static_cast<int16_t>(pins)...};
  return ClaimPinList(list);
}
/** @brief UnclaimPinList() of the pins given. */
template <typename... Pins>
void UnclaimPins(Pins... pins) noexcept {
  const std::array<int16_t, sizeof...(Pins)> list{static_cast<int16_t>(pins)...};
  UnclaimPinList(list);
}

} // namespace kb::port::esp32::detail

#endif // KB_PORT_ESP32_SRC_CLAIMS_HPP
