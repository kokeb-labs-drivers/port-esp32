// The pin claim table of the port (claims.hpp). Platform-free.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "claims.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace kb::port::esp32::detail {

namespace {
constexpr std::size_t kHardwareKinds = 4U;

std::array<BitClaims<1U>, kHardwareKinds>& HardwareClaims() noexcept {
  // codeql::misra_deviation_next_line(DEV-003)
  static std::array<BitClaims<1U>, kHardwareKinds> claims{};
  return claims;
}
} // namespace

bool ClaimHardware(Hardware kind, uint8_t number) noexcept {
  return HardwareClaims()[static_cast<std::size_t>(kind)].Claim(number);
}

void UnclaimHardware(Hardware kind, uint8_t number) noexcept {
  HardwareClaims()[static_cast<std::size_t>(kind)].Unclaim(number);
}

BitClaims<2U>& PinClaims() noexcept {
  // codeql::misra_deviation_next_line(DEV-003)
  static BitClaims<2U> claims{};
  return claims;
}

bool ClaimPinList(std::span<const int16_t> pins) noexcept {
  bool ok = true;
  uint32_t taken = 0U; // pins of the list claimed so far, by position
  uint32_t position = 0U;
  for (const int16_t pin : pins) {
    if (ok && (pin >= 0)) {
      if (PinClaims().Claim(static_cast<uint32_t>(pin))) {
        taken |= uint32_t{1U} << position;
      } else {
        ok = false;
      }
    }
    ++position;
  }
  if (!ok) {
    position = 0U;
    for (const int16_t pin : pins) {
      if ((taken & (uint32_t{1U} << position)) != 0U) {
        PinClaims().Unclaim(static_cast<uint32_t>(pin));
      }
      ++position;
    }
  }
  return ok;
}

void UnclaimPinList(std::span<const int16_t> pins) noexcept {
  for (const int16_t pin : pins) {
    if (pin >= 0) {
      PinClaims().Unclaim(static_cast<uint32_t>(pin));
    }
  }
}

} // namespace kb::port::esp32::detail
