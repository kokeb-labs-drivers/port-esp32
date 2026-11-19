// Clock and BusyWait: thin forwarding of the clock and the busy-wait delay. Platform-free; every
// SDK call goes through sdk.hpp (src/boundary/time.cpp). SPDX-License-Identifier: MIT Copyright (c)
// 2026 The port-esp32 authors
#include "kb/port/esp32/time.hpp"
#include "kb/interfaces/types.hpp"
#include "kb/utils/time.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

// NOLINTNEXTLINE(readability-convert-member-functions-to-static): the contract calls a member
kb::Instant Clock::Now() const noexcept {
  return kb::Instant{sdk::NowUs()};
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static): the contract calls a member
void BusyWait::WaitUs(kb::Microseconds us) noexcept {
  sdk::DelayUs(us.Count());
}

} // namespace kb::port::esp32
