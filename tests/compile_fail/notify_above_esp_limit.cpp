// A stream that notifies a task cannot run at an ESP-IDF level above 3 (kb priority below 4).
// EXPECT: runs above the RTOS syscall limit
// Test code. SPDX-License-Identifier: MIT
#include "kb/dma/assignment.hpp"
#include "kb/port/esp32/dma.hpp"

struct TooUrgent {
  static constexpr kb::dma::StreamAssignment kAssignment{.irq_priority = 2U,
                                                         .notify = kb::dma::Notify::Task};
};
constexpr bool kRefused =
    (kb::dma::RequireInterruptTier<TooUrgent, kb::port::esp32::EspRtosLimits>(), true);
