// A controller owns an SDK handle and a lock: it cannot be copied.
// EXPECT: (deleted|use of deleted)
// Test code. SPDX-License-Identifier: MIT
#include "kb/port/esp32/port.hpp"

void Use(const kb::port::esp32::I2cController& bus) {
  kb::port::esp32::I2cController copy{bus};
  static_cast<void>(copy);
}
