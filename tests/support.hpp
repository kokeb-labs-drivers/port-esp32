// Shared by the host tests: the harness, the fake's control and the port's public headers.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#ifndef KB_PORT_ESP32_TEST_SUPPORT_HPP
#define KB_PORT_ESP32_TEST_SUPPORT_HPP

#include <array>
#include <cstdint>
#include <span>

#include "fake_esp_idf.hpp"
#include "kb/port/esp32/port.hpp"
#include "kb_test.hpp"

namespace esp = kb::port::esp32;
using kb::Status;

// Every test starts from a fresh fake (pins high, nothing installed, no injection).
inline void Fresh() {
  fake::Reset();
}

// True when `r` failed with `s` and vendor code `err`.
template <typename R>
[[nodiscard]] bool FailedWith(const R& r, Status s, int err) {
  return !r.IsOk() && (r.Error() == s) && (r.Detail() == static_cast<uint32_t>(err));
}

#endif
