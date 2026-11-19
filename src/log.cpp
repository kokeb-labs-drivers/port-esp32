// LogSink: cuts a line to kLogMaxLine and forwards it. Platform-free; every SDK call goes through
// sdk.hpp (src/boundary/log.cpp).
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <algorithm>
#include <cstddef>
#include <span>
// The LogSink contract takes a std::string_view (kb/interfaces/log.hpp).
// misra-lint: ignore KB-CPP-012
#include <string_view>

#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/log.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

void LogSink::Write(kb::iface::LogLevel level, std::string_view text) noexcept {
  const std::size_t n = std::min(text.size(), kLogMaxLine);
  const char* const tag = (tag_ != nullptr) ? tag_ : "kb"; // never a null tag to the SDK
  sdk::LogWrite(level, tag, std::span<const char>{text.begin(), n});
}

} // namespace kb::port::esp32
