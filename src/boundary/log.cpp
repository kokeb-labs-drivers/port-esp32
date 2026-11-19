// Boundary file (deviations/boundary-files.yml): the log call of ESP-IDF v5.5 behind sdk.hpp.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <cstdint>
#include <span>

#include "esp_log.h"
#include "kb/interfaces/types.hpp"
#include "sdk.hpp"

namespace kb::port::esp32::sdk {

namespace {
[[nodiscard]] esp_log_level_t LevelOf(kb::iface::LogLevel level) noexcept {
  esp_log_level_t out = ESP_LOG_DEBUG;
  if (level == kb::iface::LogLevel::Error) {
    out = ESP_LOG_ERROR;
  } else if (level == kb::iface::LogLevel::Warning) {
    out = ESP_LOG_WARN;
  } else if (level == kb::iface::LogLevel::Info) {
    out = ESP_LOG_INFO;
  } else {
    out = ESP_LOG_DEBUG;
  }
  return out;
}

[[nodiscard]] const char* LetterOf(kb::iface::LogLevel level) noexcept {
  const char* letter = nullptr;
  if (level == kb::iface::LogLevel::Error) {
    letter = "E";
  } else if (level == kb::iface::LogLevel::Warning) {
    letter = "W";
  } else if (level == kb::iface::LogLevel::Info) {
    letter = "I";
  } else {
    letter = "D";
  }
  return letter;
}
} // namespace

void LogWrite(kb::iface::LogLevel level, const char* tag, std::span<const char> text) noexcept {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): DEV-002, the SDK's log call is variadic
  esp_log_write(LevelOf(level), tag, "%s (%lu) %s: %.*s\n", LetterOf(level),
                static_cast<unsigned long>(esp_log_timestamp()), tag, static_cast<int>(text.size()),
                text.data());
}

} // namespace kb::port::esp32::sdk
