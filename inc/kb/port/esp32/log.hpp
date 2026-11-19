/**
 * @file log.hpp
 * @brief `kb::port::esp32::LogSink`: lines to the ESP-IDF log (`esp_log_write`) under one tag;
 * satisfies `kb::iface::LogSink`.
 *
 * A line longer than kLogMaxLine characters is cut. Nothing is allocated; the call takes as long as
 * the console needs for one line. The tag must be a string literal (ESP-IDF keeps the pointer); a
 * null tag is written as "kb". Task context only: `esp_log_write` takes a lock, so never call
 * Write() from an interrupt handler.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_LOG_HPP
#define KB_PORT_ESP32_LOG_HPP

#include <cstddef>
// The LogSink contract takes a std::string_view (kb/interfaces/log.hpp).
// misra-lint: ignore KB-CPP-012
#include <string_view>

#include "kb/interfaces/log.hpp"
#include "kb/interfaces/types.hpp"

namespace kb::port::esp32 {

/** @brief The longest line written; longer text is cut. */
inline constexpr std::size_t kLogMaxLine = 160U;

/** @brief Log lines under one tag; satisfies kb::iface::LogSink. */
class LogSink {
public:
  /** @brief Writes under `tag`, which must be a string literal. */
  explicit constexpr LogSink(const char* tag) noexcept : tag_(tag) {}
  /** @brief Writes one line at `level`. */
  void Write(kb::iface::LogLevel level, std::string_view text) noexcept;
  /** @brief The tag. */
  [[nodiscard]] const char* Tag() const noexcept {
    return tag_;
  }

private:
  const char* tag_;
};

static_assert(kb::iface::LogSink<LogSink>, "LogSink satisfies kb::iface::LogSink");

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_LOG_HPP
