// LogSink over the fake esp_log. Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <string>
#include <string_view>

#include "support.hpp"

KB_TEST("log: each level reaches esp_log_write under the tag, cut at the limit") {
  Fresh();
  esp::LogSink log{"kb"};
  KB_CHECK(std::string_view{log.Tag()} == "kb");
  log.Write(kb::iface::LogLevel::Error, "bus down");
  KB_CHECK(fake::LastLogLevel() == ESP_LOG_ERROR);
  KB_CHECK(fake::LastLog().find("E (") == 0U);
  KB_CHECK(fake::LastLog().find("kb: bus down\n") != std::string::npos);
  log.Write(kb::iface::LogLevel::Warning, "w");
  KB_CHECK(fake::LastLogLevel() == ESP_LOG_WARN);
  log.Write(kb::iface::LogLevel::Info, "i");
  KB_CHECK(fake::LastLogLevel() == ESP_LOG_INFO);
  log.Write(kb::iface::LogLevel::Debug, "d");
  KB_CHECK(fake::LastLogLevel() == ESP_LOG_DEBUG);
  KB_CHECK(fake::LastLog().find("D (") == 0U);
  const std::string long_line(400U, 'x');
  log.Write(kb::iface::LogLevel::Info, long_line);
  const std::size_t shown = fake::LastLog().size() - fake::LastLog().find("kb: ") - 5U;
  KB_CHECK_EQ(shown, esp::kLogMaxLine);
}

KB_TEST("log: a null tag is written as kb, never passed to esp_log_write") {
  Fresh();
  esp::LogSink log{nullptr};
  log.Write(kb::iface::LogLevel::Warning, "no tag");
  KB_CHECK(fake::LastLog().find("kb: no tag\n") != std::string::npos);
}
