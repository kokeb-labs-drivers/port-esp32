/**
 * @file version.hpp
 * @brief The version of the ESP32 port (CMakeLists.txt and idf_component.yml carry the same).
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_VERSION_HPP
#define KB_PORT_ESP32_VERSION_HPP

#include <cstdint>

namespace kb::port::esp32 {

inline constexpr uint32_t kVersionMajor = 0U; ///< Major version.
inline constexpr uint32_t kVersionMinor = 1U; ///< Minor version.
inline constexpr uint32_t kVersionPatch = 0U; ///< Patch version.
/** @brief The version as text. */
inline constexpr const char* kVersion = "0.1.0";
/** @brief The ESP-IDF release line this port is written and tested for. */
inline constexpr const char* kSdkLine = "ESP-IDF v5.5";

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_VERSION_HPP
