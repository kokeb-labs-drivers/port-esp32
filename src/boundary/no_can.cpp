// Boundary file (deviations/boundary-files.yml): the CAN seam for chips without a TWAI controller
// (CMakeLists.txt picks it, instead of can.cpp, when SOC_TWAI_SUPPORTED is not set). Every call
// reports ESP_ERR_NOT_SUPPORTED, which the port maps to kb::Status::Unsupported, and touches
// nothing. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include <cstdint>

#include "esp_err.h"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/can.hpp"
#include "sdk.hpp"

namespace kb::port::esp32::sdk {

// The chip has no TWAI controller: every call reports ESP_ERR_NOT_SUPPORTED
// (kb::Status::Unsupported).
uint8_t TwaiCount() noexcept {
  return 0U;
}
int32_t TwaiOpen([[maybe_unused]] const CanConfig& config) noexcept {
  return ESP_ERR_NOT_SUPPORTED;
}
int32_t TwaiClose([[maybe_unused]] uint8_t controller) noexcept {
  return ESP_ERR_NOT_SUPPORTED;
}
int32_t TwaiSend([[maybe_unused]] uint8_t controller,
                 [[maybe_unused]] const kb::iface::CanFrame& frame,
                 [[maybe_unused]] uint32_t timeout_ms) noexcept {
  return ESP_ERR_NOT_SUPPORTED;
}
int32_t TwaiReceive([[maybe_unused]] uint8_t controller,
                    [[maybe_unused]] kb::iface::CanFrame& frame,
                    [[maybe_unused]] uint32_t timeout_ms) noexcept {
  return ESP_ERR_NOT_SUPPORTED;
}
int32_t TwaiStatus([[maybe_unused]] uint8_t controller,
                   [[maybe_unused]] CanStatus& status) noexcept {
  return ESP_ERR_NOT_SUPPORTED;
}
int32_t TwaiRecover([[maybe_unused]] uint8_t controller) noexcept {
  return ESP_ERR_NOT_SUPPORTED;
}
int32_t TwaiStart([[maybe_unused]] uint8_t controller) noexcept {
  return ESP_ERR_NOT_SUPPORTED;
}
} // namespace kb::port::esp32::sdk
