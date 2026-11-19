// Boundary file (deviations/boundary-files.yml): checks the esp_err_t values that
// kb/port/esp32/error.hpp names against ESP-IDF v5.5, so a change in the SDK stops the build. No
// code. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include "kb/port/esp32/error.hpp"
#include "esp_err.h"

namespace kb::port::esp32::sdk {

static_assert(esp_err::kOk == ESP_OK);
static_assert(esp_err::kFail == ESP_FAIL);
static_assert(esp_err::kNoMem == ESP_ERR_NO_MEM);
static_assert(esp_err::kInvalidArg == ESP_ERR_INVALID_ARG);
static_assert(esp_err::kInvalidState == ESP_ERR_INVALID_STATE);
static_assert(esp_err::kInvalidSize == ESP_ERR_INVALID_SIZE);
static_assert(esp_err::kNotFound == ESP_ERR_NOT_FOUND);
static_assert(esp_err::kNotSupported == ESP_ERR_NOT_SUPPORTED);
static_assert(esp_err::kTimeout == ESP_ERR_TIMEOUT);
static_assert(esp_err::kInvalidResponse == ESP_ERR_INVALID_RESPONSE);
static_assert(esp_err::kInvalidCrc == ESP_ERR_INVALID_CRC);
static_assert(esp_err::kNotFinished == ESP_ERR_NOT_FINISHED);
static_assert(esp_err::kNotAllowed == ESP_ERR_NOT_ALLOWED);

} // namespace kb::port::esp32::sdk
