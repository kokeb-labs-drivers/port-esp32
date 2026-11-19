// Boundary file (deviations/boundary-files.yml): the one-shot ADC and calibration calls of ESP-IDF
// v5.5 behind sdk.hpp. Each function converts its arguments, calls the SDK and returns its
// esp_err_t; the SDK handles live in the tables below. Chip differences use the SDK's capability
// macros (soc/soc_caps.h, esp_adc/adc_cali_scheme.h) here and nowhere else.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <array>
#include <cstddef>
#include <cstdint>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "kb/port/esp32/adc.hpp"
#include "sdk.hpp"
#include "soc/soc_caps.h"

namespace kb::port::esp32::sdk {

namespace {
constexpr std::size_t kAdcSlots = 16U;

// SDK handles only: the users of a unit, its lock and the slot choice are kept by the logic
// (src/adc_pwm.cpp).
struct AdcSlot {
  uint8_t unit{0U};
  adc_channel_t channel{};
  adc_atten_t atten{};
  adc_cali_handle_t cali{nullptr};
};

std::array<adc_oneshot_unit_handle_t, SOC_ADC_PERIPH_NUM>& AdcUnits() noexcept {
  // codeql::misra_deviation_next_line(DEV-001)
  static std::array<adc_oneshot_unit_handle_t, SOC_ADC_PERIPH_NUM> units{};
  return units;
}

std::array<AdcSlot, kAdcSlots>& AdcSlots() noexcept {
  // codeql::misra_deviation_next_line(DEV-001)
  static std::array<AdcSlot, kAdcSlots> slots{};
  return slots;
}

[[nodiscard]] adc_atten_t AttenOf(AdcAttenuation attenuation) noexcept {
  adc_atten_t atten = ADC_ATTEN_DB_12;
  if (attenuation == AdcAttenuation::Db0) {
    atten = ADC_ATTEN_DB_0;
  } else if (attenuation == AdcAttenuation::Db2p5) {
    atten = ADC_ATTEN_DB_2_5;
  } else if (attenuation == AdcAttenuation::Db6) {
    atten = ADC_ATTEN_DB_6;
  } else {
    atten = ADC_ATTEN_DB_12;
  }
  return atten;
}

// The number of a channel, found by comparison (the enumeration has no fixed underlying type).
[[nodiscard]] uint8_t ChannelNumber(adc_channel_t ch) noexcept {
  constexpr uint8_t kChannels = 10U;
  uint8_t number = 0U;
  for (uint8_t i = 0U; i < kChannels; ++i) {
    if (ch == static_cast<adc_channel_t>(i)) {
      number = i;
    }
  }
  return number;
}

[[nodiscard]] adc_unit_t UnitOf(uint8_t unit) noexcept {
  return static_cast<adc_unit_t>(unit);
}

[[nodiscard]] int32_t CaliCreate(AdcSlot& slot,
                                 [[maybe_unused]] uint16_t default_vref_mv) noexcept {
  esp_err_t err = ESP_ERR_NOT_SUPPORTED;
#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
  adc_cali_curve_fitting_config_t config{};
  config.unit_id = UnitOf(slot.unit);
  config.chan = slot.channel;
  config.atten = slot.atten;
  config.bitwidth = ADC_BITWIDTH_DEFAULT;
  err = adc_cali_create_scheme_curve_fitting(&config, &slot.cali);
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
  adc_cali_line_fitting_config_t config{};
  config.unit_id = UnitOf(slot.unit);
  config.atten = slot.atten;
  config.bitwidth = ADC_BITWIDTH_DEFAULT;
#if defined(CONFIG_IDF_TARGET_ESP32)
  config.default_vref = default_vref_mv; // used only on chips without a Vref in eFuse
#endif
  err = adc_cali_create_scheme_line_fitting(&config, &slot.cali);
#endif
  if (err != ESP_OK) {
    slot.cali = nullptr;
  }
  return err;
}

[[nodiscard]] int32_t CaliDelete(AdcSlot& slot) noexcept {
  esp_err_t err = ESP_OK;
  if (slot.cali != nullptr) {
#if defined(ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
    err = adc_cali_delete_scheme_curve_fitting(slot.cali);
#elif defined(ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
    err = adc_cali_delete_scheme_line_fitting(slot.cali);
#endif
    slot.cali = nullptr;
  }
  return err;
}
} // namespace

uint8_t AdcBits() noexcept {
  return static_cast<uint8_t>(SOC_ADC_RTC_MAX_BITWIDTH);
}

int32_t AdcPinChannel(int16_t pin, uint8_t& unit, uint8_t& channel) noexcept {
  adc_unit_t u{};
  adc_channel_t ch{};
  const esp_err_t err = (pin < 0) ? ESP_ERR_INVALID_ARG : adc_oneshot_io_to_channel(pin, &u, &ch);
  if (err == ESP_OK) {
    unit = (u == ADC_UNIT_1) ? 0U : 1U;
    channel = ChannelNumber(ch);
  }
  return err;
}

uint8_t AdcUnitCount() noexcept {
  return static_cast<uint8_t>(SOC_ADC_PERIPH_NUM);
}

uint8_t AdcSlotCount() noexcept {
  return static_cast<uint8_t>(kAdcSlots);
}

int32_t AdcUnitOpen(uint8_t unit) noexcept {
  adc_oneshot_unit_init_cfg_t config{};
  config.unit_id = UnitOf(unit);
  config.ulp_mode = ADC_ULP_MODE_DISABLE;
  const esp_err_t err = adc_oneshot_new_unit(&config, &AdcUnits()[unit]);
  if (err != ESP_OK) {
    AdcUnits()[unit] = nullptr;
  }
  return err;
}

int32_t AdcUnitClose(uint8_t unit) noexcept {
  esp_err_t err = ESP_OK;
  if (AdcUnits()[unit] != nullptr) {
    err = adc_oneshot_del_unit(AdcUnits()[unit]);
    AdcUnits()[unit] = nullptr;
  }
  return err;
}

int32_t AdcChannelOpen(uint8_t slot, uint8_t unit, uint8_t channel,
                       AdcAttenuation attenuation) noexcept {
  AdcSlot& s = AdcSlots()[slot];
  s.channel = static_cast<adc_channel_t>(channel);
  s.unit = unit;
  s.atten = AttenOf(attenuation);
  s.cali = nullptr;
  adc_oneshot_chan_cfg_t config{};
  config.atten = s.atten;
  config.bitwidth = ADC_BITWIDTH_DEFAULT;
  return adc_oneshot_config_channel(AdcUnits()[unit], s.channel, &config);
}

int32_t AdcCalibrate(uint8_t slot, uint16_t default_vref_mv) noexcept {
  return CaliCreate(AdcSlots()[slot], default_vref_mv);
}

int32_t AdcChannelClose(uint8_t slot) noexcept {
  return CaliDelete(AdcSlots()[slot]);
}

int32_t AdcRead(uint8_t slot, int32_t& raw) noexcept {
  const AdcSlot& s = AdcSlots()[slot];
  // codeql::misra_deviation_next_line(DEV-004)
  int value = 0;
  const esp_err_t err = adc_oneshot_read(AdcUnits()[s.unit], s.channel, &value);
  raw = static_cast<int32_t>(value);
  return err;
}

int32_t AdcToMillivolts(uint8_t slot, int32_t raw, int32_t& mv) noexcept {
  // codeql::misra_deviation_next_line(DEV-004)
  int value = 0;
  const esp_err_t err =
      adc_cali_raw_to_voltage(AdcSlots()[slot].cali, static_cast<int>(raw), &value);
  mv = static_cast<int32_t>(value);
  return err;
}

} // namespace kb::port::esp32::sdk
