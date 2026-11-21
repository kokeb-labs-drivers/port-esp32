// Boundary file (deviations/boundary-files.yml): the one-shot ADC, calibration and continuous-mode
// calls of ESP-IDF v5.5 (component esp_adc) behind sdk.hpp. Each function converts its arguments,
// calls the SDK and returns its esp_err_t; the SDK handles live in the tables below. Chip
// differences use the SDK's capability macros (soc/soc_caps.h, esp_adc/adc_cali_scheme.h) here and
// nowhere else. The two continuous-mode callbacks run in the SDK's interrupt and are IRAM_ATTR;
// what they touch (the stream table, the port's stream object) is in internal RAM.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "esp_private/esp_cache_private.h"
#include "esp_timer.h"
#include "kb/port/esp32/adc.hpp"
#include "kb/port/esp32/adc_stream.hpp"
#include "sdk.hpp"
#include "sdkconfig.h"
#include "soc/soc.h"
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

// --- ADC continuous
// --------------------------------------------------------------------------------

namespace {
// adc_continuous.c of v5.5: INTERNAL_BUF_NUM, the frame buffers its DMA ring cycles through.
constexpr uint32_t kSdkFrameBuffers = 5U;
// adc_continuous_read() calls per drain: the pool holds at most kAdcStreamMaxPoolFrames frames.
constexpr uint32_t kDrainReads = kAdcStreamMaxPoolFrames + 1U;
constexpr std::size_t kDrainBytes = 256U;

// The one continuous driver of the chip and the port stream it feeds (both in internal RAM).
struct AdcStreamTable {
  adc_continuous_handle_t handle{nullptr};
  AdcStreamSink* sink{nullptr};
};

IRAM_ATTR AdcStreamTable& StreamTable() noexcept {
  // codeql::misra_deviation_next_line(DEV-001)
  static AdcStreamTable table{};
  return table;
}

// on_conv_done: the frame goes to the port at once, with the time read here; the SDK's buffer
// pointer is not kept.
IRAM_ATTR bool OnConvDone(adc_continuous_handle_t /*handle*/,
                          const adc_continuous_evt_data_t* edata, void* user_data) {
  static_cast<void>(user_data);
  const uint32_t start = esp_cpu_get_cycle_count();
  AdcStreamSink* const sink = StreamTable().sink;
  if ((sink != nullptr) && (edata != nullptr)) {
    const auto at = static_cast<uint64_t>(esp_timer_get_time());
    // The entry point returns nothing (the lint matches the member name `frame` only).
    // misra-lint: ignore 0.1.2
    sink->frame(*sink, std::span<const uint8_t>(edata->conv_frame_buffer, edata->size), at);
    sink->cost(*sink, esp_cpu_get_cycle_count() - start);
  }
  return false;
}

// on_pool_ovf: a separate callback; the SDK passes no data with it, and the port publishes none.
IRAM_ATTR bool OnPoolOverflow(adc_continuous_handle_t /*handle*/,
                              const adc_continuous_evt_data_t* edata, void* user_data) {
  static_cast<void>(edata);
  static_cast<void>(user_data);
  AdcStreamSink* const sink = StreamTable().sink;
  if (sink != nullptr) {
    sink->overflow(*sink, static_cast<uint64_t>(esp_timer_get_time()));
  }
  return false;
}

// esp_ptr_in_iram()'s test on the address of a function (a function pointer is not an object
// pointer, so it is compared as an integer).
template <typename Fn>
[[nodiscard]] bool CodeInIram(Fn fn) noexcept {
  const auto address = std::bit_cast<std::uintptr_t>(fn);
  return (address >= static_cast<std::uintptr_t>(SOC_IRAM_LOW)) &&
         (address < static_cast<std::uintptr_t>(SOC_IRAM_HIGH));
}

[[nodiscard]] adc_digi_convert_mode_t ConvMode(uint8_t unit) noexcept {
  return (unit == 0U) ? ADC_CONV_SINGLE_UNIT_1 : ADC_CONV_SINGLE_UNIT_2;
}

[[nodiscard]] adc_digi_output_format_t OutputFormat() noexcept {
#if (SOC_ADC_DIGI_RESULT_BYTES == 2) && !defined(CONFIG_IDF_TARGET_ESP32S2)
  return ADC_DIGI_OUTPUT_FORMAT_TYPE1;
#else
  return ADC_DIGI_OUTPUT_FORMAT_TYPE2;
#endif
}

[[nodiscard]] int32_t AdcStreamConfigure(adc_continuous_handle_t handle,
                                         const AdcStreamSetup& setup) noexcept {
  std::array<adc_digi_pattern_config_t, SOC_ADC_PATT_LEN_MAX> pattern{};
  esp_err_t err = (setup.channels.empty() || (setup.channels.size() > pattern.size()))
                      ? ESP_ERR_INVALID_ARG
                      : ESP_OK;
  for (std::size_t i = 0U; (err == ESP_OK) && (i < setup.channels.size()); ++i) {
    pattern[i].atten = static_cast<uint8_t>(setup.attenuation); // the values of adc_atten_t
    pattern[i].channel = setup.channels[i].channel;
    pattern[i].unit = setup.channels[i].unit;
    pattern[i].bit_width = static_cast<uint8_t>(SOC_ADC_DIGI_MAX_BITWIDTH);
  }
  if (err == ESP_OK) {
    adc_continuous_config_t config{};
    config.pattern_num = static_cast<uint32_t>(setup.channels.size());
    config.adc_pattern = pattern.data();
    config.sample_freq_hz = setup.sample_hz;
    config.conv_mode = ConvMode(setup.channels[0].unit);
    config.format = OutputFormat();
    err = adc_continuous_config(handle, &config);
  }
  return err;
}
} // namespace

int32_t AdcStreamOpen(const AdcStreamSetup& setup, AdcStreamSink& sink) noexcept {
  AdcStreamTable& table = StreamTable();
  esp_err_t err = ESP_OK;
  if (table.handle != nullptr) {
    err = ESP_ERR_NOT_ALLOWED; // the chip has one continuous driver
  } else if (setup.iram_safe) {
#if defined(CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE)
    const bool resident = CodeInIram(sink.frame) && CodeInIram(sink.overflow) &&
                          CodeInIram(sink.cost) && esp_ptr_internal(&sink);
    err = resident ? ESP_OK : ESP_ERR_INVALID_ARG;
#else
    err = ESP_ERR_NOT_SUPPORTED; // the board writes flash while streaming: IRAM-safe ISR needed
#endif
  } else {
    err = ESP_OK;
  }
  adc_continuous_handle_t handle = nullptr;
  if (err == ESP_OK) {
    adc_continuous_handle_cfg_t config{};
    config.max_store_buf_size = static_cast<uint32_t>(setup.pool_bytes);
    config.conv_frame_size = static_cast<uint32_t>(setup.frame_bytes);
    config.flags.flush_pool = 0U; // a full pool is an overflow, never a silent drop
    err = adc_continuous_new_handle(&config, &handle);
  }
  if (err == ESP_OK) {
    err = AdcStreamConfigure(handle, setup);
  }
  if (err == ESP_OK) {
    table.sink = &sink;
    adc_continuous_evt_cbs_t callbacks{};
    callbacks.on_conv_done = &OnConvDone;
    callbacks.on_pool_ovf = &OnPoolOverflow;
    err = adc_continuous_register_event_callbacks(handle, &callbacks, nullptr);
  }
  if (err == ESP_OK) {
    table.handle = handle;
  } else {
    table.sink = nullptr;
    if (handle != nullptr) {
      static_cast<void>(adc_continuous_deinit(handle));
    }
  }
  return err;
}

int32_t AdcStreamStart() noexcept {
  return adc_continuous_start(StreamTable().handle);
}

int32_t AdcStreamStop() noexcept {
  return adc_continuous_stop(StreamTable().handle);
}

int32_t AdcStreamClose() noexcept {
  AdcStreamTable& table = StreamTable();
  const esp_err_t err = adc_continuous_deinit(table.handle);
  if (err == ESP_OK) {
    table.handle = nullptr;
    table.sink = nullptr;
  }
  return err;
}

int32_t AdcStreamDrainPool() noexcept {
  std::array<uint8_t, kDrainBytes> scratch{};
  esp_err_t err = ESP_OK;
  uint32_t got = 1U;
  for (uint32_t i = 0U; (i < kDrainReads) && (err == ESP_OK) && (got != 0U); ++i) {
    // Timeout 0: no wait at all (the bytes were already copied by the interrupt).
    err = adc_continuous_read(StreamTable().handle, scratch.data(),
                              static_cast<uint32_t>(scratch.size()), &got, 0U);
  }
  return (err == ESP_ERR_TIMEOUT) ? ESP_OK : err;
}

std::size_t AdcResultBytes() noexcept {
  return static_cast<std::size_t>(SOC_ADC_DIGI_RESULT_BYTES);
}

std::size_t AdcConvBytes() noexcept {
  return static_cast<std::size_t>(SOC_ADC_DIGI_DATA_BYTES_PER_CONV);
}

std::size_t AdcFrameUnit() noexcept {
  std::size_t cache = 0U;
  if (esp_cache_get_alignment(MALLOC_CAP_DMA, &cache) != ESP_OK) {
    cache = 0U;
  }
  const std::size_t conv = AdcConvBytes();
  return (cache > conv) ? cache : conv;
}

uint32_t AdcSdkFrameBuffers() noexcept {
  return kSdkFrameBuffers;
}

uint32_t AdcStreamMinHz() noexcept {
  return static_cast<uint32_t>(SOC_ADC_SAMPLE_FREQ_THRES_LOW);
}

uint32_t AdcStreamMaxHz() noexcept {
  return static_cast<uint32_t>(SOC_ADC_SAMPLE_FREQ_THRES_HIGH);
}

// The bit layout of one conversion result (adc_digi_output_data_t of the chip's output format, as
// hal/adc_types.h declares it), read and written as a little-endian integer.
namespace {
#if (SOC_ADC_DIGI_RESULT_BYTES == 2) && !defined(CONFIG_IDF_TARGET_ESP32S2)
constexpr uint32_t kDataMask = 0xFFFU; // type 1: data 12, channel 4
constexpr uint32_t kChannelShift = 12U;
constexpr uint32_t kChannelMask = 0xFU;
constexpr uint32_t kUnitShift = 0U; // no unit field
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
constexpr uint32_t kDataMask = 0x7FFU; // type 2: data 11, channel 4, unit 1
constexpr uint32_t kChannelShift = 11U;
constexpr uint32_t kChannelMask = 0xFU;
constexpr uint32_t kUnitShift = 15U;
#elif defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32H2) ||                  \
    defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C61)
constexpr uint32_t kDataMask = 0xFFFU; // type 2: data 12, reserved 1, channel 4
constexpr uint32_t kChannelShift = 13U;
constexpr uint32_t kChannelMask = 0xFU;
constexpr uint32_t kUnitShift = 0U;
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C2)
constexpr uint32_t kDataMask = 0xFFFU; // type 2: data 12, reserved 1, channel 3, unit 1
constexpr uint32_t kChannelShift = 13U;
constexpr uint32_t kChannelMask = 0x7U;
constexpr uint32_t kUnitShift = 16U;
#else
constexpr uint32_t kDataMask = 0xFFFU; // type 2 (S3, P4): data 12, reserved 1, channel 4, unit 1
constexpr uint32_t kChannelShift = 13U;
constexpr uint32_t kChannelMask = 0xFU;
constexpr uint32_t kUnitShift = 17U;
#endif
static_assert(sizeof(adc_digi_output_data_t) == SOC_ADC_DIGI_RESULT_BYTES);
} // namespace

bool AdcResultDecode(std::span<const uint8_t> result, AdcSample& out) noexcept {
  bool ok = result.size() == static_cast<std::size_t>(SOC_ADC_DIGI_RESULT_BYTES);
  if (ok) {
    uint32_t v = 0U;
    for (std::size_t i = result.size(); i > 0U; --i) {
      v = (v << 8U) | result[i - 1U];
    }
    out.raw = static_cast<uint16_t>(v & kDataMask);
    out.channel = static_cast<uint8_t>((v >> kChannelShift) & kChannelMask);
    out.unit = (kUnitShift == 0U) ? uint8_t{0U} : static_cast<uint8_t>((v >> kUnitShift) & 1U);
    ok = out.channel <
         static_cast<uint8_t>(SOC_ADC_MAX_CHANNEL_NUM); // the arbiter marks invalid data
  }
  return ok;
}

bool AdcResultEncode(const AdcSample& sample, std::span<uint8_t> out) noexcept {
  const bool ok = (out.size() == static_cast<std::size_t>(SOC_ADC_DIGI_RESULT_BYTES)) &&
                  (sample.channel < static_cast<uint8_t>(SOC_ADC_MAX_CHANNEL_NUM));
  if (ok) {
    uint32_t v = (uint32_t{sample.raw} & kDataMask) |
                 ((uint32_t{sample.channel} & kChannelMask) << kChannelShift);
    if (kUnitShift != 0U) {
      v |= (uint32_t{sample.unit} & 1U) << kUnitShift;
    }
    for (uint8_t& byte : out) {
      byte = static_cast<uint8_t>(v & 0xFFU);
      v >>= 8U;
    }
  }
  return ok;
}

} // namespace kb::port::esp32::sdk
