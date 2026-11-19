/**
 * @file adc.hpp
 * @brief `kb::port::esp32::AdcChannel`: one ADC input (ESP-IDF v5.5 `adc_oneshot` with
 * `adc_cali`), which satisfies `kb::iface::AdcChannel`.
 *
 * The pin selects the unit and channel (`adc_oneshot_io_to_channel`). The unit is shared by every
 * channel object on it: the port counts its users and opens, reads and closes it only under the
 * unit's lock (at most `AdcConfig::lock_timeout`), so a Release() never deletes a unit another
 * channel is reading. ReadMillivolts() uses the chip's eFuse calibration through the scheme the
 * chip has: curve fitting (ESP32-S3, C3, C6, H2, P4) or line fitting (ESP32, ESP32-S2). An ESP32
 * without a Vref in eFuse needs `AdcConfig::default_vref_mv` (the board's measured Vref, about
 * 1100 mV); without calibration data ReadMillivolts() returns `Unsupported` and ReadRaw() still
 * works. LastError() reports the calling task's own last call.
 *
 * | Result | Meaning |
 * | --- | --- |
 * | `Ok` | One conversion |
 * | `Timeout` | The conversion did not finish (ADC2 held by the radio on ESP32 and ESP32-S3) |
 * | `Unsupported` | ReadMillivolts(): no calibration scheme or no eFuse data |
 * | `InvalidArgument` | The pin is not an ADC pin |
 * | `Busy` | The unit's lock was not had in time (another channel of the unit is reading), or the
 * pin or a channel slot is taken | | `NotReady` | EnsureReady() has not succeeded |
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_ADC_HPP
#define KB_PORT_ESP32_ADC_HPP

// Lock-free claims, counters and flags shared between tasks (deviations/deviations.yml, DEV-003).
// misra-lint: ignore KB-CPP-012
#include <atomic>
#include <cstdint>

#include "kb/interfaces/adc.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"

namespace kb::port::esp32 {

/** @brief Input attenuation: the range the ADC measures. */
enum class AdcAttenuation : uint8_t {
  Db0 = 0,   ///< 0 dB: about 0 to 0.95 V.
  Db2p5 = 1, ///< 2.5 dB: about 0 to 1.25 V.
  Db6 = 2,   ///< 6 dB: about 0 to 1.75 V.
  Db12 = 3,  ///< 12 dB: about 0 to 3.1 V (chip dependent).
};

/** @brief One ADC input. */
struct AdcConfig {
  int16_t pin{-1};                                  ///< GPIO with an ADC function.
  AdcAttenuation attenuation{AdcAttenuation::Db12}; ///< Range.
  kb::Milliseconds lock_timeout{
      kb::Milliseconds{10U}}; ///< Wait for the shared unit (to kMaxTimeout).
  uint16_t default_vref_mv{
      0U}; ///< ESP32 line fitting without eFuse Vref: the Vref in mV (0: none).
};

/** @brief One ADC input; satisfies kb::iface::AdcChannel. */
class AdcChannel {
public:
  /** @brief Keeps `config`. */
  explicit AdcChannel(const AdcConfig& config) noexcept : config_(config) {}
  AdcChannel(const AdcChannel&) = delete;
  AdcChannel& operator=(const AdcChannel&) = delete;
  AdcChannel(AdcChannel&&) = delete;
  AdcChannel& operator=(AdcChannel&&) = delete;
  /** @brief Releases the channel. */
  ~AdcChannel() noexcept;

  /** @brief Opens the unit (shared), configures the channel and its calibration; idempotent. */
  [[nodiscard]] kb::Status EnsureReady() noexcept;
  /** @brief One conversion, in counts (0 to FullScale()). Detail(): the esp_err_t. */
  [[nodiscard]] kb::Result<uint32_t> ReadRaw() noexcept;
  /** @brief One conversion through the eFuse calibration, in millivolts. */
  [[nodiscard]] kb::Result<int32_t> ReadMillivolts() noexcept;
  /** @brief The largest count: 2^bits - 1 of the chip's ADC (4095 for 12 bits). */
  [[nodiscard]] uint32_t FullScale() const noexcept {
    return full_scale_;
  }
  /** @brief True when ReadMillivolts() has calibration data. */
  [[nodiscard]] bool Calibrated() const noexcept {
    return calibrated_.load();
  }
  /** @brief Frees the calibration, and the unit when no other channel uses it (under the unit's
   * lock; Busy when not had in time). */
  [[nodiscard]] kb::Status Release() noexcept;
  /** @brief The calling task's last call: status and its esp_err_t in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return error_.Get();
  }

private:
  [[nodiscard]] kb::Status openLocked() noexcept;

  AdcConfig config_;
  CallerRecord error_;
  uint32_t full_scale_{0U};
  uint8_t unit_{0U};
  uint8_t channel_{0U};
  uint8_t slot_{0U};
  std::atomic<bool> calibrated_{false};
  std::atomic<bool> ready_{false};
};

static_assert(kb::iface::AdcChannel<AdcChannel>, "AdcChannel satisfies kb::iface::AdcChannel");

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_ADC_HPP
