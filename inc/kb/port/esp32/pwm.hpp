/**
 * @file pwm.hpp
 * @brief `kb::port::esp32::PwmChannel`: one LEDC channel (ESP-IDF v5.5 `ledc`, low-speed mode),
 * which satisfies `kb::iface::PwmOutput`.
 *
 * Each channel uses one LEDC timer. The port keeps one record per timer (frequency, resolution,
 * users): channels may share a timer only at one frequency, so a second channel at another
 * frequency, or a SetFrequencyHz() that would change the frequency under another channel, is
 * refused with `Busy`, and no channel is ever left with a compare value for another resolution.
 * The duty resolution is the finest the timer's clock allows at the frequency (up to
 * SOC_LEDC_TIMER_BIT_WIDTH bits); the duty is rounded to it and clamped to the period
 * (2^bits counts is 100 %, the top of ESP-IDF's range). Enable(false) stops the channel at its idle
 * level; the duty and frequency are kept for the next Enable(true).
 *
 * An idle-high channel is configured with its output inverted, so the pin shows high from the
 * moment LEDC takes it (no low pulse at start-up); the duty is converted, so SetDuty() keeps
 * meaning the high time. One object per channel, used from one context at a time.
 *
 * | Result | Meaning |
 * | --- | --- |
 * | `Ok` | Done |
 * | `InvalidArgument` | Frequency 0 or too high for 1 bit; bad channel, timer or pin |
 * | `Busy` | The timer runs another channel at another frequency, or is being changed; or the pin
 * is taken | | `NotReady` | EnsureReady() has not succeeded |
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_PWM_HPP
#define KB_PORT_ESP32_PWM_HPP

#include <cstdint>

#include "kb/interfaces/pwm.hpp"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"

namespace kb::port::esp32 {

/** @brief One PWM output. */
struct PwmConfig {
  int16_t pin{-1};              ///< GPIO.
  uint8_t channel{0U};          ///< LEDC channel, 0 to SOC_LEDC_CHANNEL_NUM - 1.
  uint8_t timer{0U};            ///< LEDC timer, 0 to 3.
  uint32_t frequency_hz{1000U}; ///< Initial frequency.
  bool idle_high{false};        ///< Level while disabled.
};

/** @brief One LEDC channel; satisfies kb::iface::PwmOutput. */
class PwmChannel {
public:
  /** @brief Keeps `config`. */
  explicit PwmChannel(const PwmConfig& config) noexcept : config_(config) {}
  PwmChannel(const PwmChannel&) = delete;
  PwmChannel& operator=(const PwmChannel&) = delete;
  PwmChannel(PwmChannel&&) = delete;
  PwmChannel& operator=(PwmChannel&&) = delete;
  /** @brief Stops the channel. */
  ~PwmChannel() noexcept;

  /** @brief Configures the timer and the channel, stopped at the idle level; idempotent. */
  [[nodiscard]] kb::Status EnsureReady() noexcept;
  /** @brief The period; the duty is kept as a fraction. */
  [[nodiscard]] kb::Status SetFrequencyHz(uint32_t hz) noexcept;
  /** @brief High time per period, 0 to 100 %. Applied at once when enabled. */
  [[nodiscard]] kb::Status SetDuty(kb::iface::Duty duty) noexcept;
  /** @brief Starts or stops the output (stopped: the idle level). */
  [[nodiscard]] kb::Status Enable(bool on) noexcept;
  /** @brief The duty resolution in bits of the channel's timer now (0 before EnsureReady()). */
  [[nodiscard]] uint8_t ResolutionBits() const noexcept;
  /** @brief The compare value for `duty` at `bits` of resolution, rounded to nearest (at most
   * 2^bits). */
  [[nodiscard]] static constexpr uint32_t DutyCounts(kb::iface::Duty duty, uint8_t bits) noexcept {
    const uint64_t full = uint64_t{1U} << bits;
    const uint64_t counts = ((uint64_t{duty.Permyriad()} * full) + (kb::iface::Duty::kFull / 2U)) /
                            kb::iface::Duty::kFull;
    return static_cast<uint32_t>(counts);
  }
  /** @brief Stops the channel and forgets the configuration. */
  [[nodiscard]] kb::Status Release() noexcept;
  /** @brief The last call's status and its esp_err_t in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return error_.Get();
  }

private:
  [[nodiscard]] kb::Status applyDuty(uint8_t bits) noexcept;
  [[nodiscard]] kb::Status retime(uint32_t hz, bool joining) noexcept;

  PwmConfig config_;
  LastErrorRecord error_;
  kb::iface::Duty duty_;
  bool enabled_{false};
  bool ready_{false};
};

static_assert(kb::iface::PwmOutput<PwmChannel>, "PwmChannel satisfies kb::iface::PwmOutput");

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_PWM_HPP
