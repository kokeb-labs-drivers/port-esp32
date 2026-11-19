/**
 * @file gpio.hpp
 * @brief `kb::port::esp32::OutputPin` (satisfies `kb::iface::GpioOutput`) and
 * `kb::port::esp32::InputPin` (satisfies `kb::iface::GpioInput` and `GpioInterrupt`), over the
 * ESP-IDF v5.5 `gpio` driver.
 *
 * Levels are electrical. An output pin is configured as input and output, so Toggle() reads the
 * pin itself; Set() and Toggle() are single register writes and may be called from an interrupt.
 * One object per pin, used from one context at a time: a pin already owned by another port object
 * (another pin object, or a bus, PWM, ADC or CAN pin) is refused with `Busy` by EnsureReady().
 *
 * The output level is written before the pin becomes an output, so it never shows another level
 * (ESP-IDF's `gpio_config` would otherwise drive the latch's 0 first). Release() and the destructor
 * drive `OutputConfig::safe` (by default the initial level) and leave the pin an output at that
 * level: they never reset it with `gpio_reset_pin`, which turns the output off and the pull-up on
 * and could switch an active-high load on. No pull resistor is enabled unless configured.
 *
 * Interrupts use the SDK's per-pin ISR service (`gpio_install_isr_service`, installed by the port
 * on the first EnableInterrupt() without `ESP_INTR_FLAG_IRAM`, so the callbacks may be in flash and
 * are held off while the flash cache is disabled). When the application installed the service
 * first, the port cannot read its flags: a service installed with `ESP_INTR_FLAG_IRAM` calls the
 * callback while the cache is off, which crashes with a callback in flash. EnableInterrupt() then
 * takes only a callback in IRAM, or `InputConfig::foreign_isr_service_ok` (the application states
 * that its service has no `ESP_INTR_FLAG_IRAM`); otherwise it returns `Unsupported`. The callback
 * runs in interrupt context and may only use the `...FromIsr` calls of kb::rtos.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_GPIO_HPP
#define KB_PORT_ESP32_GPIO_HPP

#include <cstdint>

#include "kb/interfaces/gpio.hpp"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/utils/result.hpp"
#include "kb/utils/status.hpp"

namespace kb::port::esp32 {

/** @brief Internal pull resistors of a pin. */
enum class Pull : uint8_t {
  None = 0, ///< Floating.
  Up = 1,   ///< Weak pull-up (about 45 kOhm).
  Down = 2, ///< Weak pull-down.
};

/** @brief The level an output keeps after Release() or destruction. */
enum class SafeLevel : uint8_t {
  Initial = 0, ///< OutputConfig::initial (the level the pin started at).
  Low = 1,     ///< Low.
  High = 2,    ///< High.
};

/** @brief An output pin. */
struct OutputConfig {
  int16_t pin{-1};        ///< GPIO number.
  bool open_drain{false}; ///< Open drain instead of push-pull.
  Pull pull{Pull::None};  ///< Pull resistor (open drain needs one, or an external).
  kb::iface::Level initial{kb::iface::Level::Low}; ///< Level driven by EnsureReady().
  SafeLevel safe{SafeLevel::Initial}; ///< Level driven and kept by Release() and the destructor.
};

/** @brief An input pin. */
struct InputConfig {
  int16_t pin{-1};       ///< GPIO number.
  Pull pull{Pull::None}; ///< Pull resistor.
  /// The application installed the GPIO ISR service itself, without ESP_INTR_FLAG_IRAM, so a
  /// callback in flash is safe (see the file comment).
  bool foreign_isr_service_ok{false};
};

/** @brief One output pin; satisfies kb::iface::GpioOutput. */
class OutputPin {
public:
  /** @brief Keeps `config`. */
  explicit OutputPin(const OutputConfig& config) noexcept : config_(config) {}
  OutputPin(const OutputPin&) = delete;
  OutputPin& operator=(const OutputPin&) = delete;
  OutputPin(OutputPin&&) = delete;
  OutputPin& operator=(OutputPin&&) = delete;
  /** @brief Gives the pin back, driving its safe level (see the file comment). */
  ~OutputPin() noexcept;

  /** @brief Configures the pin and drives `OutputConfig::initial`; idempotent. */
  [[nodiscard]] kb::Status EnsureReady() noexcept;
  /** @brief Drives `level`. */
  [[nodiscard]] kb::Status Set(kb::iface::Level level) noexcept;
  /** @brief Drives the opposite of the level the pin reads. */
  [[nodiscard]] kb::Status Toggle() noexcept;
  /** @brief Drives `OutputConfig::safe`, keeps the pin an output at it and gives the claim back. */
  [[nodiscard]] kb::Status Release() noexcept;
  /** @brief The last call's status and its esp_err_t in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return error_.Get();
  }

private:
  OutputConfig config_;
  LastErrorRecord error_;
  bool ready_{false};
};

/** @brief One input pin; satisfies kb::iface::GpioInput and GpioInterrupt. */
class InputPin {
public:
  /** @brief Keeps `config`. */
  explicit InputPin(const InputConfig& config) noexcept : config_(config) {}
  InputPin(const InputPin&) = delete;
  InputPin& operator=(const InputPin&) = delete;
  InputPin(InputPin&&) = delete;
  InputPin& operator=(InputPin&&) = delete;
  /** @brief Disables the interrupt and gives the pin back. */
  ~InputPin() noexcept;

  /** @brief Configures the pin; idempotent. */
  [[nodiscard]] kb::Status EnsureReady() noexcept;
  /** @brief The level the pin reads. */
  [[nodiscard]] kb::Status Get(kb::iface::Level& out) noexcept;
  /** @brief Calls `fn(context)` in interrupt context on `edge`; replaces an earlier callback. */
  [[nodiscard]] kb::Status EnableInterrupt(kb::iface::Edge edge, kb::iface::GpioCallback fn,
                                           void* context) noexcept;
  /** @brief Stops calling the callback. */
  [[nodiscard]] kb::Status DisableInterrupt() noexcept;
  /** @brief Disables the interrupt and gives the claim back; the pin stays an input. */
  [[nodiscard]] kb::Status Release() noexcept;
  /** @brief The last call's status and its esp_err_t in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return error_.Get();
  }

private:
  InputConfig config_;
  LastErrorRecord error_;
  bool ready_{false};
  bool irq_{false};
};

static_assert(kb::iface::GpioOutput<OutputPin>, "OutputPin satisfies kb::iface::GpioOutput");
static_assert(kb::iface::GpioInput<InputPin>, "InputPin satisfies kb::iface::GpioInput");
static_assert(kb::iface::GpioInterrupt<InputPin>, "InputPin satisfies kb::iface::GpioInterrupt");

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_GPIO_HPP
