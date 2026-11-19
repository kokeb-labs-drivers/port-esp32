// Boundary file (deviations/boundary-files.yml): the GPIO calls of ESP-IDF v5.5 behind sdk.hpp.
// Each function converts its arguments, calls the SDK and returns its esp_err_t. Chip differences
// use the SDK's capability macros (soc/soc_caps.h) here and nowhere else. SPDX-License-Identifier:
// MIT Copyright (c) 2026 The port-esp32 authors
#include <bit>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"
#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/gpio.hpp"
#include "sdk.hpp"
#include "soc/soc.h"
#include "soc/soc_caps.h"

namespace kb::port::esp32::sdk {

namespace {
// A pin number as the SDK's gpio_num_t (-1 is GPIO_NUM_NC).
[[nodiscard]] gpio_num_t Pin(int16_t pin) noexcept {
  return static_cast<gpio_num_t>(pin);
}

// The bit of a pin in gpio_config_t::pin_bit_mask.
[[nodiscard]] uint64_t Bit(int16_t pin) noexcept {
  return uint64_t{1U} << static_cast<uint32_t>(pin);
}

[[nodiscard]] gpio_pull_mode_t PullOf(Pull pull) noexcept {
  gpio_pull_mode_t mode = GPIO_FLOATING;
  if (pull == Pull::Up) {
    mode = GPIO_PULLUP_ONLY;
  } else if (pull == Pull::Down) {
    mode = GPIO_PULLDOWN_ONLY;
  } else {
    mode = GPIO_FLOATING;
  }
  return mode;
}

[[nodiscard]] gpio_int_type_t EdgeOf(kb::iface::Edge edge) noexcept {
  gpio_int_type_t type = GPIO_INTR_ANYEDGE;
  if (edge == kb::iface::Edge::Rising) {
    type = GPIO_INTR_POSEDGE;
  } else if (edge == kb::iface::Edge::Falling) {
    type = GPIO_INTR_NEGEDGE;
  } else {
    type = GPIO_INTR_ANYEDGE;
  }
  return type;
}
} // namespace

bool GpioValidOutput(int16_t pin) noexcept {
  return (pin >= 0) && GPIO_IS_VALID_OUTPUT_GPIO(pin);
}
bool GpioValid(int16_t pin) noexcept {
  return (pin >= 0) && GPIO_IS_VALID_GPIO(pin);
}

int32_t GpioConfigOutput(const OutputConfig& config) noexcept {
  // The level first, so the pin starts driving the asked level and never glitches.
  esp_err_t err =
      gpio_set_level(Pin(config.pin), (config.initial == kb::iface::Level::High) ? 1U : 0U);
  if (err == ESP_OK) {
    gpio_config_t io{};
    io.pin_bit_mask = Bit(config.pin);
    io.mode = config.open_drain ? GPIO_MODE_INPUT_OUTPUT_OD : GPIO_MODE_INPUT_OUTPUT;
    io.pull_up_en = (config.pull == Pull::Up) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io.pull_down_en = (config.pull == Pull::Down) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    err = gpio_config(&io);
  }
  return err;
}

int32_t GpioConfigInput(const InputConfig& config) noexcept {
  gpio_config_t io{};
  io.pin_bit_mask = Bit(config.pin);
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = (config.pull == Pull::Up) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  io.pull_down_en = (config.pull == Pull::Down) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  esp_err_t err = gpio_config(&io);
  if (err == ESP_OK) {
    err = gpio_set_pull_mode(Pin(config.pin), PullOf(config.pull));
  }
  return err;
}

int32_t GpioSet(int16_t pin, bool high) noexcept {
  return gpio_set_level(Pin(pin), high ? 1U : 0U);
}

bool GpioGet(int16_t pin) noexcept {
  return gpio_get_level(Pin(pin)) != 0;
}

int32_t GpioIsrService() noexcept {
  return gpio_install_isr_service(0); // no ESP_INTR_FLAG_IRAM: callbacks may live in flash
}

bool GpioCallbackInIram(kb::iface::GpioCallback fn) noexcept {
  // esp_ptr_in_iram()'s test on the address as an integer (a function pointer is not an object
  // pointer, so it is not converted to `const void*`).
  const auto address = std::bit_cast<std::uintptr_t>(fn);
#if defined(CONFIG_IDF_TARGET_ESP32) && defined(CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE)
  const std::uintptr_t low = SOC_CACHE_APP_LOW;
#else
  const std::uintptr_t low = SOC_IRAM_LOW;
#endif
  return (address >= low) && (address < std::uintptr_t{SOC_IRAM_HIGH});
}

int32_t GpioIrqEnable(int16_t pin, kb::iface::Edge edge, kb::iface::GpioCallback fn,
                      void* context) noexcept {
  const gpio_int_type_t type = EdgeOf(edge);
  static_cast<void>(gpio_isr_handler_remove(Pin(pin))); // replace an earlier callback
  esp_err_t err = gpio_set_intr_type(Pin(pin), type);
  if (err == ESP_OK) {
    err = gpio_isr_handler_add(Pin(pin), fn, context);
  }
  if (err == ESP_OK) {
    err = gpio_intr_enable(Pin(pin));
  }
  return err;
}

int32_t GpioIrqDisable(int16_t pin) noexcept {
  esp_err_t err = gpio_intr_disable(Pin(pin));
  const esp_err_t removed = gpio_isr_handler_remove(Pin(pin));
  return (err == ESP_OK) ? removed : err;
}

} // namespace kb::port::esp32::sdk
