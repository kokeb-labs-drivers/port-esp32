// Boundary file (deviations/boundary-files.yml): the I2C master calls of ESP-IDF v5.5 behind
// sdk.hpp. Each function converts its arguments, calls the SDK and returns its esp_err_t; the SDK
// handles live in the table below, indexed by the port number. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "driver/i2c_master.h"
#include "esp_clk_tree.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "hal/i2c_ll.h"
#include "kb/port/esp32/error.hpp"
#include "kb/port/esp32/i2c.hpp"
#include "sdk.hpp"
#include "soc/soc_caps.h"

namespace kb::port::esp32::sdk {

namespace {
constexpr uint64_t kMsPerSecond = 1000U;
// The port caps every wait at kMaxTimeout, so no conversion below reaches portMAX_DELAY.
static_assert((configTICK_RATE_HZ >= 1U) && (configTICK_RATE_HZ <= kMsPerSecond),
              "the tick conversions assume a tick of 1 ms or longer");
static_assert(((uint64_t{kMaxTimeout.Count()} + 1U) * configTICK_RATE_HZ) < portMAX_DELAY);

// Milliseconds to FreeRTOS ticks, rounded up, so a wait is never shorter than asked.
[[nodiscard]] TickType_t TicksCeil(uint32_t ms) noexcept {
  const uint64_t ticks =
      ((uint64_t{ms} * uint64_t{configTICK_RATE_HZ}) + (kMsPerSecond - 1U)) / kMsPerSecond;
  return static_cast<TickType_t>(ticks);
}

// A pin number as the SDK's gpio_num_t (-1 is GPIO_NUM_NC).
[[nodiscard]] gpio_num_t Pin(int16_t pin) noexcept {
  return static_cast<gpio_num_t>(pin);
}

// The I2C driver converts its millisecond timeout with pdMS_TO_TICKS, which rounds down (a 1 to
// 9 ms timeout is 0 ticks at 100 Hz). The value handed over is whole ticks, rounded up, plus one
// for the tick in progress, back in milliseconds: never a zero-tick poll, never shorter than asked.
[[nodiscard]] int32_t I2cMs(uint32_t ms) noexcept {
  const uint64_t ticks = uint64_t{TicksCeil(ms)} + 1U;
  const uint64_t back = ((ticks * kMsPerSecond) + configTICK_RATE_HZ - 1U) / configTICK_RATE_HZ;
  return static_cast<int32_t>(back); // at most (kMaxTimeout + 2 ticks) in ms
}

struct I2cSlot {
  i2c_master_bus_handle_t bus{nullptr};
  i2c_master_dev_handle_t dev{nullptr};
};

std::array<I2cSlot, SOC_HP_I2C_NUM>& I2cSlots() noexcept {
  // codeql::misra_deviation_next_line(DEV-001)
  static std::array<I2cSlot, SOC_HP_I2C_NUM> slots{};
  return slots;
}

[[nodiscard]] i2c_port_num_t I2cPort(uint8_t port) noexcept {
  return static_cast<i2c_port_num_t>(port);
}
} // namespace

uint8_t I2cPortCount() noexcept {
  return static_cast<uint8_t>(SOC_HP_I2C_NUM);
}

int32_t I2cOpen(const I2cConfig& config) noexcept {
  I2cSlot& slot = I2cSlots()[config.port];
  esp_err_t err = ESP_ERR_NOT_ALLOWED; // the port is owned by another controller object
  if (slot.bus == nullptr) {
    i2c_master_bus_config_t bus{};
    bus.i2c_port = I2cPort(config.port);
    bus.sda_io_num = Pin(config.sda_pin);
    bus.scl_io_num = Pin(config.scl_pin);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-union-access): the SDK's config struct
    bus.clk_source = I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt = config.glitch_filter_cycles;
    bus.intr_priority = 0;
    bus.trans_queue_depth = 0U; // synchronous transfers only
    bus.flags.enable_internal_pullup = config.internal_pullups ? 1U : 0U;
    err = i2c_new_master_bus(&bus, &slot.bus);
    if (err == ESP_OK) {
      i2c_device_config_t dev{};
      dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
      dev.device_address = kI2cFirstAddress;
      dev.scl_speed_hz = config.clock_hz;
      dev.scl_wait_us = config.stretch_limit_us; // 0: the SDK default
      err = i2c_master_bus_add_device(slot.bus, &dev, &slot.dev);
      if (err != ESP_OK) {
        static_cast<void>(i2c_del_master_bus(slot.bus));
        slot.bus = nullptr;
      }
    } else {
      slot.bus = nullptr;
    }
  }
  return err;
}

int32_t I2cClose(uint8_t port) noexcept {
  I2cSlot& slot = I2cSlots()[port];
  esp_err_t err = ESP_OK;
  if (slot.dev != nullptr) {
    err = i2c_master_bus_rm_device(slot.dev);
    slot.dev = nullptr;
  }
  if (slot.bus != nullptr) {
    const esp_err_t del = i2c_del_master_bus(slot.bus);
    err = (err == ESP_OK) ? del : err;
    slot.bus = nullptr;
  }
  return err;
}

int32_t I2cSetAddress(uint8_t port, uint8_t address, uint32_t timeout_ms) noexcept {
  return i2c_master_device_change_address(I2cSlots()[port].dev, address,
                                          static_cast<int>(I2cMs(timeout_ms)));
}

int32_t I2cTransmit(uint8_t port, std::span<const uint8_t> tx, uint32_t timeout_ms) noexcept {
  return i2c_master_transmit(I2cSlots()[port].dev, tx.data(), tx.size(),
                             static_cast<int>(I2cMs(timeout_ms)));
}

int32_t I2cTransmitReceive(uint8_t port, std::span<const uint8_t> tx, std::span<uint8_t> rx,
                           uint32_t timeout_ms) noexcept {
  return i2c_master_transmit_receive(I2cSlots()[port].dev, tx.data(), tx.size(), rx.data(),
                                     rx.size(), static_cast<int>(I2cMs(timeout_ms)));
}

int32_t I2cReceive(uint8_t port, std::span<uint8_t> rx, uint32_t timeout_ms) noexcept {
  return i2c_master_receive(I2cSlots()[port].dev, rx.data(), rx.size(),
                            static_cast<int>(I2cMs(timeout_ms)));
}

int32_t I2cProbe(uint8_t port, uint8_t address, uint32_t timeout_ms) noexcept {
  return i2c_master_probe(I2cSlots()[port].bus, address, static_cast<int>(I2cMs(timeout_ms)));
}

int32_t I2cReset(uint8_t port) noexcept {
  return i2c_master_bus_reset(I2cSlots()[port].bus);
}

uint32_t I2cStretchLimitUs(uint32_t configured_us) noexcept {
  const uint32_t asked =
      (configured_us == 0U) ? uint32_t{I2C_LL_SCL_WAIT_US_VAL_DEFAULT} : configured_us;
  uint32_t hz = 0U;
  uint64_t cap_us =
      0U; // unknown clock: no lower bound, every failure before W + slack is ambiguous
  if ((esp_clk_tree_src_get_freq_hz(static_cast<soc_module_clk_t>(I2C_CLK_SRC_DEFAULT),
                                    ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &hz) == ESP_OK) &&
      (hz >= 1000000U)) {
    cap_us = (uint64_t{I2C_LL_MAX_TIMEOUT_PERIOD} * 1000000U) / hz; // as s_i2c_timeout_range_check
  }
  return static_cast<uint32_t>(std::min<uint64_t>(asked, cap_us));
}

int32_t I2cTransferFailedCode() noexcept {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
  return ESP_ERR_INVALID_RESPONSE; // v6.0: every transfer that did not reach DONE
#else
  return ESP_ERR_INVALID_STATE; // v5.x: NACK, stretch timeout and lost arbitration alike
#endif
}

} // namespace kb::port::esp32::sdk
