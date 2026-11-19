// The example pins from menuconfig (main/Kconfig.projbuild), as typed constants. Example code.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#ifndef KB_PORT_ESP32_EXAMPLE_BOARD_PINS_HPP
#define KB_PORT_ESP32_EXAMPLE_BOARD_PINS_HPP

#include <cstdint>

#include "sdkconfig.h"

namespace pins {
inline constexpr int16_t kI2cSda = CONFIG_EXAMPLE_I2C_SDA;
inline constexpr int16_t kI2cScl = CONFIG_EXAMPLE_I2C_SCL;
inline constexpr int16_t kSpiMosi = CONFIG_EXAMPLE_SPI_MOSI;
inline constexpr int16_t kSpiMiso = CONFIG_EXAMPLE_SPI_MISO;
inline constexpr int16_t kSpiSclk = CONFIG_EXAMPLE_SPI_SCLK;
inline constexpr int16_t kSpiCs = CONFIG_EXAMPLE_SPI_CS;
inline constexpr int16_t kAdc = CONFIG_EXAMPLE_ADC_PIN;
inline constexpr int16_t kLed = CONFIG_EXAMPLE_LED_PIN;
inline constexpr int16_t kCanTx = CONFIG_EXAMPLE_CAN_TX;
inline constexpr int16_t kCanRx = CONFIG_EXAMPLE_CAN_RX;
inline constexpr int16_t kUartTx = CONFIG_EXAMPLE_UART_TX;
inline constexpr int16_t kUartRx = CONFIG_EXAMPLE_UART_RX;
} // namespace pins

#endif // KB_PORT_ESP32_EXAMPLE_BOARD_PINS_HPP
