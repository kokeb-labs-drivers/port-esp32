/**
 * @file sdk.hpp
 * @brief The seam between the port's platform-free logic (src/<header>.cpp) and its boundary files
 * (src/boundary/<component>.cpp, one per SDK component, each defining the section below that
 carries
 * its name, and the chip capabilities of that peripheral): one function per SDK operation, in kb
 * types only. Each returns the `esp_err_t` of the SDK call as an int32_t. Private to the port.
 *

 * The boundary files (src/boundary/) are the only files that include ESP-IDF headers
 * (deviations/boundary-files.yml). Each function converts its arguments, makes one SDK call (or a
 * fixed short sequence) and returns the SDK's code; the logic decides what the code means. SDK
 * handles stay in the boundary files, in tables indexed by the hardware number (port, host, unit,
 * controller) or by a slot.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_SRC_SDK_HPP
#define KB_PORT_ESP32_SRC_SDK_HPP

#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/interfaces/types.hpp"
#include "kb/port/esp32/adc.hpp"
#include "kb/port/esp32/can.hpp"
#include "kb/port/esp32/gpio.hpp"
#include "kb/port/esp32/i2c.hpp"
#include "kb/port/esp32/pwm.hpp"
#include "kb/port/esp32/spi.hpp"
#include "kb/port/esp32/uart.hpp"

namespace kb::port::esp32::sdk {

// --- chip capabilities (soc_caps.h) -----------------------------------------------------------
/** @brief Number of HP I2C controllers. */
[[nodiscard]] uint8_t I2cPortCount() noexcept;
/** @brief Number of HP UARTs. */
[[nodiscard]] uint8_t UartPortCount() noexcept;
/** @brief Highest SPI host number usable by the port (SPI2_HOST and up). */
[[nodiscard]] uint8_t SpiHostLast() noexcept;
/** @brief Number of LEDC channels and the LEDC timer width in bits. */
[[nodiscard]] uint8_t LedcChannelCount() noexcept;
[[nodiscard]] uint8_t LedcTimerBits() noexcept; ///< See LedcChannelCount.
/** @brief Number of TWAI controllers (0 when the chip has none). */
[[nodiscard]] uint8_t TwaiCount() noexcept;
/** @brief True when `pin` can drive an output. */
[[nodiscard]] bool GpioValidOutput(int16_t pin) noexcept;
/** @brief True when `pin` is a GPIO. */
[[nodiscard]] bool GpioValid(int16_t pin) noexcept;

// --- time ---------------------------------------------------------------------------------------
/** @brief esp_timer_get_time(). */
[[nodiscard]] uint64_t NowUs() noexcept;
/** @brief esp_rom_delay_us(). */
void DelayUs(uint32_t us) noexcept;
/** @brief vTaskDelay of at least `ms` (rounded up to ticks). */
void SleepMs(uint32_t ms) noexcept;

// --- I2C ----------------------------------------------------------------------------------------
[[nodiscard]] int32_t I2cOpen(const I2cConfig& config) noexcept;
[[nodiscard]] int32_t I2cClose(uint8_t port) noexcept;
[[nodiscard]] int32_t I2cSetAddress(uint8_t port, uint8_t address, uint32_t timeout_ms) noexcept;
[[nodiscard]] int32_t I2cTransmit(uint8_t port, std::span<const uint8_t> tx,
                                  uint32_t timeout_ms) noexcept;
[[nodiscard]] int32_t I2cTransmitReceive(uint8_t port, std::span<const uint8_t> tx,
                                         std::span<uint8_t> rx, uint32_t timeout_ms) noexcept;
[[nodiscard]] int32_t I2cReceive(uint8_t port, std::span<uint8_t> rx, uint32_t timeout_ms) noexcept;
[[nodiscard]] int32_t I2cProbe(uint8_t port, uint8_t address, uint32_t timeout_ms) noexcept;
/** @brief i2c_master_bus_reset(): nine SCL clocks and a STOP, then the controller is reset. */
[[nodiscard]] int32_t I2cReset(uint8_t port) noexcept;
/**
 * @brief The code a transfer returns when it did not complete on the bus (a NACK, a clock-stretch
 * timeout or lost arbitration): ESP_ERR_INVALID_STATE up to ESP-IDF v5.x, ESP_ERR_INVALID_RESPONSE
 * from v6.0.
 */
[[nodiscard]] int32_t I2cTransferFailedCode() noexcept;
/**
 * @brief The SCL low limit the hardware applies for `configured_us` (0: the SDK default), capped
 * as ESP-IDF caps it for the chip's I2C clock, in microseconds. A stretch timeout cannot end
 * before it (the register rounds up, never down).
 */
[[nodiscard]] uint32_t I2cStretchLimitUs(uint32_t configured_us) noexcept;

// --- SPI ----------------------------------------------------------------------------------------
[[nodiscard]] int32_t SpiBusOpen(const SpiBusConfig& config) noexcept;
[[nodiscard]] int32_t SpiBusClose(uint8_t host) noexcept;
/** @brief Device slots of the handle table (slots are chosen by the logic). */
[[nodiscard]] uint8_t SpiSlotCount() noexcept;
/** @brief Adds a device at `clock_hz` in `slot`; `actual_hz` receives the clock the SDK set. */
[[nodiscard]] int32_t SpiDeviceAdd(uint8_t host, const SpiDeviceConfig& config, uint32_t clock_hz,
                                   uint8_t slot, uint32_t& actual_hz) noexcept;
[[nodiscard]] int32_t SpiDeviceRemove(uint8_t slot) noexcept;
/**
 * @brief spi_device_polling_start() of one transaction (descriptor kept in the slot's table entry
 * until it ends). An empty `tx` clocks zeros (at most SpiZeroBytes()).
 */
[[nodiscard]] int32_t SpiStart(uint8_t slot, std::span<const uint8_t> tx,
                               std::span<uint8_t> rx) noexcept;
/** @brief spi_device_polling_end(): waits at most `timeout_ms`; may be called again after a
 * timeout, the transaction stays in flight until it returns ESP_OK. */
[[nodiscard]] int32_t SpiEnd(uint8_t slot, uint32_t timeout_ms) noexcept;
/** @brief The zero buffer behind an empty `tx`, in bytes. */
[[nodiscard]] std::size_t SpiZeroBytes() noexcept;
/**
 * @brief Allocates the DMA buffer of `host` once: two halves of `bytes` each, DMA-capable internal
 * RAM, start and size aligned to the DMA and cache rules (whole words at least). ESP_ERR_NO_MEM
 * when the heap has no room.
 */
[[nodiscard]] int32_t SpiDmaBufferOpen(uint8_t host, std::size_t bytes, std::span<uint8_t>& tx,
                                       std::span<uint8_t>& rx) noexcept;
/** @brief Frees the DMA buffer of `host` (after spi_bus_free succeeded). */
void SpiDmaBufferClose(uint8_t host) noexcept;

// --- UART ---------------------------------------------------------------------------------------
[[nodiscard]] int32_t UartOpen(const UartConfig& config) noexcept;
[[nodiscard]] int32_t UartClose(uint8_t port) noexcept;
/** @brief uart_tx_chars(): as many bytes as fit in the FIFO now; `written` receives the count. */
[[nodiscard]] int32_t UartTxFifo(uint8_t port, std::span<const uint8_t> tx,
                                 std::size_t& written) noexcept;
/** @brief uart_read_bytes(): up to `rx.size()` bytes, waiting at most `timeout_ms`. */
[[nodiscard]] int32_t UartRead(uint8_t port, std::span<uint8_t> rx, uint32_t timeout_ms,
                               std::size_t& got) noexcept;
[[nodiscard]] int32_t UartBuffered(uint8_t port, std::size_t& bytes) noexcept;
[[nodiscard]] int32_t UartFlushInput(uint8_t port) noexcept;
[[nodiscard]] int32_t UartSetBaud(uint8_t port, uint32_t baud) noexcept;
[[nodiscard]] int32_t UartWaitTxDone(uint8_t port, uint32_t timeout_ms) noexcept;
/**
 * @brief Drains the SDK event queue without waiting; ORs uart_flags into `flags`, with
 * uart_flags::kEventsLost when the queue was full (ESP-IDF drops events then).
 */
[[nodiscard]] int32_t UartTakeErrors(uint8_t port, uint32_t& flags) noexcept;

// --- GPIO ---------------------------------------------------------------------------------------
[[nodiscard]] int32_t GpioConfigOutput(const OutputConfig& config) noexcept;
[[nodiscard]] int32_t GpioConfigInput(const InputConfig& config) noexcept;
[[nodiscard]] int32_t GpioSet(int16_t pin, bool high) noexcept;
[[nodiscard]] bool GpioGet(int16_t pin) noexcept;
/** @brief gpio_install_isr_service(): ESP_OK when installed now, ESP_ERR_INVALID_STATE when it was
 * installed before (by the port or by the application). */
[[nodiscard]] int32_t GpioIsrService() noexcept;
/** @brief True when `fn` runs from IRAM (safe while the flash cache is off). */
[[nodiscard]] bool GpioCallbackInIram(kb::iface::GpioCallback fn) noexcept;
[[nodiscard]] int32_t GpioIrqEnable(int16_t pin, kb::iface::Edge edge, kb::iface::GpioCallback fn,
                                    void* context) noexcept;
[[nodiscard]] int32_t GpioIrqDisable(int16_t pin) noexcept;

// --- ADC ----------------------------------------------------------------------------------------
/** @brief Unit and channel of a pin. */
[[nodiscard]] int32_t AdcPinChannel(int16_t pin, uint8_t& unit, uint8_t& channel) noexcept;
/** @brief Number of ADC units and channel slots of the handle tables. */
[[nodiscard]] uint8_t AdcUnitCount() noexcept;
[[nodiscard]] uint8_t AdcSlotCount() noexcept; ///< See AdcUnitCount.
/** @brief adc_oneshot_new_unit / adc_oneshot_del_unit (the logic counts the users). */
[[nodiscard]] int32_t AdcUnitOpen(uint8_t unit) noexcept;
[[nodiscard]] int32_t AdcUnitClose(uint8_t unit) noexcept;
/** @brief Configures `channel` of `unit` in `slot`. */
[[nodiscard]] int32_t AdcChannelOpen(uint8_t slot, uint8_t unit, uint8_t channel,
                                     AdcAttenuation attenuation) noexcept;
/** @brief Creates the calibration of `slot`; ESP_ERR_NOT_SUPPORTED without scheme or eFuse data.
 * `default_vref_mv` is used by the line-fitting scheme on chips without an eFuse Vref (0: none). */
[[nodiscard]] int32_t AdcCalibrate(uint8_t slot, uint16_t default_vref_mv) noexcept;
/** @brief Deletes the calibration of `slot`. */
[[nodiscard]] int32_t AdcChannelClose(uint8_t slot) noexcept;
/** @brief One conversion (the logic holds the unit's lock). */
[[nodiscard]] int32_t AdcRead(uint8_t slot, int32_t& raw) noexcept;
[[nodiscard]] int32_t AdcToMillivolts(uint8_t slot, int32_t raw, int32_t& mv) noexcept;
/** @brief The ADC's output width in bits. */
[[nodiscard]] uint8_t AdcBits() noexcept;

// --- LEDC ---------------------------------------------------------------------------------------
[[nodiscard]] int32_t LedcTimer(uint8_t timer, uint32_t hz, uint8_t bits) noexcept;
/** @brief Configures the channel at duty 0, output inverted when `idle_high`, so the pin shows the
 * idle level from the moment it is routed. */
[[nodiscard]] int32_t LedcChannel(const PwmConfig& config) noexcept;
[[nodiscard]] int32_t LedcDuty(uint8_t channel, uint32_t counts) noexcept;
/** @brief ledc_stop() at idle level 0 (the pin shows the idle level, see LedcChannel). */
[[nodiscard]] int32_t LedcStop(uint8_t channel) noexcept;

// --- TWAI ---------------------------------------------------------------------------------------
[[nodiscard]] int32_t TwaiOpen(const CanConfig& config) noexcept;
[[nodiscard]] int32_t TwaiClose(uint8_t controller) noexcept;
[[nodiscard]] int32_t TwaiSend(uint8_t controller, const kb::iface::CanFrame& frame,
                               uint32_t timeout_ms) noexcept;
[[nodiscard]] int32_t TwaiReceive(uint8_t controller, kb::iface::CanFrame& frame,
                                  uint32_t timeout_ms) noexcept;
[[nodiscard]] int32_t TwaiStatus(uint8_t controller, CanStatus& status) noexcept;
[[nodiscard]] int32_t TwaiRecover(uint8_t controller) noexcept;
[[nodiscard]] int32_t TwaiStart(uint8_t controller) noexcept;

// --- log ----------------------------------------------------------------------------------------
/** @brief esp_log_write of `text` (already cut to kLogMaxLine) under `tag`. */
void LogWrite(kb::iface::LogLevel level, const char* tag, std::span<const char> text) noexcept;

} // namespace kb::port::esp32::sdk

#endif // KB_PORT_ESP32_SRC_SDK_HPP
