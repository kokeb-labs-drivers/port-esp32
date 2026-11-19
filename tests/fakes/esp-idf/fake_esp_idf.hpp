// The test control of the ESP-IDF fake (fake_esp_idf.cpp): error injection per SDK function, call
// counts and a small model of each peripheral, so the tests drive every branch of the port.
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#ifndef KB_FAKE_ESP_IDF_CONTROL_HPP
#define KB_FAKE_ESP_IDF_CONTROL_HPP

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "esp_idf_fake.h"

namespace fake {

/** @brief Everything back to power-on: no injection, no handles, pins high, time at 0. */
void Reset();
/** @brief The next `count` calls of `function` return `err` without doing anything. */
void Fail(const char* function, esp_err_t err, uint32_t count = 1U);
/** @brief Calls `fn(context)` once, at the start of the next call of `function` (re-entrancy
 * tests). */
void Hook(const char* function, void (*fn)(void*), void* context);
/** @brief Calls of `function` since Reset(). */
[[nodiscard]] uint32_t Calls(const char* function);

// --- time: each esp_timer_get_time() advances the clock by the step
// ---------------------------------
void SetTimeStepUs(uint64_t step);
[[nodiscard]] uint64_t TimeUs();
void SetTimeUs(uint64_t us);

// --- GPIO
// ---------------------------------------------------------------------------------------------
/** @brief The level the pin reads: -1 follows the driven level, 0 or 1 forces it. */
void ForceLevel(int pin, int level);
[[nodiscard]] int DrivenLevel(int pin);
[[nodiscard]] gpio_mode_t Mode(int pin);
[[nodiscard]] gpio_int_type_t IrqType(int pin);
[[nodiscard]] bool IrqEnabled(int pin);
/** @brief Calls the registered handler of `pin`, as the ISR service would. */
void FireIrq(int pin);

// --- I2C
// ----------------------------------------------------------------------------------------------
void I2cPresent(uint16_t address, bool present);
void I2cReply(std::span<const uint8_t> bytes);
[[nodiscard]] const std::vector<uint8_t>& I2cWritten();
[[nodiscard]] uint16_t I2cAddress();
[[nodiscard]] uint32_t I2cClockHz();
/** @brief Makes every transfer to a present device fail with a data NACK. */
void I2cDataNack(bool on);
/** @brief The next transfer stretches `us` (fake time) and then fails with the SDK's one code. */
void I2cStretchUs(uint64_t us);
/** @brief The millisecond timeout of the last transfer or probe. */
[[nodiscard]] int I2cLastTimeoutMs();
/** @brief scl_wait_us of the device handle. */
[[nodiscard]] uint32_t I2cStretchLimitUs();

// --- SPI
// ----------------------------------------------------------------------------------------------
void SpiReply(std::span<const uint8_t> bytes);
[[nodiscard]] const std::vector<uint8_t>& SpiSent();
[[nodiscard]] uint32_t SpiTransactions();
[[nodiscard]] uint32_t SpiKeepCs();
[[nodiscard]] int SpiClockHz();
[[nodiscard]] spi_dma_chan_t SpiDma();
/** @brief The hardware takes `factor` times the wire time of a transaction. */
void SpiSlow(uint32_t factor);
/** @brief Transactions never end. */
void SpiStuck(bool on);
/** @brief Calls that would wait for ever on the SDK's bus lock (polling_start, acquire_bus). */
[[nodiscard]] uint32_t SpiWouldBlock();
/** @brief True while a transaction of `host` holds the SDK's bus lock. */
[[nodiscard]] bool SpiInFlight(int host);
/** @brief esp_cache_get_alignment's value. */
void CacheAlign(size_t bytes);
/** @brief Blocks of heap_caps_aligned_alloc not yet freed; the alignment of the last one. */
[[nodiscard]] int HeapBlocks();
/** @brief Transactions ESP-IDF bounced through a heap block (receive not on whole cache lines). */
[[nodiscard]] uint32_t SdkBounces();
/** @brief spi_device_get_actual_freq reports this many kHz (-1: the clock asked). */
void SpiReportKhz(int khz);
[[nodiscard]] size_t HeapLastAlign();
/** @brief What esp_ptr_in_iram answers. */
void IramAll(bool on);

// --- UART
// ---------------------------------------------------------------------------------------------
void UartFeed(std::span<const uint8_t> bytes);
[[nodiscard]] const std::vector<uint8_t>& UartWritten();
/** @brief Bytes uart_tx_chars accepts per call (default 128). */
void UartFifoSpace(uint32_t bytes);
/** @brief An event from the ISR; dropped (counted) when the queue is full, as ESP-IDF does. */
void UartEvent(uart_event_type_t type);
[[nodiscard]] uint32_t UartEventsDropped();
[[nodiscard]] uint32_t UartBaud();
[[nodiscard]] uart_mode_t UartMode();
[[nodiscard]] TickType_t UartLastReadTicks();
[[nodiscard]] const uart_config_t& UartLastConfig();

// --- LEDC
// ---------------------------------------------------------------------------------------------
/** @brief ledc_timer_config fails when hz << bits exceeds this (default 40 MHz). */
void LedcSourceHz(uint64_t hz);
[[nodiscard]] uint32_t LedcBits();
[[nodiscard]] uint32_t LedcFrequency();
[[nodiscard]] uint32_t LedcDuty(int channel);
[[nodiscard]] bool LedcRunning(int channel);
[[nodiscard]] uint32_t LedcIdle(int channel);
[[nodiscard]] uint32_t LedcTimerBits(int timer);
[[nodiscard]] uint32_t LedcTimerFrequency(int timer);
[[nodiscard]] bool LedcInverted(int channel);

// --- ADC
// ----------------------------------------------------------------------------------------------
/** @brief Pins 1 to 10 are ADC1 channels 0 to 9, 11 to 20 are ADC2; others are not ADC pins. */
void AdcRaw(int raw);
[[nodiscard]] int AdcUnitsAlive();
[[nodiscard]] int AdcCalisAlive();
/** @brief default_vref of the last line-fitting calibration. */
[[nodiscard]] uint32_t AdcDefaultVref();

// --- TWAI
// ---------------------------------------------------------------------------------------------
void TwaiState(twai_state_t state);
/** @brief After twai_initiate_recovery_v2, this many status reads say RECOVERING, then STOPPED. */
void TwaiRecoveryReads(uint32_t reads);
void TwaiFeed(const twai_message_t& message);
[[nodiscard]] const std::vector<twai_message_t>& TwaiSent();
[[nodiscard]] uint32_t TwaiBitrateQuanta();
[[nodiscard]] int TwaiHandlesAlive();

// --- log
// ----------------------------------------------------------------------------------------------
[[nodiscard]] const std::string& LastLog();
[[nodiscard]] esp_log_level_t LastLogLevel();

} // namespace fake

#endif // KB_FAKE_ESP_IDF_CONTROL_HPP
