/**
 * @file port.hpp
 * @brief Every public header of the ESP32 port, and the proof that each type satisfies its
 * kb::iface contract and converts to the contract's `*Ref`.
 *
 * | Contract | Type | Header |
 * | --- | --- | --- |
 * | `I2cBus` | `I2cController` | i2c.hpp |
 * | `SpiDevice`, `SpiClockControl` | `SpiDevice` (on an `SpiBus`) | spi.hpp |
 * | `UartPort`, `UartBaudControl`, `UartTxDrain` | `Uart` | uart.hpp |
 * | `GpioOutput` | `OutputPin` | gpio.hpp |
 * | `GpioInput`, `GpioInterrupt` | `InputPin` | gpio.hpp |
 * | `AdcChannel` | `AdcChannel` | adc.hpp |
 * | `PwmOutput` | `PwmChannel` | pwm.hpp |
 * | `CanBus` | `CanController` | can.hpp |
 * | `MonotonicClock` | `Clock` | time.hpp |
 * | `BusyWait` | `BusyWait` | time.hpp |
 * | `LogSink` | `LogSink` | log.hpp |
 * | `kb::dma::CacheOps` | `EspCacheOps` | dma.hpp |
 * | `kb::dma::ReportsCapabilities` | `SpiBus`, `SpiDmaChannel<R>`, `AdcStream<F, N>` | spi.hpp,
 * spi_dma.hpp, adc_stream.hpp |
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_PORT_HPP
#define KB_PORT_ESP32_PORT_HPP

#include <type_traits>

#include "kb/interfaces/interfaces.hpp"
#include "kb/port/esp32/adc.hpp"
#include "kb/port/esp32/adc_stream.hpp"
#include "kb/port/esp32/can.hpp"
#include "kb/port/esp32/dma.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/port/esp32/gpio.hpp"
#include "kb/port/esp32/i2c.hpp"
#include "kb/port/esp32/log.hpp"
#include "kb/port/esp32/pwm.hpp"
#include "kb/port/esp32/spi.hpp"
#include "kb/port/esp32/spi_dma.hpp"
#include "kb/port/esp32/time.hpp"
#include "kb/port/esp32/uart.hpp"
#include "kb/port/esp32/version.hpp"

namespace kb::port::esp32 {

// Each type can be named by its contract's type-erased reference (run-time choice in a HAL).
static_assert(std::is_constructible_v<kb::iface::I2cBusRef, I2cController&>);
static_assert(std::is_constructible_v<kb::iface::SpiDeviceRef, SpiDevice&>);
static_assert(std::is_constructible_v<kb::iface::UartPortRef, Uart&>);
static_assert(std::is_constructible_v<kb::iface::GpioOutputRef, OutputPin&>);
static_assert(std::is_constructible_v<kb::iface::GpioInputRef, InputPin&>);
static_assert(std::is_constructible_v<kb::iface::AdcChannelRef, AdcChannel&>);
static_assert(std::is_constructible_v<kb::iface::PwmOutputRef, PwmChannel&>);
static_assert(std::is_constructible_v<kb::iface::CanBusRef, CanController&>);
static_assert(std::is_constructible_v<kb::iface::MonotonicClockRef, const Clock&>);
static_assert(std::is_constructible_v<kb::iface::BusyWaitRef, BusyWait&>);
static_assert(std::is_constructible_v<kb::iface::LogSinkRef, LogSink&>);

} // namespace kb::port::esp32

static_assert(kb::dma::ReportsCapabilities<kb::port::esp32::SpiBus>);
static_assert(
    kb::dma::ReportsCapabilities<kb::port::esp32::SpiDmaChannel<kb::port::esp32::InternalRam>>);
static_assert(kb::dma::ReportsCapabilities<kb::port::esp32::AdcStreamDefault>);

#endif // KB_PORT_ESP32_PORT_HPP
