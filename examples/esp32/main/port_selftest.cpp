// The port's self test: every contract once on the bare board, with no external chip (clock, busy wait,
// log, GPIO, PWM, ADC, I2C and SPI buses, CAN in no-ack mode, UART). Example code.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <array>
#include <cinttypes>
#include <cstdint>

#include "board_pins.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kb/port/esp32/port.hpp"

namespace {
namespace esp = kb::port::esp32;

// --- the port's self test: every contract once, on any board without external chips ---------------
void SelfTest() {
  const char* tag = "selftest";
  const esp::Clock clock{};
  esp::BusyWait wait{};
  const kb::Instant t0 = clock.Now();
  wait.WaitUs(kb::Microseconds{500U});
  ESP_LOGI(tag, "clock %" PRIu64 " us, busy wait 500 us took %" PRIu64 " us", t0.Us(),
           kb::utils::ElapsedUs(clock.Now(), t0));
  esp::LogSink log{"selftest"};
  log.Write(kb::iface::LogLevel::Info, "LogSink: one line through esp_log");
  {
    esp::OutputConfig o{};
    o.pin = pins::kLed;
    esp::OutputPin led{o};
    const kb::Status s = led.EnsureReady();
    for (int k = 0; (s == kb::Status::Ok) && (k < 6); ++k) {
      static_cast<void>(led.Toggle());
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(tag, "GPIO %d toggled: %s", pins::kLed, kb::ToString(s));
  }
  esp::PwmConfig p{};
  p.pin = pins::kLed;
  p.frequency_hz = 5000U;
  esp::PwmChannel pwm{p};
  kb::Status s = pwm.EnsureReady();
  s = (s == kb::Status::Ok) ? pwm.SetDuty(kb::iface::Duty{2500U}) : s;
  s = (s == kb::Status::Ok) ? pwm.Enable(true) : s;
  ESP_LOGI(tag, "PWM 5 kHz 25 %% at %u bits: %s", static_cast<unsigned>(pwm.ResolutionBits()), kb::ToString(s));
  esp::AdcConfig a{};
  a.pin = pins::kAdc;
  esp::AdcChannel adc{a};
  if (adc.EnsureReady() == kb::Status::Ok) {
    const kb::Result<uint32_t> raw = adc.ReadRaw();
    const kb::Result<int32_t> mv = adc.ReadMillivolts();
    ESP_LOGI(tag, "ADC pin %d: %" PRIu32 " of %" PRIu32 ", %" PRId32 " mV (%s)", pins::kAdc, raw.ValueOr(0U),
             adc.FullScale(), mv.ValueOr(-1), kb::ToString(kb::ToStatus(mv)));
  }
  {
    esp::I2cConfig ic{};
    ic.port = 0U;
    ic.sda_pin = pins::kI2cSda;
    ic.scl_pin = pins::kI2cScl;
    ic.clock_hz = 400000U;
    ic.timeout = kb::Milliseconds{20U};
    esp::I2cController i2c{ic};
    ESP_LOGI(tag, "I2C controller: %s (no device is wired)", kb::ToString(i2c.EnsureReady()));
  }
  {
    esp::SpiBusConfig sb{};
    sb.mosi_pin = pins::kSpiMosi;
    sb.miso_pin = pins::kSpiMiso;
    sb.sclk_pin = pins::kSpiSclk;
    esp::SpiDeviceConfig sd{};
    sd.cs_pin = pins::kSpiCs;
    esp::SpiBus spi_bus{sb};
    esp::SpiDevice spi{spi_bus, sd};
    ESP_LOGI(tag, "SPI device: %s (no device is wired)", kb::ToString(spi.EnsureReady()));
  }
  esp::CanConfig c{};
  c.tx_pin = pins::kCanTx;
  c.rx_pin = pins::kCanRx;
  c.mode = esp::CanMode::NoAck;
  esp::CanController can{c};
  kb::iface::CanFrame f{};
  f.id = 0x123U;
  f.dlc = 2U;
  f.data[0] = 0xCAU;
  f.data[1] = 0xFEU;
  s = can.EnsureReady();
  s = (s == kb::Status::Ok) ? can.Send(f, kb::Milliseconds{10U}) : s;
  ESP_LOGI(tag, "CAN (no-ack mode) send: %s", kb::ToString(s));
  if (pins::kUartTx >= 0) {
    esp::UartConfig u{};
    u.port = 1U;
    u.tx_pin = pins::kUartTx;
    u.rx_pin = pins::kUartRx;
    esp::Uart uart{u};
    const std::array<uint8_t, 4> out{0x55U, 0xAAU, 0x01U, 0x02U};
    std::array<uint8_t, 4> in{};
    s = uart.EnsureReady();
    s = (s == kb::Status::Ok) ? uart.Write(out) : s;
    const kb::Result<std::size_t> got = uart.Read(in, kb::Milliseconds{20U});
    ESP_LOGI(tag, "UART write %s, read %u byte(s) (wire TX to RX for a loopback)", kb::ToString(s),
             static_cast<unsigned>(got.ValueOr(0U)));
  }
}

} // namespace

extern "C" void app_main(void) {
  ESP_LOGI("selftest", "kb port %s on %s", kb::port::esp32::kVersion, kb::port::esp32::kSdkLine);
  SelfTest();
}
