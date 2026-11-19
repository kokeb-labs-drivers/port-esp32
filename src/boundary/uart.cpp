// Boundary file (deviations/boundary-files.yml): the UART calls of ESP-IDF v5.5 behind sdk.hpp.
// Each function converts its arguments, calls the SDK and returns its esp_err_t; the event queue
// handles live in the table below, indexed by the port number. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "kb/port/esp32/error.hpp"
#include "kb/port/esp32/uart.hpp"
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

constexpr std::size_t kFifoBytes = SOC_UART_FIFO_LEN;
constexpr int32_t kUartIntrFlags = 0;

struct UartQueue {
  QueueHandle_t queue{nullptr};
  uint32_t length{0U};
};

std::array<UartQueue, SOC_UART_HP_NUM>& UartQueues() noexcept {
  // codeql::misra_deviation_next_line(DEV-001)
  static std::array<UartQueue, SOC_UART_HP_NUM> queues{};
  return queues;
}

[[nodiscard]] uart_port_t UartPort(uint8_t port) noexcept {
  return static_cast<uart_port_t>(port);
}

[[nodiscard]] uart_parity_t ParityOf(UartParity parity) noexcept {
  uart_parity_t out = UART_PARITY_DISABLE;
  if (parity == UartParity::Even) {
    out = UART_PARITY_EVEN;
  } else if (parity == UartParity::Odd) {
    out = UART_PARITY_ODD;
  } else {
    out = UART_PARITY_DISABLE;
  }
  return out;
}

[[nodiscard]] uart_config_t UartFrame(const UartConfig& config) noexcept {
  uart_config_t frame{};
  frame.baud_rate = static_cast<int>(config.baud);
  frame.data_bits = static_cast<uart_word_length_t>(config.data_bits - 5U);
  frame.parity = ParityOf(config.parity);
  frame.stop_bits = (config.stop_bits == UartStopBits::Two) ? UART_STOP_BITS_2 : UART_STOP_BITS_1;
  frame.flow_ctrl = (config.cts_pin >= 0) ? UART_HW_FLOWCTRL_CTS : UART_HW_FLOWCTRL_DISABLE;
  frame.rx_flow_ctrl_thresh = 0U;
  frame.source_clk = UART_SCLK_DEFAULT;
  return frame;
}

// The uart_flags bit of an SDK event; 0 for data and pattern events, which carry no error.
[[nodiscard]] uint32_t ErrorFlag(uart_event_type_t type) noexcept {
  uint32_t flag = 0U;
  if (type == UART_FIFO_OVF) {
    flag = uart_flags::kFifoOverflow;
  } else if (type == UART_BUFFER_FULL) {
    flag = uart_flags::kBufferFull;
  } else if (type == UART_FRAME_ERR) {
    flag = uart_flags::kFrameError;
  } else if (type == UART_PARITY_ERR) {
    flag = uart_flags::kParityError;
  } else if (type == UART_BREAK) {
    flag = uart_flags::kBreak;
  } else {
    flag = 0U;
  }
  return flag;
}
} // namespace

uint8_t UartPortCount() noexcept {
  return static_cast<uint8_t>(SOC_UART_HP_NUM);
}

int32_t UartOpen(const UartConfig& config) noexcept {
  const uart_port_t port = UartPort(config.port);
  UartQueue& q = UartQueues()[config.port];
  QueueHandle_t& queue = q.queue;
  esp_err_t err = uart_driver_install(port, static_cast<int>(config.rx_buffer_bytes), 0,
                                      static_cast<int>(config.event_queue), &queue,
                                      static_cast<int>(kUartIntrFlags));
  if (err == ESP_OK) {
    const uart_config_t frame = UartFrame(config);
    err = uart_param_config(port, &frame);
    if (err == ESP_OK) {
      err = uart_set_pin(port, config.tx_pin, config.rx_pin, config.rts_pin, config.cts_pin);
    }
    if ((err == ESP_OK) && config.rs485) {
      err = uart_set_mode(port, UART_MODE_RS485_HALF_DUPLEX);
    }
    if (err != ESP_OK) {
      static_cast<void>(uart_driver_delete(port));
      queue = nullptr;
    } else {
      q.length = config.event_queue; // only for a driver this call installed
    }
  }
  return err;
}

int32_t UartClose(uint8_t port) noexcept {
  UartQueues()[port] = UartQueue{};
  return uart_driver_delete(UartPort(port));
}

int32_t UartTxFifo(uint8_t port, std::span<const uint8_t> tx, std::size_t& written) noexcept {
  // The SDK takes characters: the bytes are copied, unchanged, into a FIFO-sized buffer (the FIFO
  // takes no more in one call anyway).
  std::array<char, kFifoBytes> chars{};
  const std::size_t count = std::min(tx.size(), chars.size());
  for (std::size_t i = 0U; i < count; ++i) {
    chars[i] = std::bit_cast<char>(tx[i]); // the same bits, no numeric conversion
  }
  const int32_t n = uart_tx_chars(UartPort(port), chars.data(), static_cast<uint32_t>(count));
  written = (n > 0) ? static_cast<std::size_t>(n) : 0U;
  return (n < 0) ? ESP_FAIL : ESP_OK;
}

int32_t UartRead(uint8_t port, std::span<uint8_t> rx, uint32_t timeout_ms,
                 std::size_t& got) noexcept {
  const int32_t n = uart_read_bytes(UartPort(port), rx.data(), static_cast<uint32_t>(rx.size()),
                                    TicksCeil(timeout_ms));
  got = (n > 0) ? static_cast<std::size_t>(n) : 0U;
  return (n < 0) ? ESP_FAIL : ESP_OK;
}

int32_t UartBuffered(uint8_t port, std::size_t& bytes) noexcept {
  return uart_get_buffered_data_len(UartPort(port), &bytes);
}

int32_t UartFlushInput(uint8_t port) noexcept {
  return uart_flush_input(UartPort(port));
}

int32_t UartSetBaud(uint8_t port, uint32_t baud) noexcept {
  return uart_set_baudrate(UartPort(port), baud);
}

int32_t UartWaitTxDone(uint8_t port, uint32_t timeout_ms) noexcept {
  return uart_wait_tx_done(UartPort(port), TicksCeil(timeout_ms));
}

int32_t UartTakeErrors(uint8_t port, uint32_t& flags) noexcept {
  const UartQueue& q = UartQueues()[port];
  uart_event_t event{};
  if ((q.queue != nullptr) && (uxQueueMessagesWaiting(q.queue) >= q.length)) {
    flags |= uart_flags::kEventsLost; // full: the ISR dropped every event since, errors included
  }
  // At most twice the queue length: events arriving meanwhile are left for the next call.
  for (uint32_t i = 0U; (q.queue != nullptr) && (i < (2U * q.length)) &&
                        (xQueueReceive(q.queue, &event, 0U) == pdTRUE);
       ++i) {
    flags |= ErrorFlag(event.type);
  }
  return ESP_OK;
}

} // namespace kb::port::esp32::sdk
