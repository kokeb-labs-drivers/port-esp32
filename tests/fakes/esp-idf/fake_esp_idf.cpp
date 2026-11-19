// The ESP-IDF fake: definitions of every SDK function the boundary files call (the link seam), with
// error injection by function name and a small model of each peripheral. Test code.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "fake_esp_idf.hpp"

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <vector>

struct fake_queue {
  int id;
};
struct i2c_master_bus_t {
  int id;
};
struct i2c_master_dev_t {
  int id;
};
struct spi_device_t {
  int id;
};
struct adc_oneshot_unit_ctx_t {
  int id;
};
struct adc_cali_scheme_t {
  int id;
};
struct twai_obj_t {
  int id;
};

namespace fake {
namespace {
constexpr int kPins = 64;

struct Injection {
  std::string function;
  esp_err_t err;
  uint32_t count;
};

struct Pin {
  int driven{1};
  int forced{-1};
  gpio_mode_t mode{GPIO_MODE_DISABLE};
  gpio_int_type_t type{GPIO_INTR_DISABLE};
  bool enabled{false};
  gpio_isr_t isr{nullptr};
  void* arg{nullptr};
};

struct PendingHook {
  std::string function;
  void (*fn)(void*);
  void* context;
};

struct State {
  std::vector<Injection> injections;
  std::vector<PendingHook> hooks;
  std::map<std::string, uint32_t> calls;
  uint64_t now{0U};
  uint64_t step{1U};
  std::array<Pin, kPins> pins{};
  bool isr_service{false};
  // I2C
  std::array<i2c_master_bus_t, 2> buses{};
  std::array<i2c_master_dev_t, 2> devs{};
  std::array<bool, 128> present{};
  std::vector<uint8_t> i2c_reply;
  std::vector<uint8_t> i2c_written;
  uint16_t i2c_address{0U};
  uint32_t i2c_clock{0U};
  uint32_t i2c_stretch_limit{0U};
  bool i2c_data_nack{false};
  uint64_t i2c_stretch_us{
      0U}; // the next transfer to a present device stretches this long, then fails
  int i2c_last_ms{0};
  // SPI: each device remembers its host and clock; each host its SDK bus lock and the transaction
  // in flight (ESP-IDF v5.5 semantics: polling_start takes the lock, polling_end gives it back only
  // when the transaction ended, remove_device refuses a device with a transaction in flight).
  std::array<bool, 3> spi_bus{};
  std::array<spi_device_t, 8> spi_devs{};
  std::array<int, 8> spi_dev_host{};
  std::array<int, 8> spi_dev_clock{};
  std::array<bool, 8> spi_dev_used{};
  std::array<int, 3> spi_holder{-1, -1, -1};
  std::array<uint64_t, 3> spi_remaining_us{};
  uint32_t spi_slow{1U};
  bool spi_stuck{false};
  uint32_t spi_would_block{0U};
  int heap_blocks{0};
  uint32_t sdk_bounces{0U};
  int spi_khz_override{-1};
  size_t heap_last_align{0U};
  size_t cache_align{0U};
  bool iram_all{false};
  std::vector<uint8_t> spi_reply;
  std::vector<uint8_t> spi_sent;
  uint32_t spi_transactions{0U};
  uint32_t spi_keep_cs{0U};
  int spi_clock{0};
  spi_dma_chan_t spi_dma{SPI_DMA_DISABLED};
  bool spi_pending{false};
  // UART
  std::array<fake_queue, 3> queues{};
  std::array<bool, 3> uart_installed{};
  uint32_t uart_queue_len{0U};
  uint32_t uart_events_dropped{0U};
  std::deque<uint8_t> uart_rx;
  std::vector<uint8_t> uart_tx;
  std::deque<uart_event_t> uart_events;
  uint32_t uart_fifo{128U};
  uint32_t uart_baud{0U};
  uart_mode_t uart_mode{UART_MODE_UART};
  TickType_t uart_read_ticks{0U};
  uart_config_t uart_config{};
  // LEDC
  uint64_t ledc_source{40000000U};
  uint32_t ledc_bits{0U};
  uint32_t ledc_freq{0U};
  std::array<uint32_t, 4> ledc_timer_bits{};
  std::array<uint32_t, 4> ledc_timer_freq{};
  std::array<int, 8> ledc_channel_timer{};
  std::array<bool, 8> ledc_inverted{};
  std::array<uint32_t, 8> ledc_duty{};
  std::array<bool, 8> ledc_running{};
  std::array<uint32_t, 8> ledc_idle{};
  // ADC
  int adc_raw{0};
  uint32_t adc_default_vref{0U};
  std::array<adc_oneshot_unit_ctx_t, 2> adc_units{};
  int adc_units_alive{0};
  std::array<adc_cali_scheme_t, 32> calis{};
  int calis_alive{0};
  // TWAI
  std::array<twai_obj_t, 2> twai{};
  int twai_alive{0};
  twai_state_t twai_state{TWAI_STATE_STOPPED};
  uint32_t twai_recovery_reads{2U};
  uint32_t twai_recovery_left{0U};
  std::deque<twai_message_t> twai_rx;
  std::vector<twai_message_t> twai_tx;
  uint32_t twai_quanta{0U};
  // log
  std::string log;
  esp_log_level_t log_level{ESP_LOG_NONE};
};

State& S() {
  static State state{};
  return state;
}

// Counts the call and returns true (with the injected code) when an injection is pending.
bool Injected(const char* function, esp_err_t& err) {
  State& s = S();
  ++s.calls[function];
  for (std::size_t h = 0U; h < s.hooks.size(); ++h) {
    if (s.hooks[h].function == function) {
      const PendingHook hook = s.hooks[h];
      s.hooks.erase(s.hooks.begin() + static_cast<std::ptrdiff_t>(h));
      hook.fn(hook.context);
      break;
    }
  }
  for (Injection& i : s.injections) {
    if ((i.count > 0U) && (i.function == function)) {
      --i.count;
      err = i.err;
      return true;
    }
  }
  return false;
}

#define FAKE_INJECT(name)                                                                          \
  do {                                                                                             \
    esp_err_t injected_ = ESP_OK;                                                                  \
    if (Injected(name, injected_)) {                                                               \
      return injected_;                                                                            \
    }                                                                                              \
  } while (false)

int Level(int pin) {
  const Pin& p = S().pins[static_cast<std::size_t>(pin)];
  return (p.forced >= 0) ? p.forced : p.driven;
}
} // namespace

void Reset() {
  S() = State{};
}
void Fail(const char* function, esp_err_t err, uint32_t count) {
  S().injections.push_back(Injection{function, err, count});
}
void Hook(const char* function, void (*fn)(void*), void* context) {
  S().hooks.push_back(PendingHook{function, fn, context});
}
uint32_t Calls(const char* function) {
  const auto it = S().calls.find(function);
  return (it == S().calls.end()) ? 0U : it->second;
}
void SetTimeStepUs(uint64_t step) {
  S().step = step;
}
uint64_t TimeUs() {
  return S().now;
}
void SetTimeUs(uint64_t us) {
  S().now = us;
}
void ForceLevel(int pin, int level) {
  S().pins[static_cast<std::size_t>(pin)].forced = level;
}
int DrivenLevel(int pin) {
  return S().pins[static_cast<std::size_t>(pin)].driven;
}
gpio_mode_t Mode(int pin) {
  return S().pins[static_cast<std::size_t>(pin)].mode;
}
gpio_int_type_t IrqType(int pin) {
  return S().pins[static_cast<std::size_t>(pin)].type;
}
bool IrqEnabled(int pin) {
  return S().pins[static_cast<std::size_t>(pin)].enabled;
}
void FireIrq(int pin) {
  const Pin& p = S().pins[static_cast<std::size_t>(pin)];
  if (p.enabled && (p.isr != nullptr)) {
    p.isr(p.arg);
  }
}
void I2cPresent(uint16_t address, bool present) {
  S().present[address & 0x7FU] = present;
}
void I2cReply(std::span<const uint8_t> bytes) {
  S().i2c_reply.assign(bytes.begin(), bytes.end());
}
const std::vector<uint8_t>& I2cWritten() {
  return S().i2c_written;
}
uint16_t I2cAddress() {
  return S().i2c_address;
}
uint32_t I2cClockHz() {
  return S().i2c_clock;
}
void I2cDataNack(bool on) {
  S().i2c_data_nack = on;
}
void I2cStretchUs(uint64_t us) {
  S().i2c_stretch_us = us;
}
int I2cLastTimeoutMs() {
  return S().i2c_last_ms;
}
uint32_t I2cStretchLimitUs() {
  return S().i2c_stretch_limit;
}
void SpiSlow(uint32_t factor) {
  S().spi_slow = factor;
}
void SpiStuck(bool on) {
  S().spi_stuck = on;
}
uint32_t SpiWouldBlock() {
  return S().spi_would_block;
}
bool SpiInFlight(int host) {
  return S().spi_holder[static_cast<std::size_t>(host)] >= 0;
}
int HeapBlocks() {
  return S().heap_blocks;
}
uint32_t SdkBounces() {
  return S().sdk_bounces;
}
void SpiReportKhz(int khz) {
  S().spi_khz_override = khz;
}
size_t HeapLastAlign() {
  return S().heap_last_align;
}
void CacheAlign(size_t bytes) {
  S().cache_align = bytes;
}
void IramAll(bool on) {
  S().iram_all = on;
}
uint32_t UartEventsDropped() {
  return S().uart_events_dropped;
}
uint32_t LedcTimerBits(int timer) {
  return S().ledc_timer_bits[static_cast<std::size_t>(timer)];
}
uint32_t LedcTimerFrequency(int timer) {
  return S().ledc_timer_freq[static_cast<std::size_t>(timer)];
}
bool LedcInverted(int channel) {
  return S().ledc_inverted[static_cast<std::size_t>(channel)];
}
uint32_t AdcDefaultVref() {
  return S().adc_default_vref;
}
void SpiReply(std::span<const uint8_t> bytes) {
  S().spi_reply.assign(bytes.begin(), bytes.end());
}
const std::vector<uint8_t>& SpiSent() {
  return S().spi_sent;
}
uint32_t SpiTransactions() {
  return S().spi_transactions;
}
uint32_t SpiKeepCs() {
  return S().spi_keep_cs;
}
int SpiClockHz() {
  return S().spi_clock;
}
spi_dma_chan_t SpiDma() {
  return S().spi_dma;
}
void UartFeed(std::span<const uint8_t> bytes) {
  S().uart_rx.insert(S().uart_rx.end(), bytes.begin(), bytes.end());
}
const std::vector<uint8_t>& UartWritten() {
  return S().uart_tx;
}
void UartFifoSpace(uint32_t bytes) {
  S().uart_fifo = bytes;
}
void UartEvent(uart_event_type_t type) {
  // As the SDK's ISR: an event that finds the queue full is dropped.
  if ((S().uart_queue_len > 0U) && (S().uart_events.size() >= S().uart_queue_len)) {
    ++S().uart_events_dropped;
  } else {
    S().uart_events.push_back(uart_event_t{type, 0U, 0});
  }
}
uint32_t UartBaud() {
  return S().uart_baud;
}
uart_mode_t UartMode() {
  return S().uart_mode;
}
TickType_t UartLastReadTicks() {
  return S().uart_read_ticks;
}
const uart_config_t& UartLastConfig() {
  return S().uart_config;
}
void LedcSourceHz(uint64_t hz) {
  S().ledc_source = hz;
}
uint32_t LedcBits() {
  return S().ledc_bits;
}
uint32_t LedcFrequency() {
  return S().ledc_freq;
}
uint32_t LedcDuty(int channel) {
  return S().ledc_duty[static_cast<std::size_t>(channel)];
}
bool LedcRunning(int channel) {
  return S().ledc_running[static_cast<std::size_t>(channel)];
}
uint32_t LedcIdle(int channel) {
  return S().ledc_idle[static_cast<std::size_t>(channel)];
}
void AdcRaw(int raw) {
  S().adc_raw = raw;
}
int AdcUnitsAlive() {
  return S().adc_units_alive;
}
int AdcCalisAlive() {
  return S().calis_alive;
}
void TwaiState(twai_state_t state) {
  S().twai_state = state;
}
void TwaiRecoveryReads(uint32_t reads) {
  S().twai_recovery_reads = reads;
}
void TwaiFeed(const twai_message_t& message) {
  S().twai_rx.push_back(message);
}
const std::vector<twai_message_t>& TwaiSent() {
  return S().twai_tx;
}
uint32_t TwaiBitrateQuanta() {
  return S().twai_quanta;
}
int TwaiHandlesAlive() {
  return S().twai_alive;
}
const std::string& LastLog() {
  return S().log;
}
esp_log_level_t LastLogLevel() {
  return S().log_level;
}

} // namespace fake

using fake::Injected;
using fake::S;

extern "C" {

// --- FreeRTOS
// --------------------------------------------------------------------------------------
BaseType_t xQueueReceive(QueueHandle_t, void* item, TickType_t) {
  esp_err_t err = ESP_OK;
  if (Injected("xQueueReceive", err) || S().uart_events.empty()) {
    return pdFALSE;
  }
  std::memcpy(item, &S().uart_events.front(), sizeof(uart_event_t));
  S().uart_events.pop_front();
  return pdTRUE;
}
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t) {
  return static_cast<UBaseType_t>(S().uart_events.size());
}
void vTaskDelay(TickType_t ticks) {
  esp_err_t err = ESP_OK;
  static_cast<void>(Injected("vTaskDelay", err));
  S().now += (uint64_t{ticks} * 1000000U) / configTICK_RATE_HZ;
}
void* heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t) {
  esp_err_t err = ESP_OK;
  if (Injected("heap_caps_aligned_alloc", err)) {
    return nullptr;
  }
  ++S().heap_blocks;
  S().heap_last_align = alignment;
  // Over-allocate and keep the block's origin just below the aligned address: portable to targets
  // whose C library has no aligned_alloc (the Cortex-M scan builds these fakes too).
  const size_t a = std::max(alignment, alignof(std::max_align_t)); // the host's minimum
  void* const raw = std::malloc(size + a + sizeof(void*));
  if (raw == nullptr) {
    return nullptr;
  }
  const uintptr_t base = reinterpret_cast<uintptr_t>(raw) + sizeof(void*);
  const uintptr_t aligned = (base + (a - 1U)) & ~(static_cast<uintptr_t>(a) - 1U);
  void* const origin_slot = reinterpret_cast<void*>(aligned - sizeof(void*));
  std::memcpy(origin_slot, &raw, sizeof(void*));
  return reinterpret_cast<void*>(aligned);
}
void heap_caps_free(void* ptr) {
  if (ptr == nullptr) {
    return;
  }
  --S().heap_blocks;
  void* raw = nullptr;
  std::memcpy(&raw, reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(ptr) - sizeof(void*)),
              sizeof(void*));
  std::free(raw);
}
esp_err_t esp_clk_tree_src_get_freq_hz(soc_module_clk_t, esp_clk_tree_src_freq_precision_t,
                                       uint32_t* freq) {
  FAKE_INJECT("esp_clk_tree_src_get_freq_hz");
  *freq = 40000000U;
  return ESP_OK;
}
uintptr_t fake_iram_low(void) {
  return 0U;
}
uintptr_t fake_iram_high(void) {
  return S().iram_all ? UINTPTR_MAX : 0U;
}
esp_err_t esp_cache_get_alignment(uint32_t, size_t* out_alignment) {
  *out_alignment = S().cache_align;
  return ESP_OK;
}

// --- GPIO
// ------------------------------------------------------------------------------------------
esp_err_t gpio_config(const gpio_config_t* config) {
  FAKE_INJECT("gpio_config");
  for (int pin = 0; pin < fake::kPins; ++pin) {
    if ((config->pin_bit_mask & (uint64_t{1U} << static_cast<unsigned>(pin))) != 0U) {
      S().pins[static_cast<std::size_t>(pin)].mode = config->mode;
    }
  }
  return ESP_OK;
}
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level) {
  FAKE_INJECT("gpio_set_level");
  S().pins[static_cast<std::size_t>(pin)].driven = (level != 0U) ? 1 : 0;
  return ESP_OK;
}
int gpio_get_level(gpio_num_t pin) {
  esp_err_t err = ESP_OK;
  static_cast<void>(Injected("gpio_get_level", err));
  return fake::Level(static_cast<int>(pin));
}
esp_err_t gpio_set_pull_mode(gpio_num_t, gpio_pull_mode_t) {
  FAKE_INJECT("gpio_set_pull_mode");
  return ESP_OK;
}
esp_err_t gpio_reset_pin(gpio_num_t pin) {
  FAKE_INJECT("gpio_reset_pin");
  S().pins[static_cast<std::size_t>(pin)].mode = GPIO_MODE_DISABLE;
  return ESP_OK;
}
esp_err_t gpio_install_isr_service(int) {
  FAKE_INJECT("gpio_install_isr_service");
  if (S().isr_service) {
    return ESP_ERR_INVALID_STATE;
  }
  S().isr_service = true;
  return ESP_OK;
}
esp_err_t gpio_isr_handler_add(gpio_num_t pin, gpio_isr_t handler, void* args) {
  FAKE_INJECT("gpio_isr_handler_add");
  S().pins[static_cast<std::size_t>(pin)].isr = handler;
  S().pins[static_cast<std::size_t>(pin)].arg = args;
  return ESP_OK;
}
esp_err_t gpio_isr_handler_remove(gpio_num_t pin) {
  FAKE_INJECT("gpio_isr_handler_remove");
  S().pins[static_cast<std::size_t>(pin)].isr = nullptr;
  return ESP_OK;
}
esp_err_t gpio_set_intr_type(gpio_num_t pin, gpio_int_type_t type) {
  FAKE_INJECT("gpio_set_intr_type");
  S().pins[static_cast<std::size_t>(pin)].type = type;
  return ESP_OK;
}
esp_err_t gpio_intr_enable(gpio_num_t pin) {
  FAKE_INJECT("gpio_intr_enable");
  S().pins[static_cast<std::size_t>(pin)].enabled = true;
  return ESP_OK;
}
esp_err_t gpio_intr_disable(gpio_num_t pin) {
  FAKE_INJECT("gpio_intr_disable");
  S().pins[static_cast<std::size_t>(pin)].enabled = false;
  return ESP_OK;
}

// --- I2C
// -------------------------------------------------------------------------------------------
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t* config, i2c_master_bus_handle_t* bus) {
  FAKE_INJECT("i2c_new_master_bus");
  *bus = &S().buses[static_cast<std::size_t>(config->i2c_port)];
  return ESP_OK;
}
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus, const i2c_device_config_t* config,
                                    i2c_master_dev_handle_t* dev) {
  FAKE_INJECT("i2c_master_bus_add_device");
  *dev = &S().devs[static_cast<std::size_t>(bus - S().buses.data())];
  S().i2c_address = config->device_address;
  S().i2c_clock = config->scl_speed_hz;
  S().i2c_stretch_limit = config->scl_wait_us;
  return ESP_OK;
}
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t) {
  FAKE_INJECT("i2c_del_master_bus");
  return ESP_OK;
}
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t) {
  FAKE_INJECT("i2c_master_bus_rm_device");
  return ESP_OK;
}
namespace {
#ifdef KB_FAKE_IDF_V6
constexpr esp_err_t kTransferFailed = ESP_ERR_INVALID_RESPONSE; // v6.0: every failed transfer
#else
constexpr esp_err_t kTransferFailed = ESP_ERR_INVALID_STATE; // v5.5: NACK, timeout, arbitration
#endif
// As ESP-IDF's synchronous transfer: a NACK ends it at once; a stretch past the SCL limit (or the
// software timeout) ends it later; both return the same code. ESP_ERR_TIMEOUT only for the lock.
esp_err_t I2cAck() {
  if (S().i2c_stretch_us > 0U) {
    S().now += S().i2c_stretch_us;
    S().i2c_stretch_us = 0U;
    return kTransferFailed;
  }
  if (!S().present[S().i2c_address & 0x7FU]) {
    return kTransferFailed;
  }
  return S().i2c_data_nack ? kTransferFailed : ESP_OK;
}
void I2cCopyReply(uint8_t* rx, size_t n) {
  for (size_t i = 0U; i < n; ++i) {
    rx[i] = (i < S().i2c_reply.size()) ? S().i2c_reply[i] : 0xFFU;
  }
}
} // namespace
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t* tx, size_t n, int ms) {
  S().i2c_last_ms = ms;
  FAKE_INJECT("i2c_master_transmit");
  S().i2c_written.assign(tx, tx + n);
  return I2cAck();
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t* tx, size_t n,
                                      uint8_t* rx, size_t m, int ms) {
  S().i2c_last_ms = ms;
  FAKE_INJECT("i2c_master_transmit_receive");
  S().i2c_written.assign(tx, tx + n);
  const esp_err_t err = I2cAck();
  if (err == ESP_OK) {
    I2cCopyReply(rx, m);
  }
  return err;
}
esp_err_t i2c_master_receive(i2c_master_dev_handle_t, uint8_t* rx, size_t m, int ms) {
  S().i2c_last_ms = ms;
  FAKE_INJECT("i2c_master_receive");
  const esp_err_t err = I2cAck();
  if (err == ESP_OK) {
    I2cCopyReply(rx, m);
  }
  return err;
}
esp_err_t i2c_master_probe(i2c_master_bus_handle_t, uint16_t address, int ms) {
  S().i2c_last_ms = ms;
  FAKE_INJECT("i2c_master_probe");
  return S().present[address & 0x7FU] ? ESP_OK : ESP_ERR_NOT_FOUND;
}
esp_err_t i2c_master_bus_reset(i2c_master_bus_handle_t) {
  FAKE_INJECT("i2c_master_bus_reset");
  return ESP_OK;
}
esp_err_t i2c_master_device_change_address(i2c_master_dev_handle_t, uint16_t address, int) {
  FAKE_INJECT("i2c_master_device_change_address");
  S().i2c_address = address;
  return ESP_OK;
}

// --- SPI
// -------------------------------------------------------------------------------------------
esp_err_t spi_bus_initialize(spi_host_device_t host, const spi_bus_config_t*, spi_dma_chan_t dma) {
  FAKE_INJECT("spi_bus_initialize");
  if (S().spi_bus[static_cast<std::size_t>(host)]) {
    return ESP_ERR_INVALID_STATE;
  }
  S().spi_bus[static_cast<std::size_t>(host)] = true;
  S().spi_dma = dma;
  return ESP_OK;
}
esp_err_t spi_bus_free(spi_host_device_t host) {
  FAKE_INJECT("spi_bus_free");
  S().spi_bus[static_cast<std::size_t>(host)] = false;
  return ESP_OK;
}
namespace {
int SpiIndex(spi_device_handle_t handle) {
  return static_cast<int>(handle - S().spi_devs.data());
}
} // namespace
esp_err_t spi_bus_add_device(spi_host_device_t host, const spi_device_interface_config_t* config,
                             spi_device_handle_t* handle) {
  FAKE_INJECT("spi_bus_add_device");
  std::size_t i = 0U;
  while ((i < S().spi_dev_used.size()) && S().spi_dev_used[i]) {
    ++i;
  }
  if (i == S().spi_dev_used.size()) {
    return ESP_ERR_NO_MEM;
  }
  S().spi_dev_used[i] = true;
  S().spi_dev_host[i] = static_cast<int>(host);
  S().spi_dev_clock[i] = config->clock_speed_hz;
  *handle = &S().spi_devs[i];
  S().spi_clock = config->clock_speed_hz;
  return ESP_OK;
}
esp_err_t spi_device_get_actual_freq(spi_device_handle_t handle, int* freq_khz) {
  FAKE_INJECT("spi_device_get_actual_freq");
  *freq_khz = (S().spi_khz_override >= 0)
                  ? S().spi_khz_override
                  : S().spi_dev_clock[static_cast<std::size_t>(SpiIndex(handle))] / 1000;
  return ESP_OK;
}
esp_err_t spi_bus_remove_device(spi_device_handle_t handle) {
  FAKE_INJECT("spi_bus_remove_device");
  const int i = SpiIndex(handle);
  const auto host = static_cast<std::size_t>(S().spi_dev_host[static_cast<std::size_t>(i)]);
  if (S().spi_holder[host] == i) {
    return ESP_ERR_INVALID_STATE; // "Have unfinished transactions"
  }
  S().spi_dev_used[static_cast<std::size_t>(i)] = false;
  return ESP_OK;
}
esp_err_t spi_device_polling_start(spi_device_handle_t handle, spi_transaction_t* t,
                                   TickType_t ticks) {
  FAKE_INJECT("spi_device_polling_start");
  if (ticks != portMAX_DELAY) {
    return ESP_ERR_INVALID_ARG; // "currently timeout is not available for polling transactions"
  }
  if ((t->flags & static_cast<uint32_t>(SPI_TRANS_CS_KEEP_ACTIVE)) != 0U) {
    return ESP_ERR_INVALID_ARG; // the bus is not acquired
  }
  const int i = SpiIndex(handle);
  const auto host = static_cast<std::size_t>(S().spi_dev_host[static_cast<std::size_t>(i)]);
  if (S().spi_holder[host] >= 0) {
    ++S().spi_would_block; // the real call waits for the SDK bus lock for ever
    return ESP_ERR_TIMEOUT;
  }
  const size_t n = t->length / 8U;
  const auto* tx = static_cast<const uint8_t*>(t->tx_buffer);
  auto* rx = static_cast<uint8_t*>(t->rx_buffer);
  if ((rx != nullptr) && (S().spi_dma != SPI_DMA_DISABLED) && (S().cache_align > 1U) &&
      (((reinterpret_cast<uintptr_t>(rx) | n) % S().cache_align) != 0U)) {
    // setup_dma_priv_buffer: on a cached chip a receive buffer whose start or length is not a whole
    // number of cache lines is bounced through a heap block for this transaction.
    esp_err_t nomem = ESP_OK;
    if (Injected("spi_sdk_bounce_alloc", nomem)) {
      return ESP_ERR_NO_MEM;
    }
    ++S().sdk_bounces;
  }
  const size_t offset = S().spi_sent.size();
  for (size_t k = 0U; k < n; ++k) {
    S().spi_sent.push_back(tx[k]);
    if (rx != nullptr) {
      const size_t at = offset + k;
      rx[k] = (at < S().spi_reply.size()) ? S().spi_reply[at] : 0x00U;
    }
  }
  if ((rx != nullptr) && (S().spi_dma != SPI_DMA_DISABLED)) {
    // ESP-IDF rounds a DMA receive descriptor up to whole words: the hardware writes past n.
    for (size_t k = n; (k % 4U) != 0U; ++k) {
      rx[k] = 0xEEU;
    }
  }
  ++S().spi_transactions;
  S().spi_pending = true;
  S().spi_holder[host] = i;
  const uint64_t clock = static_cast<uint64_t>(S().spi_dev_clock[static_cast<std::size_t>(i)]);
  S().spi_remaining_us[host] = ((uint64_t{t->length} * 1000000U) / clock + 1U) * S().spi_slow;
  return ESP_OK;
}
esp_err_t spi_device_polling_end(spi_device_handle_t handle, TickType_t ticks) {
  const int i = SpiIndex(handle);
  const auto host = static_cast<std::size_t>(S().spi_dev_host[static_cast<std::size_t>(i)]);
  esp_err_t injected = ESP_OK;
  if (Injected("spi_device_polling_end", injected)) {
    if (injected != ESP_ERR_TIMEOUT) { // any other code comes after the transaction ended
      S().spi_holder[host] = -1;
      S().spi_pending = false;
    }
    return injected;
  }
  if (S().spi_holder[host] != i) {
    return ESP_ERR_INVALID_STATE; // asserts in the real SDK
  }
  const uint64_t budget = (uint64_t{ticks} * 1000000U) / configTICK_RATE_HZ;
  if (S().spi_stuck || (S().spi_remaining_us[host] > budget)) {
    S().now += budget;
    if (!S().spi_stuck) {
      S().spi_remaining_us[host] -= budget;
    }
    return ESP_ERR_TIMEOUT; // still in flight, the lock stays taken
  }
  S().now += S().spi_remaining_us[host];
  S().spi_remaining_us[host] = 0U;
  S().spi_holder[host] = -1;
  S().spi_pending = false;
  return ESP_OK;
}
esp_err_t spi_device_acquire_bus(spi_device_handle_t handle, TickType_t ticks) {
  FAKE_INJECT("spi_device_acquire_bus");
  if (ticks != portMAX_DELAY) {
    return ESP_ERR_INVALID_ARG; // "acquire finite time not supported now"
  }
  const int i = SpiIndex(handle);
  if (S().spi_holder[static_cast<std::size_t>(S().spi_dev_host[static_cast<std::size_t>(i)])] >=
      0) {
    ++S().spi_would_block;
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}
void spi_device_release_bus(spi_device_handle_t) {
  esp_err_t err = ESP_OK;
  static_cast<void>(Injected("spi_device_release_bus", err));
}

// --- UART
// ------------------------------------------------------------------------------------------
esp_err_t uart_driver_install(uart_port_t port, int, int, int queue_size, QueueHandle_t* queue,
                              int) {
  FAKE_INJECT("uart_driver_install");
  if (S().uart_installed[static_cast<std::size_t>(port)]) {
    return ESP_FAIL; // "UART driver already installed", the queue untouched
  }
  S().uart_queue_len = static_cast<uint32_t>(queue_size);
  S().uart_installed[static_cast<std::size_t>(port)] = true;
  *queue = &S().queues[static_cast<std::size_t>(port)];
  return ESP_OK;
}
esp_err_t uart_driver_delete(uart_port_t port) {
  FAKE_INJECT("uart_driver_delete");
  S().uart_installed[static_cast<std::size_t>(port)] = false;
  return ESP_OK;
}
esp_err_t uart_param_config(uart_port_t, const uart_config_t* config) {
  FAKE_INJECT("uart_param_config");
  S().uart_config = *config;
  S().uart_baud = static_cast<uint32_t>(config->baud_rate);
  return ESP_OK;
}
esp_err_t uart_set_pin(uart_port_t, int, int, int, int) {
  FAKE_INJECT("uart_set_pin");
  return ESP_OK;
}
esp_err_t uart_set_mode(uart_port_t, uart_mode_t mode) {
  FAKE_INJECT("uart_set_mode");
  S().uart_mode = mode;
  return ESP_OK;
}
int uart_tx_chars(uart_port_t, const char* buffer, uint32_t len) {
  esp_err_t err = ESP_OK;
  if (Injected("uart_tx_chars", err)) {
    return -1;
  }
  const uint32_t n = std::min(len, S().uart_fifo);
  const std::size_t old = S().uart_tx.size();
  S().uart_tx.resize(old + n);
  if (n > 0U) {
    std::memcpy(S().uart_tx.data() + old, buffer, n);
  }
  return static_cast<int>(n);
}
int uart_read_bytes(uart_port_t, void* buf, uint32_t length, TickType_t ticks) {
  esp_err_t err = ESP_OK;
  if (Injected("uart_read_bytes", err)) {
    return -1;
  }
  S().uart_read_ticks = ticks;
  auto* out = static_cast<uint8_t*>(buf);
  uint32_t n = 0U;
  while ((n < length) && !S().uart_rx.empty()) {
    out[n++] = S().uart_rx.front();
    S().uart_rx.pop_front();
  }
  return static_cast<int>(n);
}
esp_err_t uart_get_buffered_data_len(uart_port_t, size_t* size) {
  FAKE_INJECT("uart_get_buffered_data_len");
  *size = S().uart_rx.size();
  return ESP_OK;
}
esp_err_t uart_flush_input(uart_port_t) {
  FAKE_INJECT("uart_flush_input");
  S().uart_rx.clear();
  return ESP_OK;
}
esp_err_t uart_set_baudrate(uart_port_t, uint32_t baud) {
  FAKE_INJECT("uart_set_baudrate");
  S().uart_baud = baud;
  return ESP_OK;
}
esp_err_t uart_wait_tx_done(uart_port_t, TickType_t) {
  FAKE_INJECT("uart_wait_tx_done");
  return ESP_OK;
}

// --- LEDC
// ------------------------------------------------------------------------------------------
esp_err_t ledc_timer_config(const ledc_timer_config_t* config) {
  FAKE_INJECT("ledc_timer_config");
  const unsigned bits = static_cast<unsigned>(config->duty_resolution);
  if ((uint64_t{config->freq_hz} << bits) > S().ledc_source) {
    return ESP_FAIL; // the divider is out of range at this resolution
  }
  S().ledc_bits = bits;
  S().ledc_freq = config->freq_hz;
  S().ledc_timer_bits[static_cast<std::size_t>(config->timer_num)] = bits;
  S().ledc_timer_freq[static_cast<std::size_t>(config->timer_num)] = config->freq_hz;
  return ESP_OK;
}
esp_err_t ledc_channel_config(const ledc_channel_config_t* config) {
  FAKE_INJECT("ledc_channel_config");
  S().ledc_running[static_cast<std::size_t>(config->channel)] = true;
  S().ledc_duty[static_cast<std::size_t>(config->channel)] = config->duty;
  S().ledc_channel_timer[static_cast<std::size_t>(config->channel)] =
      static_cast<int>(config->timer_sel);
  S().ledc_inverted[static_cast<std::size_t>(config->channel)] = config->flags.output_invert != 0U;
  return ESP_OK;
}
esp_err_t ledc_set_duty(ledc_mode_t, ledc_channel_t channel, uint32_t duty) {
  FAKE_INJECT("ledc_set_duty"); // like ESP-IDF v5.5: no check against the timer's resolution
  S().ledc_duty[static_cast<std::size_t>(channel)] = duty;
  return ESP_OK;
}
esp_err_t ledc_update_duty(ledc_mode_t, ledc_channel_t channel) {
  FAKE_INJECT("ledc_update_duty");
  S().ledc_running[static_cast<std::size_t>(channel)] = true;
  return ESP_OK;
}
esp_err_t ledc_stop(ledc_mode_t, ledc_channel_t channel, uint32_t idle) {
  FAKE_INJECT("ledc_stop");
  S().ledc_running[static_cast<std::size_t>(channel)] = false;
  S().ledc_idle[static_cast<std::size_t>(channel)] = idle;
  return ESP_OK;
}

// --- ADC
// -------------------------------------------------------------------------------------------
esp_err_t adc_oneshot_io_to_channel(int io, adc_unit_t* unit, adc_channel_t* channel) {
  FAKE_INJECT("adc_oneshot_io_to_channel");
  if ((io < 1) || (io > 20)) {
    return ESP_ERR_NOT_FOUND;
  }
  *unit = (io <= 10) ? ADC_UNIT_1 : ADC_UNIT_2;
  *channel = static_cast<adc_channel_t>((io - 1) % 10);
  return ESP_OK;
}
esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t* config,
                               adc_oneshot_unit_handle_t* handle) {
  FAKE_INJECT("adc_oneshot_new_unit");
  *handle = &S().adc_units[static_cast<std::size_t>(config->unit_id)];
  ++S().adc_units_alive;
  return ESP_OK;
}
esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t, adc_channel_t,
                                     const adc_oneshot_chan_cfg_t*) {
  FAKE_INJECT("adc_oneshot_config_channel");
  return ESP_OK;
}
esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t, adc_channel_t, int* raw) {
  FAKE_INJECT("adc_oneshot_read");
  *raw = S().adc_raw;
  return ESP_OK;
}
esp_err_t adc_oneshot_del_unit(adc_oneshot_unit_handle_t) {
  FAKE_INJECT("adc_oneshot_del_unit");
  --S().adc_units_alive;
  return ESP_OK;
}
namespace {
esp_err_t CaliNew(adc_cali_handle_t* handle) {
  *handle = &S().calis[static_cast<std::size_t>(S().calis_alive % 32)];
  ++S().calis_alive;
  return ESP_OK;
}
} // namespace
esp_err_t adc_cali_create_scheme_curve_fitting(const adc_cali_curve_fitting_config_t*,
                                               adc_cali_handle_t* handle) {
  FAKE_INJECT("adc_cali_create_scheme_curve_fitting");
  return CaliNew(handle);
}
esp_err_t adc_cali_delete_scheme_curve_fitting(adc_cali_handle_t) {
  FAKE_INJECT("adc_cali_delete_scheme_curve_fitting");
  --S().calis_alive;
  return ESP_OK;
}
esp_err_t adc_cali_create_scheme_line_fitting(const adc_cali_line_fitting_config_t* config,
                                              adc_cali_handle_t* handle) {
  S().adc_default_vref = config->default_vref;
  FAKE_INJECT("adc_cali_create_scheme_line_fitting");
  return CaliNew(handle);
}
esp_err_t adc_cali_delete_scheme_line_fitting(adc_cali_handle_t) {
  FAKE_INJECT("adc_cali_delete_scheme_line_fitting");
  --S().calis_alive;
  return ESP_OK;
}
esp_err_t adc_cali_raw_to_voltage(adc_cali_handle_t, int raw, int* voltage) {
  FAKE_INJECT("adc_cali_raw_to_voltage");
  *voltage = (raw * 3100) / 4095;
  return ESP_OK;
}

// --- TWAI
// ------------------------------------------------------------------------------------------
esp_err_t twai_driver_install_v2(const twai_general_config_t* general,
                                 const twai_timing_config_t* timing, const twai_filter_config_t*,
                                 twai_handle_t* handle) {
  FAKE_INJECT("twai_driver_install_v2");
  *handle = &S().twai[static_cast<std::size_t>(general->controller_id)];
  S().twai_quanta = timing->quanta_resolution_hz;
  ++S().twai_alive;
  S().twai_state = TWAI_STATE_STOPPED;
  return ESP_OK;
}
esp_err_t twai_driver_uninstall_v2(twai_handle_t) {
  FAKE_INJECT("twai_driver_uninstall_v2");
  --S().twai_alive;
  return ESP_OK;
}
esp_err_t twai_start_v2(twai_handle_t) {
  FAKE_INJECT("twai_start_v2");
  if (S().twai_state != TWAI_STATE_STOPPED) {
    return ESP_ERR_INVALID_STATE;
  }
  S().twai_state = TWAI_STATE_RUNNING;
  return ESP_OK;
}
esp_err_t twai_stop_v2(twai_handle_t) {
  FAKE_INJECT("twai_stop_v2");
  S().twai_state = TWAI_STATE_STOPPED;
  return ESP_OK;
}
esp_err_t twai_transmit_v2(twai_handle_t, const twai_message_t* message, TickType_t) {
  FAKE_INJECT("twai_transmit_v2");
  if (S().twai_state != TWAI_STATE_RUNNING) {
    return ESP_ERR_INVALID_STATE;
  }
  S().twai_tx.push_back(*message);
  return ESP_OK;
}
esp_err_t twai_receive_v2(twai_handle_t, twai_message_t* message, TickType_t) {
  FAKE_INJECT("twai_receive_v2");
  if (S().twai_rx.empty()) {
    return ESP_ERR_TIMEOUT;
  }
  *message = S().twai_rx.front();
  S().twai_rx.pop_front();
  return ESP_OK;
}
esp_err_t twai_get_status_info_v2(twai_handle_t, twai_status_info_t* info) {
  FAKE_INJECT("twai_get_status_info_v2");
  if ((S().twai_state == TWAI_STATE_RECOVERING) && (S().twai_recovery_left > 0U)) {
    --S().twai_recovery_left;
    if (S().twai_recovery_left == 0U) {
      S().twai_state = TWAI_STATE_STOPPED;
    }
  }
  *info = twai_status_info_t{};
  info->state = S().twai_state;
  info->tx_error_counter = 7U;
  info->rx_error_counter = 3U;
  info->bus_error_count = 1U;
  return ESP_OK;
}
esp_err_t twai_initiate_recovery_v2(twai_handle_t) {
  FAKE_INJECT("twai_initiate_recovery_v2");
  if (S().twai_state != TWAI_STATE_BUS_OFF) {
    return ESP_ERR_INVALID_STATE;
  }
  S().twai_state = TWAI_STATE_RECOVERING;
  S().twai_recovery_left = S().twai_recovery_reads;
  return ESP_OK;
}

// --- time, log
// -------------------------------------------------------------------------------------
int64_t esp_timer_get_time(void) {
  const uint64_t now = S().now;
  S().now += S().step;
  return static_cast<int64_t>(now);
}
void esp_rom_delay_us(uint32_t us) {
  esp_err_t err = ESP_OK;
  static_cast<void>(Injected("esp_rom_delay_us", err));
  S().now += us;
}
uint32_t esp_log_timestamp(void) {
  return static_cast<uint32_t>(S().now / 1000U);
}
void esp_log_write(esp_log_level_t level, const char*, const char* format, ...) {
  std::array<char, 512> line{};
  va_list args;
  va_start(args, format);
  static_cast<void>(std::vsnprintf(line.data(), line.size(), format, args));
  va_end(args);
  S().log = line.data();
  S().log_level = level;
  ++S().calls["esp_log_write"];
}

} // extern "C"
