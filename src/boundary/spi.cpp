// Boundary file (deviations/boundary-files.yml): the SPI master calls of ESP-IDF v5.5 behind
// sdk.hpp. Each function converts its arguments, calls the SDK and returns its esp_err_t; the SDK
// handles, the transaction descriptors and the DMA buffers live in the tables below, indexed by
// host number or slot. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "freertos/FreeRTOS.h"
#include "kb/port/esp32/error.hpp"
#include "kb/port/esp32/spi.hpp"
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

constexpr std::size_t kSpiDevices = 8U;
constexpr std::size_t kSpiZeroBytes = 64U;
constexpr std::size_t kSpiDmaAlign = 4U;
static_assert(SOC_SPI_MAXIMUM_BUFFER_SIZE >= kSpiMaxFrameBytes, "the FIFO holds a whole frame");

// One device handle and the descriptor of its transaction per slot: the descriptor must outlive a
// polling_start whose polling_end timed out (ESP-IDF keeps a pointer to it until the end).
struct SpiSlot {
  spi_device_handle_t handle{nullptr};
  spi_transaction_t transaction{};
};

std::array<SpiSlot, kSpiDevices>& SpiSlots() noexcept {
  // codeql::misra_deviation_next_line(DEV-001)
  static std::array<SpiSlot, kSpiDevices> slots{};
  return slots;
}

// The DMA buffer of each host (SpiDmaBufferOpen), allocated once and kept until spi_bus_free.
struct SpiDmaBuffer {
  void* block{nullptr};
  std::size_t half{0U}; // bytes of each half, as allocated
};

std::array<SpiDmaBuffer, SOC_SPI_PERIPH_NUM>& SpiDmaBuffers() noexcept {
  // codeql::misra_deviation_next_line(DEV-001)
  static std::array<SpiDmaBuffer, SOC_SPI_PERIPH_NUM> buffers{};
  return buffers;
}

// Clocked out when a Transfer has no transmit data: in internal RAM, word aligned (DMA capable).
std::array<uint8_t, kSpiZeroBytes>& SpiZeros() noexcept {
  // codeql::misra_deviation_next_line(DEV-001)
  alignas(4) static std::array<uint8_t, kSpiZeroBytes> zeros{};
  return zeros;
}

[[nodiscard]] spi_host_device_t SpiHost(uint8_t host) noexcept {
  return static_cast<spi_host_device_t>(host);
}
} // namespace

uint8_t SpiHostLast() noexcept {
  return static_cast<uint8_t>(SOC_SPI_PERIPH_NUM - 1);
}
std::size_t SpiZeroBytes() noexcept {
  return kSpiZeroBytes;
}

int32_t SpiBusOpen(const SpiBusConfig& config) noexcept {
  spi_bus_config_t bus{};
  bus.mosi_io_num = config.mosi_pin;
  bus.miso_io_num = config.miso_pin;
  bus.sclk_io_num = config.sclk_pin;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = config.dma ? static_cast<int>(config.dma_buffer_bytes) : 0;
  bus.flags = SPICOMMON_BUSFLAG_MASTER;
  return spi_bus_initialize(SpiHost(config.host), &bus,
                            config.dma ? SPI_DMA_CH_AUTO : SPI_DMA_DISABLED);
}

int32_t SpiBusClose(uint8_t host) noexcept {
  return spi_bus_free(SpiHost(host));
}

uint8_t SpiSlotCount() noexcept {
  return static_cast<uint8_t>(kSpiDevices);
}

int32_t SpiDeviceAdd(uint8_t host, const SpiDeviceConfig& config, uint32_t clock_hz, uint8_t slot,
                     uint32_t& actual_hz) noexcept {
  SpiSlot& s = SpiSlots()[slot];
  spi_device_interface_config_t dev{};
  dev.mode = config.mode;
  dev.clock_speed_hz = static_cast<int>(clock_hz);
  dev.spics_io_num = config.cs_pin;
  dev.cs_ena_pretrans = config.cs_setup_cycles;
  dev.cs_ena_posttrans = config.cs_hold_cycles;
  dev.queue_size = 1;
  esp_err_t err = spi_bus_add_device(SpiHost(host), &dev, &s.handle);
  if (err == ESP_OK) {
    // codeql::misra_deviation_next_line(DEV-004)
    int khz = 0;
    actual_hz = (spi_device_get_actual_freq(s.handle, &khz) == ESP_OK)
                    ? (static_cast<uint32_t>(khz) * 1000U)
                    : clock_hz;
  } else {
    s.handle = nullptr;
  }
  return err;
}

int32_t SpiDeviceRemove(uint8_t slot) noexcept {
  SpiSlot& s = SpiSlots()[slot];
  const esp_err_t err = spi_bus_remove_device(s.handle);
  if (err == ESP_OK) {
    s.handle = nullptr;
  }
  return err;
}

int32_t SpiStart(uint8_t slot, std::span<const uint8_t> tx, std::span<uint8_t> rx) noexcept {
  SpiSlot& s = SpiSlots()[slot];
  const std::size_t n = tx.empty() ? rx.size() : tx.size();
  s.transaction = spi_transaction_t{};
  s.transaction.length = n * 8U;
  s.transaction.tx_buffer = tx.empty() ? SpiZeros().data() : tx.data();
  s.transaction.rx_buffer = rx.empty() ? nullptr : rx.data();
  // ESP-IDF v5.5 takes only portMAX_DELAY here: it is the wait for the SDK's bus lock, which is
  // free under the port's bus lock on a host the port owns (spi.hpp).
  return spi_device_polling_start(s.handle, &s.transaction, portMAX_DELAY);
}

int32_t SpiEnd(uint8_t slot, uint32_t timeout_ms) noexcept {
  return spi_device_polling_end(SpiSlots()[slot].handle, TicksCeil(timeout_ms) + 1U);
}

int32_t SpiDmaBufferOpen(uint8_t host, std::size_t bytes, std::span<uint8_t>& tx,
                         std::span<uint8_t>& rx) noexcept {
  std::size_t cache = 0U;
  if (esp_cache_get_alignment(MALLOC_CAP_DMA, &cache) != ESP_OK) {
    cache = 0U;
  }
  const std::size_t align = std::max(kSpiDmaAlign, cache);
  const std::size_t half = ((bytes + align - 1U) / align) * align; // whole words and cache lines
  SpiDmaBuffer& buffer = SpiDmaBuffers()[host];
  void*& block = buffer.block;
  esp_err_t err = ESP_OK;
  if ((block != nullptr) && (buffer.half != half)) {
    heap_caps_free(block); // another size: never hand out a block smaller than asked
    block = nullptr;
  }
  if (block == nullptr) {
    block = heap_caps_aligned_alloc(align, 2U * half, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    buffer.half = (block != nullptr) ? half : 0U;
  }
  if (block == nullptr) {
    err = ESP_ERR_NO_MEM;
  } else {
    // codeql::misra_deviation_next_line(DEV-005)
    const std::span<uint8_t> all{static_cast<uint8_t*>(block), 2U * half};
    tx = all.first(bytes);
    rx = all.subspan(half, bytes);
  }
  return err;
}

void SpiDmaBufferClose(uint8_t host) noexcept {
  SpiDmaBuffer& buffer = SpiDmaBuffers()[host];
  heap_caps_free(buffer.block);
  buffer = SpiDmaBuffer{};
}

} // namespace kb::port::esp32::sdk
