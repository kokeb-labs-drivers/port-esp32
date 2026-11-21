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
#include "esp_intr_alloc.h"
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
// polling_start whose polling_end timed out, and a queued transaction until its result was taken
// (ESP-IDF keeps a pointer to it until the end).
struct SpiSlot {
  spi_device_handle_t handle{nullptr};
  spi_transaction_t transaction{};
};

// The interrupt-allocation flag of a level (0: ESP-IDF's default, which may share a level 1 to 3).
[[nodiscard]] int32_t IntrFlags(uint8_t level) noexcept {
  int32_t flags = 0;
  if (level == 1U) {
    flags = ESP_INTR_FLAG_LEVEL1;
  } else if (level == 2U) {
    flags = ESP_INTR_FLAG_LEVEL2;
  } else if (level == 3U) {
    flags = ESP_INTR_FLAG_LEVEL3;
  } else {
    flags = 0;
  }
  return flags;
}

// The cache line in front of DMA-capable internal RAM (0 or 1 when there is none).
[[nodiscard]] std::size_t InternalCacheLine() noexcept {
  std::size_t cache = 0U;
  if (esp_cache_get_alignment(MALLOC_CAP_DMA, &cache) != ESP_OK) {
    cache = 0U;
  }
  return cache;
}

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

int32_t SpiBusOpen(const SpiBusConfig& config, bool dma) noexcept {
  spi_bus_config_t bus{};
  bus.mosi_io_num = config.mosi_pin;
  bus.miso_io_num = config.miso_pin;
  bus.sclk_io_num = config.sclk_pin;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = dma ? static_cast<int>(kSpiMaxDmaFrameBytes) : 0;
  bus.flags = SPICOMMON_BUSFLAG_MASTER;
  bus.intr_flags = static_cast<int>(IntrFlags(config.interrupt_level));
  return spi_bus_initialize(SpiHost(config.host), &bus, dma ? SPI_DMA_CH_AUTO : SPI_DMA_DISABLED);
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

int32_t SpiStart(uint8_t slot, std::span<const uint8_t> tx, std::span<uint8_t> rx,
                 bool dma) noexcept {
  SpiSlot& s = SpiSlots()[slot];
  const std::size_t n = tx.empty() ? rx.size() : tx.size();
  s.transaction = spi_transaction_t{};
  // With DMA the buffers are the host's staging: ESP-IDF must refuse rather than allocate a copy.
  s.transaction.flags = dma ? static_cast<uint32_t>(SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL) : 0U;
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
  const std::size_t align = std::max(kSpiDmaAlign, InternalCacheLine());
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

std::size_t SpiDmaRxUnit() noexcept {
  return std::max(std::size_t{1U}, InternalCacheLine());
}

int32_t SpiQueue(uint8_t slot, std::span<const std::byte> tx, std::span<std::byte> rx,
                 std::size_t bytes) noexcept {
  SpiSlot& s = SpiSlots()[slot];
  s.transaction = spi_transaction_t{};
  s.transaction.flags = static_cast<uint32_t>(SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL);
  s.transaction.length = bytes * 8U;
  s.transaction.rxlength = rx.empty() ? 0U : (bytes * 8U);
  s.transaction.tx_buffer = tx.empty() ? nullptr : tx.data();
  s.transaction.rx_buffer = rx.empty() ? nullptr : rx.data();
  // The queue has room for one transaction and the port queues one at a time: no wait.
  return spi_device_queue_trans(s.handle, &s.transaction, 0U);
}

int32_t SpiQueueResult(uint8_t slot, uint32_t timeout_ms, uint32_t& flags) noexcept {
  SpiSlot& s = SpiSlots()[slot];
  spi_transaction_t* done = nullptr;
  const TickType_t ticks = (timeout_ms == 0U) ? TickType_t{0U} : (TicksCeil(timeout_ms) + 1U);
  const esp_err_t err = spi_device_get_trans_result(s.handle, &done, ticks);
  flags = 0U;
  if ((err == ESP_OK) || (err == ESP_ERR_INVALID_STATE)) {
    flags = dma_flags::kComplete;
    constexpr uint32_t kLost = SPI_TRANS_DMA_RX_FAIL | SPI_TRANS_DMA_TX_FAIL;
    if ((done != nullptr) && ((done->flags & kLost) != 0U)) {
      flags = dma_flags::kTransferError | dma_flags::kFifoError; // DMA overflow or underflow
    }
  }
  return err;
}

int32_t SpiHostReset(uint8_t host) noexcept {
  static_cast<void>(host);
  return ESP_ERR_NOT_SUPPORTED; // ESP-IDF v5.5 cannot stop a started transaction
}

} // namespace kb::port::esp32::sdk
