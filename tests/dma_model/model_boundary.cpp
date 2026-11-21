// The model boundary: a second implementation of the boundary calls that the port's DMA logic makes
// (src/sdk.hpp: the SPI, DMA-memory, cache and ADC continuous sections), over kb-core's host
// DMA-and-cache model (kb::dma::host::Machine, DmaEngine) instead of ESP-IDF. A link seam, like the
// ESP-IDF fakes; the conformance rigs (port_rig.hpp) drive it. It behaves as ESP-IDF v5.5 does
// where the port depends on it: a started SPI transaction cannot be stopped, only waited for; a
// queued transaction's end is reported once; the receive descriptor may round up (the engine's
// knob). Test code. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/dma/host/engine.hpp"
#include "kb/dma/host/machine.hpp"
#include "kb/port/esp32/error.hpp"
#include "model.hpp"
#include "sdk.hpp"

namespace esp = kb::port::esp32;
namespace h = kb::dma::host;

namespace {

struct Job {
  bool active{false};
  bool queued{false};
  std::byte* rx{nullptr}; // the receive that follows the transmit (polling full duplex)
  std::size_t rx_len{0U};
  bool rx_started{false};
};

struct Model {
  h::Machine* machine{nullptr};
  h::DmaEngine* engine{nullptr};
  uint8_t seed{0U};
  bool stalled{false};
  std::size_t ticks_per_ms{64U};
  std::size_t rx_unit{1U};
  std::array<bool, 3> bus_open{};
  Job job{};
  int job_slot{-1};
  esp::AdcStreamSink* sink{nullptr};
  bool adc_open{false};
  uint32_t adc_starts{0U};
  uint32_t resets{0U};
};

Model& M() {
  static Model m{};
  return m;
}

std::byte* Bytes(const uint8_t* p) {
  return reinterpret_cast<std::byte*>(const_cast<uint8_t*>(p));
}

// One engine cycle unless the transaction is stalled.
void Step() {
  if (!M().stalled) {
    M().engine->Tick();
  }
}

void Run(std::size_t ticks) {
  for (std::size_t i = 0U; (i < ticks) && M().engine->Enabled() && !M().stalled; ++i) {
    Step();
  }
}

// Advances the job by at most `ticks` engine cycles; true when everything ended.
bool Advance(std::size_t ticks) {
  Model& m = M();
  Run(ticks);
  if (!m.engine->Enabled() && !m.job.rx_started && (m.job.rx != nullptr)) {
    m.engine->ClearFlags(h::kFlagComplete);
    m.job.rx_started = true;
    // misra-lint: ignore 0.1.2
    m.engine->Start(kb::dma::Direction::FromDevice, m.job.rx, m.job.rx_len, m.seed);
    Run(ticks);
  }
  return !m.engine->Enabled();
}

// The SDK's interrupt: the flags of the ended transaction, each cleared on its own.
uint32_t TakeFlags() {
  const uint32_t raw = M().engine->Flags();
  uint32_t flags = 0U;
  if ((raw & h::kFlagTransferError) != 0U) {
    flags |= esp::sdk::dma_flags::kTransferError;
  }
  if ((raw & h::kFlagFifoError) != 0U) {
    flags |= esp::sdk::dma_flags::kFifoError;
  }
  if (flags == 0U) {
    flags = esp::sdk::dma_flags::kComplete;
  }
  M().engine->ClearFlags(h::kFlagTransferError);
  M().engine->ClearFlags(h::kFlagFifoError);
  M().engine->ClearFlags(h::kFlagComplete);
  return flags;
}

void EndJob() {
  M().job = Job{};
  M().job_slot = -1;
  M().machine->ClearWindows();
}

} // namespace

namespace model {
void Attach(h::Machine& machine, h::DmaEngine& engine) {
  M() = Model{};
  M().machine = &machine;
  M().engine = &engine;
}
void NextSeed(uint8_t seed) {
  M().seed = seed;
}
void Stall(bool on) {
  M().stalled = on;
}
bool Stalled() {
  return M().stalled;
}
void TicksPerMs(std::size_t ticks) {
  M().ticks_per_ms = ticks;
}
void RxUnit(std::size_t unit) {
  M().rx_unit = unit;
}
esp::AdcStreamSink* Sink() {
  return M().sink;
}
uint32_t AdcStarts() {
  return M().adc_starts;
}
uint32_t HostResets() {
  return M().resets;
}
} // namespace model

namespace kb::port::esp32::sdk {

bool GpioValidOutput(int16_t pin) noexcept {
  return (pin >= 0) && (pin < 64);
}
uint8_t SpiHostLast() noexcept {
  return 2U;
}
uint8_t SpiSlotCount() noexcept {
  return 8U;
}
std::size_t SpiZeroBytes() noexcept {
  return 64U;
}
std::size_t SpiDmaRxUnit() noexcept {
  return M().rx_unit;
}

int32_t SpiBusOpen(const SpiBusConfig& config, bool dma) noexcept {
  static_cast<void>(dma);
  int32_t err = esp_err::kOk;
  if (M().bus_open[config.host]) {
    err = esp_err::kInvalidState;
  } else {
    M().bus_open[config.host] = true;
  }
  return err;
}
int32_t SpiBusClose(uint8_t host) noexcept {
  M().bus_open[host] = false;
  return esp_err::kOk;
}
int32_t SpiDeviceAdd(uint8_t host, const SpiDeviceConfig& config, uint32_t clock_hz, uint8_t slot,
                     uint32_t& actual_hz) noexcept {
  static_cast<void>(host);
  static_cast<void>(config);
  static_cast<void>(slot);
  actual_hz = clock_hz;
  return esp_err::kOk;
}
int32_t SpiDeviceRemove(uint8_t slot) noexcept {
  return (M().job_slot == static_cast<int>(slot)) ? esp_err::kInvalidState : esp_err::kOk;
}

int32_t SpiDmaBufferOpen(uint8_t host, std::size_t bytes, std::span<uint8_t>& tx,
                         std::span<uint8_t>& rx) noexcept {
  static_cast<void>(host);
  static_cast<void>(bytes);
  static_cast<void>(tx);
  static_cast<void>(rx);
  return esp_err::kNoMem; // the rigs give the bus a static staging buffer
}
void SpiDmaBufferClose(uint8_t host) noexcept {
  static_cast<void>(host);
}

int32_t SpiStart(uint8_t slot, std::span<const uint8_t> tx, std::span<uint8_t> rx,
                 bool dma) noexcept {
  Model& m = M();
  int32_t err = esp_err::kOk;
  if (m.job.active) {
    err = esp_err::kInvalidState;
  } else if (!dma) {
    // The FIFO path: the CPU copies; no DMA, no cache maintenance.
    for (std::size_t i = 0U; i < rx.size(); ++i) {
      rx[i] = static_cast<uint8_t>(m.seed + i);
    }
    m.job.active = true;
    m.job_slot = slot;
  } else {
    // The staging halves are the DMA's while the transaction runs (padding included).
    constexpr std::size_t kPad = 64U;
    static_cast<void>(m.machine->OpenWindow(Bytes(tx.data()), kb::dma::RoundUp(tx.size(), kPad)));
    if (!rx.empty()) {
      static_cast<void>(m.machine->OpenWindow(Bytes(rx.data()), kb::dma::RoundUp(rx.size(), kPad)));
    }
    m.job.active = true;
    m.job_slot = slot;
    m.job.rx = rx.empty() ? nullptr : Bytes(rx.data());
    m.job.rx_len = rx.size();
    // misra-lint: ignore 0.1.2
    m.engine->Start(kb::dma::Direction::ToDevice, Bytes(tx.data()), tx.size(), 0U);
  }
  return err;
}

int32_t SpiEnd(uint8_t slot, uint32_t timeout_ms) noexcept {
  static_cast<void>(slot);
  Model& m = M();
  int32_t err = esp_err::kOk;
  if (m.job.active && !m.job.queued) {
    if (Advance(timeout_ms * m.ticks_per_ms)) {
      static_cast<void>(TakeFlags());
      EndJob();
    } else {
      err = esp_err::kTimeout;
    }
  }
  return err;
}

int32_t SpiQueue(uint8_t slot, std::span<const std::byte> tx, std::span<std::byte> rx,
                 std::size_t bytes) noexcept {
  Model& m = M();
  int32_t err = esp_err::kOk;
  if (m.job.active) {
    err = esp_err::kInvalidState;
  } else {
    m.job.active = true;
    m.job.queued = true;
    m.job_slot = slot;
    if (!tx.empty()) {
      // misra-lint: ignore 0.1.2
      m.engine->Start(kb::dma::Direction::ToDevice, const_cast<std::byte*>(tx.data()), bytes, 0U);
    } else {
      // misra-lint: ignore 0.1.2
      m.engine->Start(kb::dma::Direction::FromDevice, rx.data(), bytes, m.seed);
    }
  }
  return err;
}

int32_t SpiQueueResult(uint8_t slot, uint32_t timeout_ms, uint32_t& flags) noexcept {
  static_cast<void>(slot);
  Model& m = M();
  flags = 0U;
  int32_t err = esp_err::kTimeout;
  if (m.job.active && m.job.queued && Advance(timeout_ms * m.ticks_per_ms)) {
    flags = TakeFlags();
    err = ((flags & dma_flags::kTransferError) != 0U) ? esp_err::kInvalidState : esp_err::kOk;
    EndJob();
  }
  return err;
}

int32_t SpiHostReset(uint8_t host) noexcept {
  static_cast<void>(host);
  // The model's peripheral can be reset; ESP-IDF v5.5 cannot (the real boundary says so).
  ++M().resets;
  M().engine->HardReset();
  M().stalled = false;
  EndJob();
  return esp_err::kOk;
}

bool DmaMemoryUsable(std::span<const uint8_t> memory) noexcept {
  return (kb::dma::AddressOf(Bytes(memory.data())) % 4U) == 0U;
}
int32_t CacheWriteBack(std::span<const std::byte> range) noexcept {
  M().machine->CleanRange(range);
  return esp_err::kOk;
}
int32_t CacheInvalidate(std::span<std::byte> range) noexcept {
  M().machine->InvalidateRange(range);
  return esp_err::kOk;
}
void MemoryBarrier() noexcept {
  M().machine->Barrier();
}

// --- ADC continuous: pins 1 to 8 are ADC1 channels 0 to 7; 2-byte results, the raw value only.
int32_t AdcPinChannel(int16_t pin, uint8_t& unit, uint8_t& channel) noexcept {
  int32_t err = esp_err::kInvalidArg;
  if ((pin >= 1) && (pin <= 8)) {
    unit = 0U;
    channel = static_cast<uint8_t>(pin - 1);
    err = esp_err::kOk;
  }
  return err;
}
int32_t AdcStreamOpen(const AdcStreamSetup& setup, AdcStreamSink& sink) noexcept {
  static_cast<void>(setup);
  int32_t err = esp_err::kOk;
  if (M().adc_open) {
    err = esp_err::kNotAllowed;
  } else {
    M().adc_open = true;
    M().sink = &sink;
  }
  return err;
}
int32_t AdcStreamStart() noexcept {
  ++M().adc_starts;
  return esp_err::kOk;
}
int32_t AdcStreamStop() noexcept {
  return esp_err::kOk;
}
int32_t AdcStreamClose() noexcept {
  M().adc_open = false;
  M().sink = nullptr;
  return esp_err::kOk;
}
int32_t AdcStreamDrainPool() noexcept {
  return esp_err::kOk;
}
std::size_t AdcResultBytes() noexcept {
  return 2U;
}
std::size_t AdcConvBytes() noexcept {
  return 4U;
}
std::size_t AdcFrameUnit() noexcept {
  return 4U;
}
uint32_t AdcSdkFrameBuffers() noexcept {
  return 2U; // the rig's ring has two halves
}
uint32_t AdcStreamMinHz() noexcept {
  return 1U;
}
uint32_t AdcStreamMaxHz() noexcept {
  return 1000000U;
}
bool AdcResultDecode(std::span<const uint8_t> result, AdcSample& out) noexcept {
  out = AdcSample{};
  out.raw = static_cast<uint16_t>(result[0] | (result[1] << 8U));
  return true;
}
bool AdcResultEncode(const AdcSample& sample, std::span<uint8_t> out) noexcept {
  out[0] = static_cast<uint8_t>(sample.raw & 0xFFU);
  out[1] = static_cast<uint8_t>(sample.raw >> 8U);
  return true;
}

} // namespace kb::port::esp32::sdk
