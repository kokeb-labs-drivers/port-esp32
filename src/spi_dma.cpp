// SpiDmaCore: the non-template part of SpiDmaChannel (admission, the bus lock, the queued DMA
// transaction and its bounded end, the failed-host state). Platform-free; every SDK call goes
// through sdk.hpp (src/boundary/spi.cpp).
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "kb/port/esp32/spi_dma.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/dma/capabilities.hpp"
#include "kb/dma/completion.hpp"
#include "kb/dma/region.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/port/esp32/spi.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

static_assert(spi_dma_flags::kComplete == sdk::dma_flags::kComplete);
static_assert(spi_dma_flags::kTransferError == sdk::dma_flags::kTransferError);
static_assert(spi_dma_flags::kFifoError == sdk::dma_flags::kFifoError);

kb::Status SpiDmaCore::Admit(std::size_t used, kb::dma::Direction dir) const noexcept {
  const SpiBus& bus = device_.bus_;
  kb::Status status = kb::Status::Ok;
  if (!device_.added_.load()) {
    status = kb::Status::NotReady;
  } else if (bus.IsFailed()) {
    status = kb::Status::Fault;
  } else if ((used == 0U) || (used > bus.MaxFrameBytes())) {
    status = kb::Status::OutOfRange;
  } else if (!bus.DmaActive()) {
    status = kb::Status::Unsupported; // zero-copy needs the DMA; the FIFO path copies (Transfer)
  } else if ((dir == kb::dma::Direction::FromDevice) && ((used % sdk::SpiDmaRxUnit()) != 0U)) {
    status = kb::Status::InvalidArgument; // ESP32-P4: a DMA receive in whole cache lines only
  } else {
    status = kb::Status::Ok;
  }
  return status;
}

bool SpiDmaCore::Lock() noexcept {
  const bool locked = device_.bus_.TryLockFor(kb::Milliseconds{BoundedMs(device_.config_.timeout)});
  if (!locked) {
    device_.lock_timeouts_.Add();
  }
  return locked;
}

void SpiDmaCore::Unlock() noexcept {
  device_.bus_.Unlock();
}

int32_t SpiDmaCore::Begin(std::span<std::byte> memory, kb::dma::Direction dir,
                          std::size_t used) noexcept {
  device_.transfers_.Add();
  const std::span<std::byte> data = memory.first(used);
  return (dir == kb::dma::Direction::ToDevice)
             ? sdk::SpiQueue(device_.slot_, data, std::span<std::byte>{}, used)
             : sdk::SpiQueue(device_.slot_, std::span<const std::byte>{}, data, used);
}

int32_t SpiDmaCore::Result(uint32_t wait_ms, uint32_t& flags) const noexcept {
  return sdk::SpiQueueResult(device_.slot_, wait_ms, flags);
}

uint32_t SpiDmaCore::EndWaitMs(std::size_t used) const noexcept {
  return device_.waitMs(used);
}

void SpiDmaCore::MarkFailed() noexcept {
  device_.failures_.Add();
  device_.bus_.quarantines_.Add();
  device_.bus_.MarkFailed();
}

void SpiDmaCore::NoteError() noexcept {
  device_.failures_.Add();
  device_.bus_.dma_errors_.Add();
}

kb::dma::StopEvidence SpiDmaCore::Settle() noexcept {
  uint32_t flags = 0U;
  bool stopped = sdk::SpiQueueResult(device_.slot_, 0U, flags) != esp_err::kTimeout;
  if (!stopped) {
    stopped = sdk::SpiHostReset(device_.bus_.Config().host) == esp_err::kOk;
  }
  if (stopped) {
    device_.bus_.failed_.store(false); // the transaction is over: the host may serve again
  }
  return stopped ? kb::dma::StopEvidence::Confirmed : kb::dma::StopEvidence::NotConfirmed;
}

kb::dma::Capabilities SpiDmaCore::Capabilities() const noexcept {
  return device_.bus_.Capabilities();
}

kb::dma::DmaHealth SpiDmaCore::Health() const noexcept {
  return device_.bus_.Health();
}

} // namespace kb::port::esp32
