/**
 * @file dma.hpp
 * @brief The ESP32 port's DMA vocabulary over `kb::dma`: the memory regions its DMA masters reach,
 * the per-target SPI DMA limits as `kb::dma::BusLimits`, the static staging buffer type of an SPI
 * host, the interrupt tiers of ESP-IDF as `kb::dma::InterruptLimits`, and `EspCacheOps`, the cache
 * hooks over `esp_cache_msync`.
 *
 * **No private GDMA.** The port never allocates a GDMA channel itself (the GDMA API is private in
 * ESP-IDF v5.5); it uses the peripheral drivers that own their channels: `spi_master` with
 * `SPI_DMA_CH_AUTO` (SpiBus) and `adc_continuous` (AdcStream). Each takes one channel pair at its
 * start; the budget is 3 pairs on the ESP32-C6, 5 on the ESP32-S3, 3 AHB plus 3 AXI on the
 * ESP32-P4; the ESP32 has dedicated SPI DMA channels and runs ADC continuous mode through I2S0.
 *
 * **Regions.**
 *
 * | Tag | Policy | Line | Align | Where |
 * | --- | --- | --- | --- | --- |
 * | `InternalRam` | Coherent | 1 | 4 | internal SRAM of the ESP32, S2, S3, C3, C6, H2 (no cache in
 * front) | | `InternalRamL1` | Maintained | 64 | 64 | internal RAM of the ESP32-P4, reached through
 * the L1 cache | | `Psram<L>` | Maintained | L | L | ESP32-S3 PSRAM (L =
 * `CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE`) |
 *
 * The port's own transfers use internal RAM only (`InternalRam`, or `InternalRamL1` on the P4).
 *
 * **SPI receive rounding.** ESP-IDF rounds every SPI DMA receive descriptor up to whole words, and
 * the ESP32 writes up to 3 bytes past an odd length; the P4 needs receive address and length in
 * whole 64-byte cache lines. The port's staging buffers are padded to the target's unit, so the
 * rounding lands in port memory; the length the SDK gets is the frame length.
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_DMA_HPP
#define KB_PORT_ESP32_DMA_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/dma/assignment.hpp"
#include "kb/dma/cache.hpp"
#include "kb/dma/region.hpp"

namespace kb::port::esp32 {

/** @name DMA masters of the ESP32 family (bits of `kb::dma::MasterSet`)
 * @{ */
inline constexpr kb::dma::MasterSet kMasterGdma = kb::dma::MasterBit(0U); ///< AHB GDMA (S3, C6, P4)
inline constexpr kb::dma::MasterSet kMasterSpiDma = kb::dma::MasterBit(1U); ///< ESP32 SPI2/SPI3 DMA
inline constexpr kb::dma::MasterSet kMasterI2sDma = kb::dma::MasterBit(2U); ///< ESP32 ADC via I2S0
inline constexpr kb::dma::MasterSet kMasterAxiGdma = kb::dma::MasterBit(3U); ///< P4 AXI GDMA
/** @} */

/** @brief Internal SRAM without a cache in front of it (every target except the ESP32-P4). */
struct InternalRam {
  static constexpr kb::dma::CachePolicy kPolicy = kb::dma::CachePolicy::Coherent;
  static constexpr std::size_t kLineBytes = 1U;
  static constexpr std::size_t kAlignBytes = 4U; ///< Descriptor and receive-word rule.
  static constexpr kb::dma::MasterSet kMasters = kMasterGdma | kMasterSpiDma | kMasterI2sDma;
};

/** @brief ESP32-P4 internal RAM, reached through the 64-byte L1 data cache. */
struct InternalRamL1 {
  static constexpr kb::dma::CachePolicy kPolicy = kb::dma::CachePolicy::Maintained;
  static constexpr std::size_t kLineBytes = 64U;
  static constexpr std::size_t kAlignBytes = 64U;
  static constexpr kb::dma::MasterSet kMasters = kMasterGdma | kMasterAxiGdma;
};

/** @brief ESP32-S3 PSRAM behind the data cache, line `Line` (16, 32 or 64 bytes). */
template <std::size_t Line>
struct Psram {
  static_assert((Line == 16U) || (Line == 32U) || (Line == 64U),
                "the ESP32-S3 data cache line is 16, 32 or 64 bytes");
  static constexpr kb::dma::CachePolicy kPolicy = kb::dma::CachePolicy::Maintained;
  static constexpr std::size_t kLineBytes = Line;
  static constexpr std::size_t kAlignBytes = Line;
  static constexpr kb::dma::MasterSet kMasters = kMasterGdma;
};

static_assert(kb::dma::Region<InternalRam>);
static_assert(kb::dma::Region<InternalRamL1>);
static_assert(kb::dma::Region<Psram<32U>>);

/** @brief The ESP32 targets, for the compile-time limits (a board names its target once). */
enum class Target : uint8_t {
  Esp32 = 0,
  Esp32S2 = 1,
  Esp32S3 = 2,
  Esp32C3 = 3,
  Esp32C6 = 4,
  Esp32H2 = 5,
  Esp32P4 = 6,
};

/** @brief The longest SPI frame without DMA, on every ESP32 target (the SPI FIFO). */
inline constexpr std::size_t kSpiMaxFrameBytes = 64U;
/** @brief The longest SPI frame with DMA: the largest staging buffer of a host. */
inline constexpr std::size_t kSpiMaxDmaFrameBytes = 4092U;
/** @brief The smallest staging buffer a host may be given. */
inline constexpr std::size_t kSpiMinDmaBufferBytes = 4U;
/** @brief The alignment of a static staging buffer: the largest unit of every target (P4 line). */
inline constexpr std::size_t kSpiStagingAlign = 64U;

/** @brief The padding unit of SPI DMA buffers on `target`: 64 (P4 cache line) or 4 (one word). */
[[nodiscard]] constexpr std::size_t SpiDmaUnitBytes(Target target) noexcept {
  return (target == Target::Esp32P4) ? 64U : 4U;
}

/**
 * @brief The DMA facts of one SPI host on `target` (`kb::dma::BusLimits`): with `dma`, frames up
 * to `staging_bytes` (at most kSpiMaxDmaFrameBytes); without, kSpiMaxFrameBytes. Receive lengths
 * round up to the target's unit (whole words; whole lines on the P4), which the staging covers.
 */
[[nodiscard]] constexpr kb::dma::BusLimits SpiBusLimits(Target target, bool dma,
                                                        std::size_t staging_bytes) noexcept {
  const std::size_t frame =
      (staging_bytes > kSpiMaxDmaFrameBytes) ? kSpiMaxDmaFrameBytes : staging_bytes;
  return kb::dma::BusLimits{.dma = dma,
                            .max_frame_dma = frame,
                            .max_frame_no_dma = kSpiMaxFrameBytes,
                            .rx_length_multiple = SpiDmaUnitBytes(target),
                            .align_bytes = SpiDmaUnitBytes(target)};
}

/**
 * @brief The limits of one SPI bus as a tag type for `kb::dma::RequireBusCarries`:
 * `static_assert`s a driver's largest frame against the bus at build time.
 *
 * @code
 * using Spi2 = kb::port::esp32::SpiLimits<kb::port::esp32::Target::Esp32C6, true, 1028U>;
 * constexpr bool kFits = (kb::dma::RequireBusCarries<Spi2, 1028U>(), true);
 * @endcode
 */
template <Target T, bool Dma, std::size_t StagingBytes = kSpiMaxDmaFrameBytes>
struct SpiLimits {
  static_assert(!Dma || ((StagingBytes >= kSpiMinDmaBufferBytes) &&
                         (StagingBytes <= kSpiMaxDmaFrameBytes)),
                "an SPI staging buffer holds 4 to 4092 bytes");
  static constexpr kb::dma::BusLimits kBus = SpiBusLimits(T, Dma, StagingBytes);
};

/**
 * @brief Refuses, at build time, an SPI device that would use half-duplex DMA with both a write
 * and a read phase on the ESP32 (ESP-IDF `spi_master` "Known Issues": the combination is not
 * supported there). The port itself runs every device full-duplex; a board or binding that adds a
 * half-duplex device states its phases here.
 */
template <Target T, bool Dma, bool WritePhase, bool ReadPhase>
consteval void RequireSpiHalfDuplexPhases() {
  static_assert(!((T == Target::Esp32) && Dma && WritePhase && ReadPhase),
                "kb::port::esp32: half-duplex SPI DMA with both a write and a read phase is not "
                "supported on the ESP32 (use full duplex, or no DMA on this bus)");
}

/**
 * @brief ESP-IDF's interrupt tiers as `kb::dma::InterruptLimits`. A kb priority value p maps to
 * the ESP-IDF interrupt level 7 - p: levels 1 to 3 (p 6 to 4) run C handlers that may call FreeRTOS
 * from-ISR functions; levels 4 to 7 need assembly handlers and are not used by the port.
 */
inline constexpr kb::dma::InterruptLimits kInterruptLimits{.max_syscall_priority = 4U,
                                                           .priority_bits = 3U};

/** @brief The tag type of kInterruptLimits, for `kb::dma::RequireInterruptTier`. */
struct EspRtosLimits {
  static constexpr kb::dma::InterruptLimits kLimits = kInterruptLimits;
};

/** @brief The ESP-IDF interrupt level of kb priority value `priority` (0 when out of range). */
[[nodiscard]] constexpr uint8_t InterruptLevelOf(uint8_t priority) noexcept {
  constexpr uint8_t kTop = 7U;
  return ((priority >= 1U) && (priority <= kTop)) ? static_cast<uint8_t>(kTop - priority) : 0U;
}

/** @brief True for the levels the port can ask ESP-IDF for: 0 (the SDK default) and 1 to 3. */
[[nodiscard]] constexpr bool InterruptLevelUsable(uint8_t level) noexcept {
  constexpr uint8_t kHighestC = 3U;
  return level <= kHighestC;
}

/** @brief A host's staging memory: two halves (transmit, receive) of `half` bytes from `base`. */
struct SpiStagingRef {
  uint8_t* base{nullptr}; ///< The first byte (aligned to kSpiStagingAlign); null: none given.
  std::size_t half{0U};   ///< Bytes of each half (a whole number of kSpiStagingAlign units).
  std::size_t frame{0U};  ///< The longest frame the host carries (at most `half`).
};

/**
 * @brief The static staging buffer of one DMA SPI host: define it at namespace scope (internal RAM,
 * `.bss`) and hand `Ref()` to `SpiBusConfig::staging`; the port checks at `EnsureReady()` that the
 * memory is DMA-capable and aligned. Without one the port allocates the same once in
 * `SpiBus::EnsureReady()`.
 *
 * @tparam Bytes The longest frame of the host (kSpiMinDmaBufferBytes to kSpiMaxDmaFrameBytes).
 */
template <std::size_t Bytes>
class SpiStaging {
  static_assert((Bytes >= kSpiMinDmaBufferBytes) && (Bytes <= kSpiMaxDmaFrameBytes),
                "an SPI staging buffer holds 4 to 4092 bytes");

public:
  /// Bytes of each half: the frame padded to the largest unit of every target.
  static constexpr std::size_t kHalf = kb::dma::RoundUp(Bytes, kSpiStagingAlign);

  /** @brief The memory, for `SpiBusConfig::staging`. */
  [[nodiscard]] SpiStagingRef Ref() noexcept {
    return SpiStagingRef{.base = bytes_.data(), .half = kHalf, .frame = Bytes};
  }

private:
  alignas(kSpiStagingAlign) std::array<uint8_t, 2U * kHalf> bytes_{};
};

/**
 * @brief `kb::dma::CacheOps` over ESP-IDF `esp_cache_msync`: `Clean` writes dirty lines back
 * (C2M), `Invalidate` discards lines (M2C), `Barrier` orders memory. Only `Maintained` regions
 * reach the first two (`kb::dma::Buffer` calls them only there). A refused sync is counted.
 */
class EspCacheOps {
public:
  /** @brief Writes back every line of `range` (whole lines of one buffer). */
  void Clean(std::span<const std::byte> range) noexcept;
  /** @brief Discards every line of `range` (whole lines of one buffer). */
  void Invalidate(std::span<std::byte> range) noexcept;
  /** @brief A full memory barrier before the DMA starts or after it stopped. */
  void Barrier() noexcept;
  /** @brief Syncs ESP-IDF refused since start (an unaligned or uncached range). */
  [[nodiscard]] uint32_t Failures() const noexcept {
    return failures_;
  }

private:
  uint32_t failures_{0U};
};

static_assert(kb::dma::CacheOps<EspCacheOps>, "EspCacheOps satisfies kb::dma::CacheOps");

/**
 * @brief True when `memory` can be a DMA target of the port: DMA-capable internal RAM, start and
 * size aligned to the chip's DMA and cache rules (`esp_ptr_dma_capable`,
 * `esp_dma_is_buffer_alignment_satisfied`). The start-time check of every staging buffer.
 */
[[nodiscard]] bool DmaMemoryUsable(std::span<const uint8_t> memory) noexcept;

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_DMA_HPP
