/**
 * @file adc_stream.hpp
 * @brief `kb::port::esp32::AdcStream<Frame, Frames>`: ESP-IDF v5.5 ADC continuous mode (DMA) into a
 * `kb::dma::Stream` that the owner task reads.
 *
 * - `EnsureReady()` creates the SDK driver (`adc_continuous_new_handle`: its pool, frames and
 *   descriptors are allocated there, at start), configures the pattern and registers two separate
 *   callbacks, both in IRAM in the boundary: `on_conv_done` and `on_pool_ovf`.
 * - In the conversion-done interrupt the port copies the finished frame into the next slot of the
 *   kb stream, with the completion time read in that interrupt (`esp_timer_get_time`). The SDK's
 *   frame pointer never leaves the callback (the driver reuses its five frame buffers).
 * - A pool overflow is counted (`OnOverrun`) and never delivered as a frame; the next frame carries
 *   `Gap()`. The SDK's pool is emptied without waiting by every TryTake() (`flush_pool` is off), so
 *   an overflow means the owner task fell behind.
 * - The consumer reads the kb stream (TryTake, Give); the port never calls `adc_continuous_read`
 *   with a millisecond timeout (its conversion to ticks rounds down).
 * - Lapping. The SDK's interrupt hands over finished buffers of a ring of five; if it is held off
 *   for four frame periods or more, the DMA may have refilled a buffer before it is copied. A
 *   frame that arrives later than (buffers - 1) frame periods after the previous one, and the rest
 *   of that burst, are published `Torn` (the stream counts them).
 * - Shape check (start-time). `conv_frame_size` is `sizeof(Frame)`: a whole number of
 *   `SOC_ADC_DIGI_DATA_BYTES_PER_CONV` (and of the 64-byte cache line on the ESP32-P4). The board's
 *   `kb::dma::StreamShape` must match: frames = `Frames`, element bytes = the chip's result size,
 *   elements x element bytes = `sizeof(Frame)`. A mismatch refuses DMA (`NotReady`), and the
 *   stream runs from the declared fallback producer instead: one-shot conversions that the handler
 *   publishes with PublishFallback(), flagged `Fallback()`; Capabilities() and Health() say
 *   `Polled`.
 * - IRAM safety. With `AdcStreamConfig::iram_safe` (the board writes flash while streaming), the
 *   port requires `CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE`, its interrupt path in IRAM and the stream
 *   object in internal RAM, and refuses `EnsureReady()` otherwise (`Unsupported` without the
 *   option, `InvalidArgument` otherwise). The interrupt path is in IRAM for the port's default
 *   instantiation, `AdcStreamDefault` (compiled in src/adc_stream.cpp, which the component's
 *   linker fragment places in IRAM with that option); a stream of another frame type is refused
 *   under `iram_safe`.
 *
 * Supervision (stall budget with signed time, content check, restart policy) is the kb stream's:
 * CheckStall(), Supervise(), NoteContent(), NoteRestart().
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The port-esp32 authors
 */
#ifndef KB_PORT_ESP32_ADC_STREAM_HPP
#define KB_PORT_ESP32_ADC_STREAM_HPP

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

#include "kb/dma/assignment.hpp"
#include "kb/dma/capabilities.hpp"
#include "kb/dma/stream.hpp"
#include "kb/port/esp32/adc.hpp"
#include "kb/port/esp32/dma.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/utils/status.hpp"
#include "kb/utils/time.hpp"

namespace kb::port::esp32 {

/** @brief The most channels in one continuous pattern. */
inline constexpr std::size_t kAdcStreamMaxChannels = 8U;
/** @brief The longest SDK pool, in frames. */
inline constexpr uint32_t kAdcStreamMaxPoolFrames = 16U;

/** @brief One conversion result. */
struct AdcSample {
  uint8_t unit{0U};    ///< 0: ADC1, 1: ADC2.
  uint8_t channel{0U}; ///< The channel of the unit.
  uint16_t raw{0U};    ///< The raw count.
};

/** @brief A frame of `Bytes` raw result bytes, as the chip writes them (decode with Decode()). */
template <std::size_t Bytes>
struct AdcFrame {
  std::array<uint8_t, Bytes> bytes{};
};

/** @brief How a continuous stream is set up. */
struct AdcStreamConfig {
  /// The pattern's pins, all on one ADC unit; -1 ends the list.
  std::array<int16_t, kAdcStreamMaxChannels> pins{-1, -1, -1, -1, -1, -1, -1, -1};
  AdcAttenuation attenuation{AdcAttenuation::Db12}; ///< Range of every channel.
  uint32_t sample_hz{3000U};    ///< Conversions per second, all channels together (SoC limits).
  uint32_t pool_frames{4U};     ///< The SDK's pool in frames, 1 to kAdcStreamMaxPoolFrames.
  kb::dma::StreamShape shape{}; ///< The board's declaration (checked at start).
  kb::dma::StreamConfig supervision{}; ///< Stall budget, bad-frame and restart policy.
  bool iram_safe{false}; ///< Flash is written while streaming: the interrupt path must be IRAM.
};

/**
 * @brief The interrupt entry points of a stream, which the boundary's IRAM callbacks call. A
 * stream object starts with one (its private base).
 */
struct AdcStreamSink {
  void (*frame)(AdcStreamSink& sink, std::span<const uint8_t> bytes,
                uint64_t at_us) noexcept {nullptr};                         ///< One finished frame.
  void (*overflow)(AdcStreamSink& sink, uint64_t at_us) noexcept {nullptr}; ///< Pool overflow.
  void (*cost)(AdcStreamSink& sink, uint32_t cycles) noexcept {nullptr};    ///< Interrupt cost.
};

/** @brief The non-template part of AdcStream (checks, the SDK driver, the pins). Use AdcStream. */
class AdcStreamCore {
public:
  /** @brief Keeps `config`. */
  explicit AdcStreamCore(const AdcStreamConfig& config) noexcept : config_(config) {}
  AdcStreamCore(const AdcStreamCore&) = delete;
  AdcStreamCore& operator=(const AdcStreamCore&) = delete;
  AdcStreamCore(AdcStreamCore&&) = delete;
  AdcStreamCore& operator=(AdcStreamCore&&) = delete;
  ~AdcStreamCore() = default;

  /** @brief The start-time checks for frames of `frame_bytes` in a ring of `frames`: `Ok`,
   * `InvalidArgument` (configuration), `NotReady` (the board's shape differs: no DMA). */
  [[nodiscard]] kb::Status Check(std::size_t frame_bytes, std::size_t frames) noexcept;
  /** @brief Claims the pins, creates and configures the SDK driver and registers `sink`. */
  [[nodiscard]] kb::Status Open(std::size_t frame_bytes, AdcStreamSink& sink) noexcept;
  [[nodiscard]] kb::Status Start() noexcept; ///< adc_continuous_start().
  [[nodiscard]] kb::Status Stop() noexcept;  ///< adc_continuous_stop().
  [[nodiscard]] kb::Status Close() noexcept; ///< adc_continuous_deinit(), pins released.
  static void Drain() noexcept;              ///< Empties the SDK pool without waiting.
  /** @brief Microseconds after which a frame may have been lapped (0: no check); set by Open(). */
  [[nodiscard]] uint64_t LapThresholdUs() const noexcept {
    return lap_threshold_us_;
  }
  /** @brief The frame buffers of the SDK's DMA ring (five in v5.5); set by Open(). */
  [[nodiscard]] uint32_t SdkBuffers() const noexcept {
    return sdk_buffers_;
  }
  /** @brief The capabilities of a stream of `frame_bytes` x `frames` in mode `dma`. */
  [[nodiscard]] static kb::dma::Capabilities Capabilities(std::size_t frame_bytes,
                                                          std::size_t frames, bool dma) noexcept;
  /** @brief Result `index` of `frame` (bytes as the chip wrote them). */
  [[nodiscard]] static kb::Status Decode(std::span<const uint8_t> frame, std::size_t index,
                                         AdcSample& out) noexcept;
  /** @brief Encodes `samples` into `frame` in the chip's format (unused results stay 0). */
  [[nodiscard]] static kb::Status Encode(std::span<const AdcSample> samples,
                                         std::span<uint8_t> frame) noexcept;
  /** @brief True between Open() and Close(). */
  [[nodiscard]] bool IsOpen() const noexcept {
    return open_;
  }
  /** @brief The calling task's last call: status and its esp_err_t in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return error_.Get();
  }
  /** @brief The configuration. */
  [[nodiscard]] const AdcStreamConfig& Config() const noexcept {
    return config_;
  }

private:
  [[nodiscard]] std::size_t pinCount() const noexcept;
  [[nodiscard]] bool rangesValid(std::size_t frame_bytes) const noexcept;
  void cacheInterruptFacts(std::size_t frame_bytes) noexcept;

  AdcStreamConfig config_;
  CallerRecord error_;
  bool open_{false};
  uint64_t lap_threshold_us_{0U};
  uint32_t sdk_buffers_{0U};
};

/**
 * @brief The stream (see the file note).
 *
 * @tparam Frame  The frame type: trivially copyable, `sizeof(Frame)` raw result bytes.
 * @tparam Frames The slots of the kb ring, at least 2.
 */
template <kb::dma::StreamFrame Frame, std::size_t Frames>
class AdcStream : private AdcStreamSink {
public:
  /// The kb stream type.
  using StreamType = kb::dma::Stream<Frame, Frames, InternalRam>;
  /// A taken frame.
  using FrameView = typename StreamType::FrameView;
  /// The frame type.
  using FrameType = Frame;

  /** @brief Keeps `config`. */
  explicit AdcStream(const AdcStreamConfig& config) noexcept
      : AdcStreamSink{}, core_(config), stream_(config.supervision) {}
  AdcStream(const AdcStream&) = delete;
  AdcStream& operator=(const AdcStream&) = delete;
  AdcStream(AdcStream&&) = delete;
  AdcStream& operator=(AdcStream&&) = delete;
  /** @brief Stops and releases the driver. */
  ~AdcStream() noexcept {
    static_cast<void>(Release());
  }

  /** @brief The start-time checks and the SDK driver (see the file note); idempotent. */
  [[nodiscard]] kb::Status EnsureReady() noexcept;
  /** @brief Starts the conversions (or the fallback mode) with the stall clock at `now`;
   * idempotent while running. */
  [[nodiscard]] kb::Status Start(kb::Instant now) noexcept {
    kb::Status status = kb::Status::Ok;
    if (!core_.IsOpen() && !fallback_) {
      status = kb::Status::NotReady;
    } else if (!running_) {
      last_frame_us_ = now.Us();
      suspect_left_ = 0U;
      if (fallback_) {
        stream_.EnterFallback(now);
      }
      // kb::dma::Stream::Start returns nothing (the lint matches the name only).
      stream_.Start(now); // misra-lint: ignore 0.1.2
      status = fallback_ ? kb::Status::Ok : core_.Start();
      running_ = (status == kb::Status::Ok);
      if (!running_) {
        stream_.Stop(); // misra-lint: ignore 0.1.2 (kb::dma::Stream::Stop returns nothing)
      }
    } else {
      status = kb::Status::Ok; // a second start of a running stream is a success
    }
    return status;
  }
  /** @brief Stops the conversions; the frames already published stay to be taken. */
  [[nodiscard]] kb::Status Stop() noexcept {
    kb::Status status = kb::Status::Ok;
    if (running_) {
      status = fallback_ ? kb::Status::Ok : core_.Stop();
      if (status == kb::Status::Ok) {
        running_ = false;
        stream_.Stop(); // misra-lint: ignore 0.1.2 (kb::dma::Stream::Stop returns nothing)
      }
    }
    return status;
  }
  /** @brief Stops and deletes the SDK driver; EnsureReady() may be called again. */
  [[nodiscard]] kb::Status Release() noexcept {
    kb::Status status = Stop();
    if ((status == kb::Status::Ok) && core_.IsOpen()) {
      status = core_.Close();
    }
    if (status == kb::Status::Ok) {
      fallback_ = false;
    }
    return status;
  }

  /** @brief The oldest unread frame (see kb::dma::Stream::TryTake); empties the SDK pool first. */
  [[nodiscard]] kb::Status TryTake(FrameView& out) noexcept {
    if (core_.IsOpen()) {
      AdcStreamCore::Drain();
    }
    return stream_.TryTake(out);
  }
  /** @brief Gives a frame back at once. */
  void Give(FrameView&& view) noexcept {
    stream_.Give(std::move(view));
  }
  /** @brief The stall check (signed elapsed time). */
  [[nodiscard]] bool CheckStall(kb::Instant now) noexcept {
    return stream_.CheckStall(now);
  }
  /** @brief The supervisor's next step. */
  [[nodiscard]] kb::dma::SupervisorAction Supervise(kb::Instant now) noexcept {
    return stream_.Supervise(now);
  }
  /** @brief Records a content verdict. */
  void NoteContent(bool good) noexcept {
    stream_.NoteContent(good);
  }
  /** @brief Records that the owner restarted the stream (Stop, Start). */
  void NoteRestart(kb::Instant now) noexcept {
    stream_.NoteRestart(now);
  }
  /** @brief The stream's counters and state. */
  [[nodiscard]] kb::dma::StreamStats Stats() const noexcept {
    return stream_.Stats();
  }
  /** @brief Who produces now. */
  [[nodiscard]] kb::dma::StreamMode Mode() const noexcept {
    return stream_.Mode();
  }
  /** @brief What the stream offers now. */
  [[nodiscard]] kb::dma::Capabilities Capabilities() const noexcept {
    return AdcStreamCore::Capabilities(sizeof(Frame), Frames, !fallback_);
  }
  /** @brief The DMA part of a handler's health record. */
  [[nodiscard]] kb::dma::DmaHealth Health() const noexcept {
    kb::dma::DmaHealth h{};
    h.mode = Capabilities().mode;
    h.errors = stream_.Stats().hw_overruns;
    return h;
  }
  /** @brief The calling task's last call: status and its esp_err_t in Detail(). */
  [[nodiscard]] kb::Result<void> LastError() const noexcept {
    return core_.LastError();
  }

  /**
   * @brief Publishes one frame of the declared fallback producer (one-shot conversions read by the
   * owner task) while the stream runs in fallback mode; `NotReady` otherwise.
   */
  [[nodiscard]] kb::Status PublishFallback(std::span<const AdcSample> samples,
                                           kb::Instant at) noexcept {
    kb::Status status = kb::Status::NotReady;
    if (fallback_ && running_) {
      std::array<uint8_t, sizeof(Frame)> raw{};
      status = AdcStreamCore::Encode(samples, raw);
      if (status == kb::Status::Ok) {
        status = stream_.Publish(std::bit_cast<Frame>(raw), at, 0U);
      }
    }
    return status;
  }

  /** @brief Result `index` of `frame`. */
  [[nodiscard]] static kb::Status Decode(const Frame& frame, std::size_t index,
                                         AdcSample& out) noexcept {
    const auto raw = std::bit_cast<std::array<uint8_t, sizeof(Frame)>>(frame);
    return AdcStreamCore::Decode(raw, index, out);
  }

private:
  static void onFrame(AdcStreamSink& sink, std::span<const uint8_t> bytes, uint64_t at_us) noexcept;
  static void onOverflow(AdcStreamSink& sink, uint64_t at_us) noexcept;
  static void onCost(AdcStreamSink& sink, uint32_t cycles) noexcept;

  AdcStreamCore core_;
  StreamType stream_;
  // Interrupt side (written by Start() before the hardware runs, then by the interrupt only).
  uint64_t last_frame_us_{0U};
  uint32_t suspect_left_{0U};
  // Owner side.
  bool running_{false};
  bool fallback_{false};
};

template <kb::dma::StreamFrame Frame, std::size_t Frames>
kb::Status AdcStream<Frame, Frames>::EnsureReady() noexcept {
  kb::Status status = kb::Status::Ok;
  if (!core_.IsOpen() && !fallback_) {
    status = core_.Check(sizeof(Frame), Frames);
    if (status == kb::Status::NotReady) {
      fallback_ = true; // the shape differs: no DMA, the declared fallback producer (reported)
    } else if (status == kb::Status::Ok) {
      AdcStreamSink& sink = *this;
      sink.frame = &onFrame;
      sink.overflow = &onOverflow;
      sink.cost = &onCost;
      status = core_.Open(sizeof(Frame), sink);
    } else {
      // A configuration error: nothing was set up.
    }
  } else if (fallback_) {
    status = kb::Status::NotReady; // still no DMA: the fallback serves
  } else {
    status = kb::Status::Ok;
  }
  return status;
}

template <kb::dma::StreamFrame Frame, std::size_t Frames>
void AdcStream<Frame, Frames>::onFrame(AdcStreamSink& sink, std::span<const uint8_t> bytes,
                                       uint64_t at_us) noexcept {
  auto& self = static_cast<AdcStream&>(sink);
  kb::dma::Integrity integrity = kb::dma::Integrity::Clean;
  const uint64_t threshold = self.core_.LapThresholdUs();
  if ((threshold != 0U) &&
      (kb::dma::SignedElapsedUs(kb::Instant{at_us}, kb::Instant{self.last_frame_us_}) >=
       static_cast<int64_t>(threshold))) {
    self.suspect_left_ = self.core_.SdkBuffers(); // held off: this burst may hold lapped buffers
  }
  if (self.suspect_left_ != 0U) {
    integrity = kb::dma::Integrity::Torn;
    --self.suspect_left_;
  }
  self.last_frame_us_ = at_us;
  std::array<uint8_t, sizeof(Frame)> raw{};
  const std::size_t n = (bytes.size() < raw.size()) ? bytes.size() : raw.size();
  for (std::size_t i = 0U; i < n; ++i) {
    raw[i] = bytes[i];
  }
  const uint32_t short_frame = (bytes.size() == raw.size()) ? 0U : 1U;
  static_cast<void>(
      self.stream_.Publish(std::bit_cast<Frame>(raw), kb::Instant{at_us}, short_frame,
                           (short_frame != 0U) ? kb::dma::Integrity::Torn : integrity));
}

template <kb::dma::StreamFrame Frame, std::size_t Frames>
void AdcStream<Frame, Frames>::onOverflow(AdcStreamSink& sink, uint64_t at_us) noexcept {
  auto& self = static_cast<AdcStream&>(sink);
  static_cast<void>(at_us);
  constexpr uint32_t kPoolOverflow = 1U;
  self.stream_.OnOverrun(kPoolOverflow);
}

template <kb::dma::StreamFrame Frame, std::size_t Frames>
void AdcStream<Frame, Frames>::onCost(AdcStreamSink& sink, uint32_t cycles) noexcept {
  auto& self = static_cast<AdcStream&>(sink);
  self.stream_.NoteIsrCycles(cycles);
}

/** @brief The default frame of the port: 192 result bytes (48 conversions of 4 bytes, or 96 of 2
 * bytes on the ESP32). */
inline constexpr std::size_t kAdcDefaultFrameBytes = 192U;
/** @brief The default ring of the port. */
inline constexpr std::size_t kAdcDefaultFrames = 4U;
/** @brief The port's default stream: its interrupt path is compiled once, in src/adc_stream.cpp
 * (in IRAM when the board needs it, see the file note). */
using AdcStreamDefault = AdcStream<AdcFrame<kAdcDefaultFrameBytes>, kAdcDefaultFrames>;
extern template class AdcStream<AdcFrame<kAdcDefaultFrameBytes>, kAdcDefaultFrames>;

} // namespace kb::port::esp32

#endif // KB_PORT_ESP32_ADC_STREAM_HPP
