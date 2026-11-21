// AdcStreamCore (the start-time checks, the pins, the ADC continuous driver) and the port's default
// AdcStream instantiation, whose interrupt path this translation unit holds (placed in IRAM by the
// component's linker fragment when CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE is set). Platform-free;
// every SDK call goes through sdk.hpp (src/boundary/adc.cpp).
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include "kb/port/esp32/adc_stream.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "claims.hpp"
#include "kb/dma/capabilities.hpp"
#include "kb/port/esp32/error.hpp"
#include "kb/utils/status.hpp"
#include "sdk.hpp"

namespace kb::port::esp32 {

template class AdcStream<AdcFrame<kAdcDefaultFrameBytes>, kAdcDefaultFrames>;

namespace {
constexpr uint64_t kUsPerSecond = 1000000U;

[[nodiscard]] kb::Status MapStreamErr(int32_t err) noexcept {
  kb::Status status = MapEspErr(err);
  if (err == esp_err::kNotAllowed) {
    status = kb::Status::Busy; // another continuous stream has the driver
  }
  return status;
}
} // namespace

std::size_t AdcStreamCore::pinCount() const noexcept {
  std::size_t n = 0U;
  while ((n < config_.pins.size()) && (config_.pins[n] >= 0)) {
    ++n;
  }
  return n;
}

bool AdcStreamCore::rangesValid(std::size_t frame_bytes) const noexcept {
  return (pinCount() > 0U) && (config_.sample_hz >= sdk::AdcStreamMinHz()) &&
         (config_.sample_hz <= sdk::AdcStreamMaxHz()) && (config_.pool_frames >= 1U) &&
         (config_.pool_frames <= kAdcStreamMaxPoolFrames) && (frame_bytes > 0U) &&
         ((frame_bytes % sdk::AdcConvBytes()) == 0U) && ((frame_bytes % sdk::AdcFrameUnit()) == 0U);
}

kb::Status AdcStreamCore::Check(std::size_t frame_bytes, std::size_t frames) noexcept {
  const std::size_t pins = pinCount();
  bool ok = rangesValid(frame_bytes);
  uint8_t first_unit = 0U;
  for (std::size_t i = 0U; ok && (i < pins); ++i) {
    uint8_t unit = 0U;
    uint8_t channel = 0U;
    ok = (sdk::AdcPinChannel(config_.pins[i], unit, channel) == esp_err::kOk) &&
         ((i == 0U) || (unit == first_unit)); // one unit per pattern
    first_unit = (i == 0U) ? unit : first_unit;
  }
  kb::Status status = kb::Status::Ok;
  if (!ok) {
    status = error_.Set(kb::Status::InvalidArgument, esp_err::kInvalidArg);
  } else if ((config_.shape.frames != frames) || (config_.shape.FrameBytes() != frame_bytes) ||
             (config_.shape.element_bytes != sdk::AdcResultBytes())) {
    // The configured conversions per frame differ from the board's declaration: a ring that
    // wraps on another boundary than the consumer expects. No DMA; the fallback serves.
    status = error_.Set(kb::Status::NotReady, esp_err::kInvalidSize);
  } else {
    status = error_.Clear();
  }
  return status;
}

kb::Status AdcStreamCore::Open(std::size_t frame_bytes, AdcStreamSink& sink) noexcept {
  const std::size_t pins = pinCount();
  std::array<sdk::AdcStreamChannel, kAdcStreamMaxChannels> pattern{};
  for (std::size_t i = 0U; i < pins; ++i) {
    static_cast<void>(sdk::AdcPinChannel(config_.pins[i], pattern[i].unit, pattern[i].channel));
  }
  kb::Status status = kb::Status::Ok;
  if (!detail::ClaimPinList(std::span<const int16_t>(config_.pins.data(), pins))) {
    status = error_.Set(kb::Status::Busy, esp_err::kNotAllowed); // a pin has another owner
  } else {
    const sdk::AdcStreamSetup setup{
        .channels = std::span<const sdk::AdcStreamChannel>(pattern.data(), pins),
        .attenuation = config_.attenuation,
        .sample_hz = config_.sample_hz,
        .frame_bytes = frame_bytes,
        .pool_bytes = frame_bytes * config_.pool_frames,
        .iram_safe = config_.iram_safe};
    const int32_t err = sdk::AdcStreamOpen(setup, sink);
    status = error_.Set(MapStreamErr(err), err);
    if (status == kb::Status::Ok) {
      cacheInterruptFacts(frame_bytes);
      open_ = true;
    } else {
      detail::UnclaimPinList(std::span<const int16_t>(config_.pins.data(), pins));
    }
  }
  return status;
}

kb::Status AdcStreamCore::Start() noexcept {
  const int32_t err = sdk::AdcStreamStart();
  return error_.Set(MapStreamErr(err), err);
}

kb::Status AdcStreamCore::Stop() noexcept {
  const int32_t err = sdk::AdcStreamStop();
  return error_.Set(MapStreamErr(err), err);
}

kb::Status AdcStreamCore::Close() noexcept {
  const int32_t err = sdk::AdcStreamClose();
  const kb::Status status = error_.Set(MapStreamErr(err), err);
  if (status == kb::Status::Ok) {
    open_ = false;
    detail::UnclaimPinList(std::span<const int16_t>(config_.pins.data(), pinCount()));
  }
  return status;
}

void AdcStreamCore::Drain() noexcept { // NOLINT(readability-convert-member-functions-to-static)
  static_cast<void>(sdk::AdcStreamDrainPool());
}

void AdcStreamCore::cacheInterruptFacts(std::size_t frame_bytes) noexcept {
  // Computed here, in the owner's context: the interrupt path reads only these two members (the
  // SDK queries live in the boundary, in flash).
  uint64_t period = config_.supervision.frame_period.Count();
  const std::size_t result = sdk::AdcResultBytes();
  if ((period == 0U) && (config_.sample_hz != 0U) && (result != 0U)) {
    period = (uint64_t{frame_bytes / result} * kUsPerSecond) / config_.sample_hz;
  }
  sdk_buffers_ = sdk::AdcSdkFrameBuffers();
  lap_threshold_us_ = (sdk_buffers_ > 1U) ? (period * (sdk_buffers_ - 1U)) : 0U;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static): part of the stream's interface
kb::dma::Capabilities AdcStreamCore::Capabilities(std::size_t frame_bytes, std::size_t frames,
                                                  bool dma) const noexcept {
  kb::dma::Capabilities c{};
  c.mode = dma ? kb::dma::TransferMode::Dma : kb::dma::TransferMode::Polled;
  c.zero_copy = false;      // frames are copied out in the interrupt
  c.confirmed_abort = true; // adc_continuous_stop() returns once the DMA is stopped
  c.max_frame_bytes = frame_bytes;
  c.staging_bytes = frame_bytes * frames;
  c.align_bytes = sdk::AdcFrameUnit();
  return c;
}

kb::Status AdcStreamCore::Decode(std::span<const uint8_t> frame, std::size_t index,
                                 AdcSample& out) noexcept {
  const std::size_t width = sdk::AdcResultBytes();
  kb::Status status = kb::Status::OutOfRange;
  if ((width != 0U) && (index < (frame.size() / width))) {
    status = sdk::AdcResultDecode(frame.subspan(index * width, width), out)
                 ? kb::Status::Ok
                 : kb::Status::InvalidArgument; // the hardware marked the result invalid
  }
  return status;
}

kb::Status AdcStreamCore::Encode(std::span<const AdcSample> samples,
                                 std::span<uint8_t> frame) noexcept {
  const std::size_t width = sdk::AdcResultBytes();
  kb::Status status = kb::Status::OutOfRange;
  if ((width != 0U) && (samples.size() <= (frame.size() / width))) {
    status = kb::Status::Ok;
    for (std::size_t i = 0U; (i < samples.size()) && (status == kb::Status::Ok); ++i) {
      if (!sdk::AdcResultEncode(samples[i], frame.subspan(i * width, width))) {
        status = kb::Status::InvalidArgument;
      }
    }
  }
  return status;
}

} // namespace kb::port::esp32
