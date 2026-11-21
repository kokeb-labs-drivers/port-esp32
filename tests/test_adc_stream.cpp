// AdcStream over the ESP-IDF fakes (adc_continuous): the driver created at EnsureReady, frames
// copied in the conversion-done interrupt with its time, the pool overflow as a separate counted
// signal, the SDK pool drained without waiting, the shape check (B33) with the reported fallback,
// the lap check, the IRAM-safe requirement, an idempotent start and the stall supervision. Test
// code. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/port/esp32/adc_stream.hpp"
#include "support.hpp"

namespace {
using kb::dma::TransferMode;
using Stream = esp::AdcStreamDefault; // 192-byte frames, 4 slots
constexpr std::size_t kResults = esp::kAdcDefaultFrameBytes / 4U;

esp::AdcStreamConfig Config() {
  esp::AdcStreamConfig c{};
  c.pins = {1, 2, 3, -1, -1, -1, -1, -1}; // ADC1 channels 0 to 2 (the fake's pin map)
  c.sample_hz = 3000U;                    // 1 kHz per channel
  c.pool_frames = 2U;
  c.shape = kb::dma::StreamShape{.frames = 4U, .element_bytes = 4U, .elements = kResults};
  c.supervision.frame_period = kb::Microseconds{16000U}; // 48 results at 3 kHz
  c.supervision.stall_periods = 4U;
  return c;
}

// A frame of results counting from `base`, in the fake's format (12-bit data, 4-bit channel).
std::array<uint8_t, esp::kAdcDefaultFrameBytes> Frame(uint16_t base) {
  std::array<uint8_t, esp::kAdcDefaultFrameBytes> bytes{};
  for (std::size_t i = 0U; i < kResults; ++i) {
    const uint32_t v =
        (static_cast<uint32_t>(base + i) & 0xFFFU) | (static_cast<uint32_t>(i % 3U) << 13U);
    bytes[(4U * i) + 0U] = static_cast<uint8_t>(v & 0xFFU);
    bytes[(4U * i) + 1U] = static_cast<uint8_t>((v >> 8U) & 0xFFU);
    bytes[(4U * i) + 2U] = static_cast<uint8_t>((v >> 16U) & 0xFFU);
  }
  return bytes;
}
} // namespace

KB_TEST("adc stream: EnsureReady creates and configures the driver, two separate callbacks") {
  Fresh();
  Stream s{Config()};
  KB_CHECK_EQ(s.Start(kb::Instant{0U}), Status::NotReady);
  KB_REQUIRE(s.EnsureReady() == Status::Ok);
  KB_CHECK_EQ(s.EnsureReady(), Status::Ok); // idempotent
  KB_CHECK(fake::AdcStreamAlive());
  KB_CHECK_EQ(fake::AdcStreamHandleConfig().conv_frame_size, esp::kAdcDefaultFrameBytes);
  KB_CHECK_EQ(fake::AdcStreamHandleConfig().max_store_buf_size, 2U * esp::kAdcDefaultFrameBytes);
  KB_CHECK_EQ(fake::AdcStreamHandleConfig().flags.flush_pool, 0U);
  KB_CHECK_EQ(fake::AdcStreamConfig().sample_freq_hz, 3000U);
  KB_CHECK(fake::AdcStreamConfig().conv_mode == ADC_CONV_SINGLE_UNIT_1);
  KB_CHECK(fake::AdcStreamConfig().format == ADC_DIGI_OUTPUT_FORMAT_TYPE2);
  KB_CHECK_EQ(fake::AdcStreamPattern().size(), 3U);
  KB_CHECK_EQ(fake::AdcStreamPattern()[2].channel, 2U);
  KB_CHECK_EQ(fake::AdcStreamPattern()[0].bit_width, 12U);
  const kb::dma::Capabilities caps = s.Capabilities();
  KB_CHECK(caps.mode == TransferMode::Dma);
  KB_CHECK(!caps.zero_copy);
  KB_CHECK_EQ(caps.max_frame_bytes, esp::kAdcDefaultFrameBytes);
  KB_CHECK_EQ(caps.staging_bytes, 4U * esp::kAdcDefaultFrameBytes);
  // A second stream: the chip has one continuous driver.
  Stream other{Config()};
  KB_CHECK_EQ(other.EnsureReady(), Status::Busy);
  KB_CHECK_EQ(s.Release(), Status::Ok);
  KB_CHECK(!fake::AdcStreamAlive());
}

KB_TEST(
    "adc stream: frames are copied in the interrupt with its time; the SDK buffer is not kept") {
  Fresh();
  Stream s{Config()};
  KB_REQUIRE(s.EnsureReady() == Status::Ok);
  fake::SetTimeUs(1000U);
  KB_REQUIRE(s.Start(kb::Instant{1000U}) == Status::Ok);
  KB_CHECK_EQ(s.Start(kb::Instant{1000U}), Status::Ok); // a second start is a success
  KB_CHECK_EQ(fake::Calls("adc_continuous_start"), 1U);
  fake::SetTimeUs(17000U);
  static_cast<void>(fake::AdcStreamFrame(Frame(100U)));
  Stream::FrameView view{};
  KB_REQUIRE(s.TryTake(view) == Status::Ok);
  KB_CHECK_EQ(view.At().Us(), 17000U); // the time read in the interrupt
  KB_CHECK(!view.Gap());
  KB_CHECK(!view.Torn());
  esp::AdcSample sample{};
  KB_CHECK_EQ(Stream::Decode(view.Data(), 0U, sample), Status::Ok);
  KB_CHECK_EQ(sample.raw, 100U); // the SDK refilled its buffer: the copy is intact
  KB_CHECK_EQ(Stream::Decode(view.Data(), 4U, sample), Status::Ok);
  KB_CHECK_EQ(sample.channel, 1U);
  KB_CHECK_EQ(sample.raw, 104U);
  KB_CHECK_EQ(Stream::Decode(view.Data(), kResults, sample), Status::OutOfRange);
  KB_CHECK_EQ(view.Seq(), 0U);
  s.Give(std::move(view));
  KB_CHECK(s.Stats().max_isr_cycles > 0U); // the interrupt's cost, in cycles
  KB_CHECK_EQ(s.Stop(), Status::Ok);
  KB_CHECK(!fake::AdcStreamRunning());
  KB_CHECK_EQ(s.Stop(), Status::Ok);
}

KB_TEST("adc stream: a pool overflow is counted and never a frame; the next frame has Gap()") {
  Fresh();
  Stream s{Config()};
  KB_REQUIRE(s.EnsureReady() == Status::Ok);
  KB_REQUIRE(s.Start(kb::Instant{0U}) == Status::Ok);
  // Nothing drains the SDK pool (no TryTake): after two frames it is full, then on_pool_ovf.
  for (uint16_t k = 0U; k < 3U; ++k) {
    fake::SetTimeUs(16000U * (k + 1U));
    static_cast<void>(fake::AdcStreamFrame(Frame(k)));
  }
  KB_CHECK_EQ(s.Stats().hw_overruns, 1U);
  KB_CHECK_EQ(s.Stats().published, 3U); // the overflow published nothing
  KB_CHECK_EQ(s.Health().errors, 1U);
  Stream::FrameView view{};
  KB_REQUIRE(s.TryTake(view) == Status::Ok); // drains the SDK pool without waiting
  KB_CHECK_EQ(fake::AdcStreamPool(), 0U);
  KB_CHECK(fake::AdcStreamReads() >= 2U);
  KB_CHECK(!view.Gap());
  s.Give(std::move(view));
  KB_REQUIRE(s.TryTake(view) == Status::Ok);
  s.Give(std::move(view));
  KB_REQUIRE(s.TryTake(view) == Status::Ok);
  s.Give(std::move(view));
  fake::SetTimeUs(64000U);
  static_cast<void>(fake::AdcStreamFrame(Frame(9U)));
  KB_REQUIRE(s.TryTake(view) == Status::Ok);
  KB_CHECK(view.Gap()); // the frame after the overflow
}

KB_TEST("adc stream: a frame after a hold-off of the SDK's ring is published Torn") {
  Fresh();
  Stream s{Config()};
  KB_REQUIRE(s.EnsureReady() == Status::Ok);
  KB_REQUIRE(s.Start(kb::Instant{0U}) == Status::Ok);
  fake::SetTimeUs(16000U);
  static_cast<void>(fake::AdcStreamFrame(Frame(0U)));
  // The interrupt was held off for 5 periods (the SDK's ring holds 5 buffers): the burst may mix
  // passes.
  fake::SetTimeUs(16000U * 6U);
  static_cast<void>(fake::AdcStreamFrame(Frame(1U)));
  Stream::FrameView view{};
  KB_REQUIRE(s.TryTake(view) == Status::Ok);
  KB_CHECK(!view.Torn());
  s.Give(std::move(view));
  KB_REQUIRE(s.TryTake(view) == Status::Ok);
  KB_CHECK(view.Torn());
  KB_CHECK_EQ(s.Stats().torn, 1U);
}

KB_TEST("adc stream: the board's shape must match, else no DMA and the fallback reports (B33)") {
  Fresh();
  esp::AdcStreamConfig c = Config();
  c.shape.elements = kResults - 3U; // 45 conversions declared, the frame holds 48
  Stream s{c};
  KB_CHECK_EQ(s.EnsureReady(), Status::NotReady);
  KB_CHECK(FailedWith(s.LastError(), Status::NotReady, ESP_ERR_INVALID_SIZE));
  KB_CHECK(!fake::AdcStreamAlive()); // DMA refused
  KB_CHECK(s.Capabilities().mode == TransferMode::Polled);
  KB_CHECK(s.Health().mode == TransferMode::Polled);
  KB_CHECK_EQ(s.EnsureReady(), Status::NotReady);
  const std::array<esp::AdcSample, 3> samples{esp::AdcSample{.unit = 0U, .channel = 0U, .raw = 7U},
                                              esp::AdcSample{.unit = 0U, .channel = 1U, .raw = 8U},
                                              esp::AdcSample{.unit = 0U, .channel = 2U, .raw = 9U}};
  KB_CHECK_EQ(s.PublishFallback(samples, kb::Instant{5U}), Status::NotReady); // not started
  KB_REQUIRE(s.Start(kb::Instant{0U}) == Status::Ok);
  KB_CHECK(s.Mode() == kb::dma::StreamMode::Fallback);
  KB_CHECK_EQ(s.PublishFallback(samples, kb::Instant{5U}), Status::Ok);
  Stream::FrameView view{};
  KB_REQUIRE(s.TryTake(view) == Status::Ok);
  KB_CHECK(view.Fallback());
  esp::AdcSample got{};
  KB_CHECK_EQ(Stream::Decode(view.Data(), 2U, got), Status::Ok);
  KB_CHECK_EQ(got.raw, 9U);
  KB_CHECK_EQ(got.channel, 2U);
  s.Give(std::move(view));
  const std::array<esp::AdcSample, 1> bad{esp::AdcSample{.unit = 0U, .channel = 12U, .raw = 1U}};
  KB_CHECK_EQ(s.PublishFallback(bad, kb::Instant{6U}), Status::InvalidArgument);
  std::array<esp::AdcSample, kResults + 1U> many{};
  KB_CHECK_EQ(s.PublishFallback(many, kb::Instant{6U}), Status::OutOfRange);
  KB_CHECK_EQ(s.Stop(), Status::Ok);
  KB_CHECK_EQ(s.Release(), Status::Ok);
  // Another frame count, or another element size, is the same refusal.
  c = Config();
  c.shape.frames = 2U;
  Stream frames{c};
  KB_CHECK_EQ(frames.EnsureReady(), Status::NotReady);
  c = Config();
  c.shape = kb::dma::StreamShape{.frames = 4U, .element_bytes = 2U, .elements = 96U};
  Stream width{c};
  KB_CHECK_EQ(width.EnsureReady(), Status::NotReady);
}

KB_TEST("adc stream: configuration errors are refused before the SDK") {
  Fresh();
  const auto refused = [](esp::AdcStreamConfig c) {
    Stream s{c};
    return s.EnsureReady() == Status::InvalidArgument;
  };
  esp::AdcStreamConfig c = Config();
  c.pins = {-1, -1, -1, -1, -1, -1, -1, -1};
  KB_CHECK(refused(c));
  c = Config();
  c.pins[1] = 40; // not an ADC pin
  KB_CHECK(refused(c));
  c = Config();
  c.pins[1] = 12; // ADC2: one unit per pattern
  KB_CHECK(refused(c));
  c = Config();
  c.sample_hz = 100U; // below the SoC's lower limit
  KB_CHECK(refused(c));
  c = Config();
  c.sample_hz = 100000U;
  KB_CHECK(refused(c));
  c = Config();
  c.pool_frames = 0U;
  KB_CHECK(refused(c));
  c = Config();
  c.pool_frames = 17U;
  KB_CHECK(refused(c));
  KB_CHECK_EQ(fake::Calls("adc_continuous_new_handle"), 0U);
  using Odd = esp::AdcStream<esp::AdcFrame<6U>, 2U>; // not whole conversions
  esp::AdcStreamConfig o = Config();
  o.shape = kb::dma::StreamShape{.frames = 2U, .element_bytes = 4U, .elements = 1U};
  Odd odd{o};
  KB_CHECK_EQ(odd.EnsureReady(), Status::InvalidArgument);
}

KB_TEST("adc stream: SDK failures are mapped and nothing is left behind") {
  Fresh();
  for (const char* fn : {"adc_continuous_new_handle", "adc_continuous_config",
                         "adc_continuous_register_event_callbacks"}) {
    fake::Fail(fn, ESP_ERR_INVALID_ARG);
    Stream s{Config()};
    KB_CHECK_EQ(s.EnsureReady(), Status::InvalidArgument);
    KB_CHECK(!fake::AdcStreamAlive());
  }
  Stream s{Config()};
  KB_REQUIRE(s.EnsureReady() == Status::Ok);
  esp::AdcStreamConfig c = Config();
  Stream pin_taken{c}; // the same pins
  fake::Fail("adc_continuous_start", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(s.Start(kb::Instant{0U}), Status::NotReady);
  KB_CHECK(!s.Stats().running);
  KB_REQUIRE(s.Start(kb::Instant{0U}) == Status::Ok);
  fake::Fail("adc_continuous_stop", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(s.Stop(), Status::NotReady);
  KB_CHECK_EQ(s.Release(), Status::Ok);
  KB_CHECK(!fake::AdcStreamAlive());
  KB_CHECK_EQ(pin_taken.EnsureReady(), Status::Ok); // the pins are free again
  fake::Fail("adc_continuous_deinit", ESP_ERR_INVALID_STATE);
  KB_CHECK_EQ(pin_taken.Release(), Status::NotReady);
  KB_CHECK_EQ(pin_taken.Release(), Status::Ok);
}

KB_TEST("adc stream: with iram_safe the interrupt path must be in IRAM and the stream internal") {
  Fresh();
  esp::AdcStreamConfig c = Config();
  c.iram_safe = true;
  {
    Stream s{c};
    KB_CHECK_EQ(s.EnsureReady(), Status::InvalidArgument); // the fake's code is not in IRAM
  }
  fake::IramAll(true);
  fake::InternalAll(false);
  {
    Stream s{c};
    KB_CHECK_EQ(s.EnsureReady(), Status::InvalidArgument); // the object is not in internal RAM
  }
  fake::InternalAll(true);
  {
    Stream s{c};
    KB_CHECK_EQ(s.EnsureReady(), Status::Ok);
  }
  KB_CHECK(!fake::AdcStreamAlive());
}

KB_TEST("adc stream: a stopped trigger is a stall (signed time), restarts are supervised") {
  Fresh();
  Stream s{Config()};
  KB_REQUIRE(s.EnsureReady() == Status::Ok);
  KB_REQUIRE(s.Start(kb::Instant{0U}) == Status::Ok);
  fake::SetTimeUs(16000U);
  static_cast<void>(fake::AdcStreamFrame(Frame(0U)));
  KB_CHECK(!s.CheckStall(kb::Instant{15000U})); // stamped after the task read its clock
  KB_CHECK(!s.CheckStall(kb::Instant{16000U + 63000U}));
  KB_CHECK(s.CheckStall(kb::Instant{16000U + 64000U}));
  KB_CHECK(s.Supervise(kb::Instant{16000U + 64000U}) == kb::dma::SupervisorAction::Restart);
  KB_CHECK_EQ(s.Stop(), Status::Ok);
  KB_REQUIRE(s.Start(kb::Instant{100000U}) == Status::Ok);
  s.NoteRestart(kb::Instant{100000U});
  s.NoteContent(true);
  KB_CHECK_EQ(s.Stats().restarts, 1U);
  KB_CHECK_EQ(s.Stats().stalls, 1U);
}
