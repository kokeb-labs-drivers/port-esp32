// The kb-core DMA conformance suite, cases T1-T18 of kb/dma/host/conformance.hpp, run against the
// ESP32 port's real classes with their boundary linked against kb-core's host DMA-and-cache model
// (dma_model/). The buffer cases run twice: with the port's Coherent region (InternalRam: ESP32,
// S3, C6 internal SRAM, no cache) and with its Maintained region (InternalRamL1: the ESP32-P4's
// 64-byte L1 lines), so the cache cases T1-T4 meet a real cache model. Each case prints one line
// `CONFORMANCE <case> <variant> PASS|FAIL [verdict]` (the table in docs/dma.md comes from it).
// Test code. SPDX-License-Identifier: MIT
// Copyright (c) 2026 The port-esp32 authors
#include <cstdio>

#include "kb/dma/host/conformance.hpp"
#include "kb/dma/host/stream_rig.hpp"
#include "kb/port/esp32/adc_stream.hpp"
#include "kb/port/esp32/dma.hpp"
#include "kb_test.hpp"
#include "model.hpp"
#include "port_rig.hpp"

namespace c = kb::dma::host::conformance;
namespace h = kb::dma::host;
namespace esp = kb::port::esp32;

namespace {

bool Report(const char* name, const char* variant, const char* verdict) {
  std::printf("CONFORMANCE %s %s %s%s%s\n", name, variant, (verdict == nullptr) ? "PASS" : "FAIL",
              (verdict == nullptr) ? "" : " ", (verdict == nullptr) ? "" : verdict);
  return verdict == nullptr;
}

template <typename Region, typename Case>
bool RunSpi(const char* name, Case fn, const rig::Options& options = rig::Options{}) {
  rig::SpiRig<Region> r{options};
  constexpr bool kCached = Region::kPolicy == kb::dma::CachePolicy::Maintained;
  return Report(name, kCached ? "maintained" : "coherent",
                r.Ready() ? fn(r) : "the rig did not start");
}

template <typename Case>
void Both(const char* name, Case fn, const rig::Options& options = rig::Options{}) {
  KB_CHECK(RunSpi<esp::InternalRam>(name, fn, options));
  KB_CHECK(RunSpi<esp::InternalRamL1>(name, fn, options));
}

rig::Options RoundTo4() {
  rig::Options o{};
  o.round_rx_to = 4U; // the ESP32's receive rounding to whole words
  return o;
}

constexpr kb::dma::StreamConfig kCfg{.frame_period = kb::Microseconds{1000U},
                                     .stall_periods = 10U,
                                     .bad_frames_to_restart = 8U,
                                     .restart_backoff = kb::Milliseconds{1000U},
                                     .restarts_before_fallback = 0U};
using PortStream = esp::AdcStream<h::Scan, 2U>;

esp::AdcStreamConfig StreamConfig() {
  esp::AdcStreamConfig cfg{};
  cfg.pins = {1, 2, 3, 4, -1, -1, -1, -1};
  cfg.sample_hz = 4000U;
  cfg.pool_frames = 2U;
  cfg.shape = kb::dma::StreamShape{.frames = 2U, .element_bytes = 2U, .elements = 4U};
  cfg.supervision = kCfg;
  return cfg;
}

template <typename Case>
void RunStream(const char* name, Case fn) {
  PortStream stream{StreamConfig()};
  rig::AdcRig<PortStream> r{stream};
  KB_CHECK(Report(name, "adc-stream", fn(r)));
}

} // namespace

KB_TEST("conformance T1: transmit needs a clean") {
  Both("T1", [](auto& r) { return c::T1TxNeedsClean(r); });
}
KB_TEST("conformance T2: receive needs an invalidate before and after") {
  Both("T2", [](auto& r) { return c::T2RxNeedsInvalidate(r); });
}
KB_TEST("conformance T3: the CPU path does no maintenance") {
  Both("T3", [](auto& r) {
    const char* v = c::T3CpuPathNoMaintenance(r);
    return ((v == nullptr) && (r.FifoStatus() != kb::Status::Ok)) ? "the FIFO transfer failed" : v;
  });
}
KB_TEST("conformance T4: a neighbour on a shared line survives") {
  Both("T4", [](auto& r) { return c::T4NeighbourSurvives(r); });
}
KB_TEST("conformance T5: no CPU view during a lease") {
  Both("T5", [](auto& r) { return c::T5NoCpuDuringLease(r); });
}
KB_TEST("conformance T6: no DMA write after a timeout") {
  Both("T6", [](auto& r) { return c::T6NoWriteAfterTimeout(r); });
}
KB_TEST("conformance T7: an abort waits for the end") {
  Both("T7", [](auto& r) { return c::T7AbortWaitsForEnable(r); });
}
KB_TEST("conformance T8: receive rounding stays inside the buffer") {
  Both("T8", [](auto& r) { return c::T8RoundingStaysInside(r); }, RoundTo4());
}
KB_TEST("conformance T14: error flags") {
  Both("T14", [](auto& r) { return c::T14ErrorFlags(r); });
}
KB_TEST("conformance T15: a cancel racing a completion") {
  Both("T15", [](auto& r) { return c::T15CancelRacesCompletion(r); });
}
KB_TEST("conformance T16: an unconfirmed stop quarantines") {
  rig::Options o{};
  o.never_ends = true;
  Both("T16", [](auto& r) { return c::T16UnconfirmedQuarantines(r); }, o);
}
KB_TEST("conformance T17: fallback is reported") {
  rig::Options o{};
  o.dma = false;
  Both("T17", [](auto& r) { return c::T17FallbackIsReported(r, esp::kSpiMaxFrameBytes); }, o);
}
KB_TEST("conformance T18: synchronous transfer through staging") {
  Both("T18", [](auto& r) { return c::T18SyncThroughStaging(r); }, RoundTo4());
}

KB_TEST("conformance T9: a held frame is stable and the overrun shows") {
  RunStream("T9", [](auto& r) { return c::T9HeldFrame(r); });
}
KB_TEST("conformance T10: a lapped half is flagged") {
  RunStream("T10", [](auto& r) { return c::T10LappedIsFlagged(r); });
}
KB_TEST("conformance T11: one drainer") {
  RunStream("T11", [](auto& r) { return c::T11SingleDrainer(r); });
}
KB_TEST("conformance T12: the stall check is signed") {
  RunStream("T12", [](auto& r) { return c::T12SignedStall(r); });
}
KB_TEST("conformance T13: an overflow is not data") {
  RunStream("T13", [](auto& r) { return c::T13OverflowIsNotData(r); });
}

// --- port items beyond T1-T18 (docs/dma.md, "Conformance") ---------------------------------------

KB_TEST("port item B31: an overflow with nothing pending is counted once, publishes nothing") {
  PortStream stream{StreamConfig()};
  rig::AdcRig<PortStream> r{stream};
  KB_REQUIRE(r.Start(kb::Instant{0U}) == kb::Status::Ok);
  r.RaiseOverrun();
  r.Isr(kb::Instant{10U});
  r.Isr(kb::Instant{20U}); // the flag was served: a second entry finds nothing to do
  KB_CHECK_EQ(stream.Stats().hw_overruns, 1U);
  KB_CHECK_EQ(stream.Stats().published, 0U);
  PortStream::FrameView view{};
  KB_CHECK_EQ(stream.TryTake(view), kb::Status::Empty);
}

KB_TEST("port item: a second start of a running stream is a success and starts nothing") {
  PortStream stream{StreamConfig()};
  rig::AdcRig<PortStream> r{stream};
  KB_REQUIRE(r.Start(kb::Instant{0U}) == kb::Status::Ok);
  KB_CHECK_EQ(stream.Start(kb::Instant{5U}), kb::Status::Ok);
  KB_CHECK_EQ(model::AdcStarts(), 1U);
}

KB_TEST("port item B33: a shape the frame does not have refuses DMA and reports the fallback") {
  esp::AdcStreamConfig cfg = StreamConfig();
  cfg.shape.elements = 3U;
  PortStream stream{cfg};
  rig::AdcRig<PortStream> r{stream};
  KB_CHECK_EQ(r.Start(kb::Instant{0U}), kb::Status::NotReady);
  KB_CHECK(stream.Capabilities().mode == kb::dma::TransferMode::Polled);
  KB_CHECK(model::Sink() == nullptr); // no DMA driver
}

KB_TEST("port item: a never-ending transaction is recovered only through a proven stop") {
  rig::Options o{};
  o.never_ends = true;
  rig::SpiRig<esp::InternalRam> r{o};
  KB_REQUIRE(r.Ready());
  KB_REQUIRE(r.StartRx(16U, 0x10U) == kb::Status::Ok);
  KB_CHECK_EQ(r.Port().Cancel(), kb::Status::Fault);
  KB_CHECK_EQ(model::HostResets(), 0U);
  KB_CHECK_EQ(r.Port().Recover(r.RxStage()), kb::Status::Ok);
  KB_CHECK_EQ(model::HostResets(), 1U); // the proof: on ESP-IDF v5.5 this reset is refused
}
