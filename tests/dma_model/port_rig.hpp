// The adapter rigs of the kb-core DMA conformance suite for the ESP32 port: the same member
// functions as kb::dma::host::HostRig (cases T1-T8, T14-T18) and AdcRig (T9-T13), over the port's
// real classes (SpiBus, SpiDevice, SpiDmaChannel<R>, AdcStream<F, N>) whose boundary calls run on
// kb-core's host DMA-and-cache model (model_boundary.cpp). `Port()` is a forwarding view of the
// real SpiDmaChannel: it converts the poll and stats records into the suite's types and nothing
// else. Test code. SPDX-License-Identifier: MIT Copyright (c) 2026 The port-esp32 authors
#ifndef KB_PORT_ESP32_TEST_PORT_RIG_HPP
#define KB_PORT_ESP32_TEST_PORT_RIG_HPP

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

#include "kb/dma/buffer.hpp"
#include "kb/dma/host/engine.hpp"
#include "kb/dma/host/machine.hpp"
#include "kb/dma/host/model_port.hpp"
#include "kb/dma/host/rig.hpp"
#include "kb/dma/host/stream_rig.hpp"
#include "kb/port/esp32/adc_stream.hpp"
#include "kb/port/esp32/spi.hpp"
#include "kb/port/esp32/spi_dma.hpp"
#include "model.hpp"

namespace rig {

namespace d = kb::dma;
namespace h = kb::dma::host;
namespace esp = kb::port::esp32;

/** @brief How a port rig is built (the suite's options, mapped to the ESP32's terms). */
struct Options {
  bool speculation{true};      ///< The model cache refills lines of leased buffers.
  std::size_t round_rx_to{1U}; ///< The receive descriptor rounds up (4: the ESP32's rule).
  bool never_ends{false};      ///< T16: the transaction is never reported finished.
  bool dma{true};              ///< T17: false gives the host no DMA channel.
};

/** @brief The forwarding view `Port()` returns (the real SpiDmaChannel underneath). */
template <d::Region R>
class PortView {
public:
  PortView(esp::SpiDmaChannel<R>& channel, h::DmaEngine& engine) noexcept
      : channel_(channel), engine_(engine) {}
  [[nodiscard]] h::PortPoll Poll(kb::Instant now) noexcept {
    const esp::SpiDmaResult r = channel_.Poll(now);
    h::PortPoll out{};
    out.state = (r.state == esp::SpiDmaState::Pending) ? h::PollState::Pending
                : (r.state == esp::SpiDmaState::Done)  ? h::PollState::Done
                                                       : h::PollState::Idle;
    out.status = r.status;
    out.count = r.count;
    out.detail = r.flags;
    return out;
  }
  [[nodiscard]] kb::Status Cancel() noexcept {
    return channel_.Cancel();
  }
  template <std::size_t Bytes>
  [[nodiscard]] kb::Status Recover(d::Buffer<R, Bytes>& buffer) noexcept {
    return channel_.Recover(buffer);
  }
  [[nodiscard]] d::Capabilities Capabilities() const noexcept {
    return channel_.Capabilities();
  }
  [[nodiscard]] d::DmaHealth Health() const noexcept {
    return channel_.Health();
  }
  [[nodiscard]] h::PortStats Stats() const noexcept {
    const esp::SpiDmaStats s = channel_.Stats();
    h::PortStats out{};
    out.transfer_errors = s.transfer_errors;
    out.fifo_errors = s.fifo_errors;
    out.completions = s.completions;
    out.timeouts = s.timeouts;
    out.cancels = s.cancels;
    return out;
  }
  /** @brief One cycle of the hardware (the SDK owns the interrupt; the port polls). */
  void Tick() noexcept {
    if (!model::Stalled()) {
      engine_.Tick();
    }
  }

private:
  esp::SpiDmaChannel<R>& channel_;
  h::DmaEngine& engine_;
};

/** @brief The buffer-transfer rig (HostRig's members) over the port, in region `R`. */
template <d::Region R>
// NOLINTNEXTLINE(clang-analyzer-optin.performance.Padding)
class SpiRig {
public:
  using Region = R;
  static constexpr std::size_t kStageBytes = 100U;
  using Stage = d::Buffer<R, kStageBytes>;
  static constexpr std::size_t kNeighbourBytes = 32U;
  static constexpr std::size_t kCallerOffset = 259U;
  static constexpr uint8_t kRxSeed = 0x60U;
  static constexpr std::size_t kCallerBytes = 5U;
  static constexpr std::size_t kCanaryBytes = 16U;
  static constexpr bool kCached = (R::kPolicy == d::CachePolicy::Maintained);

  explicit SpiRig(const Options& options = Options{})
      : machine_(kCached ? R::kLineBytes : 4U, options.speculation),
        engine_(machine_, Engine(options)), bus_(BusConfig(options, staging_)),
        dev_(bus_, DevConfig(10)), fifo_bus_(FifoConfig()), fifo_dev_(fifo_bus_, DevConfig(20)),
        channel_(dev_), view_(channel_, engine_) {
    model::Attach(machine_, engine_);
    const auto key = d::PortKey<SpiRig>{};
    static_cast<void>(machine_.RegisterBuffer(tx_.RawMemory(key), tx_.Header(), kCached));
    static_cast<void>(
        machine_.RegisterBuffer(pack_.stage.RawMemory(key), pack_.stage.Header(), kCached));
    static_cast<void>(machine_.RegisterRaw(pack_.neighbour, kCached));
    static_cast<void>(machine_.RegisterRaw(arena_.mem, kCached));
    const esp::SpiStagingRef s = staging_.Ref();
    static_cast<void>(machine_.RegisterRaw(
        std::span<std::byte>(reinterpret_cast<std::byte*>(s.base), 2U * s.half), false));
    ready_ = (dev_.EnsureReady() == kb::Status::Ok) && (fifo_dev_.EnsureReady() == kb::Status::Ok);
    model::Stall(options.never_ends);
  }
  SpiRig(const SpiRig&) = delete;
  SpiRig& operator=(const SpiRig&) = delete;
  SpiRig(SpiRig&&) = delete;
  SpiRig& operator=(SpiRig&&) = delete;
  // A rig that ended with a failed host proves the stop (the model's host reset) so that its
  // objects release their pins and host claims for the next case.
  ~SpiRig() {
    model::Stall(false);
    static_cast<void>(channel_.Cancel());
    static_cast<void>(channel_.Recover(pack_.stage));
    static_cast<void>(channel_.Recover(tx_));
  }

  [[nodiscard]] bool Ready() const noexcept {
    return ready_;
  }
  [[nodiscard]] h::Machine& M() noexcept {
    return machine_;
  }
  [[nodiscard]] h::DmaEngine& Engine() noexcept {
    return engine_;
  }
  [[nodiscard]] PortView<R>& Port() noexcept {
    return view_;
  }
  [[nodiscard]] esp::SpiDmaChannel<R>& Channel() noexcept {
    return channel_;
  }
  [[nodiscard]] Stage& TxStage() noexcept {
    return tx_;
  }
  [[nodiscard]] Stage& RxStage() noexcept {
    return pack_.stage;
  }
  [[nodiscard]] std::span<std::byte> TxView() noexcept {
    return tx_.CpuView();
  }
  [[nodiscard]] std::span<std::byte> RxView() noexcept {
    return pack_.stage.CpuView();
  }

  void CpuFill(std::span<std::byte> view, std::size_t n, uint8_t seed) noexcept {
    for (std::size_t i = 0U; (i < n) && (i < view.size()); ++i) {
      const auto b = static_cast<std::byte>(static_cast<uint8_t>(seed + i));
      machine_.CpuStore(&view[i], std::span<const std::byte>(&b, 1U));
    }
  }
  [[nodiscard]] bool CpuReadsBack(std::span<std::byte> view, std::size_t n, uint8_t seed) noexcept {
    bool same = view.size() >= n;
    for (std::size_t i = 0U; same && (i < n); ++i) {
      std::byte b{0U};
      machine_.CpuLoad(&view[i], std::span<std::byte>(&b, 1U));
      same = (b == static_cast<std::byte>(static_cast<uint8_t>(seed + i)));
    }
    return same;
  }

  [[nodiscard]] kb::Status StartTx(std::size_t n) noexcept {
    model::NextSeed(0U);
    return channel_.Start(tx_, d::Direction::ToDevice, n, far());
  }
  [[nodiscard]] kb::Status StartRx(std::size_t n, uint8_t seed,
                                   kb::utils::Deadline deadline = kb::utils::Deadline{}) noexcept {
    model::NextSeed(seed);
    return channel_.Start(pack_.stage, d::Direction::FromDevice, n,
                          deadline.Expired(now_) ? far() : deadline);
  }

  [[nodiscard]] kb::Instant Now() const noexcept {
    return now_;
  }
  void AdvanceUs(uint64_t us) noexcept {
    now_ = kb::Instant{now_.Us() + us};
  }
  [[nodiscard]] kb::utils::Deadline DeadlineIn(uint32_t ms) const noexcept {
    return kb::utils::Deadline::After(now_, kb::Milliseconds{ms});
  }
  [[nodiscard]] h::PortPoll RunToDone(std::size_t max_ticks = 4096U) noexcept {
    h::PortPoll poll{};
    poll.state = h::PollState::Pending;
    for (std::size_t i = 0U; (i < max_ticks) && (poll.state == h::PollState::Pending); ++i) {
      view_.Tick();
      poll = view_.Poll(now_);
    }
    return poll;
  }
  void Ticks(std::size_t n) noexcept {
    for (std::size_t i = 0U; i < n; ++i) {
      view_.Tick();
    }
  }

  /** @brief T3: the CPU fills the buffer, then the same bytes go out on the port's FIFO path. */
  void CpuPathCopyIn(std::span<std::byte> view, std::size_t n, uint8_t seed) noexcept {
    CpuFill(view, n, seed);
    std::array<uint8_t, kStageBytes> bytes{};
    for (std::size_t i = 0U; (i < n) && (i < bytes.size()); ++i) {
      bytes[i] = static_cast<uint8_t>(seed + i);
    }
    const std::size_t chunk = (n < esp::kSpiMaxFrameBytes) ? n : esp::kSpiMaxFrameBytes;
    fifo_status_ =
        fifo_dev_.Transfer(std::span<const uint8_t>(bytes.data(), chunk), std::span<uint8_t>{});
  }
  [[nodiscard]] kb::Status FifoStatus() const noexcept {
    return fifo_status_;
  }

  [[nodiscard]] std::byte* Neighbour(std::size_t n) noexcept {
    static_cast<void>(n);
    return pack_.neighbour.data();
  }
  void FillNeighbour(std::size_t n, uint8_t value) noexcept {
    std::array<std::byte, kNeighbourBytes> bytes{};
    bytes.fill(static_cast<std::byte>(value));
    machine_.CpuStore(Neighbour(n), bytes);
  }
  [[nodiscard]] bool NeighbourIntact(std::size_t n, uint8_t value) noexcept {
    std::array<std::byte, kNeighbourBytes> bytes{};
    machine_.CpuLoad(Neighbour(n), bytes);
    bool same = true;
    for (const std::byte b : bytes) {
      same = same && (b == static_cast<std::byte>(value));
    }
    return same;
  }
  [[nodiscard]] bool BeginReceive(std::size_t n, uint8_t seed) noexcept {
    return StartRx(n, seed) == kb::Status::Ok;
  }
  [[nodiscard]] bool FinishReceive() noexcept {
    return RunToDone().status == kb::Status::Ok;
  }
  [[nodiscard]] bool ReceiveIntoTarget(std::size_t n, uint8_t seed) noexcept {
    return BeginReceive(n, seed) && FinishReceive();
  }
  [[nodiscard]] bool TargetReadsBack(std::size_t n, uint8_t seed) noexcept {
    return CpuReadsBack(pack_.stage.CpuView(), n, seed);
  }

  [[nodiscard]] std::span<std::byte> CallerBuffer() noexcept {
    return {arena_.mem.data() + kCallerOffset, kCallerBytes};
  }
  void ArmCallerCanaries() noexcept {
    std::array<std::byte, kCanaryBytes> pattern{};
    pattern.fill(std::byte{0xC3U});
    machine_.CpuStore(arena_.mem.data() + kCallerOffset - kCanaryBytes, pattern);
    machine_.CpuStore(arena_.mem.data() + kCallerOffset + kCallerBytes, pattern);
    machine_.EvictAll();
  }
  [[nodiscard]] bool CallerCanariesIntact() noexcept {
    std::array<std::byte, kCanaryBytes> before{};
    std::array<std::byte, kCanaryBytes> after{};
    machine_.CpuLoad(arena_.mem.data() + kCallerOffset - kCanaryBytes, before);
    machine_.CpuLoad(arena_.mem.data() + kCallerOffset + kCallerBytes, after);
    bool intact = true;
    for (std::size_t i = 0U; i < kCanaryBytes; ++i) {
      intact = intact && (before[i] == std::byte{0xC3U}) && (after[i] == std::byte{0xC3U});
    }
    return intact;
  }

  /** @brief T18: the port's real synchronous Transfer (copy through the staging buffer). */
  [[nodiscard]] kb::Status SyncTransfer(std::span<const std::byte> tx,
                                        std::span<std::byte> rx) noexcept {
    kb::Status status = kb::Status::Ok;
    if (!tx.empty()) {
      model::NextSeed(0U);
      status = dev_.Transfer(
          std::span<const uint8_t>(reinterpret_cast<const uint8_t*>(tx.data()), tx.size()),
          std::span<uint8_t>{});
    }
    if ((status == kb::Status::Ok) && !rx.empty()) {
      model::NextSeed(kRxSeed);
      status = dev_.Transfer(std::span<const uint8_t>{},
                             std::span<uint8_t>(reinterpret_cast<uint8_t*>(rx.data()), rx.size()));
    }
    return status;
  }

private:
  struct alignas(128) Arena {
    std::array<std::byte, 512> mem{};
  };
  struct Pack {
    Stage stage;
    // One whole line of the largest cache model (the machine registers whole lines only).
    alignas(64) std::array<std::byte, 64> neighbour{};
  };

  static h::EngineConfig Engine(const Options& o) noexcept {
    h::EngineConfig c{};
    c.round_rx_to = o.round_rx_to;
    return c;
  }
  static esp::SpiBusConfig BusConfig(const Options& o, esp::SpiStaging<128U>& staging) noexcept {
    esp::SpiBusConfig c{};
    c.host = 1U;
    c.mosi_pin = 11;
    c.miso_pin = 13;
    c.sclk_pin = 12;
    c.dma = o.dma;
    c.staging = staging.Ref();
    return c;
  }
  static esp::SpiBusConfig FifoConfig() noexcept {
    esp::SpiBusConfig c{};
    c.host = 2U;
    c.mosi_pin = 21;
    c.miso_pin = 23;
    c.sclk_pin = 22;
    return c;
  }
  static esp::SpiDeviceConfig DevConfig(int16_t cs) noexcept {
    esp::SpiDeviceConfig c{};
    c.cs_pin = cs;
    c.clock_hz = 1000000U;
    return c;
  }
  [[nodiscard]] kb::utils::Deadline far() const noexcept {
    return kb::utils::Deadline::After(now_, kb::Milliseconds{1000000U});
  }

  h::Machine machine_;
  h::DmaEngine engine_;
  esp::SpiStaging<128U> staging_{};
  esp::SpiBus bus_;
  esp::SpiDevice dev_;
  esp::SpiBus fifo_bus_;
  esp::SpiDevice fifo_dev_;
  esp::SpiDmaChannel<R> channel_;
  PortView<R> view_;
  Stage tx_;
  Pack pack_{};
  Arena arena_{};
  kb::Instant now_;
  kb::Status fifo_status_{kb::Status::Ok};
  bool ready_{false};
};

/** @brief The stream rig (AdcRig's members) over the port's real AdcStream `S`: the SDK's DMA ring
 * is modelled by kb-core's HwRing, the SDK's interrupt calls the port's registered entry points. */
template <typename S>
class AdcRig {
public:
  using HwBuffer = d::Buffer<h::ModelUncached, 2U * sizeof(h::Scan)>;

  explicit AdcRig(S& stream) noexcept : stream_(stream), machine_(1U, false), engine_(machine_) {
    model::Attach(machine_, engine_);
    static_cast<void>(machine_.RegisterBuffer(hw_.RawMemory(Key()), hw_.Header(), false));
  }

  [[nodiscard]] d::PortKey<AdcRig> Key() const noexcept {
    return d::PortKey<AdcRig>{};
  }
  /** @brief The port's EnsureReady() and Start(now); the SDK's DMA buffer goes to the ring. */
  [[nodiscard]] kb::Status Start(kb::Instant now) noexcept {
    kb::Status status = stream_.EnsureReady();
    if (status == kb::Status::Ok) {
      status = hw_.Lend(Key(), d::Direction::FromDevice, HwBuffer::kBytes, ops_, lease_);
    }
    if (status == kb::Status::Ok) {
      ring_.Attach(machine_, lease_.DeviceView(Key()).data());
      status = stream_.Start(now);
    }
    return status;
  }
  void Advance(std::size_t n) noexcept {
    ring_.Advance(n);
  }
  void RunHalves(std::size_t halves, kb::Instant at) noexcept {
    for (std::size_t i = 0U; i < halves; ++i) {
      ring_.Advance(h::HwRing::kSamples);
      Isr(at);
    }
  }
  /** @brief ESP-IDF's ADC interrupt: overflow first (on_pool_ovf), then each finished buffer
   * (on_conv_done); it hands over buffers, it cannot tell the port that one was lapped. */
  void Isr(kb::Instant at) noexcept {
    esp::AdcStreamSink* const sink = model::Sink();
    const uint32_t flags = ring_.Flags();
    uint32_t handled = 0U;
    if ((sink != nullptr) && ((flags & h::kRingOverrun) != 0U)) {
      sink->overflow(*sink, at.Us());
      handled |= h::kRingOverrun;
    }
    if ((sink != nullptr) && ((flags & h::kRingHalfTransfer) != 0U)) {
      deliver(*sink, 0U, at);
      handled |= h::kRingHalfTransfer;
    }
    if ((sink != nullptr) && ((flags & h::kRingTransferComplete) != 0U)) {
      deliver(*sink, 1U, at);
      handled |= h::kRingTransferComplete;
    }
    ring_.ClearFlags(handled);
  }
  /** @brief The port has no task-side drain of the interrupt's flags: nothing to do (T11). */
  void TaskDrainPreempted(kb::Instant at) noexcept {
    static_cast<void>(at);
  }
  void RaiseOverrun() noexcept {
    ring_.RaiseOverrun();
  }
  [[nodiscard]] uint32_t HalvesDone() const noexcept {
    return ring_.Halves();
  }
  [[nodiscard]] S& StreamRef() noexcept {
    return stream_;
  }

private:
  void deliver(esp::AdcStreamSink& sink, std::size_t half, kb::Instant at) noexcept {
    std::array<std::byte, sizeof(h::Scan)> raw{};
    machine_.PortLoad(ring_.Memory() + (half * sizeof(h::Scan)), raw);
    const auto bytes = std::bit_cast<std::array<uint8_t, sizeof(h::Scan)>>(raw);
    sink.frame(sink, bytes, at.Us()); // misra-lint: ignore 0.1.2
  }

  S& stream_;
  h::Machine machine_;
  h::DmaEngine engine_;
  d::NoCacheOps ops_{};
  HwBuffer hw_;
  d::Lease<h::ModelUncached> lease_;
  h::HwRing ring_{};
};

} // namespace rig

#endif
