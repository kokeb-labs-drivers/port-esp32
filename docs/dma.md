# DMA

How the ESP32 port uses DMA, in the fixed form of every kb-core port: regions, masters, rules, SDK behaviour handled,
conformance, and what was verified on hardware. The vocabulary (`kb::dma::Buffer`, `Lease`, `Stream`, `BusLimits`,
`Capabilities`) is kb-core's `kb::dma`; the port's part is in `dma.hpp`, `spi.hpp`, `spi_dma.hpp` and
`adc_stream.hpp`.

| What | Class | DMA through |
| --- | --- | --- |
| Synchronous SPI frames, any caller buffer | `SpiDevice::Transfer` on an `SpiBus` with `dma` | the host's staging buffer (copy in and out) |
| Zero-copy SPI of a typed buffer | `SpiDmaChannel<Region>` | a `kb::dma::Lease` of a board `kb::dma::Buffer` |
| Continuous ADC | `AdcStream<Frame, Frames>` (`AdcStreamDefault`) | ESP-IDF `adc_continuous`, copied into a `kb::dma::Stream` |

The port never allocates a GDMA channel itself: the GDMA API is private in ESP-IDF v5.5. It uses the drivers that own
their channels, `spi_master` (`SPI_DMA_CH_AUTO`) and `adc_continuous`; each takes one channel pair at its start.

## Regions

| Tag | Policy | Line | Align | Masters | Memory |
| --- | --- | --- | --- | --- | --- |
| `InternalRam` | Coherent | 1 | 4 | GDMA, ESP32 SPI DMA, ESP32 I2S0 | internal SRAM of the ESP32, S2, S3, C3, C6, H2 |
| `InternalRamL1` | Maintained | 64 | 64 | GDMA (AHB, AXI) | internal RAM of the ESP32-P4 (behind the L1 data cache) |
| `Psram<L>` | Maintained | L | L | GDMA | ESP32-S3 PSRAM, `L` = `CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE` (16, 32, 64) |

The port's own transfers use internal RAM only. A `Maintained` region's cache work is done by `EspCacheOps`
(`esp_cache_msync`: C2M before the DMA reads, M2C before the CPU reads what it wrote), at the lease transitions, over
whole lines of one buffer; drivers and handlers never call a cache function. A transfer by CPU copies (the FIFO path)
does no cache work at all.

## Masters and channel budget

| Bit | Master | Targets | Pairs |
| --- | --- | --- | --- |
| `kMasterGdma` | AHB GDMA | S3, C6, P4 (and C3, H2) | S3: 5, C6: 3, P4: 3 AHB |
| `kMasterSpiDma` | dedicated SPI DMA | ESP32 (SPI2, SPI3) | one per host |
| `kMasterI2sDma` | I2S0 DMA (ADC continuous) | ESP32 | one |
| `kMasterAxiGdma` | AXI GDMA | P4 | 3 |

The ESP32-C6's three pairs are the tightest budget: an SPI host with DMA and ADC continuous take two. When none is
free, `spi_bus_initialize` returns `ESP_ERR_NOT_FOUND`: `SpiBus::EnsureReady()` fails with `Unsupported`, or, with
`SpiBusConfig::dma_fallback`, runs the host without DMA and says so (below).

## Rules

**SPI staging (every DMA host).** One port-owned staging buffer per host, a transmit and a receive half: the board's
static `SpiStaging<N>` in `SpiBusConfig::staging` (namespace scope, so internal RAM), or, without one, one block from
`heap_caps_aligned_alloc` taken once in `SpiBus::EnsureReady()`. Each half is the host's largest frame rounded up to
64 bytes (the largest unit of every target), at most 4092. `EnsureReady()` checks the memory with
`esp_ptr_dma_capable` and `esp_dma_is_buffer_alignment_satisfied` and refuses it otherwise.

**No allocation after start.** Every DMA transaction (polling or queued) carries `SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL`:
ESP-IDF refuses (`ESP_ERR_INVALID_ARG`) rather than allocating a bounce buffer. The staging buffers pass its check, so
nothing is allocated per transfer, by the port or by the SDK.

**Any caller buffer.** `SpiDevice::Transfer` copies `tx` into the staging buffer and `rx` out of it under the bus
lock: unaligned, odd-length, stack, member or PSRAM buffers all work, and the caller's memory is never a DMA target.

**Receive padding per target.** ESP-IDF rounds every SPI receive descriptor up to whole words, and the ESP32 writes up
to 3 bytes past an odd length; the staging covers it (`SpiBusLimits(target).rx_length_multiple` = 4). The length the
SDK gets is the frame length. On the ESP32-P4 a DMA receive must be whole 64-byte lines in address and length, or
ESP-IDF would bounce it; with the manual-alignment flag the port refuses such a frame (`InvalidArgument`, before the
bus) and `SpiBusLimits(Target::Esp32P4)` says 64 at build time.

**Late completion.** A started ESP-IDF SPI transaction cannot be aborted. When a frame ends late (after its wire time
W plus the timeout T) the port waits once more, W + T; when it still has not ended the host is failed and the staging
buffer stays held (counted in `Health().quarantined`) until the host is torn down. A late DMA write only ever reaches
port memory.

**Zero-copy only through a lease.** `SpiDmaChannel<R>::Start` lends a `kb::dma::Buffer<R, N>` (the CPU has no view
while it is out), queues one DMA transaction over it and holds the bus lock until the lease is settled: `Poll` reports
the end once (`Ok`, or `Fault` with the raw flags when ESP-IDF reports a DMA overflow or underflow); an overdue
transfer waits once more and is then `Timeout`, with the buffer back (ended) or quarantined (not ended); `Cancel`
returns `Ok` only once ESP-IDF reported the end, otherwise `Fault` and the buffer is quarantined; `Recover` returns a
quarantined buffer only on proof of the stop. One direction per transfer: `ToDevice` has no MISO phase, `FromDevice`
no MOSI phase.

**Limits at build time.** `SpiLimits<Target, Dma, StagingBytes>::kBus` is a `kb::dma::BusLimits`; a binding writes
`kb::dma::RequireBusCarries<Bus, Driver::kMaxFrameBytes>()` and a frame the bus cannot carry fails the build.

**Capabilities and health, never silent.** `SpiBus::Capabilities()`, `SpiDmaChannel::Capabilities()` and
`AdcStream::Capabilities()` say the mode (`Dma` or `Polled`), the ceiling, the staging size and the alignment unit;
`Health()` adds the failed host, the quarantines and the DMA errors. A fallback (no DMA channel, or an ADC shape that
does not match) shows as `Polled` in both, and `LastError()` keeps the cause.

**Half-duplex on the ESP32.** The port runs every device full-duplex. A board or binding that adds a half-duplex
device with DMA states its phases with `RequireSpiHalfDuplexPhases<Target, Dma, Write, Read>()`; both phases on the
ESP32 fail the build (ESP-IDF "Known Issues").

**Interrupt levels.** `kInterruptLimits` maps a kb priority value p onto ESP-IDF level 7 - p: levels 1 to 3 run C
handlers that may call FreeRTOS. `SpiBusConfig::interrupt_level` takes `InterruptLevelOf(assignment.irq_priority)` and
is passed to `spi_bus_initialize` as `ESP_INTR_FLAG_LEVELn`; ESP-IDF allocates exactly that level or fails, so a
different level cannot be substituted silently. A level above 3 is refused before the SDK;
`kb::dma::RequireInterruptTier<Stream, EspRtosLimits>()` refuses a task-notifying stream above the RTOS limit at build
time.

**ADC continuous into a kb stream.** `AdcStream::EnsureReady()` creates the driver (`adc_continuous_new_handle`: its
pool, frames and descriptors are allocated there, at start) with `flush_pool` off, configures the pattern (one unit)
and registers two separate IRAM callbacks. `on_conv_done` copies the finished frame into the next slot of the kb
stream with the time read in that interrupt; the driver's frame pointer never leaves the callback. `on_pool_ovf` calls
`OnOverrun()`: an overflow is counted and never a frame, and the next frame carries `Gap()`. The owner task reads the
kb stream (`TryTake`, `Give`); each `TryTake` empties the SDK's pool with `adc_continuous_read(..., 0)`, never with a
millisecond timeout, so an overflow means the owner fell behind. A frame that arrives later than (buffers - 1) frame
periods after the previous one, and the rest of that burst, are published `Torn`: the SDK's ring of five buffers may
have been refilled before the interrupt copied them.

**ADC frame shape.** `conv_frame_size` is `sizeof(Frame)`: a whole number of `SOC_ADC_DIGI_DATA_BYTES_PER_CONV`, and
of the cache line on the ESP32-P4. The board's `kb::dma::StreamShape` must match (frames, the chip's result size,
results per frame). A mismatch refuses DMA (`NotReady`, detail `ESP_ERR_INVALID_SIZE`); the stream then runs from the
declared fallback producer, one-shot conversions the handler publishes with `PublishFallback()`, flagged
`Fallback()`, with `Capabilities()` and `Health()` saying `Polled`.

**IRAM safety.** With `AdcStreamConfig::iram_safe` (flash is written while streaming) the port needs
`CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE` (`Unsupported` without it), the interrupt path in IRAM and the stream object in
internal RAM (`InvalidArgument` otherwise). The path is in IRAM for `AdcStreamDefault`: its instantiation is compiled
once in `src/adc_stream.cpp`, which the component's `linker.lf` places in IRAM with that option, and the values the
interrupt needs are computed at `EnsureReady()`. In the ESP32-C6 and ESP32-S3 bench builds every function on that path
(the two callbacks, the stream's entry points, `kb::dma::Stream::Publish`, `memcpy`, `esp_timer_get_time`) links to
IRAM or ROM. Set `CONFIG_ESP_TIMER_IN_IRAM` too.

## SDK behaviour handled

| Behaviour (ESP-IDF v5.5) | Source | Handled by |
| --- | --- | --- |
| The SPI master allocates a DMA-capable copy of a buffer that is not DMA-capable or not aligned, per transaction, unless the manual-alignment flag is set | `spi_master.c` `setup_dma_priv_buffer` | staging buffers and `SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL` |
| SPI DMA receive descriptors round up to whole words; on the ESP32 the DMA may overwrite up to 3 bytes past the buffer | `spi_common.c` descriptor set-up; `spi_master.rst` ESP32 notes | padded staging; caller memory never a target |
| On a chip with a cache in front of internal RAM (P4) a receive must be whole cache lines in address and length | `setup_dma_priv_buffer` | refusal before the bus; `SpiBusLimits` |
| A started SPI transaction cannot be aborted; `polling_end` and `get_trans_result` end it or time out | `spi_master.c` | wait once more, then quarantine and failed host |
| `get_trans_result` reports a DMA receive overflow or transmit underflow as `ESP_ERR_INVALID_STATE` with `SPI_TRANS_DMA_RX_FAIL` / `_TX_FAIL` | `spi_master.c` | `Fault` with the flags, counted |
| No free GDMA channel: `ESP_ERR_NOT_FOUND` | `spi_bus_initialize` | `Unsupported`, or the reported fallback |
| Half-duplex DMA with both phases is not supported on the ESP32 | `spi_master.rst` "Known Issues" | build-time refusal |
| The ADC continuous pool-overflow callback receives no event data | `adc_continuous.c`, `adc_continuous.rst` "Pool Overflow Event" | separate callback, `OnOverrun()` |
| The ADC continuous driver reuses its five frame buffers | `adc_continuous.c` `INTERNAL_BUF_NUM` | copy in the callback; lap check |
| `adc_continuous_read` converts its millisecond timeout to ticks by rounding down | `adc_continuous.c` | never called with a timeout other than 0 |
| With `CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE` callbacks and everything they touch must be in IRAM / DRAM | `adc_continuous.rst` "IRAM Safe" | `linker.lf`, start-time check |

Observed with an earlier implementation on ESP32-C6 hardware (ESP-IDF v5.5), and the reason for the ADC rules:
ADC continuous mode ran at 1 kHz with 64-sample frames and a four-frame pool, the conversion-done callback in IRAM,
and 12 of 12 ADC test cases passed. Sharing one callback for data and overflow makes an overflow look like an empty
frame, so overflow is a separate, counted signal here.

## Conformance

The kb-core DMA conformance suite (`kb/dma/host/conformance.hpp`, cases T1 to T18) runs in
`tests/test_conformance.cpp` against the port's real classes, with the port's DMA logic (`spi.cpp`, `spi_dma.cpp`,
`dma.cpp`, `adc_stream.cpp`) linked against kb-core's host DMA-and-cache model by a second implementation of its
boundary calls (`tests/dma_model/model_boundary.cpp`). The rigs (`tests/dma_model/port_rig.hpp`) have the member
functions of kb-core's `HostRig` and `AdcRig`; `Port()` is a forwarding view of the real `SpiDmaChannel` that only
converts its poll and stats records into the suite's types. The buffer cases run twice: with the port's `Coherent`
region (`InternalRam`, the C6 and S3 case, no cache) and with its `Maintained` region (`InternalRamL1`, 64-byte lines),
so T1 to T4 meet the cache model.

| Case | Coherent | Maintained | Stream | How it applies to the ESP32 |
| --- | --- | --- | --- | --- |
| T1 transmit needs a clean | pass | pass | | The clean is `esp_cache_msync` C2M at the lend (Maintained); no cache in front of internal RAM otherwise |
| T2 receive needs an invalidate | pass | pass | | M2C at the lend and at the completion, with speculative refills on |
| T3 CPU path does no maintenance | pass | pass | | The rig runs the bytes through the port's FIFO path (a second, non-DMA host): no cache call |
| T4 neighbour on a shared line | pass | pass | | `kb::dma::Buffer` owns whole lines |
| T5 no CPU view during a lease | pass | pass | | |
| T6 no DMA write after a timeout | pass | pass | | The overdue transfer is waited for once more; not ended, the buffer is quarantined, so the late write lands in a buffer nobody uses |
| T7 abort waits for the end | pass | pass | | ESP-IDF has no stop request: a cancel waits (W + T) for ESP-IDF to report the end |
| T8 receive rounding stays inside | pass | pass | | Run with the engine rounding receives to whole words (the ESP32 rule) |
| T9 held frame stable, overrun shows | | | pass | |
| T10 lapped half flagged | | | pass | The SDK cannot report lapping; the port flags frames that arrive after a hold-off of (buffers - 1) periods |
| T11 one drainer | | | pass | The port has no task-side drain of the interrupt's flags (the SDK owns them; the owner only empties the SDK's data pool) |
| T12 signed stall check | | | pass | |
| T13 overflow is not data | | | pass | `on_pool_ovf` is its own callback |
| T14 error flags | pass | pass | | The SDK owns the SPI interrupt and clears its flags; the model boundary plays that part. The port maps the reported error to one `Fault` with the flags and counts it |
| T15 cancel racing completion | pass | pass | | |
| T16 unconfirmed stop quarantines | pass | pass | | Modelled as a transaction ESP-IDF never reports finished. The recovery leg uses the model's host reset; on ESP-IDF v5.5 `SpiHostReset` returns `ESP_ERR_NOT_SUPPORTED`, so `Recover()` stays `Fault` until the SDK reports the end (unit test `spi dma: a cancel returns Ok only once ESP-IDF reported the end`) |
| T17 fallback is reported | pass | pass | | A host without DMA: `Polled`, ceiling 64 bytes |
| T18 sync transfer through staging | pass | pass | | The port's real `SpiDevice::Transfer`, receive rounding on |

Every case is printed as `CONFORMANCE <case> <variant> PASS|FAIL` by the test binary. The suite was also run against
seven deliberate defects of the port, each removing one guard; every one was caught: an overdue transfer returned
without its end (T6), a cancel that does not wait (T7), the receive straight into the caller's buffer (T18), an
overflow published as a frame (T13), no lap check (T10), an error reported as success (T14), an unconfirmed stop
returning the buffer (T16).

Port items beyond T1 to T18:

| Item | Status | Test |
| --- | --- | --- |
| B31 error flag with nothing in flight | On the ESP32 the SDK owns the interrupts and their flags. The port's form: a pool overflow with no frame pending is counted once, publishes nothing, and a second interrupt entry finds nothing to do | `port item B31` (conformance) |
| B32 interrupt priority from the assignment | Level written as `ESP_INTR_FLAG_LEVELn` from `InterruptLevelOf(irq_priority)`; ESP-IDF allocates that level or fails (no read-back API in v5.5; the allocation itself is the check); a level above 3 refused; `RequireInterruptTier` at build time | `dma: interrupt levels ...`, compile-fail `notify_above_esp_limit` |
| B33 conversion count vs frame | Shape mismatch refuses DMA, fallback reported | `adc stream: the board's shape must match ...`, `port item B33` |
| Signed stall time | `kb::dma::Stream::CheckStall` | T12, `adc stream: a stopped trigger is a stall ...` |
| Overrun counted | ring overrun and pool overflow, separately | T9, T13, `adc stream: a pool overflow ...` |
| Single drainer | no task drain | T11 |
| Seqlock publication | Not part of the port: derived values are published by the owner task (handler); kb-core has no `SeqlockSnapshot` yet | — |
| Idempotent start | a second `Start` of a running stream is `Ok` and starts nothing | `port item: a second start ...` |

## Verified on hardware

Nothing of the DMA paths of this port has run on a board yet. The bench kit is ready: the apps in `tests/target/dma/`
(built in CI for the ESP32-C6, ESP32-S3 and ESP32) and the procedure, wiring and evidence form in
[the bench folder](assurance/bench/README.md). Filled forms go to `docs/assurance/bench/runs/`; this section will list
each item with its run once one exists.
