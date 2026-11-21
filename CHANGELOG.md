# Changelog

All notable changes of port-esp32. The format follows Keep a Changelog; versions follow Semantic Versioning (before
1.0 a minor version may break).

## [Unreleased]

### Added

- Figures: a matrix of the bus contracts the port implements (native, adapted, planned, not available) on the home
  page, and a sequence of one transaction through the port on the I2C, SPI, UART and CAN pages, drawn by figkit from
  models in `docs/model/` (light and dark SVG, readable on a phone).
- CI path filter: a pull request that changes only documentation runs only the docs, handbook and DCO checks; the
  gate allows the code jobs to be skipped only then. The docs publish only from main and `v*` tags.
- DMA (docs/dma.md), on kb-core's `kb::dma`: `dma.hpp` (the regions `InternalRam`, `InternalRamL1`, `Psram<L>`, the
  DMA masters, `SpiLimits<Target, Dma, StagingBytes>` as `kb::dma::BusLimits`, the static `SpiStaging<N>`,
  `kInterruptLimits` and `InterruptLevelOf`, `RequireSpiHalfDuplexPhases`, `EspCacheOps` over `esp_cache_msync`);
  `spi_dma.hpp` (`SpiDmaChannel<Region>`: zero-copy SPI DMA of a `kb::dma::Buffer` through a `kb::dma::Lease`, with
  `Start`, `Poll`, `Cancel` and `Recover`); `adc_stream.hpp` (`AdcStream<Frame, Frames>` and `AdcStreamDefault`: ADC
  continuous mode into a `kb::dma::Stream`, the frame copied in the conversion-done interrupt with its time, the pool
  overflow a separate counted signal, a start-time shape check with a reported one-shot fallback, an IRAM-safe
  interrupt path placed by the component's `linker.lf`).
- SPI: `SpiBusConfig::staging` (a board's static staging buffer, checked at start), `interrupt_level` (the ESP-IDF
  level from the board's stream assignment), `dma_fallback` (no free DMA channel: run without DMA, reported);
  `SpiBus::Capabilities()`, `Health()`, `DmaActive()`.
- The kb-core DMA conformance suite, T1 to T18, run against the port's classes linked against kb-core's host
  DMA-and-cache model (`kb_port_esp32_conformance`, `tests/dma_model/`).
- The DMA bench kit: the apps `dma_spi_bench` and `dma_adc_bench` in `tests/target/dma/` (built in CI for esp32c6,
  esp32s3 and esp32), the procedure for an ESP32-C6, an ESP32-S3 and an ESP32 devkit with wiring figures, the evidence
  form and the runs folder (`docs/assurance/bench/`).

### Changed

- SPI with DMA: every DMA transaction carries `SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL`, so ESP-IDF refuses instead of
  allocating a bounce buffer; nothing is allocated per transfer on any target. Breaking on the ESP32-P4: a DMA
  receive whose length is not a whole number of 64-byte cache lines is refused with `InvalidArgument` (before, ESP-IDF
  bounced it through the heap per transfer).
- SPI with DMA and no free DMA channel: `EnsureReady()` returns `Unsupported` (was `NoDevice`).
- The staging buffer of a host whose transaction never ended stays held until the host is torn down, and is counted
  in `Health().quarantined`.
- The kb-core copy adds the `dma` part and its host model (`components/lib/dma/{CMakeLists.txt,inc,host}`);
  `tools/kb_core_snapshot.py` copies what kb-core's `tools/layout/layout.py vendor` lists.

- The port stands alone: the pcal9555, tmc5160 and bno08x driver submodules, `drivers/drivers.lock.yml`, the pin-hygiene
  workflow and the driver examples are removed; the one example is `port_selftest` (it also brings up the I2C and SPI
  buses). Chip drivers are bound through the port in their own repositories.
- The kb-core copy follows kb-core's `components/` layout (kb-core `main` at 2976aaf: `lib/` -> `components/lib/`,
  the host RTOS backend in `components/ports/host/`, plus `tools/layout/paths.yml`, which kb-core's CMake reads); 65 files
  in the lock.
- Source layout: every public header has a source file of the same name (`src/adc.cpp` for `adc.hpp`, and so
  on; `error.hpp` and `version.hpp` are header-only), and the SDK boundary is one file per SDK component in
  `src/boundary/` (`i2c.cpp`, `spi.cpp`, `uart.cpp`, `gpio.cpp`, `adc.cpp`, `pwm.cpp`, `can.cpp`, `no_can.cpp`,
  `time.cpp`, `log.cpp`, `error.cpp`). The combined files (`adc_pwm.cpp`, `can_time_log.cpp`,
  `esp32_*_boundary.cpp`) are gone; `i2c_controller.cpp` is `i2c.cpp` and `spi_device.cpp` is `spi.cpp`.
  `tools/check_layout.py` (preflight and ctest) fails on a header without its source or a file that matches no
  header. No behaviour change; the MISRA records name the new files.

### Fixed

- SPI: every transfer was refused on ESP-IDF v5.5, which takes only `portMAX_DELAY` in `polling_start`. A frame is
  one polling transaction; the end wait is the frame's wire time plus the timeout, then once more; a transaction
  that never ends marks the host failed instead of hanging a task or the host.
- SPI with DMA: frames go through a port-owned DMA buffer per host, allocated once, so any caller buffer works
  and the DMA's word rounding of receive lengths never writes past a caller's buffer. On chips with a cache in
  front of internal RAM (ESP32-P4) ESP-IDF still bounces a receive not in whole cache lines per transfer; a
  transient lack of heap there is `Busy`. An SPI timeout too long for any frame is refused at `EnsureReady()`.
- I2C: failures are classified in three time bands (ESP-IDF returns one code for NACK and stretch timeout; v6
  another one): faster than the earliest timeout a NACK, slower than the wire time a timeout, between unsplit;
  `ESP_ERR_TIMEOUT` is reported as `Busy`. Millisecond timeouts reach ESP-IDF as whole ticks plus one.
- `LastError()` and `LastFailure()` of shared objects report the calling task's own call; counters are atomic;
  `Release()` and the destructors take the object's lock; CAN `Release()` waits for running calls.
- Every timeout is capped at `kMaxTimeout` (2 s); kb-core's "no limit" never reaches ESP-IDF as a wait for ever.
  An SPI frame whose worst case would exceed it is refused. Each peripheral page gives a call's worst case.
- Controller numbers (I2C, UART, SPI host, TWAI) are claimed before the SDK is called: a second object on the
  same controller is refused and cannot disturb the first.
- PWM: channels share a timer only at one frequency, the duty is clamped to the period, and an idle-high channel
  shows high from the start.
- GPIO: `Release()` and the destructor drive a safe level instead of `gpio_reset_pin` (which enabled the
  pull-up); an ISR service installed by the application takes only IRAM callbacks unless stated otherwise.
- UART: a full event queue is reported as lost events (`Overflow`, `kEventsLost`); `WaitTxDone()` takes the
  writer lock.
- A failed RTOS lock creation fails `EnsureReady()`; `LogSink` refuses a null tag.

### Added

- `I2cConfig::stretch_limit_us`, per-call I2C timeouts, `I2cController::AllowNackProbe()` (the probe after a NACK
  is now opt-in per address), a bus reset at `EnsureReady()`.
- Pin claims across every peripheral; `OutputConfig::safe`; `InputConfig::foreign_isr_service_ok`;
  `UartConfig::event_queue`; `AdcConfig::default_vref_mv`; `SpiDevice::ActualClockHz()`; `SpiStats::late`;
  `SpiBusConfig::dma_buffer_bytes`, `SpiBus::MaxFrameBytes()`, `kSpiMaxFrameBytes`, `kSpiMaxDmaFrameBytes`.
- CI calls ci-tools at a commit SHA and names the ESP-IDF image by digest.

### Changed

- **Breaking:** an SPI frame is one transaction: at most `kSpiMaxFrameBytes` (64) without DMA and the host's
  `dma_buffer_bytes` (at most `kSpiMaxDmaFrameBytes`, 4092) with DMA; longer frames are refused with
  `InvalidArgument`. 0.1.0-rc.1 split longer frames into chip-select-held chunks. isf15acp4 (96-byte data
  chunks) needs DMA or chunks of 60 bytes or less; bmp581 and bno08x over SPI need DMA.
- **Breaking:** `SpiStats::rebuilds` is replaced by `SpiStats::late`; SPI clocks below 1 kHz are refused.
- With DMA, an SPI host allocates its DMA buffer (twice `dma_buffer_bytes`) once in `EnsureReady()`.
- The kb-core copy is kb-core `main`; `tools/kb_core_snapshot.py` refuses commits that are neither tagged nor on
  kb-core's main branch.

## [0.1.0-rc.1]

First release candidate: kb-core's contracts over ESP-IDF v5.5 for the ESP32 family.

### Added

- `I2cController` (`I2cBus`): per-transfer addressing, address and data NACK told apart, stuck-bus detection and
  nine-clock recovery, statistics, a bounded lock.
- `SpiBus` and `SpiDevice` (`SpiDevice`, `SpiClockControl`): frames of any length with chip select held, clock
  changes, re-creation after an SDK timeout, a bounded bus lock.
- `Uart` (`UartPort`, `UartBaudControl`, `UartTxDrain`): a FIFO writer bounded by its timeout, a first-byte reader,
  latched receive errors, RS-485 half duplex, separate reader and writer locks.
- `OutputPin` and `InputPin` (`GpioOutput`, `GpioInput`, `GpioInterrupt`), with pin claims.
- `AdcChannel` (`AdcChannel`) with shared units and eFuse calibration (curve or line fitting).
- `PwmChannel` (`PwmOutput`) on LEDC with the finest resolution the clock allows.
- `CanController` (`CanBus`) on TWAI with bus-off recovery; `Unsupported` on chips without TWAI.
- `Clock` (`MonotonicClock`), `BusyWait` (`BusyWait`), `LogSink` (`LogSink`).
- The `esp_err_t` to `kb::Status` mapping with the vendor code kept in `Detail()`.
- Host tests over fakes of every ESP-IDF call, compile-fail tests, header checks; ESP-IDF examples binding the
  pcal9555, tmc5160 and bno08x drivers for every ESP32 target.
