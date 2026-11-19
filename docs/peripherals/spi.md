# SPI

`kb::port::esp32::SpiBus` owns one SPI host; `kb::port::esp32::SpiDevice` is one chip select on it and
satisfies `kb::iface::SpiDevice` and `kb::iface::SpiClockControl`. ESP-IDF `spi_master`, polling transactions.

## How it works

| Step | SDK call |
| --- | --- |
| `SpiBus::EnsureReady()` | Claims the host and SCLK, MOSI and MISO, then `spi_bus_initialize` (`SPI_DMA_DISABLED`, or `SPI_DMA_CH_AUTO` with `dma`, `max_transfer_sz` = `dma_buffer_bytes`); it fails when another component initialised the host, so the port owns every host it uses. With `dma`, the host's DMA buffer is allocated once (`heap_caps_aligned_alloc`, `MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL`, two halves of `dma_buffer_bytes` aligned to the DMA and cache rules); no room is `Fault` |
| `SpiDevice::EnsureReady()` | The bus, a device slot and the CS pin claimed, then `spi_bus_add_device` (mode, clock, CS pin, CS setup and hold cycles, queue of 1) and `spi_device_get_actual_freq` |
| `Transfer(tx, rx)` | One transaction: `spi_device_polling_start` and `spi_device_polling_end` (see the timing limits) |
| `SetClockHz(hz)` | `spi_bus_remove_device` and `spi_bus_add_device` with the new clock (ESP-IDF fixes the clock per device); on a refusal the device is added back at the old clock |
| `Release()` | `spi_bus_remove_device` under the bus lock; `SpiBus::Release()` is `spi_bus_free` under the bus lock |

A frame is one transaction, so the hardware holds chip select for the whole frame. A read-only frame (`tx` empty)
clocks zeros from an internal, word-aligned zero buffer of 64 bytes. The sampling point is ESP-IDF's default
(`SPI_SAMPLING_POINT_PHASE_0`, the zero-initialised device configuration), the only one the ESP32-C6 supports
(observed on ESP32-C6 and ESP32-S3 with an earlier implementation). The port never calls `spi_device_acquire_bus`:
ESP-IDF v5.5 takes only `portMAX_DELAY` there, and it is what a frame split into several chip-select-held
transactions would need, so frames are never split.

## Pins and clock

| Item | Note |
| --- | --- |
| Host | `SpiBusConfig::host` is ESP-IDF's `spi_host_device_t`: 1 is SPI2, 2 is SPI3 (where it exists). SPI1 is the flash bus and is refused |
| Pins | Any GPIO through the GPIO matrix; the IO MUX pins of the host allow the full 80 MHz |
| Clock | `clock_hz` up to 80 MHz on IO MUX pins, about 26 MHz through the GPIO matrix (ESP-IDF rounds to the nearest divider) |
| Mode | 0 to 3 (CPOL, CPHA) |
| DMA | Off by default: the CPU copies the data through the FIFO, buffers may be anywhere. With `dma` every frame is copied into and out of the host's DMA buffer under the bus lock, so any caller buffer works (stack, unaligned, odd lengths), the port allocates nothing per transfer, and the DMA's rounding of receive lengths up to whole words (ESP-IDF rounds every receive descriptor; ESP32 notes: "DMA may overwrite the data in the unaligned part") only ever writes port memory. Memory: twice `dma_buffer_bytes` per DMA host, rounded up to the alignment. On a chip whose internal RAM sits behind a cache (ESP32-P4, 64-byte lines) ESP-IDF still bounces a receive whose length is not a whole number of cache lines through a heap block of its own, per transfer (`setup_dma_priv_buffer`); full-duplex receive cannot be longer than the frame, so the port cannot pad it. Frames sized in whole lines avoid it; a transient lack of heap there is reported as `Busy` (detail `ESP_ERR_NO_MEM`). On the ESP32-C3 and ESP32-H2, which have little RAM, set `dma_buffer_bytes` to the largest frame of the drivers on the host |

## Timing limits

| Wait | Bound |
| --- | --- |
| The bus lock (every device of the host) | `SpiDeviceConfig::timeout` T (default 20 ms; 1 ms to about 660 ms: `EnsureReady()` and `SetClockHz()` refuse a T for which even a one-byte frame would exceed `kMaxTimeout`); `Busy` when not had |
| ESP-IDF's bus lock (`polling_start`) | Taken with `portMAX_DELAY`, which ESP-IDF v5.5 requires; free whenever the port asks (below) |
| The end of the transaction | W + T, then once more W + T. W is the frame's wire time at the clock ESP-IDF set (`ActualClockHz()`) plus the CS setup and hold cycles, rounded up to milliseconds |
| One `Transfer()` | At most T + 2 x (W + T), plus up to five RTOS ticks (included in the check); a frame for which this exceeds `kMaxTimeout` (2 s) is refused with `InvalidArgument` (with the default T of 20 ms: a 4092-byte frame below about 34 kHz) |
| Frame size | `kSpiMaxFrameBytes` (64) without DMA, on every target; with DMA the host's `dma_buffer_bytes` (`kSpiMinDmaBufferBytes` 4 to `kSpiMaxDmaFrameBytes` 4092, default 4092). `SpiBus::MaxFrameBytes()` returns the limit; longer frames are refused, never split |
| Clock | At least 1 kHz (ESP-IDF reports the clock it set in kHz; the wire time needs it) |

The end wait is computed from the frame, so the default T of 20 ms is a margin, not a frame limit: a full
4092-byte DMA frame at 1 MHz (32.7 ms on the wire) completes in time. `spi_device_polling_end` spins the CPU
while it waits (ESP-IDF's polling mode): keep T short for high-priority tasks, so that lower-priority tasks on the
same core are not starved.

Why `polling_start` cannot block: the port's bus lock serialises every device of the host, the port initialises
the host itself, and a host with an unfinished transaction is never used again (below), so ESP-IDF's lock is free
when the port asks for it. Requirement: no other component may add a device to a host the port owns (for example
an LCD or SD-card driver on the same `spi_host_device_t`).

## Results

| Result | When |
| --- | --- |
| `Ok` | The frame was clocked |
| `InvalidArgument` | Both spans empty, or of different non-zero lengths; a frame longer than `MaxFrameBytes()`; a frame whose worst case exceeds `kMaxTimeout`; a bad configuration |
| `Busy` | The bus lock was not had in time, or the CS pin or every device slot (8 for all hosts together; ESP-IDF also limits chip selects per host, `SOC_SPI_MAX_CS_NUM`, 3 or 6) is taken; nothing was clocked |
| `Timeout` | The transaction ended after W + T, within the second wait: the frame is complete, the deadline was missed (`Stats().late`) |
| `Fault` | The transaction did not end in the second wait either: the host is failed (`SpiBus::IsFailed()`); or no memory for the DMA buffer at `EnsureReady()` |
| `Busy` (detail `ESP_ERR_NO_MEM`) | ESP-IDF found no heap for its per-transfer bounce (cached chips, receive not in whole cache lines): transient, try again |
| `NotReady` | Before `EnsureReady()`, or ESP-IDF refused a removal (`ESP_ERR_INVALID_STATE`) |

## When a transaction never ends

ESP-IDF's `polling_end` returns `ESP_ERR_TIMEOUT` with the transaction still in flight: the SDK's bus lock stays
taken and the device cannot be removed. The master drives SCLK, so the hardware finishes, and the port waits a
second time. If it still has not ended, the peripheral itself is at fault: the port marks the host failed, and
every later call on any device of that host returns `Fault` at once, without calling ESP-IDF again (only
`Release()` still asks ESP-IDF, which refuses the stuck device), so no task can
hang on the SDK's lock. Restart the application to use the host again.

`tx` and `rx` must not overlap (the port clears `rx` before it reads `tx`).

A `Timeout` frame was fully clocked: a driver that retries on `Timeout` repeats it, which matters for
non-idempotent frames (an EEPROM write, a FIFO read, a motor command).

With DMA, the transaction of a failed host still points at the host's DMA buffer, which the port keeps (it is
freed only after `spi_bus_free` succeeds, which ESP-IDF refuses while the device is attached), so a late
completion can only write port memory, never a caller's buffer that has since gone out of scope.

## Frame limits and drivers

Drivers can check the limit at build time: `kb::port::esp32::kSpiMaxFrameBytes` (64, without DMA) and
`kSpiMaxDmaFrameBytes` (4092, the largest DMA buffer); `SpiBus::MaxFrameBytes()` gives the configured one.

| Driver | Longest frame with chip select held | On this port |
| --- | --- | --- |
| tmc5160, tmc9660 | 5, 8 | Works without DMA |
| ads9324 | 50 | Works without DMA |
| at25040b | 18 (chunked by the driver) | Works without DMA |
| as5047u, ads7952, tle92466ed, max22200 | 2 to 4 | Works without DMA |
| isf15acp4 | 96 pixel bytes per data transfer | Needs `dma = true`, or the driver's data chunk at 60 bytes or less |
| bmp581 (SPI) | 97 | Needs `dma = true` (the DMA buffer takes its member buffers as they are) |
| bno08x (SPI) | up to 1028 | Needs `dma = true`, `dma_buffer_bytes` at least the driver's read buffer |

## Example

```cpp
esp::SpiBus bus{bus_cfg};
esp::SpiDevice tmc{bus, dev_cfg}; // mode 3, 4 MHz
std::array<uint8_t, 5> tx{0x6FU, 0, 0, 0, 0}, rx{};
const kb::Status s = tmc.Transfer(tx, rx);
```
