# port-esp32

kb-core's bus, pin, clock and log contracts (`kb::iface`) implemented over **ESP-IDF v5.5** for the **ESP32
family**: `esp32`, `esp32s2`, `esp32s3`, `esp32c3`, `esp32c6`, `esp32h2` and `esp32p4`. Drivers written against the
kb contracts run on an ESP32 with no ESP-IDF code of their own.

| | |
| --- | --- |
| MCU family | Espressif ESP32, Xtensa and RISC-V |
| SDK | ESP-IDF v5.5: `i2c_master`, `spi_master`, `uart`, `gpio`, `ledc`, `adc_oneshot` + `adc_cali`, `twai`, `esp_timer`, ROM delay, `esp_log` |
| Namespace / CMake target / component | `kb::port::esp32` / `kb::port_esp32` / `kb_port_esp32` |
| Include root | `kb/port/esp32/` (`#include "kb/port/esp32/port.hpp"`) |
| C++ | ISO C++20; no heap after start-up (a DMA SPI host allocates its DMA buffer once in `EnsureReady()`), no exceptions, no RTTI, no virtual functions |
| Assurance | Level high: MISRA C++:2023 (CodeQL); records for the SDK boundary files and one for the port-wide tables (`docs/deviations.md`) |

## Contracts

| kb contract | Type | ESP-IDF |
| --- | --- | --- |
| `I2cBus` | `I2cController` | `i2c_master` (synchronous), stuck-bus recovery (nine clocks and a STOP), reset at start-up |
| `SpiDevice`, `SpiClockControl` | `SpiBus` + `SpiDevice` | `spi_master`, one polling transaction per frame (64 bytes, or up to 4092 through the port's DMA buffer), waits sized to the wire time |
| `UartPort`, `UartBaudControl`, `UartTxDrain` | `Uart` | `uart`, RS-485 half duplex optional |
| `GpioOutput`; `GpioInput`, `GpioInterrupt` | `OutputPin`; `InputPin` | `gpio`, per-pin ISR service |
| `AdcChannel` | `AdcChannel` | `adc_oneshot`, eFuse calibration |
| `PwmOutput` | `PwmChannel` | `ledc`, low-speed mode |
| `CanBus` | `CanController` | `twai` (`_v2` handle API), bus-off recovery |
| `MonotonicClock`, `BusyWait`, `LogSink` | `Clock`, `BusyWait`, `LogSink` | `esp_timer_get_time`, `esp_rom_delay_us`, `esp_log_write` |

Every type is `static_assert`ed against its concepts and its `*Ref`. Every wait is bounded by a configured timeout,
each capped at `kMaxTimeout` (2 s; kb-core's "no limit" included), and each peripheral page gives a call's worst
case; every failure is a
`kb::Status` with the `esp_err_t` kept as the vendor code (`LastError().Detail()`, per calling task).

## Use

```cpp
#include "kb/port/esp32/port.hpp"
namespace esp = kb::port::esp32;

esp::I2cConfig cfg{};
cfg.sda_pin = 8;
cfg.scl_pin = 9;
static esp::I2cController bus{cfg};
if (bus.EnsureReady() == kb::Status::Ok) {
  const std::array<uint8_t, 1> reg{0x00U};
  std::array<uint8_t, 2> rx{};
  const kb::Status s = bus.WriteRead(0x4AU, reg, rx); // NoDevice, Nack, Timeout, BusError, Busy ...
}
```

Add the components `kb` and `kb_port_esp32` to an ESP-IDF project as `examples/esp32/components/` shows
([Getting started](docs/getting-started.md)).

## Examples

`examples/esp32` builds for every target: a self test of every contract on a bare board
([Examples](docs/examples.md)).

## Build and test

```bash
cmake -S . -B build -G Ninja -DKB_PORT_ESP32_BUILD_TESTS=ON
cmake --build build && ctest --test-dir build             # host tests over ESP-IDF fakes
tools/preflight.sh                                        # what CI checks, locally
```

## Source layout

| Header (`inc/kb/port/esp32/`) | Logic (`src/`) | SDK boundary (`src/boundary/`, the only files that include ESP-IDF headers) |
| --- | --- | --- |
| `adc.hpp` | `adc.cpp` | `adc.cpp` |
| `can.hpp` | `can.cpp` | `can.cpp` (`no_can.cpp` on chips without TWAI) |
| `gpio.hpp` | `gpio.cpp` | `gpio.cpp` |
| `i2c.hpp` | `i2c.cpp` | `i2c.cpp` |
| `log.hpp` | `log.cpp` | `log.cpp` |
| `pwm.hpp` | `pwm.cpp` | `pwm.cpp` |
| `spi.hpp` | `spi.cpp` | `spi.cpp` |
| `time.hpp` | `time.cpp` | `time.cpp` |
| `uart.hpp` | `uart.cpp` | `uart.cpp` |
| `error.hpp`, `version.hpp`, `port.hpp` | `port.cpp` | `error.cpp` |

`tools/check_layout.py` (run by `tools/preflight.sh` and `ctest`) fails when a header has no source of its name or a
file matches no header, so a combined file cannot come back.

## Documentation

The site (`docs/`, `mkdocs build -f _config/mkdocs.yml --strict`) has one page per peripheral with the SDK calls,
pins and clocks, timing limits and the error mapping, plus the [error mapping](docs/error-mapping.md),
[thread safety](docs/thread-safety.md), [testing](docs/testing.md) and [MISRA boundary](docs/deviations.md).

## Licence

MIT, copyright The port-esp32 authors ([LICENSE](LICENSE)). Contact: <kokeblabs@gmail.com>.
