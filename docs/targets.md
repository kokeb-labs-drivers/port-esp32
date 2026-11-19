# Targets

One port serves every ESP32 target (kb-core ADR-0008 rule 3). Chip differences are read from ESP-IDF's
capability macros (`soc/soc_caps.h`) in the boundary files only; the public API is the same on every target.

## What each target has

| Target | Core | HP I2C | SPI hosts | HP UART | LEDC channels / timer bits | ADC bits / calibration | TWAI | GPIO |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `esp32` | Xtensa LX6 x2 | 2 | SPI2, SPI3 | 3 | 8 / 20 | 12 / line fitting | 1 | 0-39 (34-39 input only) |
| `esp32s2` | Xtensa LX7 | 2 | SPI2, SPI3 | 2 | 8 / 14 | 13 / line fitting | 1 | 0-46 |
| `esp32s3` | Xtensa LX7 x2 | 2 | SPI2, SPI3 | 3 | 8 / 14 | 12 / curve fitting | 1 | 0-48 |
| `esp32c3` | RISC-V | 1 | SPI2 | 2 | 6 / 14 | 12 / curve fitting | 1 | 0-21 |
| `esp32c6` | RISC-V | 1 | SPI2 | 2 | 6 / 20 | 12 / curve fitting | 2 | 0-30 |
| `esp32h2` | RISC-V | 2 | SPI2 | 2 | 6 / 20 | 12 / curve fitting | 1 | 0-27 |
| `esp32p4` | RISC-V x2 | 2 | SPI2, SPI3 | 5 | 8 / 20 | 12 / curve fitting | 3 | 0-54 |

The values are those of ESP-IDF v5.5's `soc_caps.h` (`SOC_HP_I2C_NUM`, `SOC_SPI_PERIPH_NUM`,
`SOC_UART_HP_NUM`, `SOC_LEDC_CHANNEL_NUM`, `SOC_LEDC_TIMER_BIT_WIDTH`, `SOC_ADC_RTC_MAX_BITWIDTH`,
`SOC_TWAI_CONTROLLER_NUM`, `SOC_GPIO_PIN_COUNT`); the port reads them from the SDK, never from this table.

## Gating

| Situation | Behaviour |
| --- | --- |
| A chip without TWAI (`SOC_TWAI_SUPPORTED` unset, for example a future target) | `CMakeLists.txt` builds `src/boundary/no_can.cpp` instead of `src/boundary/can.cpp`; `CanController::EnsureReady()` returns `Unsupported` with `ESP_ERR_NOT_SUPPORTED` and touches nothing |
| A controller number the chip does not have (I2C port 1 on ESP32-C3) | `EnsureReady()` returns `InvalidArgument` before any SDK call |
| No ADC calibration scheme, or eFuse without calibration data | `ReadMillivolts()` returns `Unsupported`; `ReadRaw()` works |
| ESP32-C5, ESP32-C61, ESP32-H21 and other targets of ESP-IDF v5.5 | Not built in CI; expected to work where the drivers exist, not verified |

## Tick rate and SDK options

The port works at any FreeRTOS tick rate from 1 Hz to 1000 Hz: every millisecond value is converted to ticks
rounded up (plus one where ESP-IDF converts again), and a `static_assert` in each boundary file checks the range
and that `kMaxTimeout` never reaches `portMAX_DELAY`. The examples use `CONFIG_FREERTOS_HZ=1000` so that
millisecond waits are whole ticks; at ESP-IDF's default 100 Hz waits are rounded up to 10 ms steps.

The example builds use ESP-IDF's defaults for interrupt placement: none of the `*_ISR_IN_IRAM` options is set, so
the I2C, SPI, UART, GPIO and TWAI interrupts are held off while the flash cache is disabled (NVS or OTA writes),
and a UART at a high baud rate can overrun its FIFO meanwhile. An earlier implementation was exercised on ESP32-C6
with the I2C master, SPI master, GPTimer, RMT and GDMA interrupt handlers placed in IRAM
(`CONFIG_I2C_MASTER_ISR_HANDLER_IN_IRAM`, `CONFIG_SPI_MASTER_ISR_IN_IRAM`, `CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM`,
`CONFIG_RMT_ISR_HANDLER_IN_IRAM`, `CONFIG_GDMA_CTRL_FUNC_IN_IRAM`, `CONFIG_ESP_SPI_BUS_LOCK_ISR_FUNCS_IN_IRAM`), a
5 s task watchdog and a 300 ms interrupt watchdog; this port has not been run on hardware with or without them.
The GPIO callbacks of the port are plain functions in flash: see [GPIO](peripherals/gpio.md) for an ISR service
installed with `ESP_INTR_FLAG_IRAM`.

## Memory

The per-task outcome records (`CallerRecord`) live in thread-local storage: ESP-IDF places the static TLS block
on every task's stack when the task is created, so every task, the idle, timer and system tasks included, gives
about 80 bytes of its stack to it, whether or not it calls the port. Size task stacks with that in mind. A DMA SPI
host takes twice its `dma_buffer_bytes` of internal DMA-capable RAM once.

## Built in CI

| Build | Targets |
| --- | --- |
| `port_selftest` example, Release (ci-cmake-matrix.yml) | every target above |
| Host tests over the fakes (both chip variants: with TWAI and curve fitting; without TWAI and line fitting) | host |
