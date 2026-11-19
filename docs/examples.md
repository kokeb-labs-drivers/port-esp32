# Examples

`examples/esp32` is one ESP-IDF project for every ESP32 target. The app is one source file in `main/`, chosen by
`app_config.yml`; the pins come from menuconfig (`kb port examples: pins`, with a default per target in
`main/Kconfig.projbuild`).

| App | What it shows | Hardware |
| --- | --- | --- |
| `port_selftest` | Every contract once: clock, busy wait, log, GPIO toggle, PWM, ADC, I2C and SPI bus bring-up, CAN in no-ack mode, UART | None (a wire TX to RX makes the UART a loopback) |

The example is the port alone: no chip driver is pulled in. Chip drivers are bound to the port through the kb
contracts (see [Getting started](getting-started.md)); their own repositories carry the examples.

## Build

```bash
cd examples/esp32
idf.py set-target esp32s3
idf.py -DAPP_TYPE=port_selftest -DAPP_SOURCE_FILE=port_selftest.cpp -DBUILD_TYPE=Release build
idf.py -p /dev/ttyUSB0 flash monitor
```

or with the ESP-IDF build scripts (kokeb-labs espidf-tools, cloned into `examples/esp32/scripts`):
`./scripts/build_app.sh port_selftest Release`.

## Default pins

| Signal | ESP32 | S2 | S3 | C3 | C6 | H2 | P4 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| I2C SDA, SCL | 21, 22 | 8, 9 | 8, 9 | 4, 5 | 6, 7 | 1, 2 | 7, 8 |
| SPI MOSI, MISO, SCLK, CS | 23, 19, 18, 5 | 11, 13, 12, 10 | 11, 13, 12, 10 | 7, 2, 6, 10 | 19, 20, 21, 18 | 25, 11, 10, 12 | 30, 31, 32, 33 |
| ADC input | 34 | 1 | 1 | 0 | 0 | 1 | 16 |
| LED (GPIO, PWM) | 2 | 15 | 2 | 8 | 15 | 0 | 36 |
| TWAI TX, RX | 26, 27 | 6, 7 | 6, 7 | 18, 19 | 2, 3 | 22, 23 | 37, 38 |
| UART1 TX, RX | 17, 16 | 17, 18 | 17, 18 | off | off | off | off |

Change them in menuconfig to match a board; any GPIO the chip routes works. Some defaults are strapping pins
(GPIO 8 and 2 on ESP32-C3, GPIO 15 on ESP32-C6, GPIO 2 on ESP32): fine for a demo on a development board, where
the level at reset decides the boot mode; move them on a real board.

## What was run

The app is built in CI for every target. It has not been run
on hardware as part of this repository's CI; see the changelog for what has been checked on a bench.
