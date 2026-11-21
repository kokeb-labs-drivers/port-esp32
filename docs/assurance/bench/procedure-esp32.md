# Bench procedure: ESP32-C6, ESP32-S3, ESP32

The step-by-step run of the DMA bench items on three devkits. The ESP32-C6 and ESP32-S3 run every item; the ESP32
(classic) runs the SPI items, for T8 (its DMA rounds a receive up to whole words). Each item says what it shows and
what it needs.

## What to have

| Item | Needed for |
| --- | --- |
| ESP32-C6 devkit (any with GPIO 0, 1, 4, 18 to 21 on headers) | every item |
| ESP32-S3 devkit (GPIO 1 to 3, 10 to 13 on headers); a module with PSRAM for SPI-PSRAM | every item |
| ESP32 devkit (GPIO 5, 18, 19, 23 on headers) | the SPI items, T8 |
| One jumper wire, under 10 cm | MOSI to MISO loopback (all SPI items) |
| Four 10 kOhm resistors, a breadboard, three wires | the ADC divider (ADC items) |
| USB cable; ESP-IDF v5.5 (`idf.py monitor`, 115200 Bd) | the console and the log |
| Multimeter | the wiring check (divider tap voltages) |
| Logic analyser, two channels (optional) | SCLK and CS timing, the late-completion item |

## Wiring

### ESP32-C6

![ESP32-C6 bench wiring: SPI loopback jumper, divider, console](figures/wiring-esp32c6.svg#only-light)
![ESP32-C6 bench wiring: SPI loopback jumper, divider, console](figures/wiring-esp32c6.dark.svg#only-dark)

### ESP32-S3

![ESP32-S3 bench wiring: SPI loopback jumper, divider, console](figures/wiring-esp32s3.svg#only-light)
![ESP32-S3 bench wiring: SPI loopback jumper, divider, console](figures/wiring-esp32s3.dark.svg#only-dark)

### ESP32

![ESP32 bench wiring: SPI loopback jumper, console](figures/wiring-esp32.svg#only-light)
![ESP32 bench wiring: SPI loopback jumper, console](figures/wiring-esp32.dark.svg#only-dark)

The pins are those of `tests/target/dma/main/bench.hpp`. On the ESP32 the bench uses VSPI so that no strapping pin
(GPIO12) is on MISO. The divider: four 10 kOhm resistors in series from 3V3 to GND; the three inner nodes (about
0.8 V, 1.65 V and 2.5 V) go to the three ADC inputs in that order.

## Steps

1. Wire the board as its figure shows. Measure the three divider taps and note them on the evidence form.
2. Copy [the evidence form](evidence-form.md) to `runs/<yyyy-mm-dd>-<board>.md`; fill the run fields.
3. Build and flash the SPI app, logging the console:

    ```bash
    cd tests/target/dma
    idf.py set-target esp32c6        # esp32s3, esp32
    idf.py -DAPP_TYPE=dma_spi_bench -DAPP_SOURCE_FILE=dma_spi_bench.cpp build flash monitor | tee spi.log
    ```

4. Wait for `KB-BENCH SUMMARY`; copy each `KB-BENCH` line into the item table with the expected value below.
5. On the ESP32-C6 and ESP32-S3, the same for `dma_adc_bench` (`APP_TYPE=dma_adc_bench`,
   `APP_SOURCE_FILE=dma_adc_bench.cpp`).
6. Commit the form and the logs. A `FAIL` is recorded as it is, with a remark; it is never re-run until it passes
   without a note.

## Items

| Item | Shows (design reference) | Boards | Hardware | Expected |
| --- | --- | --- | --- | --- |
| SPI-START | the host comes up with DMA and the static staging buffer | C6, S3, ESP32 | devkit | `status=Ok mode=dma staging=2048` |
| SPI-5B | a 5-byte receive through the staging buffer: data back, the canaries after the caller's bytes intact | C6, S3, ESP32 | jumper | `mismatches=0 canary=intact` |
| T8 | the ESP32's receive rounding to whole words lands in port memory (conformance T8 on silicon) | ESP32 | jumper | `canary=intact` |
| SPI-1028 | a 1028-byte frame (the largest bno08x read) | C6, S3, ESP32 | jumper | `mismatches=0` |
| SPI-UNALIGNED | a caller buffer at an odd address with an odd length | C6, S3, ESP32 | jumper | `mismatches=0` |
| SPI-PSRAM | a caller buffer in PSRAM goes through the staging buffer | S3 with PSRAM | PSRAM module, jumper | `status=Ok same=1` (else SKIP) |
| SPI-ZEROCOPY | a typed buffer lent for transmit and receive, reported once, back to the CPU | C6, S3, ESP32 | jumper | `tx=1 rx=1 reported_once=1` |
| SPI-HEAP | no allocation after start: 1000 frames, heap blocks and bytes unchanged | C6, S3, ESP32 | jumper | `failures=0`, before = after |
| SPI-LATE | a transfer that ends after its deadline: reported `Timeout`, the buffer back only after ESP-IDF reported the end, the host not failed | C6, S3, ESP32 | jumper; analyser optional (CS stays low about 82 ms) | `status=Timeout buffer=cpu host_failed=0` |
| SPI-GDMA | every GDMA pair taken: a strict DMA host is refused, a host with the fallback runs without DMA and says so | C6, S3 | devkit | `strict=Unsupported fallback=Ok mode=polled ceiling=64` (ESP32: SKIP, no GDMA) |
| ADC-START | ADC continuous with the IRAM-safe ISR path, three channels, 1 kHz each | C6, S3 | divider | `status=Ok mode=dma rate_hz=3000` |
| ADC-RATE | frames per second and the jitter of the interrupt stamps | C6, S3 | divider | within 5 % of 62.5 frames/s, `gaps=0` |
| ADC-DATA | results in pattern order and in range | C6, S3 | divider | `bad_order=0 invalid=0` |
| ADC-ISR | the cost of the port's interrupt path in CPU cycles (not visible in RTOS statistics) | C6, S3 | divider | `max_isr_cycles` above 0 and below 50000 |
| ADC-HEAP | no allocation after start while streaming | C6, S3 | divider | before = after |
| ADC-BLOCKED | the consumer held for 1 s: ring overruns and SDK pool overflows counted separately, the next frame flagged | C6, S3 | divider | `ring_overruns`, `pool_overflows` and `gaps_after` all above 0 |
| ADC-NVS | NVS writes for 2 s while streaming with the IRAM-safe option: no crash, frames keep coming | C6, S3 | divider | `err=0x0`, frame rate above 80 % of nominal |
| ADC-STALL | no false stall in 2 s of supervision with real interrupt stamps; the restart path brings frames back | C6, S3 | divider | `false_stalls=0`, frames after the restart |
| ADC-SHAPE | a board shape that differs from the frame: DMA refused, the fallback reported | C6, S3 | devkit | `ensure_ready=NotReady mode=polled` |

## What the bench does not show

- Stall detection when the conversion trigger stops while the stream still runs: the port has no way to stop the
  trigger without stopping the stream, so this is shown on host only (conformance T12 and the unit test).
- The recovery of a host whose transaction never ends: ESP-IDF v5.5 cannot stop it, so it needs a restart; the
  quarantine is shown on host (T6, T16).
- ESP32-P4 cache sync: no P4 board is planned; the `Maintained` region is covered by the conformance suite.
