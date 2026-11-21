# Evidence form

Copy this page to `runs/<yyyy-mm-dd>-<board>.md` for each run, fill every field, keep the monitor log next to it
(`runs/<yyyy-mm-dd>-<board>.log`). One form per board and app run. A field that does not apply says why; none is left
empty.

## Run

| Field | Value |
| --- | --- |
| Date | |
| Board and revision | (for example "ESP32-C6 devkit, rev 1.2") |
| Silicon revision | (from `KB-BENCH-RUN ... revision=`) |
| ESP-IDF version and commit | (from `KB-BENCH-RUN ... idf=`, and `git -C $IDF_PATH rev-parse HEAD`) |
| Toolchain | (`riscv32-esp-elf-gcc --version` or `xtensa-esp32s3-elf-gcc --version`) |
| Clock tree | (CPU from `cpu_hz=`; APB and the SPI clock source as configured) |
| RTOS and priorities | (FreeRTOS tick from `tick_hz=`; the bench task priority; SPI interrupt level) |
| Image commit | (`git rev-parse HEAD` of port-esp32 the image was built from) |
| App | `dma_spi_bench` or `dma_adc_bench` |
| Wiring check | (the figure followed, the jumper and divider measured: tap voltages in V) |
| Allocations after start | (SPI-HEAP and ADC-HEAP: blocks and bytes before and after, and the allocating site of any difference) |

## Items

One row per `KB-BENCH` line, in the order printed.

| Item | Expected | Measured (the line's key=value fields) | PASS / FAIL / SKIP | Log file and line |
| --- | --- | --- | --- | --- |
| | | | | |

## Remarks

Anything seen that the items do not capture (resets, watchdog messages, a logic-analyser trace), with the file name of
any capture.
