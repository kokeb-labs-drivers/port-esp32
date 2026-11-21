# MISRA boundary and deviations

The port is scanned with the CodeQL MISRA C++:2023 pack and the KB-CPP20 profile pack at level high
(`ci-misra.yml`, kb-core ADR-0011), on a traced host build and a traced Cortex-M build of `inc/` and `src/`.

## The boundary

ESP-IDF headers are included only by the files of `src/boundary/` (`deviations/boundary-files.yml`): one file per
SDK component, named after the public header it serves, next to the logic file `src/<name>.cpp` of the same name
(`tools/check_layout.py` fails on a header without its source, on a combined file and on a file that matches no
header):

| File | SDK areas |
| --- | --- |
| `src/boundary/i2c.cpp` | `i2c_master`, the I2C clock limits (`hal/i2c_ll.h`, `esp_clk_tree.h`) |
| `src/boundary/spi.cpp` | `spi_master` (polling and queued transactions), the staging buffer allocation (`esp_heap_caps.h`, `esp_cache_private.h`) |
| `src/boundary/uart.cpp` | `uart` and its event queue |
| `src/boundary/gpio.cpp` | `gpio` |
| `src/boundary/adc.cpp` | `adc_oneshot`, `adc_cali`, `adc_continuous` (two IRAM callbacks) |
| `src/boundary/dma.cpp` | `esp_cache_msync`, `esp_ptr_dma_capable`, `esp_dma_is_buffer_alignment_satisfied` |
| `src/boundary/pwm.cpp` | `ledc` |
| `src/boundary/can.cpp` | `twai` (chips with TWAI) |
| `src/boundary/no_can.cpp` | `esp_err.h` only (chips without TWAI; stands in for `can.cpp`) |
| `src/boundary/time.cpp` | `esp_timer`, `esp_rom_delay_us`, `vTaskDelay` |
| `src/boundary/log.cpp` | `esp_log` |
| `src/boundary/error.cpp` | `esp_err.h`: the value check behind `error.hpp` |

Each boundary function converts its arguments, makes one SDK call (or a fixed short sequence) and returns the
`esp_err_t`; the boundary tables hold SDK handles only. Every decision (what a code means, retries, recovery,
locking, which slot or pin an object gets, the users of a shared unit or timer) is in the platform-free files,
which include no SDK header. Their one record is DEV-003: the port-wide claim tables and per-task records. The layering check (`ci-layering.yml`) enforces the boundary, and
its poison build compiles the platform-free code with every SDK header replaced by one that stops the build.

## Records

| Id | Guideline | Where | Why | How the risk is held |
| --- | --- | --- | --- | --- |
| DEV-001 | Rule 6.7.1 (local variables with static storage) | the ADC, CAN, I2C, SPI and UART boundary files | ESP-IDF hands out driver handles that outlive each call, and no SDK type may reach a public header: the handles live in one function-local static table per peripheral (also the SPI transaction descriptors, the SPI DMA buffer pointers, the UART queue lengths, the SPI zero buffer, and the ADC continuous handle with the port stream its IRAM callbacks feed) | One accessor per table in an anonymous namespace; handles only; the index (I2C port, UART port, SPI host and slot, TWAI controller, ADC slot) is claimed by the logic before the SDK is called (DEV-003), so a second object is refused before it reaches a table; entries written by the owner's `EnsureReady()` after its claim succeeded (only the claimant reaches them) and cleared by `Release()` under the owner's lock; every accessor exercised by the host tests |
| DEV-002 | Rule 8.2.11 (arguments through an ellipsis) | the log boundary file, one call | ESP-IDF's `esp_log_write` is variadic; the clang-tidy check behind the rule reports every variadic call | A literal format; each argument cast to the exact type its conversion names; the length bounded by `kLogMaxLine`; no user text as a format |
| DEV-003 | Rule 6.7.1 (local variables with static or thread storage) | `claims.cpp`, `caller_record.cpp`, `spi.cpp`, `adc.cpp`, `pwm.cpp`, `gpio.cpp` | Facts of the chip shared by every object: taken pins, controller numbers, SPI and ADC slots, LEDC timer records, ADC unit users and locks, the ISR-service flag; and per task (thread storage) the outcome of its last call on each shared object | One accessor each; atomic claims (compare-and-swap); timer records under a busy flag with bounded tries; ADC users under the unit's mutex; per-task records touched only by their task; host tests for each |
| DEV-004 | Rule 6.9.2 (standard integer type names), advisory | the ADC and SPI boundary files, three variables | `adc_oneshot_read`, `adc_cali_raw_to_voltage` and `spi_device_get_actual_freq` return through `int*`, and `int32_t` is `long` on the ESP32 toolchains | One `int` local per call, converted at once to a fixed-width type; every other SDK `int` is held in `int32_t` and cast at the call |
| DEV-006 | Rule 8.2.3 (casting away const) | the DMA boundary file, one call | `kb::dma::CacheOps::Clean` takes the range as `const` (a clean only writes cache lines back to memory), and ESP-IDF's `esp_cache_msync` takes `void*` for both directions | One conversion, in `CacheWriteBack`, of whole lines of one `kb::dma::Buffer`, passed only with the C2M flag, which writes nothing through the pointer; host-tested over the fakes |
| DEV-005 | Rule 8.2.6 (`void*` to an object pointer) | the SPI boundary file, one conversion | The SPI DMA buffer comes from `heap_caps_aligned_alloc` (`void*`) | One conversion of a block the call allocated, covered exactly by the spans; allocated once per host, freed only after `spi_bus_free` |

The records are in `deviations/deviations.yml`. A marker `// codeql::misra_deviation_next_line(DEV-001)` stands
on the line before each finding it covers. A clang-tidy suppression of a check the MISRA matrix relies on names its
record inside the comment, never as its first or last word (a bare id there would be read as a CodeQL marker):
`// NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): DEV-002, the SDK's log call is variadic`. A marker that
covers nothing, an unknown id or a lapsed record (`review_by` passed) fails the scan.

## Findings avoided by design

| Guideline | What the SDK would have needed | What the port does instead |
| --- | --- | --- |
| Rule 8.2.5 (`reinterpret_cast`) | `uart_tx_chars` takes `const char*`; the DMA alignment check and the IRAM check need addresses | Converts at most one FIFO of bytes with `std::bit_cast` into a `char` buffer; addresses through `std::bit_cast` |
| Rule 24.5.2 (`memcpy`) | Copying bytes into that buffer with `memcpy` | An element loop |
| Rule 10.2.3 (numeric value of an unscoped enum) | `adc_unit_t`, `adc_channel_t` as numbers | Finds the number by comparison with each enumerator |
| Rule 8.2.6 (`void*` to object pointer) | Handles as `void*` in public headers | Handles stay in the boundary files' tables |

## Advisory findings

The six advisory findings of the first CI scan are resolved: the five Rule 6.9.2 findings are fixed (`int32_t`
for SDK returns and constants) or recorded (DEV-004, the three `int*` out-parameters), and cppcheck's Rule 0.0.2
note on the pin-claim test is gone with the claim table, which moved to the platform-free logic as lock-free bit
claims (DEV-003).
