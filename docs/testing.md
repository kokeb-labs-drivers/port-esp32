# Testing and CI

## Host tests over SDK fakes

The port's logic (`src/*.cpp`, one per public header) and its boundary files (`src/boundary/*.cpp`, one per SDK
component) are built unchanged on the host
against `tests/fakes/esp-idf`: one-line headers with ESP-IDF's names (`driver/i2c_master.h`, `esp_adc/adc_oneshot.h`,
...) over one declaration file, and `fake_esp_idf.cpp`, which defines every SDK function the boundary files call.
That is a link seam per SDK call: the tests inject any `esp_err_t` into any function (`fake::Fail`), drive a small
model of each peripheral (pin levels, I2C devices that ACK, NACK or stretch, a UART receive queue that drops events
when full, TWAI state and recovery, LEDC clock limits per timer) and re-enter the port from inside an SDK call
(`fake::Hook`) to prove each lock.

The fakes model ESP-IDF v5.5's behaviour as read in its sources, not its headers, so a test fails where the port
would fail on silicon: a failed I2C transfer returns `ESP_ERR_INVALID_STATE` whether it was a NACK or a stretch
(`ESP_ERR_INVALID_RESPONSE` in the v6 variant); `spi_device_polling_start` refuses any wait but `portMAX_DELAY`,
takes the SDK bus lock and counts a call that would wait for it for ever; `polling_end` keeps a slow transaction
in flight after a timeout and `spi_bus_remove_device` refuses its device; a DMA receive writes whole words, past
an odd length, as ESP-IDF's descriptors do; a second `uart_driver_install` on a port fails; on a cached chip a DMA receive not in whole cache
lines is counted as ESP-IDF's per-transfer bounce; `ledc_set_duty` takes
any duty.

| Test | Proves |
| --- | --- |
| `kb_port_esp32_tests` | Every member of every type, every refusal and every SDK error path, over the fakes of a chip with TWAI and curve-fitting calibration; the `*Ref` of each contract at run time |
| `kb_port_esp32_minimal_tests` | The same boundary files built for a chip without TWAI, with line-fitting calibration, ESP-IDF v6's I2C code and a 100 Hz tick; and the GPIO ISR service installed by the application |
| `kb_port_esp32_compile_fail_*` | Misuse is a compile error: a `*Ref` of the wrong contract, a copied controller, an output pin as an input |
| `kb_port_esp32_header_checks` | Each public header compiles on its own as ISO C++20 with the strict flags |
| `static_assert`s in `port.hpp` and beside each type | Each type satisfies its concepts and converts to its `*Ref` |

```bash
cmake -S . -B build -G Ninja -DKB_PORT_ESP32_BUILD_TESTS=ON && cmake --build build && ctest --test-dir build
```

## Local pre-flight

`tools/preflight.sh` runs what CI runs, before a push: the kb-core copy check, clang-format 20, the tests with Clang
(strict flags) and GCC at -O2, ASan and UBSan, coverage at the high level's limits (lines 85 %, branches 75 %),
lizard (CCN 15, 60 lines, 6 parameters), clang-tidy 20, `misra_lint --strict`, the handbook check, the layering
check and the platform-free build. The MISRA scan runs in the ci-tools MISRA image; the ESP-IDF builds need
ESP-IDF v5.5 (`idf.py`).

## CI

Every check is a caller of a `kokeb-labs/ci-tools` reusable workflow at a commit SHA (the `v1` tag when it was
pinned), joined by one gate
(`CI gate / gate`).

| Caller | Reusable workflow | What |
| --- | --- | --- |
| `ci-host.yml` | `ru-host-ci` | GCC and Clang builds, the tests, cppcheck over `inc/` and `src/` (its clang-tidy 18 cannot read the org configuration; clang-tidy runs in `ci-cpp-lint.yml`) |
| `ci-quality.yml` | `ru-host-quality` | Coverage, sanitizers and complexity at level high |
| `ci-misra.yml` | `ru-misra` | MISRA C++:2023 and the KB-CPP20 profile, host and Cortex-M traced builds, the boundary files' records |
| `ci-cpp-lint.yml` | `ru-cpp-lint` | clang-format 20 and clang-tidy 20 |
| `ci-cross-build.yml` | `ru-cross-build` | The platform-free logic for Cortex-M0+, M4F and M7F |
| `ci-cmake-matrix.yml` | `ru-cmake-matrix` | In the ESP-IDF v5.5 container: `port_selftest` for every ESP32 target |
| `ci-layering.yml` | `ru-layering` | SDK headers only in boundary files, the poison build of the platform-free code |
| `ci-docs.yml` | `ru-mkdocs-publish` | The strict site build and its checks |
| `ci-handbook.yml` | `ru-handbook-check` | The handbook's rules for a library at level high |
| `ci-dco.yml` | `dco` | Every commit of a pull request is signed off |

Not covered by a host test, reviewed only: a failed `kb::rtos::Mutex::Create()` (the host's `KB_RTOS=NONE`
backend always creates); and the controller claims under truly concurrent bring-up (the boundary tables refuse
an occupied entry as a second guard, so a single-threaded fake cannot tell the two apart). A CAN `Release()`
whose wait ends because a running call returned is tested with two threads that take turns through fake hooks.

## Not verified here

The port has not been run on hardware by this repository's CI: the host tests prove the logic and the SDK call
sequence against fakes, and the ESP-IDF builds prove it compiles and links for each target. Timing on silicon
(clock stretching limits, bus recovery on a real stuck slave, TWAI bus-off recovery) needs a bench run per target.

## DMA conformance

`kb_port_esp32_conformance` runs kb-core's DMA conformance suite (cases T1 to T18) against the port's real classes,
with their boundary calls linked against kb-core's host DMA-and-cache model (`tests/dma_model/`), and prints one
`CONFORMANCE <case> <variant> PASS|FAIL` line per case and variant. The results and how each case applies to the ESP32
are in [DMA](dma.md#conformance); the bench apps for the same paths on real boards are in `tests/target/dma/`
([bench kit](assurance/bench/README.md)), built in CI for esp32c6, esp32s3 and esp32.
