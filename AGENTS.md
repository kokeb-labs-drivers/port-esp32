# AGENTS.md

port-esp32: kb-core's contracts (`kb::iface`) over ESP-IDF v5.5 for every ESP32 target, namespace
`kb::port::esp32`, component `kb_port_esp32`. It follows the
[Kokeb Labs engineering standard](https://github.com/kokeb-labs/.github/blob/main/standards/README.md) and the
handbook at level `high` (`.kokeb/handbook.yml`).

## Layout

- `inc/kb/port/esp32/`: public headers, one per peripheral, no SDK type in any signature; `port.hpp` includes them
  all and proves every `*Ref`.
- `src/<stem>.cpp`: the platform-free logic, one file per public header (`adc`, `can`, `gpio`, `i2c`, `log`, `port`,
  `pwm`, `spi`, `time`, `uart`); `error.hpp` and `version.hpp` are header-only (`tools/layout.yml`). Shared internals:
  `sdk.hpp` (the seam to the boundary), `claims.*` (pin, controller and slot claims) and `caller_record.cpp`
  (per-task outcomes), used by several of these files.
- `src/boundary/<component>.cpp`: the SDK boundary, one file per SDK component (`i2c`, `spi`, `uart`, `gpio`, `adc`,
  `pwm`, `can` or its stand-in `no_can`, `time`, `log`, `error`), the only files that include ESP-IDF headers
  (`deviations/boundary-files.yml`). `tools/check_layout.py` (preflight, ctest) keeps header, source and
  boundary file in step: a new peripheral adds `inc/.../<x>.hpp`, `src/<x>.cpp` and `src/boundary/<x>.cpp`.
- `tests/`: host tests; `tests/fakes/esp-idf/` fakes every SDK function the boundary files call.
- `examples/esp32/`: one ESP-IDF project for every target; the port stands alone, with no driver submodules.
- `third_party/kb-core/`: the pinned, unchanged kb-core copy (`tools/kb_core_snapshot.py`, `third_party/kb-core.lock`).
- `_config/`: tool configuration; `tools/`: pre-flight, layering rules, the snapshot tool.

## Rules

- Logic in the platform-free files, SDK calls in `src/boundary/`; a boundary function converts, calls and returns
  the `esp_err_t`, nothing more. Chip differences use `soc_caps.h` macros in boundary files only.
- Every wait bounded by a configured timeout; every failure a `kb::Status` with the `esp_err_t` in `Detail()`.
- C++20, no heap, no exceptions, no RTTI, no virtual functions; each type `static_assert`ed against its concepts.
- Never edit `third_party/kb-core` by hand.
- Every change: tests, docs and `CHANGELOG.md` together; `tools/preflight.sh` before a push; commits signed off;
  no attribution lines.
- Ask the maintainer before anything irreversible or public: releases, visibility, deleting branches or tags.
