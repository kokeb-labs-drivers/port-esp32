# SOUP and kb-core

## Software of unknown provenance

| Item | Version | Where | Use | Known anomalies the port handles |
| --- | --- | --- | --- | --- |
| ESP-IDF | v5.5 (`release/v5.5`; CI image `espressif/idf:v5.5.6@sha256:bf72fe7a15c22849a9a94147301165a545fb8905d97c176cf748e3e4614cac84`, set in `ci-cmake-matrix.yml`) | The user's ESP-IDF install | Every peripheral driver, FreeRTOS, esp_timer, esp_log | See "ESP-IDF anomalies" below |
| FreeRTOS | ESP-IDF's (SMP) | ESP-IDF | Through `kb::rtos` (mutexes) and the SDK drivers | |

## ESP-IDF anomalies the port handles

Read in the ESP-IDF v5.5.6 sources (`release/v5.5`); each has a host test over the fakes, which model the
behaviour.

| Area | Anomaly | What the port does |
| --- | --- | --- |
| I2C | A transfer returns `ESP_ERR_INVALID_STATE` for a NACK, a stretch or software timeout and lost arbitration alike; `ESP_ERR_TIMEOUT` only when its bus lock was not had. The header's return list says otherwise (the code differs from the documentation). v6.0 returns `ESP_ERR_INVALID_RESPONSE` for the same set | Classifies by time against the wire time; the probe that splits address from data NACK is opt-in per address; the code is chosen by `ESP_IDF_VERSION` |
| I2C | Millisecond timeouts are converted with `pdMS_TO_TICKS` (rounds down: 1 to 9 ms is 0 ticks at 100 Hz); `-1` waits for ever | Every value is whole ticks rounded up plus one; every timeout is capped at `kMaxTimeout` |
| I2C | `i2c_master_bus_reset` reports an error when no line is held | Ignored at start-up |
| I2C | `i2c_master_device_change_address` returns `ESP_ERR_INVALID_ARG` without giving its bus lock back | Never reached: the port checks 0x08 to 0x77 first |
| SPI | `spi_device_polling_start` and `spi_device_acquire_bus` take only `portMAX_DELAY` | `acquire_bus` is never called (one transaction per frame); `polling_start` only under the port's bus lock, on a host the port owns and that has no unfinished transaction |
| SPI | `spi_device_polling_end` returns `ESP_ERR_TIMEOUT` with the transaction in flight, the SDK bus lock held and the device not removable; it keeps a pointer to the transaction descriptor | The wait is sized to the wire time; a second wait; then the host is marked failed; descriptors live in a static table |
| SPI | A DMA buffer that is not DMA-capable or aligned gets a heap copy per transaction; on chips without a cache only the address is checked, and every receive descriptor is rounded up to whole words, so the DMA can write up to 3 bytes past a buffer whose length is not a multiple of 4 | Every DMA frame goes through a port-owned, aligned buffer per host, allocated once; the buffer outlives a failed host |
| I2C | The hardware SCL limit (`scl_wait_us`) is capped per chip and rounded up to a power of two of the clock (newer chips) | The time bands use the capped value, which a stretch timeout can never undercut |
| UART | `uart_driver_install` on an installed port returns `ESP_FAIL` and leaves the first driver alone | The port number is claimed by the logic; the queue length is recorded only after a successful install |
| UART | The ISR drops an event when the event queue is full (logged at verbose level only), error events included; `uart_wait_tx_done` holds the TX mutex for its wait and `uart_tx_chars` waits for it without a limit | A full queue is reported as `kEventsLost`; the queue length is configurable; `WaitTxDone` takes the port's writer lock; the FIFO is written with `uart_tx_chars` (never `uart_write_bytes`, which waits without bound for buffer space) |
| LEDC | `ledc_set_duty` does not check the duty against the timer's resolution | Per-timer records; a shared timer keeps one frequency; the duty is clamped to 2^bits |
| GPIO | `gpio_reset_pin` disables the output and enables the pull-up; `gpio_config` drives the latch's 0 first | Release drives a safe level and leaves the pin an output; the level is written before the direction |
| GPIO | An ISR service installed with `ESP_INTR_FLAG_IRAM` calls handlers while the flash cache is off | The port installs it without the flag; a service installed by the application takes only IRAM callbacks unless the application states otherwise |
| ADC | ESP32 line fitting without an eFuse Vref needs `default_vref` | `AdcConfig::default_vref_mv` |
| ADC | ADC2 shared with the radio on ESP32 and ESP32-S3 | `Timeout` |

The CI workflows call `kokeb-labs/ci-tools` at a full commit SHA (the `v1` tag at the time of pinning,
`dc73327`); a bump is a reviewed change of every caller.

ESP-IDF is never copied into this repository. The SDK headers are reached as system headers on target; on the
host the fakes in `tests/fakes/esp-idf` stand in for them.

## kb-core

kb-core (`kokeb-labs/kb-core`) is a private repository of another organisation, so this repository's CI cannot
fetch it, and kb-core has no release tag yet. The port therefore carries an **unchanged copy** of the kb-core files
it builds against in `third_party/kb-core`, at the commit recorded in `third_party/kb-core.lock` with the ref it
was taken from and the SHA-256 of every file:

- `tools/kb_core_snapshot.py --update <kb-core checkout>` is the only writer. It refuses a checkout with local
  changes in the copied paths, and a commit that is neither tagged nor on kb-core's `main` branch, so the lock
  never names a state that kb-core can delete (a pull-request branch);
- `tools/kb_core_snapshot.py --check` and every CMake configure re-hash every file against the lock; a difference
  stops the build;
- the port uses kb-core only through its public `kb/<part>/` headers (kb-core ADR-0007), checked by the layering
  rules (`tools/layering/rules.yml`).

| kb-core | Commit | Parts used |
| --- | --- | --- |
| `main` (no release tag yet) | see `third_party/kb-core.lock` | `utils`, `rtos` (FreeRTOS backend on target, none backend on the host), `interfaces` |

### The plan

The copy is transitional. In this order:

1. kb-core tags a release that contains what the port needs (its `main` builds and passes the port's tests today),
   and the copy moves to that tag (`--update` records `ref tag <name>`).
2. kb-core publishes itself as an ESP-IDF component (an `idf_component.yml` in kb-core, released on its tags).
   The port's own `idf_component.yml` then names `kb` at that exact version, ESP-IDF's `dependencies.lock` pins the
   commit and hash, and the forwarding `examples/esp32/components/kb` folder goes away. Host builds use
   `find_package(kb)` (already supported: an installed kb-core is preferred over the copy).
3. kb-core's `catalog/compat.yml` records the kb-core x port row; `third_party/kb-core` and the snapshot tool are
   deleted.

Making the port public before kb-core would publish kb-core's sources inside the port: that is the kb-core
owner's decision, not this repository's.
