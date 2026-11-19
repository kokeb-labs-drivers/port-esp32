# Error mapping

Every call returns `kb::Status` or `kb::Result<T>`. The ESP-IDF `esp_err_t` behind a failure is kept as the
vendor code: in `Detail()` of a `kb::Result`, and in `LastError().Detail()` of the object for calls that return a
bare `kb::Status`. `ESP_FAIL` (-1) appears as `0xFFFFFFFF`. On the shared objects (I2C, SPI, UART, ADC, CAN)
`LastError()` is the calling task's own last call; pins and PWM channels keep one record (one context each).

## The general mapping

`MapEspErr()` in `kb/port/esp32/error.hpp`; the values are checked against ESP-IDF's `esp_err.h` by a
`static_assert` in `src/boundary/error.cpp`, so an SDK change stops the build.

| esp_err_t | Value | kb::Status |
| --- | --- | --- |
| `ESP_OK` | 0 | `Ok` |
| `ESP_FAIL` | -1 | `Fault` |
| `ESP_ERR_NO_MEM` | 0x101 | `Fault` |
| `ESP_ERR_INVALID_ARG` | 0x102 | `InvalidArgument` |
| `ESP_ERR_INVALID_STATE` | 0x103 | `NotReady` |
| `ESP_ERR_INVALID_SIZE` | 0x104 | `OutOfRange` |
| `ESP_ERR_NOT_FOUND` | 0x105 | `NoDevice` |
| `ESP_ERR_NOT_SUPPORTED` | 0x106 | `Unsupported` |
| `ESP_ERR_TIMEOUT` | 0x107 | `Timeout` |
| `ESP_ERR_INVALID_RESPONSE` | 0x108 | `BusError` |
| `ESP_ERR_INVALID_CRC` | 0x109 | `CrcMismatch` |
| `ESP_ERR_NOT_FINISHED` | 0x10C | `Busy` |
| `ESP_ERR_NOT_ALLOWED` | 0x10D | `Busy` (the port's "owned by another object") |
| anything else | | `Fault` |

## Where a peripheral refines it

| Peripheral | SDK result | kb::Status | Why |
| --- | --- | --- | --- |
| I2C | `ESP_ERR_INVALID_STATE` (v5.x) or `ESP_ERR_INVALID_RESPONSE` (v6.0) from a transfer, later than its wire time plus 1 ms | `Timeout` (`Timeout`) | ESP-IDF uses this one code for NACK, stretch timeout and lost arbitration; a NACK ends the transfer early |
| I2C | The same code, faster than the earliest timeout (the bands of [I2C](peripherals/i2c.md#results-and-failure-kinds)), probe not allowed for the address (the default) | `Nack` (`DataNack`) | Unsplit: the address or a data byte was refused; no extra frame is sent |
| I2C | The same code, between the earliest timeout and the wire time plus 1 ms (the ambiguous band) | `BusError` (`Bus`) | A NACK or an early stretch timeout; an allowed probe may settle it (`NoDevice`) |
| I2C | The same code, fast or ambiguous, allowed probe not acknowledged | `NoDevice` (`AddressNack`) | `AllowNackProbe()` |
| I2C | The same code, fast, allowed probe acknowledged | `Nack` (`DataNack`) | |
| I2C | `ESP_ERR_TIMEOUT` from a transfer | `Busy` | Only ESP-IDF's bus lock returns it; nothing was sent |
| I2C | `ESP_ERR_TIMEOUT` from a probe (empty write, lines high) | `Timeout` | The probe's lock or a stretch past 20 ms |
| I2C | Any failure with SDA or SCL low | `BusError` (`BusStuck`) | A stuck line, whatever the SDK said |
| SPI | `ESP_ERR_TIMEOUT` from `polling_end`, then `ESP_OK` from the second wait | `Timeout` | The frame is complete, but late |
| SPI | `ESP_ERR_TIMEOUT` from both waits | `Fault` (detail `ESP_ERR_INVALID_STATE`) | The host is failed; no later SDK call on it |
| SPI | `ESP_ERR_NO_MEM` from a transfer | `Busy` | ESP-IDF's per-transfer bounce on cached chips found no heap: transient |
| CAN | `ESP_ERR_INVALID_STATE` from send or receive | `BusError` | The controller is off the bus or recovering |
| CAN | `ESP_FAIL` from send | `BusError` | The frame was refused |
| UART | Error events (overflow, framing, parity, break) | `Overflow` or `BusError` | Detail is the `uart_flags` mask, not an `esp_err_t` |
| UART | The event queue found full | `Overflow` | `kEventsLost`: ESP-IDF dropped events, errors included |
| GPIO | ISR service installed by the application, callback not in IRAM | `Unsupported` | Detail `ESP_ERR_NOT_SUPPORTED`; see `InputConfig::foreign_isr_service_ok` |
| Lock waits | (no SDK call) | `Busy` | Detail is `ESP_ERR_TIMEOUT` |
| A pin, slot or timer of another object | (no SDK call) | `Busy` | Detail is `ESP_ERR_NOT_ALLOWED` (or `ESP_ERR_NO_MEM` when every slot is taken) |
| Calls before `EnsureReady()` | (no SDK call) | `NotReady` | Detail is `ESP_ERR_INVALID_STATE` |
| Arguments refused by the port | (no SDK call) | `InvalidArgument` | Detail is `ESP_ERR_INVALID_ARG` |
