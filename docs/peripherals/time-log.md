# Clock, delay and log

## Clock

`kb::port::esp32::Clock` satisfies `kb::iface::MonotonicClock`: `Now()` is `esp_timer_get_time()` as a
`kb::Instant` (microseconds since boot, 64 bits). It never goes back and does not wrap in practice (584 000
years); it works before the scheduler starts and from an interrupt. Drivers with a 32-bit clock hook narrow it
with `kb::utils::NarrowUs` (the bno08x adapter does).

## Busy wait

`kb::port::esp32::BusyWait` satisfies `kb::iface::BusyWait`: `WaitUs(us)` is `esp_rom_delay_us(us)`, a CPU spin of
at least `us` microseconds. The contract is for waits below 1 ms; longer waits should sleep
(`kb::rtos::SleepFor`). Interrupts may lengthen the spin; nothing shortens it.

## Log

`kb::port::esp32::LogSink` satisfies `kb::iface::LogSink`: `Write(level, text)` is one `esp_log_write` line in
ESP-IDF's format (`I (timestamp) tag: text`), under the tag given to the constructor (a string literal: ESP-IDF
keeps the pointer). Text longer than `kLogMaxLine` (160 characters) is cut. The level maps to `ESP_LOG_ERROR`,
`ESP_LOG_WARN`, `ESP_LOG_INFO` and `ESP_LOG_DEBUG`, and ESP-IDF's level filter applies. Nothing is allocated; the
call takes as long as the console needs for one line. A null tag is written as `kb`. Task context only:
`esp_log_write` takes a lock, so never call `Write()` from an interrupt handler.

| Type | State | Copies |
| --- | --- | --- |
| `Clock` | None | Any number |
| `BusyWait` | None | Any number |
| `LogSink` | The tag pointer | Any number |
