# GPIO

`kb::port::esp32::OutputPin` satisfies `kb::iface::GpioOutput`; `kb::port::esp32::InputPin` satisfies
`kb::iface::GpioInput` and `kb::iface::GpioInterrupt`. ESP-IDF `gpio` driver.

## How it works

| Member | SDK call |
| --- | --- |
| `OutputPin::EnsureReady()` | Claims the pin, `gpio_set_level(initial)` first (no glitch), then `gpio_config` as input and output (open drain with `open_drain`) with the pulls |
| `Set(level)` | `gpio_set_level` |
| `Toggle()` | `gpio_get_level` of the pin itself, then `gpio_set_level` of the opposite |
| `InputPin::EnsureReady()` | Claims the pin, `gpio_config` as input, `gpio_set_pull_mode` |
| `Get(level)` | `gpio_get_level` |
| `EnableInterrupt(edge, fn, ctx)` | `gpio_install_isr_service(0)` once (no `ESP_INTR_FLAG_IRAM`); then `gpio_set_intr_type`, `gpio_isr_handler_add`, `gpio_intr_enable`. A service installed by the application first takes only a callback in IRAM, or `InputConfig::foreign_isr_service_ok` (else `Unsupported`) |
| `DisableInterrupt()` | `gpio_intr_disable`, `gpio_isr_handler_remove` |
| `OutputPin::Release()`, destructor | `gpio_set_level(safe)`: the pin stays an output at `OutputConfig::safe` (default: the initial level); the claim is given back |
| `InputPin::Release()`, destructor | The interrupt disabled; the pin stays an input as configured; the claim is given back |

## Pins

| Item | Note |
| --- | --- |
| Valid pins | `GPIO_IS_VALID_GPIO` for inputs and `GPIO_IS_VALID_OUTPUT_GPIO` for outputs ([Targets](../targets.md)) |
| One object per pin | A pin claimed by another port object (a pin, or an I2C, SPI, UART, ADC, PWM or CAN pin) gets `Busy` from `EnsureReady()` |
| Strapping pins | Usable after boot; the level at reset decides the boot mode (see the chip's datasheet) |
| Levels | Electrical; whether low means asserted is the caller's (the drivers' adapters say so) |

## Release and safe levels

The port never calls `gpio_reset_pin`: it disables the output and enables the internal pull-up, which lifts an
active-high enable line and can switch its load on. Release and the destructor drive the safe level instead and
leave the pin an output, so an object going out of scope on a fault path cuts its output rather than floating it.
The level is also written before the pin becomes an output, because `gpio_config()` alone leaves the output latch
at 0 (observed on ESP32 with an earlier implementation: an active-low output booted asserted and held a slave in
reset for about 180 ms). No pull resistor is enabled unless configured.

## Interrupts and the flash cache

The port installs the ISR service without `ESP_INTR_FLAG_IRAM`, so callbacks may be in flash: they are held off
while the flash cache is disabled (during NVS or OTA writes) and run afterwards. A service installed by the
application with `ESP_INTR_FLAG_IRAM` runs handlers with the cache off, and a callback in flash then crashes the
chip. The port cannot read the flags of a service it did not install, so it takes only an IRAM callback
(`IRAM_ATTR`) in that case, unless `InputConfig::foreign_isr_service_ok` says the application installed it
without the flag. The entry address is all the port can check: everything the callback calls and the data it reads
must also be in IRAM and DRAM.

## Timing and context

`Set()`, `Toggle()` and `Get()` are register accesses with no wait and may be called from an interrupt. The
interrupt callback runs in the ISR service's interrupt context and may only use the `...FromIsr` calls of
`kb::rtos`. A pin is used from one context at a time: `Toggle()` from two contexts at once may lose a toggle.

## Results

| Result | When |
| --- | --- |
| `Ok` | Done |
| `InvalidArgument` | Not a GPIO (or not an output pin), a null callback, `ESP_ERR_INVALID_ARG` |
| `Busy` | Another object has the pin |
| `NotReady` | Before `EnsureReady()`, or `ESP_ERR_INVALID_STATE` (the ISR service not installed) |
| `Unsupported` | A flash callback on an ISR service the application installed (see above) |
| `Fault` | `ESP_ERR_NO_MEM` from the ISR service |
