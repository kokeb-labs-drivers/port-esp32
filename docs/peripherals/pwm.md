# PWM (LEDC)

`kb::port::esp32::PwmChannel` satisfies `kb::iface::PwmOutput` over one channel of ESP-IDF's `ledc` driver in
low-speed mode (every target has it).

## How it works

| Member | SDK call |
| --- | --- |
| `EnsureReady()` | Claims the pin; takes the timer (configures it, or joins it when it runs at the same frequency); `ledc_channel_config` (duty 0, output inverted for `idle_high`), `ledc_stop` |
| `SetFrequencyHz(hz)` | `ledc_timer_config` with `LEDC_AUTO_CLK` at the finest duty resolution the clock allows, stepping down until the SDK accepts it; the duty is applied again. `Busy` when another channel shares the timer at another frequency |
| `SetDuty(duty)` | While enabled: `ledc_set_duty` and `ledc_update_duty`; otherwise kept for `Enable(true)` |
| `Enable(true)` | `ledc_set_duty` and `ledc_update_duty` (which turns the output back on) |
| `Enable(false)` | `ledc_stop` at the idle level |
| `Release()` | `ledc_stop` (the idle level), then the timer and pin given back |

## Resolution and range

The resolution search starts at the largest number of bits with `hz << bits` at most 80 MHz (the fastest LEDC
source of the family), capped by `SOC_LEDC_TIMER_BIT_WIDTH` (14 or 20 bits), and steps down while
`ledc_timer_config` refuses. `ResolutionBits()` reports the result. The duty is `permyriad x 2^bits / 10000`
rounded to nearest (`DutyCounts`) and clamped to `2^bits` (100 %, always high; the top of ESP-IDF's documented
range). ESP-IDF's `ledc_set_duty` does not check the duty against the timer's resolution, so the port takes the
resolution from its record of the timer at every write, never from a value cached by the channel.

| Frequency | Resolution (80 MHz source) |
| --- | --- |
| 1 kHz | 14 bits (16 on chips with 20-bit timers) |
| 20 kHz | 11 bits |
| 1 MHz | 6 bits |
| 40 MHz | 1 bit |

A frequency of 0 or above what 1 bit allows is `InvalidArgument`.

## Pins and sharing

Any output GPIO, claimed by the channel. Channel 0 to `SOC_LEDC_CHANNEL_NUM` - 1, timer 0 to 3. The port keeps one
record per timer (frequency, resolution, users). Channels may share a timer only at one frequency: a second
channel at another frequency gets `Busy` from `EnsureReady()`, and `SetFrequencyHz()` that would change the
frequency under another channel gets `Busy`; with one user the timer follows that channel. So a channel's compare
value always fits its timer's period (otherwise a 50 % duty could leave a heater or motor at full power).

An `idle_high` channel is configured with the output inverted: duty 0 then shows high from the moment LEDC takes
the pin (no low pulse at start-up), and the port writes the complement of the duty, so `SetDuty()` keeps meaning
the high time.

## Timing

Each call is a few register writes in the SDK. The timer record is taken with a few 1 ms tries, then `Busy`. A duty
change takes effect at the next period. One object per channel, used from one context at a time.
