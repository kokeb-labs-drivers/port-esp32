# ADC

`kb::port::esp32::AdcChannel` satisfies `kb::iface::AdcChannel` over ESP-IDF's `adc_oneshot` driver with the
`adc_cali` calibration (`esp_adc`).

## How it works

| Member | SDK call |
| --- | --- |
| `EnsureReady()` | `adc_oneshot_io_to_channel` (pin to unit and channel); the pin and a channel slot claimed; under the unit's lock `adc_oneshot_new_unit` once per unit (shared and counted) and `adc_oneshot_config_channel` (attenuation, default width); then the calibration: `adc_cali_create_scheme_curve_fitting` or `..._line_fitting` (with `default_vref_mv` on ESP32) |
| `ReadRaw()` | `adc_oneshot_read` under the unit's lock |
| `ReadMillivolts()` | `ReadRaw()` then `adc_cali_raw_to_voltage` |
| `FullScale()` | 2^`SOC_ADC_RTC_MAX_BITWIDTH` - 1 (4095 on 12-bit chips, 8191 on ESP32-S2) |
| `Release()` | Under the unit's lock (`Busy` when not had): the calibration deleted, the unit deleted with its last channel, the pin and slot given back |

## Pins and range

| Attenuation | Range (approximate, chip dependent) |
| --- | --- |
| `Db0` | 0 to 0.95 V |
| `Db2p5` | 0 to 1.25 V |
| `Db6` | 0 to 1.75 V |
| `Db12` (default) | 0 to 3.1 V |

Only ADC pins work (the pin's ADC function, [Targets](../targets.md)). ADC2 is shared with the radio on ESP32 and
ESP32-S3: while Wi-Fi runs, an ADC2 read can fail with `ESP_ERR_TIMEOUT` (`Timeout`); prefer ADC1.

## Calibration

The chip's eFuse holds factory calibration; the scheme is the chip's: curve fitting on ESP32-S3, C3, C6, H2 and
P4, line fitting on ESP32 and ESP32-S2. Without calibration data `Calibrated()` is false and `ReadMillivolts()`
returns `Unsupported` (`ESP_ERR_NOT_SUPPORTED`); `ReadRaw()` still works. An ESP32 whose eFuse has no Vref (early
chips) can still calibrate by line fitting with the board's measured Vref: set `AdcConfig::default_vref_mv` (about
1100 mV; ESP-IDF's `default_vref`). Without it, line fitting refuses (`ESP_ERR_INVALID_ARG` inside ESP-IDF) and the
port reports `Unsupported`.

## Timing and thread safety

One conversion takes some tens of microseconds. ESP-IDF refuses a read while another task reads the same unit, so
the port serialises every channel of a unit with the unit's lock, waiting at most `lock_timeout` (default 10 ms,
at most `kMaxTimeout`). Opening and closing the unit happen under the same lock, so `Release()` of one channel can
never delete the unit while another channel reads it. `LastError()` is the calling task's own last call.

## Results

| Result | When |
| --- | --- |
| `Ok` | One conversion |
| `Timeout` | `ESP_ERR_TIMEOUT`: ADC2 held by the radio |
| `Unsupported` | `ReadMillivolts()` without calibration |
| `NoDevice` | The pin has no ADC function (`ESP_ERR_NOT_FOUND`) |
| `InvalidArgument` | A negative pin, `ESP_ERR_INVALID_ARG` |
| `Busy` | The unit's lock was not had in time (`ESP_ERR_NOT_FINISHED`) |
| `NotReady` | Before `EnsureReady()` |
