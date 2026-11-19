# CAN (TWAI)

`kb::port::esp32::CanController` satisfies `kb::iface::CanBus` over ESP-IDF's TWAI driver, handle API
(`twai_driver_install_v2` and the `_v2` calls), so every controller of the chip can be used. Classic CAN 2.0A/B.

## How it works

| Member | SDK call |
| --- | --- |
| `EnsureReady()` | Claims TX and RX, `twai_driver_install_v2` (mode, pins, queues, timing, accept-all filter), `twai_start_v2` |
| `Send(frame, timeout)` | `twai_transmit_v2`, waiting at most `timeout` for a queue slot |
| `Receive(frame, timeout)` | `twai_receive_v2`, waiting at most `timeout` |
| `Status()` | `twai_get_status_info_v2`: state, TEC, REC, failed, missed, bus errors |
| `Recover(timeout)` | Bus-off: `twai_initiate_recovery_v2`, then status reads every millisecond until stopped (at most `timeout`), then `twai_start_v2`. Stopped: `twai_start_v2`. Running: nothing |
| `Release()` | Under the controller lock: new calls refused, running `Send`/`Receive`/`Status` calls waited for (at most `kMaxTimeout` plus 100 ms; else `Busy` and the controller stays up), then `twai_stop_v2`, `twai_driver_uninstall_v2` |

## Pins and bit timing

| Item | Note |
| --- | --- |
| TX, RX | Any GPIO to a CAN transceiver's TXD and RXD (the TWAI needs a transceiver on the bus) |
| Bit rates | 25k, 50k, 100k, 125k, 250k, 500k, 800k and 1M bit/s from ESP-IDF's `TWAI_TIMING_CONFIG_*` (sample point about 80 %); others are `InvalidArgument` |
| Modes | `Normal`; `NoAck` (sends without an acknowledgement: a node alone on the bus); `ListenOnly` (never drives the bus) |
| Controllers | `SOC_TWAI_CONTROLLER_NUM`: 2 on ESP32-C6, 3 on ESP32-P4, 1 elsewhere |

## Timing limits

| Wait | Bound |
| --- | --- |
| `Send` | The call's `timeout` for a transmit queue slot (the frame is then sent by the SDK's interrupt) |
| `Receive` | The call's `timeout` |
| `Recover` | The call's `timeout`; bus-off recovery takes 128 x 11 recessive bits (1.4 ms at 1 Mbit/s, 56 ms at 25 kbit/s) |

Every timeout is capped at `kMaxTimeout` (2 s), so kb-core's `kMaxMilliseconds` never becomes `portMAX_DELAY`. The
SDK queues are safe from several tasks; `Send()` and `Receive()` take no lock of their own but are counted, so
`Release()` never uninstalls the driver under a running call. `Recover()` holds the controller's lock.

## Results

| Result | When |
| --- | --- |
| `Ok` | Queued, or taken |
| `Timeout` | No slot or no frame in time (`ESP_ERR_TIMEOUT`) |
| `BusError` | The controller is not running: bus-off or recovering (`ESP_ERR_INVALID_STATE`), or the SDK refused the frame (`ESP_FAIL`) |
| `InvalidArgument` | dlc above 8, an identifier wider than 11 (or 29) bits, a bit rate without timing, a bad pin or controller |
| `Unsupported` | A chip without TWAI (`ESP_ERR_NOT_SUPPORTED`) |
| `Busy` | Another object has the controller; `Recover()` while another `Recover()` runs |
| `NotReady` | Before `EnsureReady()` |
