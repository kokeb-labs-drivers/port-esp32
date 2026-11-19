# UART

`kb::port::esp32::Uart` satisfies `kb::iface::UartPort`, `UartBaudControl` and `UartTxDrain` over ESP-IDF's
`uart` driver, with optional RS-485 half duplex.

## How it works

| Step | SDK call |
| --- | --- |
| `EnsureReady()` | Claims the pins, `uart_driver_install` (receive ring buffer, no transmit buffer, an event queue of `event_queue`, default 32), `uart_param_config`, `uart_set_pin`, `uart_set_mode(UART_MODE_RS485_HALF_DUPLEX)` with `rs485` |
| `Write(tx)` | `uart_tx_chars` into the hardware FIFO, again after eight character times, until every byte is in |
| `Read(rx, timeout)` | Pending error events first (a full queue is `kEventsLost`, then `xQueueReceive` with no wait); then `uart_read_bytes` of one byte with the timeout, then of what `uart_get_buffered_data_len` says is there, without waiting |
| `FlushInput()` | `uart_flush_input` and the pending error events |
| `SetBaud(baud)` | `uart_set_baudrate` with both locks held |
| `WaitTxDone(timeout)` | `uart_wait_tx_done` under the writer lock |
| `Release()` | `uart_driver_delete` with both locks held (`Busy` when either is not had) |

## Pins and clock

| Item | Note |
| --- | --- |
| TX, RX | Any GPIO through the GPIO matrix (TX must be output-capable) |
| RTS / DE | `rts_pin`; with `rs485` the controller drives it as the transceiver's DE while sending |
| CTS | `cts_pin` enables CTS flow control; a held CTS stops `Write()` until its timeout |
| Baud | 1 to 5 000 000 bit/s from `UART_SCLK_DEFAULT`; data bits 5 to 8, parity none/even/odd, 1 or 2 stop bits |
| Port 0 | Often the console; use another port for devices |

## Timing limits

| Wait | Bound |
| --- | --- |
| The writer lock, and `Write()` as a whole | `UartConfig::write_timeout` (default 100 ms) |
| The reader lock and the first byte | The `timeout` of the call |
| `WaitTxDone()` | Its `timeout` for the writer lock and again for the SDK (ESP-IDF holds its TX mutex for the whole wait, and `uart_tx_chars` waits for that mutex without a limit, so the port's writer lock keeps a `Write()` from waiting behind it) |
| Any wait | Capped at `kMaxTimeout` (2 s): kb-core's `kMaxMilliseconds` reads at most 2 s |
| Refill of the FIFO | A busy wait of eight character times below 1 ms, else a 1 ms sleep, so there is no gap longer than the FIFO's drain time |

## Results

| Result | When |
| --- | --- |
| `Ok` | `Write`: every byte is in the FIFO. `Read`: one byte or more (`Value()` is the count) |
| `Timeout` | `Read`: nothing arrived in time. `Write`: the FIFO did not drain (CTS held) |
| `Overflow` | `Read`: the FIFO or the ring buffer overflowed, or the event queue was full (`kEventsLost`), since the last `Read` (`Detail()`: `uart_flags`) |
| `BusError` | `Read`: a framing, parity or break error since the last `Read` (`Detail()`: `uart_flags`) |
| `Busy` | The reader or writer lock was not had in time |
| `InvalidArgument` | Empty span, baud 0 or above 5 Mbit/s, a bad configuration |
| `NotReady` | Before `EnsureReady()` |
| `Fault` | `ESP_FAIL` from `uart_tx_chars` or `uart_read_bytes` |

An error is reported once, by the next `Read()`; the bytes received stay in the ring buffer and the following
`Read()` returns them.

ESP-IDF reports receive errors through one event queue shared with the data events (one per FIFO-full or
receive-timeout interrupt) and drops an event when the queue is full, error events included, logging it only at
verbose level. The port reads the queue's fill before draining it: a full queue means events may have been lost,
and `Read()` reports `Overflow` with `kEventsLost`, so a lost framing or parity error is never reported as `Ok`.
The error state of the bytes received meanwhile is unknown; treat them as suspect. Size `event_queue` for the
bursts that arrive between two `Read()` calls. Residual risk: an error that arrives while the queue is full is
not identified, only flagged. Exactly `event_queue` events with none dropped are also reported as lost: the check
fails closed. After a receive error the next `Read()` returns the bytes that arrived meanwhile as `Ok`: a protocol
driver (Modbus, a framed sensor stream) should call `FlushInput()` and resynchronise after an error. Reader and writer have separate locks, so one task can block in `Read()` while another
writes.
