# Thread safety

The port follows kb-core's thread models (kb-core `docs/design/handlers.md`): a bus is either owned by one task
(model O) or shared through a guarded facade (model G). The port is safe under both, because every shared
resource has its own bounded lock, a `kb::rtos::Mutex` taken with `TryLockFor(timeout)`.

| Object | Lock | Held for | When not had in time |
| --- | --- | --- | --- |
| `I2cController` | One per controller | One transfer (plus an opted-in probe and a recovery after a failure); `Release` | `Busy`, nothing sent |
| `SpiBus` | One per host, shared by its devices | One frame (one transaction) or one device change; `Release` | `Busy`, nothing clocked |
| `Uart` | One for the reader, one for the writer | One `Read`; one `Write` or `WaitTxDone`; `SetBaud` and `Release` take both | `Busy` |
| `AdcChannel` | One per ADC unit, shared by its channels | One conversion; opening and closing the unit | `Busy` |
| `CanController` | One per controller | `Recover`, `Release`; `Send` and `Receive` use the SDK's queues and are counted, so `Release` waits for them | `Busy` |
| `PwmChannel` | A busy flag per LEDC timer, shared by its channels | One timer change or duty write (a few 1 ms tries) | `Busy` |
| `OutputPin`, `InputPin` | None | Register writes; one object per pin, used from one context | |
| `Clock`, `BusyWait`, `LogSink` | None | Stateless | |

Rules:

- **Create, then share.** `EnsureReady()` creates the object's lock; call it once from one task before the object
  is used by others. Later calls are idempotent and cheap. A failed lock creation fails `EnsureReady()` (`Fault`).
- **Release under the lock.** `Release()` and the destructors take the object's lock (bounded) before they delete
  the SDK object, so a call in another task never runs on a deleted driver; when the lock is not had they return
  `Busy` and delete nothing. Destroy a shared object only after the tasks that use it have stopped.
- **Each task reads its own outcome.** `LastError()` (and `LastFailure()` of I2C) of a shared object return the
  calling task's last call on that object, from thread-local records (up to `kCallerRecordSlots`, 4, objects per
  task; the oldest is reused). Counters are atomic.
- **No lock across calls.** A lock is never held when a call returns. Every timeout is capped at `kMaxTimeout`
  (2 s) and every wait is bounded; a call may make several waits, so each peripheral page gives its worst case
  (lock wait, SDK waits, probe, recovery). An SPI frame whose worst case exceeds `kMaxTimeout` is refused.
- **One owner per pin, controller, slot and unit.** Pins, controller numbers (I2C port, UART port, SPI host, TWAI),
  SPI device slots, ADC channel slots and LEDC timers are claimed atomically by `EnsureReady()` before the SDK is
  called, so two objects (in any task) never share one.
- **Interrupts never take a lock.** The GPIO interrupt callback runs in interrupt context; it may only use
  `kb::rtos` `...FromIsr` calls.
- **Under a model G facade** the handler's lock is taken first, then the port's bus lock (kb-core's fixed order);
  the port's lock is the inner one and never waits for a handler.

Under `KB_RTOS=NONE` (the host tests) the same locks exist and a nested call is refused with `Busy`, which the
host tests use to prove every lock path.
