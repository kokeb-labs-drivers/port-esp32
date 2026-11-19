# Getting started

## 1. Add the components

The port is the ESP-IDF component `kb_port_esp32`; it requires the component `kb` (kb-core). The repository's
`examples/esp32/components/` shows the layout: one folder named after each component, forwarding to the
sources.

```text
your-project/
├── CMakeLists.txt
├── components/
│   ├── kb/CMakeLists.txt             include(<port-esp32>/third_party/kb-core/CMakeLists.txt)
│   └── kb_port_esp32/CMakeLists.txt  include(<port-esp32>/CMakeLists.txt)
└── main/CMakeLists.txt               idf_component_register(... REQUIRES kb kb_port_esp32)
```

Set in `sdkconfig.defaults`:

```text
CONFIG_COMPILER_CXX_EXCEPTIONS=n
CONFIG_COMPILER_CXX_RTTI=n
CONFIG_FREERTOS_HZ=1000   # optional: millisecond waits are whole ticks (any rate works, rounded up)
CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION=y
```

## 2. Create the objects

```cpp
#include "kb/port/esp32/port.hpp"

namespace esp = kb::port::esp32;

esp::I2cConfig MakeI2c() {
  esp::I2cConfig c{};
  c.port = 0U;
  c.sda_pin = 8;
  c.scl_pin = 9;
  c.clock_hz = 400000U;
  return c;
}

extern "C" void app_main(void) {
  static esp::I2cController bus{MakeI2c()}; // static: drivers keep references
  if (bus.EnsureReady() != kb::Status::Ok) {
    // bus.LastError().Detail() is the esp_err_t
  }
  const std::array<uint8_t, 2> tx{0x06U, 0x00U};
  const kb::Status s = bus.Write(0x20U, tx); // NoDevice, Nack, Timeout, BusError, Busy ...
}
```

Objects keep their configuration and touch no hardware until `EnsureReady()`. Call `EnsureReady()` once from
one task before the object is shared (it creates the object's lock). After that the bus objects (I2C, SPI, UART,
ADC, CAN) may be called from several tasks, each reading its own `LastError()`; pins and PWM channels are used
from one context each ([Thread safety](thread-safety.md)). Release or destroy a shared object only after the
tasks that use it have stopped.

## 3. Bind a driver

A driver's port is a thin adapter from its hooks to the kb contracts (each driver repository documents its
adapter). Adapters are templates over the contracts, so the same adapter runs over this port, another MCU port, a host fake or a `*Ref` chosen at run time.

## Building the host tests

```bash
cmake -S . -B build -G Ninja -DKB_PORT_ESP32_BUILD_TESTS=ON
cmake --build build && ctest --test-dir build --output-on-failure
```

No ESP-IDF is needed: the boundary files build against fakes of every ESP-IDF function they call
([Testing and CI](testing.md)).

## kb-core

The port builds against kb-core's public `kb/<part>/` headers only. Off target it uses an installed kb-core
(`find_package(kb)`) when `CMAKE_PREFIX_PATH` finds one, else the pinned copy in `third_party/kb-core`
([SOUP and kb-core](soup.md)).
