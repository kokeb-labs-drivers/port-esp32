# Figures

<!-- Written by kokeb-docs figures from docs/diagrams/figures/*.py. Do not edit by hand. -->

Every figure on this site is drawn by a Python module from facts extracted from the code, so it is rebuilt,
not redrawn, when the code changes. CI runs `kokeb-docs figures --check` and fails when a
committed figure no longer matches the code. Each figure below lists the files it is drawn from.

## Sending and receiving a CAN frame

<!-- figure: can-send-receive -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="can-send-receive.dark.svg">
  <img src="can-send-receive.svg"
    alt="Sequence of a CAN Send and a Receive on the ESP32 port, in which Send returns once the TWAI driver
    has queued the frame, the controller sends it on the bus by interrupt, and Receive takes a frame
    another node sent.">
</picture>
<!-- /figure -->

Hardware description: [`docs/model/can-send-receive.seq.yml`](../model/can-send-receive.seq.yml)

Drawn from:

- [`docs/model/can-send-receive.seq.yml`](../model/can-send-receive.seq.yml)
- [`inc/kb/port/esp32/can.hpp`](../../inc/kb/port/esp32/can.hpp)
- [`src/boundary/can.cpp`](../../src/boundary/can.cpp)
- [`src/can.cpp`](../../src/can.cpp)

## One I2C register read

<!-- figure: i2c-write-read -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="i2c-write-read.dark.svg">
  <img src="i2c-write-read.svg"
    alt="Sequence of an I2C WriteRead on the ESP32 port, in which the controller takes its lock, ESP-IDF
    sends the register address and reads the reply after a repeated START, and the controller turns the
    result into a kb::Status.">
</picture>
<!-- /figure -->

Hardware description: [`docs/model/i2c-write-read.seq.yml`](../model/i2c-write-read.seq.yml)

Drawn from:

- [`docs/model/i2c-write-read.seq.yml`](../model/i2c-write-read.seq.yml)
- [`inc/kb/port/esp32/i2c.hpp`](../../inc/kb/port/esp32/i2c.hpp)
- [`src/boundary/i2c.cpp`](../../src/boundary/i2c.cpp)
- [`src/i2c.cpp`](../../src/i2c.cpp)

## Contracts on the ESP32 port

<!-- figure: port-contracts -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="port-contracts.dark.svg">
  <img src="port-contracts.svg"
    alt="Matrix of the kb-core contracts the ESP32 port implements for I2C, SPI, UART, CAN, ADC, PWM, GPIO,
    clock and log, each marked native, adapted, planned or not available.">
</picture>
<!-- /figure -->

Hardware description: [`docs/model/port-contracts.matrix.yml`](../model/port-contracts.matrix.yml)

Drawn from:

- [`docs/model/port-contracts.matrix.yml`](../model/port-contracts.matrix.yml)
- [`inc/kb/port/esp32/can.hpp`](../../inc/kb/port/esp32/can.hpp)
- [`inc/kb/port/esp32/gpio.hpp`](../../inc/kb/port/esp32/gpio.hpp)
- [`inc/kb/port/esp32/i2c.hpp`](../../inc/kb/port/esp32/i2c.hpp)
- [`inc/kb/port/esp32/log.hpp`](../../inc/kb/port/esp32/log.hpp)
- [`inc/kb/port/esp32/port.hpp`](../../inc/kb/port/esp32/port.hpp)
- [`inc/kb/port/esp32/spi.hpp`](../../inc/kb/port/esp32/spi.hpp)
- [`inc/kb/port/esp32/time.hpp`](../../inc/kb/port/esp32/time.hpp)
- [`inc/kb/port/esp32/uart.hpp`](../../inc/kb/port/esp32/uart.hpp)
- [`src/i2c.cpp`](../../src/i2c.cpp)
- [`src/spi.cpp`](../../src/spi.cpp)

## One SPI frame

<!-- figure: spi-transfer -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="spi-transfer.dark.svg">
  <img src="spi-transfer.svg"
    alt="Sequence of an SPI Transfer on the ESP32 port, in which the device takes the bus lock, ESP-IDF
    clocks the whole frame as one polling transaction with chip select low, and the port waits for its
    end with a bound sized to the frame.">
</picture>
<!-- /figure -->

Hardware description: [`docs/model/spi-transfer.seq.yml`](../model/spi-transfer.seq.yml)

Drawn from:

- [`docs/model/spi-transfer.seq.yml`](../model/spi-transfer.seq.yml)
- [`inc/kb/port/esp32/spi.hpp`](../../inc/kb/port/esp32/spi.hpp)
- [`src/boundary/spi.cpp`](../../src/boundary/spi.cpp)
- [`src/spi.cpp`](../../src/spi.cpp)

## A UART request and reply

<!-- figure: uart-write-read -->
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="uart-write-read.dark.svg">
  <img src="uart-write-read.svg"
    alt="Sequence of a UART Write followed by a Read on the ESP32 port, in which the port fills the transmit
    FIFO itself, the reply lands in ESP-IDF's receive ring buffer, and Read waits only for its first
    byte.">
</picture>
<!-- /figure -->

Hardware description: [`docs/model/uart-write-read.seq.yml`](../model/uart-write-read.seq.yml)

Drawn from:

- [`docs/model/uart-write-read.seq.yml`](../model/uart-write-read.seq.yml)
- [`inc/kb/port/esp32/uart.hpp`](../../inc/kb/port/esp32/uart.hpp)
- [`src/boundary/uart.cpp`](../../src/boundary/uart.cpp)
- [`src/uart.cpp`](../../src/uart.cpp)
