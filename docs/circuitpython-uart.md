# CircuitPython UART behavior (half-duplex Dynamixel bus)

Why `hardware.py` drives the Dynamixel DIR pin differently on each board. Checked against the CircuitPython and pico-sdk sources on `main`/`master`, 2026-10-04:

- https://github.com/adafruit/circuitpython/blob/main/ports/raspberrypi/common-hal/busio/UART.c
- https://github.com/adafruit/circuitpython/blob/main/ports/espressif/common-hal/busio/UART.c
- https://github.com/adafruit/circuitpython/blob/main/shared-bindings/time/__init__.c
- https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_uart/include/hardware/uart.h

## `time.sleep()` rounds to whole milliseconds

```c
mp_float_t msecs = 1000.0 * seconds + 0.5;
common_hal_time_delay_ms(msecs);
```

So `time.sleep(len(packet) * 10 / baudrate)` (about 0.1 ms at 3 Mbaud) is effectively `sleep(0)`. It can't be used to wait for the end of a packet.

## ESP32: `write()` waits until everything has been sent

`common_hal_busio_uart_write` loops on `uart_wait_tx_done()` before returning. So toggling DIR from Python around `uart.write()` works, which is why the original ESP32 code worked.

The espressif port also supports `rs485_dir`, using the chip's real hardware RS485 half-duplex mode on the RTS pin. The firmware doesn't use it on the ESP32, because the GPIO toggle is proven there.

## RP2040 (Pico): `write()` returns before the data is sent

Without `rs485_dir`, `write()` returns as soon as the last bytes are in the 32-byte TX FIFO, so dropping DIR from Python would truncate the packet.

With `rs485_dir`, the driver toggles the pin in C around the write and waits for the UART's BUSY flag. BUSY clears only after the last stop bit has gone out:

```c
if (self->rs485_dir_pin != NO_PIN) {
    uart_tx_wait_blocking(self->uart);          // previous TX finished
    gpio_put(self->rs485_dir_pin, !self->rs485_invert);
}
/* ... fill TX FIFO ... */
if (self->rs485_dir_pin != NO_PIN) {
    uart_tx_wait_blocking(self->uart);          // last stop bit sent
    gpio_put(self->rs485_dir_pin, self->rs485_invert);
}
```

```c
static inline void uart_tx_wait_blocking(uart_inst_t *uart) {
    while (uart_get_hw(uart)->fr & UART_UARTFR_BUSY_BITS) tight_loop_contents();
}
```

The RP2040 UART has no hardware direction-control pin. This is GPIO switching in firmware, but it's accurate to within about a microsecond, well inside the motors' 500 µs return delay. `rs485_invert=False` (the default) means DIR is high while transmitting, matching the ESP32 wiring.

## RP2040 UART pin pairs

TX and RX must come from the same UART instance. Pins available on the Pico header:

| UART | TX | RX |
|---|---|---|
| UART0 | GP0, GP12, GP16, GP28 | GP1, GP13, GP17 |
| UART1 | GP4, GP8, GP20 | GP5, GP9, GP21 |

See the RP2040 datasheet, GPIO function table, section 2.19.2. At 3 Mbaud from the 125 MHz peripheral clock, the actual rate is within about 0.2 % of nominal.
