"""Board pin mapping and bus setup.

Supports the ESP32 DevKitC V4 and the Raspberry Pi Pico (RP2040). The board
is detected from board.board_id; any pin can be overridden with a "pins"
object in config.json, e.g. {"pins": {"dxl_dir": "GP17"}}.
"""

import board
import busio
import json
import digitalio
from dynamixel import Dynamixel

# dxl_*: Dynamixel half-duplex bus.  cmd_*: JSON command UART to the host.
# dxl_dir_hw: let the UART drive the direction pin itself (rs485_dir).
# RP2040 UART writes return before the last bytes leave the FIFO and
# time.sleep() can't wait microseconds, so the Pico must use dxl_dir_hw.
PROFILES = {
    "esp32": {
        "dxl_tx": "IO4",
        "dxl_rx": "IO13",
        "dxl_dir": "IO17",
        "dxl_dir_hw": False,
        "cmd_tx": "IO33",
        "cmd_rx": "IO32",
    },
    # TX/RX pairs must belong to the same RP2040 UART:
    # GP4/GP5 = UART1, GP0/GP1 = UART0.
    "pico": {
        "dxl_tx": "GP4",
        "dxl_rx": "GP5",
        "dxl_dir": "GP6",
        "dxl_dir_hw": True,
        "cmd_tx": "GP0",
        "cmd_rx": "GP1",
    },
}


def detect_profile():
    board_id = board.board_id
    if board_id.startswith("raspberry_pi_pico"):
        return "pico"
    if "esp32" in board_id:
        return "esp32"
    raise RuntimeError(
        f"Unsupported board '{board_id}'. Set \"board\" to one of "
        f"{list(PROFILES)} in config.json."
    )


def load_config(config=None):
    """Return config, reading /config.json if none is given (e.g. at the REPL)."""
    if config is None:
        with open("/config.json", "r") as f:
            config = json.load(f)
    return config


def load_pins(config=None):
    """Return the pin settings for this board, with config.json overrides."""
    config = load_config(config)
    name = config.get("board") or detect_profile()
    pins = dict(PROFILES[name])
    pins.update(config.get("pins", {}))
    pins["board"] = name
    return pins


def _pin(name):
    return getattr(board, name)


def dynamixel_bus(config=None):
    """Create the Dynamixel driver at config["dxl_baudrate"]."""
    config = load_config(config)
    pins = load_pins(config)
    uart_args = {
        "tx": _pin(pins["dxl_tx"]),
        "rx": _pin(pins["dxl_rx"]),
        "baudrate": config["dxl_baudrate"],
        "timeout": 0.1,
        "receiver_buffer_size": 256,
    }
    if pins["dxl_dir_hw"]:
        uart = busio.UART(rs485_dir=_pin(pins["dxl_dir"]), **uart_args)
        return Dynamixel(uart)
    uart = busio.UART(**uart_args)
    return Dynamixel(uart, digitalio.DigitalInOut(_pin(pins["dxl_dir"])))


def command_uart(config=None):
    """Create the host command UART at config["cmd_baudrate"]."""
    config = load_config(config)
    pins = load_pins(config)
    return busio.UART(
        tx=_pin(pins["cmd_tx"]),
        rx=_pin(pins["cmd_rx"]),
        baudrate=config["cmd_baudrate"],
        timeout=0.05,
        receiver_buffer_size=512,
    )
