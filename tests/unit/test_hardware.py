"""hardware.py: board detection, pin overrides and bus setup."""

import sys

import pytest

import fakes
import hardware

board = sys.modules["board"]


@pytest.fixture
def uarts(monkeypatch):
    made = []

    def factory(**kwargs):
        made.append(kwargs)
        return fakes.FakeDynamixelBus(baudrate=kwargs["baudrate"])

    monkeypatch.setattr(fakes, "_uart_factory", factory)
    return made


@pytest.fixture
def board_id(monkeypatch):
    def set_id(value):
        monkeypatch.setattr(board, "board_id", value)
    return set_id


@pytest.mark.parametrize("bid,profile", [
    ("raspberry_pi_pico", "pico"),
    ("raspberry_pi_pico_w", "pico"),
    ("espressif_esp32_devkitc_v4_wroom_32e", "esp32"),
])
def test_detects_board(board_id, bid, profile):
    board_id(bid)
    assert hardware.detect_profile() == profile


def test_unknown_board_raises(board_id):
    board_id("adafruit_feather_rp2350")
    with pytest.raises(RuntimeError, match="Unsupported board"):
        hardware.detect_profile()


def test_config_can_force_board_and_override_pins(board_id):
    board_id("raspberry_pi_pico")
    pins = hardware.load_pins({"board": "esp32", "pins": {"dxl_dir": "IO5"}})
    assert pins["board"] == "esp32"
    assert pins["dxl_dir"] == "IO5"
    assert pins["dxl_tx"] == "IO4"


def test_pico_bus_uses_uart1_and_hardware_direction(board_id, uarts):
    board_id("raspberry_pi_pico")
    dxl = hardware.dynamixel_bus(fakes.load_config())
    assert uarts[0]["tx"] == "GP4" and uarts[0]["rx"] == "GP5"
    assert uarts[0]["rs485_dir"] == "GP6"
    assert uarts[0]["baudrate"] == 3000000
    assert dxl.dir is None  # the UART drives DIR itself


def test_esp32_bus_toggles_a_gpio_direction_pin(board_id, uarts):
    board_id("espressif_esp32_devkitc_v4_wroom_32e")
    dxl = hardware.dynamixel_bus(fakes.load_config())
    assert "rs485_dir" not in uarts[0]
    assert dxl.dir.pin == "IO17"


def test_command_uart(board_id, uarts):
    board_id("raspberry_pi_pico")
    hardware.command_uart(fakes.load_config())
    assert (uarts[0]["tx"], uarts[0]["rx"], uarts[0]["baudrate"]) == ("GP0", "GP1", 115200)
