"""dynamixel.py: packets, CRC, retries and the per-model effort readings."""

import os

import pytest

import dynamixel
from dynamixel import Dynamixel, INST_PING, _crc16
from fakes import (BROADCAST, SYNC_WRITE, DigitalInOut, FakeDynamixelBus,
                   FakeMotor, crc16)


@pytest.fixture
def bus():
    return FakeDynamixelBus([FakeMotor(1), FakeMotor(2, model=1210),
                             FakeMotor(3, model=1220)])


@pytest.fixture
def dxl(bus):
    return Dynamixel(bus)


def test_ping_packet_matches_robotis_example():
    # Protocol 2.0 e-manual: ping ID 1 is FF FF FD 00 01 03 00 01 19 4E
    pkt = Dynamixel(None)._build_packet(1, INST_PING)
    assert bytes(pkt) == bytes.fromhex("FFFFFD00010300 01 194E".replace(" ", ""))


def test_crc_table_matches_bitwise_crc():
    for n in (0, 1, 7, 64, 255):
        data = os.urandom(n)
        assert _crc16(data) == crc16(data)


def test_ping_reports_model_name_and_firmware(dxl):
    assert dxl.ping(1) == {"model": 1080, "name": "XC430-W240", "firmware": 50}
    assert dxl.ping(3)["name"] == "XC330-T288"


def test_ping_caches_model(dxl, bus):
    dxl.ping(2)
    sent = len(bus.packets)
    assert dxl.get_model(2) == 1210
    assert len(bus.packets) == sent


def test_write_then_read_back(dxl, bus):
    dxl.write_u32(1, 116, 3000)
    assert bus.motors[1].goal == 3000
    assert dxl.read_u32(1, 116) == 3000
    dxl.set_led(1, True)
    assert dxl.read_u8(1, 65) == 1


def test_signed_reads(dxl, bus):
    m = bus.motors[1]
    m.effort = -150
    assert dxl.read_s16(1, 126) == -150
    m._anchor = (0, -500, -500)  # e.g. extended position mode
    assert dxl.get_position(1) == -500


def test_sync_move_is_one_broadcast_packet(dxl, bus):
    dxl.sync_move([(1, 100, 1500), (2, 200, 2500), (3, 300, 3500)])
    mid, inst, params = bus.packets[-1]
    assert (mid, inst) == (BROADCAST, SYNC_WRITE)
    assert params[:4] == bytes([112, 0, 8, 0])  # Profile Velocity + Goal, 8 bytes
    assert [m.goal for m in bus.motors.values()] == [1500, 2500, 3500]
    assert [m.profile_velocity for m in bus.motors.values()] == [100, 200, 300]
    assert bus.sync_goals[-1] == {1: 1500, 2: 2500, 3: 3500}


def test_missing_motor_raises_after_three_attempts(dxl, bus):
    with pytest.raises(RuntimeError, match="No valid response from motor 9"):
        dxl.ping(9)
    assert [p[0] for p in bus.packets] == [9, 9, 9]


def test_lost_reply_is_retried(dxl, bus):
    bus.drop_replies = 1
    assert dxl.read_u8(1, 146) == 36
    assert len(bus.packets) == 2


def test_noise_before_reply_is_skipped(dxl, bus):
    bus.noise = b"\x00\xff\x13"
    assert dxl.get_temperature(1) == 36


def test_error_status_raises(dxl):
    with pytest.raises(RuntimeError, match="Motor 1 error: 0x06"):
        dxl.write_u32(1, 116, 5000)  # beyond Max Position Limit


def test_get_load_xc430_is_present_load_percent(dxl, bus):
    bus.motors[1].effort = 125  # 0.1 % units
    assert dxl.get_load(1) == 12.5


def test_get_load_xc330_is_percent_of_current_limit(dxl, bus):
    bus.motors[2].effort = 455  # mA, limit 910
    assert dxl.get_load(2) == 50.0
    assert dxl.get_current(2) == 455


def test_get_current_needs_an_xc330(dxl):
    with pytest.raises(RuntimeError, match="no current sensing"):
        dxl.get_current(1)


def test_gpio_direction_pin_brackets_each_write(bus):
    pin = DigitalInOut("IO17")
    dxl = Dynamixel(bus, pin)
    assert pin.history == [False]  # RX by default
    dxl.ping(1)
    assert pin.history == [False, True, False]


def test_model_names_cover_delta_motors():
    assert set(dynamixel.MODEL_NAMES) >= {1080, 1210, 1220}
