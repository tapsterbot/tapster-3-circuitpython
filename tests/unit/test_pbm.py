"""pbm.py on the real Dynamixel driver, with simulated motors."""

import pytest

from dynamixel import Dynamixel
from fakes import FakeDynamixelBus, FakeMotor, delta_motors, pbm_motors
from pbm import MoveResult, PBM

GOAL = 116


@pytest.fixture
def bus():
    return FakeDynamixelBus(delta_motors() + pbm_motors()
                            + [FakeMotor(7, model=9999)])


@pytest.fixture
def pbm(bus):
    return PBM(Dynamixel(bus), ids=(4, 5, 6, 7))


def motor(bus, mid):
    return bus.motors[mid]


# -- safety ---------------------------------------------------------------

CALLS = [
    ("ping", ()), ("led_on", ()), ("led_off", ()), ("get_led_status", ()),
    ("blink", (1,)), ("torque_on", ()), ("torque_off", ()),
    ("get_torque_status", ()), ("get_position", ()), ("get_effort", ()),
    ("get_model_number", ()), ("get_model_name", ()), ("stop", ()),
    ("move", (1500,)), ("tap", (2048, 1700)),
]


@pytest.mark.parametrize("mid", [1, 2, 3, 8])
@pytest.mark.parametrize("name,args", CALLS)
def test_only_pbm_ids_are_accepted(pbm, bus, name, args, mid):
    with pytest.raises(ValueError, match="not a PBM motor"):
        getattr(pbm, name)(mid, *args)
    assert all(p[0] != mid for p in bus.packets)


@pytest.mark.parametrize("kwargs", [
    dict(goal_position="1500"), dict(goal_position=1500.0),
    dict(goal_position=True), dict(goal_position=1500, timeout="soon"),
    dict(goal_position=1500, timeout=None),
    dict(goal_position=1500, position_tolerance="x"),
])
def test_bad_move_arguments_rejected_before_motion(pbm, bus, kwargs):
    with pytest.raises((ValueError, TypeError)):
        pbm.move(4, wait=True, **kwargs)
    assert bus.writes(address=GOAL) == []


@pytest.mark.parametrize("kwargs", [
    dict(from_position=2048, to_position="1700"),
    dict(from_position=None, to_position=1700),
    dict(from_position=2048, to_position=1700, hold="long"),
    dict(from_position=2048, to_position=1700, timeout=[1]),
])
def test_bad_tap_arguments_rejected_before_motion(pbm, bus, kwargs):
    with pytest.raises((ValueError, TypeError)):
        pbm.tap(4, **kwargs)
    assert bus.writes(address=GOAL) == []


# -- identity -------------------------------------------------------------

def test_ping(pbm, bus):
    assert pbm.ping(4) is True
    del bus.motors[6]
    assert pbm.ping(6) is False


def test_model_names(pbm):
    assert pbm.get_model_name(4) == "XC330-T181-T"
    assert pbm.get_model_name(5) == "XC430-W240-T"


def test_unsupported_model_raises(pbm):
    with pytest.raises(RuntimeError, match="unsupported model number 9999"):
        pbm.get_model_name(7)


def test_effort_units(pbm, bus):
    motor(bus, 4).effort = 210    # XC330: mA
    motor(bus, 5).effort = -125   # XC430: 0.1 %
    assert pbm.get_effort(4) == (210.0, "mA")
    assert pbm.get_effort(5) == (-12.5, "%")


# -- LED and torque -------------------------------------------------------

def test_led(pbm, bus):
    pbm.led_on(4)
    assert pbm.get_led_status(4) and motor(bus, 4).led
    pbm.led_off(4)
    assert not pbm.get_led_status(4)


def test_blink_ends_off(pbm, bus):
    pbm.blink(5, times=2, interval=0.001)
    leds = [d for mid, addr, d in bus.writes(address=65) if mid == 5]
    assert leds == [b"\x01", b"\x00", b"\x01", b"\x00"]


def test_blink_rejects_negative_times(pbm):
    with pytest.raises(ValueError):
        pbm.blink(4, times=-1)


def test_torque(pbm, bus):
    pbm.torque_on(4)
    assert pbm.get_torque_status(4) and motor(bus, 4).torque
    pbm.torque_off(4)
    assert not pbm.get_torque_status(4)


# -- moving ---------------------------------------------------------------

def test_move_without_wait_returns_at_once_and_enables_torque(pbm, bus):
    result = pbm.move(4, 1800)
    assert result.position is None
    assert motor(bus, 4).torque and motor(bus, 4).goal == 1800


def test_move_wait_reaches_goal(pbm):
    result = pbm.move(4, 1800, wait=True)
    assert abs(result.position - 1800) <= 10
    assert not result.contact
    assert result.effort_unit == "mA"


def test_move_wait_times_out_short_of_goal(pbm, bus):
    motor(bus, 4).speed = 100  # ticks/s: far too slow
    result = pbm.move(4, 1000, wait=True, timeout=0.1)
    assert not result.contact
    assert result.position > 1900


def test_stop_holds_current_position(pbm, bus):
    motor(bus, 4).speed = 1000
    pbm.move(4, 1000)
    position = pbm.stop(4)
    assert motor(bus, 4).goal == position
    assert 1000 < position < 2048


def test_out_of_range_goal_is_reported(pbm):
    with pytest.raises(RuntimeError, match="error: 0x06"):
        pbm.move(4, 5000)


# -- tapping --------------------------------------------------------------

def test_tap_with_nothing_in_the_way(pbm, bus):
    result = pbm.tap(4, from_position=2048, to_position=1700)
    assert not result.contact
    assert abs(result.position - 1700) <= 10
    assert abs(motor(bus, 4).position - 2048) <= 10  # released


def test_tap_stops_on_a_button(pbm, bus):
    motor(bus, 4).obstacle = 1850
    result = pbm.tap(4, from_position=2048, to_position=1700)
    assert result.contact
    assert 1850 <= result.position < 1870
    assert result.peak_effort is not None
    assert abs(motor(bus, 4).position - 2048) <= 10


def test_tap_without_contact_detection_pushes_until_timeout(pbm, bus):
    motor(bus, 4).obstacle = 1850
    result = pbm.tap(4, 2048, 1700, stop_on_contact=False, timeout=0.2)
    assert not result.contact
    assert result.position == 1850
    assert abs(motor(bus, 4).position - 2048) <= 10


def test_tap_releases_even_when_the_press_fails(pbm, bus):
    bus.silent_reads.add((4, 126))  # effort reads never answer
    with pytest.raises(RuntimeError, match="No valid response"):
        pbm.tap(4, from_position=2048, to_position=1700)
    assert motor(bus, 4).goal == 2048


def test_contact_needs_the_motor_to_have_moved(pbm, bus):
    # blocked from the start: never travels contact_min_travel, so no contact
    motor(bus, 4).obstacle = 2048
    result = pbm.move(4, 1700, stop_on_contact=True, timeout=0.2)
    assert not result.contact


def test_move_result():
    r = MoveResult(1852, True, 210.0, 403.0, "mA")
    assert r.to_dict() == {"position": 1852, "contact": True,
                           "final_effort": 210.0, "peak_effort": 403.0,
                           "effort_unit": "mA"}
    assert "contact=True" in repr(r)
