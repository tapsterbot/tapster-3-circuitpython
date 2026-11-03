"""PBM commands, end to end: tapster.py -> serial -> code.py -> pbm.py -> bus."""

import pytest

from fakes import FakeMotor, delta_motors

GOAL = 116


def released(robot, mid, position=2048):
    return abs(robot.motor(mid).position - position) <= 10


def test_ping_reports_each_motor(make_robot):
    r = make_robot(motors=delta_motors() + [FakeMotor(4, model=1210),
                                            FakeMotor(5, model=1080)])
    assert r.t.pbm.ping() == [
        {"id": 4, "found": True, "name": "XC330-T181-T"},
        {"id": 5, "found": True, "name": "XC430-W240-T"},
        {"id": 6, "found": False},
    ]
    assert r.t.pbm.ping(4) is True and r.t.pbm.ping(6) is False
    assert "PBM motor 6: not found" in r.fw.text


@pytest.mark.parametrize("transport", ["usb", "uart"])
def test_tap_stops_on_the_button(make_robot, transport):
    r = make_robot(transport=transport)
    r.motor(4).obstacle = 1850
    result = r.t.pbm.tap(4, from_position=2048, to_position=1700)
    assert result.contact
    assert 1850 <= result.position < 1870
    assert result.effort_unit == "mA"
    assert released(r, 4)


def test_tap_with_nothing_in_the_way(robot):
    result = robot.t.pbm.tap(5, from_position=2000, to_position=1000)
    assert not result.contact
    assert abs(result.position - 1000) <= 10
    assert result.effort_unit == "%"
    assert released(robot, 5, 2000)


def test_move_and_position(robot):
    assert robot.t.pbm.move(6, 1500).position is None
    result = robot.t.pbm.move(6, 1600, wait=True)
    assert abs(result.position - 1600) <= 10
    assert abs(robot.t.pbm.get_position(6) - 1600) <= 10


def test_stop(robot):
    robot.motor(4).speed = 500
    robot.t.pbm.move(4, 1000)
    held = robot.t.pbm.stop(4)
    assert robot.motor(4).goal == held and 1000 < held < 2048


def test_led_torque_and_effort(robot):
    robot.t.pbm.led_on(4)
    assert robot.t.pbm.get_led_status(4) and robot.motor(4).led
    robot.t.pbm.led_off(4)
    assert not robot.t.pbm.get_led_status(4)
    robot.t.pbm.torque_on(5)
    assert robot.t.pbm.get_torque_status(5)
    robot.t.pbm.torque_off(5)
    assert robot.motor(5).torque == 0
    robot.motor(4).effort = 210
    robot.motor(5).effort = 33
    assert robot.t.pbm.get_effort(4) == (210.0, "mA")
    assert robot.t.pbm.get_effort(5) == (pytest.approx(3.3), "%")


def test_blink(robot):
    robot.t.pbm.blink(6, times=2, interval=0.01)
    assert robot.motor(6).led == 0


def test_missing_motor_is_an_error(make_robot):
    r = make_robot(motors=delta_motors() + [FakeMotor(4)])
    with pytest.raises(RuntimeError, match="No valid response from motor 6"):
        r.t.pbm.tap(6, 2048, 1700)


def test_lost_reply_is_retried(robot):
    robot.bus.drop_replies = 1
    assert robot.t.pbm.get_position(4) == 2048


# -- do no harm -----------------------------------------------------------

@pytest.mark.parametrize("mid", [1, 2, 3])
def test_delta_motors_are_refused(robot, mid):
    since = len(robot.bus.packets)
    goals = robot.bus.sync_goals[-1]
    with pytest.raises(RuntimeError, match="not a PBM motor"):
        robot.t.pbm.tap(mid, 2048, 1700)
    with pytest.raises(RuntimeError, match="not a PBM motor"):
        robot.t.pbm.torque_off(mid)
    assert robot.bus.packets[since:] == []
    assert robot.motor(mid).goal == goals[mid] and robot.motor(mid).torque


BAD = [
    {"cmd": "pbm_tap", "id": 4, "from_position": 2048, "to_position": "abc"},
    {"cmd": "pbm_tap", "id": 4, "from_position": 2048, "to_position": 1700, "hold": "x"},
    {"cmd": "pbm_tap", "id": 4, "from_position": 2048},
    {"cmd": "pbm_move", "id": 4, "position": 1.5},
    {"cmd": "pbm_move", "id": 4, "position": True},
    {"cmd": "pbm_move", "id": 4, "position": 1700, "timeout": "soon", "wait": True},
    {"cmd": "pbm_move", "id": "4", "position": 1700},
    {"cmd": "pbm_move", "position": 1700},
    {"cmd": "pbm_blink", "id": 4, "times": "x"},
    {"cmd": "pbm_blink", "id": 4, "interval": "x"},
    {"cmd": "pbm_dance", "id": 4},
]


@pytest.mark.parametrize("msg", BAD, ids=[str(i) for i in range(len(BAD))])
def test_bad_input_is_answered_without_moving(robot, msg):
    since = len(robot.bus.packets)
    resp = robot.t.send(msg)
    assert "error" in resp
    assert robot.bus.writes(since, address=GOAL) == []
    assert robot.t.status()["ok"]  # main loop still running


def test_out_of_range_position_is_reported(robot):
    with pytest.raises(RuntimeError, match="error: 0x06"):
        robot.t.pbm.move(4, 5000)


def test_pbm_ids_come_from_config(make_robot):
    r = make_robot(config={"pbm_ids": [5]})
    with pytest.raises(RuntimeError, match=r"not a PBM motor \(PBM ids: \[5\]\)"):
        r.t.pbm.move(4, 1700)
    assert [m["id"] for m in r.t.pbm.ping()] == [5]


def test_robot_works_without_a_pbm(make_robot):
    r = make_robot(motors=delta_motors())
    assert r.t.tap()["ok"]
    assert all(not m["found"] for m in r.t.pbm.ping())


def test_robot_works_without_pbm_py(make_robot):
    r = make_robot(without={"pbm"})
    assert "PBM support not loaded" in r.fw.text
    assert r.t.tap()["ok"]
    with pytest.raises(RuntimeError, match="PBM support not loaded"):
        r.t.pbm.ping()


def test_garbled_reply_at_boot_still_homes(make_robot):
    r = make_robot(setup=lambda bus: bus.short_ping.add(5))
    assert "PBM motor 5: " in r.fw.text and r.fw.BOOTED in r.fw.text
    assert r.t.tap()["ok"]
