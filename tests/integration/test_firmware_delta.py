"""Delta commands, end to end: tapster.py -> serial -> code.py -> bus -> motors."""

import json

import pytest

from fakes import ROBOTS


def approx(*xyz, tol=0.3):
    return pytest.approx(xyz, abs=tol)


def raw_command(robot, line):
    """Send a raw line and return the first JSON reply."""
    robot.t.ser.write(line + b"\n")
    for _ in range(20):
        reply = robot.t.ser.readline().decode().strip()
        if reply.startswith("{"):
            return json.loads(reply)
    raise AssertionError("no reply")


# -- boot -----------------------------------------------------------------

def test_boot_homes_and_powers_only_the_delta(robot):
    assert robot.xyz() == approx(0, 0, 0)
    for mid in (1, 2, 3):
        assert robot.motor(mid).torque == 1 and robot.motor(mid).led == 1
    for mid in (4, 5, 6):
        assert robot.motor(mid).torque == 0
        assert robot.motor(mid).goal == 2048
    assert "Tapster ready." in robot.fw.text
    assert "PBM motor 4: XC330-T181-T" in robot.fw.text


def test_esp32_board(make_robot):
    r = make_robot(board_id="espressif_esp32_devkitc_v4_wroom_32e")
    assert "rs485_dir" not in r.fw.uart_kwargs[0]  # GPIO direction pin instead
    assert r.t.tap()["ok"]
    assert r.t.pbm.tap(5, 2048, 1700).position is not None


# -- transports -----------------------------------------------------------

@pytest.mark.parametrize("transport", ["usb", "uart"])
def test_ping(make_robot, transport):
    r = make_robot(transport=transport)
    resp = r.t.ping()
    assert resp["ok"]
    assert [m["name"] for m in resp["motors"]] == ["XC430-W240"] * 3


@pytest.mark.parametrize("transport", ["usb", "uart"])
def test_tap(make_robot, transport):
    r = make_robot(transport=transport)
    assert r.t.tap(x=5, y=5) == {"ok": True, "tap": {"x": 5, "y": 5, "z": -10}}


# -- moving ---------------------------------------------------------------

@pytest.mark.parametrize("model", sorted(ROBOTS))
def test_move_ends_at_target(make_robot, model):
    r = make_robot(robot=model)
    resp = r.t.move(x=20, y=-10, z=-5)
    assert resp == {"ok": True, "target": {"x": 20, "y": -10, "z": -5}}
    assert r.xyz() == approx(20, -10, -5)
    assert r.t.position == (20, -10, -5)


def test_move_is_a_straight_line(robot):
    mark = robot.mark()
    robot.t.move(x=40, y=20, z=-20, easing="ease_in_out_cubic", step_size=2)
    path = robot.path(mark)
    assert len(path) >= 10
    for x, y, z in path:
        t = x / 40
        assert (y, z) == pytest.approx((20 * t, -20 * t), abs=0.5)


def test_draw_square(robot):
    corners = [(-10, 10), (10, 10), (10, -10), (-10, -10), (-10, 10)]
    robot.t.move(x=-10, y=10, z=0)
    mark = robot.mark()
    for x, y in corners[1:]:
        assert robot.t.move(x=x, y=y, z=0, speed=50)["ok"]
        assert robot.xyz() == approx(x, y, 0)
    for x, y, z in robot.path(mark):  # every waypoint is on an edge
        assert z == pytest.approx(0, abs=0.3)
        assert min(abs(abs(x) - 10), abs(abs(y) - 10)) < 0.3


def test_unreachable_move_is_refused_without_moving(robot):
    mark = robot.mark()
    error = robot.t.move(x=999, y=999)["error"]
    assert error.startswith(("Unreachable", "Out of reach")), error
    assert robot.bus.sync_goals[mark:] == []
    assert robot.xyz() == approx(0, 0, 0)
    assert robot.t.position == (0, 0, 0)


def test_move_after_a_refused_one_starts_where_the_arm_is(robot):
    robot.t.move(x=999, y=999)
    mark = robot.mark()
    assert robot.t.move(x=10, y=0)["ok"]
    path = robot.path(mark)
    assert path[0] == approx(2, 0, 0, tol=1)  # first 1/5 of the way, no jump
    assert path[-1] == approx(10, 0, 0)


# On the Tapster 3, (50, -50, -50) can be solved but needs motor 1 at
# position ~3139, past motor_max (3072). Clamping it would put the tip
# somewhere else, so it's refused.
PAST_MOTOR_MAX = dict(x=50, y=-50, z=-50)


def test_move_past_motor_limits_is_refused_without_moving(make_robot):
    r = make_robot(robot="tapster3")
    mark = r.mark()
    error = r.t.move(**PAST_MOTOR_MAX)["error"]
    assert "Out of reach" in error and "motor_min-motor_max (1024-3072)" in error
    assert r.bus.sync_goals[mark:] == []
    assert r.t.position == (0, 0, 0)


def test_hop_past_motor_limits_is_refused_without_moving(make_robot):
    r = make_robot(robot="tapster3")
    mark = r.mark()
    assert "Out of reach" in r.t.hop(arc_height=5, **PAST_MOTOR_MAX)["error"]
    assert r.bus.sync_goals[mark:] == []


def test_move_to_the_edge_of_the_area_still_works(make_robot):
    r = make_robot(robot="tapster3")
    assert r.t.move(x=50, y=-50, z=-30)["ok"]
    assert r.xyz() == approx(50, -50, -30)


def test_home(robot):
    robot.t.move(x=20, y=20)
    assert robot.t.home()["ok"]
    assert robot.xyz() == approx(0, 0, 0)


# -- tapping --------------------------------------------------------------

def test_tap_presses_to_surface_and_returns(robot):
    robot.t.move(x=10, y=10, z=0)
    mark = robot.mark()
    assert robot.t.tap()["tap"] == {"x": 10, "y": 10, "z": -10}
    path = robot.path(mark)
    assert min(z for _, _, z in path) == pytest.approx(-10, abs=0.3)
    assert path[-1] == approx(10, 10, 0)


def test_side_push_never_goes_below_safety(robot):
    mark = robot.mark()
    robot.t.tap(dx=-5, dz=0)
    path = robot.path(mark)
    assert min(z for _, _, z in path) == pytest.approx(0, abs=0.3)
    assert min(x for x, _, _ in path) == pytest.approx(-5, abs=0.3)
    assert path[-1] == approx(0, 0, 0)


def test_tap_refused_when_surface_is_not_below_safety(robot):
    robot.t.surface_height = 5
    assert "surface_height must be below" in robot.t.tap()["error"]
    assert robot.t.tap(dz=-3)["tap"]["z"] == -3


# -- hop ------------------------------------------------------------------

def hop_path_length(start, end, **kwargs):
    from motion import distance, hop
    length, here = 0.0, start
    for w in hop(start, end, **kwargs):
        length, here = length + distance(here, w), w
    return length


def test_hop_arcs_over_and_lands(make_robot):
    r = make_robot(robot="tapster3")
    mark = r.mark()
    assert r.t.hop(x=30, y=30)["ok"]
    path = r.path(mark)
    assert path[0] == approx(0, 0, 5)  # lifts straight up first
    assert max(z for _, _, z in path) > 25
    assert path[-1] == approx(30, 30, 0)
    assert r.t.position == (30, 30, 0)


def test_hop_lift_is_used(make_robot):
    r = make_robot(robot="tapster3")
    mark = r.mark()
    r.t.hop(x=10, y=0, lift=12, arc_height=5)
    assert r.path(mark)[0] == approx(0, 0, 12)


@pytest.mark.parametrize("speed", [20, 80])
def test_hop_speed_sets_how_long_it_takes(make_robot, speed):
    r = make_robot(robot="tapster3", config={"max_speed": 1000})
    expected = hop_path_length((0, 0, 0), (30, 30, 0), arc_height=30) / (10 * speed)
    start = r.bus.sync_times and r.bus.sync_times[-1]
    mark = r.mark()
    r.t.hop(x=30, y=30, speed=speed)
    # first waypoint is sent at t=0, the last one a segment before the end
    times = r.bus.sync_times[mark:]
    took = times[-1] - times[0]
    assert expected * 0.6 < took < expected * 1.1


def test_hop_sets_each_motors_velocity_to_arrive_together(make_robot):
    r = make_robot(robot="tapster3", config={"max_speed": 100})
    mark = r.mark()
    r.t.hop(x=20, y=0, speed=50, arc_height=10)  # 50 mm/s
    goals = r.bus.sync_goals[mark - 1:]  # starting from home
    path = r.path(mark - 1)
    from motion import distance
    for i, vel in enumerate(r.bus.sync_velocities[mark:]):
        seconds = distance(path[i], path[i + 1]) / 50
        for m in (1, 2, 3):
            ticks = abs(goals[i + 1][m] - goals[i][m])
            arrives = ticks / (vel[m] * 0.229 / 60 * 4096)
            assert arrives <= seconds * 1.02          # never late
            assert arrives > seconds * 0.8 or ticks < 30  # nor much early
        assert max(vel.values()) < 1023


def test_unreachable_hop_is_refused_before_moving(make_robot):
    r = make_robot(robot="tapster3plus")
    mark = r.mark()
    assert "Unreachable" in r.t.hop(x=30, y=30)["error"]  # 30 mm arc: too high
    assert r.bus.sync_goals[mark:] == []
    assert r.t.position == (0, 0, 0)


def test_low_hop_on_tapster3plus(make_robot):
    r = make_robot(robot="tapster3plus")
    assert r.t.hop(x=30, y=30, arc_height=10)["ok"]
    assert r.xyz() == approx(30, 30, 0)


@pytest.mark.xfail(strict=True, reason=(
    "Tapster 3+ config reaches only ~18 mm above home, so the default "
    "30 mm hop arc from z=0 is out of reach (it's refused)"))
def test_default_hop_on_tapster3plus(make_robot):
    r = make_robot(robot="tapster3plus")
    assert r.t.hop(x=30, y=30)["ok"]


# -- heights and probing --------------------------------------------------

def test_surface_heights_are_saved(robot):
    robot.t.send({"cmd": "surface", "surface_height": -12, "safety_height": -2})
    resp = robot.t.send({"cmd": "surface"})
    assert resp == {"ok": True, "surface_height": -12, "safety_height": -2,
                    "saved": True}
    saved = json.loads(robot.fw.config_path.read_text())
    assert (saved["surface_height"], saved["safety_height"]) == (-12, -2)


def test_probe_finds_the_surface(robot):
    def load(motor):  # 10 % once the tip is at or below z = -23
        return 100 if robot.xyz()[2] <= -22.5 else 0
    for mid in (1, 2, 3):
        robot.motor(mid).effort_fn = load
    assert robot.t.probe(threshold=5) == -22
    assert (robot.t.surface_height, robot.t.safety_height) == (-22, -12)
    assert robot.xyz() == approx(0, 0, 0)


def test_probe_gives_up_at_max_depth(robot):
    assert robot.t.probe(max_depth=5) is None
    assert robot.xyz() == approx(0, 0, 0)


# -- motors ---------------------------------------------------------------

def test_load_and_temperature(robot):
    robot.motor(2).effort = 125
    assert robot.t.get_load() == [0.0, 12.5, 0.0]
    assert robot.t.get_temperature() == [36, 36, 36]


def test_disable_and_enable_touch_only_the_delta(robot):
    robot.t.pbm.torque_on(4)
    robot.t.disable()
    assert [robot.motor(m).torque for m in (1, 2, 3, 4)] == [0, 0, 0, 1]
    robot.t.enable()
    assert [robot.motor(m).torque for m in (1, 2, 3, 4)] == [1, 1, 1, 1]


def test_blink(robot):
    assert robot.t.blink(2, count=1)["ok"]
    assert robot.motor(2).led == 0


def test_bad_json_and_unknown_commands(robot):
    assert raw_command(robot, b"not json") == {"error": "invalid json"}
    assert raw_command(robot, b'{"cmd":"dance"}') == {"error": "unknown cmd: dance"}
    assert robot.t.status()["ok"]


def test_slow_hop_reply_is_not_lost(make_robot):
    # 5 % of 400 mm/s = 20 mm/s: this hop takes ~3.5 s, longer than the
    # client's usual 3 s reply window
    r = make_robot(robot="tapster3", config={"max_speed": 400})
    assert r.t.hop(x=20, y=0, arc_height=20, speed=5)["ok"]
    assert r.t.status()["xyz"] == {"x": 20, "y": 0, "z": 0}
