"""End to end on a real robot: draws a small square with straight moves, then
again with hops, taps once at home, then taps each PBM motor.

MOVES THE ROBOT. Clear the area under the tip and around the PBM first.
Run with `make test-hw` (or `make test-hw PORT=/dev/cu.usbmodem2101`).

Settings (environment variables):
    TAPSTER_PORT       serial port (auto-detected if unset)
    SQUARE_SIZE        side of the square in mm (default 20)
    PBM_IDS            PBM motors to tap (default 4,5,6)
    PBM_FROM_POSITION  resting position (default 2000)
    PBM_TO_POSITION    pressed position (default 1000)
    PBM_PARK           where to leave motors after their tap, as id:position
                       pairs (default 5:1500); others stay at the resting
                       position
"""

import os

import pytest

from tapster import Tapster

pytestmark = pytest.mark.hardware

SQUARE = float(os.environ.get("SQUARE_SIZE", 20))
PBM_IDS = [int(i) for i in os.environ.get("PBM_IDS", "4,5,6").split(",")]
FROM = int(os.environ.get("PBM_FROM_POSITION", 2000))
TO = int(os.environ.get("PBM_TO_POSITION", 1000))
PARK = {int(k): int(v) for k, v in
        (pair.split(":") for pair in
         os.environ.get("PBM_PARK", "5:1500").split(",") if pair)}


@pytest.fixture(scope="module")
def t():
    robot = Tapster(port=os.environ.get("TAPSTER_PORT") or None)
    yield robot
    robot.home(speed=50)
    robot.close()


def test_delta_motors_answer(t):
    resp = t.ping()
    assert resp["ok"], resp
    assert len(resp["motors"]) == 3


def square_corners():
    h = SQUARE / 2
    return [(-h, h), (h, h), (h, -h), (-h, -h), (-h, h)]


def test_draw_square(t):
    z = t.safety_height  # in the air, above any screen
    assert t.home(speed=50)["ok"]
    for x, y in square_corners():
        resp = t.move(x=x, y=y, z=z, speed=30)
        assert resp == {"ok": True, "target": {"x": x, "y": y, "z": z}}, resp
        assert t.position == (x, y, z)
    assert t.home(speed=50)["ok"]


def test_hop_square(t):
    # the same square, hopping between corners with the robot's default arc
    z = t.safety_height
    assert t.home(speed=50)["ok"]
    for x, y in square_corners():
        resp = t.hop(x=x, y=y, z=z)
        assert resp == {"ok": True, "target": {"x": x, "y": y, "z": z}}, resp
        assert t.position == (x, y, z)
    assert t.home(speed=50)["ok"]


def test_tap(t):
    # presses down to surface_height at home, then back up to safety_height
    surface, safety = t.surface_height, t.safety_height
    resp = t.tap()
    assert resp == {"ok": True, "tap": {"x": 0, "y": 0, "z": surface}}, resp
    assert t.position == (0, 0, safety)


def test_pbm_motors_answer(t):
    for mid in PBM_IDS:
        assert t.pbm.ping(mid), f"PBM motor {mid} did not answer"


@pytest.mark.parametrize("mid", PBM_IDS)
def test_pbm_tap(t, mid):
    try:
        result = t.pbm.tap(mid, from_position=FROM, to_position=TO)
        # with nothing in the way it reaches TO; on a button it stops early
        assert result.contact or abs(result.position - TO) <= 10, result
        assert abs(t.pbm.get_position(mid) - FROM) <= 10  # released
    finally:
        if mid in PARK:  # even if the tap failed
            t.pbm.move(mid, PARK[mid], wait=True)
    if mid in PARK:
        assert abs(t.pbm.get_position(mid) - PARK[mid]) <= 10


def test_pbm_refuses_delta_motors(t):
    with pytest.raises(RuntimeError, match="not a PBM motor"):
        t.pbm.move(1, FROM)
