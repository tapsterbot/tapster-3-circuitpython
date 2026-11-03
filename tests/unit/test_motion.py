"""motion.py: easing, straight-line interpolation and hop arcs."""

import pytest

import motion
from motion import EASING, hop, interpolate, steps_for_distance


@pytest.mark.parametrize("name", sorted(EASING))
def test_easing_starts_at_0_ends_at_1_and_never_reverses(name):
    fn = EASING[name]
    assert fn(0.0) == pytest.approx(0.0)
    assert fn(1.0) == pytest.approx(1.0)
    values = [fn(i / 100) for i in range(101)]
    assert values == sorted(values)


def test_interpolate_excludes_start_and_ends_on_target():
    pts = list(interpolate((0, 0, 0), (10, 20, -30), 5, "linear"))
    assert len(pts) == 5
    assert pts[0] == pytest.approx((2, 4, -6))
    assert pts[-1] == pytest.approx((10, 20, -30))


@pytest.mark.parametrize("name", sorted(EASING))
def test_interpolate_stays_on_the_line(name):
    for x, y, z in interpolate((0, 0, 0), (10, 20, -30), 12, name):
        t = x / 10
        assert (y, z) == pytest.approx((20 * t, -30 * t))


def test_unknown_easing_falls_back_to_ease_in_out():
    a = list(interpolate((0, 0, 0), (10, 0, 0), 4, "nope"))
    b = list(interpolate((0, 0, 0), (10, 0, 0), 4, "ease_in_out"))
    assert a == b


@pytest.mark.parametrize("dist,step,expected", [
    (0, 5, 5),       # min_steps
    (40, 5, 8),
    (1000, 5, 30),   # max_steps
    (40, 2, 20),
])
def test_steps_for_distance(dist, step, expected):
    assert steps_for_distance((0, 0, 0), (dist, 0, 0), step) == expected


def test_hop_lifts_first_then_arcs_to_target():
    pts = list(hop((0, 0, -10), (30, 30, -10), arc_height=20, lift=5))
    assert pts[0] == (0, 0, -5)
    assert pts[-1] == pytest.approx((30, 30, -10))
    assert max(p[2] for p in pts) == pytest.approx(-7.5 + 20)  # mid-arc


def test_distance():
    assert motion.distance((0, 0, 0), (3, 4, 12)) == 13
    assert motion.distance_xy((0, 0, 5), (3, 4, -5)) == 5
