"""kinematics.py: inverse and forward kinematics agree, for both robots."""

import itertools

import pytest

from fakes import ROBOTS, load_config
from kinematics import DeltaKinematics


@pytest.fixture(params=sorted(ROBOTS))
def config(request):
    return load_config(request.param)


def kinematics(config):
    return DeltaKinematics(f=config["f"], e=config["e"], rf=config["rf"],
                           re=config["re"])


def raw(config, x, y, z):
    ox, oy, oz = config["origin"]
    return (x + ox, y + oy, z + oz)


# the README's usable area, +/-50 mm in X and Y, down to 30 mm below home
# (on the Tapster 3 the far corners 50 mm down need positions past motor_max)
AREA = list(itertools.product((-50, -25, 0, 25, 50), (-50, 0, 50), (0, -10, -30)))


@pytest.mark.parametrize("x,y,z", AREA)
def test_forward_undoes_inverse(config, x, y, z):
    dk = kinematics(config)
    angles = dk.inverse(*raw(config, x, y, z))
    assert dk.forward(*angles) == pytest.approx(raw(config, x, y, z), abs=1e-6)


@pytest.mark.parametrize("x,y,z", AREA)
def test_usable_area_is_within_motor_limits(config, x, y, z):
    for i, angle in enumerate(kinematics(config).inverse(*raw(config, x, y, z))):
        ticks = angle * 4096 / 360
        pos = config["motor_offsets"][i] + config["motor_direction"][i] * ticks
        assert config["motor_min"] <= pos <= config["motor_max"]


def test_home_is_symmetric(config):
    t1, t2, t3 = kinematics(config).inverse(*raw(config, 0, 0, 0))
    assert t1 == pytest.approx(t2) == pytest.approx(t3)


@pytest.mark.parametrize("x,y,z", [(500, 0, 0), (0, -500, 0), (0, 0, -1000)])
def test_unreachable_raises(config, x, y, z):
    with pytest.raises(ValueError):
        kinematics(config).inverse(*raw(config, x, y, z))
