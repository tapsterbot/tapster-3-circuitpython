"""Fixtures: the real firmware (code.py's main loop) on simulated motors,
driven by the real tapster.py client over a fake serial port."""

import time
import types
from dataclasses import dataclass

import pytest

import tapster
from fakes import Firmware, FakeDynamixelBus, delta_motors, delta_xyz, pbm_motors


@dataclass
class Robot:
    t: tapster.Tapster
    fw: Firmware
    bus: FakeDynamixelBus

    def motor(self, mid):
        return self.bus.motors[mid]

    def xyz(self):
        """Where the delta is now, from its motors' goal positions."""
        goals = {mid: self.motor(mid).goal for mid in self.fw.config["motor_ids"]}
        return delta_xyz(goals, self.fw.config)

    def path(self, since=0):
        """(x, y, z) of every delta Sync Write since a mark."""
        return [delta_xyz(g, self.fw.config) for g in self.bus.sync_goals[since:]]

    def mark(self):
        return len(self.bus.sync_goals)


@pytest.fixture
def make_robot(tmp_path, monkeypatch):
    """make_robot(motors=None, transport="usb", setup=None, **Firmware kwargs)
    -> Robot. setup(bus) runs before the firmware boots.

    Afterwards, checks the firmware is still running and sent only valid
    packets: no test may crash the main loop.
    """
    robots = []

    def make(motors=None, transport="usb", setup=None, **kwargs):
        if motors is None:
            motors = delta_motors() + pbm_motors()
        bus = FakeDynamixelBus(motors)
        if setup is not None:
            setup(bus)
        fw = Firmware(bus, tmp_path / f"board{len(robots)}", **kwargs)
        robots.append(fw)
        fw.start()
        monkeypatch.setattr(tapster.serial, "Serial",
                            lambda port, baud, timeout=None: fw.serial(transport, timeout))
        # skip the client's 0.5 s settle delay; leave time.time alone
        monkeypatch.setattr(tapster, "time",
                            types.SimpleNamespace(time=time.time, sleep=lambda s: None))
        t = tapster.Tapster(port=f"fake-{transport}")
        return Robot(t, fw, bus)

    yield make

    for fw in robots:
        try:
            assert fw.error is None, fw.error
            assert fw.alive, "firmware main loop exited"
            assert fw.bus.errors == [], fw.bus.errors
        finally:
            fw.stop()


@pytest.fixture
def robot(make_robot):
    return make_robot()
