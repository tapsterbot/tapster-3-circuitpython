"""tapster.py: reply parsing and the PBM client, against a scripted port."""

import json

import pytest

import tapster
from tapster import MoveResult, PBM


class ScriptedSerial:
    """Answers each command line with scripted reply lines."""

    def __init__(self, replies):
        self.replies = replies  # cmd -> list of lines, or callable(msg)
        self.sent = []
        self._rx = bytearray()
        self.timeout = 0.5

    def write(self, data):
        for line in data.decode().splitlines():
            msg = json.loads(line)
            self.sent.append(msg)
            reply = self.replies[msg["cmd"]]
            lines = reply(msg) if callable(reply) else reply
            for out in lines:
                self._rx += (out + "\r\n").encode()

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return len(self._rx)

    def read(self, n=1):
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def readline(self):
        i = self._rx.find(b"\n")
        if i < 0:
            return self.read(len(self._rx))
        return self.read(i + 1)

    def reset_input_buffer(self):
        self._rx.clear()

    def close(self):
        pass


@pytest.fixture
def client(monkeypatch):
    def make(replies):
        port = ScriptedSerial(replies)
        monkeypatch.setattr(tapster.serial, "Serial", lambda *a, **k: port)
        monkeypatch.setattr(tapster.time, "sleep", lambda s: None)
        t = tapster.Tapster(port="scripted")
        return t, port
    return make


def test_send_skips_log_lines(client):
    t, _ = client({"status": ["Tapster ready.", "Motor 1: XC430",
                              '{"ok": true, "xyz": {"x": 1, "y": 2, "z": 3}}']})
    assert t.position == (1, 2, 3)


def test_motion_methods_return_the_reply(client):
    t, port = client({"move": ['{"ok": true, "target": {"x": 5, "y": 0, "z": 0}}'],
                      "tap": ['{"error": "nope"}']})
    assert t.move(x=5)["ok"]
    assert port.sent[-1] == {"cmd": "move", "x": 5, "y": 0, "z": 0, "speed": 100,
                             "easing": "linear", "step_size": 5.0}
    assert t.tap() == {"error": "nope"}


def test_hop_leaves_arc_height_to_the_robot_unless_given(client):
    t, port = client({"hop": ['{"ok": true}']})
    t.hop(x=10)
    assert "arc_height" not in port.sent[-1]
    t.hop(x=10, arc_height=12)
    assert port.sent[-1]["arc_height"] == 12


def test_probe_prints_progress_and_returns_height(client, capsys):
    t, _ = client({"probe": ["z=-1.0  load=0,0,0  avg=0.0",
                             '{"ok": true, "surface_height": -22}']})
    assert t.probe() == -22
    assert "z=-1.0" in capsys.readouterr().out


def test_pbm_is_attached(client):
    t, _ = client({})
    assert isinstance(t.pbm, PBM)


TAP_REPLY = ('{"ok": true, "id": 4, "position": 1852, "contact": true, '
             '"final_effort": 210.0, "peak_effort": 403.0, "effort_unit": "mA"}')


def test_pbm_tap(client):
    t, port = client({"pbm_tap": [TAP_REPLY]})
    result = t.pbm.tap(4, from_position=2048, to_position=1700, hold=0.5)
    assert result == MoveResult(1852, True, 210.0, 403.0, "mA")
    assert port.sent[-1] == {"cmd": "pbm_tap", "id": 4, "from_position": 2048,
                             "to_position": 1700, "stop_on_contact": True,
                             "hold": 0.5, "timeout": 1.0}


def test_pbm_move(client):
    t, port = client({"pbm_move": [TAP_REPLY.replace("true, \"final", "false, \"final")]})
    assert t.pbm.move(4, 1700, wait=True).position == 1852
    assert port.sent[-1]["wait"] is True


def test_pbm_ping(client):
    t, port = client({"pbm_ping": lambda m: [
        '{"ok": true, "id": 4, "found": true, "name": "XC330-T181-T"}' if "id" in m
        else '{"ok": true, "motors": [{"id": 4, "found": true}]}']})
    assert t.pbm.ping(4) is True
    assert t.pbm.get_model_name(4) == "XC330-T181-T"
    assert t.pbm.ping() == [{"id": 4, "found": True}]
    assert "id" not in port.sent[-1]


def test_pbm_simple_reads(client):
    t, _ = client({
        "pbm_position": ['{"ok": true, "id": 4, "position": 2000}'],
        "pbm_effort": ['{"ok": true, "id": 4, "effort": 1.5, "unit": "%"}'],
        "pbm_torque": ['{"ok": true, "id": 4, "on": true}'],
        "pbm_led": ['{"ok": true, "id": 4, "on": false}'],
    })
    assert t.pbm.get_position(4) == 2000
    assert t.pbm.get_effort(4) == (1.5, "%")
    assert t.pbm.get_torque_status(4) is True
    assert t.pbm.get_led_status(4) is False


def test_pbm_error_raises(client):
    t, _ = client({"pbm_move": ['{"error": "motor 1 is not a PBM motor"}']})
    with pytest.raises(RuntimeError, match="pbm_move: motor 1 is not a PBM motor"):
        t.pbm.move(1, 2000)


def test_pbm_no_reply_raises(client):
    t, _ = client({"pbm_stop": []})
    t.ser.timeout = 0.01
    with pytest.raises(RuntimeError, match="no reply"):
        t.pbm.stop(4)


def test_find_port_raises_when_nothing_is_connected(monkeypatch):
    monkeypatch.setattr(tapster.glob, "glob", lambda pattern: [])
    with pytest.raises(RuntimeError, match="No serial port found"):
        tapster.find_port()
