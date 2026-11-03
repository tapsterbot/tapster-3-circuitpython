"""Tapster client — send commands to the robot (ESP32 or Pico) over USB serial."""

import atexit
import glob
import json
import serial
import sys
import time
from dataclasses import dataclass
from typing import Optional


def find_port():
    """Auto-detect the robot's serial port."""
    patterns = [
        "/dev/cu.usbserial-*",      # macOS FTDI/CP2102/CH340
        "/dev/cu.usbmodem*",         # macOS native USB
        "/dev/cu.SLAB_USBtoUART*",   # macOS CP2104
        "/dev/ttyUSB*",              # Linux FTDI/CP2102/CH340
        "/dev/ttyACM*",              # Linux native USB
    ]
    for pattern in patterns:
        ports = sorted(glob.glob(pattern))
        if ports:
            return ports[0]
    raise RuntimeError(
        "No serial port found. Is the board connected?"
    )


@dataclass
class MoveResult:
    """Result of PBM.move() or PBM.tap(), as in valet-pbm."""
    position: Optional[int]
    contact: bool = False
    final_effort: Optional[float] = None
    peak_effort: Optional[float] = None
    effort_unit: Optional[str] = None


class PBM:
    """Push Button Module: the side-button actuators (Dynamixel IDs 4, 5, 6
    by default), driven through the robot's firmware.

    Same method names as valet-pbm (https://github.com/tapsterbot/valet-pbm).
    Use it as `t.pbm` on a Tapster. Raises RuntimeError if the robot
    replies with an error.
    """

    def __init__(self, tapster):
        self._t = tapster

    def _call(self, cmd, reply_timeout=5, **params):
        msg = {"cmd": cmd}
        msg.update({k: v for k, v in params.items() if v is not None})
        resp = self._t.send_wait(msg, timeout=reply_timeout)
        if resp is None:
            raise RuntimeError(f"{cmd}: no reply from robot")
        if "error" in resp:
            raise RuntimeError(f"{cmd}: {resp['error']}")
        return resp

    @staticmethod
    def _result(resp):
        return MoveResult(resp["position"], resp["contact"],
                          resp["final_effort"], resp["peak_effort"],
                          resp["effort_unit"])

    def ping(self, motor_id=None):
        """True if the motor answers. With no id, a list for all PBM motors."""
        resp = self._call("pbm_ping", id=motor_id)
        if motor_id is None:
            return resp["motors"]
        return resp["found"]

    def get_model_name(self, motor_id):
        resp = self._call("pbm_ping", id=motor_id)
        if not resp["found"]:
            raise RuntimeError(f"PBM motor {motor_id} did not respond")
        return resp["name"]

    def led_on(self, motor_id):
        self._call("pbm_led", id=motor_id, on=True)

    def led_off(self, motor_id):
        self._call("pbm_led", id=motor_id, on=False)

    def get_led_status(self, motor_id):
        return self._call("pbm_led", id=motor_id)["on"]

    def blink(self, motor_id, times=5, interval=0.1):
        self._call("pbm_blink", reply_timeout=5 + times * interval * 2,
                   id=motor_id, times=times, interval=interval)

    def torque_on(self, motor_id):
        self._call("pbm_torque", id=motor_id, on=True)

    def torque_off(self, motor_id):
        self._call("pbm_torque", id=motor_id, on=False)

    def get_torque_status(self, motor_id):
        return self._call("pbm_torque", id=motor_id)["on"]

    def get_position(self, motor_id):
        return self._call("pbm_position", id=motor_id)["position"]

    def get_effort(self, motor_id):
        """Returns (value, unit): mA on XC330, % on XL430/XC430."""
        resp = self._call("pbm_effort", id=motor_id)
        return resp["effort"], resp["unit"]

    def stop(self, motor_id):
        return self._call("pbm_stop", id=motor_id)["position"]

    def move(self, motor_id, goal_position, *, stop_on_contact=False,
             wait=False, timeout=1.0, position_tolerance=10):
        resp = self._call("pbm_move", reply_timeout=5 + timeout,
                          id=motor_id, position=goal_position,
                          stop_on_contact=stop_on_contact, wait=wait,
                          timeout=timeout,
                          position_tolerance=position_tolerance)
        return self._result(resp)

    def tap(self, motor_id, from_position, to_position, *,
            stop_on_contact=True, hold=0.05, timeout=1.0):
        # a tap is up to three moves of `timeout` each, plus the hold
        resp = self._call("pbm_tap", reply_timeout=5 + 3 * timeout + hold,
                          id=motor_id, from_position=from_position,
                          to_position=to_position,
                          stop_on_contact=stop_on_contact, hold=hold,
                          timeout=timeout)
        return self._result(resp)


class Tapster:
    def __init__(self, port=None, baud=115200):
        if port is None:
            port = find_port()
        self.port = port
        self.pbm = PBM(self)
        self.ser = serial.Serial(port, baud, timeout=0.5)
        # close before interpreter teardown, or pyserial's finalizer errors
        atexit.register(self.close)
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        print(f"Connected: {port}")

    def send(self, cmd):
        """Send a command dict and return the response dict."""
        line = json.dumps(cmd) + "\n"
        self.ser.write(line.encode())
        self.ser.flush()
        buf = b""
        start = time.time()
        while time.time() - start < 3.0:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                buf += chunk
                if b"\n" in buf:
                    # extract lines and try to parse each
                    for part in buf.split(b"\n"):
                        text = part.decode().replace("\r", "").strip()
                        if not text:
                            continue
                        try:
                            return json.loads(text)
                        except (json.JSONDecodeError, ValueError):
                            continue
        return None

    def send_wait(self, cmd, timeout=30):
        """Send a command and wait for JSON response, skipping non-JSON lines."""
        old_timeout = self.ser.timeout
        self.ser.timeout = timeout
        line = json.dumps(cmd) + "\n"
        self.ser.write(line.encode())
        try:
            while True:
                resp = self.ser.readline().decode().strip()
                if not resp:
                    return None
                try:
                    return json.loads(resp)
                except json.JSONDecodeError:
                    print(resp)  # show probe progress
        finally:
            self.ser.timeout = old_timeout

    def move(self, x=0, y=0, z=0, speed=100, easing="linear", step_size=5.0):
        self.send({"cmd": "move", "x": x, "y": y, "z": z,
                   "speed": speed, "easing": easing, "step_size": step_size})

    def tap(self, x=None, y=None, dx=0, dy=0, dz=None, speed=75):
        cmd = {"cmd": "tap", "dx": dx, "dy": dy, "speed": speed}
        if x is not None:
            cmd["x"] = x
        if y is not None:
            cmd["y"] = y
        if dz is not None:
            cmd["dz"] = dz
        self.send(cmd)

    def hop(self, x=0, y=0, z=None, speed=50, arc_height=30, lift=5,
            easing="ease_in_out"):
        cmd = {"cmd": "hop", "x": x, "y": y,
               "speed": speed, "arc_height": arc_height, "lift": lift}
        if z is not None:
            cmd["z"] = z
        if easing is not None:
            cmd["easing"] = easing
        self.send(cmd)

    def home(self, speed=100):
        self.send({"cmd": "home", "speed": speed})

    def status(self):
        return self.send({"cmd": "status"})

    def ping(self):
        return self.send({"cmd": "ping"})

    def enable(self):
        self.send({"cmd": "enable"})

    def disable(self):
        self.send({"cmd": "disable"})

    def blink(self, motor_id=1, count=3):
        self.send({"cmd": "blink", "id": motor_id, "count": count})

    def probe(self, threshold=3.0, step=1.0, max_depth=80):
        """Probe for surface by descending until load threshold is hit.
        Returns the detected surface height or None.
        """
        resp = self.send_wait({"cmd": "probe", "threshold": threshold,
                               "step": step, "max_depth": max_depth})
        if resp and "surface_height" in resp:
            return resp["surface_height"]
        return None

    @property
    def surface_height(self):
        """Get the current surface height (from probe or config)."""
        resp = self.send({"cmd": "surface"})
        if resp and "surface_height" in resp:
            return resp["surface_height"]
        return None

    @surface_height.setter
    def surface_height(self, value):
        self.send({"cmd": "surface", "surface_height": value})

    @property
    def safety_height(self):
        """Get the current safety height (from probe or config)."""
        resp = self.send({"cmd": "surface"})
        if resp and "safety_height" in resp:
            return resp["safety_height"]
        return None

    @safety_height.setter
    def safety_height(self, value):
        self.send({"cmd": "surface", "safety_height": value})

    @property
    def position(self):
        """Get current (x, y, z) — instant, no motor reads."""
        s = self.status()
        if s and "xyz" in s:
            p = s["xyz"]
            return (p["x"], p["y"], p["z"])
        return None

    def get_load(self, motor_id=None):
        """Get load for one motor or all motors.
        Returns percentage (-100 to 100) or list of them.
        """
        resp = self.send({"cmd": "load"})
        if resp and "load" in resp:
            loads = resp["load"]
            if motor_id is not None:
                idx = MOTOR_IDS.index(motor_id) if hasattr(self, '_motor_ids') else motor_id - 1
                return loads[motor_id - 1]
            return loads
        return None

    def get_temperature(self, motor_id=None):
        """Get temperature for one motor or all motors (degrees C)."""
        resp = self.send({"cmd": "temperature"})
        if resp and "temperature" in resp:
            temps = resp["temperature"]
            if motor_id is not None:
                return temps[motor_id - 1]
            return temps
        return None

    def close(self):
        atexit.unregister(self.close)
        self.ser.close()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Tapster delta robot client")
    parser.add_argument("--port", "-p", default=None, help="Serial port (auto-detect if omitted)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("-i", "--interactive", action="store_true", help="Drop into interactive REPL")
    args = parser.parse_args()

    t = Tapster(port=args.port, baud=args.baud)

    if args.interactive:
        import os
        os.environ["PYTHONINSPECT"] = "1"
    else:
        print(t.ping())
        print(t.status())
        t.close()
