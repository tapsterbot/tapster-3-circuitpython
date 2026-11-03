"""Tapster client — send commands to the robot (ESP32 or Pico) over USB serial."""

import atexit
import glob
import json
import serial
import sys
import time


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


class Tapster:
    def __init__(self, port=None, baud=115200):
        if port is None:
            port = find_port()
        self.port = port
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
