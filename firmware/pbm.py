"""Push Button Module (PBM) for CircuitPython.

A port of https://github.com/tapsterbot/valet-pbm to this firmware's
Dynamixel driver. The PBM's linear actuators share the Dynamixel bus with
the delta robot's motors, so PBM takes the same Dynamixel object:

    from hardware import dynamixel_bus
    from pbm import PBM

    pbm = PBM(dynamixel_bus())
    pbm.tap(4, from_position=2048, to_position=1700)

Only the IDs passed as `ids` can be commanded, so a typo can't drive one of
the delta motors (IDs 1-3) to a raw position.

Retries and the half-duplex DIR pin are handled by dynamixel.py and
hardware.py, so the Raspberry Pi transport options from valet-pbm (device,
baudrate, direction_pin, response_timeout_ms, retry_delay) aren't here.
"""

import time

ADDR_TORQUE_ENABLE = 64
ADDR_LED = 65
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_EFFORT = 126

DEFAULT_IDS = (4, 5, 6)

# model number -> (name, effort unit, scale from the raw address 126 value)
# XC330: Present Current in mA. XL430/XC430: Present Load in 0.1 %.
MOTOR_PROFILES = {
    1210: ("XC330-T181-T", "mA", 1.0),
    1220: ("XC330-T288-T", "mA", 1.0),
    1060: ("XL430-W250-T", "%", 0.1),
    1080: ("XC430-W240-T", "%", 0.1),
}


def _ns(seconds):
    return int(seconds * 1000000000)


def _position(value):
    # bool is an int subclass; True would mean position 1
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError("position must be an integer, got %r" % (value,))
    return value


class MoveResult:
    def __init__(self, position, contact=False, final_effort=None,
                 peak_effort=None, effort_unit=None):
        self.position = position
        self.contact = contact
        self.final_effort = final_effort
        self.peak_effort = peak_effort
        self.effort_unit = effort_unit

    def to_dict(self):
        return {
            "position": self.position,
            "contact": self.contact,
            "final_effort": self.final_effort,
            "peak_effort": self.peak_effort,
            "effort_unit": self.effort_unit,
        }

    def __repr__(self):
        return ("MoveResult(position=%r, contact=%r, final_effort=%r, "
                "peak_effort=%r, effort_unit=%r)" % (
                    self.position, self.contact, self.final_effort,
                    self.peak_effort, self.effort_unit))


class PBM:
    def __init__(
        self,
        dxl,
        ids=DEFAULT_IDS,
        poll_interval=0.005,
        contact_samples=2,
        contact_ignore_time=0.04,
        contact_window=0.04,
        contact_min_progress=20,
        contact_min_travel=10,
    ):
        self.dxl = dxl
        self.ids = tuple(ids)
        self.poll_interval = poll_interval
        self.contact_samples = contact_samples
        self.contact_ignore_time = contact_ignore_time
        self.contact_window = contact_window
        self.contact_min_progress = contact_min_progress
        self.contact_min_travel = contact_min_travel

    def _check_id(self, motor_id):
        if motor_id not in self.ids:
            raise ValueError("motor %r is not a PBM motor (PBM ids: %s)" % (
                motor_id, list(self.ids)))

    # -- identity --

    def get_model_number(self, motor_id):
        self._check_id(motor_id)
        return self.dxl.get_model(motor_id)

    def _profile(self, motor_id):
        model = self.get_model_number(motor_id)
        if model not in MOTOR_PROFILES:
            raise RuntimeError("DYNAMIXEL %d: unsupported model number %d" % (
                motor_id, model))
        return MOTOR_PROFILES[model]

    def get_model_name(self, motor_id):
        return self._profile(motor_id)[0]

    # -- basic operations --

    def ping(self, motor_id):
        """True if the motor answers."""
        self._check_id(motor_id)
        try:
            self.dxl.ping(motor_id)
            return True
        except RuntimeError:
            return False

    def led_on(self, motor_id):
        self._check_id(motor_id)
        self.dxl.write_u8(motor_id, ADDR_LED, 1)

    def led_off(self, motor_id):
        self._check_id(motor_id)
        self.dxl.write_u8(motor_id, ADDR_LED, 0)

    def get_led_status(self, motor_id):
        self._check_id(motor_id)
        return bool(self.dxl.read_u8(motor_id, ADDR_LED))

    def blink(self, motor_id, times=5, interval=0.1):
        interval = float(interval)
        if times < 0:
            raise ValueError("times must be >= 0")
        for _ in range(times):
            self.led_on(motor_id)
            time.sleep(interval)
            self.led_off(motor_id)
            time.sleep(interval)

    def torque_on(self, motor_id):
        self._check_id(motor_id)
        self.dxl.write_u8(motor_id, ADDR_TORQUE_ENABLE, 1)

    def torque_off(self, motor_id):
        self._check_id(motor_id)
        self.dxl.write_u8(motor_id, ADDR_TORQUE_ENABLE, 0)

    def get_torque_status(self, motor_id):
        self._check_id(motor_id)
        return bool(self.dxl.read_u8(motor_id, ADDR_TORQUE_ENABLE))

    def _ensure_torque(self, motor_id):
        if not self.get_torque_status(motor_id):
            self.torque_on(motor_id)

    def get_position(self, motor_id):
        self._check_id(motor_id)
        return self.dxl.get_position(motor_id)

    def get_effort(self, motor_id):
        """Returns (value, unit): mA on XC330, % on XL430/XC430."""
        _, unit, scale = self._profile(motor_id)
        return self.dxl.read_s16(motor_id, ADDR_PRESENT_EFFORT) * scale, unit

    # -- movement / contact --

    def _set_goal(self, motor_id, position):
        self.dxl.write_u32(motor_id, ADDR_GOAL_POSITION, position)

    def stop(self, motor_id):
        """Hold the motor where it is now. Returns that position."""
        position = self.get_position(motor_id)
        self._set_goal(motor_id, position)
        return position

    def move(self, motor_id, goal_position, *, stop_on_contact=False,
             wait=False, timeout=1.0, position_tolerance=10):
        """Move to goal_position.

        wait: poll until the motor is within position_tolerance of the goal.
        stop_on_contact: also stop early if the motor stops making progress
            short of the goal (it hit something). Implies waiting.
        """
        # check every argument before anything moves
        self._check_id(motor_id)
        goal_position = _position(goal_position)
        timeout = float(timeout)
        position_tolerance = float(position_tolerance)

        self._ensure_torque(motor_id)
        _, unit, _ = self._profile(motor_id)
        start_position = (self.get_position(motor_id)
                          if stop_on_contact else None)

        self._set_goal(motor_id, goal_position)

        if not wait and not stop_on_contact:
            return MoveResult(position=None)

        # monotonic_ns, not monotonic: CircuitPython floats lose millisecond
        # precision after the board has been up for a few hours.
        start = time.monotonic_ns()
        deadline = start + _ns(timeout)
        ignore_ns = _ns(self.contact_ignore_time)
        window_ns = _ns(self.contact_window)
        full_window_ns = _ns(self.contact_window * 0.8)

        contact_count = 0
        last_position = start_position
        last_effort = None
        peak_effort = None
        history = []  # (time_ns, position), oldest first

        while time.monotonic_ns() < deadline:
            now = time.monotonic_ns()
            last_position = self.get_position(motor_id)

            if stop_on_contact:
                effort, _ = self.get_effort(motor_id)
                last_effort = abs(effort)
                if peak_effort is None or last_effort > peak_effort:
                    peak_effort = last_effort

            if abs(last_position - goal_position) <= position_tolerance:
                return MoveResult(last_position, False, last_effort,
                                  peak_effort, unit)

            if stop_on_contact:
                history.append((now, last_position))
                while history and now - history[0][0] > window_ns:
                    history.pop(0)

                travel = abs(last_position - start_position)
                remaining = abs(goal_position - last_position)
                full_window = bool(history) and (
                    now - history[0][0] >= full_window_ns)

                eligible = (
                    now - start >= ignore_ns
                    and travel >= self.contact_min_travel
                    and remaining > position_tolerance
                    and full_window
                )
                stalled = full_window and (
                    abs(last_position - history[0][1])
                    < self.contact_min_progress)

                if eligible and stalled:
                    contact_count += 1
                else:
                    contact_count = 0

                if contact_count >= self.contact_samples:
                    position = self.stop(motor_id)
                    return MoveResult(position, True, last_effort,
                                      peak_effort, unit)

            time.sleep(self.poll_interval)

        return MoveResult(last_position, False, last_effort, peak_effort, unit)

    def tap(self, motor_id, from_position, to_position, *,
            stop_on_contact=True, hold=0.05, timeout=1.0):
        """Press a button: go to from_position, move toward to_position
        (stopping early on contact), hold, then return to from_position.
        Returns the MoveResult of the press.
        """
        # check every argument before anything moves
        self._check_id(motor_id)
        from_position = _position(from_position)
        to_position = _position(to_position)
        hold = float(hold)
        timeout = float(timeout)

        self.move(motor_id, from_position, wait=True, timeout=timeout)

        press_result = MoveResult(position=None)
        try:
            press_result = self.move(motor_id, to_position,
                                     stop_on_contact=stop_on_contact,
                                     wait=True, timeout=timeout)
            time.sleep(hold)
        finally:
            # Releasing the button is safety-critical. Always attempt it.
            self.move(motor_id, from_position, wait=True, timeout=timeout)

        return press_result
