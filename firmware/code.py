"""Tapster delta robot controller.

Listens for JSON commands on command UART and controls Dynamixel motors.
Cartesian origin (0,0,0) is configurable via "origin" in config.json.
"""

import json
import supervisor
import sys
import time
from hardware import dynamixel_bus, command_uart, load_pins
from kinematics import DeltaKinematics
from motion import interpolate, steps_for_distance, hop, distance
# The PBM is an optional accessory: never let it stop the robot booting.
try:
    from pbm import PBM, DEFAULT_IDS
except ImportError as e:
    print("PBM support not loaded:", e)
    PBM = None

# -- load config --
with open("/config.json", "r") as f:
    config = json.load(f)

# -- delta kinematics --
dk = DeltaKinematics(
    f=config["f"],
    e=config["e"],
    rf=config["rf"],
    re=config["re"],
)

# -- origin offset --
if "origin" in config:
    home_x, home_y, home_z = config["origin"]
else:
    home_x, home_y, home_z = dk.forward(0, 0, 0)
print(f"Origin: ({home_x:.2f}, {home_y:.2f}, {home_z:.2f})")

# -- dynamixel setup --
print("Board:", load_pins(config)["board"])
dxl = dynamixel_bus(config)

MOTOR_IDS = config["motor_ids"]
MOTOR_OFFSETS = config["motor_offsets"]
MOTOR_DIRECTION = config["motor_direction"]
SPEED = config.get("speed", 100)
MAX_SPEED = config.get("max_speed", 300)  # tip speed in mm/s at speed 100 (hop)
HOP_ARC_HEIGHT = config.get("hop_arc_height", 30)  # default hop arc, mm
SAFETY_HEIGHT = config.get("safety_height", 0)
SURFACE_HEIGHT = config.get("surface_height", 0)
MOTOR_MIN = config.get("motor_min", 1024)
MOTOR_MAX = config.get("motor_max", 3072)

# -- push button module (side-button actuators on the same bus) --
pbm = PBM(dxl, config.get("pbm_ids", DEFAULT_IDS)) if PBM else None

# -- command uart setup --
cmd_uart = command_uart(config)

# -- helpers --

def angle_to_position(motor_idx, angle_deg):
    """Convert IK angle (degrees) to Dynamixel position (0-4095)."""
    ticks = angle_deg * 4096.0 / 360.0
    pos = int(MOTOR_OFFSETS[motor_idx] + MOTOR_DIRECTION[motor_idx] * ticks)
    return max(MOTOR_MIN, min(MOTOR_MAX, pos))


def position_to_angle(motor_idx, position):
    """Convert Dynamixel position to IK angle (degrees)."""
    ticks = (position - MOTOR_OFFSETS[motor_idx]) * MOTOR_DIRECTION[motor_idx]
    return ticks * 360.0 / 4096.0


def speed_to_velocity(pct):
    """Convert speed percentage (1-100) to Dynamixel profile velocity.
    Uses quadratic curve so the range feels more natural:
    100 -> 1023, 50 -> 256, 25 -> 64, 10 -> 10
    """
    pct = max(1, min(100, pct))
    return int((pct / 100.0) ** 2 * 1023)


def save_config():
    """Write config.json. Returns False if the filesystem is read-only
    (on the Pico, CIRCUITPY is read-only to code while USB-mounted)."""
    try:
        with open("/config.json", "w") as f:
            json.dump(config, f)
        return True
    except OSError as e:
        print("Could not save config.json:", e)
        return False


# current position in user coordinates (tracked in software)
current_pos = [0.0, 0.0, 0.0]


def move_to(x, y, z, speed=None):
    """Move end effector to (x, y, z) directly (no interpolation)."""
    if speed is None:
        speed = SPEED
    velocity = speed_to_velocity(speed)
    raw_x = x + home_x
    raw_y = y + home_y
    raw_z = z + home_z
    t1, t2, t3 = dk.inverse(raw_x, raw_y, raw_z)
    angles = [t1, t2, t3]
    moves = []
    for i, mid in enumerate(MOTOR_IDS):
        moves.append((mid, velocity, angle_to_position(i, angles[i])))
    dxl.sync_move(moves)
    current_pos[0] = x
    current_pos[1] = y
    current_pos[2] = z


def line_to(x, y, z, speed=None, easing="ease_in_out", step_size=5.0):
    """Move in a straight line from current position to (x, y, z).

    speed: 1-100 percentage
    easing: easing function name (linear, ease_in, ease_out,
            ease_in_out, ease_in_cubic, ease_out_cubic, ease_in_out_cubic)
    step_size: mm between waypoints (smaller = smoother)
    """
    if speed is None:
        speed = SPEED
    start = tuple(current_pos)
    end = (x, y, z)
    steps = steps_for_distance(start, end, step_size)
    velocity = 1023
    delay = 0.002 + (1.0 - speed / 100.0) * 0.02
    # plan the whole line first: refuse it, without moving, if any of it is
    # out of reach (see motor_positions)
    path = [motor_positions(*w)
            for w in interpolate(start, end, steps, easing)]
    for positions in path:
        moves = []
        for i, mid in enumerate(MOTOR_IDS):
            moves.append((mid, velocity, positions[i]))
        dxl.sync_move(moves)
        time.sleep(delay)
    current_pos[0] = x
    current_pos[1] = y
    current_pos[2] = z


def motor_positions(x, y, z, strict=True):
    """Motor positions for user (x, y, z).

    Raises ValueError if (x, y, z) is unreachable or, when strict, if it
    needs a motor outside motor_min..motor_max. (angle_to_position would
    clamp that motor instead, putting the tip somewhere else.)
    """
    angles = dk.inverse(x + home_x, y + home_y, z + home_z)
    positions = [angle_to_position(i, angles[i]) for i in range(len(MOTOR_IDS))]
    if strict:
        for i, angle in enumerate(angles):
            pos = MOTOR_OFFSETS[i] + MOTOR_DIRECTION[i] * angle * 4096.0 / 360.0
            if not MOTOR_MIN <= pos <= MOTOR_MAX:
                raise ValueError(
                    "Out of reach: (%.1f, %.1f, %.1f) needs motor %d at %d, "
                    "outside motor_min-motor_max (%d-%d)" % (
                        x, y, z, MOTOR_IDS[i], int(pos), MOTOR_MIN, MOTOR_MAX))
    return positions


def profile_velocity(ticks, seconds):
    """Profile Velocity (0.229 rpm units) that covers `ticks` in `seconds`."""
    rpm = ticks / 4096.0 / seconds * 60.0
    # at least 1: 0 means "no limit" to the motor
    return max(1, min(1023, int(rpm / 0.229) + 1))


def plan(waypoints):
    """[(waypoint, motor positions)]. Raises ValueError if any is out of reach."""
    return [(w, motor_positions(*w)) for w in waypoints]


def follow(targets, speed):
    """Drive the tip through planned targets at `speed` % of max_speed (mm/s).

    For each segment, every motor's Profile Velocity is set so that all
    three reach the waypoint together, just as the next one is sent.
    """
    mm_per_s = MAX_SPEED * max(1, min(100, speed)) / 100.0
    here = tuple(current_pos)
    last = motor_positions(*here, strict=False)  # where the motors are now
    due = time.monotonic_ns()
    for w, positions in targets:
        seconds = max(0.002, distance(here, w) / mm_per_s)
        moves = []
        for i, mid in enumerate(MOTOR_IDS):
            vel = profile_velocity(abs(positions[i] - last[i]), seconds)
            moves.append((mid, vel, positions[i]))
        dxl.sync_move(moves)
        due += int(seconds * 1000000000)
        wait = due - time.monotonic_ns()
        if wait > 0:
            time.sleep(wait / 1000000000)
        here, last = w, positions


class ArcTooHigh(ValueError):
    """A hop that fits only with a lower arc.
    args: (message, highest arc_height that fits)"""


def highest_arc(start, end, lift, easing, too_high):
    """Highest arc_height, to 0.5 mm, below too_high that makes the hop
    reachable, or None if even a flat hop (arc 0) isn't."""
    def fits(arc):
        try:
            plan(hop(start, end, arc, lift=lift, easing=easing))
            return True
        except ValueError:
            return False

    if not fits(0):
        return None
    lo, hi = 0.0, float(too_high)
    while hi - lo > 0.25:
        mid = (lo + hi) / 2
        if fits(mid):
            lo = mid
        else:
            hi = mid
    return int(lo * 2) / 2.0  # round down to 0.5 mm


def hop_to(x, y, z, speed=None, arc_height=None, easing="ease_in_out", lift=5):
    """Hop in a smooth arc from current position to (x, y, z).

    speed: 1-100, % of max_speed (config.json, mm/s)
    arc_height: mm above the higher of start/end at the peak
        (default hop_arc_height from config.json)
    easing: easing function for XY movement
    lift: mm to rise straight up before the arc
    Raises ValueError, without moving, if any part of the arc is out of
    reach: ArcTooHigh if a lower arc would work.
    """
    if speed is None:
        speed = SPEED
    if arc_height is None:
        arc_height = HOP_ARC_HEIGHT
    start = tuple(current_pos)
    end = (x, y, z)
    try:
        targets = plan(hop(start, end, arc_height, lift=lift, easing=easing))
    except ValueError:
        best = highest_arc(start, end, lift, easing, arc_height)
        if best is None:
            raise  # the target or the lift is out of reach, not the arc
        raise ArcTooHigh(
            "Unreachable position: arc_height %g is too high here; "
            "the highest that fits is %g" % (arc_height, best), best)
    follow(targets, speed)
    current_pos[0] = x
    current_pos[1] = y
    current_pos[2] = z


def get_xyz():
    """Read current position from motors as (x, y, z) in mm."""
    angles = []
    for i, mid in enumerate(MOTOR_IDS):
        pos = dxl.get_position(mid)
        angles.append(position_to_angle(i, pos))
    raw_x, raw_y, raw_z = dk.forward(*angles)
    return (raw_x - home_x, raw_y - home_y, raw_z - home_z)


def enable_motors(on=True):
    for mid in MOTOR_IDS:
        dxl.set_torque(mid, on)
        dxl.set_led(mid, on)


def probe_surface(threshold=3.0, step=1.0, max_depth=80):
    """Descend from origin until average load exceeds threshold.

    threshold: absolute load percentage to trigger stop
    step: mm to descend per step
    max_depth: maximum mm to descend before giving up

    Returns the z position where surface was detected, or None.
    """
    global SURFACE_HEIGHT, SAFETY_HEIGHT

    # go to origin first
    move_to(0, 0, 0, 80)
    time.sleep(0.5)

    z = 0
    while z > -max_depth:
        z -= step
        move_to(0, 0, z, 80)
        time.sleep(0.01)

        try:
            loads = []
            for mid in MOTOR_IDS:
                loads.append(abs(dxl.get_load(mid)))
        except Exception as e:
            print("Read error at z=%.1f: %s" % (z, str(e)))
            continue

        avg = sum(loads) / len(loads)
        print("z=%.1f  load=%.1f,%.1f,%.1f  avg=%.1f" % (
            z, loads[0], loads[1], loads[2], avg))

        if avg > threshold:
            z += step
            move_to(0, 0, z, 50)
            SURFACE_HEIGHT = z
            SAFETY_HEIGHT = z + 10
            print("Surface found at z=%.1f" % z)
            print("Safety height set to z=%.1f" % SAFETY_HEIGHT)
            config["surface_height"] = SURFACE_HEIGHT
            config["safety_height"] = SAFETY_HEIGHT
            save_config()
            move_to(0, 0, 0, 80)
            return z

    # didn't find surface
    move_to(0, 0, 0, 80)
    print("Surface not found")
    return None


def respond(obj):
    """Send JSON response to command UART and console."""
    out = json.dumps(obj) + "\n"
    cmd_uart.write(out.encode())
    print(out, end="")


# -- PBM command handler --

def pbm_ping_one(mid):
    """{"id", "found", "name"} for one PBM motor."""
    if not pbm.ping(mid):
        return {"id": mid, "found": False}
    try:
        name = pbm.get_model_name(mid)
    except RuntimeError as e:  # answers, but isn't a supported model
        name = str(e)
    return {"id": mid, "found": True, "name": name}


def pbm_result(mid, result):
    out = result.to_dict()
    out["ok"] = True
    out["id"] = mid
    return out


def handle_pbm(cmd, msg):
    """pbm_* commands. Every one except pbm_ping needs "id"."""
    if cmd == "pbm_ping":
        if "id" in msg:
            out = pbm_ping_one(msg["id"])
            out["ok"] = True
            respond(out)
        else:
            respond({"ok": True, "motors": [pbm_ping_one(m) for m in pbm.ids]})
        return

    if "id" not in msg:
        respond({"error": f"{cmd} needs \"id\" (PBM ids: {list(pbm.ids)})"})
        return
    mid = msg["id"]

    if cmd == "pbm_led":
        if "on" in msg:
            if msg["on"]:
                pbm.led_on(mid)
            else:
                pbm.led_off(mid)
        respond({"ok": True, "id": mid, "on": pbm.get_led_status(mid)})

    elif cmd == "pbm_blink":
        times = msg.get("times", 5)
        pbm.blink(mid, times, msg.get("interval", 0.1))
        respond({"ok": True, "id": mid, "times": times})

    elif cmd == "pbm_torque":
        if "on" in msg:
            if msg["on"]:
                pbm.torque_on(mid)
            else:
                pbm.torque_off(mid)
        respond({"ok": True, "id": mid, "on": pbm.get_torque_status(mid)})

    elif cmd == "pbm_position":
        respond({"ok": True, "id": mid, "position": pbm.get_position(mid)})

    elif cmd == "pbm_effort":
        effort, unit = pbm.get_effort(mid)
        respond({"ok": True, "id": mid, "effort": effort, "unit": unit})

    elif cmd == "pbm_stop":
        respond({"ok": True, "id": mid, "position": pbm.stop(mid)})

    elif cmd == "pbm_move":
        result = pbm.move(
            mid, msg["position"],
            stop_on_contact=msg.get("stop_on_contact", False),
            wait=msg.get("wait", False),
            timeout=msg.get("timeout", 1.0),
            position_tolerance=msg.get("position_tolerance", 10),
        )
        respond(pbm_result(mid, result))

    elif cmd == "pbm_tap":
        result = pbm.tap(
            mid, msg["from_position"], msg["to_position"],
            stop_on_contact=msg.get("stop_on_contact", True),
            hold=msg.get("hold", 0.05),
            timeout=msg.get("timeout", 1.0),
        )
        respond(pbm_result(mid, result))

    else:
        respond({"error": f"unknown cmd: {cmd}"})


# -- command handler --

def handle(cmd_str):
    try:
        msg = json.loads(cmd_str)
    except ValueError:
        respond({"error": "invalid json"})
        return

    cmd = msg.get("cmd", "")
    global SURFACE_HEIGHT, SAFETY_HEIGHT

    if cmd == "ping":
        results = []
        for mid in MOTOR_IDS:
            results.append(dxl.ping(mid))
        respond({"ok": True, "motors": results})

    elif cmd == "move":
        x = msg.get("x", 0)
        y = msg.get("y", 0)
        z = msg.get("z", SAFETY_HEIGHT)
        speed = msg.get("speed", SPEED)
        easing = msg.get("easing", "linear")
        step_size = msg.get("step_size", 5.0)
        try:
            line_to(x, y, z, speed, easing, step_size)
            respond({"ok": True, "target": {"x": x, "y": y, "z": z}})
        except ValueError as e:
            respond({"error": str(e)})

    elif cmd == "home":
        speed = msg.get("speed", SPEED)
        try:
            line_to(0, 0, 0, speed)
            respond({"ok": True, "target": {"x": 0, "y": 0, "z": 0}})
        except ValueError as e:
            respond({"error": str(e)})

    elif cmd == "status":
        cx, cy, cz = current_pos
        respond({"ok": True, "xyz": {"x": cx, "y": cy, "z": cz}})

    elif cmd == "load":
        try:
            loads = []
            for mid in MOTOR_IDS:
                loads.append(dxl.get_load(mid))
            respond({"ok": True, "load": loads})
        except Exception as e:
            respond({"error": str(e)})

    elif cmd == "temperature":
        try:
            temps = []
            for mid in MOTOR_IDS:
                temps.append(dxl.get_temperature(mid))
            respond({"ok": True, "temperature": temps})
        except Exception as e:
            respond({"error": str(e)})

    elif cmd == "enable":
        enable_motors(True)
        respond({"ok": True})

    elif cmd == "disable":
        enable_motors(False)
        respond({"ok": True})

    elif cmd == "tap":
        x = msg.get("x", None)
        y = msg.get("y", None)
        dx = msg.get("dx", 0)
        dy = msg.get("dy", 0)
        dz = msg.get("dz", None)
        speed = msg.get("speed", 75)
        if dz is None and SURFACE_HEIGHT >= SAFETY_HEIGHT:
            respond({"error": "surface_height must be below safety_height "
                     "(run probe, set surface, or pass dz)"})
            return
        try:
            cx, cy, cz = current_pos
            # target position
            tx = x if x is not None else cx
            ty = y if y is not None else cy
            # move to safety height at target position
            move_to(tx, ty, SAFETY_HEIGHT, SPEED)
            time.sleep(0.05)
            # tap down to surface (or apply dz from safety)
            tap_z = SURFACE_HEIGHT if dz is None else SAFETY_HEIGHT + dz
            move_to(tx + dx, ty + dy, tap_z, speed)
            dwell = 0.05 + (1.0 - speed / 100.0) * 0.3
            time.sleep(dwell)
            # retract to safety
            move_to(tx, ty, SAFETY_HEIGHT, speed)
            time.sleep(dwell)
            respond({"ok": True, "tap": {"x": tx, "y": ty, "z": tap_z}})
        except ValueError as e:
            respond({"error": str(e)})

    elif cmd == "hop":
        x = msg.get("x", 0)
        y = msg.get("y", 0)
        z = msg.get("z", SAFETY_HEIGHT)
        speed = msg.get("speed", 50)
        lift = msg.get("lift", 5)
        arc_height = msg.get("arc_height", HOP_ARC_HEIGHT)
        easing = msg.get("easing", "ease_in_out")
        try:
            hop_to(x, y, z, speed, arc_height, easing, lift)
            respond({"ok": True, "target": {"x": x, "y": y, "z": z}})
        except ArcTooHigh as e:
            respond({"error": e.args[0], "max_arc_height": e.args[1]})
        except ValueError as e:
            respond({"error": str(e)})

    elif cmd == "blink":
        mid = msg.get("id", 1)
        count = msg.get("count", 3)
        for _ in range(count):
            dxl.set_led(mid, True)
            time.sleep(0.2)
            dxl.set_led(mid, False)
            time.sleep(0.2)
        respond({"ok": True, "blink": {"id": mid, "count": count}})

    elif cmd == "probe":
        threshold = msg.get("threshold", 5.0)
        step = msg.get("step", 1.0)
        max_depth = msg.get("max_depth", 80)
        z = probe_surface(threshold, step, max_depth)
        if z is not None:
            respond({"ok": True, "surface_height": z})
        else:
            respond({"error": "surface not found"})

    elif cmd == "surface":
        changed = False
        saved = True
        if "surface_height" in msg:
            SURFACE_HEIGHT = msg["surface_height"]
            config["surface_height"] = SURFACE_HEIGHT
            changed = True
        if "safety_height" in msg:
            SAFETY_HEIGHT = msg["safety_height"]
            config["safety_height"] = SAFETY_HEIGHT
            changed = True
        if changed:
            saved = save_config()
        respond({"ok": True, "surface_height": SURFACE_HEIGHT,
                 "safety_height": SAFETY_HEIGHT, "saved": saved})

    elif cmd.startswith("pbm_"):
        if pbm is None:
            respond({"error": "PBM support not loaded (is pbm.py on the board?)"})
            return
        # catch everything: a bad PBM command must not stop the main loop
        try:
            handle_pbm(cmd, msg)
        except KeyError as e:
            respond({"error": f"{cmd} needs {e}"})
        except (ValueError, RuntimeError) as e:
            respond({"error": str(e)})
        except Exception as e:
            respond({"error": f"{cmd}: {type(e).__name__}: {e}"})

    else:
        respond({"error": f"unknown cmd: {cmd}"})


# -- main loop --

print("Tapster ready.")

for mid in MOTOR_IDS:
    try:
        info = dxl.ping(mid)
        print(f"Motor {mid}: {info['name']} (model {info['model']})")
    except RuntimeError as e:
        print(e)

# the PBM is optional, so a missing motor is only reported
if pbm is not None:
    for mid in pbm.ids:
        try:
            info = pbm_ping_one(mid)
            print(f"PBM motor {mid}: {info.get('name', 'not found')}")
        except Exception as e:
            print(f"PBM motor {mid}: {e}")

# enable motors and go home
enable_motors(True)
move_to(0, 0, 0)
print("Home: (0, 0, 0)")

line_buf = bytearray()
console_buf = bytearray()

while True:
    # read from command UART
    data = cmd_uart.read(256)
    if data:
        line_buf.extend(data)
        while b"\n" in line_buf:
            idx = line_buf.index(b"\n")
            line = line_buf[:idx].decode().strip()
            line_buf = line_buf[idx + 1:]
            if line:
                handle(line)

    # read from console (tio)
    while supervisor.runtime.serial_bytes_available:
        c = sys.stdin.read(1)
        if c == "\n" or c == "\r":
            line = console_buf.decode().strip()
            console_buf = bytearray()
            if line:
                handle(line)
        else:
            console_buf.extend(c.encode())

    time.sleep(0.001)
