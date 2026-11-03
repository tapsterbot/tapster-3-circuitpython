"""Fakes for running the firmware and the client on a computer.

- CircuitPython modules: board, busio, digitalio, supervisor.
- FakeMotor, FakeDynamixelBus: Dynamixel motors that speak Protocol 2.0
  bytes, so the real dynamixel.py is tested down to the packet level.
- Firmware: runs firmware/code.py, main loop included, in a thread.
- FakeSerial: a pyserial stand-in wired to the firmware's USB console or
  its command UART, for the real tapster.py client.
"""

import builtins
import json
import sys
import threading
import time
import traceback
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FIRMWARE = ROOT / "firmware"

HEADER = b"\xff\xff\xfd\x00"
PING, READ, WRITE, REBOOT, SYNC_WRITE = 0x01, 0x02, 0x03, 0x08, 0x83
BROADCAST = 0xFE
ERR_DATA_LIMIT = 0x06


# -- CircuitPython modules -----------------------------------------------

def _board_getattr(name):
    if name.startswith("__"):
        raise AttributeError(name)
    return name  # a pin is just its name, e.g. board.GP4 == "GP4"


_uart_factory = None


def set_uart_factory(factory):
    """busio.UART(**kwargs) returns factory(**kwargs)."""
    global _uart_factory
    _uart_factory = factory


class _UART:
    def __new__(cls, **kwargs):
        if _uart_factory is None:
            raise RuntimeError("no fake UART factory set")
        return _uart_factory(**kwargs)


class DigitalInOut:
    def __init__(self, pin):
        self.pin = pin
        self.history = []
        self._value = False

    def switch_to_output(self, value=False):
        self.value = value

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, v):
        self._value = v
        self.history.append(v)


def install_circuitpython_stubs():
    board = types.ModuleType("board")
    board.board_id = "raspberry_pi_pico"
    board.__getattr__ = _board_getattr
    busio = types.ModuleType("busio")
    busio.UART = _UART
    digitalio = types.ModuleType("digitalio")
    digitalio.DigitalInOut = DigitalInOut
    supervisor = types.ModuleType("supervisor")
    supervisor.runtime = types.SimpleNamespace(serial_bytes_available=0)
    for mod in (board, busio, digitalio, supervisor):
        sys.modules.setdefault(mod.__name__, mod)


# -- Dynamixel motors ----------------------------------------------------

def crc16(data):
    """Protocol 2.0 CRC, computed bit by bit (independent of the driver's table)."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def status_packet(motor_id, error=0, params=b""):
    length = len(params) + 4  # instruction + error + params + CRC
    pkt = bytearray(HEADER)
    pkt += bytes([motor_id, length & 0xFF, length >> 8, 0x55, error])
    pkt += params
    crc = crc16(pkt)
    pkt += bytes([crc & 0xFF, crc >> 8])
    return bytes(pkt)


class FakeMotor:
    """One motor's control table, with a position that moves over time."""

    def __init__(self, motor_id, model=1080, position=2048, speed=None,
                 firmware=50):
        self.id = motor_id
        self.table = bytearray(256)
        self._put(0, 2, model)
        self._put(6, 1, firmware)
        self._put(7, 1, motor_id)
        self._put(38, 2, 910)      # Current Limit (XC330)
        self._put(48, 4, 4095)     # Max Position Limit
        self._put(52, 4, 0)        # Min Position Limit
        self._put(116, 4, position)
        self._put(144, 2, 120)     # 12.0 V
        self._put(146, 1, 36)      # 36 C
        self.speed = speed         # ticks/s; None = arrives instantly
        self.obstacle = None       # a button: blocks positions below this
        self.effort = 0            # raw Present Load / Present Current
        self.effort_fn = None      # optional: fn(motor) -> raw effort
        self._anchor = (time.monotonic(), position, position)

    def _put(self, addr, size, value):
        mask = (1 << (8 * size)) - 1
        self.table[addr:addr + size] = (value & mask).to_bytes(size, "little")

    def _get(self, addr, size, signed=False):
        return int.from_bytes(self.table[addr:addr + size], "little",
                              signed=signed)

    @property
    def model(self):
        return self._get(0, 2)

    @property
    def torque(self):
        return self._get(64, 1)

    @property
    def led(self):
        return self._get(65, 1)

    @property
    def goal(self):
        return self._get(116, 4, signed=True)

    @property
    def profile_velocity(self):
        return self._get(112, 4)

    @property
    def position(self):
        t0, p0, goal = self._anchor
        if not self.torque:
            return p0
        if self.speed is None:
            pos = goal
        else:
            step = min(self.speed * (time.monotonic() - t0), abs(goal - p0))
            pos = int(p0 + (1 if goal >= p0 else -1) * step)
        if self.obstacle is not None and goal < self.obstacle:
            pos = max(pos, self.obstacle)
        return pos

    def read(self, addr, size):
        effort = self.effort_fn(self) if self.effort_fn else self.effort
        self._put(126, 2, effort)
        self._put(122, 1, int(self.position != self.goal))
        self._put(132, 4, self.position)
        return bytes(self.table[addr:addr + size])

    def write(self, addr, data):
        """Apply a write. Returns the status packet's error byte."""
        end = addr + len(data)
        new = bytearray(self.table)
        new[addr:end] = data
        if addr < 120 and end > 116:
            goal = int.from_bytes(new[116:120], "little", signed=True)
            if not self._get(52, 4) <= goal <= self._get(48, 4):
                return ERR_DATA_LIMIT
        here = self.position
        self.table = new
        if addr <= 64 < end or (addr < 120 and end > 116):
            self._anchor = (time.monotonic(), here, self.goal)
        return 0


class FakeDynamixelBus:
    """Stands in for the busio.UART on the Dynamixel bus."""

    def __init__(self, motors=(), baudrate=3000000):
        self.motors = {m.id: m for m in motors}
        self.baudrate = baudrate
        self.lock = threading.RLock()
        self._rx = bytearray()
        self.packets = []        # (id, instruction, params) received
        self.sync_goals = []     # {id: goal} for each Sync Write of Goal Position
        self.sync_velocities = []  # {id: Profile Velocity} at the same moments
        self.sync_times = []     # time.monotonic() of each
        self.errors = []         # malformed packets (a driver bug)
        self.drop_replies = 0    # lose this many replies
        self.noise = b""         # junk sent ahead of the next reply
        self.short_ping = set()  # ids that send a truncated ping reply
        self.silent_reads = set()  # (id, addr) reads that never get a reply

    def write(self, data):
        with self.lock:
            reply = self._receive(bytes(data))
            if reply is not None:
                if self.drop_replies:
                    self.drop_replies -= 1
                else:
                    self._rx += self.noise + reply
                    self.noise = b""
        return len(data)

    def read(self, nbytes=None):
        with self.lock:
            if not self._rx:
                return None
            n = len(self._rx) if nbytes is None else nbytes
            out = bytes(self._rx[:n])
            del self._rx[:n]
            return out

    def reset_input_buffer(self):
        with self.lock:
            self._rx.clear()

    def writes(self, since=0, address=None):
        """[(id, address, data)] of single-motor WRITE packets."""
        out = []
        for mid, inst, params in self.packets[since:]:
            if inst != WRITE:
                continue
            addr = params[0] | params[1] << 8
            if address is None or addr == address:
                out.append((mid, addr, bytes(params[2:])))
        return out

    def _receive(self, pkt):
        if pkt[:4] != HEADER or len(pkt) < 10:
            self.errors.append(("bad header", pkt))
            return None
        length = pkt[5] | pkt[6] << 8
        if len(pkt) != 7 + length:
            self.errors.append(("bad length", pkt))
            return None
        if crc16(pkt[:-2]) != (pkt[-2] | pkt[-1] << 8):
            self.errors.append(("bad CRC", pkt))
            return None
        mid, inst, params = pkt[4], pkt[7], pkt[8:-2]
        self.packets.append((mid, inst, params))

        if inst == SYNC_WRITE and mid == BROADCAST:
            addr = params[0] | params[1] << 8
            size = params[2] | params[3] << 8
            body = params[4:]
            goals, velocities = {}, {}
            for i in range(0, len(body), size + 1):
                m = self.motors.get(body[i])
                if m is not None:
                    m.write(addr, body[i + 1:i + 1 + size])
                    goals[m.id] = m.goal
                    velocities[m.id] = m.profile_velocity
            if addr < 120 and addr + size > 116:
                self.sync_goals.append(goals)
                self.sync_velocities.append(velocities)
                self.sync_times.append(time.monotonic())
            return None

        m = self.motors.get(mid)
        if m is None:
            return None
        if inst == PING:
            if mid in self.short_ping:
                return status_packet(mid)
            return status_packet(mid, 0, m.read(0, 2) + m.read(6, 1))
        if inst == READ:
            addr = params[0] | params[1] << 8
            size = params[2] | params[3] << 8
            if (mid, addr) in self.silent_reads:
                return None
            return status_packet(mid, 0, m.read(addr, size))
        if inst == WRITE:
            addr = params[0] | params[1] << 8
            return status_packet(mid, m.write(addr, params[2:]))
        if inst == REBOOT:
            return status_packet(mid)
        self.errors.append(("unknown instruction", pkt))
        return None


def delta_motors():
    return [FakeMotor(1), FakeMotor(2), FakeMotor(3)]


def pbm_motors():
    """Motor 4 is an XC330-T181 (effort in mA); 5 and 6 are XC430s (%)."""
    return [FakeMotor(4, model=1210, speed=4000),
            FakeMotor(5, model=1080, speed=4000),
            FakeMotor(6, model=1080, speed=4000)]


# Geometry that differs between models; everything else is shared.
# From docs/development.md. config.json in the repo is set up for the 3+.
ROBOTS = {
    "tapster3": {"re": 133.5, "origin": [0, 0, -120], "hop_arc_height": 30},
    "tapster3plus": {"re": 223.5, "origin": [0, 0, -170], "hop_arc_height": 10},
}


def load_config(robot=None, **overrides):
    """firmware/config.json, optionally with a robot model's geometry."""
    config = json.loads((FIRMWARE / "config.json").read_text())
    if robot is not None:
        config.update(ROBOTS[robot])
    config.update(overrides)
    return config


def delta_xyz(goals, config):
    """User (x, y, z) for delta goal positions {1: pos, 2: pos, 3: pos}."""
    from kinematics import DeltaKinematics
    dk = DeltaKinematics(f=config["f"], e=config["e"], rf=config["rf"],
                         re=config["re"])
    angles = []
    for i, mid in enumerate(config["motor_ids"]):
        ticks = (goals[mid] - config["motor_offsets"][i])
        angles.append(ticks * config["motor_direction"][i] * 360.0 / 4096.0)
    x, y, z = dk.forward(*angles)
    ox, oy, oz = config["origin"]
    return (x - ox, y - oy, z - oz)


# -- byte pipes and serial -----------------------------------------------

class Pipe:
    """Thread-safe byte buffer, one direction."""

    def __init__(self):
        self._buf = bytearray()
        self._cv = threading.Condition()

    def write(self, data):
        with self._cv:
            self._buf += data
            self._cv.notify_all()

    def read(self, n, timeout=0):
        with self._cv:
            if not self._buf and timeout:
                self._cv.wait_for(lambda: self._buf, timeout)
            out = bytes(self._buf[:n])
            del self._buf[:n]
            return out

    def available(self):
        with self._cv:
            return len(self._buf)

    def clear(self):
        with self._cv:
            self._buf.clear()


class FakeSerial:
    """The parts of serial.Serial that tapster.py uses."""

    def __init__(self, rx, tx, timeout=None):
        self._rx = rx   # what the host reads
        self._tx = tx   # what the host writes
        self.timeout = timeout
        self.closed = False

    def write(self, data):
        self._tx.write(bytes(data))
        return len(data)

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return self._rx.available()

    def read(self, size=1):
        out = bytearray()
        deadline = time.monotonic() + (self.timeout or 0)
        while len(out) < size:
            left = deadline - time.monotonic()
            chunk = self._rx.read(size - len(out), max(left, 0))
            if not chunk:
                break
            out += chunk
        return bytes(out)

    def readline(self):
        out = bytearray()
        deadline = time.monotonic() + (self.timeout or 0)
        while not out.endswith(b"\n"):
            left = deadline - time.monotonic()
            if left <= 0:
                break
            out += self._rx.read(1, left)
        return bytes(out)

    def reset_input_buffer(self):
        self._rx.clear()

    def close(self):
        self.closed = True


# -- the firmware --------------------------------------------------------

class FirmwareStopped(BaseException):
    """Raised inside the firmware thread to end its main loop. A
    BaseException, so the firmware's `except Exception` can't swallow it."""


class _CommandUart:
    def __init__(self, fw):
        self.fw = fw

    def read(self, nbytes):
        if self.fw._stopping.is_set():
            raise FirmwareStopped
        return self.fw.uart_in.read(nbytes) or None

    def write(self, data):
        self.fw.uart_out.write(bytes(data))
        return len(data)


class Firmware:
    """Runs firmware/code.py in a thread against a FakeDynamixelBus.

    without: firmware modules to hide, e.g. {"pbm"} for a board where
    pbm.py wasn't copied.
    """

    BOOTED = "Home: (0, 0, 0)"

    def __init__(self, bus, tmp_path, config=None, robot=None,
                 board_id="raspberry_pi_pico", without=()):
        self.bus = bus
        self.config = load_config(robot, **(config or {}))
        Path(tmp_path).mkdir(parents=True, exist_ok=True)
        self.config_path = Path(tmp_path) / "config.json"
        self.config_path.write_text(json.dumps(self.config))
        self.board_id = board_id
        self.without = set(without)
        self.console_in, self.console_out = Pipe(), Pipe()
        self.uart_in, self.uart_out = Pipe(), Pipe()
        self.uart_kwargs = []
        self.output = []
        self.error = None
        self._stopping = threading.Event()
        self._booted = threading.Event()
        self._thread = None

    @property
    def text(self):
        return "".join(self.output)

    @property
    def alive(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self, timeout=10):
        sys.modules["board"].board_id = self.board_id
        set_uart_factory(self._make_uart)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        deadline = time.monotonic() + timeout
        while not self._booted.wait(0.01):
            if not self.alive or time.monotonic() > deadline:
                raise RuntimeError("firmware didn't boot:\n" + self.text
                                   + (self.error or ""))
        return self

    def stop(self):
        self._stopping.set()
        if self._thread is not None:
            self._thread.join(5)

    def serial(self, transport="usb", timeout=None):
        if transport == "usb":
            return FakeSerial(self.console_out, self.console_in, timeout)
        if transport == "uart":
            return FakeSerial(self.uart_out, self.uart_in, timeout)
        raise ValueError(transport)

    def _make_uart(self, **kwargs):
        self.uart_kwargs.append(kwargs)
        if kwargs["baudrate"] == self.config["dxl_baudrate"]:
            return self.bus
        return _CommandUart(self)

    def _run(self):
        fw = self
        real_import = builtins.__import__

        class Runtime:
            @property
            def serial_bytes_available(self):
                if fw._stopping.is_set():
                    raise FirmwareStopped
                return fw.console_in.available()

        class Stdin:
            def read(self, n):
                return fw.console_in.read(n).decode()

        fake_sys = types.SimpleNamespace(stdin=Stdin())
        fake_supervisor = types.SimpleNamespace(runtime=Runtime())

        def fw_import(name, globals=None, locals=None, fromlist=(), level=0):
            if name == "sys":
                return fake_sys
            if name == "supervisor":
                return fake_supervisor
            if name in fw.without:
                raise ImportError(f"no module named '{name}'")
            return real_import(name, globals, locals, fromlist, level)

        def fw_print(*args, sep=" ", end="\n", **kwargs):
            text = sep.join(str(a) for a in args) + end
            fw.output.append(text)
            fw.console_out.write(text.encode())
            if fw.BOOTED in text:
                fw._booted.set()

        def fw_open(path, *args, **kwargs):
            if path == "/config.json":
                path = fw.config_path
            return open(path, *args, **kwargs)

        fw_builtins = dict(vars(builtins))
        fw_builtins.update(__import__=fw_import, print=fw_print, open=fw_open)
        path = FIRMWARE / "code.py"
        scope = {"__name__": "__main__", "__file__": str(path),
                 "__builtins__": fw_builtins}
        try:
            exec(compile(path.read_text(), str(path), "exec"), scope)
        except FirmwareStopped:
            pass
        except BaseException:
            self.error = traceback.format_exc()
