# Development

This guide covers working on the Tapster 3 firmware: setting up, deploying, configuration, hardware and debugging. If you just want to use the robot, see the [README](../README.md).

The firmware is CircuitPython and runs on an ESP32 DevKitC V4 (WROOM-32D/32E) or a Raspberry Pi Pico (RP2040) on a carrier board. The same files run on both, and the board is detected automatically. The motors are Dynamixel XC430-W240-T, XC330-T288-T or XC330-T181-T (Protocol 2.0). An optional Push Button Module (PBM) adds up to three more motors on the same bus.

## Setup

```bash
brew install tio
make install                 # creates .venv with pyserial, pytest and ampy
source .venv/bin/activate
```

Patch ampy for CircuitPython compatibility (it's only needed for the ESP32):

```bash
sed -i '' 's/ubinascii/binascii/g' .venv/lib/python3.*/site-packages/ampy/files.py
```

## Files

On the board (`firmware/`):

- `code.py`: main loop. Homes on boot and listens for JSON commands.
- `config.json`: robot dimensions, bus baud rates, motor IDs and limits, PBM motor IDs, origin, heights, speeds and the default hop arc.
- `config.json.t3`, `config.json.t3plus`: example `config.json` for each model (see [Configuration](#configuration)). Not deployed.
- `hardware.py`: pin mapping for each board (ESP32 / Pico) and UART setup.
- `dynamixel.py`: Dynamixel Protocol 2.0 driver with sync write.
- `kinematics.py`: delta robot inverse and forward kinematics.
- `motion.py`: linear interpolation, parabolic arcs and easing functions.
- `pbm.py`: Push Button Module. A port of [valet-pbm](https://github.com/tapsterbot/valet-pbm) to `dynamixel.py`.

On your computer:

- `tapster.py`: Python client (see the README).
- `deploy.sh`: uploads `firmware/` to the board.
- `Makefile`: `make test`, `make test-hw`, `make deploy`. Run `make` to list targets.
- `tests/`: see [Tests](#tests).

Docs, by [Diataxis](https://diataxis.fr/) type:

- Tutorial: [`docs/tutorial.md`](tutorial.md), a beginner's first 15 minutes with the robot.
- Reference: the [README](../README.md) (commands and the Python client) and [`docs/dynamixel.md`](dynamixel.md) (control-table addresses, model numbers and the XC430/XC330 differences, with links to the ROBOTIS e-manual).
- How-to and reference for working on the code: this guide.
- Explanation: [`docs/circuitpython-uart.md`](circuitpython-uart.md), why the DIR pin is handled differently on the ESP32 and the Pico, with CircuitPython source excerpts.

## Deploying

```bash
./deploy.sh                         # auto-detect
./deploy.sh /dev/cu.usbserial-210   # ESP32, over serial with ampy
./deploy.sh /Volumes/CIRCUITPY      # Pico, copied to the USB drive
```

The ESP32 has no native USB, so files go over serial with ampy. The Pico mounts a `CIRCUITPY` drive, so files are just copied (ampy can't write there). If `CIRCUITPY` is mounted, `deploy.sh` uses it. Otherwise it looks for a `usbserial` port.

Deploying copies `config.json` too, which overwrites any heights saved on the board.

Or manually (ESP32):

```bash
for f in config.json dynamixel.py kinematics.py motion.py hardware.py pbm.py code.py; do
    ampy --port /dev/cu.usbserial-210 put firmware/$f
done
```

## Configuration

`firmware/config.json` is the configuration that gets deployed. Next to it are complete examples for each model:

| File | Robot |
|---|---|
| `firmware/config.json.t3` | Tapster 3 |
| `firmware/config.json.t3plus` | Tapster 3+ |

To set up a robot, copy its example over `config.json`, then deploy:

```bash
cp firmware/config.json.t3 firmware/config.json
./deploy.sh
```

The repo's `config.json` is currently the Tapster 3+ example. The two examples differ only in these keys:

| Key | Tapster 3 | Tapster 3+ |
|---|---|---|
| `robot` | `"tapster3"` | `"tapster3plus"` |
| `re` | `133.5` | `223.5` |
| `origin` | `[0, 0, -120]` | `[0, 0, -170]` |
| `hop_arc_height` | `30` | `10` |

`make test` checks that both examples have the same keys as `config.json`, and differ only in those four. Every key:

- `robot`: a label, `tapster3` or `tapster3plus`. The firmware doesn't read it.
- `e`, `f`, `rf`, `re`: effector and base triangle sides, upper and lower arm lengths, in mm. Only `re` differs by model.
- `dxl_baudrate`: motor bus speed. It must match the motors' Baud Rate register.
- `cmd_baudrate`: speed of the command UART on the header pins. USB serial is separate.
- `motor_min`, `motor_max`: allowed motor positions. `move` and `hop` refuse any point that needs a motor outside them. `tap`, `probe` and the startup move home clamp to them instead.
- `origin`: kinematic position that becomes the user `(0, 0, 0)`.
- `speed`: default `speed` for `move` and `home` (1–100).
- `max_speed`: tip speed in mm/s for `hop` at `speed` 100. `speed` scales it linearly. `move` doesn't use it yet.
- `hop_arc_height`: default `hop` arc in mm. 30 if missing. The Tapster 3+ uses 10 because it reaches only about 18 mm above home.
- `safety_height`, `surface_height`: the tapping heights (see the README). `probe` and `surface` update them.
- `pbm_ids`: Dynamixel IDs of the PBM motors. PBM commands refuse any other ID. Defaults to `[4, 5, 6]` if missing.
- `board`, `pins`: optional overrides (see [Pinout](#pinout)).

### Calibration

`motor_offsets` is the servo position (1024–3072) at which each arm is horizontal (0°). `motor_direction` (1 or -1) sets each motor's rotation direction. Adjust them per motor until the robot homes correctly.

## Hardware

### Device

```
ESP32: /dev/cu.usbserial-210
Pico:  /dev/cu.usbmodem*   (drive: /Volumes/CIRCUITPY)
```

### Pinout

| Function           | ESP32 | Pico GPIO (physical pin) |
|--------------------|-------|--------------------------|
| Dynamixel TX       | IO4   | GP4 (6)  — UART1 TX      |
| Dynamixel RX       | IO13  | GP5 (7)  — UART1 RX      |
| Dynamixel DIR      | IO17  | GP6 (9)                  |
| Host TX (board RX) | IO32  | GP1 (2)  — UART0 RX      |
| Host RX (board TX) | IO33  | GP0 (1)  — UART0 TX      |

The Pico carrier board routes each Pico pin to the DevKitC header position of the matching ESP32 pin. On the RP2040, a UART's TX and RX must be on the same UART instance, which is why the Pico numbers differ from the ESP32 ones.

Pins live in `firmware/hardware.py`. To override any of them without editing code, add a `pins` object to `config.json` (only the keys you list change). You can also force the profile with `board`:

```json
"board": "pico",
"pins": {"dxl_dir": "GP17"}
```

On the Pico, the UART drives the DIR pin itself (`rs485_dir`). The ESP32 keeps the GPIO toggle. See [circuitpython-uart.md](circuitpython-uart.md) for why.

### Saving config on the Pico

While `CIRCUITPY` is mounted over USB, it is read-only to the firmware. `probe` and `surface` still update the heights in memory, but they can't save them to `config.json`. The `surface` response then includes `"saved": false`. Copy the values into `firmware/config.json` and redeploy.

### Motor settings

- Baud rate: 3000000 (`dxl_baudrate`)
- Return Delay Time: 250 (500 µs)
- Motor IDs: 1, 2, 3
- Position range: 1024–3072 (90°–270°)
- Model: XC430-W240-T, XC330-T288-T or XC330-T181-T (detected by ping)

PBM motors (IDs 4, 5, 6) need the same Baud Rate and Protocol 2.0, Operating Mode 3 (position) and Status Return Level 2. Return Delay Time can be 250 like the delta motors (valet-pbm's 254 also works). Every motor on the bus needs a unique ID. The PBM also supports the XL430-W250-T.

The three models share the control table except at address 126. On the XC430 it's Present Load (0.1 %). On the XC330 it's Present Current (1 mA). `get_load()` returns a percentage for both. On the XC330 it's Present Current as a percentage of Current Limit (address 38, default 910 mA). The two percentages aren't physically identical, so the `probe` threshold may need retuning on XC330 motors. `get_current()` returns raw mA on XC330s. Details in [dynamixel.md](dynamixel.md).

## Tests

```bash
make test                          # unit + integration, no hardware, ~20 s
make test-hw                       # on a connected robot: MOVES IT
make test-hw PORT=/dev/cu.usbmodem2101 PBM_TO_POSITION=1200
make test ARGS="-k hop -v"         # extra pytest arguments
```

- `tests/unit/`: each firmware module and the client on its own. `dynamixel.py` and `pbm.py` run against simulated motors that speak Protocol 2.0 bytes (CRC checked independently of the driver's table).
- `tests/integration/`: the real `code.py`, main loop included, with the real `hardware.py`, `dynamixel.py` and `pbm.py`, driven by the real `tapster.py` client over a fake serial port (both the USB console and the command UART). Runs as a Pico and as an ESP32, with Tapster 3 and 3+ geometry. Every test also checks that the firmware's main loop is still running and that it sent only valid packets.
- `tests/integration/test_tutorial.py`: runs every `>>>` step of [the tutorial](tutorial.md) against the simulated robot, so the tutorial can't silently go out of date.
- `tests/hardware/test_e2e.py`: draws a small square at `safety_height` with straight moves, then again with hops, taps once at home (down to `surface_height`), then taps each PBM motor from `PBM_FROM_POSITION` (default 2000) to `PBM_TO_POSITION` (default 1000) and checks it was released. Motors listed in `PBM_PARK` (default `5:1500`) are then moved to that position. Skipped unless run with `--hardware`, which `make test-hw` does. Also reads `TAPSTER_PORT`, `SQUARE_SIZE` and `PBM_IDS`.

If a known bug can't be fixed straight away, mark its test `xfail`: `make test` then shows it as `x` and still succeeds. The marks are strict, so once the bug is fixed and the test passes, pytest reports a failure until the mark is removed.

`tests/fakes.py` has the simulated motors, the fake serial port and the firmware runner. The simulated bus records every packet, so tests can check exactly what reached the motors.

## Serial Console and Board REPL

```bash
tio -e /dev/cu.usbserial-210   # ESP32
tio -e /dev/cu.usbmodem*       # Pico
```

- **JSON console:** while `code.py` is running, type a JSON command and press Enter. `-e` turns on local echo, because the firmware doesn't echo input.
- **CircuitPython REPL:** press `Ctrl+C` to stop `code.py`, then any key to get the `>>>` prompt. Turn local echo off with `Ctrl+T` then `e`, because the REPL echoes for itself.
- `Ctrl+D` at the `>>>` prompt soft-reloads and restarts `code.py`.
- Exit tio with `Ctrl+T` then `Q`. Close it before using ampy or `tapster.py`.

### REPL quick start

At the board's `>>>` prompt:

```python
from hardware import dynamixel_bus

dxl = dynamixel_bus()          # pins and dxl_baudrate from /config.json

dxl.ping(1)                    # {'model': 1080, 'name': 'XC430-W240', 'firmware': 50}
dxl.set_led(1, True)
dxl.get_position(1)            # 2048
dxl.get_temperature(1)         # 36
dxl.get_load(1)                # 0.0 (%)
dxl.get_current(1)             # 12 (mA, XC330 only)
dxl.get_voltage(1)             # 120 (12.0V)
dxl.get_profile_velocity(1)    # 100
```

The PBM, on the same bus:

```python
from hardware import dynamixel_bus
from pbm import PBM

pbm = PBM(dynamixel_bus())     # ids=(4, 5, 6) by default
pbm.ping(4)                    # True
pbm.get_model_name(4)          # 'XC330-T181-T'
pbm.get_position(4)            # 2048
pbm.get_effort(4)              # (12.0, 'mA')
pbm.tap(4, from_position=2048, to_position=1700)
# MoveResult(position=1852, contact=True, final_effort=210.0, peak_effort=403.0, effort_unit='mA')
```

Contact detection can be tuned with `PBM(...)` options: `poll_interval`, `contact_samples`, `contact_ignore_time`, `contact_window`, `contact_min_progress`, `contact_min_travel`. They mean the same as in valet-pbm. Retries and the DIR pin are handled by `dynamixel.py` and `hardware.py`, so valet-pbm's Raspberry Pi transport options (`device`, `direction_pin`, `response_timeout_ms`, `retry_delay`) don't exist here.

Using kinematics and motion directly:

```python
from kinematics import DeltaKinematics
from motion import interpolate

dk = DeltaKinematics(f=173.205, e=86.6025, rf=70.0, re=133.5)
dk.inverse(0, 0, -120)        # (theta1, theta2, theta3)
dk.forward(0, 0, 0)           # (x, y, z)

for x, y, z in interpolate((0, 0, 0), (30, 30, 0), steps=20, easing="ease_in_out"):
    print(f"{x:.1f}, {y:.1f}, {z:.1f}")
```

### Raw packets

Packets use the Protocol 2.0 format: `FF FF FD 00 [ID] [LEN_L] [LEN_H] [INST] [PARAMS...] [CRC_L] [CRC_H]`.

```python
import time
from hardware import dynamixel_bus
from dynamixel import _crc16

dxl = dynamixel_bus()

def send(raw):
    pkt = bytearray.fromhex(raw)
    crc = _crc16(pkt)
    pkt.append(crc & 0xFF)
    pkt.append((crc >> 8) & 0xFF)
    dxl._send(pkt)             # handles the DIR pin on either board
    time.sleep(0.1)
    resp = dxl.uart.read(256)
    print([hex(b) for b in resp] if resp else None)

send("FF FF FD 00 01 03 00 01")              # ping motor 1
send("FF FF FD 00 01 06 00 03 41 00 01")     # LED on
send("FF FF FD 00 01 06 00 03 41 00 00")     # LED off
```
