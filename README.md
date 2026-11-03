# Tapster 3

Tapster 3 is a delta robot for tapping phone and tablet screens. It comes with its firmware already installed. You control it by sending simple JSON commands over USB serial, either with the included Python client or from any language that can open a serial port.

Updating the firmware or working on the code? See [docs/development.md](docs/development.md).

## Quick Start

**1. Plug the robot into your computer over USB.** It moves to its home position when it powers up. Its serial port shows up as:

| Robot's board | macOS | Linux | Windows |
|---|---|---|---|
| ESP32 | `/dev/cu.usbserial-*` | `/dev/ttyUSB*` | `COM*` |
| Raspberry Pi Pico | `/dev/cu.usbmodem*` | `/dev/ttyACM*` | `COM*` |

**2. Install the Python client's one dependency** (from this folder):

```bash
python3 -m venv .venv
source .venv/bin/activate    # run this in every new terminal
pip install pyserial
```

**3. Try it.** `-i` connects, then gives you a Python prompt with the robot available as `t`:

```bash
python tapster.py -i
```

```
Connected: /dev/cu.usbserial-210
>>> t.ping()
>>> t.move(x=20, y=20)
>>> t.tap()
>>> t.home()
>>> exit()
```

The port is found automatically on macOS and Linux. Otherwise, or if more than one board is plugged in, give it explicitly: `python tapster.py -i --port COM3`.

Only one program can use the serial port at a time. Close any serial terminal before running `tapster.py`, and vice versa.

## Python Client

`tapster.py` is a single file with no dependencies besides `pyserial`. Copy it into your own project, or import it from here:

```python
from tapster import Tapster

t = Tapster()                      # or Tapster(port="/dev/cu.usbmodem1101")
t.move(x=10, y=10)
t.move(x=0, y=0, easing="ease_in_out_cubic")
t.hop(x=30, y=30)
t.tap()                            # tap the screen
t.tap(dx=-5, dz=0)                 # push sideways, e.g. a side button
t.status()                         # {'ok': True, 'xyz': {...}}
t.surface_height = -12             # see Heights below
t.home()
t.close()
```

From the command line, `python tapster.py` (without `-i`) pings the motors, prints the robot's position and exits. It's a quick check that everything is connected.

## Using Another Language

Anything that can open a serial port can drive the robot:

- **Serial settings:** 115200 baud, 8N1. The Pico ignores the baud rate, but setting it doesn't hurt.
- **Send:** one JSON object per line, ending with `\n`.
- **Receive:** one JSON line in reply, `{"ok": true, ...}` or `{"error": "..."}`. The robot also prints log messages such as startup info and `probe` progress, so skip any line that doesn't start with `{`.
- **One command at a time:** wait for each reply before sending the next command. The reply comes after the motion has been sent to the motors.

For example, in Python without the client:

```python
import json, serial

ser = serial.Serial("/dev/cu.usbserial-210", 115200, timeout=5)
ser.write(b'{"cmd":"tap"}\n')
for _ in range(20):
    line = ser.readline().decode().strip()
    if line.startswith("{"):
        print(json.loads(line))
        break
```

To type commands by hand, use any serial terminal. For example, with [tio](https://github.com/tio/tio) (`brew install tio`):

```bash
tio -e /dev/cu.usbserial-210     # -e shows what you type
{"cmd":"tap"}
{"ok": true, "tap": {"x": 0, "y": 0, "z": -10}}
```

Exit tio with `Ctrl+T`, then `Q`.

## Coordinates and Heights

Positions are in millimetres. `(0, 0, 0)` is the home position, at the centre of the robot's reach. Positive Z is up, and negative Z is down toward the screen. The usable area is roughly ±50 mm in X and Y.

Two heights control tapping:

- `safety_height`: where the tip travels between taps (default 0).
- `surface_height`: where a tap presses down to (default -10, a 10 mm press).

Set them by hand with the `surface` command, or measure them with `probe`. `probe` lowers the tip until it feels the screen, then sets `surface_height` there and `safety_height` 10 mm above it. On an ESP32 the heights are saved on the robot. On a Pico they aren't saved and last only until the robot restarts. The `surface` reply then includes `"saved": false`.

## Commands

All parameters are optional unless noted. `speed` is a percentage from 1 (very slow) to 100 (fastest).

### move

Moves in a straight line to `x`, `y`, `z`.

```json
{"cmd":"move","x":10,"y":10}
{"cmd":"move","x":30,"y":30,"z":-20,"speed":50}
{"cmd":"move","x":10,"y":10,"easing":"ease_in_out_cubic","step_size":2.0}
```

- `x`, `y` default to 0, and `z` defaults to `safety_height`.
- `easing` sets how the motion speeds up and slows down: `linear` (default), `ease_in`, `ease_out`, `ease_in_out`, `ease_in_cubic`, `ease_out_cubic` or `ease_in_out_cubic`.
- `step_size` is the spacing between waypoints in mm (default 5). Smaller values give smoother motion.

#### Example: draw a square

Traces a 100 mm square, slowly. Send each line after the previous one replies.

```json
{"cmd":"move","x":-50,"y":50,"z":0,"speed":5}
{"cmd":"move","x":50,"y":50,"z":0,"speed":5}
{"cmd":"move","x":50,"y":-50,"z":0,"speed":5}
{"cmd":"move","x":-50,"y":-50,"z":0,"speed":5}
{"cmd":"move","x":-50,"y":50,"z":0,"speed":5}
```

### hop

Lifts up, travels in a smooth arc, and comes down at `x`, `y`, `z`. Use it to get from one spot to another without dragging across the screen.

```json
{"cmd":"hop","x":30,"y":30}
{"cmd":"hop","x":0,"y":0,"z":-20,"arc_height":50,"easing":"linear"}
```

- `z` defaults to `safety_height`, and `speed` defaults to 50.
- `arc_height` is how far above the start and end the arc peaks, in mm (default 30).
- `easing` defaults to `ease_in_out`. The options are the same as for `move`.

### tap

Moves to `safety_height` above the current position (or above `x`, `y` if given), presses down, then comes back up.

```json
{"cmd":"tap"}
{"cmd":"tap","x":20,"y":-15}
{"cmd":"tap","dz":-5}
{"cmd":"tap","dx":-5,"dz":0}
```

- Without `dz`, the tap presses down to `surface_height`.
- With `dz`, it presses to `safety_height + dz` instead.
- `dx`, `dy` shift sideways during the press. `{"cmd":"tap","dx":-5,"dz":0}` pushes 5 mm sideways without going down, e.g. for a side button.
- `speed` defaults to 75.

### home

Moves back to `(0, 0, 0)`. Optional `speed`.

```json
{"cmd":"home"}
```

### surface

Reads or sets the tapping heights (see [Coordinates and Heights](#coordinates-and-heights)).

```json
{"cmd":"surface"}
{"cmd":"surface","surface_height":-12,"safety_height":-2}
```

### probe

Measures the screen height by lowering the tip from home until the motors feel resistance, then sets both heights. Clear the area under the tip first.

```json
{"cmd":"probe"}
{"cmd":"probe","threshold":5.0,"step":1.0,"max_depth":80}
```

- `threshold` is the motor load that counts as touching, in % (default 5). Lower it if the probe pushes too hard, and raise it if it stops early.
- `step` is how far to lower each step, in mm (default 1).
- `max_depth` is how far to go before giving up, in mm (default 80).

The robot prints progress lines while probing, then a JSON reply.

### status

Returns the current position.

```json
{"cmd":"status"}
```

### ping, load, temperature

Check on the three motors. These return a list with one entry per motor.

```json
{"cmd":"ping"}
{"cmd":"load"}
{"cmd":"temperature"}
```

- `ping` returns each motor's model and firmware version.
- `load` returns each motor's effort, in %.
- `temperature` returns each motor's temperature, in °C.

### enable, disable

Turns motor power on or off. When disabled, the arms can be moved by hand.

```json
{"cmd":"enable"}
{"cmd":"disable"}
```

### blink

Blinks one motor's LED, which is handy for checking which motor is which.

```json
{"cmd":"blink","id":1,"count":3}
```
