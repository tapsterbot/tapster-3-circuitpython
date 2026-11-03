# Tutorial: your first 15 minutes with Tapster

In this tutorial you'll set up your computer, connect to a Tapster 3 robot, and control it from Python. By the end you will have:

- moved the robot's tip, and drawn a small square in the air
- drawn the square again, hopping from corner to corner
- sent it back home
- made it tap
- pressed a button with the Push Button Module (PBM), if your robot has one

You don't need any experience with robots. You'll type a few commands into a terminal, then drive the robot from a Python prompt. You don't need a phone either: everything here happens in the air.

You need:

- a Tapster 3 robot, with its power supply and USB cable
- a computer running macOS, Linux or Windows
- about 15 minutes

> **Safety:** the robot moves as soon as it powers up. Before you start, clear the space under the tip and around the PBM's side actuators, and keep your fingers away from the robot while it moves.

## 1. Install Python

Open a terminal. On macOS, open **Terminal** from Applications › Utilities. On Windows, open **PowerShell** from the Start menu.

Check that you have Python 3.9 or newer:

```bash
python3 --version
```

On Windows, type `py --version` instead.

You should see something like `Python 3.12.4`. If you get an error, or a version below 3.9, install Python from [python.org/downloads](https://www.python.org/downloads/), then close and reopen the terminal and check again.

## 2. Get the Tapster software

Put the Tapster folder somewhere on your computer, for example in your home folder. It's the folder that contains `tapster.py` and this `docs` folder. If you downloaded it as a `.zip` file, unzip it first.

In the terminal, go into that folder. For example, if it's called `tapster` in your home folder:

```bash
cd ~/tapster
```

Now create a private Python environment for Tapster and install the one library it needs, `pyserial`:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial
```

On Windows:

```powershell
py -m venv .venv
.venv\Scripts\activate
pip install pyserial
```

Your terminal prompt now starts with `(.venv)`. That means the environment is active. You'll need to run the `activate` line again whenever you open a new terminal.

## 3. Connect the robot

1. Plug in the robot's power supply.
2. Connect the robot to your computer with its USB cable.

The robot's arms move into their home position. Wait until they stop.

## 4. Start Python with the robot connected

In the same terminal, in the Tapster folder, run:

```bash
python tapster.py -i
```

On Windows, you need to name the robot's port. Find it in **Device Manager › Ports (COM & LPT)**; it looks like `COM3`. Then run `python tapster.py -i --port COM3`, with your port's name.

You'll see:

```
Connected: /dev/cu.usbmodem2101
>>>
```

Your port name may be different. The `>>>` is a Python prompt, and the robot is ready as `t`. Everything you type at `>>>` from now on is Python.

## 5. Say hello

Ask the robot's three motors to answer:

```python
>>> t.ping()
{'motors': [{'model': 1080, 'name': 'XC430-W240', 'firmware': 50}, {'model': 1080, 'name': 'XC430-W240', 'firmware': 50}, {'model': 1080, 'name': 'XC430-W240', 'firmware': 50}], 'ok': True}
```

`'ok': True` means the robot understood. You'll see one entry per motor. Your motor names may differ.

Now ask where the tip is:

```python
>>> t.position
(0, 0, 0)
```

That's home. Positions are `(x, y, z)` in millimetres, and home is `(0, 0, 0)`, in the middle of the robot's reach.

## 6. Move the tip

Move the tip 10 mm along x:

```python
>>> t.move(x=10)
{'target': {'z': 0, 'y': 0, 'x': 10}, 'ok': True}
```

The tip slides sideways and stops. The items in a reply can come back in any order; what matters is `'ok': True`.

Now draw a 20 mm square, one corner at a time. Type each line and press Enter. Each one prints a reply like the one above; wait for it before typing the next line:

```python
>>> t.move(x=-10, y=10, speed=30)
>>> t.move(x=10, y=10, speed=30)
>>> t.move(x=10, y=-10, speed=30)
>>> t.move(x=-10, y=-10, speed=30)
>>> t.move(x=-10, y=10, speed=30)
```

The tip traces a square in the air. `speed` is a percentage from 1 to 100, where 100 is the fastest. `move` uses 100 unless you say otherwise, so `speed=30` makes the square a little slower.

To see a command's options at the prompt, ask Python: `help(t.move)`. Press `q` to get back to `>>>`.

Check where it ended up:

```python
>>> t.position
(-10, 10, 0)
```

## 7. Hop

A hop lifts the tip, carries it over in an arc, and puts it down somewhere else. It's how the robot gets around without dragging across a screen.

The tip is at the square's top-left corner. Hop to the top-right corner:

```python
>>> t.hop(x=10, y=10)
{'target': {'z': 0, 'y': 10, 'x': 10}, 'ok': True}
```

Now hop round the rest of the square, one line at a time:

```python
>>> t.hop(x=10, y=-10)
>>> t.hop(x=-10, y=-10)
>>> t.hop(x=-10, y=10)
```

The tip bounces from corner to corner and ends where it started:

```python
>>> t.position
(-10, 10, 0)
```

## 8. Go home

```python
>>> t.home()
{'target': {'z': 0, 'y': 0, 'x': 0}, 'ok': True}
```

The tip returns to the middle.

## 9. Tap

```python
>>> t.tap()
{'tap': {'z': -10, 'y': 0, 'x': 0}, 'ok': True}
```

The tip dips 10 mm and comes back up. That's a tap. With a phone under the tip, it would press the screen.

## 10. Press a button with the PBM

The Push Button Module (PBM) is a set of up to three actuators mounted beside the device, for pressing its side buttons. Its motors are numbered 4, 5 and 6. If your robot has no PBM, you're done: skip to the end.

Check which PBM motors are there:

```python
>>> t.pbm.ping()
[{'found': True, 'id': 4, 'name': 'XC430-W240-T'}, {'found': True, 'id': 5, 'name': 'XC430-W240-T'}, {'found': True, 'id': 6, 'name': 'XC430-W240-T'}]
```

Find motor 4 by blinking its light:

```python
>>> t.pbm.blink(4, times=3)
```

A small LED on one of the PBM's motors blinks three times. That's motor 4.

Now press with it:

```python
>>> t.pbm.tap(4, from_position=2000, to_position=1000)
MoveResult(position=1003, contact=False, final_effort=0.4, peak_effort=13.2, effort_unit='%')
```

Motor 4 presses and comes back. The positions are the motor's own units, not millimetres. It starts from 2000 and pushes toward 1000. If it meets a button on the way, it stops there instead and shows `contact=True`. Your numbers will be a little different.

When you've finished, leave the Python prompt:

```python
>>> exit()
```

## What you've done

You installed the Tapster client, connected to the robot with `python tapster.py -i`, and drove it with `t.move`, `t.hop`, `t.home`, `t.tap` and `t.pbm.tap`. That's the core of driving a Tapster: one command at a time, each answered by a reply.

Next:

- Start the prompt again any time with `python tapster.py -i`, from the Tapster folder with the environment active (step 2).
- To run the same commands from your own Python script, see [Python Client](../README.md#python-client) in the README.
- To tap a real phone, the robot needs to know how high the screen is. See `probe` and `surface` in the [README](../README.md#coordinates-and-heights).
- All the commands, and their options, are listed in the [README](../README.md#commands).

## If something goes wrong

- **`No serial port found. Is the board connected?`** Check the USB cable, and use a cable that carries data, not just power. On Windows, always give `--port`.
- **`could not open port` or `Resource busy`:** another program is using the robot's port. Close any other Tapster prompt or serial terminal, then try again.
- **Linux: `Permission denied: '/dev/ttyACM0'`:** your user isn't allowed to use serial ports yet. Run `sudo usermod -aG dialout $USER`, log out and back in, then try again.
- **Linux: `python3 -m venv` says `ensurepip is not available`:** install the venv package (on Debian or Ubuntu, `sudo apt install python3-venv`), then repeat step 2.
- **Windows: `activate` fails with "running scripts is disabled on this system":** run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once in PowerShell, then run the `activate` line again.
- **`ModuleNotFoundError: No module named 'serial'`:** the environment isn't active. Run the `activate` line from step 2 again.
- **`can't open file ... tapster.py`:** you're in the wrong folder. `cd` into the Tapster folder, the one with `tapster.py` in it.
- **`t.ping()` returns `None`, or nothing moves:** check the robot's power supply is plugged in, then unplug the USB cable, plug it back in, and start again from step 4.
- **A PBM command raises `RuntimeError: ... No valid response from motor 4`:** that PBM motor isn't connected. Check `t.pbm.ping()` to see which motors are there.
