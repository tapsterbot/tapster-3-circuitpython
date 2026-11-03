# Dynamixel reference

A summary of the parts of the ROBOTIS docs that this firmware depends on. Values were copied from the e-manual pages below, retrieved 2026-10-04. Re-check them there if a motor behaves unexpectedly.

- XC430-W240-T: https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/
- XC330-T288-T: https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/
- XC330-T181-T: https://emanual.robotis.com/docs/en/dxl/x/xc330-t181/
- Protocol 2.0: https://emanual.robotis.com/docs/en/dxl/protocol2/

## Models

| Model | Model # | Gear ratio | Stall torque @ 12 V | No-load speed @ 12 V | Max baud |
|---|---|---|---|---|---|
| XC430-W240-T | 1080 | 245.22 : 1 | 1.9 N·m (1.4 A) | 70 rpm | 4.5 Mbps |
| XC330-T288-T | 1220 | 288.35 : 1 | 1.00 N·m (0.88 A) | 71 rpm | 4 Mbps |
| XC330-T181-T | 1210 | 180.62 : 1 | 0.80 N·m (0.88 A) | 113 rpm | 4 Mbps |

The PBM (`pbm.py`) also supports the XL430-W250-T, model number 1060 (https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/). Like the XC430, it has Present Load at address 126.

All three motors have 4096 pulses/rev and the same Protocol 2.0. `Dynamixel.ping()` reads the model number from address 0, and the driver uses it to choose how to read address 126.

## Control table: addresses used by the firmware

All three models use the same addresses, sizes and units, except where noted.

| Addr | Size | Name | Unit / notes |
|---|---|---|---|
| 0 | 2 | Model Number | see table above |
| 7 | 1 | ID | 0–252 |
| 8 | 1 | Baud Rate | see below |
| 9 | 1 | Return Delay Time | 2 µs (default 250 = 500 µs) |
| 11 | 1 | Operating Mode | 3 = position (default) |
| 38 | 2 | **Current Limit** | **XC330 only.** 1 mA, default/max 910 |
| 44 | 4 | Velocity Limit | 0.229 rpm. Default **306** (XC430-W240), **320** (XC330-T288), **510** (XC330-T181) |
| 48 / 52 | 4 | Max / Min Position Limit | pulse, 0–4095 |
| 64 | 1 | Torque Enable | 0/1 |
| 65 | 1 | LED | 0/1 |
| 108 | 4 | Profile Acceleration | 214.577 rev/min² |
| 112 | 4 | Profile Velocity | 0.229 rpm, 0 = max |
| 116 | 4 | Goal Position | pulse |
| 122 | 1 | Moving | 0/1 |
| 124 | 2 | Present PWM | 0.113 % |
| 126 | 2 | **Present Load / Present Current** | **Differs by model**, see below |
| 128 | 4 | Present Velocity | 0.229 rpm, signed |
| 132 | 4 | Present Position | pulse, signed |
| 144 | 2 | Present Input Voltage | 0.1 V |
| 146 | 1 | Present Temperature | 1 °C |

`sync_move()` writes 112 and 116 together as one 8-byte block starting at 112.

### Address 126: the only difference that matters

- **XC430: Present Load**, signed, 0.1 % of max torque, range −1000…1000. Positive means CCW. ROBOTIS: "an inferred value based on the internal output value; not a measured value using torque sensor". They recommend it for the direction and rough size of the force, not for measuring torque.
- **XC330: Present Current**, signed, 1 mA. Within ±Current Limit (address 38).

`get_load()` returns a percentage for both. On the XC330 that's Present Current as a percentage of Current Limit. `get_current()` returns raw mA (XC330 only). The two percentages aren't the same physical quantity, so `probe` thresholds tuned on one model may need retuning on the other.

### Velocity Limit and `speed`

How `code.py` sets Profile Velocity (address 112) depends on the command:

- `move` and `home` always use 1023, which is above every model's default Velocity Limit, so the motors run at their Velocity Limit. `speed` only changes the pause between waypoints.
- `hop` works out a Profile Velocity for each motor and each segment, so all three motors reach each waypoint together at the tip speed `speed` asks for (a percentage of `max_speed`). It's capped at 1023.
- `tap` uses `speed_to_velocity()`, a curve from `speed` (1–100) to 1–1023.

At 1023 the XC330-T181 (Velocity Limit 510) moves faster than the others. This hasn't been checked on hardware.

### Baud Rate (address 8)

| Value | Baud | XC430 | XC330 |
|---|---|---|---|
| 0 | 9,600 | ✓ | ✓ |
| 1 | 57,600 (default) | ✓ | ✓ |
| 2 | 115,200 | ✓ | ✓ |
| 3 | 1M | ✓ | ✓ |
| 4 | 2M | ✓ | ✓ |
| 5 | **3M (used here)** | ✓ | ✓ |
| 6 | 4M | ✓ | ✓ |
| 7 | 4.5M | ✓ | — |

## Packets (Protocol 2.0)

```
Instruction: FF FF FD 00 | ID | LEN_L LEN_H | INST | PARAMS... | CRC_L CRC_H
Status:      FF FF FD 00 | ID | LEN_L LEN_H | 55   | ERR | PARAMS... | CRC_L CRC_H
```

LEN counts INST + PARAMS + 2 CRC bytes. ID `0xFE` is broadcast (no reply). Instructions used: PING `0x01`, READ `0x02`, WRITE `0x03`, REBOOT `0x08`, SYNC WRITE `0x83`.
