# Raspberry Pi robot arm control

## Quick start

After installing dependencies, connect a USB keyboard to the Pi and run from
the repository root:

```sh
python raspberry_pi/control_arm.py
```

This immediately reads the keyboard in **dry-run** mode. Press **E** to enable,
tap **W/S, arrows, A/D, J/L** to move continuously, press **Space**
to stop, and press **Q** to exit. Another direction replaces the current motion. For real motors, complete
calibration below and run `python raspberry_pi/control_arm.py --live`.
The keyboard is detected automatically; no selection step is required.

## Hardware and kinematics

Python controller for this robot, a Waveshare **Servo Driver with ESP32**, and
**STS3215** motors through the vendor `scservo_sdk` library.

Configured connection: **`/dev/ttyUSB0`, 115200 baud**. Motor IDs:

For direct keyboard control from the website with a live simulated view,
see [WEB_SERIAL.md](WEB_SERIAL.md). It uses the same calibration but connects
through the browser instead of this Python process. Export the profile with
`python raspberry_pi/export_browser_profile.py` after calibration is complete.

| Simulation joint | Servo ID |
| --- | --- |
| Base yaw | 1 |
| Shoulder | 2 |
| Elbow | 3 |
| Wrist pitch | 4 |
| Gripper | 5 |

The Python kinematics read `../dist/robot/engg1101.xml` and `model.json` directly.
The FK transforms, couplings, displayed angles, position IK and pitch-holding
jog equations match the browser. NumPy evaluates the transforms; MuJoCo and the
3D meshes do not need to be installed on the Pi. This is specific to the current
hinge/slide model, not a general MJCF engine.

**Calibration values are prefilled from your existing arm project.** The code is tested against browser
MuJoCo results and your supplied SDK with an emulated serial device, not your physical arm. It deliberately
starts in dry-run mode. `--live` uses the prefilled mapping.

## 1. Install on the Pi

From the repository root:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r raspberry_pi/requirements.txt
```

Use the Waveshare SDK already working with your motors. The expected import is:

```python
from scservo_sdk import PortHandler, sms_sts
```

The adapter uses `GroupSyncRead`, `ReadPos`, `SyncWritePosEx`,
`groupSyncWrite.txPacket` and `read1ByteTxRx`. If your package has a different import name but provides this
same API, edit `SDK_MODULE` in `config.py`. Older `PacketHandler` APIs are not
interchangeable with `sms_sts` without an adapter.

If needed, obtain the official SDK from [Waveshare's Python download](https://files.waveshare.com/wiki/Bus_Servo_Adapter_A/STServo_Python.zip).
Use the **`scservo_sdk` directory**, not its bundled Windows virtual environment.
Keep that SDK outside this repository. Your existing SDK is located automatically
using `SDK_PARENT_DIR`. If you move it, update that setting. Alternatively,
for a different temporary SDK location:

```sh
export PYTHONPATH="/absolute/path/to/vendor-sdk-parent:$PYTHONPATH"
python -c "from scservo_sdk import PortHandler, sms_sts; print('SDK ready')"
```

Enable **Start Serial Forwarding** on the ESP32 board. Its host connection uses
115200 baud in forwarding mode; this does not mean changing the servo bus baud.
See the [Waveshare controller manual, section 4.2](https://files.waveshare.com/upload/d/d4/Servo_Driver_with_ESP32_User_Manual.pdf).
The script does not flash firmware, change IDs, rewrite EEPROM, change servo
mode, or turn torque on automatically. Serial forwarding and position mode
must already be working.

### Checked against your supplied examples

The supplied `read_write.py` uses **115200 baud and `/dev/ttyUSB1`**;
`sync_read_write.py` uses **1000000 baud and `/dev/ttyUSB0`**, addressing IDs 1–10.
This controller keeps your requested **115200, `/dev/ttyUSB0`, IDs 1–5**.
It sends bounded, calibrated targets instead of the examples' full 0–4095 sweep.

The supplied `GroupSyncRead.isAvailable()` returns `(available, error)`, not a
boolean. Its `txRxPacket()` can report success when an earlier ID has a missing
or corrupt reply but the last ID succeeds. The adapter checks each complete
11-byte reply, its error byte and the group's completion flag, and clears
cached responses before and after every read. Communication failures never
automatically switch to a different read method. `SYNC_READ=False` is the default, matching the individual `ReadPos` calls
in your working arm project. Set it to `True` only for verified sync-read support.

The SDK's `setBaudRate()` already opens the port. The controller calls it once
at the requested baud, avoiding the example's initial open at the SDK default.
The original SDK and example files are left unchanged.

## 2. Existing arm settings

`config.py` now includes the settings from your local
`STS3215-5DOF-Robot-Arm/arm_settings.py`:

- IDs 1–5, `/dev/ttyUSB0`, 115200 baud.
- Individual position reads (`SYNC_READ=False`), matching `arm_helpers.py`.
- Zero encoder positions: 2048 for every servo.
- Encoder scale: 4096/360 ticks per degree; negative for the gripper.
- Source travel limits intersected with the simulation CAD limits.

`wrist` maps to `wrist_pitch`; `gripper` maps to `gripper_drive`.
We assume the source angles use the simulation display convention and your
hardware middle positions are already calibrated, as requested. The source
labels its values as defaults; no additional gearing ratios were found.
Motor speed is capped at the source value of 3000 ticks/s, with acceleration 254.
For jogging, each motor speed is calculated from its target distance and update
interval so it travels between targets instead of rushing to each small step.
Joint setpoints are capped at 30°/s. Jogging uses the full simulation step rate,
with a 40 ms target update period. Actual updates depend on serial and IK timing.
Serial transactions have a 3 ms forwarding gap and discard stale buffered
bytes before fresh feedback requests. Corrupt fresh replies still stop control.
These values are copied, so future source edits do not update this file automatically.

Your SDK location is saved in `SDK_PARENT_DIR` in `config.py`. No
`PYTHONPATH` export is needed. From the repository root:

```sh
python raspberry_pi/control_arm.py --live
```

### Reference values and future adjustments

The first editable block is `RAW_AT_REFERENCE`, one entry for each of IDs 1–5.
Those values mean “the raw encoder position at this known joint angle,” **not**
“move to this position when the program starts.” Startup and re-enabling read
the arm's measured position instead.

Use the read-only helper to display current raw positions:

```sh
python raspberry_pi/read_positions.py
python raspberry_pi/read_positions.py --watch
# If Linux assigned your adapter USB1, override the port for this run:
python raspberry_pi/read_positions.py --port /dev/ttyUSB1 --baudrate 115200
```

For each joint, measure two known poses using the website's calibrated joint
angle convention. With raw readings `rA`, `rB` at angles `aA`, `aB`:

```text
RAW_AT_REFERENCE          = rA
REFERENCE_DEGREES         = aA
TICKS_PER_JOINT_DEGREE    = (rB - rA) / (aB - aA)
raw_command              = rA + (requested_degrees - aA) * TICKS_PER_JOINT_DEGREE
```

The signed scale includes servo direction and gearing. A negative scale is
valid. Do not automatically assume 4096/360 ticks per arm degree: the base gear
and gripper linkage change that relationship. Gripper +90 is open and -90 is
closed; these are calibrated opening coordinates, not the motor shaft angle.
Calibration must stay on a continuous encoder interval without wrapping across
4095 to 0. This version does not support multi-turn position mode or wraparound.

Set `SAFE_JOINT_DEGREES` to tested physical ranges, leaving clearance from hard
stops, and narrow `RAW_LIMITS` if appropriate. Allowed CAD/display ranges are:

| Joint | CAD/display range (degrees) |
| --- | --- |
| Base yaw | -180 to +180 |
| Shoulder | -102 to +90 |
| Elbow | -90 to +104 |
| Wrist pitch | about -117 to +90 |
| Gripper | -90 to +90 |

Use a narrower range if the motor, gearing, cabling or assembly requires it.
The script rejects settings outside the CAD limits or the raw servo range.
The configuration assumes your arm is already calibrated and prepared:
`CALIBRATION_CONFIRMED` and `POSITION_MODE_CONFIRMED` default to `True`.
Your existing mapping values are already entered above. It checks STS position mode and existing
torque-enable state before motion. Support the arm while preparing it with the
vendor tools; this script never enables torque against an old servo target.

## 3. Test without motors

These commands never open a serial port:

```sh
python raspberry_pi/control_arm.py --demo
python -m unittest discover -s raspberry_pi/tests -v
python raspberry_pi/control_arm.py --list-keyboards
python raspberry_pi/control_arm.py
```

The program automatically selects a connected full keyboard and prints its device path.
Just run the command; there is no keyboard selection prompt. If several keyboards
are connected, it chooses the first device path. To override it, use
`--keyboard /dev/input/eventN`; `--list-keyboards` lists candidates. Linux `evdev`
reads a **keyboard physically connected to the Pi**, including real key-down and
key-up events. It does not read keystrokes sent through SSH or the GitHub website.
Choose a dedicated keyboard; the process grabs it exclusively so typing into
another application cannot accidentally send robot commands. The account needs
access to the chosen `/dev/input/eventN` device and `/dev/ttyUSB0` through the
Pi's normal device permissions. Q releases the keyboard and exits.

## 4. Control the arm

```sh
python raspberry_pi/control_arm.py --live
# Optional per-run override (otherwise uses config.py):
python raspberry_pi/control_arm.py --live --port /dev/ttyUSB1 --baudrate 115200
```

Press **E** to enable and level the wrist, matching the website's Enable + level
action. Levelling can move the tool centre. Subsequent translations hold the
chosen pitch.

| Key | Action |
| --- | --- |
| W / S | Tool centre up / down in world Z |
| Left / Right | Base yaw left / right |
| Up / Down | Forward / backward in the arm's yaw plane |
| A / D | Pitch up / down while holding tool centre |
| J / L | Start opening / closing continuously |
| Space | Stop movement and hold measured position; next direction resumes |
| E | Enable + level / pause |
| Escape | Pause and hold measured position |
| Q | Stop commands, hold measured position, exit |

Direction keys are latched by default: tap once to continue moving after release.
Tap another direction to replace the current motion. This also applies to J/L,
which open/close at up to 30 displayed degrees/second. **Space** clears the
motion and requests a measured hold without disabling keys; tap a direction to
resume. **Escape** pauses and requires E to enable again. E still levels the wrist
before jogging; Space cancels levelling too.

Set `LATCHED_KEYS=False` in `config.py` to restore hold-to-move controls.
`SPEED_SCALE=1.0` requests the simulation's nominal Cartesian speed (31.25 mm/s
translation and 18.75°/s yaw/pitch). Joint limits can reduce that speed.
`MAX_JOINT_DEG_PER_SECOND` is 30 per joint, and `CONTROL_PERIOD_SECONDS=0.04`
requests smaller, more frequent setpoint updates. The terminal prints `cycle ms`
so serial/IK delays can be checked on the Pi. Position feedback and tracking
checks run on each active tick; mode/torque checks run on enable and at least
every 0.5 seconds per motor while moving, staggered across cycles. Serial faults and watchdog delays still stop motion.

Near singularities the controller reduces a requested Cartesian step to stay
within joint-rate limits; unreachable steps are rolled back. Motor speed and
acceleration are configured separately in `config.py`.

Five targets are sent together using an SDK sync-write packet. A successful
sync-write means the packet was sent; broadcast writes have no motor acknowledgement.
The script reads each servo position individually, matching your working arm
project. Optional sync-read uses the 56–66 register block. It checks torque and
position mode every 0.5 seconds while active, and
stops on communication errors, excessive tracking
error, lost keyboard events or a delayed control loop. Startup feedback is used
to initialise FK; during motion the solver tracks its commanded pose and checks
the measured pose against it. It is not a calibrated dynamics/torque controller.

**Space/Escape are software stops, not a hardware emergency stop.** The controller
tries to hold the measured pose and leaves torque enabled to avoid dropping a
gravity-loaded arm. If USB fails or the process/OS hangs, a hold command may not
reach the controller; motors can finish their last small setpoint. Use an
independent physical motor-power stop when commissioning. The model has no
collision avoidance, and matching equations cannot compensate for incorrect
calibration, backlash, flex or misassembled joints.

## Files and validation

- `config.py`: raw reference values, calibration, IDs, serial port and speeds.
- `kinematics.py`: FK, calibrated angles, position IK and pitch-constrained jogging.
- `controller.py`: key state, levelling, continuous/hold-to-move control and rate-limited jogging.
- `servo_bus.py`: calibration conversion and Waveshare SDK commands/feedback.
- `control_arm.py`: Pi keyboard loop, dry-run/live modes and stop handling.
- `read_positions.py`: read-only raw position calibration aid.
- `tests/`: MuJoCo parity fixtures and mocked hardware/controller tests.

`Robot.solve()` exposes the website's position-only IK for Python experiments.
It leaves the best pose found even when unreachable; it does **not** send that
pose to motors. The live keyboard path uses the stricter pitch-constrained jog
solver and bounded steps.

To regenerate comparison fixtures after changing the website model/equations:

```sh
npm ci
node tools/generate-pi-reference.mjs
python -m unittest discover -s raspberry_pi/tests -v
```

Fixtures compare 16 poses including every coupled body, levelling, translation,
yaw, pitch, unreachable rollback and position IK against the actual MuJoCo WASM
browser engine. Hash checks detect stale source/model references. Passing these
tests verifies software agreement, not the physical robot calibration.

To include the packet-level tests against your exact, unmodified SDK:

```sh
export SCSERVO_SDK_PATH="/absolute/path/to/scservo_sdk"
python -m unittest discover -s raspberry_pi/tests -v
```

`SCSERVO_SDK_PATH` points to the package itself, whereas `PYTHONPATH` above
points to its parent. These additional tests replace only the serial device;
the actual SDK builds and parses the packets. They verify five-motor write
payloads, missing replies, checksums, stale feedback, servo errors, signed raw
positions, mode/torque checks and failed transmissions without opening hardware.
They are skipped when `SCSERVO_SDK_PATH` is unset.
