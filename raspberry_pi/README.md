# Raspberry Pi keyboard controller

Python controller for this robot, a Waveshare **Servo Driver with ESP32**, and
**STS3215** motors through the vendor `scservo_sdk` library.

Configured connection: **`/dev/ttyUSB0`, 115200 baud**. Motor IDs:

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

**Hardware calibration is still required.** The code is tested against browser
MuJoCo results and a mocked serial bus, not your physical arm. It deliberately
starts in dry-run mode and refuses live operation with blank calibration.

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

The adapter uses `ReadPos`, `SyncWritePosEx`, `groupSyncWrite.txPacket` and
`read1ByteTxRx`. If your package has a different import name but provides this
same API, edit `SDK_MODULE` in `config.py`. Older `PacketHandler` APIs are not
interchangeable with `sms_sts` without an adapter.

If needed, obtain the official SDK from [Waveshare's Python download](https://files.waveshare.com/wiki/Bus_Servo_Adapter_A/STServo_Python.zip).
Use the **`scservo_sdk` directory**, not its bundled Windows virtual environment.
Keep that SDK outside this repository and expose its parent directory:

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

## 2. Set the raw reference values at the top of config.py

The first editable block is `RAW_AT_REFERENCE`, one entry for each of IDs 1–5.
Those values mean “the raw encoder position at this known joint angle,” **not**
“move to this position when the program starts.” Startup and re-enabling read
the arm's measured position instead.

Use the read-only helper to display current raw positions:

```sh
python raspberry_pi/read_positions.py
python raspberry_pi/read_positions.py --watch
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
Only after measuring/checking all five joints, set `CALIBRATION_CONFIRMED` and
`POSITION_MODE_CONFIRMED` to `True`. It checks STS position mode and existing
torque-enable state before motion. Support the arm while preparing it with the
vendor tools; this script never enables torque against an old servo target.

## 3. Test without motors

These commands never open a serial port:

```sh
python raspberry_pi/control_arm.py --demo
python -m unittest discover -s raspberry_pi/tests -v
python raspberry_pi/control_arm.py --list-keyboards
python raspberry_pi/control_arm.py --keyboard /dev/input/eventN
```

Replace `eventN` with the keyboard device reported by the listing. Linux `evdev`
reads a **keyboard physically connected to the Pi**, including real key-down and
key-up events. It does not read keystrokes sent through SSH or the GitHub website.
Choose a dedicated keyboard; the process grabs it exclusively so typing into
another application cannot accidentally send robot commands. The account needs
access to the chosen `/dev/input/eventN` device and `/dev/ttyUSB0` through the
Pi's normal device permissions. Q releases the keyboard and exits.

## 4. Control the arm

```sh
python raspberry_pi/control_arm.py --live --keyboard /dev/input/eventN
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
| J / L | Open / close gripper to the calibrated safe endpoint |
| E | Enable + level / pause |
| Escape | Pause and hold measured position |
| Q | Stop commands, hold measured position, exit |

The gripper ramps toward its endpoint after a key press; Escape cancels it.
Levelling also ramps before Cartesian jogging begins. Default requested jogging
speed is 25% of the browser, with joint-rate limits. Set `SPEED_SCALE=1.0` only
after checking the mechanism if you want the browser's nominal step rate.
Near singularities the controller reduces a requested Cartesian step to stay
within joint-rate limits; unreachable steps are rolled back. Motor speed and
acceleration are configured separately in `config.py`.

Five targets are sent together using an SDK sync-write packet. The script
monitors position feedback and stops on communication errors, excessive tracking
error, lost keyboard events or a delayed control loop. Startup feedback is used
to initialise FK; during motion the solver tracks its commanded pose and checks
the measured pose against it. It is not a calibrated dynamics/torque controller.

**Escape is a software pause, not a hardware emergency stop.** The controller
tries to hold the measured pose and leaves torque enabled to avoid dropping a
gravity-loaded arm. If USB fails or the process/OS hangs, a hold command may not
reach the controller; motors can finish their last small setpoint. Use an
independent physical motor-power stop when commissioning. The model has no
collision avoidance, and matching equations cannot compensate for incorrect
calibration, backlash, flex or misassembled joints.

## Files and validation

- `config.py`: raw reference values, calibration, IDs, serial port and speeds.
- `kinematics.py`: FK, calibrated angles, position IK and pitch-constrained jogging.
- `controller.py`: key state, levelling, gripper ramps and rate-limited jogging.
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
