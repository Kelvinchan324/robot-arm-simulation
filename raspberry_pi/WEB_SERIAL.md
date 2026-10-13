# Keyboard control from the website

Open [the robot arm website](https://kelvinchan324.github.io/robot-arm-simulation/)
in desktop Chrome or Edge. Connect the Waveshare ESP32 board by USB to **the
computer running that browser**. This can be your Raspberry Pi running a
compatible desktop browser, or a Windows/Mac/Linux laptop.

The browser sends STS3215 binary serial packets directly to the board. There is
no Python server, cloud motor relay or SDK installation needed for this route.
The Python controller remains available separately; close it and any serial
monitor before connecting, because only one program can own the port.

## Prepare calibration once

1. `raspberry_pi/config.py` is prefilled from your existing STS3215 project.
   Review its raw reference positions,
   signed ticks per displayed joint degree, and safe travel limits. Follow the
   measurement instructions in [README.md](README.md). IDs are 1–5 from base to
   gripper. Do not use values from the synthetic test fixtures on hardware.
2. The configuration already assumes calibration and position mode are confirmed.
   The existing mapping values are already entered; no extra confirmation is needed.
3. From the repository root, export the browser profile:

   ```sh
   python raspberry_pi/export_browser_profile.py --output arm-calibration.json
   ```

   This command needs the Python requirements but does not access any motors.
   It uses the same calibration and limits as the Pi controller. The resulting
   JSON is ignored by Git; keep a copy for your particular physical arm.
4. On the website, expand **USB arm → Calibration** and select that JSON file.
   The file stays in the browser's memory and is not uploaded. Load it again
   after reloading the page. Disconnect before changing calibration.

## Connect and use the keyboard

1. Put the ESP32 board in **Start Serial Forwarding** mode using its normal
   controls. Prepare the servos in position mode with torque already enabled.
   The website reads those states and never changes torque, IDs, zero offsets
   or EEPROM settings. Support the arm while preparing it with vendor tools.
2. Leave **115200 baud** selected for your board's USB forwarding connection.
   Click **Connect USB** and choose the board in the browser's device picker.
   The browser chooses the serial device; you do not type `/dev/ttyUSB0` or COM
   names into this page. The optional 1000000 setting is for a link actually
   configured for that speed, not a request to change motor EEPROM baud.
3. Connection only reads positions. With no calibration file, raw encoder
   values are still displayed, but real movement is unavailable. With a valid
   profile, **LIVE · MEASURED** means the 3D arm, joint angles, XYZ and matrix
   are being computed from the real encoders.
4. Click **Enable keys + level**, or focus the 3D view and press **E**. The wrist levels gradually from its
   measured pose. Then use the same movement keys as the simulation:

   | Keys | Action |
   | --- | --- |
   | W / S | Tool centre up / down in Z |
   | Left / Right | Base yaw |
   | Up / Down | Forward / backward in the yaw plane |
   | A / D | Tool pitch while holding tool centre |
   | Hold J / L | Gradually open / close; release to stop changing the target |
   | E | Enable + level / pause |
   | Escape | Pause and request a hold at the measured pose |

   Hold J/L to change the gripper angle at up to 30 displayed degrees/second,
   respecting any lower profile rate and calibrated travel limits. Release to
   stop further increments; the servo may finish its last small commanded step.
   Holding both keys cancels gripper motion. Escape pauses all motion. Live jogging
   defaults to 25% of the simulation's requested step rate and applies the
   per-joint rate limits from your profile. The 3D view displays the last
   complete measured pose; it does not instantly jump to a requested endpoint.
5. **Disconnect** pauses and closes the port. Simulation-only controls become
   available again. While connected, the FK sliders, reference/example poses
   and point-target IK are read-only; only keyboard jogging drives hardware.

## Troubleshooting connection

- If **Connect USB** is disabled, use desktop Chrome or Edge and open the page
  from `http://localhost:8000` (or the published HTTPS site). Web Serial is not
  available on every browser or on an insecure remote HTTP page.
- If the browser's device picker opens but no device appears, reconnect the
  board with a USB data cable and check that the operating system detects its
  USB serial device.
- If the picker opens but the connection fails, choose the ESP32 USB serial
  device, put the board in **Start Serial Forwarding** mode, leave **115200
  baud** selected, and close vendor tools, serial monitors, or Python
  controllers that may already be using the port.
- If the status says the device was disconnected or lost, the USB serial link
  ended after selection. Connect the board to the same computer running the
  browser, check that it remains powered and in serial-forwarding mode, then
  unplug/reconnect it and click **Connect USB** again. You do not need to
  click Disconnect first after a fault. Recoverable UART stream errors while
  idle reopen the browser reader; errors during feedback still stop control.
- The status below **Connect USB** displays the browser's connection error.
  A successful connection with no calibration is read-only; load a valid
  measured calibration profile separately to enable movement.

Leaving the window, hiding the tab, or moving focus away from the 3D view pauses keyboard motion.
A fresh **E** is needed to resume. A failed serial transaction, servo fault,
out-of-range feedback, excessive tracking error or delayed control step stops
further jogging. Reconnect explicitly after a serial fault; the page never
silently retries an old target or reuses cached feedback to authorize motion.

**A browser pause is not a physical emergency stop.** It requests a fresh
measured hold when communication is working and leaves torque enabled. A USB
failure, frozen browser or OS stall can prevent that request; a motor may
finish its last bounded setpoint. Use the arm's independent physical stop.
The model has no collision avoidance. Encoder agreement does not account for
backlash, flex, incorrect assembly or incorrect calibration.

## Implementation and tests

- `dist/serial-protocol.js`: FeelTech/SCS checksums, parsing and five-motor
  sync-write packets matching the supplied `sms_sts.SyncWritePosEx` layout.
- `dist/serial-link.js`: Web Serial streams, one transaction at a time,
  sync-read positions, motor errors, timeouts and explicit disconnect.
- `dist/hardware-profile.js`: measured calibration and travel validation.
- `dist/live-jog.js`: bounded keyboard motion using the existing `CartesianJog`
  equations, following `raspberry_pi/controller.py`.
- `dist/hardware-session.js`: read-only startup, feedback monitoring, command
  generation, pause handling and a separate command model.
- `dist/hardware-ui.js`: connection, local profile loading and keyboard events.

The existing MuJoCo engine renders feedback using the exported Fusion joint
transforms and couplings. Live hardware movement does not run a dynamics
simulation or use a camera to estimate position.

```sh
npm ci
node --test tools/test-serial.mjs
python -m unittest discover -s raspberry_pi/tests -v
```

The local-only browser fixture can exercise the full UI without a serial
device: run `python tools/preview-serial.py` and open its printed test URL.
It is clearly labelled **SIMULATED USB TEST** and is outside the published
`dist` folder. Its synthetic calibration is exclusively for tests.

Reference: [HKU's robot controller](https://activelearning.engg.hku.hk/engg1101/robotic-arm-controller)
for the browser connection workflow; [Chrome's Web Serial documentation](https://developer.chrome.com/docs/capabilities/serial)
for permission-based local device access and stream lifecycle. This site's
movement code uses the supplied servo SDK's binary protocol, not code copied
from the reference page.
