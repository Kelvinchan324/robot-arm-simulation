# ENGG1101 Robot Arm Simulation

[Open the simulation](https://Kelvinchan324.github.io/robot-arm-simulation/)

A browser-based MuJoCo robot arm lab using the Fusion 360 assembly geometry.
Students can explore forward and inverse kinematics, joint angles, keyboard
movement, 3D targets, and the tool-to-world transformation together.

## Start the website

From the repository folder:

```sh
python3 start.py
```

Open http://localhost:8000. Press Ctrl+C in the terminal to stop the server.
The simulation libraries and robot assets are bundled.

## Move the simulated arm

Use the joint sliders for forward kinematics, or enter XYZ / **Pick target in
3D** for inverse kinematics. Enable keyboard control and focus the 3D view:

| Keys | Movement |
| --- | --- |
| W / S | Up / down |
| Left / Right | Base yaw |
| Up / Down | Forward / backward |
| A / D | Tool pitch |
| Hold J / L | Open / close gripper; release to stop |
| E / Escape | Enable or pause / pause |

Translations hold tool pitch. Gripper motion is 30 displayed degrees per
second. Shortcuts pause while editing fields. The solver respects joint limits;
this teaching model has no collision avoidance or validated dynamics.

## Control the real arm

Choose either connection method. Both use the simulation's model, calibrated
joint angles and pitch-constrained kinematics.

| Method | How to start | Setup |
| --- | --- | --- |
| Website USB | Desktop Chrome/Edge → **Connect USB** → **Enable keys + level** | [USB setup and troubleshooting](raspberry_pi/WEB_SERIAL.md) |
| Raspberry Pi | `python raspberry_pi/control_arm.py --live` | [Pi installation and calibration](raspberry_pi/README.md) |

On the Pi, `python raspberry_pi/control_arm.py` starts keyboard control in
**dry-run** mode without motors. It automatically detects a physically connected
keyboard. Press **E** to enable, tap a direction for continuous movement, press **Space**
to stop, and **Q** to exit; no keyboard picker is needed. This continuous mode
applies to the Pi controller; the website uses hold-to-move controls.
SSH typing does not provide the key-release events required by this controller.

The Pi configuration includes your existing STS3215 project settings (2048
zero positions, signed 4096/360 scales and restricted travel). Website startup
is read-only and shows real encoder feedback; load your exported calibration
to enable motion. Use one connection method at a time.

If the website reports a lost device, close other serial applications and check
USB power, the cable, and ESP32 **Start Serial Forwarding** mode at **115200 baud**.
After a fault, **Connect USB** lets you reconnect directly. A physical USB failure
still stops commands; the software cannot restore a disconnected device.

## Publish

GitHub Pages uses the **GitHub Actions** source in repository Settings > Pages.
The included workflow publishes `dist` on pushes to `main` or manual workflow runs.
No build step or application secrets are needed.

## Validate and develop

```sh
npm ci
node tools/test-engine.mjs
node tools/test-jog.mjs
node tools/test-point-target.mjs
node tools/test-joint-angles.mjs
```

See [README.txt](README.txt) for CAD conversion details and model limitations.
URDF, MuJoCo XML and OBJ meshes are in `dist/robot`.
Third-party MuJoCo and Three.js licenses are retained in `dist/vendor`.
