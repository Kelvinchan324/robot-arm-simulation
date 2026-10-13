# ENGG1101 Robot Arm Simulation

[Open the simulation](https://Kelvinchan324.github.io/robot-arm-simulation/)

A browser-based MuJoCo robot arm lab using the Fusion 360 assembly geometry.
Students can explore forward and inverse kinematics, joint angles, keyboard
movement, 3D targets, and the tool-to-world transformation together.

## Use

- Adjust joint sliders for forward kinematics.
- Enter an XYZ target or select **Pick target in 3D** for inverse kinematics.
- Enable keyboard control: W/S moves vertically, left/right changes yaw,
  up/down moves forward/back, and A/D adjusts tool pitch. Translation holds pitch.
- With keyboard control enabled, **hold J** to open the gripper toward +90°
  or **hold L** to close it toward -90°. Release to stop changing the angle.
  The gripper moves at 30 displayed degrees/second (six seconds for full travel);
  the pinion and jaws follow every intermediate angle. Shortcuts are inactive
  while editing fields.

The solver respects joint limits and attempts the closest position it finds.
This is a kinematics teaching model, without collision avoidance or validated dynamics.

## Run locally

Serve `dist` with a static HTTP server, for example:

```sh
python -m http.server 8000 --directory dist
```

Open http://localhost:8000. All simulation libraries, meshes and robot files are bundled.

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

## Raspberry Pi hardware control

The website now supports **direct USB serial keyboard control** in desktop
Chrome/Edge. The 3D arm follows measured STS3215 positions; FK/IK edits are
disabled while connected. See [Web Serial setup](raspberry_pi/WEB_SERIAL.md)
to export your measured calibration, connect the ESP32 board and use the same
W/S, arrow, A/D and J/L keys. The initial connection is read-only.

[raspberry_pi/README.md](raspberry_pi/README.md) contains the Python controller
for Waveshare ESP32 + STS3215 servos (IDs 1–5, `/dev/ttyUSB0`, 115200 baud).
It uses the same robot geometry, calibrated angles and keyboard kinematics,
including J/L gripper control. Edit the raw reference/calibration values at the
top of `raspberry_pi/config.py` before live operation; dry-run is the default.
