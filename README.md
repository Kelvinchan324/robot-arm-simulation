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
- With keyboard control enabled, press **J** to open the gripper (+90 degrees)
  or **L** to close it (-90 degrees). Shortcuts are inactive while editing fields.

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

[raspberry_pi/README.md](raspberry_pi/README.md) contains the Python controller
for Waveshare ESP32 + STS3215 servos (IDs 1–5, `/dev/ttyUSB0`, 115200 baud).
It uses the same robot geometry, calibrated angles and keyboard kinematics,
including J/L gripper control. Edit the raw reference/calibration values at the
top of `raspberry_pi/config.py` before live operation; dry-run is the default.
