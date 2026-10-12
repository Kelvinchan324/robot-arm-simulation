ENGG1101 Robot Arm Lab

STATUS
The webpage loads the actual ENGG1101_Robot_Arm_relation.f3d geometry and joints
in MuJoCo 3.15. Nine rigid groups, eight joints and three motion links produce
five independent controls. The source is a Fusion archive, not STEP.
dist/robot contains the URDF, MJCF, metre-scale OBJ meshes and downloadable ZIP.
Model-file joint angles are relative to the extracted CAD pose; the website
applies teaching-angle calibration through dist/joint-angles.js. Inertia and effort values
are placeholders for this kinematics-only workbench; no dynamics are claimed.

FEATURES
- Calibrated controls: base yaw 0 at the former 18.8 degrees; shoulder backward
  limit +90; elbow lower limit -90; wrist backward limit +90. Gripper +90 opens
  fully and -90 closes fully. Calibration changes controls/readouts, preserving
  CAD geometry and model-file joint coordinates. Tool pitch remains world pitch.
- MuJoCo mj_forward for all link poses, joint sliders and tool transforms.
- Position-only damped least-squares IK with limits and five bounded restarts.
- Forward and inverse controls shown together with the tool-to-world transform.
- Pick target in 3D: click or drag on an XY plane, choose its Z with the height
  slider, and release to solve IK. A dashed line shows remaining tip error.
  Out-of-reach targets use the best pose found by the bounded numerical solver;
  this is not a guarantee of a globally closest pose.
- Fixed jaw-tip tool frame, 3D orbit, frames and trace.
- Keyboard: enable first levels the wrist (moving the tip). W/S change world Z,
  up/down change reach in the arm's yaw plane, left/right change base yaw,
  A/D adjust pitch at the same tip centre. Translation holds the chosen pitch.
  Escape disables keys. Fields, focus loss and hidden pages stop held keys.
  The constrained jog solver rolls back unreachable moves and respects limits.
- Minimal simulation interface; model package remains at robot/engg1101-robot-package.zip.
- No server processing of uploaded models, student accounts or tracking.

CAD EXTRACTION WORKFLOW (COMPLETED)
1. Sign in to Fusion and open the supplied archive.
2. Inspect whether actual joints, rigid groups and motion limits are defined.
3. For read-only evidence extraction, use Utilities > Scripts and Add-ins,
   add the tools/FusionExtract folder, and run FusionExtract with this model active.
   It writes cad-export/assembly.json and per-body OBJ meshes in metres.
   The extractor has been executed inside Fusion for this model.
4. FusionRelations.py extracts rigid groups, as-built joints and motion links.
   convert-fusion.py groups meshes and builds the URDF/MJCF package. The five
   joint motion audits match MuJoCo body transforms to numerical precision.
5. Do not infer physical limits or inertia from appearance. Closed-loop gripper
   linkages need MuJoCo equality constraints or a documented simplification.

Alternatively export with a maintained Fusion URDF exporter:
https://github.com/Adriaeik/fusion2URDF
Review its requirements for naming, assembly structure and supported joints.

MODEL IMPORT SCOPE
Fixed-base hinge/slider models only. Mesh filenames must be unique and selected
with the model. MJCF includes must be flattened. URDF mimic joints are rejected.
Browser viewer supports plane, sphere, box, cylinder, capsule, ellipsoid and mesh.
Mesh materials/textures are not reproduced; solid geom colors are used.
The Robot file download references the accompanying mesh files; retain them.
IK is position only. It does not constrain orientation or avoid collisions.
No dynamic stepping, validated motor model or physical safety validation is claimed.

LOCAL DEVELOPMENT
npm install
node tools/test-engine.mjs
node tools/test-jog.mjs
node tools/test-point-target.mjs
node tools/test-joint-angles.mjs
node tools/test-cad.mjs (requires local cad-export/audit-expected.json)
Serve dist/ using a local static HTTP server. Open its URL in a modern browser.
The site bundles MuJoCo and Three.js; Google Fonts are optional styling requests.
GitHub Pages publication is in .github/workflows/pages.yml. The teaching site is public.

Sources: https://github.com/google-deepmind/mujoco/tree/main/wasm
https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Joint.htm
