ENGG1101 CAD conversion

Joint values are relative to the exported Fusion pose. Base yaw has no CAD limits; UI uses +/-180 degrees.
Kinematics only. No collision avoidance or calibrated dynamics. Box inertia and effort/velocity entries are placeholders.
Tool: Midpoint between jaw-tip extents, computed from CAD meshes; +Z points out of the gripper.

URDF contains standard mimic joints. For MuJoCo use engg1101.xml, which expresses the couplings as equality constraints. Kinematic viewers must also apply coupled qpos values before mj_forward. All OBJ meshes are in metres and must remain alongside the model files.
