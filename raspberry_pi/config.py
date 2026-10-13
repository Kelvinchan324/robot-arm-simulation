"""Arm settings copied from the operator's STS3215-5DOF-Robot-Arm project.

Raw reference positions are calibration anchors, NOT startup destinations.
The controller reads the actual servo positions on every start/re-enable.
"""

# ------------------ USER-EDITABLE MOTOR CALIBRATION ------------------
# Raw encoder reading when each joint is at REFERENCE_DEGREES below.
# Source: STS3215-5DOF-Robot-Arm/arm_settings.py (ZERO_TICKS).
# Its joint angles are assumed to match the simulation display coordinates.
RAW_AT_REFERENCE = {
    'base_yaw': 2048,       # Servo ID 1
    'shoulder': 2048,       # Servo ID 2
    'elbow': 2048,          # Servo ID 3
    'wrist_pitch': 2048,    # Servo ID 4
    'gripper_drive': 2048,  # Servo ID 5
}
REFERENCE_DEGREES = dict(base_yaw=0, shoulder=0, elbow=0, wrist_pitch=0, gripper_drive=0)

# Signed encoder ticks per one displayed simulation degree. Measure at two
# known joint angles: (raw_B - raw_A) / (joint_deg_B - joint_deg_A).
# Includes motor direction and gearing. Do not assume all motors are direct
# drive. The gripper's displayed -90..90 is an opening scale, not shaft angle.
TICKS_PER_JOINT_DEGREE = dict(base_yaw=4096/360, shoulder=4096/360,
                              elbow=4096/360, wrist_pitch=4096/360,
                              gripper_drive=-4096/360)

# Tested mechanical ranges in simulation/display degrees. Keep clear of hard
# stops. All values must also fit the CAD limits; do not simply copy raw range.
# Intersection of source ANGLE_LIMITS and this simulation's CAD limits.
SAFE_JOINT_DEGREES = dict(base_yaw=[-90, 90], shoulder=[-90, 90],
                         elbow=[-90, 104], wrist_pitch=[-116.99999997815527, 90],
                         gripper_drive=[-90, 90])
RAW_LIMITS = {name: [0, 4095] for name in RAW_AT_REFERENCE}
# The arm is already calibrated and prepared, as specified by the operator.
# Encoder mappings above are still needed to convert model angles to hardware.
CALIBRATION_CONFIRMED = True
POSITION_MODE_CONFIRMED = True
# ------------------ CONNECTION AND SPEED ------------------
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200  # Pi <-> ESP32 transparent forwarding, not servo bus baud.
SDK_MODULE = 'scservo_sdk'  # Change only if your compatible SDK has another name.
SDK_PARENT_DIR = '/home/kelvin/roboarm_contest/STS3215-5DOF-Robot-Arm'
# Used automatically if the SDK is not installed in the active Python environment.
SYNC_READ = False      # Match the working arm_helpers.read_all / ReadPos path.
# Individual reads work with this controller's serial forwarding. Set True
# only for verified sync-read support. Communication errors never auto-fallback.
SERVO_IDS = dict(base_yaw=1, shoulder=2, elbow=3, wrist_pitch=4, gripper_drive=5)
MAX_JOINT_DEG_PER_SECOND = dict(base_yaw=30, shoulder=30, elbow=30, wrist_pitch=30, gripper_drive=30)
# Hold J/L to jog. Gripper opening is capped at 30 display degrees/second;
# lower gripper_drive above to move more slowly (also exported to the browser).
SERVO_SPEED = 3000      # Match arm_settings.MOVE_SPEED; setpoint rates remain limited.
SERVO_ACCELERATION = 254 # STS SDK acceleration parameter.
SPEED_SCALE = 1.0     # 1.0 requests the browser's 2.5 mm / 1.5 degree per 80 ms.
LATCHED_KEYS = True    # Tap a direction to keep moving; Space stops.
CONTROL_PERIOD_SECONDS = 0.04
HEALTH_CHECK_PERIOD_SECONDS = 0.5
MAX_TRACKING_ERROR_DEG = 8.0
LOOP_TIMEOUT_SECONDS = 0.5
SERIAL_TRANSACTION_GAP_SECONDS = 0.003  # Give ESP32 forwarding time between packets.


def settings():
    servos = {}
    for name, raw in RAW_AT_REFERENCE.items():
        span, scale = SAFE_JOINT_DEGREES[name], TICKS_PER_JOINT_DEGREE[name]
        positions = [None, None]
        if span is not None and raw is not None and scale is not None:
            positions = [raw+(angle-REFERENCE_DEGREES[name])*scale for angle in span]
        servos[name] = dict(id=SERVO_IDS[name], joint_degrees=span, raw_positions=positions,
                            raw_limits=RAW_LIMITS[name], safe_degrees=span,
                            max_deg_s=MAX_JOINT_DEG_PER_SECOND[name], speed=SERVO_SPEED,
                            acceleration=SERVO_ACCELERATION)
    return dict(port=PORT, baudrate=BAUDRATE, sdk_module=SDK_MODULE, sdk_parent_dir=SDK_PARENT_DIR, servo_family='sms_sts', sync_read=SYNC_READ,
                serial_transaction_gap_seconds=SERIAL_TRANSACTION_GAP_SECONDS,
                calibration_confirmed=CALIBRATION_CONFIRMED,
                position_mode_confirmed=POSITION_MODE_CONFIRMED, servos=servos)
