"""Edit the calibration below before enabling real motors.

Raw reference positions are calibration anchors, NOT startup destinations.
The controller reads the actual servo positions on every start/re-enable.
"""

# ------------------ USER-EDITABLE MOTOR CALIBRATION ------------------
# Raw encoder reading when each joint is at REFERENCE_DEGREES below.
# None deliberately prevents live motion until you enter measured values.
RAW_AT_REFERENCE = {
    'base_yaw': None,       # Servo ID 1
    'shoulder': None,       # Servo ID 2
    'elbow': None,          # Servo ID 3
    'wrist_pitch': None,    # Servo ID 4
    'gripper_drive': None,  # Servo ID 5
}
REFERENCE_DEGREES = dict(base_yaw=0, shoulder=0, elbow=0, wrist_pitch=0, gripper_drive=0)

# Signed encoder ticks per one displayed simulation degree. Measure at two
# known joint angles: (raw_B - raw_A) / (joint_deg_B - joint_deg_A).
# Includes motor direction and gearing. Do not assume all motors are direct
# drive. The gripper's displayed -90..90 is an opening scale, not shaft angle.
TICKS_PER_JOINT_DEGREE = dict(base_yaw=None, shoulder=None, elbow=None, wrist_pitch=None, gripper_drive=None)

# Tested mechanical ranges in simulation/display degrees. Keep clear of hard
# stops. All values must also fit the CAD limits; do not simply copy raw range.
SAFE_JOINT_DEGREES = dict(base_yaw=None, shoulder=None, elbow=None, wrist_pitch=None, gripper_drive=None)
RAW_LIMITS = {name: [0, 4095] for name in RAW_AT_REFERENCE}
CALIBRATION_CONFIRMED = False
POSITION_MODE_CONFIRMED = False
# ------------------ CONNECTION AND SPEED ------------------
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200  # Pi <-> ESP32 transparent forwarding, not servo bus baud.
SDK_MODULE = 'scservo_sdk'  # Change only if your compatible SDK has another name.
SERVO_IDS = dict(base_yaw=1, shoulder=2, elbow=3, wrist_pitch=4, gripper_drive=5)
MAX_JOINT_DEG_PER_SECOND = dict(base_yaw=10, shoulder=10, elbow=10, wrist_pitch=10, gripper_drive=30)
SERVO_SPEED = 100       # Nonzero STS speed in encoder steps/second.
SERVO_ACCELERATION = 20 # STS SDK acceleration parameter.
SPEED_SCALE = 0.25     # 1.0 requests the browser's 2.5 mm / 1.5 degree per 80 ms.
MAX_TRACKING_ERROR_DEG = 8.0
LOOP_TIMEOUT_SECONDS = 0.5


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
    return dict(port=PORT, baudrate=BAUDRATE, sdk_module=SDK_MODULE, servo_family='sms_sts',
                calibration_confirmed=CALIBRATION_CONFIRMED,
                position_mode_confirmed=POSITION_MODE_CONFIRMED, servos=servos)
