#!/usr/bin/env python3
"""Export measured Pi calibration for the website's local Web Serial connection.

This script does not import the SDK or access any serial device.
"""
import argparse
import json
from pathlib import Path
import config
from kinematics import Robot
from servo_bus import calibrated_joints


def profile():
    settings = config.settings()
    calibrated_joints(settings, Robot())
    if settings['position_mode_confirmed'] is not True:
        raise ValueError('Confirm position mode in config.py before exporting')
    return dict(version=1, servo_family='sms_sts', calibration_confirmed=True,
                position_mode_confirmed=True, servos=settings['servos'],
                speed_scale=config.SPEED_SCALE, max_tracking_error_deg=config.MAX_TRACKING_ERROR_DEG)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('arm-calibration.json'))
    args = parser.parse_args()
    data = profile()
    args.output.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(f'Saved {args.output.resolve()}. Load it under USB arm > Calibration on the website.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError) as error:
        raise SystemExit(str(error))
