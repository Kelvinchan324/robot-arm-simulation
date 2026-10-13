import json
import unittest
from unittest.mock import patch
from test_controller import test_config
import export_browser_profile


class BrowserProfileTests(unittest.TestCase):
    def test_unmeasured_calibration_is_not_exported(self):
        settings = export_browser_profile.config.settings()
        settings['servos']['base_yaw']['raw_positions'] = [None, None]
        with patch.object(export_browser_profile.config, 'settings', return_value=settings):
            with self.assertRaisesRegex(ValueError, 'calibration'):
                export_browser_profile.profile()

    def test_export_preserves_measured_motor_mapping(self):
        settings = test_config()
        with patch.object(export_browser_profile.config, 'settings', return_value=settings):
            profile = json.loads(json.dumps(export_browser_profile.profile(), allow_nan=False))
        self.assertEqual(profile['version'], 1)
        self.assertEqual(profile['servos'], settings['servos'])
        self.assertEqual(profile['servo_family'], 'sms_sts')
        self.assertTrue(profile['position_mode_confirmed'])

    def test_position_mode_must_be_confirmed(self):
        settings = test_config()
        settings['position_mode_confirmed'] = False
        with patch.object(export_browser_profile.config, 'settings', return_value=settings):
            with self.assertRaisesRegex(ValueError, 'position mode'):
                export_browser_profile.profile()

    def test_configured_arm_mapping_and_cad_limits(self):
        from kinematics import Robot
        from servo_bus import calibrated_joints
        robot = Robot()
        self.assertFalse(export_browser_profile.config.settings()['sync_read'])
        maps = calibrated_joints(export_browser_profile.config.settings(), robot)
        for i, mapping in enumerate(maps):
            self.assertEqual(mapping.to_raw(0), 2048)
            self.assertEqual(mapping.to_raw(45), 1536 if i == 4 else 2560)
            self.assertAlmostEqual(mapping.from_raw(2048), 0)
        profile = export_browser_profile.profile()
        self.assertEqual(profile['servos']['elbow']['safe_degrees'], [-90, 104])
