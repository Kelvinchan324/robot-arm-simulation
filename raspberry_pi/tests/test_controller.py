import copy
import hashlib
import json
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
import numpy as np

PI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PI))
from kinematics import Robot, CartesianJog, NAMES
from controller import Controller
from servo_bus import ServoBus, calibrated_joints
import config


class BrowserParity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = json.loads((PI/'tests/browser_reference.json').read_text())

    def assert_pose(self, robot, expected):
        np.testing.assert_allclose(robot.q, expected['q'], atol=2e-7, rtol=0)
        p, r = robot.pose()
        np.testing.assert_allclose(p, expected['p'], atol=2e-8, rtol=0)
        np.testing.assert_allclose(r.reshape(9), expected['r'], atol=2e-7, rtol=0)
        np.testing.assert_allclose(robot.display(), expected['display'], atol=2e-5, rtol=0)

    def test_reference_matches_current_sources(self):
        for name, digest in self.reference['hashes'].items():
            text = (PI.parent/name).read_text(encoding='utf-8').replace('\r\n', '\n')
            self.assertEqual(hashlib.sha256(text.encode()).hexdigest(), digest, name)

    def test_fk_and_all_coupled_links(self):
        robot = Robot()
        for case in self.reference['fk']:
            robot.q = case['q']; self.assert_pose(robot, case)
            for name, body in case['bodies'].items():
                p, r = robot.bodies[name]
                np.testing.assert_allclose(p, body['p'], atol=2e-9, rtol=0)
                np.testing.assert_allclose(r.reshape(9), body['r'], atol=2e-9, rtol=0)

    def test_jog_and_unreachable_rollback(self):
        robot = Robot(); jog = CartesianJog(robot)
        robot.q = self.reference['initial']
        self.assertEqual(jog.level_wrist(), self.reference['levelled'])
        self.assert_pose(robot, self.reference['level'])
        for case in self.reference['moves']:
            self.assertEqual(jog.step(**case['motion']), case['ok'])
            self.assert_pose(robot, case)

    def test_position_ik(self):
        robot = Robot()
        for case in self.reference['targets']:
            robot.q = case['start']; result = robot.solve(case['target'])
            self.assertEqual(result['converged'], case['result']['converged'])
            self.assert_pose(robot, case)


class KeyboardTests(unittest.TestCase):
    def test_enable_rates_keys_and_stop(self):
        robot = Robot(); c = Controller(robot)
        q = robot.q
        c.key('j', True); self.assertFalse(c.tick()); np.testing.assert_array_equal(q, robot.q)
        c.key('e', True)
        for _ in range(200):
            before = robot.display(); c.tick()
            self.assertTrue(np.all(np.abs(robot.display()-before) <= c.rates*.08+1e-8))
            if c.level_target is None: break
        self.assertIsNone(c.level_target)
        self.assertAlmostEqual(c.jog.pitch(), 0, places=6)
        c.key('j', True); c.key('j', False)
        for _ in range(100): c.tick()
        self.assertAlmostEqual(robot.display()[4], 90)
        c.key('l', True); c.key('l', False)
        for _ in range(100): c.tick()
        self.assertAlmostEqual(robot.display()[4], -90)
        c.key('w', True); before = robot.pose()[0]; pitch = c.jog.pitch()
        c.tick(); self.assertGreater(robot.pose()[0][2], before[2])
        self.assertLess(abs(c.jog.pitch()-pitch), .0002)
        c.key('w', False); q = robot.q; c.tick(); np.testing.assert_array_equal(q, robot.q)
        c.key('j', True); c.tick(); c.key('escape', True); q = robot.q
        c.tick(); np.testing.assert_array_equal(q, robot.q)
        self.assertFalse(c.enabled); self.assertIsNone(c.gripper_target)


def test_config():
    # Synthetic calibration for mocked hardware only, not this real robot.
    s = dict(port='/test', baudrate=115200, sdk_module='scservo_sdk', servo_family='sms_sts',
             position_mode_confirmed=True, calibration_confirmed=True, servos={})
    for i, name in enumerate(NAMES):
        s['servos'][name] = dict(id=i+1, joint_degrees=[-10, 10], raw_positions=[1900, 2100] if i != 4 else [2100, 1900],
                                  raw_limits=[1800, 2200], safe_degrees=[-10, 10], max_deg_s=10, speed=100, acceleration=20)
    return s


class FakeGroup:
    def __init__(self): self.pending = []; self.packets = []; self.result = 0
    def clearParam(self): self.pending = []
    def txPacket(self): self.packets.append(self.pending[:]); return self.result


class FakePort:
    def __init__(self, path): self.path = path; self.is_open = False
    def setBaudRate(self, baud): self.baud = baud; self.is_open = True; return True
    def closePort(self): self.is_open = False


class FakeHandler:
    def __init__(self, port):
        self.groupSyncWrite = FakeGroup(); self.error = 0; self.pos = 2000; self.mode = 0; self.torque = 1
    def read1ByteTxRx(self, servo_id, address): return (self.torque if address == 40 else self.mode), 0, self.error
    def ReadPos(self, servo_id): return self.pos, 0, self.error
    def SyncWritePosEx(self, servo_id, raw, speed, acc):
        self.groupSyncWrite.pending.append((servo_id, raw, speed, acc)); return True


FAKE_SDK = SimpleNamespace(COMM_SUCCESS=0, PortHandler=FakePort, sms_sts=FakeHandler)


class HardwareBoundaryTests(unittest.TestCase):
    def test_unconfigured_live_refused(self):
        with self.assertRaisesRegex(ValueError, 'calibration'):
            calibrated_joints(config.settings(), Robot())

    def test_calibration_direction_and_round_trip(self):
        maps = calibrated_joints(test_config(), Robot())
        for c in maps:
            for degree in [-10, 0, 10]: self.assertAlmostEqual(c.from_raw(c.to_raw(degree)), degree)
        self.assertLess(maps[4].to_raw(10), maps[4].to_raw(-10))

    def test_reject_bad_calibration(self):
        for field, value in [('id', 1), ('raw_positions', [None, 2000]), ('safe_degrees', [-30, 30]), ('speed', 0)]:
            s = test_config(); s['servos']['shoulder'][field] = value
            with self.assertRaises(ValueError): calibrated_joints(s, Robot())

    def test_bus_startup_read_only_and_all_or_nothing_validation(self):
        s = test_config(); maps = calibrated_joints(s, Robot()); bus = ServoBus(s, maps, FAKE_SDK)
        self.assertEqual(bus.port.baud, 115200)
        self.assertEqual(bus.handler.groupSyncWrite.packets, [])
        np.testing.assert_array_equal(bus.read_degrees(), np.zeros(5))
        bus.write_degrees([1, 2, 3, 4, 5]); packet = bus.handler.groupSyncWrite.packets[-1]
        self.assertEqual([x[0] for x in packet], [1, 2, 3, 4, 5])
        self.assertEqual([x[1] for x in packet], [2010, 2020, 2030, 2040, 1950])
        with self.assertRaises(ValueError): bus.write_degrees([0, 0, 0, 0, 90])
        self.assertEqual(len(bus.handler.groupSyncWrite.packets), 1)
        bus.handler.error = 1
        with self.assertRaises(OSError): bus.read_degrees()
        bus.handler.error = 0; bus.handler.mode = 1
        with self.assertRaises(ValueError): bus.read_degrees(check_mode=True)
        bus.handler.mode = 0; bus.handler.torque = 0
        with self.assertRaises(ValueError): bus.read_degrees(check_mode=True)
        bus.close(); self.assertFalse(bus.port.is_open)

    def test_failed_send_and_feedback_range(self):
        s = test_config(); bus = ServoBus(s, calibrated_joints(s, Robot()), FAKE_SDK)
        bus.handler.groupSyncWrite.result = -1
        with self.assertRaises(OSError): bus.write_degrees([0]*5)
        self.assertEqual(bus.handler.groupSyncWrite.pending, [])
        bus.handler.pos = 4095
        with self.assertRaises(ValueError): bus.read_degrees()


if __name__ == '__main__': unittest.main()
