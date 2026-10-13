"""Run the user's unmodified SDK against a byte-level serial emulator.

Set SCSERVO_SDK_PATH to the directory containing the SDK's __init__.py.
No pyserial installation or real serial device is used by these tests.
"""
import importlib.util
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

from test_controller import test_config
from kinematics import Robot
from servo_bus import ServoBus, calibrated_joints


class SerialEmulator:
    instances = []

    def __init__(self, *, port, baudrate, bytesize, timeout):
        if port != '/test':
            raise AssertionError('Tests must never access a physical serial device')
        self.baudrate, self.closed = baudrate, False
        self.packets, self.incoming = [], bytearray()
        self.missing, self.corrupt, self.errors = set(), set(), {}
        self.short_write = False
        self.position = {i: 2000 for i in range(1, 6)}
        self.mode, self.torque = 0, 1
        self.instances.append(self)

    def setRTS(self, value): self.rts = value
    def setDTR(self, value): self.dtr = value
    def reset_input_buffer(self): self.incoming.clear()
    def flush(self): pass
    def close(self): self.closed = True

    def read(self, length):
        data = self.incoming[:length]
        del self.incoming[:length]
        return bytes(data)

    def response(self, servo_id, address, length):
        if servo_id in self.missing:
            return b''
        registers = bytearray(80)
        registers[33], registers[40] = self.mode, self.torque
        registers[56:58] = self.position[servo_id].to_bytes(2, 'little')
        registers[58:60] = (0x8000 | 25).to_bytes(2, 'little')
        registers[66] = 1
        packet = bytearray([255, 255, servo_id, length+2, self.errors.get(servo_id, 0)])
        packet.extend(registers[address:address+length])
        packet.append((~sum(packet[2:])) & 255)
        if servo_id in self.corrupt:
            packet[-1] ^= 1
        return packet

    def write(self, packet):
        packet = bytes(packet)
        assert not self.closed
        assert packet[:2] == b'\xff\xff'
        assert len(packet) == packet[3]+4
        assert packet[-1] == (~sum(packet[2:-1]) & 255)
        self.packets.append(packet)
        if self.short_write:
            return len(packet)-1
        if packet[4] == 2:  # Individual read.
            self.incoming.extend(self.response(packet[2], packet[5], packet[6]))
        elif packet[4] == 0x82:  # Sync read.
            for servo_id in packet[7:-1]:
                self.incoming.extend(self.response(servo_id, packet[5], packet[6]))
        elif packet[4] != 0x83:
            raise AssertionError(f'Unexpected motor instruction: {packet[4]}')
        return len(packet)


@unittest.skipUnless(os.environ.get('SCSERVO_SDK_PATH'), 'Set SCSERVO_SDK_PATH to test the vendor SDK')
class VendorSDKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(os.environ['SCSERVO_SDK_PATH']).resolve()
        spec = importlib.util.spec_from_file_location('_robot_arm_vendor_sdk', path/'__init__.py',
                                                     submodule_search_locations=[str(path)])
        cls.sdk = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = cls.sdk
        # port_handler.py retains this mock module. Every packet still passes
        # through the real PortHandler, sms_sts and GroupSyncRead/Write code.
        with patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=SerialEmulator, EIGHTBITS=8)}):
            spec.loader.exec_module(cls.sdk)

    def setUp(self):
        self.settings = test_config()
        self.settings['sync_read'] = True
        self.bus = ServoBus(self.settings, calibrated_joints(self.settings, Robot()), self.sdk)
        self.addCleanup(self.bus.close)
        self.serial = self.bus.port.ser

    def test_individual_reads_discard_bytes_from_previous_transaction(self):
        self.bus.reader = None
        self.serial.incoming.extend(b'\xff\xff\x01\x03\x00\x00\x00')
        np.testing.assert_array_equal(self.bus.read_degrees(check_mode=True), [0]*5)
        self.serial.corrupt = {1}
        with self.assertRaisesRegex(OSError, 'ID 1.*Incorrect status packet'):
            self.bus.read_degrees()

    def test_startup_only_reads_at_requested_baud(self):
        self.assertEqual(self.serial.baudrate, 115200)
        self.assertFalse(self.serial.rts)
        self.assertFalse(self.serial.dtr)
        self.assertEqual([p[4] for p in self.serial.packets], [2]*10+[0x82])
        self.assertEqual(self.serial.packets[-1][5:-1], bytes([56, 11, 1, 2, 3, 4, 5]))
        np.testing.assert_array_equal(self.bus.read_degrees(), [0]*5)

    def test_trajectory_write_scales_each_motor_speed_to_interval(self):
        self.bus.write_degrees([1, 2, 0, 4, 5], previous=[0]*5, duration=.2)
        payload = self.serial.packets[-1][7:-1]
        speeds = [int.from_bytes(payload[i*8+6:i*8+8], 'little') for i in range(5)]
        self.assertEqual(speeds, [50, 100, 1, 100, 100])
        count = len(self.serial.packets)
        with self.assertRaises(ValueError):
            self.bus.write_degrees([0]*5, previous=[0]*5, duration=0)
        self.assertEqual(len(self.serial.packets), count)

    def test_health_checks_can_be_distributed_without_skipping_positions(self):
        self.bus.reader = None
        before = len(self.serial.packets)
        self.bus.read_degrees(check_mode=True, health_ids=[3])
        packets = self.serial.packets[before:]
        self.assertEqual([(p[2], p[5]) for p in packets if p[5] in (33,40)], [(3,40),(3,33)])
        self.assertEqual([p[2] for p in packets if p[5] == 56], [1,2,3,4,5])

    def test_sync_write_exact_registers_and_five_targets(self):
        self.bus.write_degrees([1, 2, 3, 4, 5])
        packet = self.serial.packets[-1]
        self.assertEqual(packet[2], 254)
        self.assertEqual(packet[4:7], bytes([0x83, 41, 7]))
        targets = [2010, 2020, 2030, 2040, 1950]
        expected = b''.join(bytes([i, 20])+raw.to_bytes(2, 'little')+bytes([0, 0, 100, 0])
                            for i, raw in enumerate(targets, 1))
        self.assertEqual(packet[7:-1], expected)
        self.assertEqual(self.bus.handler.groupSyncWrite.data_dict, {})

    def test_missing_early_id_rejected_even_if_last_id_succeeds(self):
        self.serial.missing = {1}
        with self.assertRaisesRegex(OSError, 'ID 1.*incomplete'):
            self.bus.read_degrees()
        self.assertEqual(self.bus.reader.data_dict, {})

    def test_bad_checksum_in_early_id_rejected(self):
        self.serial.corrupt = {2}
        with self.assertRaisesRegex(OSError, 'ID 2.*corrupt'):
            self.bus.read_degrees()

    def test_timeout_after_success_never_reuses_old_feedback(self):
        self.bus.read_degrees()
        self.serial.missing = set(range(1, 6))
        with self.assertRaisesRegex(OSError, 'no status packet'):
            self.bus.read_degrees()
        self.assertEqual(self.bus.reader.data_dict, {})

    def test_servo_error_reports_id_and_vendor_description(self):
        self.serial.errors[3] = 4
        with self.assertRaisesRegex(OSError, 'ID 3.*Overheat'):
            self.bus.read_degrees()

    def test_signed_position_and_range_are_checked(self):
        self.serial.position[1] = 0x8000 | 2000
        with self.assertRaisesRegex(ValueError, 'feedback outside raw range'):
            self.bus.read_degrees()

    def test_mode_and_torque_checked_during_motion(self):
        self.serial.mode = 1
        with self.assertRaisesRegex(ValueError, 'not in position mode'):
            self.bus.read_degrees(check_mode=True)
        self.serial.mode, self.serial.torque = 0, 0
        with self.assertRaisesRegex(ValueError, 'torque is not enabled'):
            self.bus.read_degrees(check_mode=True)

    def test_short_write_fails_and_clears_pending_targets(self):
        self.serial.short_write = True
        with self.assertRaisesRegex(OSError, 'Failed transmit'):
            self.bus.write_degrees([0]*5)
        self.assertEqual(self.bus.handler.groupSyncWrite.data_dict, {})

    def test_invalid_target_emits_no_packet_and_close_blocks_io(self):
        before = len(self.serial.packets)
        with self.assertRaises(ValueError):
            self.bus.write_degrees([0, 0, 0, 0, 90])
        self.assertEqual(len(self.serial.packets), before)
        self.bus.close()
        self.assertTrue(self.serial.closed)
        with self.assertRaisesRegex(OSError, 'closed'):
            self.bus.read_degrees()
        with self.assertRaisesRegex(OSError, 'closed'):
            self.bus.write_degrees([0]*5)

    def test_explicit_individual_read_option(self):
        self.settings['sync_read'] = False
        with_bus = ServoBus(self.settings, self.bus.maps, self.sdk)
        self.addCleanup(with_bus.close)
        np.testing.assert_array_equal(with_bus.read_degrees(), [0]*5)
        self.assertTrue(all(p[4] == 2 for p in with_bus.port.ser.packets))


if __name__ == '__main__':
    unittest.main()
