"""Waveshare serial forwarding + the vendor's scservo_sdk API.

No EEPROM, servo-ID, zero-offset, operating-mode or torque-enable writes.
Sync writes contain all five independently driven motors; linked jaws/gears are
mechanical and must not be assigned additional motors here.
"""
from dataclasses import dataclass
import importlib
import math
import sys
import time
from pathlib import Path
import numpy as np
from kinematics import NAMES


def load_sdk(settings):
    """Use an installed SDK, or the permanently configured vendor directory."""
    module = settings.get('sdk_module', 'scservo_sdk')
    try:
        return importlib.import_module(module)
    except ModuleNotFoundError as ex:
        if ex.name != module.split('.')[0]:
            raise  # Missing SDK dependency, rather than the SDK itself.
        parent = settings.get('sdk_parent_dir')
        if not parent or not (Path(parent).expanduser() / module.split('.')[0]).is_dir():
            raise ImportError(f'{module} not found. Set SDK_PARENT_DIR in raspberry_pi/config.py to the folder containing the SDK.') from ex
        parent = str(Path(parent).expanduser().resolve())
        sys.path.insert(0, parent)
        try:
            return importlib.import_module(module)
        finally:
            sys.path.remove(parent)


@dataclass
class Calibration:
    name: str
    servo_id: int
    degrees: np.ndarray
    ticks: np.ndarray
    raw_limits: np.ndarray
    safe_degrees: np.ndarray
    max_deg_s: float
    speed: int
    acceleration: int

    @classmethod
    def load(cls, name, data, family):
        try:
            c = cls(name, data['id'], np.asarray(data['joint_degrees'], float),
                    np.asarray(data['raw_positions'], float), np.asarray(data['raw_limits'], float),
                    np.asarray(data['safe_degrees'], float), float(data['max_deg_s']),
                    data['speed'], data['acceleration'])
            for pair in (c.degrees, c.ticks, c.raw_limits, c.safe_degrees):
                if pair.shape != (2,) or not np.isfinite(pair).all():
                    raise ValueError('Fill both measured calibration values')
            if type(c.servo_id) is not int or not 1 <= c.servo_id <= 252:
                raise ValueError('Invalid servo ID')
            if not c.degrees[0] < c.degrees[1] or c.ticks[0] == c.ticks[1]:
                raise ValueError('Calibration needs two distinct positions and increasing joint degrees')
            raw_max = 4095 if family == 'sms_sts' else 1023
            if not 0 <= c.raw_limits[0] < c.raw_limits[1] <= raw_max:
                raise ValueError('Invalid raw position limits for selected family')
            if not np.equal(c.raw_limits, np.round(c.raw_limits)).all():
                raise ValueError('Raw limits must be integer ticks')
            if not c.degrees[0] <= c.safe_degrees[0] < c.safe_degrees[1] <= c.degrees[1]:
                raise ValueError('Safe travel must lie within measured calibration span')
            if not math.isfinite(c.max_deg_s) or not 0 < c.max_deg_s <= 90:
                raise ValueError('max_deg_s must be positive and at most 90')
            if type(c.speed) is not int or not 1 <= c.speed <= 3000:
                raise ValueError('Set nonzero servo speed in 1..3000')
            if type(c.acceleration) is not int or not 0 <= c.acceleration <= 254:
                raise ValueError('Invalid acceleration')
            for degree in (*c.degrees, *c.safe_degrees):
                raw = c.to_raw_unchecked(degree)
                if not c.raw_limits[0] <= raw <= c.raw_limits[1]:
                    raise ValueError('Calibration lies outside raw limits')
            return c
        except (KeyError, TypeError, ValueError) as ex:
            raise ValueError(f'{name}: {ex}') from ex

    def to_raw_unchecked(self, degrees):
        return self.ticks[0]+(degrees-self.degrees[0])*(self.ticks[1]-self.ticks[0])/(self.degrees[1]-self.degrees[0])

    def to_raw(self, degrees):
        if not math.isfinite(degrees) or not self.safe_degrees[0]-1e-7 <= degrees <= self.safe_degrees[1]+1e-7:
            raise ValueError(f'{self.name}: outside calibrated safe joint range')
        raw = int(round(self.to_raw_unchecked(degrees)))
        if not self.raw_limits[0] <= raw <= self.raw_limits[1]:
            raise ValueError(f'{self.name}: outside raw servo range')
        return raw

    def from_raw(self, raw):
        if not math.isfinite(raw) or not self.raw_limits[0] <= raw <= self.raw_limits[1]:
            raise ValueError(f'{self.name}: feedback outside raw range')
        degree = self.degrees[0]+(raw-self.ticks[0])*(self.degrees[1]-self.degrees[0])/(self.ticks[1]-self.ticks[0])
        # Half a tick of quantization is allowed only on feedback, not commands.
        epsilon = abs((self.degrees[1]-self.degrees[0])/(self.ticks[1]-self.ticks[0]))*.51
        if not self.safe_degrees[0]-epsilon <= degree <= self.safe_degrees[1]+epsilon:
            raise ValueError(f'{self.name}: measured pose outside safe joint range')
        return float(np.clip(degree, *self.safe_degrees))


def calibrated_joints(config, robot):
    if config.get('calibration_confirmed') is not True:
        raise ValueError('Live motion requires measured calibration and calibration_confirmed=true')
    family = config.get('servo_family')
    if family not in ('sms_sts', 'scscl'):
        raise ValueError('Set servo_family to sms_sts (ST) or scscl (SC) after checking motors')
    maps = [Calibration.load(n, config['servos'][n], family) for n in NAMES]
    if len({c.servo_id for c in maps}) != 5:
        raise ValueError('All five servo IDs must be unique')
    for i, c in enumerate(maps):
        a, b = sorted((c.safe_degrees-robot.offset[i])/robot.gain[i])
        if a < robot.lo[i]-1e-6 or b > robot.hi[i]+1e-6:
            raise ValueError(f'{c.name}: safe limits exceed the CAD joint limits')
    # Compute display calibration first, then restrict mechanical travel.
    for i, c in enumerate(maps):
        robot.lo[i], robot.hi[i] = sorted((c.safe_degrees-robot.offset[i])/robot.gain[i])
    return maps


class ServoBus:
    def __init__(self, config, maps, sdk=None):
        self.maps, self.config, self.closed = maps, config, False
        self.transaction_gap = config.get('serial_transaction_gap_seconds', 0)
        if not math.isfinite(self.transaction_gap) or not 0 <= self.transaction_gap <= .02:
            raise ValueError('Serial transaction gap must be between 0 and 0.02 seconds')
        if config.get('position_mode_confirmed') is not True:
            raise ValueError('Verify all motors use position mode, then set position_mode_confirmed=true')
        sdk = sdk or load_sdk(config)
        family = config['servo_family']
        if not hasattr(sdk, family):
            raise ValueError(f'SDK must provide PortHandler and {family}; see README for supported vendor SDK')
        self.success = sdk.COMM_SUCCESS
        self.port = sdk.PortHandler(config['port'])
        try:
            # Waveshare PortHandler.setBaudRate opens the port at that baud.
            if not self.port.setBaudRate(config['baudrate']):
                raise OSError('Could not open serial port at configured baud')
            self.handler = getattr(sdk, family)(self.port)
            self.reader = None
            if family == 'sms_sts' and config.get('sync_read', True):
                # Same 11-byte position-through-moving block as the user's
                # sync_read_write.py. Mode/torque are checked separately.
                self.reader = sdk.GroupSyncRead(self.handler, 56, 11)
            self.read_degrees(check_mode=True)
        except Exception:
            if getattr(self.port, 'is_open', False):
                self.port.closePort()
            raise

    def checked(self, result, error=0, servo_id=None):
        if result != self.success or error:
            detail = []
            if result != self.success and hasattr(self.handler, 'getTxRxResult'):
                detail.append(self.handler.getTxRxResult(result))
            if error and hasattr(self.handler, 'getRxPacketError'):
                detail.append(self.handler.getRxPacketError(error))
            label = f'ID {servo_id}' if servo_id is not None else 'Servo bus'
            raise OSError(f'{label}: result={result}, error={error}; {"; ".join(filter(None, detail))}')

    def prepare_read(self):
        # All prior transactions are complete. Drain TX, allow forwarding to
        # settle, then discard bytes predating this fresh request. The SDK's
        # clearPort() only flushes TX; it does not reset its RX buffer.
        serial = getattr(self.port, 'ser', None)
        if serial is not None:
            serial.flush()
        if self.transaction_gap:
            time.sleep(self.transaction_gap)
        if serial is not None:
            serial.reset_input_buffer()

    def read_register(self, servo_id, address, word=False):
        self.prepare_read()
        read = self.handler.read2ByteTxRx if word else self.handler.read1ByteTxRx
        value, result, error = read(servo_id, address)
        self.checked(result, error, servo_id)
        return value

    def read_degrees(self, check_mode=False, health_ids=None):
        if self.closed:
            raise OSError('Servo bus is closed')
        values = []
        for c in self.maps:
            if check_mode and (health_ids is None or c.servo_id in health_ids):
                torque = self.read_register(c.servo_id, 40)
                if torque != 1:
                    raise ValueError(f'ID {c.servo_id}: torque is not enabled; support arm and enable it using vendor tools')
                if self.config['servo_family'] == 'sms_sts':
                    mode = self.read_register(c.servo_id, 33)
                    if mode != 0:
                        raise ValueError(f'ID {c.servo_id}: not in position mode')
                else:
                    lo = self.read_register(c.servo_id, 9, word=True)
                    hi = self.read_register(c.servo_id, 11, word=True)
                    if lo == 0 and hi == 0:
                        raise ValueError(f'ID {c.servo_id}: SC wheel mode detected')
            if self.reader is None:
                self.prepare_read()
                pos, result, error = self.handler.ReadPos(c.servo_id)
                self.checked(result, error, c.servo_id)
                values.append(c.from_raw(pos))
        if self.reader is not None:
            values = self._read_sync_degrees()
        return np.array(values)

    def _read_sync_degrees(self):
        group = self.reader
        # This SDK retains data across reads and can report COMM_SUCCESS when
        # an earlier ID failed but the last ID succeeded. Never reuse a sample.
        group.clearParam()
        try:
            for c in self.maps:
                if not group.addParam(c.servo_id):
                    raise OSError(f'Could not queue feedback for ID {c.servo_id}')
            self.prepare_read()
            self.checked(group.txRxPacket())
            values = []
            for c in self.maps:
                # The supplied SDK returns (available, servo_error), NOT bool.
                available, error = group.isAvailable(c.servo_id, 56, 11)
                if not available:
                    raise OSError(f'ID {c.servo_id}: incomplete or corrupt sync-read feedback')
                self.checked(self.success, error, c.servo_id)
                raw = self.handler.scs_tohost(group.getData(c.servo_id, 56, 2), 15)
                values.append(c.from_raw(raw))
            if not group.last_result:
                raise OSError('Incomplete sync-read feedback')
            return values
        finally:
            group.clearParam()

    def write_degrees(self, degrees, *, previous=None, duration=None):
        if self.closed:
            raise OSError('Servo bus is closed')
        if len(degrees) != 5:
            raise ValueError('Expected five motor targets')
        targets = [c.to_raw(v) for c, v in zip(self.maps, degrees)]
        speeds = [c.speed for c in self.maps]
        if duration is not None or previous is not None:
            if duration is None or previous is None or not math.isfinite(duration) or duration <= 0:
                raise ValueError('Trajectory writes need a positive duration and previous pose')
            if len(previous) != 5:
                raise ValueError('Expected five previous motor angles')
            starts = [c.to_raw(v) for c, v in zip(self.maps, previous)]
            # Each motor takes approximately one update interval to reach its
            # next target, instead of rushing there at the fixed speed cap.
            speeds = [max(1, min(c.speed, math.ceil(abs(raw-start)/duration)))
                      for c, raw, start in zip(self.maps, targets, starts)]
        group = self.handler.groupSyncWrite
        group.clearParam()
        try:
            for c, raw, speed in zip(self.maps, targets, speeds):
                if self.config['servo_family'] == 'sms_sts':
                    ok = self.handler.SyncWritePosEx(c.servo_id, raw, speed, c.acceleration)
                else:
                    ok = self.handler.SyncWritePos(c.servo_id, raw, 0, speed)
                if not ok:
                    raise OSError(f'Could not queue servo ID {c.servo_id}')
            self.checked(group.txPacket())
        finally:
            group.clearParam()

    def hold_measured(self):
        # Best-effort stop at current measured positions; may fail if bus is lost.
        self.write_degrees(self.read_degrees())

    def close(self):
        if not self.closed:
            self.port.closePort(); self.closed = True
