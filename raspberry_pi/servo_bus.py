"""Waveshare serial forwarding + the vendor's scservo_sdk API.

No EEPROM, servo-ID, zero-offset, operating-mode or torque-enable writes.
Sync writes contain all five independently driven motors; linked jaws/gears are
mechanical and must not be assigned additional motors here.
"""
from dataclasses import dataclass
import importlib
import math
import numpy as np
from kinematics import NAMES


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
            if type(c.servo_id) is not int or not 1 <= c.servo_id <= 253:
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
        if config.get('position_mode_confirmed') is not True:
            raise ValueError('Verify all motors use position mode, then set position_mode_confirmed=true')
        sdk = sdk or importlib.import_module(config.get('sdk_module', 'scservo_sdk'))
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
            self.read_degrees(check_mode=True)
        except Exception:
            if getattr(self.port, 'is_open', False):
                self.port.closePort()
            raise

    def checked(self, result, error=0):
        if result != self.success or error:
            raise OSError(f'Servo communication/error status: result={result}, error={error}')

    def read_degrees(self, check_mode=False):
        values = []
        for c in self.maps:
            if check_mode:
                torque, result, error = self.handler.read1ByteTxRx(c.servo_id, 40)
                self.checked(result, error)
                if torque != 1:
                    raise ValueError(f'ID {c.servo_id}: torque is not enabled; support arm and enable it using vendor tools')
                if self.config['servo_family'] == 'sms_sts':
                    mode, result, error = self.handler.read1ByteTxRx(c.servo_id, 33)
                    self.checked(result, error)
                    if mode != 0:
                        raise ValueError(f'ID {c.servo_id}: not in position mode')
                else:
                    lo, result, error = self.handler.read2ByteTxRx(c.servo_id, 9); self.checked(result, error)
                    hi, result, error = self.handler.read2ByteTxRx(c.servo_id, 11); self.checked(result, error)
                    if lo == 0 and hi == 0:
                        raise ValueError(f'ID {c.servo_id}: SC wheel mode detected')
            pos, result, error = self.handler.ReadPos(c.servo_id)
            self.checked(result, error)
            values.append(c.from_raw(pos))
        return np.array(values)

    def write_degrees(self, degrees):
        if len(degrees) != 5:
            raise ValueError('Expected five motor targets')
        targets = [c.to_raw(v) for c, v in zip(self.maps, degrees)]
        group = self.handler.groupSyncWrite
        group.clearParam()
        try:
            for c, raw in zip(self.maps, targets):
                if self.config['servo_family'] == 'sms_sts':
                    ok = self.handler.SyncWritePosEx(c.servo_id, raw, c.speed, c.acceleration)
                else:
                    ok = self.handler.SyncWritePos(c.servo_id, raw, 0, c.speed)
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
