"""Keyboard state and bounded setpoint generation, independent of I/O."""
import math
import numpy as np
from kinematics import CartesianJog

MOVE_KEYS = {'w', 's', 'a', 'd', 'up', 'down', 'left', 'right'}
CONTROL_KEYS = MOVE_KEYS | {'j', 'l'}


class Controller:
    def __init__(self, robot, max_deg_s=(10, 10, 10, 10, 30), speed_scale=.25):
        self.robot, self.jog = robot, CartesianJog(robot)
        self.rates = np.asarray(max_deg_s, float)
        if self.rates.shape != (5,) or not np.isfinite(self.rates).all() or np.any(self.rates <= 0):
            raise ValueError('Five positive joint rate limits required')
        if not math.isfinite(speed_scale) or not 0 < speed_scale <= 1:
            raise ValueError('speed_scale must be in (0, 1]')
        self.speed_scale = speed_scale
        self.held, self.enabled, self.level_target, self.gripper_target = set(), False, None, None
        self.status = 'Off'

    def stop(self):
        self.enabled = False; self.held.clear()
        self.level_target = self.gripper_target = None
        self.status = 'Off - holding position'

    def enable(self):
        initial = self.robot.q
        if not self.jog.level_wrist():
            self.status = 'Cannot level at current wrist limits'; return False
        self.level_target = self.robot.q[3]
        self.robot.q = initial
        self.held.clear(); self.gripper_target = None; self.enabled = True
        self.status = 'Levelling wrist'; return True

    def key(self, key, down):
        key = key.lower()
        if not down:
            self.held.discard(key); return
        if key == 'escape':
            self.stop(); return
        if key == 'e':
            if self.enabled: self.stop()
            else: self.enable()
            return
        if not self.enabled or key not in CONTROL_KEYS or key in self.held:
            return
        self.held.add(key)
        if key in ('j', 'l'):
            desired = (90 if key == 'j' else -90)
            raw = (desired-self.robot.offset[4])/self.robot.gain[4]
            self.gripper_target = float(np.clip(raw, self.robot.lo[4], self.robot.hi[4]))
            self.status = 'Opening gripper' if key == 'j' else 'Closing gripper'

    def tick(self, dt=.08):
        if not self.enabled:
            return False
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Invalid timestep')
        # Never catch up a delayed loop with a large movement.
        dt = min(dt, .08)
        before = self.robot.q
        max_step = self.rates*dt/np.abs(self.robot.gain)
        q = before.copy()
        if self.level_target is not None:
            q[3] += np.clip(self.level_target-q[3], -max_step[3], max_step[3])
            self.robot.q = q
            if abs(q[3]-self.level_target) < 1e-9:
                self.level_target = None; self.status = 'On - pitch held'
        else:
            v = lambda key: int(key in self.held)
            motion = dict(height=(v('w')-v('s'))*.0025,
                          reach=(v('up')-v('down'))*.0025,
                          yaw=(v('right')-v('left'))*math.pi/120,
                          pitch=(v('a')-v('d'))*math.pi/120)
            if any(motion.values()):
                moved = False
                # Reduce the Cartesian step if a near-singular pose needs too
                # much joint motion; retain the pitch constraint at each solve.
                for attempt in range(8):
                    self.robot.q = before
                    scale = self.speed_scale*(dt/.08)/(2**attempt)
                    if self.jog.step(**{k:x*scale for k, x in motion.items()}):
                        if np.all(np.abs(self.robot.q-before) <= max_step+1e-10):
                            moved = True; break
                if not moved:
                    self.robot.q = before
                self.status = 'On - pitch held' if moved else 'Limit reached'
        if self.gripper_target is not None:
            q = self.robot.q
            q[4] += np.clip(self.gripper_target-q[4], -max_step[4], max_step[4])
            self.robot.q = q
            if abs(q[4]-self.gripper_target) < 1e-9:
                self.gripper_target = None
        return not np.array_equal(before, self.robot.q)
