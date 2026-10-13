"""NumPy port of dist/engine.js, jog.js and joint-angles.js.

Reads the same MJCF and coupling metadata as the browser. No meshes, renderer,
MuJoCo installation or Raspberry Pi hardware are required for these equations.
This parser intentionally supports only the checked-in hinge/slide model.
"""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
import numpy as np

MODEL_DIR = Path(__file__).resolve().parents[1] / 'dist' / 'robot'
NAMES = ('base_yaw', 'shoulder', 'elbow', 'wrist_pitch', 'gripper_drive')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rotation(axis, angle):
    x, y, z = axis / np.linalg.norm(axis)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1-math.cos(angle)) * (k @ k)


def quaternion(values):
    w, x, y, z = values / np.linalg.norm(values)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def solve3(a, b):
    """Same pivoted three-by-three solve used by the JavaScript solver."""
    m = np.column_stack((a, b)).astype(float)
    for i in range(3):
        pivot = i + int(np.argmax(np.abs(m[i:, i])))
        m[[i, pivot]] = m[[pivot, i]]
        if abs(m[i, i]) < 1e-15:
            return np.zeros(3)
        m[i, i:] /= m[i, i]
        for j in range(3):
            if i != j:
                m[j, i:] -= m[j, i] * m[i, i:]
    return m[:, 3]


class Robot:
    def __init__(self, model_dir=MODEL_DIR):
        model_dir = Path(model_dir)
        self.xml = ET.parse(model_dir / 'engg1101.xml').getroot()
        self.couplings = json.loads((model_dir/'model.json').read_text())['couplings']
        self.joints = {j.get('name'): j for j in self.xml.findall('.//body/joint')}
        if self.xml.find('compiler').get('angle') != 'radian':
            raise ValueError('Expected radian MJCF')
        for body in self.xml.findall('.//body'):
            if len(body.findall('joint')) > 1 or any(k in body.attrib for k in ('euler', 'axisangle', 'xyaxes', 'zaxis')):
                raise ValueError('Unsupported model transform')
        for j in self.joints.values():
            if j.get('type') not in ('hinge', 'slide'):
                raise ValueError('Expected hinge/slide joints')
        self.lo, self.hi = np.array([self.limits(n) for n in NAMES]).T
        self.gain = np.full(5, 180/math.pi)
        self.offset = np.zeros(5)
        self.offset[0] = -18.8
        self.lo[0], self.hi[0] = np.deg2rad([18.8-180, 18.8+180])
        self.offset[1] = 90-self.hi[1]*self.gain[1]
        self.offset[2] = -90-self.lo[2]*self.gain[2]
        self.offset[3] = 90-self.hi[3]*self.gain[3]
        self.gain[4] = -180/(self.hi[4]-self.lo[4])
        self.offset[4] = 90-self.gain[4]*self.lo[4]
        self.q = np.zeros(5)

    def limits(self, name):
        j = self.joints[name]
        return list(map(float, j.get('range').split())) if j.get('limited') == 'true' else [-math.pi, math.pi]

    @property
    def q(self):
        return self._q.copy()

    @q.setter
    def q(self, value):
        value = np.asarray(value, dtype=float)
        if value.shape != (5,) or not np.isfinite(value).all():
            raise ValueError('Expected five finite joint values')
        self._q = np.clip(value, self.lo, self.hi)
        self.forward()

    def display(self, q=None):
        return self.gain * (self.q if q is None else np.asarray(q)) + self.offset

    def from_display(self, degrees):
        return (np.asarray(degrees, dtype=float)-self.offset)/self.gain

    def forward(self):
        values = dict(zip(NAMES, self._q))
        for c in self.couplings:
            values[c['slave']] = values[c['master']]*c['multiplier']+c['offset']
        self.values, self.bodies = values, {}

        def visit(body, parent_p, parent_r):
            local_p = np.fromstring(body.get('pos', '0 0 0'), sep=' ')
            local_r = quaternion(np.fromstring(body.get('quat', '1 0 0 0'), sep=' '))
            j = body.find('joint')
            if j is not None:
                axis = np.fromstring(j.get('axis', '0 0 1'), sep=' ')
                axis /= np.linalg.norm(axis)
                value = values[j.get('name')]
                if j.get('type') == 'slide':
                    local_p += local_r @ (axis*value)
                else:
                    rot = rotation(axis, value)
                    anchor = np.fromstring(j.get('pos', '0 0 0'), sep=' ')
                    local_p += local_r @ (anchor-rot@anchor)
                    local_r = local_r @ rot
            p, r = parent_p+parent_r@local_p, parent_r@local_r
            self.bodies[body.get('name')] = (p, r)
            for child in body.findall('body'):
                visit(child, p, r)
        for body in self.xml.find('worldbody').findall('body'):
            visit(body, np.zeros(3), np.eye(3))

    def pose(self):
        p, r = self.bodies['tool_tip']
        return p.copy(), r.copy()

    def solve(self, target, iterations=180, tolerance=.0005):
        """Position-only browser IK. Leaves best pose; caller decides whether to move."""
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError('Expected three finite target coordinates')
        initial, best, best_error, used = self.q, self.q, math.inf, 0
        for trial in range(5):
            self.q = initial if trial == 0 else [initial[i]+math.sin((trial+1)*(i+2))*.6 if i < 4 else initial[i] for i in range(5)]
            for _ in range(iterations):
                used += 1
                q, p = self.q, self.pose()[0]
                error = target-p
                err = float(np.linalg.norm(error))
                if err < best_error:
                    best_error, best = err, q.copy()
                if err < tolerance:
                    self.q = best
                    return dict(converged=True, error=err, iterations=used)
                columns = np.zeros((3, 5))
                for i in range(4):
                    h = 1e-5 if q[i]+1e-5 <= self.hi[i] else -1e-5
                    t = q.copy(); t[i] += h; self.q = t
                    columns[:, i] = (self.pose()[0]-p)/h
                self.q = q
                v = solve3(columns@columns.T+np.eye(3)*.0001, error)
                self.q = q+np.clip(columns.T@v, -.15, .15)
        self.q = best
        return dict(converged=False, error=best_error, iterations=used)


class CartesianJog:
    def __init__(self, robot):
        self.robot = robot

    def forward(self):
        r = self.robot.pose()[1]
        return np.array([-r[1, 0], r[0, 0], 0])/math.hypot(r[0, 0], r[1, 0])

    def pitch(self):
        r = self.robot.pose()[1]
        return math.atan2(r[2, 2], r[:, 2]@self.forward())

    def solve(self, position, pitch):
        e, position = self.robot, np.asarray(position)
        if not np.isfinite([*position, pitch]).all():
            return False
        initial, f, weight = e.q, self.forward(), .12
        def sample():
            p = e.pose()[0]
            return np.array([p@f, p[2], self.pitch()])
        goal = np.array([position@f, position[2], pitch])
        for _ in range(100):
            q, s = e.q, sample()
            error = goal-s; error[2] = wrap(error[2])*weight
            if np.linalg.norm(e.pose()[0]-position) < .00002 and abs(wrap(pitch-self.pitch())) < .0002:
                return True
            columns = np.zeros((3, 3))
            for col, i in enumerate((1, 2, 3)):
                h = 1e-5 if q[i]+1e-5 <= e.hi[i] else -1e-5
                t = q.copy(); t[i] += h; e.q = t; sp = sample()
                columns[:, col] = [(sp[0]-s[0])/h, (sp[1]-s[1])/h, wrap(sp[2]-s[2])*weight/h]
            e.q = q
            v = solve3(columns@columns.T+np.eye(3)*1e-6, error)
            q[1:4] += np.clip(columns.T@v, -.12, .12); e.q = q
        e.q = initial
        return False

    def step(self, height=0, reach=0, yaw=0, pitch=0):
        e, initial, held_pitch = self.robot, self.robot.q, self.pitch()
        if yaw:
            q = e.q; q[0] += yaw
            if not e.lo[0] <= q[0] <= e.hi[0]:
                return False
            e.q = q
        p = e.pose()[0]+self.forward()*reach; p[2] += height
        if not height and not reach and not pitch:
            return True
        if self.solve(p, held_pitch+pitch):
            return True
        e.q = initial
        return False

    def level_wrist(self):
        e, q = self.robot, self.robot.q
        choices = [q[3]-self.pitch()+k*2*math.pi for k in (-1, 0, 1)]
        choices = [v for v in choices if e.lo[3] <= v <= e.hi[3]]
        if not choices:
            return False
        q[3] = min(choices, key=lambda v: abs(v-q[3])); e.q = q
        return True
