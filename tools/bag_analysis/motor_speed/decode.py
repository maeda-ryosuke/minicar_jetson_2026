"""recover.py の出力から必要トピックを CDR デコードして npz に保存する。"""
import pickle
import struct
import sys

import numpy as np

d = pickle.load(open(sys.argv[1], 'rb'))
topics, msgs = d['topics'], d['msgs']
name2id = {v[0]: k for k, v in topics.items()}


class R:
    def __init__(self, b):
        self.b, self.i = b, 4  # CDR encapsulation header

    def al(self, n):
        r = (self.i - 4) % n
        if r:
            self.i += n - r

    def f64(self, k=1):
        self.al(8); v = struct.unpack_from('<%dd' % k, self.b, self.i); self.i += 8 * k
        return v if k > 1 else v[0]

    def u32(self):
        self.al(4); v = struct.unpack_from('<I', self.b, self.i)[0]; self.i += 4
        return v

    def i32(self):
        self.al(4); v = struct.unpack_from('<i', self.b, self.i)[0]; self.i += 4
        return v

    def u8(self):
        v = self.b[self.i]; self.i += 1
        return v

    def s(self):
        n = self.u32(); v = self.b[self.i:self.i + n - 1].decode('utf8', 'replace'); self.i += n
        return v

    def stamp(self):
        return self.i32() + self.u32() * 1e-9


def rows(name):
    tid = name2id[name]
    out = [(ts, data) for t, ts, data in msgs.values() if t == tid]
    out.sort()
    return out


def twist(name):
    return np.array([(ts * 1e-9, *R(b).f64(6)) for ts, b in rows(name)])


def odom(name):
    res = []
    for ts, b in rows(name):
        r = R(b); st = r.stamp(); r.s(); r.s()
        pos = r.f64(3); q = r.f64(4); r.f64(36)
        tw = r.f64(6)
        yaw = np.arctan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        res.append((ts * 1e-9, st, pos[0], pos[1], yaw, tw[0], tw[1], tw[5]))
    return np.array(res)


def f64multi(name):
    res = []
    for ts, b in rows(name):
        r = R(b); n = r.u32()
        for _ in range(n):
            r.s(); r.u32(); r.u32()
        r.u32(); k = r.u32()
        res.append((ts * 1e-9, *r.f64(k)) if k > 1 else (ts * 1e-9, r.f64()))
    return np.array(res)


def f64(name):
    return np.array([(ts * 1e-9, R(b).f64()) for ts, b in rows(name)])


def imu(name):
    res = []
    for ts, b in rows(name):
        r = R(b); st = r.stamp(); r.s()
        r.f64(4); r.f64(9); g = r.f64(3); r.f64(9); a = r.f64(3)
        res.append((ts * 1e-9, st, *g, *a))
    return np.array(res)


logs = []
for ts, b in rows('/rosout'):
    r = R(b); st = r.stamp(); lvl = r.u8(); nm = r.s(); m = r.s()
    logs.append((ts * 1e-9, lvl, nm, m))

SPEC = dict(cmd_raw=(twist, '/cmd_vel_raw'), cmd=(twist, '/cmd_vel'),
            odom=(odom, '/odom'), ekf=(odom, '/odometry/filtered'),
            pwm=(f64multi, '/motor/pwm_count'), u=(f64multi, '/motor/cmd_normalized'),
            steer=(f64, '/motor/steer_angle'), imu=(imu, '/imu'),
            sc=(f64multi, '/motor/speed_ctrl'))
out = {k: fn(t) for k, (fn, t) in SPEC.items() if t in name2id}
np.savez(sys.argv[2], **out)
pickle.dump(logs, open(sys.argv[2] + '.logs.pkl', 'wb'))
for k, v in out.items():
    print(k, v.shape, v[:2].round(3).tolist())
