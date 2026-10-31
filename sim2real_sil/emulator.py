#!/usr/bin/env python3
"""Hardware emulator for software-in-the-loop (SIL) runs of the real deploy stack.

The robot's own ROS 2 binaries (unified_node, state machine, safety node, ...) run unmodified. They talk to
"hardware" that is really this process:

* CAN: under the LD_PRELOAD shim (simcan/libsimcan.so) the drivers' SocketCAN sockets connect to the UNIX
  SEQPACKET sockets this process serves at $SIMCAN_DIR/can0 and can1. Every frame is decoded as RobStride
  protocol (enable, disable, mode write/read, MIT operation control) using the motor firmware's own scaling
  ranges, and answered with an operation-status frame, as the motors do.
* IMU: a pseudo-terminal streams WIT IM10A packets (0x51 accel, 0x52 gyro, 0x59 quaternion) at 200 Hz.
  Its slave end is symlinked to $SIMCAN_DIR/ttyIMU and passed to the node as im10a_port.
* Plant: MuJoCo (mjcf/sim2sim/robot_light.mjcf, 2 kHz). The 12 motors act on the joints through the robot's
  calibration (direction, homing offset from calibration.yaml) and, for the ankles, through the parallel
  linkage (A = s*K_P*pitch - K_R*roll, B = -s*K_P*pitch - K_R*roll; joint torques by the transpose).
  Torque limits are the motors' own. A "hand at the pelvis" support (spring-damper on the base) can be faded
  out by the scenario, the way the real robot was held.

The scenario controls the emulator over a datagram socket ($SIMCAN_DIR/ctl, JSON) and gets ground truth back.
Runs in real time; the achieved real-time factor is logged.

MIT License, Copyright (c) 2026 Abhinav Roy
"""
import argparse, json, os, pty, socket, struct, sys, time, tty, errno, signal
import numpy as np
import mujoco
import yaml

EFF = 0x80000000
FRAME = struct.Struct('<IB3x8s')                    # struct can_frame
HOST_ID = 0xFD
OP_CTRL, OP_STATUS, ENABLE, DISABLE, SET_ZERO, READ_PARAM, WRITE_PARAM = 1, 2, 3, 4, 6, 17, 18
P_MODE = 0x7005
# firmware scaling (pos_max, vel_max, torque_max, kp_max, kd_max) — the motors' real ranges
SPEC = {'rs-02': (4 * np.pi, 44.0, 17.0, 500.0, 5.0), 'rs-03': (4 * np.pi, 50.0, 60.0, 5000.0, 100.0),
        'rs-04': (4 * np.pi, 15.0, 120.0, 5000.0, 100.0)}
K_P, K_R = 41.14 / 32.249, 31.398 / 32.249          # ankle linkage (deploy/.../ankle_linkage.hpp)
MASTER = ['L_hip_pitch', 'R_hip_pitch', 'L_hip_roll', 'R_hip_roll', 'L_hip_yaw', 'R_hip_yaw',
          'L_knee', 'R_knee', 'L_foot_pitch', 'R_foot_pitch', 'L_foot_roll', 'R_foot_roll']
DEFAULT = {'L_hip_pitch': -0.08, 'R_hip_pitch': 0.08, 'L_hip_roll': 0.1, 'R_hip_roll': -0.1, 'L_hip_yaw': 0.0,
           'R_hip_yaw': 0.0, 'L_knee': 0.25, 'R_knee': 0.25, 'L_foot_pitch': -0.17, 'R_foot_pitch': -0.17,
           'L_foot_roll': 0.1, 'R_foot_roll': -0.1}
ARMATURE = {'hip_pitch': 0.025, 'hip_roll': 0.01, 'hip_yaw': 0.01, 'knee': 0.3, 'foot_pitch': 0.1, 'foot_roll': 0.1}
FRICTION = 0.1
SUF = {'hip_pitch': '04', 'hip_roll': '03', 'hip_yaw': '03', 'knee': '04', 'foot_pitch': '02', 'foot_roll': '02'}
mj_name = lambda j: f"{'left' if j[0] == 'L' else 'right'}_{j[2:]}_{SUF[j[2:]]}"


def u16(x, xmax):                                    # driver/firmware encoding ((x/max)+1)*0x7FFF
    return int(np.clip((x / xmax + 1.0) * 0x7FFF, 0, 65535))


def from_u16(u, xmax):
    return (u / 0x7FFF - 1.0) * xmax


class Motor:
    def __init__(self, name, bus, mid, model, direction, offset):
        self.name, self.bus, self.id, self.model = name, bus, mid, model
        self.dir, self.off = direction, offset
        self.pmax, self.vmax, self.tmax, self.kpmax, self.kdmax = SPEC[model]
        self.enabled, self.mode, self.fault = False, 0, 0
        self.cmd = None                               # (p_raw, v_raw, kp, kd, t_raw)
        self.n_cmd = 0
        self.ankle = 'foot' in name
        self.upper = name.endswith('_top')
        self.side = name[0]


class Emulator:
    def __init__(self, a):
        self.a = a
        self.dir = a.dir
        os.makedirs(self.dir, exist_ok=True)
        # ---- plant
        self.m = mujoco.MjModel.from_xml_path(a.mjcf)
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.jq = {}; self.jv = {}; self.act = {}
        for j in MASTER:
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, mj_name(j))
            self.jq[j] = m.jnt_qposadr[jid]; self.jv[j] = m.jnt_dofadr[jid]
            self.act[j] = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, mj_name(j))
            m.dof_armature[self.jv[j]] = ARMATURE[j[2:]]
        m.actuator_ctrllimited[:] = 0                  # torque limits are the motors', applied below
        sid = lambda n: m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, n)]
        self.s_quat, self.s_gyro = sid('orientation'), sid('angular-velocity')
        m.sensor_noise[:] = 0.0                        # noise is added explicitly (deterministic per seed)
        self.root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'root')
        self.rng = np.random.default_rng(a.seed)
        mujoco.mj_resetData(m, self.d)
        for j in MASTER:
            self.d.qpos[self.jq[j]] = DEFAULT[j]
        self.d.qpos[2] = a.base_z
        mujoco.mj_forward(m, self.d)
        self.p0 = self.d.qpos[:3].copy()
        self.support = 1.0; self.support_target = 1.0; self.support_rate = 0.0
        self.push = None
        # ---- motors from the robot's own config
        rob = yaml.safe_load(open(a.robot_config)); cal = yaml.safe_load(open(a.calibration)) if a.calibration else {}
        self.motors = {}; self.by_bus = {}
        for bus_key, b in rob.items():
            if not isinstance(b, dict) or 'motors' not in b:
                continue
            iface = b.get('interface', bus_key)
            for name, mc in b['motors'].items():
                model = 'rs-%02d' % int(str(mc['type']).lower().replace('rs', '').replace('-', ''))
                c = (cal or {}).get(name, {})
                mo = Motor(name, iface, int(mc['id']), model, int(c.get('direction', 1)), float(c.get('offset', 0.0)))
                self.motors[name] = mo; self.by_bus.setdefault(iface, {})[mo.id] = mo
        # fault injection knobs (scenario-controlled)
        self.faults = {}
        # ---- CAN servers
        self.srv = {}; self.conns = {}
        for bus in self.by_bus:
            p = os.path.join(self.dir, bus)
            if os.path.exists(p):
                os.unlink(p)
            s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET); s.bind(p); s.listen(8); s.setblocking(False)
            self.srv[bus] = s; self.conns[bus] = []
        # ---- IMU pty
        self.imu_m, self.imu_s = pty.openpty()
        tty.setraw(self.imu_s)
        os.set_blocking(self.imu_m, False)
        link = os.path.join(self.dir, 'ttyIMU')
        if os.path.lexists(link):
            os.unlink(link)
        os.symlink(os.ttyname(self.imu_s), link)
        # ---- control socket
        cp = os.path.join(self.dir, 'ctl')
        if os.path.exists(cp):
            os.unlink(cp)
        self.ctl = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM); self.ctl.bind(cp); self.ctl.setblocking(False)
        # ---- log
        self.log = {k: [] for k in ('t', 'base', 'quat', 'q', 'qd', 'tau', 'support', 'enabled', 'ncmd', 'kp_raw')}
        self.rx_frames = 0; self.tx_frames = 0; self.lag_max = 0.0

    # ------------------------------------------------------------ kinematics: joints <-> calibrated motor coords
    def motor_state(self):
        q = {j: self.d.qpos[self.jq[j]] for j in MASTER}; qd = {j: self.d.qvel[self.jv[j]] for j in MASTER}
        out = {}
        for n, mo in self.motors.items():
            if mo.ankle:
                s = 1 if mo.side == 'R' else -1
                p, r = q[f'{mo.side}_foot_pitch'], q[f'{mo.side}_foot_roll']
                pd, rd = qd[f'{mo.side}_foot_pitch'], qd[f'{mo.side}_foot_roll']
                sg = s if mo.upper else -s
                cal, cald = sg * K_P * p - K_R * r, sg * K_P * pd - K_R * rd
            else:
                cal, cald = q[n], qd[n]
            raw = mo.off + cal if mo.dir == 1 else mo.off - cal
            out[n] = (raw, mo.dir * cald)
        return out

    def joint_torques(self, ms):
        tq = {j: 0.0 for j in MASTER}; traw = {}
        if self.a.ankle_joint_space:
            tq.update(self.ankle_joint_space_torques())
        for n, mo in self.motors.items():
            if mo.ankle and self.a.ankle_joint_space:
                traw[n] = 0.0
                continue
            t = 0.0
            if mo.enabled and mo.mode == 0 and mo.cmd is not None:
                p_des, v_des, kp, kd, t_ff = mo.cmd
                p, v = ms[n]
                t = kp * (p_des - p) + kd * (v_des - v) + t_ff
                t = float(np.clip(t, -mo.tmax, mo.tmax))
            traw[n] = t
            tc = mo.dir * t                                  # calibrated-frame motor torque
            if mo.ankle:
                s = 1 if mo.side == 'R' else -1; sg = s if mo.upper else -s
                tq[f'{mo.side}_foot_pitch'] += sg * K_P * tc
                tq[f'{mo.side}_foot_roll'] += -K_R * tc
            else:
                tq[n] += tc
        return tq, traw

    def ankle_joint_space_torques(self):
        """Counterfactual plant (--ankle-joint-space): recover the joint-space ankle targets from the two motor commands
        and apply PD in joint space with the commanded gains, i.e. what training assumed. Not the real hardware."""
        out = {}
        for side in 'LR':
            A = self.motors[f'{side}_foot_top']; B = self.motors[f'{side}_foot_bottom']
            if not (A.enabled and B.enabled and A.cmd and B.cmd):
                continue
            cal = lambda mo: (mo.cmd[0] - mo.off) if mo.dir == 1 else (mo.off - mo.cmd[0])
            a, b = cal(A), cal(B); s = 1 if side == 'R' else -1
            p_des = s * (a - b) / (2 * K_P); r_des = -(a + b) / (2 * K_R)
            kp, kd = 0.5 * (A.cmd[2] + B.cmd[2]), 0.5 * (A.cmd[3] + B.cmd[3])
            for j, des in ((f'{side}_foot_pitch', p_des), (f'{side}_foot_roll', r_des)):
                q, qd = self.d.qpos[self.jq[j]], self.d.qvel[self.jv[j]]
                out[j] = float(np.clip(kp * (des - q) - kd * qd, -30, 30))
        return out

    # ------------------------------------------------------------ CAN
    def status_frame(self, mo, ms):
        p, v = ms[mo.name]
        t = self._last_traw.get(mo.name, 0.0)
        mode_status = 2 if mo.enabled else 0
        extra = (mode_status << 14) | ((mo.fault & 0x3F) << 8) | mo.id
        cid = EFF | (OP_STATUS << 24) | (extra << 8) | HOST_ID
        data = struct.pack('>HHHH', u16(p, mo.pmax), u16(v, mo.vmax), u16(t, mo.tmax), 300)
        return FRAME.pack(cid, 8, data)

    def send(self, bus, frame):
        if self.faults.get('can_drop', 0) and self.rng.random() < self.faults['can_drop']:
            return
        for c in list(self.conns[bus]):
            try:
                c.send(frame); self.tx_frames += 1
            except (BlockingIOError, InterruptedError):
                pass
            except OSError:
                self._drop(bus, c)

    def _drop(self, bus, c):
        if c in self.conns[bus]:
            self.conns[bus].remove(c)

    def handle(self, bus, raw, ms):
        cid, dlc, data = FRAME.unpack(raw[:16])
        if not cid & EFF:
            return
        cid &= 0x1FFFFFFF
        ctype, extra, dev = (cid >> 24) & 0x1F, (cid >> 8) & 0xFFFF, cid & 0xFF
        mo = self.by_bus[bus].get(dev)
        if mo is None:
            return
        if self.faults.get('can_drop', 0) and self.rng.random() < self.faults['can_drop']:
            return
        if ctype == ENABLE:
            mo.enabled = True
        elif ctype == DISABLE:
            mo.enabled = False; mo.cmd = None
        elif ctype == WRITE_PARAM:
            if (data[0] | data[1] << 8) == P_MODE:
                mo.mode = data[4]
        elif ctype == READ_PARAM:
            pid = data[0] | data[1] << 8
            val = bytes([data[0], data[1], 0, 0, mo.mode & 0xFF, 0, 0, 0])
            self.send(bus, FRAME.pack(EFF | (READ_PARAM << 24) | (mo.id << 8) | HOST_ID, 8, val))
            return
        elif ctype == OP_CTRL:
            pu, vu, kpu, kdu = struct.unpack('>HHHH', data)
            mo.cmd = (from_u16(pu, mo.pmax), from_u16(vu, mo.vmax), kpu / 0xFFFF * mo.kpmax, kdu / 0xFFFF * mo.kdmax,
                      from_u16(extra, mo.tmax))
            mo.n_cmd += 1
        self.send(bus, self.status_frame(mo, ms))

    def pump_can(self, ms):
        for bus, s in self.srv.items():
            while True:
                try:
                    c, _ = s.accept(); c.setblocking(False); self.conns[bus].append(c)
                except (BlockingIOError, InterruptedError):
                    break
            for c in list(self.conns[bus]):
                while True:
                    try:
                        raw = c.recv(64)
                    except (BlockingIOError, InterruptedError):
                        break
                    except OSError:
                        self._drop(bus, c); break
                    if not raw:
                        self._drop(bus, c); break
                    self.rx_frames += 1
                    self.handle(bus, raw, ms)

    # ------------------------------------------------------------ IMU (WIT protocol)
    def imu_packets(self):
        q = self.d.sensordata[self.s_quat:self.s_quat + 4].copy()       # w x y z, world <- imu
        w = self.d.sensordata[self.s_gyro:self.s_gyro + 3].copy()
        R = np.zeros(9); mujoco.mju_quat2Mat(R, q); R = R.reshape(3, 3)
        acc = R.T @ np.array([0, 0, 9.81])
        w = w + self.rng.normal(0, self.a.gyro_noise, 3)
        tf = self.faults.get('imu_rot')                  # mounting error / sign convention fault: 3x3 matrix
        if tf is not None:
            T = np.array(tf); w = T @ w; acc = T @ acc
            Rq = R @ T.T; qq = np.zeros(4); mujoco.mju_mat2Quat(qq, Rq.flatten()); q = qq
        if self.faults.get('imu_freeze_until', 0) > self.d.time:
            return b''
        pk = lambda typ, vals: (lambda body: body + bytes([sum(body) & 0xFF]))(bytes([0x55, typ]) + struct.pack('<4h', *vals))
        i16 = lambda x: int(np.clip(round(x), -32768, 32767))
        a = [i16(x / (16 * 9.81) * 32768) for x in acc] + [0]
        g = [i16(np.degrees(x) / 2000 * 32768) for x in w] + [0]
        qq = [i16(x * 32768) for x in (q[0], q[1], q[2], q[3])]
        return pk(0x51, a) + pk(0x52, g) + pk(0x59, qq)

    def pump_imu(self):
        try:
            while os.read(self.imu_m, 4096):                # discard config bytes the driver writes
                pass
        except (BlockingIOError, OSError):
            pass
        b = self.imu_packets()
        if b:
            try:
                os.write(self.imu_m, b)
            except (BlockingIOError, OSError):
                pass

    # ------------------------------------------------------------ scenario control
    def pump_ctl(self):
        while True:
            try:
                msg, addr = self.ctl.recvfrom(65536)
            except (BlockingIOError, InterruptedError):
                return
            c = json.loads(msg)
            reply = {'ok': True}
            if c.get('cmd') == 'support':
                self.support_target = float(c['level']); self.support_rate = 1.0 / max(float(c.get('fade_s', 0.0)), 1e-3)
            elif c.get('cmd') == 'push':
                self.push = (np.array(c['force']), self.d.time + float(c.get('dur', 0.2)))
            elif c.get('cmd') == 'fault':
                self.faults.update(c['faults'])
                if 'imu_freeze_s' in c['faults']:
                    self.faults['imu_freeze_until'] = self.d.time + c['faults']['imu_freeze_s']
            elif c.get('cmd') == 'state':
                reply = self.truth()
            elif c.get('cmd') == 'save':
                self.save(c['path']); reply = {'saved': c['path']}
            if addr:
                try:
                    self.ctl.sendto(json.dumps(reply).encode(), addr)
                except OSError:
                    pass

    def truth(self):
        d = self.d
        z = float(d.qpos[2]); qw, qx, qy, qz = d.qpos[3:7]
        up = 1 - 2 * (qx * qx + qy * qy)
        return dict(t=float(d.time), z=z, x=float(d.qpos[0]), y=float(d.qpos[1]), tilt_deg=float(np.degrees(np.arccos(np.clip(up, -1, 1)))),
                    support=self.support, enabled=sum(m.enabled for m in self.motors.values()),
                    n_cmd={n: m.n_cmd for n, m in self.motors.items()}, rtf=self.rtf, lag_max=self.lag_max,
                    kp_exec={n: (m.cmd[2] if (m.cmd and m.enabled) else 0.0) for n, m in self.motors.items()},
                    rx=self.rx_frames, tx=self.tx_frames)

    def save(self, path):
        np.savez_compressed(path, **{k: np.array(v) for k, v in self.log.items()}, master=np.array(MASTER),
                            motors=np.array(list(self.motors)))

    # ------------------------------------------------------------ main loop
    def run(self):
        m, d = self.m, self.d
        dt = m.opt.timestep
        self.rtf = 1.0; self._last_traw = {}
        t_wall0 = time.perf_counter(); n = 0; imu_every = int(round(1 / (200 * dt))); log_every = int(round(1 / (100 * dt)))
        stop = [False]
        signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
        signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
        print(f'emulator ready: buses {list(self.srv)} imu {os.ttyname(self.imu_s)}', flush=True)
        while not stop[0]:
            ms = self.motor_state()
            self.pump_can(ms)
            self.pump_ctl()
            tq, traw = self.joint_torques(ms); self._last_traw = traw
            for j in MASTER:
                qd = d.qvel[self.jv[j]]
                d.ctrl[self.act[j]] = tq[j] - FRICTION * np.sign(qd)
            # support ("hand at the pelvis") and pushes
            if self.support != self.support_target:
                step = self.support_rate * dt
                self.support = min(self.support_target, self.support + step) if self.support < self.support_target else max(self.support_target, self.support - step)
            d.xfrc_applied[self.root] = 0
            if self.support > 0:
                pe = self.p0 - d.qpos[:3]; ve = -d.qvel[:3]
                f = self.support * (np.array([3000, 3000, 6000]) * pe + np.array([300, 300, 600]) * ve)
                qw, qx, qy, qz = d.qpos[3:7]
                rot_err = -2 * np.array([qx, qy, qz]) * np.sign(qw)
                tau = self.support * (400 * rot_err - 40 * d.qvel[3:6])
                d.xfrc_applied[self.root, :3] = f; d.xfrc_applied[self.root, 3:] = tau
            if self.push is not None and d.time < self.push[1]:
                d.xfrc_applied[self.root, :3] += self.push[0]
            mujoco.mj_step(m, d); n += 1
            if n % imu_every == 0:
                self.pump_imu()
            if n % log_every == 0:
                L = self.log
                L['t'].append(d.time); L['base'].append(d.qpos[:3].copy()); L['quat'].append(d.qpos[3:7].copy())
                L['q'].append([d.qpos[self.jq[j]] for j in MASTER]); L['qd'].append([d.qvel[self.jv[j]] for j in MASTER])
                L['tau'].append([tq[j] for j in MASTER]); L['support'].append(self.support)
                L['enabled'].append(sum(mo.enabled for mo in self.motors.values()))
                L['ncmd'].append(sum(mo.n_cmd for mo in self.motors.values()))
                L['kp_raw'].append([mo.cmd[2] if mo.cmd else 0.0 for mo in self.motors.values()])
            # real-time pacing
            ahead = t_wall0 + d.time - time.perf_counter()
            if ahead > 0.001:
                time.sleep(ahead - 0.0005)
            elif ahead < 0:
                self.lag_max = max(self.lag_max, -ahead)
            if n % 2000 == 0:
                self.rtf = d.time / (time.perf_counter() - t_wall0)
        if self.a.save:
            self.save(self.a.save)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__)); repo = os.path.dirname(here)
    ap.add_argument('--dir', default=os.environ.get('SIMCAN_DIR', '/tmp/simcan'))
    ap.add_argument('--mjcf', default=os.path.join(repo, 'mjcf/sim2sim/robot_light.mjcf'))
    ap.add_argument('--robot-config', default=os.path.join(repo, 'deploy/biped_ws/src/biped_bringup/config/robot.yaml'))
    ap.add_argument('--calibration', default=os.path.join(repo, 'deploy/biped_ws/calibration.yaml'))
    ap.add_argument('--base-z', type=float, default=0.83)
    ap.add_argument('--gyro-noise', type=float, default=0.005)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--save', default='')
    ap.add_argument('--ankle-joint-space', action='store_true', help='counterfactual: ankle PD in joint space (as trained)')
    Emulator(ap.parse_args()).run()
