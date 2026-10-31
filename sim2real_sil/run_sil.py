#!/usr/bin/env python3
"""Run one software-in-the-loop scenario: the real bringup launch file and nodes against the MuJoCo emulator.

  python3 run_sil.py --ws deploy/biped_ws --model deploy/v188_student_light_4600.onnx --scenario walk \
                     --launch-arg unified:=true --launch-arg control_driver:=biped_control_cpp \
                     --launch-arg imu_type:=im10a --launch-arg imu_filter:=true --out runs/head_walk

Needs: a sourced ROS 2 environment with the workspace built; simcan/libsimcan.so; a Python with mujoco for
the emulator (--emu-python). Writes <out>/summary.json, <out>/events.jsonl, <out>/truth.npz, <out>/launch.log.

Scenario steps are (time_s, action) pairs. The operator procedure mirrors the real robot: held at the pelvis,
START -> STAND, release the support, WALK at 0 then 0.3 m/s, STOP, (optionally WALK again).

MIT License, Copyright (c) 2026 Abhinav Roy
"""
import argparse, json, os, signal, socket, subprocess, sys, time, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

HERE = os.path.dirname(os.path.abspath(__file__))

SCENARIOS = {
    # boot and stay idle: catches start-up faults (watchdogs, inverted tilt checks, motors not free)
    'boot': [(0, ('wait', 8.0))],
    # the August procedure: held, START, stand, release, step in place, walk, stop
    'walk': [(0, ('cmd', 'START')), (6, ('support', 0.0, 2.0)), (10, ('cmd', 'WALK')), (10, ('vel', 0.0)),
             (14, ('vel', 0.3)), (24, ('vel', 0.0)), (26, ('cmd', 'STOP')), (30, ('wait', 0.0))],
    # walk, stop, walk again: catches stale state carried across WALK re-entry
    'reentry': [(0, ('cmd', 'START')), (6, ('support', 0.0, 2.0)), (10, ('cmd', 'WALK')), (10, ('vel', 0.2)),
                (16, ('cmd', 'STOP')), (19, ('cmd', 'WALK')), (19, ('vel', 0.2)), (25, ('cmd', 'STOP')), (28, ('wait', 0.0))],
    # held until the policy is walking in place, then released: compares plants without relying on the STAND controller
    'walk_late_release': [(0, ('cmd', 'START')), (8, ('cmd', 'WALK')), (8, ('vel', 0.0)), (9, ('support', 0.0, 2.0)),
                          (13, ('vel', 0.3)), (23, ('vel', 0.0)), (24.5, ('support', 1.0, 0.3)), (25, ('cmd', 'STOP')),
                          (28, ('wait', 0.0))],
    # walk, stop and walk again with the robot held through STOP (as operators did): isolates WALK re-entry
    'reentry_held': [(0, ('cmd', 'START')), (6, ('support', 0.0, 2.0)), (10, ('cmd', 'WALK')), (10, ('vel', 0.2)),
                     (15.5, ('support', 1.0, 0.3)), (16, ('cmd', 'STOP')), (19, ('cmd', 'WALK')), (19, ('vel', 0.2)),
                     (19.5, ('support', 0.0, 1.0)), (24.5, ('support', 1.0, 0.3)), (25, ('cmd', 'STOP')), (28, ('wait', 0.0))],
    # stand only, held then released: catches stand-ramp and gain-ramp problems
    'stand': [(0, ('cmd', 'START')), (6, ('support', 0.0, 2.0)), (14, ('wait', 0.0))],
}


class Probe(Node):
    def __init__(self, events):
        super().__init__('sil_probe')
        self.events = events; self.state = None; self.n_js = 0; self.safety = None; self.js = {}
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/state_machine', self.on_state, 10)
        self.create_subscription(Bool, '/safety/status', self.on_safety, 10)
        self.create_subscription(String, '/safety/fault', lambda m: self.ev('safety_fault', m.data), 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, be)
        self.cmd_kp = {}
        try:   # commanded gains, to check that the motors execute what the controller asked for
            from biped_msgs.msg import MITCommandArray
            self.create_subscription(MITCommandArray, '/joint_commands',
                                     lambda m: self.cmd_kp.update({c.joint_name: float(c.kp) for c in m.commands}), 10)
        except ImportError:
            pass
        self.pub_cmd = self.create_publisher(String, '/state_command', 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel = None
        self.create_timer(0.1, self.tick)

    def ev(self, kind, data):
        self.events.append(dict(wall=time.time(), kind=kind, data=data))

    def on_state(self, m):
        if m.data != self.state:
            self.ev('state', m.data); self.state = m.data
            self.ev('joint_states_at_' + m.data, self.js)

    def on_safety(self, m):
        if m.data != self.safety:
            self.ev('safety', m.data); self.safety = m.data

    def on_js(self, m):
        self.n_js += 1
        self.js = {n: round(p, 4) for n, p in zip(m.name, m.position)}

    def tick(self):
        if self.vel is not None:
            t = Twist(); t.linear.x = float(self.vel); self.pub_vel.publish(t)

    def command(self, c):
        self.ev('command', c); self.pub_cmd.publish(String(data=c))


def _clean_env(**extra):
    """Environment for the emulator's own Python: drop the ROS/conda variables it must not inherit."""
    drop = ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH')
    env = {k: v for k, v in os.environ.items() if k not in drop and not k.startswith('CONDA')}
    env['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    env.update(extra)
    return env


def ctl(path, msg, reply=False, timeout=1.0):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    me = None
    if reply:
        me = f'{path}.r{os.getpid()}'
        if os.path.exists(me):
            os.unlink(me)
        s.bind(me); s.settimeout(timeout)
    try:
        s.sendto(json.dumps(msg).encode(), path)
        if reply:
            return json.loads(s.recv(1 << 20))
    except OSError:
        return None
    finally:
        s.close()
        if me and os.path.exists(me):
            os.unlink(me)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', default=os.path.dirname(HERE), help='checkout whose deploy code, configs and model are run')
    ap.add_argument('--plant-repo', default=os.path.dirname(HERE), help='checkout whose MJCF is the plant (keep fixed across replays)')
    ap.add_argument('--ws', default='deploy/biped_ws')
    ap.add_argument('--model', default='deploy/v188_student_light_4600.onnx')
    ap.add_argument('--scenario', default='walk', choices=list(SCENARIOS))
    ap.add_argument('--launch-file', default='bringup.launch.py')
    ap.add_argument('--launch-arg', action='append', default=[])
    ap.add_argument('--fault', default='', help='emulator faults as JSON, applied after boot')
    ap.add_argument('--out', required=True)
    ap.add_argument('--emu-python', default=os.environ.get('SIL_EMU_PYTHON', '/usr/local/bin/python3'))
    ap.add_argument('--simcan-dir', default='')
    ap.add_argument('--boot-timeout', type=float, default=40.0)
    ap.add_argument('--emu-arg', action='append', default=[], help='extra emulator flag, e.g. --emu-arg=--ankle-joint-space')
    a = ap.parse_args()
    repo = os.path.abspath(a.repo); ws = os.path.join(repo, a.ws); out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    sdir = a.simcan_dir or f'/tmp/simcan_{os.getpid()}'
    os.makedirs(sdir, exist_ok=True)
    env = dict(os.environ, SIMCAN_DIR=sdir, LD_PRELOAD=os.path.join(HERE, 'simcan', 'libsimcan.so'),
               PATH=os.path.join(HERE, 'bin') + os.pathsep + os.environ['PATH'])

    emu = subprocess.Popen([a.emu_python, os.path.join(HERE, 'emulator.py'), '--dir', sdir,
                            '--mjcf', os.path.join(os.path.abspath(a.plant_repo), 'mjcf/sim2sim/robot_light.mjcf'),
                            '--robot-config', os.path.join(ws, 'src/biped_bringup/config/robot.yaml'),
                            '--calibration', os.path.join(ws, 'calibration.yaml'),
                            '--save', os.path.join(out, 'truth.npz')] + a.emu_arg,
                           stdout=open(os.path.join(out, 'emulator.log'), 'w'), stderr=subprocess.STDOUT,
                           env=_clean_env(SIMCAN_DIR=sdir))
    for _ in range(100):
        if os.path.exists(os.path.join(sdir, 'ctl')) and os.path.exists(os.path.join(sdir, 'ttyIMU')):
            break
        time.sleep(0.1)
    args = [f'onnx_model:={os.path.join(repo, a.model)}', f'calibration_file:={os.path.join(ws, "calibration.yaml")}',
            f'im10a_port:={os.path.join(sdir, "ttyIMU")}', 'record:=false'] + a.launch_arg
    launch = subprocess.Popen(['ros2', 'launch', 'biped_bringup', a.launch_file] + args,
                              stdout=open(os.path.join(out, 'launch.log'), 'w'), stderr=subprocess.STDOUT,
                              env=env, start_new_session=True)
    events = []; truth = []
    rclpy.init()
    probe = Probe(events)
    spin = threading.Thread(target=rclpy.spin, args=(probe,), daemon=True); spin.start()
    t_boot = time.time(); booted = False
    while time.time() - t_boot < a.boot_timeout:
        if probe.state is not None and probe.n_js > 20 and probe.safety is True:   # operator waits for safety OK
            booted = True; break
        if launch.poll() is not None:
            break
        time.sleep(0.2)
    probe.ev('booted', booted)
    if a.fault:
        ctl(os.path.join(sdir, 'ctl'), {'cmd': 'fault', 'faults': json.loads(a.fault)})
        probe.ev('fault', a.fault)
    t0 = time.time(); steps = sorted(SCENARIOS[a.scenario], key=lambda s: s[0]); i = 0
    end = steps[-1][0] + (steps[-1][1][1] if steps[-1][1][0] == 'wait' else 0)
    while booted and time.time() - t0 <= end:
        el = time.time() - t0
        while i < len(steps) and steps[i][0] <= el:
            act = steps[i][1]
            if act[0] == 'cmd':
                probe.command(act[1])
            elif act[0] == 'vel':
                probe.vel = act[1]; probe.ev('vel', act[1])
            elif act[0] == 'support':
                ctl(os.path.join(sdir, 'ctl'), {'cmd': 'support', 'level': act[1], 'fade_s': act[2]}); probe.ev('support', act[1])
            i += 1
        st = ctl(os.path.join(sdir, 'ctl'), {'cmd': 'state'}, reply=True)
        if st:
            st['el'] = el; st['fsm'] = probe.state; st['cmd_kp'] = dict(probe.cmd_kp); truth.append(st)
        if launch.poll() is not None:
            probe.ev('launch_exited', launch.returncode); break
        time.sleep(0.1)
    # shutdown
    try:
        os.killpg(launch.pid, signal.SIGINT); launch.wait(10)
    except Exception:
        try:
            os.killpg(launch.pid, signal.SIGKILL)
        except Exception:
            pass
    emu.send_signal(signal.SIGTERM)
    try:
        emu.wait(20)
    except subprocess.TimeoutExpired:
        emu.kill()
    probe.destroy_node(); rclpy.shutdown()
    # summary
    after_release = [s for s in truth if s['support'] < 0.05]
    fell = any(s['z'] < 0.45 or s['tilt_deg'] > 45 for s in after_release)
    states = [e['data'] for e in events if e['kind'] == 'state']
    walk = [s for s in truth if s.get('fsm') == 'WALK' and s['support'] < 0.05]
    summ = dict(scenario=a.scenario, model=a.model, launch_args=a.launch_arg, fault=a.fault, booted=booted,
                states=states, estop=('ESTOP' in states), fell=fell,
                t_fell=next((round(s['el'], 2) for s in after_release if s['z'] < 0.45 or s['tilt_deg'] > 45), None),
                max_tilt_after_release=round(max((s['tilt_deg'] for s in after_release), default=float('nan')), 1),
                min_z_after_release=round(min((s['z'] for s in after_release), default=float('nan')), 3),
                walk_dx=round(walk[-1]['x'] - walk[0]['x'], 3) if len(walk) > 1 else None,
                walk_s=round(walk[-1]['el'] - walk[0]['el'], 1) if len(walk) > 1 else 0,
                safety_faults=sorted({e['data'] for e in events if e['kind'] == 'safety_fault'}),
                rtf_min=round(min((s['rtf'] for s in truth), default=float('nan')), 3),
                emu_lag_max_s=round(max((s['lag_max'] for s in truth), default=float('nan')), 4),
                motors_enabled_max=max((s['enabled'] for s in truth), default=0))
    json.dump(summ, open(os.path.join(out, 'summary.json'), 'w'), indent=1)
    with open(os.path.join(out, 'events.jsonl'), 'w') as f:
        for e in events:
            f.write(json.dumps(e) + '\n')
    json.dump(truth, open(os.path.join(out, 'truth_10hz.json'), 'w'))
    print(json.dumps(summ))


if __name__ == '__main__':
    main()
