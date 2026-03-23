"""Motor SysID — combined video + CSV recording.

Suspended robot (fixed base, gravity ON), same actuator model as training.
Records video via gymnasium RecordVideo + CSV data for each joint test.

Usage:
    /isaac-sim/python.sh motor_sysid_combined.py --all --headless
    /isaac-sim/python.sh motor_sysid_combined.py --joint R_hip_pitch --headless
"""
import argparse, csv, math, sys, os
sys.path.insert(0, '/workspace/biped_locomotion')

from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
parser.add_argument('--joint', type=str, help='Single joint to test')
parser.add_argument('--all', action='store_true', help='Test all 3 representative joints')
parser.add_argument('--output', type=str, default='/results/motor_sysid')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch
import yaml
from biped_env_cfg import BipedFlatEnvCfg
from isaaclab.envs import ManagerBasedRLEnv
from gymnasium.wrappers import RecordVideo

# Joint index in the 12-action vector (Isaac joint order)
FRIENDLY_TO_IDX = {
    'R_hip_pitch': 5, 'L_hip_pitch': 4,
    'R_hip_roll': 1, 'L_hip_roll': 0,
    'R_hip_yaw': 3, 'L_hip_yaw': 2,
    'R_knee': 7, 'L_knee': 6,
    'R_foot_pitch': 9, 'L_foot_pitch': 8,
    'R_foot_roll': 11, 'L_foot_roll': 10,
}

MOTOR_TYPES = {
    'R_hip_pitch': 'RS04', 'R_hip_roll': 'RS03', 'R_hip_yaw': 'RS03',
    'R_knee': 'RS04', 'R_foot_pitch': 'RS02', 'R_foot_roll': 'RS02',
}

# One representative joint per motor type
DEFAULT_TEST_JOINTS = ['R_hip_pitch', 'R_hip_roll', 'R_foot_pitch']

# Test parameters per joint: (step_target, sine_amp, sine_freqs)
TEST_PARAMS = {
    'R_hip_pitch':  {'step': 0.4, 'amp': 0.3, 'freqs': [0.5, 1.0, 2.0, 5.0, 10.0]},
    'R_hip_roll':   {'step': -0.3, 'amp': 0.3, 'freqs': [0.5, 1.0, 2.0, 5.0, 10.0]},
    'R_hip_yaw':    {'step': 0.3, 'amp': 0.3, 'freqs': [0.5, 1.0, 2.0, 5.0, 10.0]},
    'R_knee':       {'step': 0.8, 'amp': 0.3, 'freqs': [0.5, 1.0, 2.0, 5.0, 10.0]},
    'R_foot_pitch': {'step': 0.1, 'amp': 0.15, 'freqs': [0.5, 1.0, 2.0, 5.0, 10.0]},
    'R_foot_roll':  {'step': 0.1, 'amp': 0.1, 'freqs': [0.5, 1.0, 2.0, 5.0, 10.0]},
}


def save_csv(data, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['time', 'cmd_pos', 'pos', 'vel', 'torque'])
        w.writeheader()
        w.writerows(data)
    print("  Saved %s (%d rows)" % (path, len(data)))


def create_env():
    env_cfg = BipedFlatEnvCfg()
    env_cfg.scene.num_envs = 1
    # Fixed base, 500mm higher
    env_cfg.scene.robot.spawn.articulation_props.fix_root_link = True
    env_cfg.scene.robot.init_state.pos = (0.0, 0.0, 1.30)
    # Long episode, no base contact termination
    env_cfg.episode_length_s = 1000.0
    env_cfg.terminations.base_contact = None
    # Disable all randomization events — we want deterministic actuator response
    env_cfg.events.add_all_joint_default_pos = None
    env_cfg.events.scale_all_joint_armature = None
    env_cfg.events.scale_all_joint_friction = None
    env_cfg.events.scale_all_actuator_torque_constant = None
    env_cfg.events.reset_robot_joints = None
    env_cfg.events.reset_base = None
    env_cfg.events.push_robot = None
    return ManagerBasedRLEnv(cfg=env_cfg, render_mode="rgb_array")


class SysIDRunner:
    def __init__(self, env, output_dir):
        self.env = env
        self.output_dir = output_dir
        self.n_actions = env.action_manager.total_action_dim
        self.dt = env.step_dt
        self.action_scale = 0.25  # must match training config

        # Get defaults from robot
        robot = env.scene['robot']
        self.defaults = robot.data.default_joint_pos[0].clone()
        print("Defaults:", self.defaults.cpu().numpy())

    def _abs_to_action(self, desired_pos, joint_idx):
        """Convert absolute joint target (rad) to env action value."""
        default = self.defaults[joint_idx].item()
        return (desired_pos - default) / self.action_scale

    def _make_default_actions(self):
        """Actions that hold all joints at their defaults."""
        actions = torch.zeros(1, self.n_actions, device=self.env.device)
        for name, idx in FRIENDLY_TO_IDX.items():
            actions[0, idx] = self._abs_to_action(self.defaults[idx].item(), idx)
        return actions

    def _step(self, video_env, actions, joint_idx, cmd_pos, t):
        """Step env, record data point."""
        obs, _, _, _, _ = video_env.step(actions)
        robot = self.env.scene['robot']
        pos = robot.data.joint_pos[0, joint_idx].item()
        vel = robot.data.joint_vel[0, joint_idx].item()
        torque = 0.0
        if hasattr(robot.data, 'applied_torque'):
            torque = robot.data.applied_torque[0, joint_idx].item()
        return {
            'time': round(t, 5),
            'cmd_pos': round(cmd_pos, 6),
            'pos': round(pos, 6),
            'vel': round(vel, 6),
            'torque': round(torque, 6),
        }

    def run_joint(self, video_env, joint_name):
        jidx = FRIENDLY_TO_IDX[joint_name]
        mtype = MOTOR_TYPES.get(joint_name, 'UNK')
        params = TEST_PARAMS.get(joint_name, {'step': 0.3, 'amp': 0.2, 'freqs': [1.0]})
        out_dir = os.path.join(self.output_dir, '%s_%s' % (joint_name, mtype))
        default_pos = self.defaults[jidx].item()

        print("=" * 60)
        print("SysID: %s (%s) idx=%d default=%.3f" % (joint_name, mtype, jidx, default_pos))
        print("  step_target=%.3f sine_amp=%.3f" % (params['step'], params['amp']))
        print("=" * 60)

        # --- Step Response ---
        self.env.reset()
        data = []
        t = 0.0
        n_hold = int(0.5 / self.dt)
        n_step = int(2.0 / self.dt)
        n_back = int(1.0 / self.dt)

        # Hold at default
        for _ in range(n_hold):
            actions = self._make_default_actions()
            data.append(self._step(video_env, actions, jidx, default_pos, t))
            t += self.dt

        # Step to target (absolute)
        step_target = params['step']
        for _ in range(n_step):
            actions = self._make_default_actions()
            actions[0, jidx] = self._abs_to_action(step_target, jidx)
            data.append(self._step(video_env, actions, jidx, step_target, t))
            t += self.dt

        # Return to default
        for _ in range(n_back):
            actions = self._make_default_actions()
            data.append(self._step(video_env, actions, jidx, default_pos, t))
            t += self.dt

        save_csv(data, os.path.join(out_dir, 'step_response.csv'))

        # --- Sine Sweeps ---
        for freq in params['freqs']:
            self.env.reset()
            data = []
            t = 0.0
            amp = params['amp']
            dur = 5.0 / freq  # 5 cycles

            # Hold
            for _ in range(n_hold):
                actions = self._make_default_actions()
                data.append(self._step(video_env, actions, jidx, default_pos, t))
                t += self.dt

            # Sine around default
            t0 = t
            n_sine = int(dur / self.dt)
            for _ in range(n_sine):
                desired = default_pos + amp * math.sin(2 * math.pi * freq * (t - t0))
                actions = self._make_default_actions()
                actions[0, jidx] = self._abs_to_action(desired, jidx)
                data.append(self._step(video_env, actions, jidx, desired, t))
                t += self.dt

            # Hold
            for _ in range(n_hold):
                actions = self._make_default_actions()
                data.append(self._step(video_env, actions, jidx, default_pos, t))
                t += self.dt

            save_csv(data, os.path.join(out_dir, 'sine_%.1fhz.csv' % freq))

        # Metadata
        meta = {
            'joint': joint_name,
            'motor_type': mtype,
            'default_pos': float(default_pos),
            'step_target': float(params['step']),
            'sine_amp': float(params['amp']),
            'sine_freqs': params['freqs'],
            'action_scale': self.action_scale,
            'env_dt': float(self.dt),
            'decimation': int(self.env.cfg.decimation),
            'suspended': True,
            'gravity': True,
        }
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, 'metadata.yaml'), 'w') as f:
            yaml.dump(meta, f)
        print("  Complete: %s" % out_dir)


# --- Main ---
env = create_env()
runner = SysIDRunner(env, args.output)

# Determine test joints
if args.all:
    test_joints = DEFAULT_TEST_JOINTS
elif args.joint:
    test_joints = [args.joint]
else:
    print('Specify --joint or --all')
    simulation_app.close()
    sys.exit(1)

# Total steps for video length estimate
total_steps = 0
for j in test_joints:
    params = TEST_PARAMS.get(j, {'step': 0.3, 'amp': 0.2, 'freqs': [1.0]})
    n_hold = int(0.5 / env.step_dt)
    n_step = int(2.0 / env.step_dt)
    n_back = int(1.0 / env.step_dt)
    total_steps += n_hold + n_step + n_back  # step response
    for freq in params['freqs']:
        total_steps += n_hold + int(5.0 / freq / env.step_dt) + n_hold  # sine

print("Total steps: %d (%.1f sec at %.0f Hz)" % (total_steps, total_steps * env.step_dt, 1.0 / env.step_dt))

# Wrap with video
video_dir = os.path.join(args.output, 'video')
os.makedirs(video_dir, exist_ok=True)
video_env = RecordVideo(
    env,
    video_folder=video_dir,
    episode_trigger=lambda x: True,
    video_length=total_steps + 100,
    name_prefix="sysid",
)
obs, _ = video_env.reset()

# Run tests
for j in test_joints:
    runner.run_joint(video_env, j)

print("All done! Video + CSVs in %s" % args.output)
video_env.close()
simulation_app.close()
