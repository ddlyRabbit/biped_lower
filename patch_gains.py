import os

filepath_obs = 'deploy/biped_ws/src/biped_control/biped_control/obs_builder.py'
with open(filepath_obs, 'r') as f:
    content_obs = f.read()

content_obs = content_obs.replace('    "L_hip_pitch": (180.0, 6.5), "R_hip_pitch": (180.0, 6.5),', '    "L_hip_pitch": (120.0, 6.5), "R_hip_pitch": (120.0, 6.5),')
content_obs = content_obs.replace('    "L_hip_roll":  (180.0, 6.5), "R_hip_roll":  (180.0, 6.5),', '    "L_hip_roll":  (120.0, 6.5), "R_hip_roll":  (120.0, 6.5),')
content_obs = content_obs.replace('    "L_hip_yaw":   (180.0, 3.0), "R_hip_yaw":   (180.0, 3.0),', '    "L_hip_yaw":   (120.0, 3.0), "R_hip_yaw":   (120.0, 3.0),')
content_obs = content_obs.replace('    "L_knee":      (180.0, 6.5), "R_knee":      (180.0, 6.5),', '    "L_knee":      (120.0, 6.5), "R_knee":      (120.0, 6.5),')
content_obs = content_obs.replace('    "L_foot_pitch": (30.0, 1.0), "R_foot_pitch": (30.0, 1.0),', '    "L_foot_pitch": (20.0, 1.0), "R_foot_pitch": (20.0, 1.0),')
content_obs = content_obs.replace('    "L_foot_roll":  (30.0, 1.0), "R_foot_roll":  (30.0, 1.0),', '    "L_foot_roll":  (20.0, 1.0), "R_foot_roll":  (20.0, 1.0),')

with open(filepath_obs, 'w') as f:
    f.write(content_obs)


filepath_play = 'sim2sim/play_mujoco.py'
with open(filepath_play, 'r') as f:
    content_play = f.read()

content_play = content_play.replace('KP_ISAAC = {"L_hip_pitch": 180, "R_hip_pitch": 180, "L_hip_roll": 180, "R_hip_roll": 180,\n            "L_hip_yaw": 180, "R_hip_yaw": 180, "L_knee": 180, "R_knee": 180,\n            "L_foot_pitch": 30, "R_foot_pitch": 30, "L_foot_roll": 30, "R_foot_roll": 30}', 'KP_ISAAC = {"L_hip_pitch": 120, "R_hip_pitch": 120, "L_hip_roll": 120, "R_hip_roll": 120,\n            "L_hip_yaw": 120, "R_hip_yaw": 120, "L_knee": 120, "R_knee": 120,\n            "L_foot_pitch": 20, "R_foot_pitch": 20, "L_foot_roll": 20, "R_foot_roll": 20}')

with open(filepath_play, 'w') as f:
    f.write(content_play)

print('Gains patched.')
