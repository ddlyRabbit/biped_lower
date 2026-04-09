with open('deploy/biped_ws/src/biped_control/biped_control/obs_builder.py', 'r') as f:
    content = f.read()

old_gains = """DEFAULT_GAINS = {
    "L_hip_pitch": (180.0, 3.0), "R_hip_pitch": (180.0, 3.0),
    "L_hip_roll":  (180.0, 3.0), "R_hip_roll":  (180.0, 3.0),
    "L_hip_yaw":   (180.0, 3.0), "R_hip_yaw":   (180.0, 3.0),
    "L_knee":      (180.0, 3.0), "R_knee":      (180.0, 3.0),
    "L_foot_pitch": (30.0, 1.0), "R_foot_pitch": (30.0, 1.0),
    "L_foot_roll":  (30.0, 1.0), "R_foot_roll":  (30.0, 1.0),
}"""

new_gains = """DEFAULT_GAINS = {
    "L_hip_pitch": (180.0, 6.5), "R_hip_pitch": (180.0, 6.5),
    "L_hip_roll":  (180.0, 6.5), "R_hip_roll":  (180.0, 6.5),
    "L_hip_yaw":   (180.0, 3.0), "R_hip_yaw":   (180.0, 3.0),
    "L_knee":      (180.0, 6.5), "R_knee":      (180.0, 6.5),
    "L_foot_pitch": (30.0, 1.0), "R_foot_pitch": (30.0, 1.0),
    "L_foot_roll":  (30.0, 1.0), "R_foot_roll":  (30.0, 1.0),
}"""

content = content.replace(old_gains, new_gains)

with open('deploy/biped_ws/src/biped_control/biped_control/obs_builder.py', 'w') as f:
    f.write(content)
