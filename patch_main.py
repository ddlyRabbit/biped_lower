import re

with open('deploy/scripts/generate_zmp_trajectory.py', 'r') as f:
    text = f.read()

# Split before the # Format to CSV
parts = text.split('    # Format to CSV')

new_code = """
    # Ramp down
    ramp_ik_frames = int(1.0 / dt)
    ramp_joint_frames = int(1.0 / dt)

    end_com_x = com_x[-1]
    end_com_y = com_y[-1]
    end_l_foot = plan.left_foot[-1].copy()
    end_r_foot = plan.right_foot[-1].copy()
    stand_l_foot = np.array([0.0, end_l_foot[1], 0.0])
    stand_r_foot = np.array([0.0, end_r_foot[1], 0.0])
    center_x = (end_l_foot[0] + end_r_foot[0]) / 2.0

    for i in range(1, ramp_ik_frames + 1):
        alpha = i / ramp_ik_frames
        alpha = 0.5 * (1 - np.cos(np.pi * alpha))
        ramp_com_x = end_com_x + alpha * (center_x - end_com_x)
        ramp_com_y = end_com_y + alpha * (0.0 - end_com_y)
        ramp_l = end_l_foot + alpha * (stand_l_foot - end_l_foot)
        ramp_r = end_r_foot + alpha * (stand_r_foot - end_r_foot)
        
        ramp_base_x = ramp_com_x - com_offset[0]
        ramp_base_y = ramp_com_y - com_offset[1]
        
        joints = ik.solve(ramp_base_x, ramp_base_y, ramp_l, ramp_r)
        trajectory.append(joints)
        com_offset = ik.compute_com(joints)

    last_ik_joints = trajectory[-1]
    default_joints = {
        "R_hip_pitch": 0.08, "R_hip_roll": 0.0, "R_hip_yaw": 0.0,
        "R_knee": 0.25, "R_foot_pitch": -0.17, "R_foot_roll": 0.0,
        "L_hip_pitch": -0.08, "L_hip_roll": 0.0, "L_hip_yaw": 0.0,
        "L_knee": 0.25, "L_foot_pitch": -0.17, "L_foot_roll": 0.0,
    }

    for i in range(1, ramp_joint_frames + 1):
        alpha = i / ramp_joint_frames
        alpha = 0.5 * (1 - np.cos(np.pi * alpha))
        frame = {}
        for name in last_ik_joints:
            frame[name] = last_ik_joints[name] + alpha * (default_joints[name] - last_ik_joints[name])
        trajectory.append(frame)

    # Format to CSV"""

text = parts[0] + new_code + parts[1]

# Also remove the EOF garbage from earlier
text = text.replace("# --- Append Ramp Down ---\n# Actually, I should rewrite the main() function in the script to include it properly.\n", "")

with open('deploy/scripts/generate_zmp_trajectory.py', 'w') as f:
    f.write(text)
