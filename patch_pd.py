with open('sim2sim/play_mujoco.py', 'r') as f:
    text = f.read()

old_loop = """            # PD control
            jp = data.qpos[qp_idx]
            jv = data.qvel[qv_idx]
            torques = KP_MJ * (targets_mj - jp) + KD_MJ * (0.0 - jv)
            torques = np.clip(torques, -EFFORT_MJ, EFFORT_MJ)

            # Step physics
            for _ in range(SUBSTEPS):
                data.ctrl[:] = torques
                mujoco.mj_step(model, data)"""

new_loop = """            # Step physics at 2000Hz, computing PD torque every step
            for _ in range(SUBSTEPS):
                jp = data.qpos[qp_idx]
                jv = data.qvel[qv_idx]
                torques = KP_MJ * (targets_mj - jp) + KD_MJ * (0.0 - jv)
                torques = np.clip(torques, -EFFORT_MJ, EFFORT_MJ)
                data.ctrl[:] = torques
                mujoco.mj_step(model, data)"""

text = text.replace(old_loop, new_loop)

# Also update the hardcoded gains in play_mujoco.py while we are here!
old_gains = """KP_ISAAC = {"L_hip_pitch": 15, "R_hip_pitch": 15, "L_hip_roll": 10, "R_hip_roll": 10,
            "L_hip_yaw": 10, "R_hip_yaw": 10, "L_knee": 15, "R_knee": 15,
            "L_foot_pitch": 4, "R_foot_pitch": 4, "L_foot_roll": 4, "R_foot_roll": 4}
KD_ISAAC = {"L_hip_pitch": 3, "R_hip_pitch": 3, "L_hip_roll": 3, "R_hip_roll": 3,
            "L_hip_yaw": 3, "R_hip_yaw": 3, "L_knee": 3, "R_knee": 3,
            "L_foot_pitch": 0.2, "R_foot_pitch": 0.2, "L_foot_roll": 0.2, "R_foot_roll": 0.2}"""

new_gains = """KP_ISAAC = {"L_hip_pitch": 180, "R_hip_pitch": 180, "L_hip_roll": 180, "R_hip_roll": 180,
            "L_hip_yaw": 180, "R_hip_yaw": 180, "L_knee": 180, "R_knee": 180,
            "L_foot_pitch": 30, "R_foot_pitch": 30, "L_foot_roll": 30, "R_foot_roll": 30}
KD_ISAAC = {"L_hip_pitch": 6.5, "R_hip_pitch": 6.5, "L_hip_roll": 6.5, "R_hip_roll": 6.5,
            "L_hip_yaw": 3.0, "R_hip_yaw": 3.0, "L_knee": 6.5, "R_knee": 6.5,
            "L_foot_pitch": 1.0, "R_foot_pitch": 1.0, "L_foot_roll": 1.0, "R_foot_roll": 1.0}"""

text = text.replace(old_gains, new_gains)

with open('sim2sim/play_mujoco.py', 'w') as f:
    f.write(text)
print("patched")
