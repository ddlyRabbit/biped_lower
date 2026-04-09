with open('sim2sim/stand_mujoco.py', 'r') as f:
    text = f.read()

old_loop = """            jp = data.qpos[qp_idx]
            jv = data.qvel[qv_idx]
            torques = JOINT_KP * (DEFAULT_JOINT_POS - jp) + JOINT_KD * (0.0 - jv)
            torques = np.clip(torques, -EFFORT_LIMITS, EFFORT_LIMITS)

            for _ in range(SUBSTEPS):
                data.ctrl[:] = torques
                mujoco.mj_step(model, data)"""

new_loop = """            for _ in range(SUBSTEPS):
                jp = data.qpos[qp_idx]
                jv = data.qvel[qv_idx]
                torques = JOINT_KP * (DEFAULT_JOINT_POS - jp) + JOINT_KD * (0.0 - jv)
                torques = np.clip(torques, -EFFORT_LIMITS, EFFORT_LIMITS)
                data.ctrl[:] = torques
                mujoco.mj_step(model, data)"""

text = text.replace(old_loop, new_loop)

with open('sim2sim/stand_mujoco.py', 'w') as f:
    f.write(text)
