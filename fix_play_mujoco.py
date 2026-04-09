import re

with open('sim2sim/play_mujoco.py', 'r') as f:
    content = f.read()

# Add actuator_idx definition
if 'actuator_idx =' not in content:
    content = content.replace(
        "qv_idx = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in MJ_JOINTS])",
        "qv_idx = np.array([model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in MJ_JOINTS])\n    actuator_idx = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in MJ_JOINTS])"
    )

# Fix data.ctrl[:] = torques
if 'data.ctrl[:] = torques' in content:
    content = content.replace('data.ctrl[:] = torques', 'data.ctrl[actuator_idx] = torques')

with open('sim2sim/play_mujoco.py', 'w') as f:
    f.write(content)

