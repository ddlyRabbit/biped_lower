with open('sim2sim/play_mujoco.py', 'r') as f:
    text = f.read()

old = "qp_idx = np.array([model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in MJ_JOINTS])"
new = """PHYSICS_DT = model.opt.timestep
    SUBSTEPS = int(round(POLICY_DT / PHYSICS_DT))
    qp_idx = np.array([model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in MJ_JOINTS])"""

text = text.replace(old, new)
with open('sim2sim/play_mujoco.py', 'w') as f:
    f.write(text)
print("patched")
