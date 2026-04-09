import re

with open('sim2sim/render_zmp_trajectory_mujoco.py', 'r') as f:
    text = f.read()

# Fix the timestep override
text = text.replace("model.opt.timestep = dt", 
"""# Keep the model's original 2000Hz timestep
    PHYSICS_DT = model.opt.timestep
    SUBSTEPS = int(round(dt / PHYSICS_DT))
""")

# Fix the simulation loop
old_loop = """    for step in range(max_steps):
        # Send joint position commands to actuators
        for short_name, act_idx in mj_act_idx.items():
            data.ctrl[act_idx] = trajectory[short_name][step]

        # Step physics (robot walks freely)
        mujoco.mj_step(model, data)"""

new_loop = """    for step in range(max_steps):
        # Send joint position commands to actuators
        for short_name, act_idx in mj_act_idx.items():
            data.ctrl[act_idx] = trajectory[short_name][step]

        # Step physics at 2000Hz
        for _ in range(SUBSTEPS):
            mujoco.mj_step(model, data)"""

text = text.replace(old_loop, new_loop)

with open('sim2sim/render_zmp_trajectory_mujoco.py', 'w') as f:
    f.write(text)

print("patched")
