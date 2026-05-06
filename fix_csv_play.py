with open("/home/abhinavroy/biped_lower/sim2sim/play_csv_physics_mujoco.py", "r") as f:
    content = f.read()

content = content.replace("targets_mj = np.zeros(12, dtype=np.float32)\\n        for master_i, mj_i in enumerate(ISAAC_TO_MJ_IDX):\\n            targets_mj[mj_i] = targets_master[master_i]",
"""targets_mj = np.zeros(12, dtype=np.float32)
        for master_i, mj_i in enumerate(ISAAC_TO_MJ_IDX):
            targets_mj[mj_i] = targets_master[master_i]""")

with open("/home/abhinavroy/biped_lower/sim2sim/play_csv_physics_mujoco.py", "w") as f:
    f.write(content)
