import re

with open('sim2sim/play_mujoco.py', 'r') as f:
    text = f.read()

# Replace the qvel logic to rotate it into the body frame
old_qvel = """    if obs_dim == 48:
        # [0-2] base linear velocity (body frame)
        obs[0:3] = data.qvel[0:3].copy()
        offset = 3"""

new_qvel = """    # Projected gravity from orientation sensor
    quat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "orientation")
    quat_adr = model.sensor_adr[quat_id]
    base_quat = data.sensordata[quat_adr:quat_adr + 4].copy()  # w, x, y, z

    if obs_dim == 48:
        # [0-2] base linear velocity (body frame)
        # data.qvel[0:3] is in world frame. We must rotate it to body frame using inverse base_quat
        lin_vel_world = data.qvel[0:3].copy()
        lin_vel_body = quat_rotate_inverse(base_quat, lin_vel_world)
        obs[0:3] = lin_vel_body
        offset = 3"""
text = text.replace(old_qvel, new_qvel)

# Since we moved base_quat extraction up, remove the old one
old_quat = """    # Projected gravity from orientation sensor
    quat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "orientation")
    quat_adr = model.sensor_adr[quat_id]
    base_quat = data.sensordata[quat_adr:quat_adr + 4].copy()  # w, x, y, z
    gravity_world = np.array([0.0, 0.0, -1.0])"""

new_quat = """    # Projected gravity
    gravity_world = np.array([0.0, 0.0, -1.0])"""
text = text.replace(old_quat, new_quat)

with open('sim2sim/play_mujoco.py', 'w') as f:
    f.write(text)

print("patched")
