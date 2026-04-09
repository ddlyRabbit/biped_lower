import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path('mjcf/sim2sim/robot_light.mjcf')
data = mujoco.MjData(model)

# Actuator 0 (right_hip_pitch_04)
data.qpos[7] = 0.0 # joint pos
data.qvel[6] = 0.0 # joint vel
data.ctrl[0] = 0.5 # Command is supposed to be target position for "position" actuator
mujoco.mj_step(model, data)
print(f"ctrl=0.5, pos=0.0 -> force: {data.qfrc_actuator[6]}")
