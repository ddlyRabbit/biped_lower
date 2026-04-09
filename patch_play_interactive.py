import re

with open('sim2sim/play_mujoco.py', 'r') as f:
    text = f.read()

# Add the interactive state and callback
state_init = """    cmd_vel = np.array([args.cmd_vx, args.cmd_vy, args.cmd_wz], dtype=np.float32)
    last_action = np.zeros(12, dtype=np.float32)"""

interactive_state = """    cmd_vel = np.array([args.cmd_vx, args.cmd_vy, args.cmd_wz], dtype=np.float32)
    last_action = np.zeros(12, dtype=np.float32)

    def key_callback(keycode):
        if keycode == 265:  # Up arrow
            cmd_vel[0] += 0.1
        elif keycode == 264:  # Down arrow
            cmd_vel[0] -= 0.1
        elif keycode == 263:  # Left arrow
            cmd_vel[1] += 0.1
        elif keycode == 262:  # Right arrow
            cmd_vel[1] -= 0.1
        elif keycode == 81:  # Q
            cmd_vel[2] += 0.2
        elif keycode == 69:  # E
            cmd_vel[2] -= 0.2
        elif keycode == 32:  # Space
            cmd_vel[:] = 0.0
        
        # Clamp commands to training ranges
        cmd_vel[0] = np.clip(cmd_vel[0], -0.5, 1.5)
        cmd_vel[1] = np.clip(cmd_vel[1], -0.5, 0.5)
        cmd_vel[2] = np.clip(cmd_vel[2], -1.0, 1.0)
        print(f"Command update: vx={cmd_vel[0]:.2f} vy={cmd_vel[1]:.2f} wz={cmd_vel[2]:.2f}")"""

text = text.replace(state_init, interactive_state)

# Add the key_callback to the viewer
old_viewer = """    if not args.headless and not args.video:
        pass
        viewer = mujoco.viewer.launch_passive(model, data)
    else:"""

new_viewer = """    if not args.headless and not args.video:
        pass
        viewer = mujoco.viewer.launch_passive(model, data, key_callback=key_callback)
    else:"""

text = text.replace(old_viewer, new_viewer)

with open('sim2sim/play_mujoco.py', 'w') as f:
    f.write(text)

print("patched")
