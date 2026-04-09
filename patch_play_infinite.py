import re

with open('sim2sim/play_mujoco.py', 'r') as f:
    text = f.read()

# Replace the default duration to math.inf
text = text.replace('parser.add_argument("--duration", type=float, default=10.0)',
                    'parser.add_argument("--duration", type=float, default=float("inf"))')

# Replace the 'Q' key handler to exit instead of turn left (turn left is A, right is D, or wait, earlier I bound Q/E to wz).
# Wait, Q and ESC should exit.
old_callback = """    def key_callback(keycode):
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
            cmd_vel[:] = 0.0"""

new_callback = """    def key_callback(keycode):
        nonlocal playing
        if keycode == 265:  # Up arrow
            cmd_vel[0] += 0.1
        elif keycode == 264:  # Down arrow
            cmd_vel[0] -= 0.1
        elif keycode == 263:  # Left arrow
            cmd_vel[1] += 0.1
        elif keycode == 262:  # Right arrow
            cmd_vel[1] -= 0.1
        elif keycode == 65:  # A (yaw left)
            cmd_vel[2] += 0.2
        elif keycode == 68:  # D (yaw right)
            cmd_vel[2] -= 0.2
        elif keycode == 32:  # Space
            cmd_vel[:] = 0.0
        elif keycode in (81, 256):  # Q or ESC
            playing = False"""
text = text.replace(old_callback, new_callback)

# We need to define `playing` earlier in `main` so it's captured by the nonlocal closure.
old_main = """def main():
    parser = argparse.ArgumentParser(description="Play ONNX policy in MuJoCo")"""

new_main = """def main():
    playing = True
    parser = argparse.ArgumentParser(description="Play ONNX policy in MuJoCo")"""
text = text.replace(old_main, new_main)

# And in the loop, use the while loop correctly instead of `for step in range(n_steps):` if duration is inf.
# Wait, if duration is inf, `int(float('inf'))` raises an OverflowError!
old_loop_setup = """    n_steps = int(args.duration / POLICY_DT)
    try:
        for step in range(n_steps):"""

new_loop_setup = """    step = 0
    try:
        while playing and (args.duration == float('inf') or step < int(args.duration / POLICY_DT)):"""
text = text.replace(old_loop_setup, new_loop_setup)

# Increment step at end of loop
old_viewer_sync = """            if viewer is not None:
                viewer.sync()
                if not viewer.is_running():
                    break

            dt = time.perf_counter() - t0"""

new_viewer_sync = """            if viewer is not None:
                viewer.sync()
                if not viewer.is_running():
                    playing = False

            dt = time.perf_counter() - t0
            step += 1"""
text = text.replace(old_viewer_sync, new_viewer_sync)

with open('sim2sim/play_mujoco.py', 'w') as f:
    f.write(text)

print("patched")
