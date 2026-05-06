import os
import re

readme_path = "/home/abhinavroy/biped_lower/sim2sim/README.md"
with open(readme_path, "r") as f:
    content = f.read()

new_section = """### `play_csv_physics_mujoco.py` — ZMP Trajectory Physics Playback

Renders a CSV joint trajectory using full MuJoCo physics (gravity, contacts, friction). 
Unlike `play_traj_mujoco.py` which teleports kinematic links, this script computes PD torques at 2000Hz (using hardware Kp/Kd parameters) to trace the ZMP trajectory dynamically.

```bash
# Render headless (auto-skips the 8.5s idle phase in ZMP trajectories)
xvfb-run -a python sim2sim/play_csv_physics_mujoco.py \\
    --csv deploy/biped_ws/src/biped_bringup/config/trajectory.csv \\
    --video trajectory_physics.mp4
```

**Key settings:**
- Skips first 8.5 seconds of simulation instantly to bypass static holding phases.
- Renders the next 5.0 seconds at 50fps.
- Applies Kp=150 to ankle pitch/roll to maintain contact rigidity during the ZMP test.

"""

content = content.replace("**CSV format:**", new_section + "**CSV format:**")

with open(readme_path, "w") as f:
    f.write(content)
