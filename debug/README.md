# Debug Scripts

One-off debug scripts used during training development. Run inside the Isaac Sim Docker container on GCP.

| Script | Purpose |
|--------|---------|
| `biped_debug.py` | Print body/joint ordering for feet and knees |
| `biped_debug2.py` | Resolve left/right feet separately in contact sensor |
| `biped_debug4.py` | Write foot ordering to file |
| `debug_air_time.py` | Inspect feet_air_time reward components |
| `debug_feet_contact.py` | Run trained policy, print contact info |
| `debug_v37.py` | V37 config inspection |
| `test_import.py` | Test Isaac Lab imports |

## Usage

```bash
docker run --gpus all -it \
  -v /home/ubuntu/workspace:/workspace \
  isaaclab:latest /isaac-sim/python.sh \
  /workspace/biped_locomotion/debug/<script>.py --headless
```

These are not part of the deployment pipeline. Safe to ignore.
