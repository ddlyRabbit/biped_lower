#!/bin/bash
# Train biped flat terrain velocity tracking with skrl PPO
# Usage: ./train.sh [--num_envs N] [--max_iterations N] [--headless]

set -e
cd /workspace/biped_locomotion

# Register our custom env
export PYTHONPATH="/workspace/biped_locomotion:${PYTHONPATH}"

# Default args
EXTRA_ARGS="--headless"

# Pass through any CLI args
for arg in "$@"; do
    EXTRA_ARGS="$EXTRA_ARGS $arg"
done

echo "=== Starting Biped Flat Velocity Training with skrl ==="
echo "Task: Isaac-Velocity-Flat-Biped-v0"
echo "Algorithm: PPO (skrl)"
echo "Extra args: $EXTRA_ARGS"

/isaac-sim/python.sh /isaaclab/scripts/reinforcement_learning/skrl/train.py \
    --task Isaac-Velocity-Flat-Biped-v0 \
    --algorithm PPO \
    $EXTRA_ARGS
