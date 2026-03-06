#!/bin/bash
# Play/evaluate trained biped policy
# Usage: ./play.sh --checkpoint <path>

set -e
cd /workspace/biped_locomotion

export PYTHONPATH="/workspace/biped_locomotion:${PYTHONPATH}"

/isaac-sim/python.sh /isaaclab/scripts/reinforcement_learning/skrl/play.py \
    --task Isaac-Velocity-Flat-Biped-Play-v0 \
    --algorithm PPO \
    "$@"
