#!/bin/bash
# Record 4 agents with individual cameras, merge into 2x2 grid
set -e

CHECKPOINT=${1:-/results/winners/v57_rough_model_6498.pt}
VIDEO_LEN=${2:-500}
ROUGH_FLAG=${3:---rough}
OUT_DIR="/results/videos/grid"
mkdir -p "$OUT_DIR"

for i in 0 1 2 3; do
    echo "=== Recording agent $i ==="
    DIR="$OUT_DIR/agent_$i"
    mkdir -p "$DIR"
    /isaac-sim/python.sh /workspace/biped_locomotion/biped_play_rsl.py \
        --checkpoint "$CHECKPOINT" \
        $ROUGH_FLAG \
        --video --video_length "$VIDEO_LEN" \
        --num_envs 4 \
        --env_index "$i" \
        --video_dir "$DIR" \
        --headless
    echo "=== Agent $i done ==="
done

echo "=== Merging 2x2 grid ==="
ffmpeg -y \
    -i "$OUT_DIR/agent_0/rl-video-step-0.mp4" \
    -i "$OUT_DIR/agent_1/rl-video-step-0.mp4" \
    -i "$OUT_DIR/agent_2/rl-video-step-0.mp4" \
    -i "$OUT_DIR/agent_3/rl-video-step-0.mp4" \
    -filter_complex "
        [0:v]scale=640:360[v0];
        [1:v]scale=640:360[v1];
        [2:v]scale=640:360[v2];
        [3:v]scale=640:360[v3];
        [v0][v1]hstack=inputs=2[top];
        [v2][v3]hstack=inputs=2[bottom];
        [top][bottom]vstack=inputs=2[out]
    " -map "[out]" -c:v libx264 -crf 23 -preset fast \
    "$OUT_DIR/v57_rough_grid.mp4"

echo "=== DONE: $OUT_DIR/v57_rough_grid.mp4 ==="
