#!/bin/bash
# Build the deploy workspace exactly as it was at <commit> and run one SIL scenario against it.
#   replay_commit.sh <commit> <scenario> <out_dir> [launch args...]
# The only change applied before building is the Ort::Env lifetime fix (without it every C++ policy node of
# this era crashes on its first inference with current ONNX Runtime; see README).
# MIT License, Copyright (c) 2026 Abhinav Roy
set -e
C=$1; SCEN=$2; OUT=$3; shift 3
SIL=$(cd "$(dirname "$0")" && pwd); MAIN=$(dirname "$SIL")
WT=${SIL_WORKTREES:-/tmp/sil_worktrees}/$C
if [ ! -d "$WT" ]; then
  git -C "$MAIN" worktree add -f --detach "$WT" "$C" > /dev/null
  sed -i 's/^\( *\)Ort::Env env(/\1static Ort::Env env(/' \
    "$WT"/deploy/biped_ws/src/biped_unified_cpp/src/unified_node.cpp "$WT"/deploy/biped_ws/src/biped_control_cpp/src/policy_node.cpp 2>/dev/null || true
  ( cd "$WT/deploy/biped_ws" && colcon build --packages-up-to biped_bringup biped_control_cpp biped_unified_cpp \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
      "-DCMAKE_CXX_FLAGS=-I$CONDA_PREFIX/include/onnxruntime/core/session -I$CONDA_PREFIX/include/onnxruntime" > "$WT/build.log" 2>&1 )
fi
source "$WT/deploy/biped_ws/install/setup.bash"
ARGS=(); for x in "$@"; do ARGS+=(--launch-arg "$x"); done
python3 "$SIL/run_sil.py" --repo "$WT" --plant-repo "$MAIN" --model deploy/student_flat.onnx --scenario "$SCEN" --out "$OUT" "${ARGS[@]}"
