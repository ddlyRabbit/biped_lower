# source sim2real_sil/activate_ros_env.sh <prefix>   — activate the RoboStack ROS 2 Jazzy environment
# MIT License, Copyright (c) 2026 Abhinav Roy
export CONDA_PREFIX=${1:-$HOME/ros_env}
export PATH=$CONDA_PREFIX/bin:$PATH
for f in $CONDA_PREFIX/etc/conda/activate.d/*.sh; do [ -f "$f" ] && . "$f"; done
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=1
