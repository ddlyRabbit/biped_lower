# MIT License, Copyright (c) 2026 Abhinav Roy
"""Create a ROS 2 Jazzy environment from RoboStack + conda-forge with py-rattler (no conda client needed)."""
import asyncio, sys
from rattler import Channel, Platform, solve, install, VirtualPackage
SPECS = ["python 3.12.*", "ros-jazzy-ros-base", "ros-jazzy-sensor-msgs", "ros-jazzy-geometry-msgs", "ros-jazzy-std-msgs",
         "ros-jazzy-tf2-ros", "ros-jazzy-rosidl-default-generators", "ros-jazzy-ament-cmake", "ros-jazzy-ament-cmake-python",
         "ros-jazzy-rosbag2", "ros-jazzy-rosbag2-storage-mcap",
         "colcon-common-extensions", "cmake", "ninja", "make", "pkg-config", "compilers", "yaml-cpp", "onnxruntime-cpp",
         "numpy", "pyyaml", "setuptools", "ros-jazzy-robot-state-publisher", "ros-jazzy-xacro"]
async def main(prefix):
    chans = [Channel("robostack-jazzy"), Channel("conda-forge")]
    recs = await solve(chans, SPECS, platforms=[Platform.current(), Platform("noarch")],
                       virtual_packages=VirtualPackage.detect())
    print(len(recs), "packages solved", flush=True)
    await install(recs, target_prefix=prefix)
    print("installed to", prefix)
asyncio.run(main(sys.argv[1]))
