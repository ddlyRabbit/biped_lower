# Biped Robot — Bringup Guide

## Hardware Overview

- **Motors:** 12× RobStride (RS02/RS03/RS04) on CAN bus, IDs 1–12
- **IMU:** BNO085 (I2C) or IM10A (USB serial)
- **Policy:** Student ONNX (45-dim obs, 12 actions, tanh output)

---

## Part 1: Platform Setup

Choose your platform and follow the corresponding section.

### A. Raspberry Pi 5 Setup

**OS:** Ubuntu 24.04 | **ROS2:** Jazzy | **Python:** 3.12  
**CAN:** Waveshare 2-CH CAN HAT (MCP2515 × 2) — dual bus (can0=right, can1=left)

#### 1. System packages

```bash
sudo apt update && sudo apt install -y \
  net-tools can-utils i2c-tools git python3-pip libeigen3-dev
```

#### 2. ROS2 Jazzy

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

#### 3. BNO085 IMU dependencies (if using I2C IMU)

```bash
sudo apt install -y python3-lgpio
pip3 install --break-system-packages adafruit-circuitpython-bno08x adafruit-blinka
```

#### 4. Python packages

```bash
pip3 install --break-system-packages -r ~/biped_lower/deploy/requirements.txt
```

#### 5. CAN HAT device tree (one-time)

Add to `/boot/firmware/config.txt`:
```
dtparam=spi=on
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=25
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23
dtoverlay=spi-bcm2835-overlay
```
Reboot. Verify: `ls /sys/class/net/ | grep can` → should show `can0` and `can1`.

#### 6. CAN bring-up (every boot)

```bash
bash ~/biped_lower/deploy/scripts/setup_can.sh
```

#### 7. IMU wiring (BNO085)

| BNO085 Pin | RPi 5 Header | Pin # |
|---|---|---|
| SDA | I2C1_SDA | **Pin 3** |
| SCL | I2C1_SCL | **Pin 5** |
| RST | GPIO4 | **Pin 7** |
| VIN | 3.3V | **Pin 1** |
| GND | GND | **Pin 6** |

Verify: `sudo i2cdetect -y 1` → expect `4b`

---

### B. Jetson Orin Nano Setup

**OS:** Ubuntu 22.04 | **ROS2:** Humble | **Python:** 3.10  
**CAN:** Native CAN controller — single bus (can0, all 12 motors)

#### 1. System packages

```bash
sudo apt update && sudo apt install -y \
  can-utils i2c-tools python3-pip
```

#### 2. ROS2 Humble

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y \
  ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions ros-humble-rosbag2-storage-mcap
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

#### 3. BNO085 IMU dependencies (if using I2C IMU)

```bash
pip3 install adafruit-circuitpython-bno08x adafruit-blinka
sudo pip3 install Jetson.GPIO --upgrade
```

#### 4. Python packages

```bash
pip3 install onnxruntime numpy pyyaml pyserial scipy
```

#### 5. CAN bring-up (every boot)

```bash
bash ~/biped_lower/deploy/scripts/setup_can_jetson.sh
```

#### 6. IMU wiring (BNO085)

| BNO085 Pin | Jetson 40-Pin Header | Pin # |
|---|---|---|
| SDA | I2C1_SDA (Bus 7) | **Pin 3** |
| SCL | I2C1_SCL (Bus 7) | **Pin 5** |
| RST | GPIO09 | **Pin 7** |
| VIN | 3.3V | **Pin 1** |
| GND | GND | **Pin 6** |

Verify: `sudo i2cdetect -y -r 7` → expect `4b`

---

## Part 2: Build ROS2 Workspace (common)

```bash
cd ~/biped_lower/deploy/biped_ws
colcon build --symlink-install
source install/setup.bash
echo 'source ~/biped_lower/deploy/biped_ws/install/setup.bash' >> ~/.bashrc
```

---

## Part 3: Robot Bringup (common)

### Launch Command

```bash
ros2 launch biped_bringup bringup.launch.py \
  can_driver:=<driver>         \
  robot_config:=<config>       \
  imu_type:=<imu>              \
  calibration_file:=<cal>      \
  onnx_model:=<path>           \
  gain_scale:=<scale>
```

### Launch Parameters

| Parameter | Description | Default | Options |
|---|---|---|---|
| `can_driver` | CAN driver node | `can_bus_node_cpp` | `can_bus_node_cpp` (recommended), `can_bus_node_async`, `can_bus_node` |
| `robot_config` | Motor-to-CAN mapping | `robot.yaml` | `robot.yaml` (RPi dual-bus), `robot_jetson.yaml` (Jetson single-bus) |
| `imu_type` | IMU driver | `bno085` | `bno085` (I2C), `im10a` (USB serial) |
| `calibration_file` | Motor calibration | `calibration.yaml` | Any `.yaml` in `config/` |
| `onnx_model` | Student policy ONNX | `student_flat.onnx` | Absolute path to `.onnx` file |
| `gain_scale` | PD gain multiplier | `1.0` | `0.1`–`1.0` (start low!) |
| `record` | Enable rosbag recording | `false` | `true` / `false` |

### Platform-Specific Examples

**RPi 5 (dual CAN, BNO085 I2C):**
```bash
ros2 launch biped_bringup bringup.launch.py \
  can_driver:=can_bus_node_cpp \
  robot_config:=robot.yaml \
  imu_type:=bno085 \
  calibration_file:=calibration.yaml \
  onnx_model:=~/biped_lower/deploy/v76_student_5600_tanh.onnx \
  gain_scale:=0.3
```

**Jetson Orin Nano (single CAN, BNO085 I2C):**
```bash
ros2 launch biped_bringup bringup.launch.py \
  can_driver:=can_bus_node_cpp \
  robot_config:=robot_jetson.yaml \
  imu_type:=bno085 \
  calibration_file:=calibration.yaml \
  onnx_model:=~/biped_lower/deploy/v76_student_5600_tanh.onnx \
  gain_scale:=0.3
```

**Either platform with IM10A USB IMU:**
```bash
ros2 launch biped_bringup bringup.launch.py \
  can_driver:=can_bus_node_cpp \
  imu_type:=im10a \
  ...
```

### IMU Parameter Overrides

The BNO085 node auto-detects the platform (Jetson vs RPi) for GPIO reset. Override if needed:

```bash
# RPi 5: I2C bus 1
ros2 launch biped_bringup bringup.launch.py imu_type:=bno085 i2c_bus:=1

# Jetson: I2C bus 7 (default)
ros2 launch biped_bringup bringup.launch.py imu_type:=bno085 i2c_bus:=7
```

---

## Part 4: Testing Sequence

### Step 1: Scan Motors

```bash
python3 ~/biped_lower/deploy/scripts/scan_motors.py
```

### Step 2: Calibrate (once per assembly)

```bash
ros2 launch biped_bringup calibrate.launch.py
```

### Step 3: Test Hardware (no policy)

```bash
ros2 launch biped_bringup hardware.launch.py \
  can_driver:=can_bus_node_cpp \
  imu_type:=bno085 \
  calibration_file:=calibration.yaml

# Verify:
ros2 topic echo /joint_states     # motor feedback
ros2 topic echo /imu/data         # IMU data
ros2 topic echo /safety/status    # should be True
```

### Step 4: Suspended Test (robot hanging)

⚠️ **Robot must be suspended with feet off ground!**

```bash
# Terminal 1: launch
ros2 launch biped_bringup bringup.launch.py \
  can_driver:=can_bus_node_cpp \
  onnx_model:=~/biped_lower/deploy/v76_student_5600_tanh.onnx \
  gain_scale:=0.3

# Terminal 2: teleop
ros2 run biped_teleop keyboard_teleop
```

Press `SPACE` → stand, then `v` → SIM_WALK (safe viz), then `g` → WALK (real).

### Step 5: Ground Test

Same as Step 4 but with `gain_scale:=0.5` or higher.

---

## State Machine

```
 IDLE ──SPACE──▶ STAND ──g──▶ WALK ──b──▶ STAND
                  ├──v──▶ SIM_WALK ──b──▶ STAND
                  ├──p──▶ PLAY_TRAJ_SIM ──b──▶ STAND
                  ├──P──▶ PLAY_TRAJ ──b──▶ STAND
                  ├──t──▶ WIGGLE_SEQ ──(auto)──▶ STAND
                  └──y──▶ WIGGLE_ALL ──b──▶ STAND
              ESC ── any ──▶ ESTOP
```

### Keyboard Teleop Keys

| Key | Action |
|-----|--------|
| SPACE | IDLE → STAND |
| g | STAND → WALK |
| v | STAND → SIM_WALK (viz only) |
| p | STAND → PLAY_TRAJ_SIM (CSV viz) |
| P | STAND → PLAY_TRAJ (CSV motors) |
| t | STAND → WIGGLE_SEQ |
| y | STAND → WIGGLE_ALL |
| b | any → STAND |
| ESC | any → ESTOP |
| w/s | forward / backward velocity |
| a/d | left / right velocity |
| q/e | yaw left / yaw right |
| x | zero all velocities |
| 1-5 | speed presets (0.1–1.0 m/s) |

---

## Gain Tuning

| gain_scale | Use case |
|---|---|
| 0.1–0.2 | First power-on test |
| 0.3 | Suspended test |
| 0.5 | Suspended walking |
| 0.7 | Ground standing |
| 1.0 | Full sim-matched gains |

Base PD gains are in `obs_builder.py`:
```python
DEFAULT_GAINS = {
    "L_hip_pitch": (180.0, 6.5), "R_hip_pitch": (180.0, 6.5),
    "L_hip_roll":  (180.0, 6.5), "R_hip_roll":  (180.0, 6.5),
    "L_hip_yaw":   (180.0, 3.0), "R_hip_yaw":   (180.0, 3.0),
    "L_knee":      (180.0, 3.0), "R_knee":      (180.0, 3.0),
    "L_foot_pitch": (30.0, 1.0), "R_foot_pitch": (30.0, 1.0),
    "L_foot_roll":  (30.0, 1.0), "R_foot_roll":  (30.0, 1.0),
}
```

---

## Emergency Stop

ESC key in teleop, or:
```bash
ros2 topic pub --once /state_command std_msgs/String "data: ESTOP"
```

Reset: `ros2 topic pub --once /state_command std_msgs/String "data: RESET"`

---

## Platform Comparison

| | RPi 5 | Jetson Orin Nano |
|---|---|---|
| OS | Ubuntu 24.04 | Ubuntu 22.04 |
| ROS2 | Jazzy | Humble |
| Python | 3.12 | 3.10 |
| CAN | Dual MCP2515 SPI (can0+can1) | Native CAN (can0 only) |
| Motor layout | Right=can0, Left=can1 | All 12 on can0 |
| CAN setup | `setup_can.sh` | `setup_can_jetson.sh` |
| Robot config | `robot.yaml` | `robot_jetson.yaml` |
| BNO085 I2C bus | Bus 1 (`i2cdetect -y 1`) | Bus 7 (`i2cdetect -y -r 7`) |
| BNO085 GPIO reset | lgpio (RPi) | Jetson.GPIO (auto-detected) |
| IMU pin wiring | Pin 3/5/7/1/6 | Pin 3/5/7/1/6 (same physical pins!) |

---

## Troubleshooting

| Problem | Solution |
|---|---|
| No CAN device (RPi) | Run `setup_can.sh`, check dtoverlay in `/boot/firmware/config.txt` |
| No CAN device (Jetson) | Run `setup_can_jetson.sh`, check `ip link show can0` |
| Motor no response | Check 48V power, run `scan_motors.py`, verify CAN ID in `robot.yaml` |
| BNO085 not detected | Check wiring, run `i2cdetect` for correct bus (RPi: `-y 1`, Jetson: `-y -r 7`) |
| IM10A no data | Check `ls /dev/ttyUSB0`, add user to `dialout` group |
| Robot falls immediately | Reduce `gain_scale` to 0.1, verify calibration offsets |
| ESTOP triggered | Check: `ros2 topic echo /safety/fault` |
| Ankle motors track poorly | Re-calibrate ankles, check linkage rod lengths |
