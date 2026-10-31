# sim2real_sil — run the real robot software against a simulated robot

Software-in-the-loop (SIL) for biped_lower. The deploy stack runs unmodified, against a MuJoCo robot that pretends to be the hardware: the bringup launch file and every C++/Python node (`unified_node`, or `can_bus_node_cpp` + `im10a_node` + `policy_node_cpp`), plus `state_machine_node_cpp` and `safety_node_cpp`. No robot, CAN adapter or IMU is needed. The stack still believes it is talking to the real motors and IMU.

```
 bringup.launch.py (unchanged)                         emulator.py (MuJoCo, 2 kHz, real time)
 ┌──────────────────────────────┐   SocketCAN frames   ┌─────────────────────────────────────┐
 │ can_bus_node_cpp / unified   │ ◄──────────────────► │ 12 RobStride motors: MIT decode with │
 │   (LD_PRELOAD libsimcan.so)  │   UNIX SEQPACKET     │ firmware ranges, calibration offsets,│
 │ im10a_node / unified         │ ◄── /dev/pts/N ───── │ ankle linkage, torque limits         │
 │ policy, state machine, safety│   WIT IM10A packets  │ IM10A: quat, gyro, accel @ 200 Hz    │
 └──────────────────────────────┘                      │ "hand at the pelvis" support         │
              ▲  /state_command, /cmd_vel              └─────────────────────────────────────┘
              └──── run_sil.py (operator script + ground truth) ──── verdict.py (pass/fail)
```

## How the hardware is faked

- **CAN.** `simcan/libsimcan.so` is a 100-line `LD_PRELOAD` shim. A `socket(PF_CAN, …)` call becomes a UNIX `SOCK_SEQPACKET` socket. `ioctl(SIOCGIFINDEX)` connects that socket to `$SIMCAN_DIR/can0` or `can1`, and `bind`/`setsockopt` become no-ops. After that the driver's `read`/`write`/`poll` calls carry `struct can_frame`s unchanged. No kernel CAN support is needed, so it runs in containers and CI.
- **Motors.** `emulator.py` decodes the RobStride protocol: enable, disable, the mode write to 0x7005, mode read-back, MIT operation-control frames, and torque feed-forward carried in the CAN id. It decodes with the motors' **own** scaling ranges, so a driver that encodes with the wrong table (as in April) produces the wrong gains, exactly as on the robot. Every command is answered with an operation-status frame.
- **Encoders.** Raw encoder readings include the robot's calibration offsets and directions (`deploy/biped_ws/calibration.yaml`).
- **Ankles.** The two ankle motors drive pitch and roll through the parallel linkage (K_P = 1.2757, K_R = 0.9736). Joint torques come from Jᵀ.
- **Torque limits** are the motors' own: RS04 120 Nm, RS03 60 Nm, RS02 17 Nm.
- **IMU.** A pseudo-terminal streams WIT IM10A packets (0x51, 0x52, 0x59) at 200 Hz. Its slave end is `$SIMCAN_DIR/ttyIMU` and is passed as `im10a_port`. The driver's auto-baud probe finds the expected output content and proceeds.
- **Plant.** `mjcf/sim2sim/robot_light.mjcf` at 2 kHz, with the training armature, 0.1 Nm Coulomb friction and flat ground.
- **Operator.** `run_sil.py` follows the real procedure. The robot is held at the pelvis by a spring-damper support. The script sends START, the stack goes to STAND, the support fades out over 2 s, then WALK at 0 and then 0.3 m/s, then STOP. The script records `/state_machine`, `/safety/*` and ground truth from the emulator.

## Install (once)

```bash
# ROS 2 Jazzy from RoboStack (conda-forge); no apt repositories needed
pip install py-rattler
python3 sim2real_sil/install_ros_env.py ~/ros_env      # ~2.5 GB
source sim2real_sil/activate_ros_env.sh ~/ros_env
cc -O2 -shared -fPIC -o sim2real_sil/simcan/libsimcan.so sim2real_sil/simcan/simcan.c -ldl
cd deploy/biped_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
   "-DCMAKE_CXX_FLAGS=-I$CONDA_PREFIX/include/onnxruntime/core/session -I$CONDA_PREFIX/include/onnxruntime"
pip install mujoco numpy pyyaml       # for the emulator, in any Python ≥ 3.10 (pass it with --emu-python)
```

## Run

```bash
source deploy/biped_ws/install/setup.bash
# the separate C++ nodes (the configuration believed to have walked in August)
python3 sim2real_sil/run_sil.py --scenario walk --out runs/cpp_walk \
  --launch-arg unified:=false --launch-arg can_driver:=can_bus_node_cpp \
  --launch-arg control_driver:=biped_control_cpp --launch-arg imu_type:=im10a_cpp --launch-arg imu_filter:=true
python3 sim2real_sil/verdict.py runs/cpp_walk
# any historical commit, built as it was:
sim2real_sil/replay_commit.sh <commit> stand runs/<name> unified:=false can_driver:=can_bus_node_cpp \
  control_driver:=biped_control_cpp imu_type:=im10a_cpp
```

**Scenarios:**
- `boot`: start-up only.
- `stand`: hold, START, release.
- `walk`: the August procedure.
- `reentry`: WALK, STOP, WALK.
- `reentry_held`: the same, held through STOP.
- `walk_late_release`: released only after WALK starts.

**Emulator faults:** pass `--fault '{"can_drop":0.1}'`, `{"imu_rot":[[…]]}` (IMU mounting or sign error) or `{"imu_freeze_s":0.14}`.

**Counterfactual plant:** `--emu-arg=--ankle-joint-space` applies the ankle PD in joint space, as training assumed.

**Pass/fail** (`verdict.py`). These rules are generic: written once and applied to every run.

| Rule | Passes when |
|---|---|
| boot | No e-stop before START. |
| hold | Joint-velocity RMS < 0.5 rad/s while held. |
| stand | No fall, and tilt < 20°, after release. |
| gains | Every motor executes the commanded gain to within 5%. Not applied to `unified:=true`, because unified_node ignores `/joint_commands`. |
| walk | No fall or e-stop, and forward speed ≥ 50% of the command. |
| stop | No fall within 3 s of STOP. |
| reentry | The second WALK entry's torque peak is ≤ max(1.5× the first, first + 10 Nm). |

## Results (31 Oct 2026)

**Historical replays.** Each commit is built as it was, and only the Ort::Env fix is applied. The plant, emulator and verdict rules are the same for every run.

| Catalogue problem | Bug commit | Fix commit | Result |
|---|---|---|---|
| D114: C++ MIT scaling table set to 200 | `gains` FAIL: the motors executed 2.5–25× the commanded Kp (3000 for 120) | `gains` pass (1.00) | **caught** |
| D127: C++ safety tilt check inverted by the new gravity convention (fatal) | `boot` FAIL: "Pitch ±180° exceeds limit" while upright, so safety never clears and START is impossible | boot pass | **caught** |
| D130: uninitialised gain-ramp timer (STAND vibration) | hold pass | — | not reproduced: the defect is undefined behaviour, and the uninitialised value was benign in this build |
| D152: stale last action on WALK re-entry | inconclusive | inconclusive | the June policy falls while walking in this plant before re-entry can be compared |

April-era commits also fall in STAND after release, at both bug and fix commits. Their crouched default pose and lower gains do not stand unsupported on this plant, so the `stand` rule does not tell them apart.

**Ankle linkage, counterfactual at HEAD.** The robot is released after WALK starts, then walks at 0.3 m/s.

| Ankle model | Forward speed |
|---|---|
| As the hardware applies it | **0.02 m/s** (fails `walk`) |
| `--ankle-joint-space` (as trained) | **0.19 m/s** |

Same stack, same policy. The independent fault-injection harness gives 0.065 vs 0.20 m/s.

## What it found at the last pushed commit (f4e66b7)

1. **`unified_node` and `policy_node_cpp` keep their `Ort::Env` as a local in `init_onnx()`.** The session outlives it, so this is a use-after-free. With ONNX Runtime 1.31 the first inference in WALK dies with SIGBUS inside ORT's logger. It apparently did no harm with the Pi's ORT version. Fixed in this branch by making the Env `static`.
2. **`unified:=true` cannot walk.** `unified_node` publishes `/joint_commands` best-effort, while `safety_node_cpp` subscribes reliable. ROS 2 never connects the two, so the safety node's command watchdog sees nothing once the state machine stops publishing in WALK, and e-stops about 0.15 s into WALK. Not fixed here; the change is one QoS line, but it is a safety-path decision.
3. **With the separate C++ nodes the robot stands and steps, but does not walk forward.** It steps at about 1.4 Hz for 16 s at 0.3 m/s commanded and covers 0.04 m/s. The drivers send averaged ankle gains to both motors, so through the linkage the ankle is 3.25× stiffer in pitch and 1.9× in roll than trained. The independent fault-injection harness predicts the same: 0.065 m/s with the as-deployed ankle gains, 0.20 m/s with the trained ones. The `--ankle-joint-space` counterfactual tests this directly.
4. **STOP during walking drops the robot.** On STAND entry `state_machine_node_cpp` sets the gains to 10% Kp and 20% Kd for the 2 s position ramp, then ramps them back up. That soft start is meant for standing up from limp, but it also runs on WALK → STAND, so a free-standing robot collapses about 1 s after STOP. The real robot was held at that moment.
5. **The STAND controller relies on the stiff ankles.** With the ankle PD in joint space at the trained gains (30/2.8), the robot falls when released in STAND. It stands as deployed only because the linkage makes the ankles 3.25× stiffer.

## Limits

- **Plant.** The plant is the training MJCF, with a linear linkage model, rigid contacts and flat ground. CAN timing is idealised (replies within 0.5 ms, no bus arbitration).
- **IMU.** The IMU is ideal and aligned with the base, apart from gyro noise.
- **Results the SIL cannot give:**
  - It finds integration, protocol, configuration, state-machine and safety-path defects, and gross dynamics problems.
  - It does not predict hardware fall rates.
- **IMU coverage.** Only the IM10A is emulated (22 Apr 2026 onward). The BNO085 runs over I2C and is not.
- **Timing.** Real time on 2 cores; the run records its real-time factor and worst emulator lag in `summary.json`.

MIT License — see `LICENSE` at the repository root.
