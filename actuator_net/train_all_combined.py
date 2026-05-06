import os

commands = [
    "python3 train_flat.py --csv sysid_data_right_moving_only.csv sysid_data_left_moving_only.csv --joint_type hip_pitch --epochs 1000",
    "python3 train_flat.py --csv sysid_data_right_moving_only.csv sysid_data_left_moving_only.csv --joint_type hip_roll --epochs 1000",
    "python3 train_flat.py --csv sysid_data_right_moving_only.csv sysid_data_left_moving_only.csv --joint_type hip_yaw --epochs 1000",
    "python3 train_flat.py --csv sysid_data_right_moving_only.csv sysid_data_left_moving_only.csv --joint_type knee --epochs 1000",
    "python3 train_flat.py --csv sysid_data_right_moving_only.csv sysid_data_left_moving_only.csv --joint_type foot_pitch --epochs 1000",
    "python3 train_flat.py --csv sysid_data_right_moving_only.csv sysid_data_left_moving_only.csv --joint_type foot_roll --epochs 1000"
]

os.chdir("/home/abhinavroy/biped_lower/actuator_net")
for cmd in commands:
    print(f"\nRunning: {cmd}")
    os.system(cmd)
