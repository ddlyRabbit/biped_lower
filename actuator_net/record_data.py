import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import JointState
from biped_msgs.msg import MITCommandArray, MotorStateArray
import pandas as pd
import numpy as np

# Ankle Linkage Kinematics (Option 1: Decoupled Virtual Joints)
PITCH_GAIN = 1.2757
ROLL_GAIN = 0.9736

class DataRecorder(Node):
    def __init__(self):
        super().__init__('sysid_data_recorder')
        
        self.cmd_sub = message_filters.Subscriber(self, MITCommandArray, '/joint_commands')
        self.js_sub = message_filters.Subscriber(self, JointState, '/joint_states')
        self.ms_sub = message_filters.Subscriber(self, MotorStateArray, '/motor_states')
        
        # Sync the 3 topics perfectly using timestamps (slop = 20ms tolerance)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.cmd_sub, self.js_sub, self.ms_sub], queue_size=10, slop=0.02
        )
        self.ts.registerCallback(self.sync_callback)
        
        self.records = []
        self.get_logger().info("Data Recorder started. Press Ctrl+C to save to 'sysid_data.csv'")

    def sync_callback(self, cmd_msg, js_msg, ms_msg):
        row = {'time': cmd_msg.header.stamp.sec + cmd_msg.header.stamp.nanosec * 1e-9}
        
        # 1. Parse Joint Targets (from policy / state machine)
        for cmd in cmd_msg.commands:
            row[f"{cmd.joint_name}_target"] = cmd.position
            
        # 2. Parse Actual Joint States
        for i, name in enumerate(js_msg.name):
            if i < len(js_msg.position):
                row[f"{name}_pos"] = js_msg.position[i]
            if i < len(js_msg.velocity):
                row[f"{name}_vel"] = js_msg.velocity[i]
                
        # 3. Parse Motor States and extract raw torques
        motor_taus = {}
        for m in ms_msg.motors:
            motor_taus[m.joint_name] = m.torque

        # Map direct joint torques (Hips, Knees)
        direct_joints = [
            "L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee",
            "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee"
        ]
        for j in direct_joints:
            if j in motor_taus:
                row[f"{j}_tau"] = motor_taus[j]

        # 4. Compute Virtual Ankle Torques using inverse Jacobian
        if "L_foot_top" in motor_taus and "L_foot_bottom" in motor_taus:
            lt = motor_taus["L_foot_top"]
            lb = motor_taus["L_foot_bottom"]
            row["L_foot_pitch_tau"] = -1.0 * PITCH_GAIN * (lt - lb)  # pitch_sign = -1 for Left
            row["L_foot_roll_tau"] = -1.0 * ROLL_GAIN * (lt + lb)

        if "R_foot_top" in motor_taus and "R_foot_bottom" in motor_taus:
            rt = motor_taus["R_foot_top"]
            rb = motor_taus["R_foot_bottom"]
            row["R_foot_pitch_tau"] = 1.0 * PITCH_GAIN * (rt - rb)   # pitch_sign = 1 for Right
            row["R_foot_roll_tau"] = -1.0 * ROLL_GAIN * (rt + rb)

        self.records.append(row)

    def save(self):
        if len(self.records) == 0:
            self.get_logger().warn("No data recorded!")
            return
            
        df = pd.DataFrame(self.records)
        df.to_csv("sysid_data.csv", index=False)
        self.get_logger().info(f"Saved {len(df)} frames to sysid_data.csv")

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
