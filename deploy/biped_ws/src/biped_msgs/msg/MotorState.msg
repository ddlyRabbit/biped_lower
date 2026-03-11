string joint_name
uint8 can_id
float32 position        # absolute encoder position (rad, calibrated)
float32 velocity        # current velocity (rad/s)
float32 torque          # current torque (Nm)
float32 temperature     # motor temp (°C)
uint8 fault_code        # RS04 fault flags
uint8 mode_status       # 0=Reset, 1=Calibration, 2=Run
