import os

filepath = 'deploy/biped_ws/src/biped_driver_cpp/src/robstride_bus.cpp'
with open(filepath, 'r') as f:
    content = f.read()

# Add logging
content = content.replace(
    '    trq = clamp(trq, -scale.torque, scale.torque);',
    '    trq = clamp(trq, -scale.torque, scale.torque);\n    RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 1000, "Name: %s, pos: %.3f, Vel: %.3f, KP: %.1f, KD: %.1f, TRQ: %.1f", name.c_str(), pos, vel, kp, kd, trq);'
)

with open(filepath, 'w') as f:
    f.write(content)
print('Logging patched.')
