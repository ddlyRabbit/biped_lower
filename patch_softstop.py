import os

filepath = 'deploy/biped_ws/src/biped_driver_cpp/include/biped_driver_cpp/motor_manager.hpp'
with open(filepath, 'r') as f:
    content = f.read()

content = content.replace('constexpr double SOFTSTOP_KP = 20.0;  // Nm/rad restoring spring', 'constexpr double SOFTSTOP_KP = 0.0;  // Nm/rad restoring spring')

with open(filepath, 'w') as f:
    f.write(content)

filepath_mm = 'deploy/biped_ws/src/biped_driver_cpp/src/motor_manager.cpp'
with open(filepath_mm, 'r') as f:
    content_mm = f.read()

content_mm = content_mm.replace('constexpr double pitch_min = -0.87267;', 'constexpr double pitch_min = -0.7267;')
content_mm = content_mm.replace('constexpr double pitch_max =  0.52360;', 'constexpr double pitch_max =  0.42360;')
content_mm = content_mm.replace('constexpr double roll_min  = -0.26180;', 'constexpr double roll_min  = -0.20180;')
content_mm = content_mm.replace('constexpr double roll_max  =  0.26180;', 'constexpr double roll_max  =  0.20180;')

with open(filepath_mm, 'w') as f:
    f.write(content_mm)

print('Softstop and limits patched.')
