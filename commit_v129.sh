sed -i 's/"R_foot_pitch": 0.3,/"R_foot_pitch": 0.5,/' deploy/biped_ws/src/biped_control/biped_control/obs_builder.py
sed -i 's/"L_foot_pitch": 0.3,/"L_foot_pitch": 0.5,/' deploy/biped_ws/src/biped_control/biped_control/obs_builder.py
sed -i 's/scale = 0.3f;/scale = 0.5f;/' deploy/biped_ws/src/biped_control_cpp/src/obs_builder.cpp
