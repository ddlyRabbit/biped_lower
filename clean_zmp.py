with open("deploy/biped_ws/src/biped_teleop/biped_teleop/keyboard_teleop.py", "r") as f:
    text = f.read()
import re
text = re.sub(r'.*WALK_SIM_ZMP.*\n', '', text)
text = re.sub(r'.*WALK_ZMP.*\n', '', text)
with open("deploy/biped_ws/src/biped_teleop/biped_teleop/keyboard_teleop.py", "w") as f:
    f.write(text)

with open("deploy/biped_ws/src/biped_control/biped_control/state_machine_node.py", "r") as f:
    text = f.read()

text = re.sub(r'        elif cmd == "WALK_ZMP" and self._state in \("STAND", "WALK_SIM_ZMP"\):\n            self._transition\("WALK_ZMP"\)\n', '', text)
text = re.sub(r'        elif cmd == "WALK_SIM_ZMP" and self._state == "STAND":\n            self._transition\("WALK_SIM_ZMP"\)\n', '', text)
text = text.replace(' "WALK_ZMP", "WALK_SIM_ZMP",', '')
text = re.sub(r'        elif self._state == "WALK_ZMP":\n            # ZMP trajectory node publishes /joint_commands — don\'t interfere\n            pass\n', '', text)
text = re.sub(r'        elif self._state == "WALK_SIM_ZMP":\n            # ZMP plays on /policy_viz_joints, motors hold STAND\n            self._handle_stand_hold\(\)\n', '', text)
text = text.replace(' / WALK_ZMP / WALK_SIM_ZMP', '')

with open("deploy/biped_ws/src/biped_control/biped_control/state_machine_node.py", "w") as f:
    f.write(text)
