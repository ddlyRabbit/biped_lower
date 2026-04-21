import os

filepath = 'deploy/biped_ws/src/biped_control/biped_control/policy_node.py'
with open(filepath, 'r') as f:
    content = f.read()

content = content.replace('WALK_GAIN_RAMP_SECS = 5.0        # ramp gains from 10% → 100% on WALK entry', 'WALK_GAIN_RAMP_SECS = 0.1        # ramp gains from 10% → 100% on WALK entry')

with open(filepath, 'w') as f:
    f.write(content)

print('Policy ramp patched.')
