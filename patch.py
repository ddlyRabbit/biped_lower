import re

# 1. Update biped_env_cfg.py
cfg_path = 'biped_env_cfg.py'
with open(cfg_path, 'r') as f:
    text = f.read()

# Replace foot_pitch
text = re.sub(
    r'(\"foot_pitch\": ImplicitActuatorCfg\([\s\S]*?)stiffness=120\.0, damping=3\.0',
    r'\g<1>stiffness=30.0, damping=1.0',
    text
)

# Replace foot_roll
text = re.sub(
    r'(\"foot_roll\": ImplicitActuatorCfg\([\s\S]*?)stiffness=120\.0, damping=3\.0',
    r'\g<1>stiffness=30.0, damping=1.0',
    text
)

with open(cfg_path, 'w') as f:
    f.write(text)

# 2. Update obs_builder.py
obs_path = 'deploy/biped_ws/src/biped_control/biped_control/obs_builder.py'
with open(obs_path, 'r') as f:
    text = f.read()

text = re.sub(
    r'(\"L_foot_pitch\":\s*)\(120\.0,\s*3\.0\)(,\s*\"R_foot_pitch\":\s*)\(120\.0,\s*3\.0\)',
    r'\g<1>(30.0, 1.0)\g<2>(30.0, 1.0)',
    text
)

text = re.sub(
    r'(\"L_foot_roll\":\s*)\(120\.0,\s*3\.0\)(,\s*\"R_foot_roll\":\s*)\(120\.0,\s*3\.0\)',
    r'\g<1>(30.0, 1.0)\g<2>(30.0, 1.0)',
    text
)

with open(obs_path, 'w') as f:
    f.write(text)
