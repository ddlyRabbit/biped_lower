with open('biped_env_cfg.py', 'r') as f:
    content = f.read()

# Change threshold_min
content = content.replace('"threshold_min": 0.2', '"threshold_min": 0.15')

# Change threshold_max
content = content.replace('"threshold_max": 0.5', '"threshold_max": 0.6')

# Change weight for feet_air_time
import re
content = re.sub(r'(feet_air_time = RewTerm\(\s*func="biped_env_cfg:feet_air_time_berkeley",\s*weight=)10.0', r'\g<1>7.5', content)

with open('biped_env_cfg.py', 'w') as f:
    f.write(content)

print("Config updated.")
