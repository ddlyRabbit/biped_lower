import re

with open("/home/abhinavroy/biped_lower/biped_env_cfg.py", "r") as f:
    content = f.read()

# 1. Update air time impact thresholds
content = re.sub(
    r'("threshold_min": )0\.15,',
    r'\g<1>0.1,', content
)
content = re.sub(
    r'("threshold_max": )0\.35,',
    r'\g<1>0.2,', content
)

# 2. Ensure slide penalty is -0.25 (it should already be, but enforcing)
content = re.sub(
    r'(feet_slide = RewTerm\([\s\S]*?weight=)-[0-9.]+,',
    r'\g<1>-0.25,', content
)

with open("/home/abhinavroy/biped_lower/biped_env_cfg.py", "w") as f:
    f.write(content)
