import re

with open("/home/abhinavroy/biped_lower/biped_env_cfg.py", "r") as f:
    content = f.read()

# Replace all min_delay=25, max_delay=30 with min_delay=5, max_delay=10
content = content.replace("min_delay=25, max_delay=30", "min_delay=5, max_delay=10")

with open("/home/abhinavroy/biped_lower/biped_env_cfg.py", "w") as f:
    f.write(content)

with open("/home/abhinavroy/biped_lower/biped_train_rsl.py", "r") as f:
    train_content = f.read()

train_content = train_content.replace('experiment = "biped_flat_v131"', 'experiment = "biped_flat_v132"')

with open("/home/abhinavroy/biped_lower/biped_train_rsl.py", "w") as f:
    f.write(train_content)
