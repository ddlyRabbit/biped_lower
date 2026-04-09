import re

with open('biped_play_rsl.py', 'r') as f:
    text = f.read()

old_layers = """    # Build actor MLP: [512, 256, 128]
    actor_layers = [
        nn.Linear(num_obs, 128), nn.ELU(),
        nn.Linear(128, 128), nn.ELU(),
        nn.Linear(128, 128), nn.ELU(),
        nn.Linear(128, num_actions),
    ]"""

new_layers = """    # Build actor MLP: [512, 256, 128]
    actor_layers = [
        nn.Linear(num_obs, 512), nn.ELU(),
        nn.Linear(512, 256), nn.ELU(),
        nn.Linear(256, 128), nn.ELU(),
        nn.Linear(128, num_actions),
    ]"""

text = text.replace(old_layers, new_layers)

with open('biped_play_rsl.py', 'w') as f:
    f.write(text)
print("patched")
