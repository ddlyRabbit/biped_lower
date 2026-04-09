import xml.etree.ElementTree as ET
import re

def fix(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
    
    # regex to find the battery chest
    battery_pattern = r'<body name="battery_chest"[^>]*>.*?</body>'
    
    match = re.search(battery_pattern, content, flags=re.DOTALL)
    if match:
        battery_text = match.group(0)
        content = content.replace(battery_text, "")
        
        # Now find the end of the root body which is right before </worldbody>
        # (Assuming the root body is the last thing inside worldbody before </worldbody>)
        # Actually better:
        content = content.replace("</worldbody>", battery_text + "\n  </worldbody>")
        
    with open(file_path, 'w') as f:
        f.write(content)

fix('mjcf/sim2sim/robot_light.mjcf')
fix('mjcf/sim2sim/robot_heavy.mjcf')
