import xml.etree.ElementTree as ET

def check(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    worldbody = root.find('worldbody')
    root_body = worldbody.find('body')
    
    print(f"File: {file_path}")
    print(f"Worldbody children: {[b.get('name') for b in worldbody.findall('body')]}")
    if root_body is not None:
        print(f"Root body children: {[b.get('name') for b in root_body.findall('body')]}")

check('mjcf/sim2sim/robot_light.mjcf')
check('mjcf/sim2sim/robot_heavy.mjcf')
