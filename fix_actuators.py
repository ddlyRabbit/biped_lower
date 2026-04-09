import xml.etree.ElementTree as ET

def fix(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    actuator = root.find('actuator')
    if actuator is not None:
        for pos in list(actuator.findall('position')):
            # We must revert <position> to <motor> because play_mujoco.py manually calculates torques and sets data.ctrl
            # If we use <position>, MuJoCo applies: force = Kp*(ctrl - pos) - Kv*vel.
            # But the script also does: ctrl = Kp*(target - pos) - Kv*vel
            # So the force becomes: Kp * ( Kp*(target-pos) - Kv*vel - pos ) - Kv*vel -> MASSIVE INSTABILITY!
            
            # To fix, we revert to <motor ... gear="1">
            name = pos.get('name')
            joint = pos.get('joint')
            forcerange = pos.get('forcerange')
            
            new_motor = ET.Element('motor', {
                'name': name,
                'joint': joint,
                'ctrllimited': "true",
                'ctrlrange': forcerange,
                'gear': "1"
            })
            actuator.insert(list(actuator).index(pos), new_motor)
            actuator.remove(pos)

    tree.write(file_path, encoding='utf-8', xml_declaration=False)

fix('mjcf/sim2sim/robot_light.mjcf')
fix('mjcf/sim2sim/robot_heavy.mjcf')
