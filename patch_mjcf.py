import xml.etree.ElementTree as ET
import os

def set_actuators(tree):
    root = tree.getroot()
    actuator = root.find('actuator')
    if actuator is not None:
        # We need to replace <motor ...> with <position ...>
        # Let's read from robot_light.mjcf what the actuators look like
        light_tree = ET.parse('mjcf/sim2sim/robot_light.mjcf')
        light_act = light_tree.getroot().find('actuator')
        
        # Clear existing motors from heavy
        for motor in list(actuator.findall('motor')):
            actuator.remove(motor)
            
        # Copy positions from light
        for pos in light_act.findall('position'):
            actuator.append(pos)
            
def fix_heavy_masses(tree):
    heavy_ref = ET.parse('test_heavy.xml').getroot()
    ref_bodies = {b.get('name'): b for b in heavy_ref.iter('body')}
    
    for body in tree.getroot().iter('body'):
        name = body.get('name')
        if name in ref_bodies:
            ref_b = ref_bodies[name]
            ref_in = ref_b.find('inertial')
            if ref_in is not None:
                old_in = body.find('inertial')
                if old_in is not None:
                    old_in.attrib = ref_in.attrib.copy()

def add_missing_masses(tree):
    root_body = None
    for body in tree.getroot().iter('body'):
        if body.get('name') == 'root':
            root_body = body
            break
            
    if root_body is not None:
        # Remove existing fake inertials if any
        for inert in list(root_body.findall('inertial')):
            root_body.remove(inert)
            
        # Add correct torso inertial
        torso_in = ET.Element('inertial', {
            'pos': "-0.0003038 0.000275941 -0.0780049",
            'mass': "3.4392",
            'diaginertia': "0.00794758 0.00739254 0.00754132"
        })
        root_body.insert(0, torso_in)
        
        # Add battery_chest body
        # check if it already exists
        has_battery = any(b.get('name') == 'battery_chest' for b in root_body.findall('body'))
        if not has_battery:
            bat_body = ET.Element('body', {'name': 'battery_chest', 'pos': "-0.0004 0.0003 0.0436"})
            ET.SubElement(bat_body, 'inertial', {
                'pos': "0 0 0",
                'mass': "0.5",
                'diaginertia': "0.000992 0.000766 0.00155"
            })
            root_body.append(bat_body)

# Patch Heavy
h_tree = ET.parse('mjcf/sim2sim/robot_heavy.mjcf')
set_actuators(h_tree)
fix_heavy_masses(h_tree)
add_missing_masses(h_tree)
# ET.indent(h_tree, space="  ", level=0)  # Needs python 3.9+
h_tree.write('mjcf/sim2sim/robot_heavy.mjcf', encoding='utf-8', xml_declaration=False)

# Patch Light
l_tree = ET.parse('mjcf/sim2sim/robot_light.mjcf')
add_missing_masses(l_tree)
l_tree.write('mjcf/sim2sim/robot_light.mjcf', encoding='utf-8', xml_declaration=False)

