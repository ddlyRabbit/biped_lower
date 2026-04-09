import xml.etree.ElementTree as ET

def set_root_mass(tree, mass, pos, diaginertia):
    root_body = None
    for body in tree.getroot().iter('body'):
        if body.get('name') == 'root':
            root_body = body
            break
            
    if root_body is not None:
        # Remove existing fake inertials
        for inert in list(root_body.findall('inertial')):
            root_body.remove(inert)
            
        torso_in = ET.Element('inertial', {
            'pos': pos,
            'mass': mass,
            'diaginertia': diaginertia
        })
        root_body.insert(0, torso_in)

h_tree = ET.parse('mjcf/sim2sim/robot_heavy.mjcf')
set_root_mass(h_tree, "4.31925", "0.000537258 0.000627404 -0.0648322", "0.0113947 0.011234 0.0105519")
h_tree.write('mjcf/sim2sim/robot_heavy.mjcf', encoding='utf-8', xml_declaration=False)

l_tree = ET.parse('mjcf/sim2sim/robot_light.mjcf')
set_root_mass(l_tree, "3.4392", "-0.0003038 0.000275941 -0.0780049", "0.00794758 0.00739254 0.00754132")
l_tree.write('mjcf/sim2sim/robot_light.mjcf', encoding='utf-8', xml_declaration=False)

