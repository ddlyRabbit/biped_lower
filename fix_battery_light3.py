import xml.etree.ElementTree as ET

def fix_all(file_path):
    tree = ET.parse(file_path)
    worldbody = tree.getroot().find('worldbody')
    
    root_body = None
    for child in list(worldbody):
        if child.tag == 'body' and child.get('name') == 'root':
            root_body = child
            break
            
    # Find any battery_chest in worldbody
    batt_nodes = []
    for child in list(worldbody):
        if child.tag == 'body' and child.get('name') == 'battery_chest':
            batt_nodes.append(child)
            worldbody.remove(child)
            
    if root_body is not None:
        for bn in batt_nodes:
            root_body.append(bn)

    tree.write(file_path, encoding='utf-8', xml_declaration=False)

fix_all('mjcf/sim2sim/robot_light.mjcf')
fix_all('mjcf/sim2sim/robot_heavy.mjcf')
