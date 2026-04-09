import xml.etree.ElementTree as ET

def fix(file_path):
    tree = ET.parse(file_path)
    worldbody = tree.getroot().find('worldbody')
    
    root_body = None
    for b in list(worldbody.findall('body')):
        if b.get('name') == 'root':
            root_body = b
            break
            
    batt_body = None
    for b in list(worldbody.findall('body')):
        if b.get('name') == 'battery_chest':
            batt_body = b
            worldbody.remove(b)
            break
            
    if root_body is not None and batt_body is not None:
        root_body.append(batt_body)

    tree.write(file_path, encoding='utf-8', xml_declaration=False)

fix('mjcf/sim2sim/robot_light.mjcf')
fix('mjcf/sim2sim/robot_heavy.mjcf')
