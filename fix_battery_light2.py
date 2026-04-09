import xml.etree.ElementTree as ET

def fix_battery(file_path):
    tree = ET.parse(file_path)
    root_el = tree.getroot()
    worldbody = root_el.find('worldbody')
    
    if worldbody is not None:
        root_body = None
        for b in list(worldbody.findall('body')):
            if b.get('name') == 'root':
                root_body = b
                break
        
        # Check if battery_chest is a direct child of worldbody
        batt_body = None
        for b in list(worldbody.findall('body')):
            if b.get('name') == 'battery_chest':
                batt_body = b
                worldbody.remove(b)
                break
                
        if root_body is not None and batt_body is not None:
            root_body.append(batt_body)

    tree.write(file_path, encoding='utf-8', xml_declaration=False)

fix_battery('mjcf/sim2sim/robot_light.mjcf')
fix_battery('mjcf/sim2sim/robot_heavy.mjcf')
