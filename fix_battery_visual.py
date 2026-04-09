import xml.etree.ElementTree as ET

def fix(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    worldbody = root.find('worldbody')
    root_body = None
    for b in list(worldbody.findall('body')):
        if b.get('name') == 'root':
            root_body = b
            break
            
    if root_body is not None:
        battery_chest = None
        for b in list(root_body.findall('body')):
            if b.get('name') == 'battery_chest':
                battery_chest = b
                break
                
        if battery_chest is not None:
            # Check if it already has geoms
            has_geom = any(g.tag == 'geom' for g in battery_chest)
            if not has_geom:
                # Add the visual and collision geoms
                # <geom size="0.063 0.073 0.025" pos="0 0 0" type="box" rgba="0.2 0.2 0.2 1" contype="1" conaffinity="0" density="0" group="1" class="visualgeom" />
                # <geom type="box" rgba="0.2 0.2 0.2 1" size="0.063 0.073 0.025" pos="0 0 0" contype="0" conaffinity="0" />
                ET.SubElement(battery_chest, 'geom', {
                    'size': "0.063 0.073 0.025",
                    'pos': "0 0 0",
                    'type': "box",
                    'rgba': "0.2 0.2 0.2 1",
                    'contype': "1",
                    'conaffinity': "0",
                    'density': "0",
                    'group': "1",
                    'class': "visualgeom"
                })
                ET.SubElement(battery_chest, 'geom', {
                    'type': "box",
                    'rgba': "0.2 0.2 0.2 1",
                    'size': "0.063 0.073 0.025",
                    'pos': "0 0 0",
                    'contype': "0",
                    'conaffinity': "0"
                })
                
        # the heavy model had the geoms attached to the root body directly, so we should clean that up
        for g in list(root_body.findall('geom')):
            if g.get('size') == '0.063 0.073 0.025' or g.get('pos') == '-0.0004 0.0003 0.0436':
                if g.get('type') == 'box':
                    root_body.remove(g)

    tree.write(file_path, encoding='utf-8', xml_declaration=False)

fix('mjcf/sim2sim/robot_light.mjcf')
fix('mjcf/sim2sim/robot_heavy.mjcf')
