import xml.dom.minidom

def format_mjcf(file_path):
    with open(file_path, 'r') as f:
        xml_str = f.read()
    
    # Very basic hack to insert newlines after motor tags since they got collapsed into 1 line
    xml_str = xml_str.replace('gear="1" /><motor', 'gear="1" />\n    <motor')
    xml_str = xml_str.replace('gear="1" /></actuator>', 'gear="1" />\n  </actuator>')
    
    with open(file_path, 'w') as f:
        f.write(xml_str)

format_mjcf('mjcf/sim2sim/robot_light.mjcf')
format_mjcf('mjcf/sim2sim/robot_heavy.mjcf')
