import xml.etree.ElementTree as ET
import sys

def parse_urdf(filepath):
    tree = ET.parse(filepath)
    root = tree.getroot()
    links = {}
    for link in root.findall('link'):
        name = link.get('name')
        inertial = link.find('inertial')
        if inertial is not None:
            mass = float(inertial.find('mass').get('value'))
            origin = inertial.find('origin')
            pos = [float(x) for x in origin.get('xyz').split()] if origin is not None and origin.get('xyz') else [0,0,0]
            links[name] = {'mass': mass, 'pos': pos}
    joints = {}
    for joint in root.findall('joint'):
        name = joint.get('name')
        axis = joint.find('axis')
        origin = joint.find('origin')
        limit = joint.find('limit')
        if axis is not None:
            axis_val = [float(x) for x in axis.get('xyz').split()]
            joints[name] = {'axis': axis_val}
    return links, joints

def parse_mjcf(filepath):
    tree = ET.parse(filepath)
    root = tree.getroot()
    bodies = {}
    joints = {}
    def recurse(node):
        for child in node:
            if child.tag == 'body':
                name = child.get('name')
                inertial = child.find('inertial')
                if inertial is not None:
                    mass = float(inertial.get('mass'))
                    pos = [float(x) for x in inertial.get('pos').split()]
                    bodies[name] = {'mass': mass, 'pos': pos}
            if child.tag == 'joint':
                name = child.get('name')
                axis_str = child.get('axis')
                if axis_str:
                    axis_val = [float(x) for x in axis_str.split()]
                    joints[name] = {'axis': axis_val}
            recurse(child)
    recurse(root)
    return bodies, joints

def compare(urdf_path, mjcf_path):
    print(f"Comparing {urdf_path} vs {mjcf_path}")
    u_links, u_joints = parse_urdf(urdf_path)
    m_bodies, m_joints = parse_mjcf(mjcf_path)
    
    issues = []
    for name, u_data in u_links.items():
        if name not in m_bodies:
            issues.append(f"Body '{name}' missing in MJCF")
            continue
        m_data = m_bodies[name]
        if abs(u_data['mass'] - m_data['mass']) > 1e-4:
            issues.append(f"Body '{name}' mass mismatch: URDF {u_data['mass']}, MJCF {m_data['mass']}")
            
    for name, u_data in u_joints.items():
        if name not in m_joints:
            if name not in ['base_link_to_battery', 'base_link_joint', 'imu_joint', 'base_to_imu', 'battery_joint']: # ignore fixed joints
                issues.append(f"Joint '{name}' missing in MJCF")
            continue
        m_data = m_joints[name]
        if u_data['axis'] != m_data['axis']:
            issues.append(f"Joint '{name}' axis mismatch: URDF {u_data['axis']}, MJCF {m_data['axis']}")

    return issues

print("--- HEAVY ---")
heavy_issues = compare('urdf/heavy/robot.urdf', 'mjcf/sim2sim/robot_heavy.mjcf')
for i in heavy_issues: print(i)

print("--- LIGHT ---")
light_issues = compare('urdf/light/robot.urdf', 'mjcf/sim2sim/robot_light.mjcf')
for i in light_issues: print(i)

