#!/usr/bin/env python3
"""Build biped MuJoCo MJCF from URDF.

Usage:
    python sim2sim/build_mjcf.py --urdf light   # default
    python sim2sim/build_mjcf.py --urdf heavy
"""
import argparse
import mujoco
import re
import shutil
import os
import tempfile


def build(urdf_variant: str = "light"):
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf_dir = os.path.join(repo_root, "urdf", urdf_variant)
    mjcf_dir = os.path.join(repo_root, "mjcf")
    os.makedirs(mjcf_dir, exist_ok=True)

    # --- Convert URDF via MuJoCo (needs flat mesh paths) ---
    tmpdir = tempfile.mkdtemp()
    with open(os.path.join(urdf_dir, "robot.urdf")) as f:
        urdf = f.read()
    urdf = urdf.replace("package://assets/", "assets/")
    with open(os.path.join(tmpdir, "robot.urdf"), "w") as f:
        f.write(urdf)
    for stl in os.listdir(os.path.join(urdf_dir, "assets")):
        if stl.endswith(".stl"):
            shutil.copy(os.path.join(urdf_dir, "assets", stl), tmpdir)

    model = mujoco.MjModel.from_xml_path(os.path.join(tmpdir, "robot.urdf"))
    xml_path = os.path.join(tmpdir, "robot.xml")
    mujoco.mj_saveLastXML(xml_path, model)

    with open(xml_path) as f:
        xml = f.read()
    shutil.rmtree(tmpdir)

    # --- Fix mesh paths ---
    xml = re.sub(r'file="([^"]*\.stl)"', rf'file="../urdf/{urdf_variant}/assets/\1"', xml)

    # --- Visual-only meshes ---
    xml = re.sub(
        r'(<geom\b(?![^>]*contype)[^>]*type="mesh")',
        r'\1 contype="0" conaffinity="0" group="2"',
        xml,
    )

    # --- Wrap worldbody in base body with freejoint ---
    wb_match = re.search(r"<worldbody>(.*?)</worldbody>", xml, re.DOTALL)
    wb_content = wb_match.group(1)
    first_body_idx = wb_content.find("<body name=")
    torso_geoms = wb_content[:first_body_idx]
    child_bodies = wb_content[first_body_idx:]

    new_wb = f"""<worldbody>
    <body name="base" pos="0 0 0.80" quat="1 0 0 0">
      <freejoint name="base_freejoint"/>
      <site name="imu_site" pos="0 0 0.064" size="0.01"/>
      <inertial pos="0 0 0.064" mass="5.0" diaginertia="0.05 0.04 0.02"/>
{torso_geoms}
{child_bodies}
    </body>
  </worldbody>"""

    xml = xml[: wb_match.start()] + new_wb + xml[wb_match.end() :]

    # --- Add collision primitives ---
    # Feet: body-local X→world -Z, Y→world +X, Z→world -Y
    # Use quat to orient box in world frame (flat on ground)
    # size: (0.015=height, 0.08=forward, 0.04=lateral) in world, mapped to body frame
    # Box in body frame: size="0.015 0.08 0.04" with quat="0.5 0.5 0.5 -0.5"
    # pos offset: body origin is ~0.044m above ground; box center should be at ~0.015m
    # offset_world=[0,0,-0.029] → offset_body=[0, 0.029, 0] via R_bfw
    for foot_body, coll_name in [("foot_6061", "right_foot_collision"), ("foot_6061_2", "left_foot_collision")]:
        old = f'<body name="{foot_body}" pos="0 0.03 -0.03" quat="0.499998 -0.5 -0.5 -0.500002">'
        new = (
            old + "\n"
            f'                <geom name="{coll_name}" type="box" size="0.015 0.08 0.04" '
            f'pos="0 0.029 0" quat="0.5 0.5 0.5 -0.5" '
            f'contype="1" conaffinity="1" group="3" rgba="0 1 0 0.3" friction="1.0 0.005 0.0001"/>'
        )
        xml = xml.replace(old, new)

    # Shins: body frames are also rotated, use capsule along the shin length
    # Right shin body "kd_d_401r_6061" has quat="0.707 0 0.707 0"
    # Left shin body "kd_d_401l_6061" has quat="0.707 0 -0.707 0"
    for shin_body, coll_name, fromto in [
        ("kd_d_401r_6061", "right_shin_collision", "0 0 0 0.15 -0.03 0"),
        ("kd_d_401l_6061", "left_shin_collision", "0 0 0 -0.15 0.03 0"),
    ]:
        pattern = re.compile(rf'(<body name="{shin_body}"[^>]*>)')
        match = pattern.search(xml)
        if match:
            xml = xml[: match.end()] + (
                f'\n            <geom name="{coll_name}" type="capsule" size="0.04" '
                f'fromto="{fromto}" contype="1" conaffinity="1" group="3" rgba="0 0 1 0.2"/>'
            ) + xml[match.end() :]

    out_path = os.path.join(mjcf_dir, "biped.xml")
    with open(out_path, "w") as f:
        f.write(xml)

    # --- Verify ---
    model2 = mujoco.MjModel.from_xml_path(os.path.join(mjcf_dir, "biped_scene.xml"))
    total_mass = sum(model2.body_mass[i] for i in range(model2.nbody))
    print(f"Built mjcf/biped.xml from urdf/{urdf_variant}/")
    print(f"  Bodies={model2.nbody}, Joints={model2.njnt}, Actuators={model2.nu}, Mass={total_mass:.1f}kg")
    return out_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", choices=["light", "heavy"], default="light")
    args = parser.parse_args()
    build(args.urdf)
