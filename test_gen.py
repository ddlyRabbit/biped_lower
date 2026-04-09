import mujoco
import tempfile
import os
import shutil

def gen(urdf_path, out_xml):
    tmpdir = tempfile.mkdtemp()
    urdf_dir = os.path.dirname(urdf_path)
    with open(urdf_path) as f:
        urdf = f.read()
    urdf = urdf.replace("package://assets/", "")
    with open(os.path.join(tmpdir, "robot.urdf"), "w") as f:
        f.write(urdf)
    
    for stl in os.listdir(os.path.join(urdf_dir, "assets")):
        if stl.endswith(".stl"):
            shutil.copy(os.path.join(urdf_dir, "assets", stl), tmpdir)
    
    try:
        model = mujoco.MjModel.from_xml_path(os.path.join(tmpdir, "robot.urdf"))
        mujoco.mj_saveLastXML(out_xml, model)
    except Exception as e:
        print(f"Failed {urdf_path}: {e}")
    shutil.rmtree(tmpdir)

gen('urdf/heavy/robot.urdf', 'test_heavy.xml')
gen('urdf/light/robot.urdf', 'test_light.xml')
