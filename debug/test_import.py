import argparse, sys
sys.argv = [sys.argv[0], "--headless"]
parser = argparse.ArgumentParser()
from isaaclab.app import AppLauncher
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

sys.path.insert(0, "/workspace/biped_locomotion")

results = []
try:
    from biped_env_cfg import BIPED_CFG, BipedFlatEnvCfg
    results.append("ENV_CONFIG_LOADED")
    results.append(f"actuators={list(BIPED_CFG.actuators.keys())}")
    cfg = BipedFlatEnvCfg()
    results.append(f"num_envs={cfg.scene.num_envs}")
    results.append(f"terrain={cfg.scene.terrain.terrain_type}")
    results.append("ALL_GOOD")
except Exception as e:
    results.append(f"ERROR: {e}")
    import traceback
    results.append(traceback.format_exc())

with open("/results/import_test_result.txt", "w") as f:
    f.write("\n".join(results))

app.close()
