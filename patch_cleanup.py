import re

with open('sim2sim/play_mujoco.py', 'r') as f:
    text = f.read()

# Replace the hardcoded CSV logging setup
old_csv_setup = """    import csv
    log_file = open('/results/videos/sim2sim_log.csv', 'w', newline='')
    csv_writer = csv.writer(log_file)
    header = ['time'] + [f'cmd_{j}' for j in ISAAC_JOINTS] + [f'pos_{j}' for j in ISAAC_JOINTS] + [f'act_{j}' for j in ISAAC_JOINTS]
    csv_writer.writerow(header)"""

new_csv_setup = """    csv_writer = None
    if args.video:
        import csv
        csv_path = args.video.replace('.mp4', '.csv')
        log_file = open(csv_path, 'w', newline='')
        csv_writer = csv.writer(log_file)
        header = ['time'] + [f'cmd_{j}' for j in ISAAC_JOINTS] + [f'pos_{j}' for j in ISAAC_JOINTS] + [f'act_{j}' for j in ACTION_ORDER]
        csv_writer.writerow(header)"""
text = text.replace(old_csv_setup, new_csv_setup)

old_csv_write = """                        # Save to CSV
            row = [step * POLICY_DT] + targets_isaac.tolist() + data.qpos[qp_idx][ISAAC_TO_MJ_IDX].tolist() + actions_isaac.tolist()
            csv_writer.writerow(row)"""

new_csv_write = """            if csv_writer is not None:
                row = [step * POLICY_DT] + targets_isaac.tolist() + data.qpos[qp_idx][ISAAC_TO_MJ_IDX].tolist() + actions_isaac.tolist()
                csv_writer.writerow(row)"""

text = text.replace(old_csv_write, new_csv_write)

with open('sim2sim/play_mujoco.py', 'w') as f:
    f.write(text)
print("cleaned up")
