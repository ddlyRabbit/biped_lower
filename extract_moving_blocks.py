import pandas as pd
import numpy as np

# Load raw right-side data
print("Loading raw data...")
df = pd.read_csv('actuator_net/sysid_data_right.csv')

# The 6 right-side joints
joints = [
    'R_hip_pitch', 'R_hip_roll', 'R_hip_yaw', 
    'R_knee', 'R_foot_pitch', 'R_foot_roll'
]

# We expect ~50Hz data. A chirp is 60 seconds -> 3000 samples.
# We'll take a slightly generous window, say 3100 samples.
WINDOW_SAMPLES = 3100 
output_dfs = []

for joint in joints:
    target_col = f'{joint}_target'
    pos_col = f'{joint}_pos'
    vel_col = f'{joint}_vel'
    tau_col = f'{joint}_tau'
    
    if target_col not in df.columns:
        print(f"Missing target column for {joint}")
        continue
        
    # Calculate rolling standard deviation of the target to find the chirp
    # Use a 100-sample (2 sec) window to detect sustained movement
    rolling_std = df[target_col].rolling(window=100, center=True).std()
    
    # Find the index where movement clearly starts
    moving_indices = rolling_std[rolling_std > 0.005].index
    
    if len(moving_indices) == 0:
        print(f"No movement detected for {joint}")
        continue
        
    start_idx = moving_indices[0]
    
    # Since rolling is centered, the actual start is a bit earlier. Back up ~50 samples.
    start_idx = max(0, start_idx - 50)
    end_idx = min(len(df), start_idx + WINDOW_SAMPLES)
    
    print(f"{joint}: Movement found from {start_idx} to {end_idx}. Length: {end_idx - start_idx}")
    
    # Extract this slice
    block = df.iloc[start_idx:end_idx].copy()
    
    # Only keep the columns for THIS joint + time
    cols_to_keep = ['time', target_col, pos_col, vel_col, tau_col]
    
    # If any column is missing, skip
    if not all(c in block.columns for c in cols_to_keep):
        print(f"Missing columns for {joint}")
        continue
        
    block = block[cols_to_keep].dropna()
    
    # Rename columns to standard names so we can stack them vertically
    # The network is trained generically on (target, pos, vel, tau)
    block.columns = ['time', 'cmd_pos', 'pos', 'vel', 'tau']
    block['joint_name'] = joint  # Add label
    
    output_dfs.append(block)

if output_dfs:
    final_df = pd.concat(output_dfs, ignore_index=True)
    # Sort by time just in case, though it doesn't strictly matter for vertically stacked independent blocks
    # Actually, keep blocks contiguous:
    out_path = 'actuator_net/sysid_data_right_moving_only.csv'
    final_df.to_csv(out_path, index=False)
    print(f"\nSuccessfully saved {len(final_df)} rows to {out_path}")
else:
    print("\nNo data extracted.")
