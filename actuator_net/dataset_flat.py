import pandas as pd
import numpy as np
import torch
from torch.utils.data import Dataset

class ActuatorDatasetFlattened(Dataset):
    def __init__(self, csv_file, joint_type, k_history=6):
        """
        csv_file: path to flattened CSV
        joint_type: string, e.g. "knee", "hip_pitch", "foot_roll"
        """
        df = pd.read_csv(csv_file).dropna()
        
        self.X = []
        self.Y = []
        
        # Filter for rows where joint_name contains the joint_type (e.g. "hip_pitch" matches "R_hip_pitch")
        df_joint = df[df['joint_name'].str.contains(joint_type)].copy()
        
        if len(df_joint) == 0:
            raise ValueError(f"No data parsed for joint type: {joint_type}")
            
        print(f"Parsed {len(df_joint)} rows for {joint_type}")
        
        # Ensure we sort by time just to be safe
        df_joint = df_joint.sort_values(by='time').reset_index(drop=True)
            
        # Calculate position error
        # Isaac Lab uses (cmd_pos - pos) for error.
        q_err = (df_joint['cmd_pos'] - df_joint['pos']).values
        dq = df_joint['vel'].values
        tau = df_joint['tau'].values
        
        # Create sliding windows
        for i in range(k_history - 1, len(df_joint)):
            window_q_err = q_err[i - k_history + 1 : i + 1]
            window_dq = dq[i - k_history + 1 : i + 1]
            
            # Format matches Isaac Lab ActuatorNetMLP input schema exactly
            x = np.concatenate([window_q_err, window_dq])
            self.X.append(x)
            self.Y.append([tau[i]])
            
        self.X = torch.tensor(self.X, dtype=torch.float32)
        self.Y = torch.tensor(self.Y, dtype=torch.float32)
        
    def __len__(self):
        return len(self.X)
    
    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]
