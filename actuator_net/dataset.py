import torch
from torch.utils.data import Dataset
import pandas as pd
import numpy as np

class ActuatorDataset(Dataset):
    def __init__(self, csv_files, joint_type, k_history=6):
        """
        csv_files: list of paths to generated CSVs (e.g., ['air.csv', 'ground.csv'])
        joint_type: string, e.g. "knee", "hip_pitch", "foot_roll"
        k_history: number of past steps (e.g. 6 steps = 120ms at 50Hz)
        """
        if isinstance(csv_files, str):
            csv_files = [csv_files]
            
        self.X = []
        self.Y = []
        
        for csv_file in csv_files:
            print(f"Processing {csv_file}...")
            df = pd.read_csv(csv_file).dropna()
            
            # Combine both Left and Right side data into a single dataset
            sides = ["L", "R"]
            for side in sides:
                jname = f"{side}_{joint_type}"
                
                # Check if columns exist in CSV
                if f"{jname}_target" not in df.columns:
                    print(f"Warning: {jname}_target not found in {csv_file}.")
                    continue
                    
                # Calculate position error
                q_err = (df[f"{jname}_target"] - df[f"{jname}_pos"]).values
                dq = df[f"{jname}_vel"].values
                tau = df[f"{jname}_tau"].values
                
                # Create sliding windows
                for i in range(k_history - 1, len(df)):
                    window_q_err = q_err[i - k_history + 1 : i + 1]
                    window_dq = dq[i - k_history + 1 : i + 1]
                    
                    # Format matches Isaac Lab ActuatorNetMLP input schema exactly
                    x = np.concatenate([window_q_err, window_dq])
                    y = tau[i]
                    
                    self.X.append(x)
                    self.Y.append(y)
                
        if len(self.X) == 0:
            raise ValueError(f"No data parsed for joint type: {joint_type}")
            
        self.X = torch.tensor(np.array(self.X), dtype=torch.float32)
        self.Y = torch.tensor(np.array(self.Y), dtype=torch.float32).unsqueeze(1)

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]
