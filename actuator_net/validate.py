import torch
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
from dataset_flat import ActuatorDatasetFlattened
from model import ActuatorNet
import os

def validate_model(csv_file, joint_type, k_history=6):
    print(f"Validating {joint_type} model...")
    
    # Load dataset
    dataset = ActuatorDatasetFlattened(csv_file, joint_type, k_history=k_history)
    
    # Load JIT compiled model
    model_path = f"{joint_type}_net.pt"
    if not os.path.exists(model_path):
        print(f"Model not found: {model_path}")
        return
        
    model = torch.jit.load(model_path)
    model.eval()
    
    y_true = []
    y_pred = []
    
    with torch.no_grad():
        for i in range(len(dataset)):
            x, y = dataset[i]
            x = x.unsqueeze(0)  # Add batch dimension
            pred = model(x)
            
            y_true.append(y.item())
            y_pred.append(pred.item())
            
    y_true = np.array(y_true)
    y_pred = np.array(y_pred)
    
    mse = np.mean((y_true - y_pred) ** 2)
    rmse = np.sqrt(mse)
    
    print(f"RMSE for {joint_type}: {rmse:.4f} Nm")
    
    # Plot first 1000 samples for visual inspection
    plt.figure(figsize=(10, 5))
    plot_len = min(1000, len(y_true))
    plt.plot(y_true[:plot_len], label="Ground Truth Torque", alpha=0.8)
    plt.plot(y_pred[:plot_len], label="Predicted Torque", alpha=0.8, linestyle='--')
    plt.title(f"ActuatorNet Validation: {joint_type} (RMSE: {rmse:.4f})")
    plt.xlabel("Timestep (20ms)")
    plt.ylabel("Torque (Nm)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"val_{joint_type}.png")
    print(f"Saved plot to val_{joint_type}.png\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default="sysid_data_right_moving_only.csv")
    args = parser.parse_args()
    
    joints = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "foot_pitch", "foot_roll"]
    for j in joints:
        validate_model(args.csv, j)
