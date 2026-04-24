import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
from dataset import ActuatorDataset
from model import ActuatorNet

def train_model(csv_file, joint_type, epochs=100, batch_size=256, lr=1e-3, k_history=6):
    print(f"Loading data for {joint_type} (combining L/R)...")
    dataset = ActuatorDataset(csv_file, joint_type, k_history=k_history)
    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_ds, val_ds = random_split(dataset, [train_size, val_size])

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False)

    model = ActuatorNet(k_history=k_history)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)

    best_val_loss = float('inf')
    
    for epoch in range(epochs):
        model.train()
        train_loss = 0.0
        for x, y in train_loader:
            optimizer.zero_grad()
            y_pred = model(x)
            loss = criterion(y_pred, y)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * len(x)
        train_loss /= train_size
        
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for x, y in val_loader:
                y_pred = model(x)
                loss = criterion(y_pred, y)
                val_loss += loss.item() * len(x)
        val_loss /= val_size
        
        print(f"Epoch {epoch+1}/{epochs} - Train Loss: {train_loss:.4f} | Val Loss: {val_loss:.4f}")
        
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            # Export JIT script for Isaac Lab
            example_input = torch.randn(1, k_history * 2)
            traced_model = torch.jit.trace(model, example_input)
            traced_model.save(f"{joint_type}_net.pt")
            print(f"Saved best model with Val Loss: {best_val_loss:.4f}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, help="Path to sysid_data.csv")
    parser.add_argument("--joint_type", required=True, help="e.g. knee, hip_pitch, foot_roll")
    parser.add_argument("--epochs", type=int, default=100)
    args = parser.parse_args()
    
    train_model(args.csv, args.joint_type, epochs=args.epochs)
