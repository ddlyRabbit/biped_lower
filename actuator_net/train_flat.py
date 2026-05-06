import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split, ConcatDataset
from dataset_flat import ActuatorDatasetFlattened
from model import ActuatorNet

def train_model(csv_files, joint_type, epochs=100, batch_size=256, lr=1e-3, k_history=6):
    print(f"Loading data for {joint_type} from {csv_files}...")
    
    datasets = []
    for csv_file in csv_files:
        try:
            ds = ActuatorDatasetFlattened(csv_file, joint_type, k_history=k_history)
            datasets.append(ds)
        except ValueError as e:
            print(f"Skipping {csv_file} for {joint_type}: {e}")
            
    if not datasets:
        print(f"No data available to train {joint_type}")
        return
        
    full_dataset = ConcatDataset(datasets)
    
    train_size = int(0.8 * len(full_dataset))
    val_size = len(full_dataset) - train_size
    train_ds, val_ds = random_split(full_dataset, [train_size, val_size])

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
        
        if epoch % 50 == 0 or epoch == epochs - 1:
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
    parser.add_argument("--csv", nargs='+', required=True, help="List of CSV files")
    parser.add_argument("--joint_type", required=True)
    parser.add_argument("--epochs", type=int, default=100)
    args = parser.parse_args()
    
    train_model(args.csv, args.joint_type, epochs=args.epochs)