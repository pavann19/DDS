"""
LSTM Sequence Predictor for DDS Autopilot
"""
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import numpy as np
import json
import os
import joblib
from sklearn.model_selection import train_test_split

class DrivingLSTM(nn.Module):
    def __init__(self, input_size, hidden_size, num_layers, num_classes):
        super(DrivingLSTM, self).__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        
        # LSTM layer
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        
        # Fully connected layer
        self.fc = nn.Linear(hidden_size, num_classes)
        
    def forward(self, x):
        # Initialize hidden and cell states
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        
        # Forward propagate LSTM
        out, _ = self.lstm(x, (h0, c0))
        
        # Decode the hidden state of the last time step
        out = self.fc(out[:, -1, :])
        return out

def train_lstm(seq_file="sequences.npz", epochs=50, batch_size=32):
    """Trains the LSTM model on sequence data."""
    if not os.path.exists(seq_file):
        print(f"File {seq_file} not found. Run data_prep.py first.")
        return
        
    data = np.load(seq_file)
    X = data['X']
    y = data['y']
    
    # Scale features using the existing scaler, or create a new one
    # Note: X is 3D (samples, window, features). We need to reshape for scaling.
    samples, window, features = X.shape
    X_flat = X.reshape(-1, features)
    
    # Use a separate scaler for LSTM because feature sets might differ
    from sklearn.preprocessing import StandardScaler
    scaler = StandardScaler()
    X_scaled_flat = scaler.fit_transform(X_flat)
    joblib.dump(scaler, "lstm_scaler.pkl")
        
    X_scaled = X_scaled_flat.reshape(samples, window, features)
    
    # Train/val/test split
    X_train, X_temp, y_train, y_temp = train_test_split(X_scaled, y, test_size=0.3, stratify=y, random_state=42)
    X_val, X_test, y_val, y_test = train_test_split(X_temp, y_temp, test_size=0.5, stratify=y_temp, random_state=42)
    
    # Convert to PyTorch tensors
    X_train_t = torch.FloatTensor(X_train)
    y_train_t = torch.LongTensor(y_train)
    X_val_t = torch.FloatTensor(X_val)
    y_val_t = torch.LongTensor(y_val)
    X_test_t = torch.FloatTensor(X_test)
    y_test_t = torch.LongTensor(y_test)
    
    # Create DataLoaders
    train_dataset = TensorDataset(X_train_t, y_train_t)
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    
    # Model parameters
    input_size = features
    hidden_size = 64
    num_layers = 2
    num_classes = len(np.unique(y))
    
    # Initialize model
    model = DrivingLSTM(input_size, hidden_size, num_layers, num_classes)
    
    # Device configuration
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = model.to(device)
    
    # Handle class imbalance with weighted loss
    class_counts = np.bincount(y_train)
    weights = 1.0 / torch.FloatTensor(class_counts)
    weights = weights / weights.sum() * num_classes
    criterion = nn.CrossEntropyLoss(weight=weights.to(device))
    
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    
    # Training loop
    print(f"Training LSTM on {device} for {epochs} epochs...")
    best_val_loss = float('inf')
    
    for epoch in range(epochs):
        model.train()
        train_loss = 0
        for batch_X, batch_y in train_loader:
            batch_X, batch_y = batch_X.to(device), batch_y.to(device)
            
            # Forward pass
            outputs = model(batch_X)
            loss = criterion(outputs, batch_y)
            
            # Backward and optimize
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
            
        # Validation
        model.eval()
        with torch.no_grad():
            X_val_t, y_val_t = X_val_t.to(device), y_val_t.to(device)
            val_outputs = model(X_val_t)
            val_loss = criterion(val_outputs, y_val_t).item()
            
            _, predicted = torch.max(val_outputs.data, 1)
            val_acc = (predicted == y_val_t).sum().item() / y_val_t.size(0)
            
        if (epoch+1) % 10 == 0:
            print(f'Epoch [{epoch+1}/{epochs}], Train Loss: {train_loss/len(train_loader):.4f}, Val Loss: {val_loss:.4f}, Val Acc: {val_acc:.4f}')
            
        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), 'lstm_model.pt')
            
    # Test evaluation
    model.load_state_dict(torch.load('lstm_model.pt'))
    model.eval()
    with torch.no_grad():
        X_test_t, y_test_t = X_test_t.to(device), y_test_t.to(device)
        test_outputs = model(X_test_t)
        _, predicted = torch.max(test_outputs.data, 1)
        test_acc = (predicted == y_test_t).sum().item() / y_test_t.size(0)
        
    print(f'\\n✅ LSTM Training Complete! Test Accuracy: {test_acc:.4f}')
    
    # Save metrics
    metrics = {
        "test_accuracy": float(test_acc),
        "best_val_loss": float(best_val_loss),
        "window_size": window
    }
    with open("lstm_metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)
        
    return test_acc

if __name__ == "__main__":
    train_lstm()
