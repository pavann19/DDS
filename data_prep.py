import pandas as pd
import numpy as np
import json

def create_sequences(df, features, target_col, window_size=10):
    """
    Creates sequences for LSTM training.
    Returns (X_seq, y_seq) where X_seq is of shape (samples, window_size, features)
    and y_seq is (samples,)
    """
    # Map string labels to numeric for sequence generation
    df_copy = df.copy()
    df_copy['target_numeric'] = df_copy[target_col].astype('category').cat.codes
    
    X = df_copy[features].values
    y = df_copy['target_numeric'].values
    
    X_seq, y_seq = [], []
    for i in range(len(df_copy) - window_size):
        X_seq.append(X[i:i+window_size])
        y_seq.append(y[i+window_size])
        
    return np.array(X_seq), np.array(y_seq)

def prep_dataset(input_file="OBD_2_dataset.csv", output_file="processed_telemetry.csv", window_size=10):
    """
    Processes raw OBD-II telemetry data into a clean, ML-ready dataset.
    
    Key design decisions:
    - Target ('Driving_Decision') is derived from OBD Speed delta.
    - OBD Speed and GPS Speed are EXCLUDED from features to prevent data leakage.
    - Latitude/Longitude are dropped (not useful for driving decision prediction).
    - Delta features (RPM_Delta, CO2_Delta, Fuel_Rate_Delta) are engineered to
      capture temporal dynamics without leaking the target.
    """
    df = pd.read_csv(input_file)
    
    # Sort by index (assumes temporal ordering)
    if 'Unnamed: 0' in df.columns:
        df = df.sort_values(by='Unnamed: 0')
        df = df.drop(columns=['Unnamed: 0'])
    
    # --- TARGET GENERATION ---
    # Compute speed delta to determine driving decision
    df['Speed_Delta'] = df['OBD Speed'].diff()
    
    def label_action(delta):
        if pd.isna(delta):
            return "Maintain Speed"
        if delta > 2:
            return "Accelerate"
        elif delta < -2:
            return "Decelerate"
        else:
            return "Maintain Speed"
            
    df['Driving_Decision'] = df['Speed_Delta'].apply(label_action)
    
    # --- FEATURE ENGINEERING ---
    # Temporal delta features capture change dynamics without leaking the target
    df['RPM_Delta'] = df['RPM'].diff().fillna(0)
    df['CO2_Delta'] = df['CO2'].diff().fillna(0)
    df['Fuel_Rate_Delta'] = df['Litre per 100km(Instant)'].diff().fillna(0)
    
    # --- DROP LEAKED / IRRELEVANT COLUMNS ---
    # OBD Speed & GPS Speed: leak the target (Driving_Decision is derived from OBD Speed)
    # Latitude & Longitude: not predictive for driving decisions
    # Speed_Delta: this IS the target, keeping it would be direct leakage
    cols_to_drop = ['OBD Speed', 'GPS Speed', 'Latitude', 'Longitude', 'Speed_Delta']
    df = df.drop(columns=[c for c in cols_to_drop if c in df.columns])
    
    # Drop rows with NaN (from diff operations)
    df = df.dropna()
    
    # --- FINAL FEATURE SET ---
    feature_cols = ['Altitude', 'CO2', 'Coolant', 'Litre per 100km(Instant)', 
                    'RPM', 'RPM_Delta', 'CO2_Delta', 'Fuel_Rate_Delta']
    target_col = 'Driving_Decision'
    
    # Keep only features + target
    df = df[feature_cols + [target_col]]
    
    # Save
    df.to_csv(output_file, index=False)
    
    # --- REPORTING ---
    print(f"Dataset processed and saved to {output_file}")
    print(f"Total samples: {len(df)}")
    print(f"Features ({len(feature_cols)}): {feature_cols}")
    print(f"\nClass distribution:")
    counts = df['Driving_Decision'].value_counts()
    for cls, count in counts.items():
        print(f"  {cls}: {count} ({count/len(df)*100:.1f}%)")
    
    # Save feature list for downstream use
    feature_info = {
        "features": feature_cols,
        "target": target_col,
        "total_samples": len(df),
        "class_distribution": counts.to_dict()
    }
    with open("feature_info.json", "w") as f:
        json.dump(feature_info, f, indent=2)
    print(f"\nFeature info saved to feature_info.json")

    # Generate sequences for LSTM
    print(f"\n🔄 Generating sequences for LSTM (Window Size: {window_size})...")
    X_seq, y_seq = create_sequences(df, feature_cols, target_col, window_size=window_size)
    np.savez("sequences.npz", X=X_seq, y=y_seq)
    print(f"✅ Sequences saved to sequences.npz (Shape: {X_seq.shape})")
    
if __name__ == "__main__":
    prep_dataset()
