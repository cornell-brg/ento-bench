#!/usr/bin/env python3
import pandas as pd
import numpy as np
import sys

# Path to the Excel file
excel_file = 'robofly-traj1.xls'

try:
    # Read the Excel file
    print(f"Reading trajectory from {excel_file}...")
    df = pd.read_excel(excel_file)
    
    # Print column names
    print("\nColumns in the file:")
    print(df.columns.tolist())
    
    # Print data dimensions
    print(f"\nDataset dimensions: {df.shape[0]} rows x {df.shape[1]} columns")
    
    # Print first few rows
    print("\nFirst 5 rows of data:")
    print(df.head())
    
    # Print summary statistics
    print("\nSummary statistics:")
    print(df.describe())
    
    # Extract specific column data that matches our state vector
    print("\nExtracting potential state vector columns...")
    
    # Try to identify position columns (x, y, z)
    pos_cols = [col for col in df.columns if any(pos in col.lower() for pos in ['x', 'y', 'z', 'pos'])]
    if pos_cols:
        print(f"Potential position columns: {pos_cols}")
        
    # Try to identify velocity columns
    vel_cols = [col for col in df.columns if any(vel in col.lower() for vel in ['vel', 'v_', 'velocity'])]
    if vel_cols:
        print(f"Potential velocity columns: {vel_cols}")
    
    # Try to identify angle columns
    angle_cols = [col for col in df.columns if any(ang in col.lower() for ang in ['ang', 'angle', 'phi', 'theta', 'psi'])]
    if angle_cols:
        print(f"Potential angle columns: {angle_cols}")
    
    # Try to identify angular velocity columns
    omega_cols = [col for col in df.columns if any(ang in col.lower() for ang in ['omega', 'angular vel'])]
    if omega_cols:
        print(f"Potential angular velocity columns: {omega_cols}")
    
    # Try to identify time columns
    time_cols = [col for col in df.columns if any(t in col.lower() for pos in ['time', 't'])]
    if time_cols:
        print(f"Potential time columns: {time_cols}")
    
    # Print example state vectors at specific time points
    if df.shape[0] > 0:
        print("\nExample state vectors:")
        indices = [0, df.shape[0]//4, df.shape[0]//2, 3*df.shape[0]//4, df.shape[0]-1]
        for i in indices:
            if i < df.shape[0]:
                print(f"\nState at row {i}:")
                print(df.iloc[i])
    
except Exception as e:
    print(f"Error reading Excel file: {e}")
    sys.exit(1) 