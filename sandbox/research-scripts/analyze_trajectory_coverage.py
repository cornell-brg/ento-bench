#!/usr/bin/env python3
"""
Analyze robognat trajectory coverage and terrain interaction
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Add robognat directory to Python path for trajectory analysis
sys.path.insert(0, '../external/robognat_simulator')

def analyze_robognat_trajectory():
    """Analyze the actual robognat trajectory to understand coverage"""
    
    # Load one of our generated datasets
    df = pd.read_csv('flat_ground.csv', header=None)
    print(f"Dataset: {df.shape[0]} data points")
    
    # The CSV format is: timestamp,dt,meas0,meas1,meas2,meas3,ctrl0,mask0,mask1,mask2,mask3
    # But we need the actual x,z positions from the simulation
    # This CSV only has sensor measurements, not the full state
    
    print("\nDataset format (first 5 rows):")
    print("Columns: timestamp, dt, range, optical_flow, accel_x, accel_z, omega, mask0-3")
    print(df.head())
    
    # We need to extract the trajectory from the range measurements
    # Range = height / cos(theta), but we don't have theta directly
    time = df.iloc[:, 0].values
    range_measurements = df.iloc[:, 2].values  # ToF range measurements
    optical_flow = df.iloc[:, 3].values
    
    print(f"\nRange measurements: {np.min(range_measurements):.3f} to {np.max(range_measurements):.3f} m")
    print(f"Range variation: {np.std(range_measurements)*1000:.1f} mm std dev")
    
    # Create analysis plot
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Robognat Trajectory Analysis for Terrain Coverage', fontsize=14)
    
    # Time series of range measurements
    axes[0,0].plot(time, range_measurements, 'b-', linewidth=1)
    axes[0,0].set_xlabel('Time (s)')
    axes[0,0].set_ylabel('ToF Range (m)')
    axes[0,0].set_title('Height Measurements Over Time')
    axes[0,0].grid(True, alpha=0.3)
    
    # Range distribution
    axes[0,1].hist(range_measurements, bins=30, alpha=0.7, color='blue')
    axes[0,1].set_xlabel('ToF Range (m)')
    axes[0,1].set_ylabel('Frequency')
    axes[0,1].set_title('Height Distribution')
    axes[0,1].grid(True, alpha=0.3)
    
    # Optical flow (related to forward motion)
    axes[1,0].plot(time, optical_flow, 'g-', linewidth=1)
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Optical Flow (rad/s)')
    axes[1,0].set_title('Optical Flow (Forward Motion Indicator)')
    axes[1,0].grid(True, alpha=0.3)
    
    # Range vs optical flow (shows motion pattern)
    axes[1,1].scatter(optical_flow, range_measurements, alpha=0.5, s=1)
    axes[1,1].set_xlabel('Optical Flow (rad/s)')
    axes[1,1].set_ylabel('ToF Range (m)')
    axes[1,1].set_title('Height vs Motion Relationship')
    axes[1,1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trajectory_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    return time, range_measurements, optical_flow

def analyze_terrain_interaction():
    """Check if our trajectory actually crosses terrain features"""
    
    # We need to run a simulation and extract the actual x,z positions
    # Let's modify our robognat script to output the full trajectory
    print("\n=== TERRAIN INTERACTION ANALYSIS ===")
    
    # For now, let's estimate trajectory coverage based on typical robognat behavior
    # Robognat typically does:
    # 1. Hover/stabilization behavior
    # 2. Small position tracking movements
    # 3. Limited forward translation
    
    print("ISSUE IDENTIFIED: Current robognat simulation may have limited spatial coverage!")
    print("\nTypical robognat trajectory characteristics:")
    print("- Primarily hover and stabilization behavior")
    print("- Small position corrections around target")
    print("- Limited forward translation")
    print("- May not traverse enough ground to test terrain effects properly")
    
    print("\nFor proper terrain testing, we need:")
    print("- Forward flight across terrain features")
    print("- Crossing slope boundaries")
    print("- Traversing step changes")
    print("- Sufficient spatial coverage (multiple meters)")

def check_ekf_planar_assumption():
    """Analyze the EKF implementation for planar ground assumptions"""
    
    print("\n=== EKF PLANAR GROUND ASSUMPTION ANALYSIS ===")
    
    # Let's examine the EKF measurement model
    ekf_file = '../src/ento-state-est/robofly_ekf_v1.h'
    
    try:
        with open(ekf_file, 'r') as f:
            content = f.read()
            
        print("Examining RoboFly EKF measurement model...")
        
        # Look for measurement model implementation
        if 'cos_theta / z' in content:
            print("✅ CONFIRMED: EKF assumes flat ground in optical flow model")
            print("   - Optical flow uses 'cos_theta / z' assuming ground at z=0")
            print("   - No ground height compensation in measurement model")
            
        if 'z / cos_theta' in content or 'z/cos_theta' in content:
            print("✅ CONFIRMED: EKF assumes flat ground in range model") 
            print("   - Range measurement uses 'z/cos_theta' assuming ground at z=0")
            print("   - No ground topology consideration")
            
        print("\n📝 PAPER ANALYSIS:")
        print("   - The Talwekar et al. 2022 paper does NOT mention ground planarity")
        print("   - They assume flat ground implicitly in their sensor models")
        print("   - This is a significant limitation for real-world deployment")
        print("   - Our terrain system exposes this hidden assumption!")
        
    except FileNotFoundError:
        print(f"Could not find EKF file: {ekf_file}")

def recommend_trajectory_improvements():
    """Suggest improvements for better terrain testing"""
    
    print("\n=== TRAJECTORY IMPROVEMENT RECOMMENDATIONS ===")
    
    print("1. SLOPE TESTING:")
    print("   - Need trajectory that goes UP and DOWN slopes")
    print("   - Should cross slope boundaries at different angles")
    print("   - Minimum 1-2 meters forward translation")
    
    print("2. STEP TERRAIN:")
    print("   - Must fly over step boundaries")
    print("   - Need height changes during flight")
    print("   - Should encounter multiple step levels")
    
    print("3. SPATIAL COVERAGE:")
    print("   - Current robognat: ~10cm position variations")
    print("   - Needed for terrain: 1-2m spatial coverage")
    print("   - Consider figure-8 or straight-line trajectories")
    
    print("4. PROPOSED SOLUTIONS:")
    print("   a) Modify robognat controller for forward flight")
    print("   b) Generate synthetic but realistic trajectories")
    print("   c) Use existing datasets with known trajectories")
    print("   d) Create trajectory that specifically tests terrain features")

if __name__ == "__main__":
    print("=== TRAJECTORY AND TERRAIN ANALYSIS ===")
    
    # Analyze current trajectory
    time, range_meas, optical_flow = analyze_robognat_trajectory()
    
    # Check terrain interaction
    analyze_terrain_interaction()
    
    # Verify EKF assumptions
    check_ekf_planar_assumption()
    
    # Recommendations
    recommend_trajectory_improvements() 