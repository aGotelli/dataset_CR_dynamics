import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

def load_and_plot_data():
    # Define the data folder path
    data_folder = r"C:\Users\crladmin\Desktop\dataset_CR_dynamics\data\test_experiment_20250915_123313"
    
    # Load files directly
    ati_file = os.path.join(data_folder, 'ati_data.csv')
    mark10_1_file = os.path.join(data_folder, 'mark10_1.csv')
    mark10_2_file = os.path.join(data_folder, 'mark10_2.csv')
    vicon_file = os.path.join(data_folder, 'vicon_data.csv')

    # Create figure with multiple subplots
    fig = plt.figure(figsize=(15, 12))
    fig.suptitle('Sensor Data Analysis - Test Experiment 20250915_123313', fontsize=16)
    
    # Load all data
    df_ati = pd.read_csv(ati_file)
    df_mark10_1 = pd.read_csv(mark10_1_file)
    df_mark10_2 = pd.read_csv(mark10_2_file)
    df_vicon = pd.read_csv(vicon_file)

    # Convert timestamps to relative time (starting from 0)
    df_ati['time_rel'] = df_ati['timestamp'] - df_ati['timestamp'].iloc[0]
    df_mark10_1['time_rel'] = df_mark10_1['timestamp'] - df_mark10_1['timestamp'].iloc[0]
    df_mark10_2['time_rel'] = df_mark10_2['timestamp'] - df_mark10_2['timestamp'].iloc[0]
    df_vicon['time_rel'] = df_vicon['timestamp'] - df_vicon['timestamp'].iloc[0]

    # Plot force components (Fx, Fy, Fz)
    ax1 = plt.subplot(3, 1, 1)
    plt.plot(df_ati['time_rel'], df_ati['Fx'], label='Fx', alpha=0.8)
    plt.plot(df_ati['time_rel'], df_ati['Fy'], label='Fy', alpha=0.8)
    plt.plot(df_ati['time_rel'], df_ati['Fz'], label='Fz', alpha=0.8)
    plt.title('ATI Force Components')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot moment components (Mx, My, Mz)
    ax2 = plt.subplot(3, 1, 2)
    plt.plot(df_ati['time_rel'], df_ati['Mx'], label='Mx', alpha=0.8)
    plt.plot(df_ati['time_rel'], df_ati['My'], label='My', alpha=0.8)
    plt.plot(df_ati['time_rel'], df_ati['Mz'], label='Mz', alpha=0.8)
    plt.title('ATI Moment Components')
    plt.xlabel('Time (s)')
    plt.ylabel('Moment (N⋅m)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot Mark 10 data
    ax3 = plt.subplot(3, 1, 3)
    plt.plot(df_mark10_1['time_rel'], df_mark10_1['force'], label='Mark10 Motor 1', linewidth=1.5, alpha=0.8)
    plt.plot(df_mark10_2['time_rel'], df_mark10_2['force'], label='Mark10 Motor 2', linewidth=1.5, alpha=0.8)
    plt.title('Mark 10 Force Gauge Data - Both Motors')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

    fig = plt.figure(figsize=(18, 12))
    fig.suptitle('Vicon Motion Capture Data - All Steel Rods', fontsize=16)
    # Plot positions for all 5 steel rods
    for i in range(5): # goes up to 4
        ax_pos = plt.subplot(2, 5, i+1)
        plt.plot(df_vicon['time_rel'], df_vicon[f'tongjiaSteelRod_{i}_pos_X'], label='X', alpha=0.8)
        plt.plot(df_vicon['time_rel'], df_vicon[f'tongjiaSteelRod_{i}_pos_Y'], label='Y', alpha=0.8)
        plt.plot(df_vicon['time_rel'], df_vicon[f'tongjiaSteelRod_{i}_pos_Z'], label='Z', alpha=0.8)
        plt.title(f'Steel Rod {i} - Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (mm)')
        plt.legend()
        plt.grid(True, alpha=0.3)
    
    # Plot rotations for all 5 steel rods
    for i in range(5):
        ax_rot = plt.subplot(2, 5, i+6) #another loop to subplot in the row below
        plt.plot(df_vicon['time_rel'], df_vicon[f'tongjiaSteelRod_{i}_rot_X'], label='Rx', alpha=0.8)
        plt.plot(df_vicon['time_rel'], df_vicon[f'tongjiaSteelRod_{i}_rot_Y'], label='Ry', alpha=0.8)
        plt.plot(df_vicon['time_rel'], df_vicon[f'tongjiaSteelRod_{i}_rot_Z'], label='Rz', alpha=0.8)
        plt.title(f'Steel Rod {i} - Rotation')
        plt.xlabel('Time (s)')
        plt.ylabel('Rotation (rad)')
        plt.legend()
        plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


    fig = plt.figure(figsize=(15, 10))
    fig.suptitle('3D Trajectories of Steel Rods - Vicon Motion Capture', fontsize=16)
    
    # 3D trajectory plot
    ax = fig.add_subplot(111, projection='3d')
    
    colors = ['red', 'blue', 'green', 'orange', 'purple']
    
    for i in range(5):
        x = df_vicon[f'tongjiaSteelRod_{i}_pos_X']
        y = df_vicon[f'tongjiaSteelRod_{i}_pos_Y'] 
        z = df_vicon[f'tongjiaSteelRod_{i}_pos_Z']
        
        # Plot trajectory
        ax.plot(x, y, z, color=colors[i], alpha=0.7, linewidth=2, label=f'Steel Rod {i}')
        
        # Mark start and end points
        ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color=colors[i], s=100, marker='o', alpha=1.0)
        ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color=colors[i], s=100, marker='s', alpha=1.0)
    
    ax.set_xlabel('X Position (mm)')
    ax.set_ylabel('Y Position (mm)')
    ax.set_zlabel('Z Position (mm)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print("=== Plotting sensor data ===")
    load_and_plot_data()
    print("\nPlotting completed!")