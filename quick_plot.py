"""
Quick plotter for experiment data
Plots ATI and IMU data from the latest experiment
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob

def find_latest_experiment():
    """Find the most recent experiment folder"""
    data_dir = "data"
    experiment_folders = glob.glob(os.path.join(data_dir, "test_experiment_*"))
    if not experiment_folders:
        print("No experiment folders found!")
        return None
    
    # Sort by modification time and get the latest
    latest_folder = max(experiment_folders, key=os.path.getmtime)
    print(f"📁 Latest experiment: {latest_folder}")
    return latest_folder

def plot_ati_data(folder_path):
    """Plot ATI force/torque data"""
    ati_file = os.path.join(folder_path, "ati_data.csv")
    
    if not os.path.exists(ati_file):
        print(f"❌ ATI file not found: {ati_file}")
        return None
    
    # Read ATI data
    ati_data = pd.read_csv(ati_file)
    print(f"✅ ATI data loaded: {len(ati_data)} samples")
    print(f"   Columns: {list(ati_data.columns)}")
    
    # Calculate relative time
    if 'timestamp' in ati_data.columns:
        start_time = ati_data['timestamp'].iloc[0]
        ati_data['time'] = ati_data['timestamp'] - start_time
    
    # Create subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Plot Forces
    if all(col in ati_data.columns for col in ['Fx', 'Fy', 'Fz']):
        ax1.plot(ati_data['time'], ati_data['Fx'], label='Fx', alpha=0.8)
        ax1.plot(ati_data['time'], ati_data['Fy'], label='Fy', alpha=0.8)
        ax1.plot(ati_data['time'], ati_data['Fz'], label='Fz', alpha=0.8)
        ax1.set_ylabel('Force (N)')
        ax1.set_title('ATI Forces')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
    
    # Plot Torques
    if all(col in ati_data.columns for col in ['Mx', 'My', 'Mz']):
        ax2.plot(ati_data['time'], ati_data['Mx'], label='Mx', alpha=0.8)
        ax2.plot(ati_data['time'], ati_data['My'], label='My', alpha=0.8)
        ax2.plot(ati_data['time'], ati_data['Mz'], label='Mz', alpha=0.8)
        ax2.set_ylabel('Torque (Nm)')
        ax2.set_xlabel('Time (s)')
        ax2.set_title('ATI Torques')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_imu_data(folder_path):
    """Plot IMU gyroscope data"""
    # Look for sensor files
    sensor_files = glob.glob(os.path.join(folder_path, "sensor*.csv"))
    
    if not sensor_files:
        print(f"❌ No IMU sensor files found in {folder_path}")
        return None
    
    print(f"✅ Found {len(sensor_files)} IMU sensor files")
    
    fig, axes = plt.subplots(len(sensor_files), 1, figsize=(12, 4*len(sensor_files)))
    if len(sensor_files) == 1:
        axes = [axes]
    
    for i, sensor_file in enumerate(sensor_files):
        print(f"   Loading: {os.path.basename(sensor_file)}")
        
        try:
            # Read IMU data
            imu_data = pd.read_csv(sensor_file)
            print(f"     Samples: {len(imu_data)}")
            print(f"     Columns: {list(imu_data.columns)}")
            
            # Calculate relative time (convert from microseconds to seconds)
            if 'time (us)' in imu_data.columns:
                start_time = imu_data['time (us)'].iloc[0]
                imu_data['time_s'] = (imu_data['time (us)'] - start_time) / 1e6
            
            # Plot gyro data
            ax = axes[i]
            if all(col in imu_data.columns for col in ['x (mdps)', 'y (mdps)', 'z(mdps)']):
                ax.plot(imu_data['time_s'], imu_data['x (mdps)'], label='X', alpha=0.8)
                ax.plot(imu_data['time_s'], imu_data['y (mdps)'], label='Y', alpha=0.8)
                ax.plot(imu_data['time_s'], imu_data['z(mdps)'], label='Z', alpha=0.8)
                ax.set_ylabel('Angular Rate (mdps)')
                ax.set_xlabel('Time (s)')
                ax.set_title(f'IMU Gyroscope - {os.path.basename(sensor_file)}')
                ax.legend()
                ax.grid(True, alpha=0.3)
            
        except Exception as e:
            print(f"     Error loading {sensor_file}: {e}")
    
    plt.tight_layout()
    return fig

def main():
    """Main plotting function"""
    print("="*60)
    print("EXPERIMENT DATA PLOTTER")
    print("="*60)
    
    # Find latest experiment
    experiment_folder = find_latest_experiment()
    if not experiment_folder:
        return
    
    # List all files in the folder
    files = os.listdir(experiment_folder)
    print(f"\n📋 Files in experiment folder:")
    for file in files:
        print(f"   - {file}")
    
    # Plot ATI data
    print("\n📊 Plotting ATI data...")
    ati_fig = plot_ati_data(experiment_folder)
    
    # Plot IMU data
    print("\n📊 Plotting IMU data...")
    imu_fig = plot_imu_data(experiment_folder)
    
    # Show plots
    if ati_fig or imu_fig:
        plt.show()
        print("\n🎉 Plots displayed!")
    else:
        print("\n❌ No data to plot!")

if __name__ == "__main__":
    main()