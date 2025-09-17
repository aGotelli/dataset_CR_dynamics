"""
Plot Experiment Data
Script to visualize synchronized acquisition data from test experiments
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import glob
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

def load_experiment_data(experiment_folder):
    """Load all data files from an experiment folder"""
    data = {}
    
    # Load ATI data
    ati_file = os.path.join(experiment_folder, "ati_data.csv")
    if os.path.exists(ati_file):
        data['ati'] = pd.read_csv(ati_file)
        print(f"✅ ATI data loaded: {len(data['ati'])} samples")
    
    # Load Mark10 data (multiple sensors)
    mark10_files = glob.glob(os.path.join(experiment_folder, "mark10_*.csv"))
    data['mark10'] = {}
    for file in mark10_files:
        sensor_name = os.path.basename(file).replace('.csv', '')
        data['mark10'][sensor_name] = pd.read_csv(file)
        print(f"✅ {sensor_name} data loaded: {len(data['mark10'][sensor_name])} samples")
    
    # Load Motor data
    motor_file = os.path.join(experiment_folder, "motor_data.csv")
    if os.path.exists(motor_file):
        data['motor'] = pd.read_csv(motor_file)
        print(f"✅ Motor data loaded: {len(data['motor'])} samples")
    
    # Load Vicon data (optional)
    vicon_file = os.path.join(experiment_folder, "vicon_data.csv")
    if os.path.exists(vicon_file):
        data['vicon'] = pd.read_csv(vicon_file)
        print(f"✅ Vicon data loaded: {len(data['vicon'])} samples")
    
    return data

def convert_timestamps_to_relative(data):
    """Convert absolute timestamps to relative time (starting from 0)"""
    
    # Find the earliest timestamp across all datasets
    earliest_time = float('inf')
    
    # Check ATI timestamps
    if 'ati' in data and 'timestamp' in data['ati'].columns:
        earliest_time = min(earliest_time, data['ati']['timestamp'].min())
    
    # Check Mark10 timestamps
    if 'mark10' in data:
        for sensor_data in data['mark10'].values():
            if 'timestamp' in sensor_data.columns:
                earliest_time = min(earliest_time, sensor_data['timestamp'].min())
    
    # Check Motor timestamps (use 'time' column)
    if 'motor' in data and 'time' in data['motor'].columns:
        # Motor time is already relative, but let's check if it starts from 0
        motor_start = data['motor']['time'].min()
        print(f"Motor time starts from: {motor_start}")
    
    # Convert timestamps to relative time
    if earliest_time != float('inf'):
        print(f"Earliest timestamp: {earliest_time}")
        
        # Convert ATI timestamps
        if 'ati' in data:
            data['ati']['time_rel'] = data['ati']['timestamp'] - earliest_time
        
        # Convert Mark10 timestamps
        if 'mark10' in data:
            for sensor_name in data['mark10']:
                data['mark10'][sensor_name]['time_rel'] = data['mark10'][sensor_name]['timestamp'] - earliest_time
    
    return data

def plot_experiment_overview(data, experiment_name, save_path=None):
    """Create comprehensive plots of the experiment data"""
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 12))
    fig.suptitle(f'Experiment Data Overview: {experiment_name}', fontsize=16, fontweight='bold')
    
    # Plot 1: Mark10 Force Data
    ax1 = axes[0]
    if 'mark10' in data:
        for sensor_name, sensor_data in data['mark10'].items():
            time_col = 'time_rel' if 'time_rel' in sensor_data.columns else 'timestamp'
            ax1.plot(sensor_data[time_col], sensor_data['force'], 
                    label=f'{sensor_name}', linewidth=1.5)
        ax1.set_title('Mark10 Force Sensors', fontweight='bold')
        ax1.set_ylabel('Force (N)')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
    else:
        ax1.text(0.5, 0.5, 'No Mark10 data', ha='center', va='center', transform=ax1.transAxes)
        ax1.set_title('Mark10 Force Sensors - No Data')
    
    # Plot 2: ATI Forces (Fx, Fy, Fz)
    ax2 = axes[1]
    if 'ati' in data:
        time_col = 'time_rel' if 'time_rel' in data['ati'].columns else 'timestamp'
        ax2.plot(data['ati'][time_col], data['ati']['Fx'], label='Fx', linewidth=1)
        ax2.plot(data['ati'][time_col], data['ati']['Fy'], label='Fy', linewidth=1)
        ax2.plot(data['ati'][time_col], data['ati']['Fz'], label='Fz', linewidth=1)
        ax2.set_title('ATI Force/Torque Sensor - Forces', fontweight='bold')
        ax2.set_ylabel('Force (N)')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
    else:
        ax2.text(0.5, 0.5, 'No ATI data', ha='center', va='center', transform=ax2.transAxes)
        ax2.set_title('ATI Forces - No Data')
    
    # Plot 3: ATI Torques (Mx, My, Mz)
    ax3 = axes[2]
    if 'ati' in data:
        time_col = 'time_rel' if 'time_rel' in data['ati'].columns else 'timestamp'
        ax3.plot(data['ati'][time_col], data['ati']['Mx'], label='Mx', linewidth=1)
        ax3.plot(data['ati'][time_col], data['ati']['My'], label='My', linewidth=1)
        ax3.plot(data['ati'][time_col], data['ati']['Mz'], label='Mz', linewidth=1)
        ax3.set_title('ATI Force/Torque Sensor - Torques', fontweight='bold')
        ax3.set_ylabel('Torque (Nm)')
        ax3.set_xlabel('Time (s)')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
    else:
        ax3.text(0.5, 0.5, 'No ATI data', ha='center', va='center', transform=ax3.transAxes)
        ax3.set_title('ATI Torques - No Data')
        ax3.set_xlabel('Time (s)')
    
    # Adjust layout
    plt.tight_layout()
    
    # Save plot if path provided
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"📊 Plot saved to: {save_path}")
    
    plt.show()
    
    return fig

def plot_vicon_3d_trajectory(data, experiment_name, save_path=None):
    """Plot 3D trajectory of Vicon markers"""
    if 'vicon' not in data:
        print("❌ No Vicon data available for 3D trajectory plot")
        return None
    
    vicon_data = data['vicon']
    
    # Create 3D plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Colors for different markers
    colors = ['red', 'blue', 'green', 'orange', 'purple']
    
    # Plot trajectory for each marker
    for i in range(5):  # 5 markers (0-4)
        pos_x_col = f'tongjiaSteelRod_{i}_pos_X'
        pos_y_col = f'tongjiaSteelRod_{i}_pos_Y'
        pos_z_col = f'tongjiaSteelRod_{i}_pos_Z'
        
        if all(col in vicon_data.columns for col in [pos_x_col, pos_y_col, pos_z_col]):
            x = vicon_data[pos_x_col]
            y = vicon_data[pos_y_col]
            z = vicon_data[pos_z_col]
            
            # Plot trajectory
            ax.plot(x, y, z, color=colors[i], label=f'Marker {i}', linewidth=1, alpha=0.7)
            
            # Mark start and end points
            ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color=colors[i], s=100, marker='o', alpha=1.0)
            ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color=colors[i], s=100, marker='s', alpha=1.0)
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title(f'Vicon 3D Trajectory - {experiment_name}', fontweight='bold')
    ax.legend()
    
    # Make axes equal
    ax.set_box_aspect([1,1,1])
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"📊 3D trajectory plot saved to: {save_path}")
    
    plt.show()
    return fig

def create_vicon_animation(data, experiment_name, save_path=None, frame_skip=5):
    """Create animated plot of rod movement over time"""
    if 'vicon' not in data:
        print("❌ No Vicon data available for animation")
        return None
    
    vicon_data = data['vicon']
    
    # Prepare data for animation
    num_frames = len(vicon_data) // frame_skip
    positions = []
    
    for frame in range(0, len(vicon_data), frame_skip):
        frame_positions = []
        for i in range(5):  # 5 markers
            pos_x_col = f'tongjiaSteelRod_{i}_pos_X'
            pos_y_col = f'tongjiaSteelRod_{i}_pos_Y'
            pos_z_col = f'tongjiaSteelRod_{i}_pos_Z'
            
            if all(col in vicon_data.columns for col in [pos_x_col, pos_y_col, pos_z_col]):
                x = vicon_data[pos_x_col].iloc[frame]
                y = vicon_data[pos_y_col].iloc[frame]
                z = vicon_data[pos_z_col].iloc[frame]
                frame_positions.append([x, y, z])
        
        if len(frame_positions) == 5:  # Only add if all markers are present
            positions.append(frame_positions)
    
    if not positions:
        print("❌ No valid position data for animation")
        return None
    
    positions = np.array(positions)
    
    # Create figure and 3D axis
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Colors for markers
    colors = ['red', 'blue', 'green', 'orange', 'purple']
    
    # Initialize plot elements
    markers = []
    lines = []
    
    for i in range(5):
        marker, = ax.plot([], [], [], 'o', color=colors[i], markersize=8, label=f'Marker {i}')
        markers.append(marker)
    
    # Rod lines (connecting consecutive markers)
    for i in range(4):
        line, = ax.plot([], [], [], '-', color='black', linewidth=2, alpha=0.7)
        lines.append(line)
    
    # Set axis limits
    all_positions = positions.reshape(-1, 3)
    ax.set_xlim(all_positions[:, 0].min() - 10, all_positions[:, 0].max() + 10)
    ax.set_ylim(all_positions[:, 1].min() - 10, all_positions[:, 1].max() + 10)
    ax.set_zlim(all_positions[:, 2].min() - 10, all_positions[:, 2].max() + 10)
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title(f'Vicon Rod Animation - {experiment_name}', fontweight='bold')
    ax.legend()
    
    # Animation function
    def animate(frame):
        if frame < len(positions):
            current_positions = positions[frame]
            
            # Update marker positions
            for i, marker in enumerate(markers):
                x, y, z = current_positions[i]
                marker.set_data([x], [y])
                marker.set_3d_properties([z])
            
            # Update rod lines
            for i, line in enumerate(lines):
                x_data = [current_positions[i][0], current_positions[i+1][0]]
                y_data = [current_positions[i][1], current_positions[i+1][1]]
                z_data = [current_positions[i][2], current_positions[i+1][2]]
                line.set_data(x_data, y_data)
                line.set_3d_properties(z_data)
            
            # Update title with time info
            time_rel = frame * frame_skip * (1/100)  # Assuming ~100Hz Vicon data
            ax.set_title(f'Vicon Rod Animation - {experiment_name} (t={time_rel:.2f}s)', fontweight='bold')
        
        return markers + lines
    
    # Create animation
    anim = animation.FuncAnimation(fig, animate, frames=len(positions), 
                                 interval=50, blit=False, repeat=True)
    
    if save_path:
        # Save as gif
        gif_path = save_path.replace('.png', '.gif')
        anim.save(gif_path, writer='pillow', fps=20)
        print(f"📺 Animation saved to: {gif_path}")
        
        # Save a few representative frames as static images
        frame_path = save_path.replace('.png', '_frames.png')
        fig_frames, axes_frames = plt.subplots(2, 3, figsize=(18, 12), subplot_kw={'projection': '3d'})
        axes_frames = axes_frames.flatten()
        
        sample_frames = np.linspace(0, len(positions)-1, 6, dtype=int)
        
        for idx, frame_num in enumerate(sample_frames):
            ax_frame = axes_frames[idx]
            current_positions = positions[frame_num]
            
            # Plot markers
            for i in range(5):
                x, y, z = current_positions[i]
                ax_frame.scatter(x, y, z, color=colors[i], s=50, label=f'Marker {i}' if idx == 0 else "")
            
            # Plot rod lines
            for i in range(4):
                x_data = [current_positions[i][0], current_positions[i+1][0]]
                y_data = [current_positions[i][1], current_positions[i+1][1]]
                z_data = [current_positions[i][2], current_positions[i+1][2]]
                ax_frame.plot(x_data, y_data, z_data, 'k-', linewidth=2, alpha=0.7)
            
            ax_frame.set_xlim(all_positions[:, 0].min() - 10, all_positions[:, 0].max() + 10)
            ax_frame.set_ylim(all_positions[:, 1].min() - 10, all_positions[:, 1].max() + 10)
            ax_frame.set_zlim(all_positions[:, 2].min() - 10, all_positions[:, 2].max() + 10)
            
            time_rel = frame_num * frame_skip * (1/100)
            ax_frame.set_title(f't={time_rel:.2f}s')
            ax_frame.set_xlabel('X (mm)')
            ax_frame.set_ylabel('Y (mm)')
            ax_frame.set_zlabel('Z (mm)')
            
            if idx == 0:
                ax_frame.legend()
        
        plt.tight_layout()
        plt.savefig(frame_path, dpi=300, bbox_inches='tight')
        plt.close(fig_frames)
        print(f"📊 Animation frames saved to: {frame_path}")
    
    plt.show()
    return anim

def analyze_experiment_folder(experiment_folder):
    """Analyze a single experiment folder"""
    print(f"\n{'='*60}")
    print(f"ANALYZING: {os.path.basename(experiment_folder)}")
    print(f"{'='*60}")
    
    # Load data
    data = load_experiment_data(experiment_folder)
    
    if not data:
        print("❌ No data files found in this experiment folder")
        return
    
    # Convert timestamps to relative time
    data = convert_timestamps_to_relative(data)
    
    # Print data summary
    print(f"\n📊 Data Summary:")
    if 'ati' in data:
        duration_ati = data['ati']['time_rel'].max() - data['ati']['time_rel'].min()
        print(f"   ATI: {len(data['ati'])} samples, Duration: {duration_ati:.2f}s")
    
    if 'mark10' in data:
        for sensor_name in data['mark10']:
            sensor_data = data['mark10'][sensor_name]
            duration = sensor_data['time_rel'].max() - sensor_data['time_rel'].min()
            print(f"   {sensor_name}: {len(sensor_data)} samples, Duration: {duration:.2f}s")
    
    if 'motor' in data:
        duration_motor = data['motor']['time'].max() - data['motor']['time'].min()
        print(f"   Motor: {len(data['motor'])} samples, Duration: {duration_motor:.2f}s")
    
    # Create plot
    experiment_name = os.path.basename(experiment_folder)
    plot_path = os.path.join(experiment_folder, f"{experiment_name}_analysis.png")
    
    plot_experiment_overview(data, experiment_name, plot_path)
    
    # Create Vicon plots if Vicon data is available
    if 'vicon' in data:
        print(f"\n📡 Creating Vicon plots...")
        
        # 3D trajectory plot
        trajectory_path = os.path.join(experiment_folder, f"{experiment_name}_vicon_3d.png")
        plot_vicon_3d_trajectory(data, experiment_name, trajectory_path)
        
        # Animation (this will show interactively and save files)
        animation_path = os.path.join(experiment_folder, f"{experiment_name}_vicon_animation.png")
        create_vicon_animation(data, experiment_name, animation_path, frame_skip=10)
    
    return data

def main():
    """Main function to analyze experiments"""
    print("🔬 EXPERIMENT DATA ANALYSIS")
    print("="*50)
    
    # Find all experiment folders
    data_dir = "data"
    experiment_folders = []
    
    if os.path.exists(data_dir):
        for item in os.listdir(data_dir):
            item_path = os.path.join(data_dir, item)
            if os.path.isdir(item_path) and item.startswith("test_experiment"):
                experiment_folders.append(item_path)
    
    if not experiment_folders:
        print("❌ No experiment folders found in data directory")
        return
    
    # Sort by date (newest first)
    experiment_folders.sort(reverse=True)
    
    print(f"📁 Found {len(experiment_folders)} experiment folders:")
    for i, folder in enumerate(experiment_folders):
        print(f"   {i+1}. {os.path.basename(folder)}")
    
    # Ask user to select experiment or analyze all
    choice = input(f"\nEnter experiment number (1-{len(experiment_folders)}) or 'all' for all experiments: ").strip()
    
    if choice.lower() == 'all':
        # Analyze all experiments
        for folder in experiment_folders:
            try:
                analyze_experiment_folder(folder)
            except Exception as e:
                print(f"❌ Error analyzing {os.path.basename(folder)}: {e}")
    else:
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(experiment_folders):
                analyze_experiment_folder(experiment_folders[idx])
            else:
                print("❌ Invalid experiment number")
        except ValueError:
            print("❌ Invalid input")

if __name__ == "__main__":
    main()
