from __future__ import print_function
import sys
import warnings
# Suppress the SyntaxWarnings from Vicon SDK
warnings.filterwarnings("ignore", category=SyntaxWarning)

# Add the Vicon SDK path
sys.path.append(r'C:\Program Files\Vicon\DataStream SDK\Win64\Python\vicon_dssdk')
import vicon_dssdk
from vicon_dssdk import ViconDataStream
import argparse
import time
import datetime
import numpy as np
import serial

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description="Read samples from Vicon")
parser.add_argument('duration', type=int, help="Duration to run the data collection (in seconds)")
parser.add_argument('filename', type=str, help="Filename to save the data")
parser.add_argument('host', nargs='?', help="Host name, in the format of server:port", default="192.168.10.2:801")
parser.add_argument('--start-time', type=float, default=None, help="Shared start timestamp (seconds)")
args = parser.parse_args()

if args.start_time is not None:
    print(f"Using shared start time {args.start_time:.6f}")
    start_time = args.start_time
else:
    start_time = time.time()

duration = args.duration
filename = args.filename
host = args.host

print(f"Vicon: Reading samples from '{host}' for {duration} seconds and saving to {filename}") 

client = ViconDataStream.Client()

try:
    client.Connect(host)

    # Check the version
    print('Version', client.GetVersion())

    # Check setting the buffer size works
    client.SetBufferSize(1)

    # Enable all the data types
    client.EnableSegmentData()
    client.EnableMarkerData()
    client.EnableUnlabeledMarkerData()
    client.EnableMarkerRayData()
    client.EnableDeviceData()
    client.EnableCentroidData()

    # Report whether the data types have been enabled
    print('Segments', client.IsSegmentDataEnabled())
    # print('Markers', client.IsMarkerDataEnabled())
    # print('Unlabeled Markers', client.IsUnlabeledMarkerDataEnabled())
    # print('Marker Rays', client.IsMarkerRayDataEnabled())
    # print('Devices', client.IsDeviceDataEnabled())
    # print('Centroids', client.IsCentroidDataEnabled())

    viconDataArray = np.zeros((duration * 110, 36))
    row_index = 0

    while (time.time() - start_time) < duration:
        try:
            if client.GetFrame():
                timestamp = time.time()
                subjectNames = client.GetSubjectNames()
                subjectNames.sort(key=lambda name: int(name[-1]))  # Sort by the last digit of the subject name, e.g., "tongjia0", "tongjia1", etc.
                row_data = [timestamp]
                occluded = False
                for subjectName in subjectNames:
                    segmentNames = client.GetSegmentNames(subjectName)
                    for segmentName in segmentNames:
                        # Get position vector
                        translation, translation_occluded = client.GetSegmentGlobalTranslation(subjectName, segmentName)  # in mm
                        translation = [t / 1000 for t in translation]  # convert to meters
                        # Get rotation matrix
                        # rotm, rotation_occluded = client.GetSegmentGlobalRotationMatrix(subjectName, segmentName)  # rotation matrix
                        # Get quaternion
                        # quaternion, rotation_occluded = client.GetSegmentGlobalRotationQuaternion(subjectName, segmentName)  # quaternion (x, y, z, w)
                        # quaternion = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]  # change to (w, x, y, z)
                        # Get Euler angles
                        rotXYZ, rotation_occluded = client.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName) # Euler angles X, Y, Z in radians
                        # Get frame index
                        subject_index = subjectName[-1]

                        # row_data.extend([subject_index, rotm[0][0], rotm[0][1], rotm[0][2], translation[0], rotm[1][0], rotm[1][1], rotm[1][2], translation[1], rotm[2][0], rotm[2][1], rotm[2][2], translation[2], 0, 0, 0, 1])
                        # row_data.extend([subject_index, translation[0], translation[1], translation[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
                        row_data.extend([subject_index, translation[0], translation[1], translation[2], rotXYZ[0], rotXYZ[1], rotXYZ[2]])
                        if translation_occluded or rotation_occluded:
                            print(f"{subjectName} is occluded")
                            occluded = True
                if not occluded and row_index < viconDataArray.shape[0]:
                    viconDataArray[row_index] = row_data
                    row_index += 1
        except ViconDataStream.DataStreamException as e:
            print('Handled data stream error', e)

    # Trim the array to only include filled rows
    viconDataArray = viconDataArray[:row_index]
    
    # Generate headers based on the number of subjects detected
    headers = ['timestamp']
    if row_index > 0:
        # Determine number of subjects from the data structure
        # Each subject contributes 7 columns: [subject_index, x, y, z, rotX, rotY, rotZ]
        num_subjects = (len(viconDataArray[0]) - 1) // 7  # Subtract 1 for timestamp, divide by 7 columns per subject
        
        for i in range(num_subjects):
            disk_name = f'disk_{i}'
            headers.extend([
                f'disk index',
                f'{disk_name} x (m)',
                f'{disk_name} y (m)', 
                f'{disk_name} z (m)',
                f'{disk_name} rotX (rad)',
                f'{disk_name} rotY (rad)',
                f'{disk_name} rotZ (rad)'
            ])

    with open(filename, 'w') as file:
        # Write headers
        file.write(','.join(headers) + '\n')
        # Write data
        for row in viconDataArray:
            file.write(','.join(map(str, row)) + '\n')
    print(f'Vicon: data saved to {filename} ({row_index} rows)')
    print(f'Vicon: Actual sampling rate: {row_index/duration:.1f} samples/second')

    # Plot all trajectories
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        if row_index > 0:
            # Extract time data (convert to relative time)
            timestamps = viconDataArray[:, 0]
            relative_time = timestamps - timestamps[0]
            
            # Determine number of subjects
            num_subjects = (viconDataArray.shape[1] - 1) // 7
            
            if num_subjects > 0:
                # Create figure with subplots - adjusted layout for rotation subplots
                fig = plt.figure(figsize=(18, 12))
                
                # 3D trajectory plot with equal axes
                ax1 = fig.add_subplot(2, 3, 1, projection='3d')
                colors = plt.cm.tab10(np.linspace(0, 1, num_subjects))
                
                # Track min/max for equal axis scaling
                all_x, all_y, all_z = [], [], []
                
                for i in range(num_subjects):
                    # Extract position data for each subject
                    start_col = 1 + i * 7  # Skip timestamp and subject_index
                    x_data = viconDataArray[:, start_col + 1]  # x position
                    y_data = viconDataArray[:, start_col + 2]  # y position
                    z_data = viconDataArray[:, start_col + 3]  # z position
                    
                    # Collect all coordinates for equal scaling
                    all_x.extend(x_data)
                    all_y.extend(y_data)
                    all_z.extend(z_data)
                    
                    ax1.plot(x_data, y_data, z_data, color=colors[i], label=f'disk_{i}', linewidth=2)
                    # Mark start and end points
                    ax1.scatter(x_data[0], y_data[0], z_data[0], color=colors[i], s=100, marker='o', alpha=0.8)
                    ax1.scatter(x_data[-1], y_data[-1], z_data[-1], color=colors[i], s=100, marker='s', alpha=0.8)
                
                # Set equal axis scaling
                max_range = np.array([all_x, all_y, all_z]).max()
                min_range = np.array([all_x, all_y, all_z]).min()
                mid_x = (max(all_x) + min(all_x)) * 0.5
                mid_y = (max(all_y) + min(all_y)) * 0.5
                mid_z = (max(all_z) + min(all_z)) * 0.5
                ax1.set_xlim(mid_x - max_range*0.5, mid_x + max_range*0.5)
                ax1.set_ylim(mid_y - max_range*0.5, mid_y + max_range*0.5)
                ax1.set_zlim(mid_z - max_range*0.5, mid_z + max_range*0.5)
                
                ax1.set_xlabel('X (m)')
                ax1.set_ylabel('Y (m)')
                ax1.set_zlabel('Z (m)')
                ax1.set_title('3D Trajectories')
                ax1.legend()
                ax1.grid(True)
                
                # X-Y position vs time
                ax2 = fig.add_subplot(2, 3, 2)
                for i in range(num_subjects):
                    start_col = 1 + i * 7
                    x_data = viconDataArray[:, start_col + 1]
                    y_data = viconDataArray[:, start_col + 2]
                    ax2.plot(relative_time, x_data, color=colors[i], label=f'disk_{i} X', linestyle='-')
                    ax2.plot(relative_time, y_data, color=colors[i], label=f'disk_{i} Y', linestyle='--')
                
                ax2.set_xlabel('Time (s)')
                ax2.set_ylabel('Position (m)')
                ax2.set_title('X-Y Position vs Time')
                ax2.legend()
                ax2.grid(True)
                
                # Z position vs time
                ax3 = fig.add_subplot(2, 3, 3)
                for i in range(num_subjects):
                    start_col = 1 + i * 7
                    z_data = viconDataArray[:, start_col + 3]
                    ax3.plot(relative_time, z_data, color=colors[i], label=f'disk_{i}', linewidth=2)
                
                ax3.set_xlabel('Time (s)')
                ax3.set_ylabel('Z Position (m)')
                ax3.set_title('Z Position vs Time')
                ax3.legend()
                ax3.grid(True)
                
                # Rotation X vs time
                ax4 = fig.add_subplot(2, 3, 4)
                for i in range(num_subjects):
                    start_col = 1 + i * 7
                    rotX_data = viconDataArray[:, start_col + 4] * 180/np.pi  # Convert to degrees
                    ax4.plot(relative_time, rotX_data, color=colors[i], label=f'disk_{i}', linewidth=2)
                
                ax4.set_xlabel('Time (s)')
                ax4.set_ylabel('Rotation X (degrees)')
                ax4.set_title('Rotation X vs Time')
                ax4.legend()
                ax4.grid(True)
                
                # Rotation Y vs time
                ax5 = fig.add_subplot(2, 3, 5)
                for i in range(num_subjects):
                    start_col = 1 + i * 7
                    rotY_data = viconDataArray[:, start_col + 5] * 180/np.pi  # Convert to degrees
                    ax5.plot(relative_time, rotY_data, color=colors[i], label=f'disk_{i}', linewidth=2)
                
                ax5.set_xlabel('Time (s)')
                ax5.set_ylabel('Rotation Y (degrees)')
                ax5.set_title('Rotation Y vs Time')
                ax5.legend()
                ax5.grid(True)
                
                # Rotation Z vs time
                ax6 = fig.add_subplot(2, 3, 6)
                for i in range(num_subjects):
                    start_col = 1 + i * 7
                    rotZ_data = viconDataArray[:, start_col + 6] * 180/np.pi  # Convert to degrees
                    ax6.plot(relative_time, rotZ_data, color=colors[i], label=f'disk_{i}', linewidth=2)
                
                ax6.set_xlabel('Time (s)')
                ax6.set_ylabel('Rotation Z (degrees)')
                ax6.set_title('Rotation Z vs Time')
                ax6.legend()
                ax6.grid(True)
                
                plt.tight_layout()
                
                # Save plot
                plot_filename = filename.rsplit('.', 1)[0] + '_trajectories.png'
                plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
                plt.close(fig)
                print(f'Vicon: trajectory plots saved to {plot_filename}')
            else:
                print('Vicon: No subject data to plot')
        else:
            print('Vicon: No data to plot')
            
    except ImportError:
        print("matplotlib not installed — install it (pip install matplotlib) to enable plotting")
    except Exception as e:
        print(f"Vicon: Error creating plots: {e}")

except ViconDataStream.DataStreamException as e:
    print('Vicon: handled data stream error', e)