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
    
    with open(filename, 'w') as file:
        for row in viconDataArray:
            file.write(','.join(map(str, row)) + '\n')
    print(f'Vicon: data saved to {filename} ({row_index} rows)')

except ViconDataStream.DataStreamException as e:
    print('Vicon: handled data stream error', e)