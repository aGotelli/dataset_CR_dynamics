from __future__ import print_function
from vicon_dssdk import ViconDataStream
import argparse
import sys
import time
import datetime
import numpy as np
import serial

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description="Read samples from Vicon")
parser.add_argument('duration', type=int, help="Duration to run the data collection (in seconds)")
parser.add_argument('filename', type=str, help="Filename to save the data")
parser.add_argument('host', nargs='?', help="Host name, in the format of server:port", default="192.168.10.2:801")
args = parser.parse_args()

duration = args.duration
filename = args.filename
host = args.host

print(f"Vicon thread started")

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

    start_time = time.time()
    viconDataArray = []

    while (time.time() - start_time) < duration:
        try:
            if client.GetFrame():
                timestamp = time.time()
                subjectNames = client.GetSubjectNames()
                subjectNames.sort(key=lambda name: int(name[-1]))  # Sort by the last digit of the subject name
                row_data = [timestamp]
                occluded = False
                for subjectName in subjectNames:
                    segmentNames = client.GetSegmentNames(subjectName)
                    for segmentName in segmentNames:
                        translation, translation_occluded = client.GetSegmentGlobalTranslation(subjectName, segmentName)  # in mm
                        translation = [t / 1000 for t in translation]  # convert to meters
                        # quaternion, rotation_occluded = client.GetSegmentGlobalRotationQuaternion(subjectName, segmentName)  # quaternion (x, y, z, w)
                        # quaternion = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]  # change to (w, x, y, z)
                        rotation, rotation_occluded = client.GetSegmentGlobalRotationMatrix(subjectName, segmentName)  # rotation matrix
                        subject_index = subjectName[-1]
                        # row_data.extend([subject_index, translation[0], translation[1], translation[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
                        row_data.extend([subject_index, rotation[0][0], rotation[0][1], rotation[0][2], translation[0], rotation[1][0], rotation[1][1], rotation[1][2], translation[1], rotation[2][0], rotation[2][1], rotation[2][2], translation[2], 0, 0, 0, 1])
                        if translation_occluded or rotation_occluded:
                            print(f"{subjectName} is occluded")
                            occluded = True
                if not occluded:
                    viconDataArray.append(row_data)
        except ViconDataStream.DataStreamException as e:
            print('Handled data stream error', e)

    with open(filename, 'w') as file:
        for row in viconDataArray:
            file.write(','.join(map(str, row)) + '\n')
    print(f'Vicon: data saved to {filename}')

except ViconDataStream.DataStreamException as e:
    print('Vicon: handled data stream error', e)