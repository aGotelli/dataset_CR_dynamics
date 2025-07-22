# for Vicon
from __future__ import print_function
from vicon_dssdk import ViconDataStream
import argparse
import sys
import time
import datetime

# for ATI FT
import nidaqmx
from nidaqmx.constants import AcquisitionType
from nidaqmx.errors import DaqError
import numpy as np

# for ResenseFT
# import serial
# from signal import signal, SIGINT
# from sys import exit
# import pandas as pd
# import datetime
# import struct
# import imp
# import numpy as np
# import os
# import matplotlib.pyplot as plt
# import time

# for threading
import serial
import threading

def readVicon(duration, filename):
	print(f"Vicon thread started")
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument('host', nargs='?', help="Host name, in the format of server:port", default="192.168.10.2:801")
	args = parser.parse_args()

	client = ViconDataStream.Client()

	try:
		client.Connect(args.host)

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
							translation, translation_occluded = client.GetSegmentGlobalTranslation(subjectName, segmentName)	# in mm
							translation = [t / 1000 for t in translation]  # convert to meters
							# quaternion, rotation_occluded = client.GetSegmentGlobalRotationQuaternion(subjectName, segmentName)	# quaternion (x, y, z, w)
							# quaternion = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]	# change to (w, x, y, z)
							rotation, rotation_occluded = client.GetSegmentGlobalRotationMatrix(subjectName, segmentName)	# rotation matrix
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

def readATIFT(duration, filename):
	print(f"ATI FT thread started")
	# Calibration matrix
	calibration_matrix = np.array([
		[0.13221, -0.07541, 0.07645, 6.18461, -0.15573, -6.10142],
		[-0.16220, -7.45852, 0.11342, 3.46326, 0.05746, 3.61610],
		[10.41723, 0.02199, 10.34789, -0.17272, 10.74723, -0.41960],
		[-0.00066, -0.04164, 0.15144, 0.01453, -0.14737, 0.02745],
		[-0.17053, -0.00023, 0.08313, -0.03642, 0.08890, 0.03090],
		[-0.00105, -0.09214, -0.00218, -0.08602, -0.00095, -0.08592]
	])

	try:
		with nidaqmx.Task() as task:
			task.ai_channels.add_ai_voltage_chan("Dev1/ai0:5", min_val=-10.0, max_val=10.0)
			task.timing.cfg_samp_clk_timing(1000.0, sample_mode=AcquisitionType.CONTINUOUS)  # 1000 samples per second. Output rate = sample rate / number of channels
			
			task.in_stream.input_buf_size = 10
			
			data = task.read(number_of_samples_per_channel=10)
			averages = [sum(channel_data) / len(channel_data) for channel_data in data]
			bias_array = np.array(averages)
			
			start_time = time.time()
			ATIFTDataArray = []
			while time.time() - start_time < duration:  # Run for 30 seconds
				data = task.read(number_of_samples_per_channel=10)  # Read 10 samples per channel
				timestamp = time.time()  # Include milliseconds
				
				# Calculate the average for each channel
				averages = [sum(channel_data) / len(channel_data) for channel_data in data]
				
				# Convert averages to a NumPy array
				averages_array = np.array(averages) - bias_array
				
				# Multiply the averages with the calibration matrix
				result = np.dot(calibration_matrix, averages_array)
				
				# Save the timestamp and calibrated values to the dataArray
				ATIFTDataArray.append([timestamp] + list(result))
			
			# Write the dataArray to the file
			with open(filename, 'w') as file:
				for row in ATIFTDataArray:
					file.write(f"{row[0]} " + " ".join(map(str, row[1:])) + "\n")
			print(f"ATI FT: data saved to {filename}")
	except DaqError as e:
		print(f"ATI FT: DAQmx Error: {e}")

def readMark10_1(duration, filename):
    print(f"Mark10 1 thread started")
    ser = serial.Serial('COM4', baudrate=38400, timeout=1)	# Windows: COM4, Linux: /dev/ttyUSB0
    dataArray = []
    startTime = time.time()
    while time.time() - startTime < duration:
        ser.write("?\r".encode())
        response = ser.readline().decode().strip()
        dataArray.append([time.time(), response])
        # print(response)
    
    with open(filename, "w+") as f:
        for row in dataArray:
            f.write(f"{row[0]} {row[1]}\n")

    print(f"Mark10 1: data saved to {filename}")

def readMark10_2(duration, filename):
    print(f"Mark10 2 thread started")
    ser = serial.Serial('COM5', baudrate=115200, timeout=1)
    dataArray = []
    startTime = time.time()
    while time.time() - startTime < duration:
        ser.write("?\r".encode())
        response = ser.readline().decode().strip()
        dataArray.append([time.time(), response])
        # print(response)
    
    with open(filename, "w+") as f:
        for row in dataArray:
            f.write(f"{row[0]} {row[1]}\n")

    print(f"Mark10 2: data saved to {filename}")
    
# def readResenseFT(duration, filename):
#     print(f"FT thread started")
#     ser = serial.Serial('/dev/ttyACM0', baudrate=12000000, timeout=1)
#     dataArray = []
#     startTime = time.time()
#     while time.time() - startTime < duration:
#         serial_line = ser.read(28)  # Read one dataset from COM Port
#         [CH2, CH1, CH4, CH3, CH6, CH5, temp] = struct.unpack('fffffff', serial_line[0:28])
#         # print(CH1)
#         dataArray.append([time.time(), CH2, CH1, CH4, CH3, CH6, CH5, temp]); # add data to pandas datafame
    
#     with open(filename, "w+") as f:
#         for row in dataArray:
#             f.write(f"{row[0]} {row[1]} {row[2]} {row[3]} {row[4]} {row[5]} {row[6]}\n")

#     print(f"Wrote to: {filename}")


duration = 100  # seconds

thVicon = threading.Thread(target=readVicon, args=(duration, "dataVicon.txt"))
thATIFT = threading.Thread(target=readATIFT, args=(duration, "dataATIFT.txt"))
thMark10_1 = threading.Thread(target=readMark10_1, args=(duration, "dataMark10_1.txt"))
thMark10_2 = threading.Thread(target=readMark10_2, args=(duration, "dataMark10_2.txt"))
# thResenseFT = threading.Thread(target=readResenseFT, args=(duration, "ResenseFT.txt"))

thVicon.start()
# thATIFT.start()
thMark10_1.start()
thMark10_2.start()
# thResenseFT.start()

thVicon.join()
# thATIFT.join()
thMark10_1.join()
thMark10_2.join()
# thResenseFT.join()