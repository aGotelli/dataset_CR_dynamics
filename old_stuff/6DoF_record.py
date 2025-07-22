import nidaqmx
from nidaqmx.constants import AcquisitionType
from nidaqmx.errors import DaqError
from datetime import datetime
import numpy as np
import time
import argparse
import csv

import matplotlib.pyplot as plt

# Set up command-line argument parsing
# parser = argparse.ArgumentParser(description="Read samples from ATI FT")
# parser.add_argument('duration', type=int, help="Duration to run the data collection (in seconds)")
# parser.add_argument('filename', type=str, help="Filename to save the data")
# args = parser.parse_args()

duration = 30 

# print(f"ATI FT: Reading samples for {duration} seconds and saving to {filename}")
print("Starting data collection...")

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
        
        # data = task.read(number_of_samples_per_channel=10)
        # averages = [sum(channel_data) / len(channel_data) for channel_data in data]
        # bias_array = np.array(averages)*0
        
        start_time = time.time()
        dataArray = []
        while time.time() - start_time < duration:  # Run for the specified duration
            data = np.array(task.read(number_of_samples_per_channel=1))  # Read 1 sample per channel
            timestamp = time.time()  # Include milliseconds
            
            # Multiply the averages with the calibration matrix
            result = np.dot(calibration_matrix, data)
            # print(f"FT: {result}")

            # Ensure result is 1D and append timestamp and result to dataArray
            result_flat = result.flatten()  # Flatten to ensure 1D array
            dataArray.append([timestamp] + result_flat.tolist())

        dataArray = np.array(dataArray)
        print(f"dataArray shape: {dataArray.shape}")
        # data has shape time Fx Fy Fz Mx My Mz
        # Convert dataArray to a NumPy array for easier manipulation
        

        


        with open('test_data.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
            writer.writerows(dataArray)
        print("Data saved to test_data.csv")
        
    

except DaqError as e:
    print(f"DAQmx Error: {e}")
