import nidaqmx
from nidaqmx.constants import AcquisitionType
from nidaqmx.errors import DaqError
from datetime import datetime
import numpy as np
import time
import argparse
import csv
import sys

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description="Read samples from ATI FT")
parser.add_argument('duration', type=int, help="Duration to run the data collection (in seconds)")
parser.add_argument('filename', type=str, help="Filename to save the data (CSV)")
parser.add_argument('--channel', type=str, default='Dev1/ai0:5', help="NI-DAQ channel string (default: 'Dev1/ai0:5')")
args = parser.parse_args()

channel = args.channel
duration = args.duration
filename = args.filename

print(f"ATI FT: Reading samples from '{channel}' for {duration} seconds and saving to {filename}")

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
        task.ai_channels.add_ai_voltage_chan(channel, min_val=-10.0, max_val=10.0)
        task.timing.cfg_samp_clk_timing(1000.0, sample_mode=AcquisitionType.CONTINUOUS)  # 1000 samples per second. Output rate = sample rate / number of channels
        
        # Initial read to compute bias per channel
        data = task.read(number_of_samples_per_channel=10)
        averages = [sum(channel_data) / len(channel_data) for channel_data in data]
        bias_array = np.array(averages)

        # Validate number of channels matches calibration matrix (6)
        num_channels = len(averages)
        if calibration_matrix.shape[1] != num_channels:
            print(f"Error: calibration matrix expects {calibration_matrix.shape[1]} channels, but channel string '{channel}' produced {num_channels} channels.")
            sys.exit(1)
        
        # Pre-allocate memory for dataArray
        # Estimate number of samples: sampling rate / samples_per_read * duration
        # With 1000 Hz sampling and reading 10 samples per channel each time,
        # we expect roughly 100 reads per second
        samples_per_read = 5  # Number of samples read per channel each time
        estimated_samples = int(duration * (1000 / samples_per_read) * 1.2)  # Add 10% buffer
        dataArray = np.zeros((estimated_samples, 7))  # 7 columns: timestamp + 6 force/torque values
        
        start_time = time.time()
        sample_index = 0
        while time.time() - start_time < duration:  # Run for the specified duration
            data = task.read(number_of_samples_per_channel=samples_per_read)  # Read 10 samples per channel
            timestamp = time.time()  # Include milliseconds
            
            # Calculate the average for each channel
            averages = [sum(channel_data) / len(channel_data) for channel_data in data]
            # print(f"[{timestamp}] Averages: {averages}")
            
            # Convert averages to a NumPy array
            averages_array = np.array(averages) - bias_array
            
            # Multiply the averages with the calibration matrix, wrench format: [Fx, Fy, Fz, Tx, Ty, Tz]
            result = np.dot(calibration_matrix, averages_array)
            # print(f"FT: {result}")
            
            # Save the timestamp and calibrated values to the pre-allocated array
            if sample_index < estimated_samples:
                dataArray[sample_index, 0] = timestamp
                dataArray[sample_index, 1:] = result
                sample_index += 1
            else:
                # If we exceed the estimated size, extend the array
                new_row = np.array([[timestamp] + list(result)])
                dataArray = np.vstack([dataArray, new_row])
                sample_index += 1
        
        # Trim the array to actual size
        dataArray = dataArray[:sample_index]
        
        # Write the dataArray to the CSV file with header
        header = ['timestamp', 'Fx (N)', 'Fy (N)', 'Fz (N)', 'Tx (Nm)', 'Ty (Nm)', 'Tz (Nm)']
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(header)
            for row in dataArray[:sample_index]:
                # format numbers to 6 decimal places for readability
                writer.writerow([f"{row[0]:.6f}"] + [f"{val:.6f}" for val in row[1:]])
        print(f"ATI FT: data saved to {filename} ({sample_index} samples)")
        print(f"Actual sampling rate: {sample_index/duration:.1f} samples/second")

        # Plot the data vs time and save as PNG (non-blocking)
        try:
            import matplotlib.pyplot as plt

            # Relative time (seconds)
            t = dataArray[:sample_index, 0]
            if t.size == 0:
                print("No samples to plot.")
            else:
                t = t - t[0]

                forces = dataArray[:sample_index, 1:4]
                torques = dataArray[:sample_index, 4:7]

                fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

                ax1.plot(t, forces[:, 0], label='Fx')
                ax1.plot(t, forces[:, 1], label='Fy')
                ax1.plot(t, forces[:, 2], label='Fz')
                ax1.set_ylabel('Force (N)')
                ax1.legend(loc='upper right')
                ax1.grid(True)

                ax2.plot(t, torques[:, 0], label='Tx')
                ax2.plot(t, torques[:, 1], label='Ty')
                ax2.plot(t, torques[:, 2], label='Tz')
                ax2.set_ylabel('Torque (Nm)')
                ax2.set_xlabel('Time (s)')
                ax2.legend(loc='upper right')
                ax2.grid(True)

                plot_filename = filename.rsplit('.', 1)[0] + '.png'
                fig.tight_layout()
                fig.savefig(plot_filename)
                plt.close(fig)
                print(f"ATI FT: plot saved to {plot_filename}")
        except ImportError:
            print("matplotlib not installed — install it (pip install matplotlib) to enable plotting")
except DaqError as e:
    print(f"DAQmx Error: {e}")
