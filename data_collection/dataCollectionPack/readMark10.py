import sys
import time
import datetime
import numpy as np
import argparse
import serial
import csv
import os

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description="Read samples from Mark-10 force gauge and save to CSV file.")
parser.add_argument('duration', type=int, help="Duration to run the data collection (in seconds)")
parser.add_argument('filename', type=str, help="Filename to save the data")
parser.add_argument('--port', type=str, default='COM5', help="Serial port to use (default: COM4)")
args = parser.parse_args()

port = args.port
duration = args.duration
filename = args.filename

print(f"Mark-10: Reading samples on {port} for {duration} seconds and saving to {filename}")

ser = serial.Serial(port, baudrate=115200, timeout=1)  # Windows: COM4, Linux: /dev/ttyUSB0
dataArray = np.empty((duration * 500, 2), dtype=float)
row_index = 0
startTime = time.time()
while time.time() - startTime < duration:
    ser.write("?\r".encode())
    response = ser.readline().decode().strip()
    try:
        response_float = float(response)
        if row_index < dataArray.shape[0]:
            dataArray[row_index] = [time.time(), response_float]
            row_index += 1
    except ValueError:
        print(f"Warning: Could not convert '{response}' to float")
    # print(response)

# Trim the array to only include filled rows
dataArray = dataArray[:row_index]

# Save data as CSV file (overwrite existing file)
with open(filename, "w", newline='') as f:
    writer = csv.writer(f)
    # Write CSV header
    writer.writerow(["timestamp", "tension (N)"])
    for row in dataArray:
        # format timestamps and tension to 6 decimals for readability
        writer.writerow([f"{row[0]:.6f}", f"{row[1]:.6f}"])

print(f"Mark-10: data saved to {filename} ({row_index} rows)")
print(f"Actual sampling rate: {row_index/duration:.1f} samples/second")

# Attempt to plot the data vs time and save as PNG next to CSV
try:
    import matplotlib.pyplot as plt

    if dataArray.shape[0] == 0:
        print("No samples to plot.")
    else:
        t = dataArray[:, 0]
        t = t - t[0]
        tension = dataArray[:, 1]

        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(t, tension, label='Tension')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Tension (N)')
        ax.set_title('Mark-10 Tension vs Time')
        ax.grid(True)
        ax.legend()

        plot_filename = os.path.splitext(filename)[0] + '.png'
        fig.tight_layout()
        fig.savefig(plot_filename)
        plt.close(fig)
        print(f"Mark-10: plot saved to {plot_filename}")
except ImportError:
    print("matplotlib not installed — install it (pip install matplotlib) to enable plotting")