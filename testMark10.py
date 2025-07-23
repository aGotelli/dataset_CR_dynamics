import sys
import time
import datetime
import numpy as np
import argparse
import serial

# # Set up command-line argument parsing
# parser = argparse.ArgumentParser(description="Read samples from Mark10-2")
# parser.add_argument('duration', type=int, help="Duration to run the data collection (in seconds)")
# parser.add_argument('filename', type=str, help="Filename to save the data")
# args = parser.parse_args()

# duration = args.duration
# filename = args.filename

# print(f"Mark10-2: Reading samples for {duration} seconds and saving to {filename}")

ser = serial.Serial('COM5', baudrate=115200, timeout=1)  # Windows: COM4, Linux: /dev/ttyUSB0

# Variables for timing measurement
timing_data = np.zeros(10000)  # Pre-allocate memory for timing data
num_samples = 10000  # Number of samples to measure
sample_count = 0

print(f"Measuring timing for {num_samples} data fetches...")
print("Sample #\tTime (ms)\tResponse")
print("-" * 50)

while sample_count < num_samples:
    start_time = time.perf_counter()
    ser.write("?\r".encode())
    response = ser.readline().decode().strip()
    end_time = time.perf_counter()
    
    fetch_time_ms = (end_time - start_time) * 1000  # Convert to milliseconds
    timing_data[sample_count] = fetch_time_ms
    sample_count += 1
    
    # print(f"{sample_count:8d}\t{fetch_time_ms:8.2f}\t{response}")

# Calculate statistics
avg_time = np.mean(timing_data)
min_time = np.min(timing_data)
max_time = np.max(timing_data)
std_time = np.std(timing_data)

print("\n" + "=" * 50)
print("TIMING STATISTICS")
print("=" * 50)
print(f"Number of samples: {num_samples}")
print(f"Average time:      {avg_time:.2f} ms")
print(f"Minimum time:      {min_time:.2f} ms")
print(f"Maximum time:      {max_time:.2f} ms")
print(f"Standard deviation: {std_time:.2f} ms")
print(f"Data rate:         {1000/avg_time:.1f} Hz")

ser.close()
print("\nSerial connection closed.")