import sys
import time
import datetime
import numpy as np
import argparse
import serial

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description="Read samples from Mark10-2")
parser.add_argument('duration', type=int, help="Duration to run the data collection (in seconds)")
parser.add_argument('filename', type=str, help="Filename to save the data")
args = parser.parse_args()

duration = args.duration
filename = args.filename

print(f"Mark10-2: Reading samples for {duration} seconds and saving to {filename}")

ser = serial.Serial('COM5', baudrate=115200, timeout=1)  # Windows: COM4, Linux: /dev/ttyUSB0
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

print(f"Mark10-2: data saved to {filename}")