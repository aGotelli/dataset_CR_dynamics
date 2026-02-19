import serial
from signal import signal, SIGINT
from sys import exit
import pandas as pd
import datetime
import struct
import imp
import numpy as np
import os
import matplotlib.pyplot as plt
import time

# Enable matrix calculation on electronics 
pfad='C:' # Path for storing the data
os.chdir(pfad)
ser = serial.Serial('COM16', 12000000) # Specify COM port as seen in device manager or FT-Explorer
name='Test.csv' # Name of the File
sample=100  # Sample rate as set by DIP configuration 100 Hz, 500Hz, 1000Hz
rec_time=1  # Record time in minutes
data = []

start = time.time()
for x in range(1,sample*60*rec_time):
    
    
    serial_line = ser.read(28)  # Read one dataset from COM Port 
    [CH2, CH1, CH4, CH3, CH6, CH5, temp] = struct.unpack('fffffff', serial_line[0:28]) # Unpack data
    data.append([datetime.datetime.now(), CH2, CH1, CH4, CH3, CH6, CH5, temp]); # add data to pandas datafame
   
end = time.time()


print(end - start)

df = pd.DataFrame(data)
values= df.to_numpy() # convert dataframe to numpy array

plt.plot(values[:,[1]],label='Fx')
plt.plot(values[:,[2]],label='Fy')
plt.plot(values[:,[3]],label='Fz')
plt.plot(values[:,[4]],label='Mx')
plt.plot(values[:,[5]],label='My')
plt.plot(values[:,[6]],label='Mz')


#plt.legend()
df.to_csv(name)
ser.close()


