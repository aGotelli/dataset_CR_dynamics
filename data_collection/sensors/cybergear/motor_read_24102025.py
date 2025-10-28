import os
import sys
sys.path.append(os.path.join(".", "cybergear"))

from pcan_cybergear import CANMotorController
import can
import math
import time
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib import ticker
import serial


def get_tension(m10):
        
        m10.write("?\r".encode())
        response = m10.readline().decode("utf-8", errors="ignore").strip()
        if not response:
            return float("nan")

        force_str = (response.replace("N", "").replace("lbF", "").replace("lb", "").strip())
        try:
            return float(force_str)
        except ValueError:
            return float("nan")


bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)

motor = CANMotorController(bus, motor_id=3, main_can_id=254)
# Set to position mode
motor.set_run_mode(motor.RunModes.POSITION_MODE)
motor.enable()

m10 = serial.Serial("COM5", baudrate=115200, timeout=0.1)


status=motor.get_motor_status()
home_angle_rad=0.0
if status[0] is not None:
    print(status)
    home_angle_rad = status[1]
    home_angle_deg = math.degrees(home_angle_rad)
    velocity=status[2]
    print(f"initial angle {home_angle_rad} initial velocity {velocity}")
else:
    print(f"Status returned None")


# start increment 
increment = input(f"\nEnter angle increment in degrees: ")

if increment.lower() == 'q':
    motor.disable()
    sys.exit()
increment_deg = float(increment)
increment_rad = math.radians(increment_deg)

target_angle_rad=home_angle_rad+increment_rad
motor.set_motor_position_control(limit_spd=1, loc_ref=target_angle_rad)

final_angle_rad =[]
final_angle_deg=[]
final_velocity=[]
final_tension=[]
timestamp=[]
cur_angle_rad=0.0
cur_angle_deg=0.0
# Monitor for 5 seconds
start= time.time()
while (time.time()-start)<2:
    try:
        status=motor.get_motor_status() # read angle
        cur_angle_rad=status[1]
        cur_angle_deg=math.degrees(cur_angle_rad)
        cur_velocity=status[2]

    except Exception as e:
        status=None
        cur_angle_rad=None
        cur_angle_deg=None

    # try:
    #     tension=get_tension(m10)
    # except Exception as e:
    tension=None
    final_tension.append(tension)

    # error=math.degrees(target_angle_rad)-cur_angle_deg
    timestamp.append(time.time())
    if cur_angle_deg is not None:
        final_angle_rad.append(cur_angle_rad-home_angle_rad)
        final_angle_deg.append(cur_angle_deg-home_angle_deg)
        final_velocity.append(cur_velocity)
        
    # print(f"  Current: {cur_angle_deg-math.degrees(home_angle_rad):.2f}° - {cur_angle_rad-home_angle_rad}rad | Target: {math.degrees(target_angle_rad-home_angle_rad):.2f}° | Error: {error:.2f}°")
    # time.sleep(0.01)

# motor.set_motor_position_control(limit_spd=2, loc_ref=math.radians(-30)+home_angle_rad)

# start= time.time()
# while (time.time()-start)<5:
#     try:
#         status=motor.get_motor_status() # read angle
#         cur_angle_rad=status[1]
#         cur_angle_deg=math.degrees(cur_angle_rad)
#         cur_velocity=status[2]
#     except Exception as e:
#         status=None
#         cur_angle_rad=None
#         cur_angle_deg=None

#     error=math.degrees(target_angle_rad)-cur_angle_deg
#     timestamp.append(time.time())
#     if cur_angle_deg is not None:
#         final_angle_rad.append(cur_angle_rad-home_angle_rad)
#         final_angle_deg.append(cur_angle_deg-home_angle_deg)
#         final_velocity.append(cur_velocity)
        
#     print(f"  Current: {cur_angle_deg-math.degrees(home_angle_rad):.2f}° - {cur_angle_rad-home_angle_rad}rad | Target: {math.degrees(target_angle_rad-home_angle_rad):.2f}° | Error: {error:.2f}°")
#     # time.sleep(0.01)
# Save the final position after monitoring (FUORI dal while)
# Create absolute path to data directory
script_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(script_dir, "data")
os.makedirs(data_dir, exist_ok=True)
csv_filename = os.path.join(data_dir, "motor_only.csv")
print(f"Saving to: {csv_filename}")  # Debug: mostra dove salva
print (final_tension)
# Always overwrite existing file and write header
with open(csv_filename, 'w', newline='') as csvfile:
    # fieldnames = ['timestamp', 'tension']
    fieldnames = ['timestamp', 'home_angle_rad', 'angle_deg', 'velocity'] #'tension'
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    # Scrivi ogni punto della traiettoria
    for ts, rad, deg, vel in zip(timestamp, final_angle_rad, final_angle_deg, final_velocity): #ten,  final_tension
        writer.writerow({
            'timestamp': ts,
            'home_angle_rad': rad,
            'angle_deg': deg,
            'velocity': vel,
            # 'tension': ten,
        })

print(f"Position data saved to {csv_filename}: {len(timestamp)} points")

# Plot angle_deg as a function of time using the in-memory data we just collected

# Convert epoch floats to matplotlib date numbers
times_num = [mdates.date2num(datetime.fromtimestamp(t)) for t in timestamp]
angles = [float(a) for a in final_angle_deg]

# # # # # fig, ax = plt.subplots()
# # # # # ax.plot(times_num, angles, '-', lw=1)
# # # # # ax.set_xlabel('Time')
# # # # # ax.set_ylabel('Angle (deg)')
# # # # # ax.set_title('Angle vs Time')

# # # # # # x-axis formatter with milliseconds
# # # # # ax.xaxis.set_major_formatter(ticker.FuncFormatter(
# # # # #     lambda x, pos: mdates.num2date(x).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
# # # # # ))
# # # # # fig.autofmt_xdate()
# # # # # plt.tight_layout()
# # # # # plt.show()
# # # # # plt.close(fig)

motor.disable()
