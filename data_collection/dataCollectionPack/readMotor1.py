import argparse
import csv
import math
import sys
import time
import os

import numpy as np

from pathlib import Path

# Aggiungi la root del progetto (che contiene "sensors")
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from sensors.cybergear.pcan_cybergear import CANMotorController 
import can

# Input
parser = argparse.ArgumentParser(
    description="Actuate Cybergear motor with a ramp and log telemetry."
)
parser.add_argument("duration", type=float, help="Duration to run the data collection (in seconds)")
parser.add_argument("filename", type=str, help="CSV filename to write")
parser.add_argument("--motor-id", type=int, default=3, help="CAN ID of the motor (default: 3)")
parser.add_argument("--increment-deg", type=float, default=90.0, help="Ramp amplitude in degrees from the current position (default: 90)")
parser.add_argument("--start-time", type=float, default=None, help="Shared start timestamp (seconds)")
args = parser.parse_args()

limit_speed = 0.25  # rad/s
duration = args.duration
increment_rad = math.radians(args.increment_deg)

name, ext = os.path.splitext(args.filename)
if not ext:
    ext = ".csv"
suffix = str(args.increment_deg).replace(".", "p")
filename = f"{name}_motor{args.motor_id}_ramp_{suffix}{ext}"

print(
    f"Motor {args.motor_id}: ramping by {args.increment_deg:.1f} deg "
    f"over {args.duration:.2f} s, logging to {filename}"
)

bus = can.interface.Bus(interface="candle", channel=0, bitrate=1_000_000)
motor = CANMotorController(bus, motor_id=args.motor_id, main_can_id=254)
motor.set_run_mode(motor.RunModes.POSITION_MODE)
motor.enable()
status = None
attempts = 0
while status is None and attempts < 20:
    s = motor.get_motor_status()
    if s and len(s) > 2 and s[1] is not None:
        status = s
        break
    attempts += 1
if status is None:
    raise RuntimeError("Initial motor status not available")
home_angle_rad = status[1]
home_angle_deg = math.degrees(home_angle_rad) 
target_rad = home_angle_rad + increment_rad  # set target angle in rad for actuation

print(f"home: {home_angle_rad} ({home_angle_deg}) - target_rad {target_rad-home_angle_rad} ({math.degrees(target_rad)-home_angle_deg})")
if args.start_time is not None:
    print(f"Using shared start time {args.start_time:.6f}")
    start_time = args.start_time
else:
    start_time = time.time()
# Safety guard: block commands beyond ±90° absolute range
# SAFE_LIMIT_RAD = math.radians(90)
# if abs(increment_rad) > SAFE_LIMIT_RAD:
#     motor.disable()
#     bus.shutdown()
#     raise ValueError("Requested increment exceeds ±90°")

# if abs(target_rad) > SAFE_LIMIT_RAD:
#     motor.disable()
#     bus.shutdown()
#     raise ValueError("Target position exceeds ±90°; aborting to protect the robot")


# preallocate
data_array = np.empty((int(duration * 2200) + 10, 5), dtype=float) #read 4 data - 2200 from experiments reading in motor_read_24102025 - actuate and read for 1-2 s gives rate of 1.87KHz
row_index = 0

motor.set_motor_position_control(limit_spd=limit_speed, loc_ref=target_rad)
while time.time() - start_time < duration:
    status = motor.get_motor_status()
    
    raw_angle = status[1] if status and len(status) > 1 else None
    angle_rad = raw_angle if isinstance(raw_angle, (int, float)) else float("nan")
    angle_rad = angle_rad-home_angle_rad

    raw_velocity = status[2] if status and len(status) > 2 else None
    velocity = raw_velocity if isinstance(raw_velocity, (int, float)) else float("nan")

    raw_torque = status[3] if status and len(status) > 3 else None
    torque = raw_torque if isinstance(raw_torque, (int, float)) else float("nan")
    
    angle_deg = math.degrees(angle_rad) if not math.isnan(angle_rad) else float("nan")
    if row_index < data_array.shape[0]:
        now = time.time()
        data_array[row_index] = [
            now,
            angle_rad,
            angle_deg,
            velocity,
            torque,
        ]
        row_index += 1


motor.disable()
bus.shutdown()

data_array = data_array[:row_index]

# Save data as CSV file (overwrite existing file)
with open(filename, "w", newline='') as f:
    writer = csv.writer(f)
    # Write CSV header
    writer.writerow(["timestamp", "angle_rad", "angle_deg", "velocity_rad_s", "torque_Nm"])
    for ts, angle_rad, angle_deg, velocity, torque in data_array:
        writer.writerow(
            [
                f"{ts:.6f}",
                "" if math.isnan(angle_rad) else f"{angle_rad:.6f}",
                "" if math.isnan(angle_deg) else f"{angle_deg:.6f}",
                "" if math.isnan(velocity) else f"{velocity:.6f}",
                "" if math.isnan(torque) else f"{torque:.6f}",
            ]
        )

print(f"Motor {args.motor_id}: data saved to {filename} ({row_index} rows)")
if duration > 0:
    print(f"Actual sampling rate: {row_index/duration:.1f} samples/second")

if row_index > 0:
    try:
        import matplotlib.pyplot as plt

        arr = data_array
        time_vals = arr[:, 0]
        angle_rad_vals = arr[:, 1]
        angle_deg_vals = arr[:, 2]
        velocity_vals = arr[:, 3]

        fig, (ax_top, ax_bottom) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        ax_top.plot(time_vals, angle_rad_vals, label="angle (rad)", color="tab:blue")
        ax_top.set_ylabel("angle (rad)", color="tab:blue")
        ax_top.tick_params(axis="y", labelcolor="tab:blue")

        ax_top_twin = ax_top.twinx()
        ax_top_twin.plot(time_vals, velocity_vals, label="velocity (rad/s)", color="tab:orange", alpha=0.7)
        ax_top_twin.set_ylabel("velocity (rad/s)", color="tab:orange")
        ax_top_twin.tick_params(axis="y", labelcolor="tab:orange")
        ax_top.set_title(f"Motor {args.motor_id} ramp response")

        ax_bottom.plot(time_vals, angle_deg_vals, color="tab:green")
        ax_bottom.set_xlabel("time (s)")
        ax_bottom.set_ylabel("angle (deg)")
        ax_bottom.grid(True)

        fig.tight_layout()
        plot_path = Path(filename).with_suffix(".png")
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        print(f"Plot saved to {plot_path}")
    except ImportError:
        print("matplotlib not installed; skipping plot generation.")
