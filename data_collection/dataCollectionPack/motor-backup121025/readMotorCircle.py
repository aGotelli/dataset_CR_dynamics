import argparse
import csv
import math
import sys
import time
from pathlib import Path
import os
import numpy as np
import can

# Make sure we can import the Cybergear driver
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from sensors.cybergear.pcan_cybergear import CANMotorController 

parser = argparse.ArgumentParser(
    description="Actuate two Cybergear motors with staggered ramps and log telemetry."
)
parser.add_argument("duration", type=float, help="Total duration of the sequence (seconds)")
parser.add_argument("filename", type=str, help="CSV filename to write")
parser.add_argument("--motor1-id", type=int, default=3, help="CAN ID for motor 1 (default: 3)")
parser.add_argument("--motor2-id", type=int, default=4, help="CAN ID for motor 2 (default: 4)")
parser.add_argument("--radius", type=float, default=45.0, help="Radius of circle in degrees")
parser.add_argument("--start-time", type=float, default=None, help="Shared start timestamp (seconds)")
args = parser.parse_args()

if args.start_time is not None:
    print(f"Using shared start time {args.start_time:.6f}")
    start_time = args.start_time
else:
    start_time = time.time()


ramp_rad = math.radians(args.radius)
limit_speed = 0.2
duration = args.duration
per_phase = duration / 4.0
angle_tol = math.radians(0.2)
vel_tol = 0.1
name, ext = os.path.splitext(args.filename)
if not ext:
    ext = ".csv"
suffix = str(args.radius).replace(".", "p")
filename = f"{name}_circle_radius_{suffix}{ext}"

print(
    f"Motors {args.motor1_id} & {args.motor2_id}: circle sequence with radius ±{args.radius:.1f}° "
    f"over {duration:.2f}s (phase {per_phase:.2f}s each), logging to {filename}"
)

bus = can.interface.Bus(interface="candle", channel=0, bitrate=1_000_000)
motor1 = CANMotorController(bus, motor_id=args.motor1_id, main_can_id=254)
motor2 = CANMotorController(bus, motor_id=args.motor2_id, main_can_id=254)

motor1.set_run_mode(motor1.RunModes.POSITION_MODE)
motor2.set_run_mode(motor2.RunModes.POSITION_MODE)
motor1.enable()
motor2.enable()

status1=None
status2=None
attempts=0
while status1 is None and status2 is None and attempts < 20:
    s1 = motor1.get_motor_status()
    s2 = motor2.get_motor_status()
    print(s1)
    print(s2)
    if s1[1] is not None and s2[1] is not None:
        status1 = s1
        status2 = s2
        break
    attempts += 1
if status1 is None or status2 is None:
    raise RuntimeError("Initial motor status not available")

home1 = status1[1]
home2 = status2[1]

phases = [
        (home1 - ramp_rad, home2),            # motor 1 up
        (home1 - ramp_rad, home2 + ramp_rad), # motor 2 up
        (home1, home2 + ramp_rad),            # motor 1 down
        (home1, home2)                        # motor 2 down
        ]

samples = []
phase_index = 0
for target1, target2 in phases:
    print(f"HOME1: {math.degrees(home1)} target1-{math.degrees(target1)} -- HOME2: {math.degrees(home2)} target2-{math.degrees(target2)}")
    phase_index += 1
    phase_start = time.time()
    # initial command
    motor1.set_motor_position_control(limit_spd=limit_speed, loc_ref=target1)
    motor2.set_motor_position_control(limit_spd=limit_speed, loc_ref=target2)
    print(phase_index)

    while True:
        now = time.time()
        status1 = motor1.get_motor_status()
        status2 = motor2.get_motor_status()

        raw_angle1 = status1[1] if status1 and len(status1) > 1 else None
        angle_rad1 = raw_angle1 if isinstance(raw_angle1, (int, float)) else float("nan")
        angle_rad1 = angle_rad1-home1

        raw_velocity1 = status1[2] if status1 and len(status1) > 2 else None
        velocity1 = raw_velocity1 if isinstance(raw_velocity1, (int, float)) else float("nan")

        raw_torque1 = status1[3] if status1 and len(status1) > 3 else None
        torque1 = raw_torque1 if isinstance(raw_torque1, (int, float)) else float("nan")
    
        raw_angle2 = status2[1] if status2 and len(status2) > 1 else None
        angle_rad2 = raw_angle2 if isinstance(raw_angle2, (int, float)) else float("nan")
        angle_rad2 = angle_rad2-home2

        raw_velocity2 = status2[2] if status2 and len(status2) > 2 else None
        velocity2 = raw_velocity2 if isinstance(raw_velocity2, (int, float)) else float("nan")

        raw_torque2 = status2[3] if status2 and len(status2) > 3 else None
        torque2 = raw_torque2 if isinstance(raw_torque2, (int, float)) else float("nan")

        samples.append(
            [
                now,
                phase_index,
                target1,
                angle_rad1,
                velocity1,
                torque1,
                target2,
                angle_rad2,
                velocity2,
                torque2,
            ]
        )
        # print(f"{angle_rad1+home1} vs {target1}")
        print(f"HOME1: {math.degrees(home1)}-cur{math.degrees(angle_rad1+home1)}-target1-{math.degrees(target1)} -- HOME2: {math.degrees(home2)}-cur{math.degrees(angle_rad2+home2)} target2-{math.degrees(target2)}")

        finish1 = abs(angle_rad1+home1 - target1) <= angle_tol and abs(velocity1) <= vel_tol
        finish2 = abs(angle_rad2+home2 - target2) <= angle_tol and abs(velocity2) <= vel_tol

        if finish1 and finish2:
            break
        if time.time()-start_time >= duration:
            print("STOPPED FOR TIME DURATION")
            break

motor1.disable()
motor2.disable()
bus.shutdown()


samples_arr = np.array(samples, dtype=float)
with open(filename, "w", newline='') as fh:
    writer = csv.writer(fh)
    writer.writerow(
        [
            "timestamp",
            "phase_index",
            "target1_rad",
            "angle1_rad",
            "velocity1_rad_s",
            "torque1_Nm",
            "target2_rad",
            "angle2_rad",
            "velocity2_rad_s",
            "torque2_Nm",
        ]
    )
    for row in samples_arr:
        writer.writerow([f"{v:.6f}" for v in row])

print(f"Sequence completed. Logged {len(samples)} samples to {filename}")

import matplotlib.pyplot as plt

ts = samples_arr[:, 0]
target1_rad = samples_arr[:, 2]
angle1_rad = samples_arr[:, 3]
velocity1 = samples_arr[:, 4]
target2_rad = samples_arr[:, 6]
angle2_rad = samples_arr[:, 7]
velocity2 = samples_arr[:, 8]

fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

ax1 = axes[0]
ax1.plot(ts, target1_rad-home1, '--', color='tab:blue', label='target1 (rad)')
ax1.plot(ts, angle1_rad, color='tab:blue', label='angle1 (rad)')
ax1.set_ylabel('Motor 1 angle (rad)', color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax1.grid(True)
ax1.set_title('Motor 1 angles (rad)')

ax2 = axes[1]
ax2.plot(ts, target2_rad-home2, '--', color='tab:green', label='target1 (deg)')
ax2.plot(ts, angle2_rad, color='tab:green', label='angle1 (deg)')
ax2.set_ylabel('Motor 2 angle (rad)', color='tab:green')
ax2.tick_params(axis='y', labelcolor='tab:green')
ax2.grid(True)
ax2.set_title('Motor 2 angles (rad)')

ax3 = axes[2]
ax3.plot(ts, velocity1, color='tab:orange', label='velocity1 (rad/s)')
ax3.plot(ts, velocity2, color='tab:red', alpha=0.7, label='velocity2 (rad/s)')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Velocity (rad/s)')
ax3.grid(True)
ax3.set_title('Motor velocities')
ax3.legend(loc='upper right')

fig.tight_layout()
plot_path = Path(filename).with_suffix('.png')
fig.savefig(plot_path, dpi=150)
plt.close(fig)
print(f'Plot saved to {plot_path}')

