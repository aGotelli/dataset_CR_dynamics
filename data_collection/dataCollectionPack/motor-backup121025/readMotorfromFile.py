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
    description="Actuate two Cybergear motors by replaying targets stored in a CSV file."
)
parser.add_argument("duration", type=float, help="Maximum duration allowed for the trajectory (seconds)")
parser.add_argument("filename", type=str, help="CSV filename to write telemetry to")
parser.add_argument("--motor1-id", type=int, default=3, help="CAN ID for motor 1 (default: 3)")
parser.add_argument("--motor2-id", type=int, default=4, help="CAN ID for motor 2 (default: 4)")
parser.add_argument("--start-time", type=float, default=None, help="Shared start timestamp (seconds)")
parser.add_argument("--input-file", type=str, default=str(Path(__file__).resolve().parent / "angela-8-12-25" / "-x-ytheta_seq.csv"), help="CSV file with two columns (rad) containing targets for motor3 and motor4")
parser.add_argument("--limit-speed", type=float, default=0.2, help="Motor position controller limit speed (rad/s)")
# parser.add_argument("--angle-tol-deg", type=float, default=0.2, help="Angle tolerance in degrees to mark a command completed")
# parser.add_argument( "--velocity-tol", type=float, default=0.1, help="Velocity tolerance (rad/s) to mark a command completed")
args = parser.parse_args()

if args.start_time is not None:
    print(f"Using shared start time {args.start_time:.6f}")
    start_time = args.start_time
else:
    start_time = time.time()

limit_speed = args.limit_speed
duration = args.duration
angle_tol = math.radians(0.2)
vel_tol = 0.1
name, ext = os.path.splitext(args.filename)
if not ext:
    ext = ".csv"
filename = f"{name}_from_file{ext}"

input_path = Path(args.input_file).expanduser()
if not input_path.is_absolute():
    input_path = (Path.cwd() / input_path).resolve()
if not input_path.is_file():
    raise FileNotFoundError(f"Input CSV '{input_path}' not found")

print(
    f"Motors {args.motor1_id} & {args.motor2_id}: replaying '{input_path.name}' "
    f"with limit speed {limit_speed:.2f} rad/s (max duration {duration:.2f}s), logging to {filename}"
)

trajectory = []
with input_path.open("r", newline="") as fh:
    reader = csv.reader(fh)
    for row_idx, row in enumerate(reader, start=1):
        if not row:
            continue
        if len(row) < 2:
            raise ValueError(f"Row {row_idx} in {input_path} does not contain at least two columns")
        target_motor1 = float(row[0])
        target_motor2 = float(row[1])
        trajectory.append((target_motor1, target_motor2))

if not trajectory:
    raise ValueError(f"No valid samples found in {input_path}")

print(f"Loaded {len(trajectory)} setpoints from {input_path}")

bus = can.interface.Bus(interface="candle", channel=0, bitrate=1_000_000)
motor1 = CANMotorController(bus, motor_id=args.motor1_id, main_can_id=254)
motor2 = CANMotorController(bus, motor_id=args.motor2_id, main_can_id=254)

motor1.set_run_mode(motor1.RunModes.POSITION_MODE)
motor2.set_run_mode(motor2.RunModes.POSITION_MODE)
motor1.enable()
motor2.enable()

status1 = None
status2 = None
attempts = 0
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
max_end_time = start_time + duration

samples = []
command_index = 0
timed_out = False
total_commands = len(trajectory)

for offset1, offset2 in trajectory:
    target1 = home1 + offset1
    target2 = home2 + offset2
    command_index += 1
    print(
        f"Command {command_index}/{total_commands}: "
        f"motor1 -> {math.degrees(target1):.2f} deg, motor2 -> {math.degrees(target2):.2f} deg"
    )
    motor1.set_motor_position_control(limit_spd=limit_speed, loc_ref=target1)
    motor2.set_motor_position_control(limit_spd=limit_speed, loc_ref=target2)

    while True:
        now = time.time()
        status1 = motor1.get_motor_status()
        status2 = motor2.get_motor_status()

        raw_angle1 = status1[1] if status1 and len(status1) > 1 else None
        angle_rad1 = raw_angle1 if isinstance(raw_angle1, (int, float)) else float("nan")
        angle_rad1 = angle_rad1 - home1

        raw_velocity1 = status1[2] if status1 and len(status1) > 2 else None
        velocity1 = raw_velocity1 if isinstance(raw_velocity1, (int, float)) else float("nan")

        raw_torque1 = status1[3] if status1 and len(status1) > 3 else None
        torque1 = raw_torque1 if isinstance(raw_torque1, (int, float)) else float("nan")

        raw_angle2 = status2[1] if status2 and len(status2) > 1 else None
        angle_rad2 = raw_angle2 if isinstance(raw_angle2, (int, float)) else float("nan")
        angle_rad2 = angle_rad2 - home2

        raw_velocity2 = status2[2] if status2 and len(status2) > 2 else None
        velocity2 = raw_velocity2 if isinstance(raw_velocity2, (int, float)) else float("nan")

        raw_torque2 = status2[3] if status2 and len(status2) > 3 else None
        torque2 = raw_torque2 if isinstance(raw_torque2, (int, float)) else float("nan")

        samples.append(
            [
                now,
                command_index,
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

        print(
            f"HOME1: {math.degrees(home1)}-cur{math.degrees(angle_rad1 + home1)}-target1-{math.degrees(target1)} "
            f"-- HOME2: {math.degrees(home2)}-cur{math.degrees(angle_rad2 + home2)} target2-{math.degrees(target2)}"
        )

        finish1 = abs(angle_rad1 + home1 - target1) <= angle_tol and abs(velocity1) <= vel_tol
        finish2 = abs(angle_rad2 + home2 - target2) <= angle_tol and abs(velocity2) <= vel_tol
        if finish1 and finish2:
            break

        if now >= max_end_time:
            print("STOPPED FOR TIME DURATION")
            timed_out = True
            break

    if timed_out:
        break

motor1.disable()
motor2.disable()
bus.shutdown()

if samples:
    samples_arr = np.array(samples, dtype=float)
else:
    samples_arr = np.empty((0, 10), dtype=float)

with open(filename, "w", newline="") as fh:
    writer = csv.writer(fh)
    writer.writerow(
        [
            "timestamp",
            "command_index",
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

if samples_arr.size == 0:
    print("No data available for plotting.")
else:
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
    ax1.plot(ts, target1_rad - home1, "--", color="tab:blue", label="target1 (rad)")
    ax1.plot(ts, angle1_rad, color="tab:blue", label="angle1 (rad)")
    ax1.set_ylabel("Motor 1 angle (rad)", color="tab:blue")
    ax1.tick_params(axis="y", labelcolor="tab:blue")
    ax1.grid(True)
    ax1.set_title("Motor 1 angles (rad)")

    ax2 = axes[1]
    ax2.plot(ts, target2_rad - home2, "--", color="tab:green", label="target2 (rad)")
    ax2.plot(ts, angle2_rad, color="tab:green", label="angle2 (rad)")
    ax2.set_ylabel("Motor 2 angle (rad)", color="tab:green")
    ax2.tick_params(axis="y", labelcolor="tab:green")
    ax2.grid(True)
    ax2.set_title("Motor 2 angles (rad)")

    ax3 = axes[2]
    ax3.plot(ts, velocity1, color="tab:orange", label="velocity1 (rad/s)")
    ax3.plot(ts, velocity2, color="tab:red", alpha=0.7, label="velocity2 (rad/s)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Velocity (rad/s)")
    ax3.grid(True)
    ax3.set_title("Motor velocities")
    ax3.legend(loc="upper right")

    fig.tight_layout()
    plot_path = Path(filename).with_suffix(".png")
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved to {plot_path}")
