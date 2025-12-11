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
parser.add_argument("--motor1-id", type=int, default=1, help="CAN ID for motor 1 (default: 1)")
parser.add_argument("--motor2-id", type=int, default=2, help="CAN ID for motor 2 (default: 2)")
parser.add_argument("--motor3-id", type=int, default=3, help="CAN ID for motor 3 antagonist 1 (default: 3)")
parser.add_argument("--motor4-id", type=int, default=4, help="CAN ID for motor 4 antagonist 2 (default: 4)")
parser.add_argument("--increment-deg1", type=float, default=40.0, help="Ramp amplitude in degrees from the current position (default: 40)")
parser.add_argument("--increment-deg2", type=float, default=40.0, help="Ramp amplitude in degrees from the current position (default: 40)")
parser.add_argument("--start-time", type=float, default=None, help="Shared start timestamp (seconds)")
args = parser.parse_args()

limit_speed = 0.1  # rad/s
duration = args.duration
increment_rad1 = math.radians(args.increment_deg1)
increment_rad2 = math.radians(args.increment_deg2)

name, ext = os.path.splitext(args.filename)
if not ext:
    ext = ".csv"
suffix1 = str(args.increment_deg1).replace(".", "p")
suffix2 = str(args.increment_deg2).replace(".", "p")
filename = f"{name}_m{args.motor1_id}_{suffix1}_m{args.motor2_id}_{suffix2}_{ext}"

bus = can.interface.Bus(interface="candle", channel=0, bitrate=1_000_000)
motor1 = CANMotorController(bus, motor_id=args.motor1_id, main_can_id=254)
motor2 = CANMotorController(bus, motor_id=args.motor2_id, main_can_id=254)
motor3 = CANMotorController(bus, motor_id=args.motor3_id, main_can_id=254)
motor4 = CANMotorController(bus, motor_id=args.motor4_id, main_can_id=254)

motor1.set_run_mode(motor1.RunModes.POSITION_MODE)
motor2.set_run_mode(motor2.RunModes.POSITION_MODE)
motor3.set_run_mode(motor3.RunModes.POSITION_MODE)
motor4.set_run_mode(motor4.RunModes.POSITION_MODE)
motor1.enable()
motor2.enable()
motor3.enable()
motor4.enable()


def valid_status(ctrl, expected_id):
    for _ in range(10):
        status = ctrl.get_motor_status()
        if status and status[0] == expected_id and status[1] is not None:
            return status
        time.sleep(0.001)
    return None



status1 = valid_status(motor1, args.motor1_id)
status2 = valid_status(motor2, args.motor2_id)
status3 = valid_status(motor3, args.motor3_id)
status4 = valid_status(motor4, args.motor4_id)
if status1 is None or status2 is None or status3 is None or status4 is None:
    raise RuntimeError("Initial motor status not available")

home1 = status1[1]
home2 = status2[1]
home3 = status3[1]
home4 = status4[1]

target1_rad = home1 + increment_rad1  # set target angle in rad for actuation for m1
target2_rad = home2 + increment_rad2  # set target angle in rad for actuation for m2
target3_rad = home3 - increment_rad1
target4_rad = home4 - increment_rad2
# print(f"home1: {home1} - increment: {increment_rad1} - Target 1: {target1_rad} ; home2: {home2} - increment: {increment_rad2} - target2: {target2_rad}")

if args.start_time is not None:
    print(f"Using shared start time {args.start_time:.6f}")
    start_time = args.start_time
else:
    start_time = time.time()

# print(f"Sleep time: {sleep_time*1000:.1f} ms (~{1/sleep_time:.1f} Hz)")

# preallocate - stima approssimativa
# estimated_freq = 1.0 / sleep_time
# expected_samples = int(duration * estimated_freq) + 10
# data_array = np.empty((expected_samples, 15), dtype=float)
samples = []
motor1.set_motor_position_control(limit_spd=limit_speed, loc_ref=target1_rad)
motor2.set_motor_position_control(limit_spd=limit_speed, loc_ref=target2_rad)
motor3.set_motor_position_control(limit_spd=limit_speed, loc_ref=target3_rad)
motor4.set_motor_position_control(limit_spd=limit_speed, loc_ref=target4_rad)

# Loop con sleep semplice
while time.time() - start_time < duration:
    now = time.time()
    
    status1 = valid_status(motor1, args.motor1_id)
    status2 = valid_status(motor2, args.motor2_id)
    status3 = valid_status(motor3, args.motor3_id)
    status4 = valid_status(motor4, args.motor4_id)
    if status1 is None or status2 is None or status3 is None or status4 is None:
        print("Skipping sample: wrong/missing frame")
        time.sleep(0.001)
        continue

    raw_angle1 = status1[1] if status1 and len(status1) > 1 else None
    angle_rad1 = raw_angle1 if isinstance(raw_angle1, (int, float)) else float("nan")
    abs_angle_rad1 = angle_rad1
    rel_angle_rad1 = angle_rad1-home1
    deg_angle1 = math.degrees(rel_angle_rad1)

    raw_velocity1 = status1[2] if status1 and len(status1) > 2 else None
    velocity1 = raw_velocity1 if isinstance(raw_velocity1, (int, float)) else float("nan")

    raw_torque1 = status1[3] if status1 and len(status1) > 3 else None
    torque1 = raw_torque1 if isinstance(raw_torque1, (int, float)) else float("nan")

    raw_angle2 = status2[1] if status2 and len(status2) > 1 else None
    angle_rad2 = raw_angle2 if isinstance(raw_angle2, (int, float)) else float("nan")
    abs_angle_rad2 = angle_rad2
    rel_angle_rad2 = angle_rad2-home2
    deg_angle2 = math.degrees(rel_angle_rad2)

    raw_velocity2 = status2[2] if status2 and len(status2) > 2 else None
    velocity2 = raw_velocity2 if isinstance(raw_velocity2, (int, float)) else float("nan")

    raw_torque2 = status2[3] if status2 and len(status2) > 3 else None
    torque2 = raw_torque2 if isinstance(raw_torque2, (int, float)) else float("nan")

    raw_angle3 = status3[1] if status3 and len(status3) > 1 else None
    angle_rad3 = raw_angle3 if isinstance(raw_angle3, (int, float)) else float("nan")
    abs_angle_rad3 = angle_rad3
    rel_angle_rad3 = angle_rad3-home3
    deg_angle3 = math.degrees(rel_angle_rad3)

    raw_velocity3 = status3[2] if status3 and len(status3) > 2 else None
    velocity3 = raw_velocity3 if isinstance(raw_velocity3, (int, float)) else float("nan")

    raw_torque3 = status3[3] if status3 and len(status3) > 3 else None
    torque3 = raw_torque3 if isinstance(raw_torque3, (int, float)) else float("nan")

    raw_angle4 = status4[1] if status4 and len(status4) > 1 else None
    angle_rad4 = raw_angle4 if isinstance(raw_angle4, (int, float)) else float("nan")
    abs_angle_rad4 = angle_rad4
    rel_angle_rad4 = angle_rad4-home4
    deg_angle4 = math.degrees(rel_angle_rad4)

    raw_velocity4 = status4[2] if status4 and len(status4) > 2 else None
    velocity4 = raw_velocity4 if isinstance(raw_velocity4, (int, float)) else float("nan")

    raw_torque4 = status4[3] if status4 and len(status4) > 3 else None
    torque4 = raw_torque4 if isinstance(raw_torque4, (int, float)) else float("nan")
    
    print(f"m1: cur: {abs_angle_rad1} - tg: {target1_rad} - m2: cur: {abs_angle_rad2} - tg: {target2_rad} - m3: cur: {abs_angle_rad3} - tg: {target3_rad} - m4: cur: {abs_angle_rad4} - tg: {target4_rad}")
    # Log data
    samples.append([
            now,
            home1,
            target1_rad,
            abs_angle_rad1,
            rel_angle_rad1,
            deg_angle1,
            velocity1,
            torque1,
            home2,
            target2_rad,
            abs_angle_rad2,
            rel_angle_rad2,
            deg_angle2,
            velocity2,
            torque2,
            home3,
            target3_rad,
            abs_angle_rad3,
            rel_angle_rad3,
            deg_angle3,
            velocity3,
            torque3,
            home4,
            target4_rad,
            abs_angle_rad4,
            rel_angle_rad4,
            deg_angle4,
            velocity4,
            torque4,
        ])
    


motor1.disable()
motor2.disable()
motor3.disable()
motor4.disable()
bus.shutdown()

samples_arr = np.array(samples, dtype=float)
with open(filename, "w", newline='') as fh:
    writer = csv.writer(fh)
    writer.writerow(
        [
            "timestamp",
            "home1_rad",
            "target1_rad",
            "abs_angle1_rad",
            "rel_angle1_rad",
            "deg_angle1",
            "velocity1_rad_s",
            "torque1_Nm",
            "home2_rad",
            "target2_rad",
            "abs_angle2_rad",
            "rel_angle2_rad",
            "deg_angle2",
            "velocity2_rad_s",
            "torque2_Nm",
            "home3_rad",
            "target3_rad",
            "abs_angle3_rad",
            "rel_angle3_rad",
            "deg_angle3",
            "velocity3_rad_s",
            "torque3_Nm",
            "home4_rad",
            "target4_rad",
            "abs_angle4_rad",
            "rel_angle4_rad",
            "deg_angle4",
            "velocity4_rad_s",
            "torque4_Nm",
        ]
    )
    for row in samples_arr:
        writer.writerow([f"{v:.6f}" for v in row])

# # data_array = data_array[:row_index]

# # # Save data as CSV file (overwrite existing file)
# # with open(filename, "w", newline='') as f:
# #     writer = csv.writer(f)
# #     # Write CSV header
# #     writer.writerow(["timestamp", "home1", "target1_rad", "abs_angle_rad1", "rel_angle_rad1","deg_angle1","velocity1","torque1","home2","target2_rad","abs_angle_rad2","rel_angle_rad2","deg_angle2","velocity2","torque2"])
# #     for ts, home1, target1_rad, abs_angle_rad1, rel_angle_rad1, deg_angle1, velocity1, torque1, home2,target2_rad,abs_angle_rad2,rel_angle_rad2,deg_angle2,velocity2,torque2 in data_array:
# #         writer.writerow(
# #             [
# #                 f"{ts:.6f}",
# #                 "" if math.isnan(home1) else f"{home1:.6f}",
# #                 "" if math.isnan(target1_rad) else f"{target1_rad:.6f}",
# #                 "" if math.isnan(abs_angle_rad1) else f"{abs_angle_rad1:.6f}",
# #                 "" if math.isnan(rel_angle_rad1) else f"{rel_angle_rad1:.6f}",
# #                 "" if math.isnan(deg_angle1) else f"{deg_angle1:.6f}",
# #                 "" if math.isnan(velocity1) else f"{velocity1:.6f}",
# #                 "" if math.isnan(torque1) else f"{torque1:.6f}",
# #                 "" if math.isnan(home2) else f"{home2:.6f}",
# #                 "" if math.isnan(target2_rad) else f"{target2_rad:.6f}",
# #                 "" if math.isnan(abs_angle_rad2) else f"{abs_angle_rad2:.6f}",
# #                 "" if math.isnan(rel_angle_rad2) else f"{rel_angle_rad2:.6f}",
# #                 "" if math.isnan(deg_angle2) else f"{deg_angle2:.6f}",
# #                 "" if math.isnan(velocity2) else f"{velocity2:.6f}",
# #                 "" if math.isnan(torque2) else f"{torque2:.6f}",
# #             ]
# #         )

print(f"Motor {args.motor1_id}: data saved to {filename} ({len(samples)} rows)")

try:
    import matplotlib.pyplot as plt

    arr = samples_arr
    time_vals = arr[:, 0]
    angle_rad_vals = arr[:, 4]
    angle_deg_vals = arr[:, 5]
    velocity_vals = arr[:, 6]

    fig, (ax_top, ax_bottom) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax_top.plot(time_vals, angle_rad_vals, label="angle (rad)", color="tab:blue")
    ax_top.set_ylabel("angle (rad)", color="tab:blue")
    ax_top.tick_params(axis="y", labelcolor="tab:blue")

    ax_top_twin = ax_top.twinx()
    ax_top_twin.plot(time_vals, velocity_vals, label="velocity (rad/s)", color="tab:orange", alpha=0.7)
    ax_top_twin.set_ylabel("velocity (rad/s)", color="tab:orange")
    ax_top_twin.tick_params(axis="y", labelcolor="tab:orange")
    ax_top.set_title(f"Motor {args.motor1_id} ramp response")

    ax_bottom.plot(time_vals, angle_deg_vals, color="tab:green")
    ax_bottom.set_xlabel("time (s)")
    ax_bottom.set_ylabel("angle (deg)")
    ax_bottom.grid(True)

    fig.tight_layout()
    plot_path = Path(filename).with_suffix(".png")
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved to {plot_path}")

    angle_rad_vals = arr[:, 11]
    angle_deg_vals = arr[:, 12]
    velocity_vals = arr[:, 13]

    fig, (ax_top, ax_bottom) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax_top.plot(time_vals, angle_rad_vals, label="angle (rad)", color="tab:blue")
    ax_top.set_ylabel("angle (rad)", color="tab:blue")
    ax_top.tick_params(axis="y", labelcolor="tab:blue")

    ax_top_twin = ax_top.twinx()
    ax_top_twin.plot(time_vals, velocity_vals, label="velocity (rad/s)", color="tab:orange", alpha=0.7)
    ax_top_twin.set_ylabel("velocity (rad/s)", color="tab:orange")
    ax_top_twin.tick_params(axis="y", labelcolor="tab:orange")
    ax_top.set_title(f"Motor {args.motor2_id} ramp response")

    ax_bottom.plot(time_vals, angle_deg_vals, color="tab:green")
    ax_bottom.set_xlabel("time (s)")
    ax_bottom.set_ylabel("angle (deg)")
    ax_bottom.grid(True)

    fig.tight_layout()
    plot_path = Path(filename).with_suffix("").with_name(Path(filename).stem + "_2").with_suffix(".png")
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved to {plot_path}")
except ImportError:
    print("matplotlib not installed; skipping plot generation.")


