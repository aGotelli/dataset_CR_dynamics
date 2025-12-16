import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import can

# Add project root to sys.path so that "sensors" can be imported
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from sensors.cybergear.pcan_cybergear import CANMotorController


def parse_args():
    parser = argparse.ArgumentParser(
        description="Actuate four Cybergear motors with target increments (radians) and log telemetry."
    )
    parser.add_argument("duration", type=float, help="Duration to run the data collection (in seconds)")
    parser.add_argument("filename", type=str, help="CSV filename to write")
    parser.add_argument("--motor1-id", type=int, default=1, help="CAN ID for motor 1 (default: 1)")
    parser.add_argument("--motor2-id", type=int, default=2, help="CAN ID for motor 2 (default: 2)")
    parser.add_argument("--motor3-id", type=int, default=3, help="CAN ID for motor 3 (default: 3)")
    parser.add_argument("--motor4-id", type=int, default=4, help="CAN ID for motor 4 (default: 4)")
    parser.add_argument("--theta1-rad", type=float, required=True, help="Increment (rad) for motor 1 relative to home")
    parser.add_argument("--theta2-rad", type=float, required=True, help="Increment (rad) for motor 2 relative to home")
    parser.add_argument("--theta3-rad", type=float, required=True, help="Increment (rad) for motor 3 relative to home")
    parser.add_argument("--theta4-rad", type=float, required=True, help="Increment (rad) for motor 4 relative to home")
    parser.add_argument("--start-time", type=float, default=None, help="Shared start timestamp (seconds)")
    return parser.parse_args()


def safe_status(ctrl, expected_id, retries=10, delay=0.001):
    """Read motor status with a few retries to avoid occasional empty frames."""
    for _ in range(retries):
        status = ctrl.get_motor_status()
        if status and status[0] == expected_id and status[1] is not None:
            return status
        time.sleep(delay)
    return None


def main():
    args = parse_args()
    limit_speed = 3  # rad/s
    duration = args.duration

    # name, ext = os.path.splitext(args.filename)
    # if not ext:
    #     ext = ".csv"

    # # # File name suffix with angles for easier bookkeeping
    # # def fmt_angle(val):
    # #     return str(val).replace(".", "p")

    # # suffix = f"m{args.motor1_id}_{fmt_angle(args.theta1_rad)}_" \
    # #          f"m{args.motor2_id}_{fmt_angle(args.theta2_rad)}_" \
    # #          f"m{args.motor3_id}_{fmt_angle(args.theta3_rad)}_" \
    # #          f"m{args.motor4_id}_{fmt_angle(args.theta4_rad)}"
    # # filename = f"{name}_{suffix}{ext}"
    filename=args.filename

    bus = can.interface.Bus(interface="candle", channel=0, bitrate=1_000_000)
    motor1 = CANMotorController(bus, motor_id=args.motor1_id, main_can_id=254)
    motor2 = CANMotorController(bus, motor_id=args.motor2_id, main_can_id=254)
    motor3 = CANMotorController(bus, motor_id=args.motor3_id, main_can_id=254)
    motor4 = CANMotorController(bus, motor_id=args.motor4_id, main_can_id=254)

    for m in (motor1, motor2, motor3, motor4):
        m.set_run_mode(m.RunModes.POSITION_MODE)
        m.enable()

    status1 = safe_status(motor1, args.motor1_id)
    status2 = safe_status(motor2, args.motor2_id)
    status3 = safe_status(motor3, args.motor3_id)
    status4 = safe_status(motor4, args.motor4_id)
    if not all([status1, status2, status3, status4]):
        raise RuntimeError("Initial motor status not available")

    home1, home2, home3, home4 = status1[1], status2[1], status3[1], status4[1]
    
    target1_rad = home1 + args.theta1_rad
    target3_rad = home3 + args.theta3_rad
    target2_rad = home2 + args.theta2_rad
    target4_rad = home4 + args.theta4_rad
    vel1= limit_speed
    vel2= limit_speed
    vel3= limit_speed
    vel4= limit_speed
    if args.theta1_rad>0: #m1 push
        target1_rad = home1 + args.theta1_rad*0.85
        vel1=limit_speed*0.85
        vel3=limit_speed
    elif args.theta1_rad<0: #m1 pull
        target3_rad = home3 + args.theta3_rad*0.85
        vel1=limit_speed
        vel3=limit_speed*0.85

    if args.theta2_rad>0: #m2 pull
        target4_rad = home4 + args.theta4_rad*0.85
        vel4=limit_speed*0.85
        vel2=limit_speed
    elif args.theta2_rad<0: #m2 push
        target2_rad = home2 + args.theta2_rad*0.85
        vel4=limit_speed
        vel2=limit_speed*0.85

   
    
    if args.start_time is not None:
        print(f"Using shared start time {args.start_time:.6f}")
        start_time = args.start_time
    else:
        start_time = time.time()

    samples = []
    motor1.set_motor_position_control(limit_spd=vel1, loc_ref=target1_rad)
    motor2.set_motor_position_control(limit_spd=vel2, loc_ref=target2_rad)
    motor3.set_motor_position_control(limit_spd=vel3, loc_ref=target3_rad)
    motor4.set_motor_position_control(limit_spd=vel4, loc_ref=target4_rad)

    while time.time() - start_time < duration:
        now = time.time()
        status1 = safe_status(motor1, args.motor1_id)
        status2 = safe_status(motor2, args.motor2_id)
        status3 = safe_status(motor3, args.motor3_id)
        status4 = safe_status(motor4, args.motor4_id)
        if not all([status1, status2, status3, status4]):
            print("Skipping sample: wrong/missing frame")
            time.sleep(0.001)
            continue

        def parse_status(status, home):
            raw_angle = status[1] if len(status) > 1 else None
            angle_rad = raw_angle if isinstance(raw_angle, (int, float)) else float("nan")
            rel_angle = angle_rad - home
            deg_angle = math.degrees(rel_angle)

            raw_vel = status[2] if len(status) > 2 else None
            velocity = raw_vel if isinstance(raw_vel, (int, float)) else float("nan")

            raw_torque = status[3] if len(status) > 3 else None
            torque = raw_torque if isinstance(raw_torque, (int, float)) else float("nan")
            return angle_rad, rel_angle, deg_angle, velocity, torque

        angle_rad1, rel_angle_rad1, deg_angle1, velocity1, torque1 = parse_status(status1, home1)
        angle_rad2, rel_angle_rad2, deg_angle2, velocity2, torque2 = parse_status(status2, home2)
        angle_rad3, rel_angle_rad3, deg_angle3, velocity3, torque3 = parse_status(status3, home3)
        angle_rad4, rel_angle_rad4, deg_angle4, velocity4, torque4 = parse_status(status4, home4)

        # print(
        #     f"m1: cur: {angle_rad1} - tg: {target1_rad} - "
        #     f"m2: cur: {angle_rad2} - tg: {target2_rad} - "
        #     f"m3: cur: {angle_rad3} - tg: {target3_rad} - "
        #     f"m4: cur: {angle_rad4} - tg: {target4_rad}"
        # )

        samples.append(
            [
                now,
                home1,
                target1_rad,
                angle_rad1,
                rel_angle_rad1,
                deg_angle1,
                velocity1,
                torque1,
                home2,
                target2_rad,
                angle_rad2,
                rel_angle_rad2,
                deg_angle2,
                velocity2,
                torque2,
                home3,
                target3_rad,
                angle_rad3,
                rel_angle_rad3,
                deg_angle3,
                velocity3,
                torque3,
                home4,
                target4_rad,
                angle_rad4,
                rel_angle_rad4,
                deg_angle4,
                velocity4,
                torque4,
            ]
        )

    # Return to home positions
    time.sleep(1)
    motor1.set_motor_position_control(limit_spd=limit_speed, loc_ref=home1)
    motor2.set_motor_position_control(limit_spd=limit_speed, loc_ref=home2)
    motor3.set_motor_position_control(limit_spd=limit_speed, loc_ref=home3)
    motor4.set_motor_position_control(limit_spd=limit_speed, loc_ref=home4)
    time.sleep(5)
    motor1.disable()
    motor2.disable()
    motor3.disable()
    motor4.disable()
    bus.shutdown()

    samples_arr = np.array(samples, dtype=float)
    with open(filename, "w", newline="") as fh:
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

    print(f"Motors {args.motor1_id},{args.motor2_id},{args.motor3_id},{args.motor4_id} data saved to {filename} ({len(samples)} rows)")

    try:
        import matplotlib.pyplot as plt

        arr = samples_arr
        time_vals = arr[:, 0]

        def plot_motor(idx_start, motor_id, suffix=""):
            # Block layout: [home, target, abs, rel, deg, vel, torque]
            angle_rad_vals = arr[:, idx_start + 2]
            angle_deg_vals = arr[:, idx_start + 4]
            velocity_vals = arr[:, idx_start + 5]

            fig, (ax_top, ax_bottom) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
            ax_top.plot(time_vals, angle_rad_vals, label="angle (rad)", color="tab:blue")
            ax_top.set_ylabel("angle (rad)", color="tab:blue")
            ax_top.tick_params(axis="y", labelcolor="tab:blue")

            ax_top_twin = ax_top.twinx()
            ax_top_twin.plot(time_vals, velocity_vals, label="velocity (rad/s)", color="tab:orange", alpha=0.7)
            ax_top_twin.set_ylabel("velocity (rad/s)", color="tab:orange")
            ax_top_twin.tick_params(axis="y", labelcolor="tab:orange")
            ax_top.set_title(f"Motor {motor_id} ramp response")

            ax_bottom.plot(time_vals, angle_deg_vals, color="tab:green")
            ax_bottom.set_xlabel("time (s)")
            ax_bottom.set_ylabel("angle (deg)")
            ax_bottom.grid(True)

            fig.tight_layout()
            plot_path = Path(filename).with_suffix("").with_name(Path(filename).stem + suffix).with_suffix(".png")
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)
            print(f"Plot saved to {plot_path}")

        # Motor 1 and 2 plots (same as original script behaviour)
        plot_motor(1, args.motor1_id, "")
        plot_motor(8, args.motor2_id, "_2")
        plot_motor(15, args.motor3_id, "_3")
        plot_motor(22, args.motor4_id, "_4")

    except ImportError:
        print("matplotlib not installed; skipping plot generation.")


if __name__ == "__main__":
    main()
