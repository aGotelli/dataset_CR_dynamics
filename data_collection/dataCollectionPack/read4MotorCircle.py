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
    description="Actuate 4 Cybergear motors in antagonistic pairs (1&3, 2&4) following circle trajectory."
)
parser.add_argument("duration", type=float, help="Total duration of the sequence (seconds)")
parser.add_argument("filename", type=str, help="CSV filename to write")
parser.add_argument("--motor1-id", type=int, default=1, help="CAN ID for motor 1 (default: 1)")
parser.add_argument("--motor2-id", type=int, default=2, help="CAN ID for motor 2 (default: 2)")
parser.add_argument("--motor3-id", type=int, default=3, help="CAN ID for motor 3 antagonist 1 (default: 3)")
parser.add_argument("--motor4-id", type=int, default=4, help="CAN ID for motor 4 antagonist 2 (default: 4)")
parser.add_argument("--radius", type=float, default=45.0, help="Radius of circle in degrees")
parser.add_argument("--start-time", type=float, default=None, help="Shared start timestamp (seconds)")
args = parser.parse_args()

if args.start_time is not None:
    print(f"Using shared start time {args.start_time:.6f}")
    start_time = args.start_time
else:
    start_time = time.time()


ramp_rad = math.radians(args.radius)
limit_speed = 1  # Much slower for smooth transitions
duration = args.duration
angle_tol = math.radians(0.5)  # Very precise: 0.05 degrees
vel_tol = 0.05  # Also tighten velocity tolerance
name, ext = os.path.splitext(args.filename)
if not ext:
    ext = ".csv"
suffix = str(args.radius).replace(".", "p")
filename = f"{name}_circle_radius_{suffix}{ext}"

print(
    f"4-Motor antagonistic circle: Motors {args.motor1_id}&{args.motor3_id} (pair1), {args.motor2_id}&{args.motor4_id} (pair2)"
    f" with radius ±{args.radius:.1f}° over {duration:.2f}s, logging to {filename}"
)

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

print(f"DEBUG - Home positions:")
print(f"  Motor 1: {home1:.3f} rad = {math.degrees(home1):.1f}°")
print(f"  Motor 2: {home2:.3f} rad = {math.degrees(home2):.1f}°")
print(f"  Motor 3: {home3:.3f} rad = {math.degrees(home3):.1f}°")
print(f"  Motor 4: {home4:.3f} rad = {math.degrees(home4):.1f}°")
print()

# Define circle phases with antagonistic pairs: 1&3 antagonistic, 2&4 antagonistic
# Phase logic: when motor moves +rad, its antagonist moves -rad automatically
# # phases = [
# #     # Phase 1: Motor 1 extends (+), Motor 3 compensates (-), others at home
# #     (home1 + ramp_rad,  home2,              home3 - ramp_rad,   home4           ),
    
# #     # Phase 2: Motor 2 extends (+), Motor 4 compensates (-), Motor 1&3 stay extended/retracted  
# #     (home1 + ramp_rad,  home2 + ramp_rad,   home3 - ramp_rad,   home4 - ramp_rad),
    
# #     # Phase 3: Motor 1 retracts (-), Motor 3 compensates (+), Motor 2&4 stay extended/retracted
# #     (home1 - ramp_rad,  home2 + ramp_rad,   home3 + ramp_rad,   home4 - ramp_rad),
    
# #     # Phase 4: Motor 2 retracts (-), Motor 4 compensates (+), all return towards home
# #     (home1 - ramp_rad,  home2 - ramp_rad,   home3 + ramp_rad,   home4 + ramp_rad)
# # ]

# Creat a pattern for circle
sqrt_2 = math.sqrt(2)
home_positions = np.array([home1, home2, home3, home4])

# Define offset vectors for each phase
# (+ push, + pull, + push, + pull)
offset_vectors = [
    np.array([ramp_rad*0.85, ramp_rad*0.1, -ramp_rad, ramp_rad*0.1]),
    np.array([ramp_rad/sqrt_2*0.8, ramp_rad/sqrt_2, -ramp_rad/sqrt_2, -ramp_rad/sqrt_2*0.8]),
    np.array([-ramp_rad*0.1, ramp_rad, -ramp_rad*0.1, -ramp_rad*0.85]),
    np.array([-ramp_rad/sqrt_2, ramp_rad/sqrt_2, ramp_rad/sqrt_2*0.8, -ramp_rad/sqrt_2*0.8]),
    np.array([-ramp_rad, ramp_rad*0.1, ramp_rad*0.85, ramp_rad*0.1]),
    np.array([-ramp_rad/sqrt_2, -ramp_rad/sqrt_2*0.8, ramp_rad/sqrt_2*0.8, ramp_rad/sqrt_2]),
    np.array([-ramp_rad*0.1, -ramp_rad*0.85, -ramp_rad*0.1, ramp_rad]),
    np.array([ramp_rad/sqrt_2*0.8, -ramp_rad/sqrt_2*0.8, -ramp_rad/sqrt_2, ramp_rad/sqrt_2]),
]

# Generate phases using vector addition
phases = [tuple(home_positions + offset) for offset in offset_vectors]

samples = []
phase_index = 0
current_phase = 0

# Main loop - continue until duration is reached
while time.time() - start_time < duration:
    target1, target2, target3, target4 = phases[current_phase]
    phase_index += 1
    # print(f"Phase {phase_index}: Pair 1&3: M1={math.degrees(target1):.1f}°, M3={math.degrees(target3):.1f}° | Pair 2&4: M2={math.degrees(target2):.1f}°, M4={math.degrees(target4):.1f}°")
    # print(f"  Antagonistic check: M1+M3={math.degrees(target1-home1):.1f}+{math.degrees(target3-home3):.1f}={math.degrees((target1-home1)+(target3-home3)):.1f}°")
    # print(f"  Antagonistic check: M2+M4={math.degrees(target2-home2):.1f}+{math.degrees(target4-home4):.1f}={math.degrees((target2-home2)+(target4-home4)):.1f}°")
    phase_start = time.time()
    # Send commands with speed limit
    
    motor1.set_motor_position_control(limit_spd=limit_speed, loc_ref=target1)
    motor2.set_motor_position_control(limit_spd=limit_speed, loc_ref=target2)
    motor3.set_motor_position_control(limit_spd=limit_speed, loc_ref=target3)
    motor4.set_motor_position_control(limit_spd=limit_speed, loc_ref=target4)

    print(phase_index)

    while True:
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

        raw_velocity1 = status1[2] if status1 and len(status1) > 2 else None
        velocity1 = raw_velocity1 if isinstance(raw_velocity1, (int, float)) else float("nan")

        raw_torque1 = status1[3] if status1 and len(status1) > 3 else None
        torque1 = raw_torque1 if isinstance(raw_torque1, (int, float)) else float("nan")
    
        raw_angle2 = status2[1] if status2 and len(status2) > 1 else None
        angle_rad2 = raw_angle2 if isinstance(raw_angle2, (int, float)) else float("nan")
        abs_angle_rad2 = angle_rad2
        rel_angle_rad2 = angle_rad2-home2

        raw_velocity2 = status2[2] if status2 and len(status2) > 2 else None
        velocity2 = raw_velocity2 if isinstance(raw_velocity2, (int, float)) else float("nan")

        raw_torque2 = status2[3] if status2 and len(status2) > 3 else None
        torque2 = raw_torque2 if isinstance(raw_torque2, (int, float)) else float("nan")

        raw_angle3 = status3[1] if status3 and len(status3) > 1 else None
        angle_rad3 = raw_angle3 if isinstance(raw_angle3, (int, float)) else float("nan")
        abs_angle_rad3 = angle_rad3
        rel_angle_rad3 = angle_rad3-home3

        raw_velocity3 = status3[2] if status3 and len(status3) > 2 else None
        velocity3 = raw_velocity3 if isinstance(raw_velocity3, (int, float)) else float("nan")

        raw_torque3 = status3[3] if status3 and len(status3) > 3 else None
        torque3 = raw_torque3 if isinstance(raw_torque3, (int, float)) else float("nan")

        raw_angle4 = status4[1] if status4 and len(status4) > 1 else None
        angle_rad4 = raw_angle4 if isinstance(raw_angle4, (int, float)) else float("nan")
        abs_angle_rad4 = angle_rad4
        rel_angle_rad4 = angle_rad4-home4

        raw_velocity4 = status4[2] if status4 and len(status4) > 2 else None
        velocity4 = raw_velocity4 if isinstance(raw_velocity4, (int, float)) else float("nan")

        raw_torque4 = status4[3] if status4 and len(status4) > 3 else None
        torque4 = raw_torque4 if isinstance(raw_torque4, (int, float)) else float("nan")



        samples.append(
            [
                now,
                phase_index,
                home1,
                target1,
                abs_angle_rad1,
                rel_angle_rad1,
                velocity1,
                torque1,
                home2,
                target2,
                abs_angle_rad2,
                rel_angle_rad2,
                velocity2,
                torque2,
                home3,
                target3,
                abs_angle_rad3,
                rel_angle_rad3,
                velocity3,
                torque3,
                home4,
                target4,
                abs_angle_rad4,
                rel_angle_rad4,
                velocity4,
                torque4,
            ]
        )
        # print(f"{angle_rad1+home1} vs {target1}")
        print(f"m1: cur: {abs_angle_rad1} - tg: {target1} - m2: cur: {abs_angle_rad2} - tg: {target2} - m3: cur: {abs_angle_rad3} - tg: {target3} - m4: cur: {abs_angle_rad4} - tg: {target4}")

        finish1 = abs(abs_angle_rad1 - target1) <= angle_tol #and abs(velocity1) <= vel_tol
        finish2 = abs(abs_angle_rad2 - target2) <= angle_tol #and abs(velocity2) <= vel_tol
        finish3 = abs(abs_angle_rad3 - target3) <= angle_tol #and abs(velocity3) <= vel_tol
        finish4 = abs(abs_angle_rad4 - target4) <= angle_tol #and abs(velocity4) <= vel_tol


        if finish1 and finish2 and finish3 and finish4:
            print(f"Phase {phase_index} completed - moving to next phase")
            # Move to next phase in the cycle (loop back to 0 after phase 3)
            current_phase = (current_phase + 1) % len(phases)
            break
            
        if time.time() - start_time >= duration:
            
            print("STOPPED FOR TIME DURATION")
            break
time.sleep(1)

# Moving all motors back to home position
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
with open(filename, "w", newline='') as fh:
    writer = csv.writer(fh)
    writer.writerow(
        [
            "timestamp",
            "phase_index",
            "home1",
            "target1_rad",
            "abs_angle1_rad",
            "rel_angle1_rad",
            "velocity1_rad_s",
            "torque1_Nm",
            "home2",
            "target2_rad",
            "abs_angle2_rad",
            "rel_angle2_rad",
            "velocity2_rad_s",
            "torque2_Nm",
            "home3",
            "target3_rad",
            "abs_angle3_rad",
            "rel_angle3_rad",
            "velocity3_rad_s",
            "torque3_Nm",
            "home4",
            "target4_rad",
            "abs_angle4_rad",
            "rel_angle4_rad",
            "velocity4_rad_s",
            "torque4_Nm",
        ]
    )
    for row in samples_arr:
        writer.writerow([f"{v:.6f}" for v in row])

print(f"Sequence completed. Logged {len(samples)} samples to {filename}")

import matplotlib.pyplot as plt

# Extract data for all 4 motors
ts = samples_arr[:, 0]
phase_indices = samples_arr[:, 1]
home1 = samples_arr[:, 2]
target1_rad = samples_arr[:, 3]
angle1_rad = samples_arr[:, 4]  # This is rel_angle1_rad
abs_angle1_rad = samples_arr[:, 5]  # This is abs_angle1_rad
velocity1 = samples_arr[:, 6]
torque1 = samples_arr[:, 7]
home2 = samples_arr[:, 8]
target2_rad = samples_arr[:, 9]
angle2_rad = samples_arr[:, 10]  # This is rel_angle2_rad
abs_angle2_rad = samples_arr[:, 11]  # This is abs_angle2_rad
velocity2 = samples_arr[:, 12]
torque2 = samples_arr[:, 13]
home3 = samples_arr[:, 14]
target3_rad = samples_arr[:, 15]
angle3_rad = samples_arr[:, 16]  # This is rel_angle3_rad
abs_angle3_rad = samples_arr[:, 17]  # This is abs_angle3_rad
velocity3 = samples_arr[:, 18]
torque3 = samples_arr[:, 19]
home4 = samples_arr[:, 20]
target4_rad = samples_arr[:, 21]
angle4_rad = samples_arr[:, 22]  # This is rel_angle4_rad
abs_angle4_rad = samples_arr[:, 23]  # This is abs_angle4_rad
velocity4 = samples_arr[:, 24]
torque4 = samples_arr[:, 25]

# Find unique phases from the phase_index column
unique_phases = np.unique(phase_indices)
num_phases = len(unique_phases)

print(f"Found {num_phases} phases: {unique_phases}")

# Create plot: one subplot per phase + velocity + torque
fig, axes = plt.subplots(num_phases + 2, 1, figsize=(15, 4*(num_phases + 2)), sharex=True)

# Handle single subplot case
if num_phases + 2 == 1:
    axes = [axes]
elif not hasattr(axes, '__len__'):
    axes = [axes]

# Plot each phase
for i, phase in enumerate(unique_phases):
    # Get data for this phase
    phase_mask = phase_indices == phase
    t_phase = ts[phase_mask]
    
    ax = axes[i]
    
    # Home positions (constant lines) - use first value since they should be constant
    home1_val = home1[phase_mask][0]
    home2_val = home2[phase_mask][0]
    home3_val = home3[phase_mask][0]
    home4_val = home4[phase_mask][0]
    
    ax.axhline(y=np.degrees(home1_val), color='tab:blue', linestyle=':', linewidth=2, alpha=0.8, 
               label=f'M1 home ({np.degrees(home1_val):.1f}°)')
    ax.axhline(y=np.degrees(home2_val), color='tab:green', linestyle=':', linewidth=2, alpha=0.8, 
               label=f'M2 home ({np.degrees(home2_val):.1f}°)')
    ax.axhline(y=np.degrees(home3_val), color='tab:red', linestyle=':', linewidth=2, alpha=0.8, 
               label=f'M3 home ({np.degrees(home3_val):.1f}°)')
    ax.axhline(y=np.degrees(home4_val), color='tab:orange', linestyle=':', linewidth=2, alpha=0.8, 
               label=f'M4 home ({np.degrees(home4_val):.1f}°)')
    
    # Target positions (dashed lines)
    ax.plot(t_phase, np.degrees(target1_rad[phase_mask]), '--', color='tab:blue', 
            linewidth=2, alpha=0.9, label='M1 target')
    ax.plot(t_phase, np.degrees(target2_rad[phase_mask]), '--', color='tab:green', 
            linewidth=2, alpha=0.9, label='M2 target')
    ax.plot(t_phase, np.degrees(target3_rad[phase_mask]), '--', color='tab:red', 
            linewidth=2, alpha=0.9, label='M3 target')
    ax.plot(t_phase, np.degrees(target4_rad[phase_mask]), '--', color='tab:orange', 
            linewidth=2, alpha=0.9, label='M4 target')
    
    # Current positions (solid thick lines) - using absolute angles
    ax.plot(t_phase, np.degrees(abs_angle1_rad[phase_mask]), color='tab:blue', 
            linewidth=3, label='M1 current')
    ax.plot(t_phase, np.degrees(abs_angle2_rad[phase_mask]), color='tab:green', 
            linewidth=3, label='M2 current')
    ax.plot(t_phase, np.degrees(abs_angle3_rad[phase_mask]), color='tab:red', 
            linewidth=3, label='M3 current')
    ax.plot(t_phase, np.degrees(abs_angle4_rad[phase_mask]), color='tab:orange', 
            linewidth=3, label='M4 current')
    
    # Calculate errors for this phase
    error1 = np.mean(np.abs(np.degrees(target1_rad[phase_mask] - abs_angle1_rad[phase_mask])))
    error2 = np.mean(np.abs(np.degrees(target2_rad[phase_mask] - abs_angle2_rad[phase_mask])))
    error3 = np.mean(np.abs(np.degrees(target3_rad[phase_mask] - abs_angle3_rad[phase_mask])))
    error4 = np.mean(np.abs(np.degrees(target4_rad[phase_mask] - abs_angle4_rad[phase_mask])))
    
    ax.set_title(f'Phase {int(phase)} - Motor Positions\nErrors: M1={error1:.1f}° M2={error2:.1f}° M3={error3:.1f}° M4={error4:.1f}°')
    ax.set_ylabel('Angle (degrees)')
    ax.grid(True, alpha=0.3)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=9)

# All velocities together
ax_vel = axes[num_phases]
ax_vel.plot(ts, velocity1, color='tab:blue', linewidth=2, label='M1 velocity')
ax_vel.plot(ts, velocity2, color='tab:green', linewidth=2, label='M2 velocity')
ax_vel.plot(ts, velocity3, color='tab:red', linewidth=2, label='M3 velocity')
ax_vel.plot(ts, velocity4, color='tab:orange', linewidth=2, label='M4 velocity')
ax_vel.set_ylabel('Velocity (rad/s)')
ax_vel.set_title('All Motor Velocities')
ax_vel.grid(True, alpha=0.3)
ax_vel.legend(loc='upper right')

# All torques together
ax_torque = axes[num_phases + 1]
ax_torque.plot(ts, torque1, color='tab:blue', linewidth=2, label='M1 torque')
ax_torque.plot(ts, torque2, color='tab:green', linewidth=2, label='M2 torque')
ax_torque.plot(ts, torque3, color='tab:red', linewidth=2, label='M3 torque')
ax_torque.plot(ts, torque4, color='tab:orange', linewidth=2, label='M4 torque')
ax_torque.set_xlabel('Time (s)')
ax_torque.set_ylabel('Torque (Nm)')
ax_torque.set_title('All Motor Torques')
ax_torque.grid(True, alpha=0.3)
ax_torque.legend(loc='upper right')

fig.tight_layout()
plot_path = Path(filename).with_suffix('.png')
fig.savefig(plot_path, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f'Phase-based motor analysis plot saved to {plot_path}')
print(f'Plot shows {num_phases} phases + velocity + torque')
