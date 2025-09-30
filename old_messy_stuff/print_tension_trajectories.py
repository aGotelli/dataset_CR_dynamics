"""
Text-Based Visualization of Tension-Controlled Trajectories
"""

import math

def print_trajectory_table():
    """Print a table showing the trajectory behavior over time"""
    
    print("🎯 TENSION-CONTROLLED TRAJECTORIES")
    print("="*80)
    print("Time(s) | Motor3 Phase  | Motor3 Angle | Motor3 Tension | Motor4 Phase    | Motor4 Angle")
    print("-"*80)
    
    # Initial conditions (from typical pretensioning)
    motor3_initial_angle = -0.19  # From README example
    motor4_initial_angle = -0.85  # From README example
    initial_tension = 2.0
    
    motor3_current_angle = motor3_initial_angle
    motor3_phase = "ramp"
    
    for t in [0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]:
        
        # Motor 3 behavior
        if t <= 3.0 and motor3_phase == "ramp":
            # Simulate tension increase with angle
            tension_per_rad = 2.0  # Rough estimate
            estimated_tension = initial_tension + abs(motor3_current_angle - motor3_initial_angle) * tension_per_rad
            
            if estimated_tension < 6.0:
                motor3_current_angle += 0.005 * 50  # Step size * frequency
                motor3_phase_str = "RAMP→6N"
            else:
                motor3_phase = "constant"
                motor3_phase_str = "CONSTANT"
        else:
            motor3_phase_str = "CONSTANT"
            estimated_tension = 6.0
        
        # Motor 4 behavior
        if t <= 3.0:
            motor4_angle = motor4_initial_angle
            motor4_phase_str = "WAIT"
        else:
            # Sine wave
            sine_offset = 0.3 * math.sin(2 * math.pi * 0.5 * (t - 3.0))
            motor4_angle = motor4_initial_angle + sine_offset
            motor4_phase_str = "SINE WAVE"
        
        print(f"{t:6.1f}  | {motor3_phase_str:11s} | {motor3_current_angle:10.3f} | {estimated_tension:12.1f}N | {motor4_phase_str:13s} | {motor4_angle:10.3f}")

def print_trajectory_description():
    """Print detailed description of the trajectories"""
    
    print("\n" + "="*80)
    print("📋 DETAILED TRAJECTORY DESCRIPTION")
    print("="*80)
    
    print("\n🔧 MOTOR 3 (ID=3, COM5) - RAMP TO 6N:")
    print("   Phase 1 (0-3s): RAMP")
    print("   • Starts at pretensioning angle (~-0.19 rad, 2N)")
    print("   • Continuously increases angle in 0.005 rad steps")
    print("   • Reads Mark10 tension in real-time")
    print("   • Stops increasing when tension reaches 6N")
    print("   • Target: 6N in 3 seconds maximum")
    print("   Phase 2 (3-10s): CONSTANT")
    print("   • Maintains the angle that gives 6N tension")
    print("   • No further angle changes")
    
    print("\n🔧 MOTOR 4 (ID=4, COM4) - DELAYED SINE:")
    print("   Phase 1 (0-3s): WAIT")
    print("   • Stays exactly at pretensioning angle (~-0.85 rad)")
    print("   • No movement while Motor 3 ramps up")
    print("   Phase 2 (3-10s): SINE WAVE")
    print("   • Sine wave around initial angle")
    print("   • Amplitude: ±0.3 rad")
    print("   • Frequency: 0.5 Hz (one cycle every 2 seconds)")
    print("   • Range: -1.15 to -0.55 rad (approximately)")
    
    print("\n🎯 COORDINATION:")
    print("   • Both motors start from pretensioning results")
    print("   • Transition happens at exactly t=3s")
    print("   • Motor 3 reaches target tension, Motor 4 starts motion")
    print("   • Real-time Mark10 feedback controls Motor 3")
    print("   • Motor 4 follows pure time-based sine function")

if __name__ == "__main__":
    print_trajectory_table()
    print_trajectory_description()
