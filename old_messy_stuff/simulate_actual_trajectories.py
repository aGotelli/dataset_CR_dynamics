"""
Direct Simulation of Tension-Controlled Trajectories
Simulates exactly what the actual trajectory classes do
"""

import numpy as np
import matplotlib.pyplot as plt
import math

class SimulatedMotor3RampTrajectory:
    """Direct simulation of Motor3RampTrajectory class behavior"""
    
    def __init__(self, motor_id=3, initial_angle=-0.19, initial_tension=-2.0):
        self.motor_id = motor_id
        self.initial_angle = initial_angle
        self.initial_tension = initial_tension
        self.current_angle = initial_angle
        
        self.target_tension = -6.0  # Target: -6N (more negative = higher tension)
        self.ramp_duration = 3.0    # Ramp time: 3 seconds
        self.angle_step = 0.005     # Step size for angle increments
        self.phase = "ramp"         # "ramp" or "constant"
        self.min_tension = initial_tension  # Never go above this (less negative = slack)
        
    def simulate_read_tension(self):
        """Simulate Mark10 reading based on current angle"""
        # Simple linear model: more negative angle = more negative tension
        tension_per_rad = -2.0  # More angle = more negative tension
        simulated_tension = self.initial_tension + (self.current_angle - self.initial_angle) * tension_per_rad
        return simulated_tension
        
    def __call__(self, t):
        """Exact copy of the trajectory function logic"""
        if t <= self.ramp_duration:
            # Ramp phase: increase tension until we reach -6N
            if self.phase == "ramp":
                current_tension = self.simulate_read_tension()
                if current_tension is not None:
                    # Check if we need more tension (more negative)
                    if current_tension > self.target_tension:  # e.g., -2 > -6, need more tension
                        # Increase angle to increase tension (make more negative)
                        self.current_angle += self.angle_step
                    elif current_tension <= self.target_tension:  # e.g., -6.1 <= -6, target reached
                        # Target reached, switch to constant phase
                        self.phase = "constant"
                        
            return self.current_angle
        else:
            # Constant phase: maintain current angle
            self.phase = "constant"
            return self.current_angle

class SimulatedMotor4DelayedSineTrajectory:
    """Direct simulation of Motor4DelayedSineTrajectory class behavior"""
    
    def __init__(self, motor_id=4, initial_angle=-0.85, initial_tension=-2.0):
        self.motor_id = motor_id
        self.initial_angle = initial_angle
        self.initial_tension = initial_tension
        self.current_angle = initial_angle
        self.delay_time = 3.0         # Stay constant for 3 seconds
        self.cosine_amplitude = 0.15  # Reduced amplitude to stay above baseline
        self.cosine_frequency = 0.5   # Cosine frequency
        self.min_tension_baseline = abs(initial_tension)  # Never go below this tension
        
    def simulate_read_tension(self):
        """Simulate Mark10 reading based on current angle"""
        # Simple linear model: more negative angle = more negative tension
        tension_per_rad = -2.0  # More angle = more negative tension
        simulated_tension = self.initial_tension + (self.current_angle - self.initial_angle) * tension_per_rad
        return simulated_tension
        
    def __call__(self, t):
        """Exact copy of the trajectory function logic"""
        if t <= self.delay_time:
            # Constant phase: stay at initial angle
            self.current_angle = self.initial_angle
            return self.initial_angle
        else:
            # Cosine phase: oscillates BELOW baseline for higher tension
            # Cosine goes from 1 to -1, we want only negative offset (more tension)
            cosine_value = math.cos(2 * math.pi * self.cosine_frequency * (t - self.delay_time))
            # Map cosine from [-1,1] to [-amplitude, 0] so it's always negative offset
            negative_offset = -self.cosine_amplitude * (1 + cosine_value) / 2
            self.current_angle = self.initial_angle + negative_offset  # Always more negative = more tension
            return self.current_angle

def plot_actual_trajectory_behavior():
    """Plot exactly what the trajectory classes do"""
    
    # Create trajectory instances with realistic pretension values
    motor3_traj = SimulatedMotor3RampTrajectory(
        motor_id=3, 
        initial_angle=-0.19,  # From typical pretension
        initial_tension=-2.0  # From typical pretension
    )
    
    motor4_traj = SimulatedMotor4DelayedSineTrajectory(
        motor_id=4,
        initial_angle=-0.85,  # From typical pretension
        initial_tension=-2.0  # From typical pretension
    )
    
    # Simulate at 50Hz (matching motor controller frequency)
    dt = 0.02  # 50Hz = 0.02s intervals
    t_array = np.arange(0, 10, dt)
    
    motor3_angles = []
    motor3_tensions = []
    motor4_angles = []
    motor4_tensions = []
    
    print("🔍 SIMULATING ACTUAL TRAJECTORY BEHAVIOR...")
    print("Time(s) | M3 Phase | M3 Angle | M3 Tension | M4 Phase | M4 Angle | M4 Tension")
    print("-" * 85)
    
    for i, t in enumerate(t_array):
        # Call trajectory functions exactly as motor controller would
        motor3_angle = motor3_traj(t)
        motor4_angle = motor4_traj(t)
        
        # Get simulated tensions for both motors
        motor3_tension = motor3_traj.simulate_read_tension()
        motor4_tension = motor4_traj.simulate_read_tension()
        
        motor3_angles.append(motor3_angle)
        motor3_tensions.append(motor3_tension)
        motor4_angles.append(motor4_angle)
        motor4_tensions.append(motor4_tension)
        
        # Print every 0.5 seconds
        if i % 25 == 0:  # Every 0.5s at 50Hz
            m3_phase = motor3_traj.phase.upper()
            m4_phase = "WAIT" if t <= 3.0 else "COSINE"
            print(f"{t:6.1f}  | {m3_phase:8s} | {motor3_angle:8.3f} | {motor3_tension:9.2f} | {m4_phase:8s} | {motor4_angle:8.3f} | {motor4_tension:9.2f}")
    
    # Convert to numpy arrays
    motor3_angles = np.array(motor3_angles)
    motor3_tensions = np.array(motor3_tensions)
    motor4_angles = np.array(motor4_angles)
    motor4_tensions = np.array(motor4_tensions)
    
    # Create plots
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('ACTUAL Tension-Controlled Trajectories Simulation', fontsize=16, fontweight='bold')
    
    # Plot 1: Motor 3 Angle vs Time
    ax1.plot(t_array, motor3_angles, 'b-', linewidth=2, label='Motor 3 Angle')
    ax1.axvline(x=3, color='r', linestyle='--', alpha=0.7, label='Ramp End (3s)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (rad)')
    ax1.set_title('Motor 3: Actual Ramp Trajectory')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Plot 2: Both Motors Tension vs Time
    ax2.plot(t_array, motor3_tensions, 'g-', linewidth=2, label='Motor 3 Tension')
    ax2.plot(t_array, motor4_tensions, 'orange', linewidth=2, label='Motor 4 Tension')
    ax2.axhline(y=-6, color='r', linestyle='--', alpha=0.7, label='M3 Target (-6N)')
    ax2.axhline(y=-2, color='gray', linestyle=':', alpha=0.7, label='Baseline (-2N)')
    ax2.axvline(x=3, color='r', linestyle='--', alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Tension (N)')
    ax2.set_title('Both Motors: Tension Profiles')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Plot 3: Motor 4 Angle vs Time
    ax3.plot(t_array, motor4_angles, 'purple', linewidth=2, label='Motor 4 Angle')
    ax3.axvline(x=3, color='r', linestyle='--', alpha=0.7, label='Sine Start (3s)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (rad)')
    ax3.set_title('Motor 4: Safe Cosine Trajectory (No Cable Slack)')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # Plot 4: Both Motors Angles Together
    ax4.plot(t_array, motor3_angles, 'b-', linewidth=2, label='Motor 3 (Ramp)')
    ax4.plot(t_array, motor4_angles, 'purple', linewidth=2, label='Motor 4 (Safe Cosine)')
    ax4.axvline(x=3, color='r', linestyle='--', alpha=0.7, label='Transition (3s)')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Angle (rad)')
    ax4.set_title('Both Motors: Safe Trajectory Behavior')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    plt.tight_layout()
    plt.show()
    
    # Summary
    print(f"\n✅ SIMULATION COMPLETE")
    print(f"📊 Motor 3: {motor3_angles[0]:.3f} → {motor3_angles[-1]:.3f} rad")
    print(f"📊 Motor 3: {motor3_tensions[0]:.1f} → {motor3_tensions[-1]:.1f} N")
    print(f"📊 Motor 4: {motor4_angles[0]:.3f} rad (const) → {min(motor4_angles[300:]):.3f} to {max(motor4_angles[300:]):.3f} rad (sine)")
    print(f"📊 Motor 4: {motor4_tensions[0]:.1f} N (const) → {min(motor4_tensions[300:]):.1f} to {max(motor4_tensions[300:]):.1f} N (sine)")
    print(f"🎯 M3 Target reached: {'YES' if any(np.array(motor3_tensions) <= -6.0) else 'NO - only reached ' + str(min(motor3_tensions))[:5] + 'N'}")
    print(f"🎯 M4 Tension range: {min(motor4_tensions):.1f} to {max(motor4_tensions):.1f} N")

if __name__ == "__main__":
    plot_actual_trajectory_behavior()
