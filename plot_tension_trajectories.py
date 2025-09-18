"""
Plot Tension-Controlled Trajectories
Visualize the behavior of Motor 3 (ramp) and Motor 4 (delayed sine) trajectories
"""

import numpy as np
import matplotlib.pyplot as plt
import math

def simulate_motor3_trajectory(t_array, initial_angle=-0.19, initial_tension=-2.0, target_tension=-6.0, ramp_duration=3.0):
    """
    Simulate Motor 3 ramp trajectory behavior with correct tension physics
    Negative tension values: more negative = higher tension, less negative = cable slack
    """
    angles = []
    tensions = []
    phase = "ramp"
    current_angle = initial_angle
    
    # Simple linear model: more negative angle = more negative tension (higher tension)
    # Assume 1 rad angle change = 2N tension change
    tension_per_rad = -2.0  # Negative: more angle = more negative tension
    
    for t in t_array:
        if t <= ramp_duration and phase == "ramp":
            # Simulate tension based on current angle
            estimated_tension = initial_tension + (current_angle - initial_angle) * tension_per_rad
            
            # Check if we need more tension (more negative)
            if estimated_tension > target_tension:  # e.g., -2 > -6, need more tension
                # Increase angle to increase tension (make more negative)
                current_angle += 0.005  # Step size from trajectory
            else:
                # Target reached
                phase = "constant"
                
        # Store values
        angles.append(current_angle)
        estimated_tension = initial_tension + (current_angle - initial_angle) * tension_per_rad
        tensions.append(estimated_tension)  # Keep negative values
    
    return np.array(angles), np.array(tensions)

def simulate_motor4_trajectory(t_array, initial_angle=-0.85, delay_time=3.0, 
                             sine_amplitude=0.3, sine_frequency=0.5):
    """
    Simulate Motor 4 delayed sine trajectory
    """
    angles = []
    
    for t in t_array:
        if t <= delay_time:
            # Constant phase
            angle = initial_angle
        else:
            # Sine phase
            sine_offset = sine_amplitude * math.sin(2 * math.pi * sine_frequency * (t - delay_time))
            angle = initial_angle + sine_offset
        
        angles.append(angle)
    
    return np.array(angles)

def plot_tension_trajectories():
    """Create comprehensive plots of the tension-controlled trajectories"""
    
    # Time array for 10 seconds
    t = np.linspace(0, 10, 1000)
    
    # Simulate trajectories
    motor3_angles, motor3_tensions = simulate_motor3_trajectory(t)
    motor4_angles = simulate_motor4_trajectory(t)
    
    # Create figure with subplots
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Tension-Controlled Trajectories Visualization', fontsize=16, fontweight='bold')
    
    # Plot 1: Motor 3 Angle vs Time
    ax1.plot(t, motor3_angles, 'b-', linewidth=2, label='Motor 3 Angle')
    ax1.axvline(x=3, color='r', linestyle='--', alpha=0.7, label='Ramp End (3s)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (rad)')
    ax1.set_title('Motor 3: Ramp Trajectory (Angle)')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Plot 2: Motor 3 Tension vs Time
    ax2.plot(t, motor3_tensions, 'g-', linewidth=2, label='Motor 3 Tension')
    ax2.axhline(y=-6, color='r', linestyle='--', alpha=0.7, label='Target (-6N)')
    ax2.axhline(y=-2, color='orange', linestyle=':', alpha=0.7, label='Baseline (-2N)')
    ax2.axvline(x=3, color='r', linestyle='--', alpha=0.7, label='Ramp End (3s)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Tension (N)')
    ax2.set_title('Motor 3: Tension Profile (Negative = Higher Tension)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.text(5, -4, 'More negative\n= Higher tension', 
             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
    
    # Plot 3: Motor 4 Angle vs Time
    ax3.plot(t, motor4_angles, 'purple', linewidth=2, label='Motor 4 Angle')
    ax3.axvline(x=3, color='r', linestyle='--', alpha=0.7, label='Sine Start (3s)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (rad)')
    ax3.set_title('Motor 4: Delayed Sine Trajectory')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # Plot 4: Both Motors Angle Comparison
    ax4.plot(t, motor3_angles, 'b-', linewidth=2, label='Motor 3 (Ramp)')
    ax4.plot(t, motor4_angles, 'purple', linewidth=2, label='Motor 4 (Delayed Sine)')
    ax4.axvline(x=3, color='r', linestyle='--', alpha=0.7, label='Transition Point (3s)')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Angle (rad)')
    ax4.set_title('Both Motors: Angle Comparison')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    # Add text annotations
    ax1.text(1.5, motor3_angles[int(len(t)*0.3)], 'Ramp Phase\n(0-3s)', 
             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
    ax1.text(6, motor3_angles[-1], 'Constant Phase\n(3-10s)', 
             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
    
    ax3.text(1.5, motor4_angles[0], 'Constant Phase\n(0-3s)', 
             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.7))
    ax3.text(6, motor4_angles[-1], 'Sine Wave Phase\n(3-10s)', 
             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral", alpha=0.7))
    
    plt.tight_layout()
    plt.show()
    
    # Print trajectory summary
    print("🎯 TENSION-CONTROLLED TRAJECTORIES SUMMARY")
    print("="*50)
    print(f"📈 Motor 3 (ID=3, COM5):")
    print(f"   • 0-3s: Ramp from {motor3_tensions[0]:.1f}N to {motor3_tensions[int(len(t)*0.3)]:.1f}N")
    print(f"   • Target: -6N tension (more negative = higher tension)")
    print(f"   • 3-10s: Constant at {motor3_tensions[-1]:.1f}N")
    print()
    print(f"📈 Motor 4 (ID=4, COM4):")
    print(f"   • 0-3s: Constant at {motor4_angles[0]:.3f} rad")
    print(f"   • 3-10s: Sine wave (amp={0.3:.1f} rad, freq={0.5:.1f} Hz)")
    print(f"   • Range: {min(motor4_angles[int(len(t)*0.3):]):.3f} to {max(motor4_angles[int(len(t)*0.3):]):.3f} rad")
    print()
    print("⚠️  CABLE PHYSICS:")
    print("   • Negative tension = Cable under tension")
    print("   • More negative = Higher tension")
    print("   • Less negative = Approaching slack")
    print("   • Never go above baseline (-2N) = Cable slack!")

if __name__ == "__main__":
    plot_tension_trajectories()
