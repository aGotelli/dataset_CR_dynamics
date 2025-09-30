"""
Tension-Controlled Trajectories
Trajectories that control motor angle based on real-time tension feedback
"""

import time
import math
import serial

class TensionControlledTrajectory:
    def __init__(self, motor_id, mark10_com_port, initial_angle, initial_tension):
        self.motor_id = motor_id
        self.mark10_com_port = mark10_com_port
        self.initial_angle = initial_angle
        self.initial_tension = initial_tension
        self.current_angle = initial_angle
        
        # Setup Mark10 connection
        self.serial = serial.Serial(mark10_com_port, baudrate=115200, timeout=0.2)
        
    def read_tension(self):
        """Read current tension from Mark10"""
        import re
        try:
            self.serial.write("?\r".encode())
            response = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if not response:
                return None
            match = re.search(r'-?\d+(\.\d+)?', response)
            if match:
                return float(match.group(0))
            return None
        except Exception as e:
            print(f"[ERROR] Reading tension: {e}")
            return None
    
    def cleanup(self):
        """Close serial connection"""
        self.serial.close()

class Motor3RampTrajectory(TensionControlledTrajectory):
    """Motor 3: Ramp from pretension to 6N in 3s, then constant"""
    
    def __init__(self, motor_id, mark10_com_port, initial_angle, initial_tension):
        super().__init__(motor_id, mark10_com_port, initial_angle, initial_tension)
        self.target_tension = -12.0  # Target: -6N (more negative = higher tension)
        self.ramp_duration = 3.0    # Ramp time: 3 seconds
        self.angle_step = 0.005     # Step size for angle increments
        self.phase = "ramp"         # "ramp" or "constant"
        self.min_tension = initial_tension  # Never go above this (less negative = slack)
        
    def __call__(self, t):
        """Trajectory function called by motor controller"""
        if t <= self.ramp_duration:
            # Ramp phase: increase tension until we reach -6N
            if self.phase == "ramp":
                current_tension = self.read_tension()
                if current_tension is not None:
                    # Check if we need more tension (more negative)
                    if current_tension > self.target_tension:  # e.g., -2 > -6, need more tension
                        # Increase angle to increase tension (make more negative)
                        self.current_angle += self.angle_step
                        print(f"Motor {self.motor_id}: t={t:.2f}s, tension={current_tension:.2f}N, target={self.target_tension:.2f}N, increasing angle")
                    elif current_tension <= self.target_tension:  # e.g., -6.1 <= -6, target reached
                        # Target reached, switch to constant phase
                        self.phase = "constant"
                        print(f"Motor {self.motor_id}: Target {self.target_tension}N reached at t={t:.2f}s with tension={current_tension:.2f}N")
                    
                    # Safety check: never allow tension to go above minimum (cable slack prevention)
                    if current_tension > self.min_tension:
                        print(f"[WARNING] Motor {self.motor_id}: Tension {current_tension:.2f}N above minimum {self.min_tension:.2f}N - cable may be slack!")
                        
                return self.current_angle
        else:
            # Constant phase: maintain current angle
            self.phase = "constant"
            return self.current_angle

class Motor4DelayedSineTrajectory(TensionControlledTrajectory):
    """Motor 4: Constant until 3s, then cosine wave (safe for cable tension)"""
    
    def __init__(self, motor_id, mark10_com_port, initial_angle, initial_tension):
        super().__init__(motor_id, mark10_com_port, initial_angle, initial_tension)
        self.delay_time = 4.0      # Stay constant for 4 seconds
        self.cosine_amplitude = 1  # Reduced amplitude to stay above baseline
        self.cosine_frequency = 0.5   # Cosine frequency
        self.min_tension_baseline = abs(initial_tension)  # Never go below this tension
        
    def __call__(self, t):
        """Trajectory function called by motor controller"""
        if t <= self.delay_time:
            # Constant phase: stay at initial angle
            return self.initial_angle
        else:
            # Cosine phase: oscillates BELOW baseline for higher tension
            # Cosine goes from 1 to -1, we want only negative offset (more tension)
            cosine_value = math.cos(2 * math.pi * self.cosine_frequency * (t - self.delay_time))
            # Map cosine from [-1,1] to [-amplitude, 0] so it's always negative offset
            negative_offset = -self.cosine_amplitude * (1 + cosine_value) / 2
            return self.initial_angle + negative_offset  # Always more negative = more tension

def create_tension_trajectories(pretension_results):
    """
    Create tension-controlled trajectories based on pretensioning results
    
    Args:
        pretension_results: Dict with motor angles and tensions from pretensioning
        
    Returns:
        tuple: (motor3_trajectory, motor4_trajectory)
    """
    
    # Extract pretensioning results
    motor3_angle = pretension_results.get('motor3_angle', 0.0)
    motor3_tension = pretension_results.get('motor3_tension', 2.0)
    motor4_angle = pretension_results.get('motor4_angle', 0.0)
    motor4_tension = pretension_results.get('motor4_tension', 2.0)
    
    print(f"Creating trajectories from pretension:")
    print(f"Motor 3: angle={motor3_angle:.4f} rad, tension={motor3_tension:.4f} N (baseline)")
    print(f"Motor 4: angle={motor4_angle:.4f} rad, tension={motor4_tension:.4f} N (baseline)")
    print(f"Motor 3 will ramp from {motor3_tension:.1f}N to -6.0N (more negative = higher tension)")
    
    # Create trajectory objects
    motor3_traj = Motor3RampTrajectory(
        motor_id=3,
        mark10_com_port="COM5",  # Motor 3 uses COM5
        initial_angle=motor3_angle,
        initial_tension=motor3_tension
    )
    
    motor4_traj = Motor4DelayedSineTrajectory(
        motor_id=4,
        mark10_com_port="COM4",  # Motor 4 uses COM4
        initial_angle=motor4_angle,
        initial_tension=motor4_tension
    )
    
    return motor3_traj, motor4_traj

def cleanup_trajectories(motor3_traj, motor4_traj):
    """Cleanup trajectory objects"""
    if motor3_traj:
        motor3_traj.cleanup()
    if motor4_traj:
        motor4_traj.cleanup()
