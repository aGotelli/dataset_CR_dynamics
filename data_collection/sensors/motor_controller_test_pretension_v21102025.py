import os
import sys
sys.path.append(os.path.join(".", "cybergear"))
from cybergear.pcan_cybergear import CANMotorController
import can

import serial

import time
import math
import numpy as np


import threading

class Motor:
    def __init__(self, motor_cfg, bus):
        """
        Initialize motor with integrated Mark10 sensors
        """
        self.motor_id = motor_cfg[0] #3 or 4
        self.max_speed = 10
        # Initialize motors and Mark10 sensors
        self.motor = CANMotorController(bus, motor_id=self.motor_id, main_can_id=254)
        self.motor.set_run_mode(self.motor.RunModes.POSITION_MODE)
        self.motor.enable()

        self.mark10_motor = motor_cfg[1]   # Mark10 associated to motor
        curPosMotor=self.get_current_angle()
        self.angle = curPosMotor[0]
        
        print(f"✅ Motor {self.motor_id} initialized at  {self.angle}rad - {curPosMotor[1]:.2f}°")
        
        self.stop_requested = False  # For user interruption
    def get_current_angle(self, max_retries=5, retry_delay=0.2):
        """Get current motor angle with retry mechanism"""
        for attempt in range(max_retries):
            status = self.motor.get_motor_status()
            print(f"Motor {status[0]} pos: {status[1] if status else None}")
            if status and len(status) > 1:
                return status[1], math.degrees(status[1]) #rad, deg
            
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
        print(f"CRITICAL: Failed to read motor position after {max_retries} attempts")
        return None, None
       
class Mark10Sensor:
    def __init__(self, com_port, sampling_rate, timeout=0.1):
        """Initialize Mark10 sensor"""
        self.com_port = com_port
        self.sampling_rate = sampling_rate
        self.timeout = timeout
        self.serial_connection = None
        
        # Establish serial connection in constructor
        try:
            self.serial_connection = serial.Serial(
                self.com_port, 
                baudrate=115200, 
                timeout=self.timeout
            )

            timing = self.measure_timing(50)
            print(f"Mark-10 initialized with {timing['data_rate']:.1f} Hz (desired {sampling_rate} Hz) on {com_port} ✅")
        except Exception as e:
            print(f"❌ Failed to connect to Mark-10 on {com_port}: {e}")
            self.serial_connection = None
            raise  # Re-raise exception so setup fails properly   
    def measure_timing(self, num_samples=1000):
        """ Measure timing performance using existing connection
            Args:
                num_samples: Number of samples to measure
            Returns:
                timing_stats: Dictionary with timing statistics"""
        print(f"Measuring timing for {num_samples} data fetches...")
        if not self.serial_connection or not self.serial_connection.is_open:
            print("❌ Serial connection not available")
            return None
        
        timing_data = np.zeros(num_samples)
        
        for i in range(num_samples):
            start_time = time.perf_counter()
            self.serial_connection.write("?\r".encode())
            response = self.serial_connection.readline().decode().strip()
            end_time = time.perf_counter()
            
            timing_data[i] = (end_time - start_time) * 1000  # Convert to ms
        
        # Calculate statistics
        stats = {
            'avg_time': np.mean(timing_data),
            'min_time': np.min(timing_data),
            'max_time': np.max(timing_data),
            'std_time': np.std(timing_data),
            'data_rate': 1000 / np.mean(timing_data),
            'num_samples': num_samples
        }
        return stats
    
    def get_tension(self):  
        self.serial_connection.write("?\r".encode())
        response = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
        force_value = 0
        if response:
            # Parse force value
            force_str = response.replace('N', '').replace('lbF', '').replace('lb', '').strip()
            force_value = float(force_str)
        return force_value   
    def close(self):
        """Close serial connection"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print(f"Serial connection to {self.com_port} closed")


def pretension_single_motor(motor, mark10_sensor, target_tension, direction=1, max_angle_change=2.0, step_size=0.01, wait_time=0.2, tolerance=0.01):
    """  Pretension single motor to target tension  """
    print(f"🔧 Starting pretensioning Motor {motor.motor_id} to {target_tension}N...")
    
    # Fix: get_current_angle() returns (rad, deg) tuple - take only radians
    angle_data = motor.get_current_angle()
    current_angle = angle_data[0] if angle_data[0] is not None else 0.0
    
    max_steps = int(max_angle_change / step_size)
    tension = mark10_sensor.get_tension()
    
    for step in range(max_steps):
        abs_tension = abs(tension)
        abs_target = abs(target_tension)
        
        print(f"Motor {motor.motor_id} - Step {step+1}: angle = {current_angle:.4f} rad, tension = {abs_tension:.4f} N, target = {abs_target:.4f} N")
        
        # Check if target reached
        if abs(abs_tension - abs_target) <= tolerance:
            motor.angle = current_angle
            print(f"✅ Motor {motor.motor_id} pretensioning successful!")
            return current_angle, abs_tension
        
        # Adjust angle based on tension error
        elif abs_tension < abs_target:
            current_angle += direction * step_size
            motor.motor.set_motor_position_control(limit_spd=motor.max_speed, loc_ref=current_angle)
        else:
            current_angle -= direction * step_size
            motor.motor.set_motor_position_control(limit_spd=motor.max_speed, loc_ref=current_angle)
        
        # Read new tension for next iteration
        tension = mark10_sensor.get_tension()
        time.sleep(wait_time)
    
    motor.angle = current_angle
    print(f"⚠️ Motor {motor.motor_id} pretensioning reached max steps, current angle {motor.angle}")
    return current_angle, abs(tension)

def run_pretensioning_thread(motor1, motor2, m10_motor1, m10_motor2, target_tension):
    """  Run pretensioning for both motors in parallel threads  """
    print("\n🔧 Starting pretensioning for both motors...")
    
    results = {"motor1": False, "motor2": False, "angle1": 0, "tension1": 0, "angle2": 0, "tension2": 0}
    
    def pretension_motor1():
        try:
            angle, tension = pretension_single_motor(
                motor1, m10_motor1, target_tension, direction=1
            )
            results["motor1"] = True
            results["angle1"] = angle
            results["tension1"] = tension
        except Exception as e:
            print(f"❌ Motor 1 pretensioning failed: {e}")
            results["motor1"] = False
    
    def pretension_motor2():
        try:
            angle, tension = pretension_single_motor(
                motor2, m10_motor2, target_tension, direction=-1
            )
            results["motor2"] = True
            results["angle2"] = angle
            results["tension2"] = tension
        except Exception as e:
            print(f"❌ Motor 2 pretensioning failed: {e}")
            results["motor2"] = False
    
    # Create and start threads
    thread1 = threading.Thread(target=pretension_motor1, name="Pretension_Motor1")
    thread2 = threading.Thread(target=pretension_motor2, name="Pretension_Motor2")
    
    try:
        thread1.start()
        thread2.start()
        
        # Wait for both threads to complete
        thread1.join()
        thread2.join()
        
        # Check results
        if results["motor1"] and results["motor2"]:
            print(f"✅ Pretensioning concluded")
            print(f"Motor 1: angle = {motor1.angle} rad memorized vs current {motor1.get_current_angle()[0]}, tension = {m10_motor1.get_tension()} N")
            print(f"Motor 2: angle = {motor2.angle} rad memorized vs current {motor2.get_current_angle()[0]}, tension = {m10_motor1.get_tension()} N")
            return True
        else:
            print(f"❌ Pretensioning failed - Motor1: {results['motor1']}, Motor2: {results['motor2']}")
            return False
            
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected: stopping pretension...")
        motor1.stop_requested = True
        motor2.stop_requested = True
        thread1.join(timeout=5)
        thread2.join(timeout=5)
        return False

def execute_trajectory(motor, time_points, angles_deg):
    """Execute trajectory and monitor motor status"""
    print(f"Executing trajectory with {len(angles_deg)} points over {time_points[-1]:.1f} seconds")
    
    start_time = time.time()
    
    for i, (t, target_angle_deg) in enumerate(zip(time_points, angles_deg)):
        # Wait for the right time
        while time.time() - start_time < t:
            time.sleep(0.01)
        
        # Send position command
        target_angle_rad = math.radians(target_angle_deg)
        motor.motor.set_motor_position_control(limit_spd=6, loc_ref=target_angle_rad)
        

def sinusoidal_trajectory_offset(base_angle_deg, amplitude_deg=15, frequency=0.3, duration=10, sample_rate=20):
    """Generate sinusoidal trajectory starting from base angle"""
    t = np.linspace(0, duration, int(duration * sample_rate))
    angles_deg = base_angle_deg + amplitude_deg * np.sin(2 * np.pi * frequency * t)
    return t, angles_deg

def run_motor_trajectories(motor1, motor2):
    """Run trajectories for both motors starting from current angles"""
    # Read current angles (starting points for trajectories)
    angle1_rad, current_angle1_deg = motor1.get_current_angle()
    angle2_rad, current_angle2_deg = motor2.get_current_angle()

    # Generate trajectories from current positions
    amplitude = 15  # degrees - small amplitude to not stress cables
    frequency = 0.3  # Hz - slow frequency
    duration = 10    # seconds
        
    # Generate sine for motor1, cosine for motor2 (90° phase difference)
    t, angles1_deg = sinusoidal_trajectory_offset(current_angle1_deg, amplitude, frequency, duration)
    _, angles2_deg = sinusoidal_trajectory_offset(current_angle2_deg, amplitude, frequency, duration)
    
    # Make motor2 cosine (90° phase shift)
    angles2_deg = current_angle2_deg + amplitude * np.cos(2 * np.pi * frequency * t)
    
    def run_motor1_trajectory():
        execute_trajectory(motor1, t, angles1_deg)
    
    def run_motor2_trajectory():
        execute_trajectory(motor2, t, angles2_deg)
    
    # Run both trajectories in parallel
    thread1 = threading.Thread(target=run_motor1_trajectory, name="Trajectory_Motor1")
    thread2 = threading.Thread(target=run_motor2_trajectory, name="Trajectory_Motor2")
    
    try:
        thread1.start()
        thread2.start()
        
        thread1.join()
        thread2.join()
        
        print("✅ Trajectory execution completed!")
        return True
        
    except KeyboardInterrupt:
        print("❌ Trajectory interrupted by user")
        return False
    

def main(motor1_cfg, motor2_cfg, mark10_rate):
    """Main function for motor controller"""
    print("🚀 Initializing Motor Controller...")
    
    # Initialize CAN bus
    bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)

    # Initialize motors
    motor1 = Motor(motor1_cfg, bus)
    motor2 = Motor(motor2_cfg, bus)
    
    # Initialize Mark10 sensors
    m10_motor1 = Mark10Sensor(motor1_cfg[1], mark10_rate)  # motor1_cfg[1] is COM port
    m10_motor2 = Mark10Sensor(motor2_cfg[1], mark10_rate)  # motor2_cfg[1] is COM port
    
    # Run pretensioning
    success = run_pretensioning_thread(motor1, motor2, m10_motor1, m10_motor2, target_tension=5.0)
    
    if success:
        # Step 2: Ask user if they want to run trajectories
        print("\n" + "="*50)
        print("Pretensioning completed successfully!")
        print("="*50)
        
        user_input = input("🎬 Press ENTER to start motor trajectories (or 'q' to quit): ")
        
        if user_input.lower() != 'q':
            print("Starting trajectories in 3 seconds...")
            
            # Run trajectories
            run_motor_trajectories(motor1, motor2)
        else:
            print("Trajectory execution skipped by user")
    else:
        print("❌ Pretensioning failed, skipping trajectories")
    
    # Cleanup
    motor1.motor.disable()
    motor2.motor.disable()
    m10_motor1.close()
    m10_motor2.close()
    
    return success
    


if __name__ == "__main__":
    # Test configuration
    motor1_cfg = [3, "COM5"]  # [motor_id, com_port]
    motor2_cfg = [4, "COM4"]  # [motor_id, com_port]
    mark10_rate = 350
    
    success = main(motor1_cfg, motor2_cfg, mark10_rate)
    print(f"Final result: {success}")


