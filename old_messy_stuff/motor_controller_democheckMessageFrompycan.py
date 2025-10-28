import logging
import time
import os
import sys
sys.path.append(os.path.join(".", "cybergear"))
from cybergear.pcan_cybergear import CANMotorController
import can
import serial
import math
import numpy as np

def get_current_angle(motor, max_retries=5, retry_delay=0.2):
    """Get current motor angle with retry mechanism"""
    for attempt in range(max_retries):
        status = motor.get_motor_status()
        print(f"Motor {status[0]} pos: {status[1] if status else None}")
        if status and len(status) > 1:
            return status[1], math.degrees(status[1]) #rad, deg
        
        if attempt < max_retries - 1:
            time.sleep(retry_delay)

def sinusoidal_trajectory_offset(base_angle_deg, amplitude_deg=30, frequency=0.5, duration=10, sample_rate=10):
    """Generate sinusoidal trajectory starting from base angle"""
    t = np.linspace(0, duration, int(duration * sample_rate))
    # angles_deg =np.ones(len(t))*base_angle_deg #+ amplitude_deg * np.sin(2 * np.pi * frequency * t)
    angles_deg = base_angle_deg + amplitude_deg * np.sin(2 * np.pi * frequency * t)
    print(f"base angle: {base_angle_deg}")
    return t, angles_deg

def execute_trajectory(motor, time_points, angles_deg):
    """Execute trajectory and monitor motor status"""
    # logging.info(f"Executing trajectory with {len(angles_deg)} points over {time_points[-1]:.1f} seconds")
    # logging.info(f"Amplitude: ±{max(abs(angles_deg - np.mean(angles_deg))):.1f}°, Frequency: ~{1/(time_points[1]-time_points[0])/len(time_points)*2:.2f} Hz")
    
    start_time = time.time()
    print(angles_deg)
    for i, (t, target_angle_deg) in enumerate(zip(time_points, angles_deg)):
        # Wait for the right time
        while time.time() - start_time < t:
            time.sleep(0.01)
        
        # Send position command
        target_angle_rad = math.radians(target_angle_deg)
        motor.set_motor_position_control(limit_spd=6, loc_ref=target_angle_rad)
        
        # # Read current status
        # angle_rad, angle_deg = get_current_angle(motor)
        # if angle_deg is not None:
        #     error = target_angle_deg - angle_deg
        #     elapsed = time.time() - start_time
        #     logging.info(f"t={elapsed:.2f}s | Target: {target_angle_deg:+6.1f}° | Current: {angle_deg:+6.1f}° | Error: {error:+5.1f}°")
        # else:
        #     logging.warning(f"t={t:.2f}s | Target: {target_angle_deg:+6.1f}° | No status received")


        
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
def motorClean(motor1,motor2):
        motor1.disable()
        motor1.set_0_pos()
        motor2.disable()
        motor2.set_0_pos()


def main():
    bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
    motor1_cfg = [3, "COM5"]  # [motor_id, com_port]
    motor2_cfg = [4, "COM4"]  # [motor_id, com_port]
    mark10_rate = 350
    try:
        motor1 = CANMotorController(bus, motor_id=motor1_cfg[0], main_can_id=254)
        motor1.set_run_mode(motor1.RunModes.POSITION_MODE)
        motor1.enable()
        motor2 = CANMotorController(bus, motor_id=motor2_cfg[0], main_can_id=254)
        motor2.set_run_mode(motor2.RunModes.POSITION_MODE)
        motor2.enable()
       
       

        # Initialize Mark10 sensors
        m10_motor1 = Mark10Sensor(motor1_cfg[1], mark10_rate)  # motor1_cfg[1] is COM port
        # m10_motor2 = Mark10Sensor(motor2_cfg[1], mark10_rate)  # motor2_cfg[1] is COM port
        
        # Run pretensioning
        # success = run_pretensioning_thread(motor1, motor2, m10_motor1, m10_motor2, target_tension=5.0)
        
        logging.info("Connected to motor (ID: 3)")
        
        
        # Read initial angle
        angle_rad_m1, current_angle_deg_m1 = get_current_angle(motor1)
        print(f"current angle deg: {current_angle_deg_m1}")
        amplitude = 20
        frequency = 0.5
        duration = 6
        
        logging.info(f"\nGenerating sinusoidal trajectory from current position:")
        logging.info(f"  Base angle: {current_angle_deg_m1:.1f}°")
        logging.info(f"  Amplitude: ±{amplitude}°")
        logging.info(f"  Frequency: {frequency} Hz")
        logging.info(f"  Duration: {duration}s")
        
        t, angles = sinusoidal_trajectory_offset(current_angle_deg_m1, amplitude, frequency, duration)
        
        input("\nPress Enter to start trajectory...")
        print(angles)
        execute_trajectory(motor1, t, angles)
        
    except KeyboardInterrupt:
        logging.info("Interrupted by user")
    finally:
        try:
            motorClean(motor1,motor2)
            bus.shutdown()
        except:
            pass
        logging.info("Test completed.")


if __name__ == "__main__":
    main()
