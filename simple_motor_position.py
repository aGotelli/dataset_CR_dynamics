import os
import sys
sys.path.append(os.path.join(".", "cybergear"))

from pcan_cybergear import CANMotorController
import can
import time
import math

class MotorController:
    def __init__(self, motor_id=3, main_can_id=254, interface="candle", channel=0, bitrate=1000000):
        """Initialize motor controller with CAN bus connection."""
        self.bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
        self.motor = CANMotorController(self.bus, motor_id=motor_id, main_can_id=main_can_id)
        self.home_position = 0.0
        self.motor_data = []
        
        # Setup motor
        self.motor.set_run_mode(self.motor.RunModes.POSITION_MODE)
        self.motor.enable()
        
   
    
    def go_home(self, tolerance=0.05, max_attempts=100):
        """Move motor back to home position with feedback control."""
        print("Returning to home position...")
        target_position = self.home_position
        attempt = 0
        
        while attempt < max_attempts:
            self.motor.set_motor_position_control(limit_spd=10, loc_ref=target_position)
            time.sleep(0.1)
            
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg and ((msg.arbitration_id >> 24) & 0xFF) == 2:
                    current_pos = self.motor._uint_to_float(
                        (msg.data[0] << 8) + msg.data[1],
                        self.motor.P_MIN, self.motor.P_MAX, self.motor.TWO_BYTES_BITS
                    )
                    error = abs(current_pos - target_position)
                    print(f"Position: {current_pos:.3f} rad, Target: {target_position:.3f} rad, Error: {error:.3f} rad")
                    
                    if error < tolerance:
                        print(f"Reached home position in {attempt + 1} attempts!")
                        return True
                else:
                    print("No feedback received, continuing...")
            except:
                print("Error reading position, continuing...")
            
            attempt += 1
        
        print(f"Warning: Could not reach home position after {max_attempts} attempts")
        return False
    
    def execute_trajectory(self, position_function, duration, frequency=50.0, speed_limit=50, save_data=True):
        """Execute a trajectory defined by a position function of time.
        
        Args:
            position_function: Function that takes time (t) and returns position relative to home
            duration: Duration in seconds
            frequency: Control frequency in Hz
            speed_limit: Motor speed limit
            save_data: Whether to save trajectory data to CSV
        """
        print(f"Executing trajectory for {duration}s at {frequency}Hz...")
        
        dt = 1.0 / frequency
        self.motor_data = []
        
        start_time = time.perf_counter()
        loop_count = 0
        
        while True:
            target_time = start_time + loop_count * dt
            current_time = time.perf_counter()
            
            if current_time - start_time >= duration:
                break
                
            t = current_time - start_time

            target_position = position_function(t)
            self.motor.set_motor_position_control(limit_spd=speed_limit, loc_ref=target_position)

            # Read motor feedback
            try:
                msg = self.bus.recv(timeout=0.001)
                if msg and ((msg.arbitration_id >> 24) & 0xFF) == 2:
                    actual_pos = self.motor._uint_to_float(
                        (msg.data[0] << 8) + msg.data[1],
                        self.motor.P_MIN, self.motor.P_MAX, self.motor.TWO_BYTES_BITS
                    )
                    self.motor_data.append([t, target_position, actual_pos])
            except:
                self.motor_data.append([t, target_position, None])
            
            loop_count += 1
            
            # Sleep until next loop time
            sleep_time = target_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        if save_data:
            self.save_data()
        
        print(f"Trajectory completed. Collected {len(self.motor_data)} data points.")
    
    def save_data(self, filename="motor_data.csv"):
        """Save collected motor data to CSV file."""
        print(f"Saving {len(self.motor_data)} data points to {filename}...")
        with open(filename, "w") as f:
            f.write("time,commanded_position,actual_position\n")
            for data_point in self.motor_data:
                f.write(f"{data_point[0]:.6f},{data_point[1]:.6f},{data_point[2]}\n")
    
    def cleanup(self):
        """Disable motor and shutdown CAN bus."""
        try:
            self.motor.disable()
        except Exception as e:
            print(f"Warning: Error disabling motor: {e}")
        try:
            self.bus.shutdown()
        except Exception as e:
            print(f"Warning: Error shutting down CAN bus: {e}")
        print("Motor disabled and CAN bus shut down.")

def main():
    # Create motor controller instance
    controller = MotorController(motor_id=3, main_can_id=254)
    
    try:
        # Set current position as home
        controller.go_home(0.001)
        
        # Define trajectory functions
        
        # Example 1: Sine wave trajectory
        def sine_trajectory(t):
            amplitude = 1  # Amplitude in radians
            cycles = 1.0
            duration = 5.0
            omega = 2 * math.pi 
            return amplitude * math.sin(omega * t)
        
        # Example 2: Linear rotation trajectory
        def linear_trajectory(t):
            rotations_per_second = 0.5
            return 2 * math.pi * rotations_per_second * t
        
        # Example 3: Custom lambda trajectory
        triangle_wave = lambda t: 2.0 * (2 * abs((t % 4) - 2) - 1)  # Triangle wave ±2 rad
        
        # Execute sine trajectory
        print("\n=== Executing Sine Trajectory ===")
        controller.execute_trajectory(sine_trajectory, duration=5.0, frequency=50.0)
        
        # Go home between trajectories
        controller.go_home()
        time.sleep(1)
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()
