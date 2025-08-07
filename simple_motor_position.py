import os
import sys
sys.path.append(os.path.join(".", "cybergear"))

from pcan_cybergear import CANMotorController
import can
import time
import math
from datetime import datetime

class DualMotorController:
    """Simple controller for managing two motors without threading."""
    
    def __init__(self, motor1_id=1, motor2_id=2, main_can_id=254, interface="candle", channel=0, bitrate=1000000):
        """Initialize dual motor controller.
        
        Args:
            motor1_id: ID of first motor
            motor2_id: ID of second motor
            main_can_id: Main CAN ID
            interface: CAN interface type
            channel: CAN channel
            bitrate: CAN bitrate
        """
        print(f"Initializing dual motor controller for motors: {motor1_id}, {motor2_id}")
        
        # Create shared CAN bus
        self.bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
        print("CAN bus created")
        
        # Initialize motors
        self.motor1 = CANMotorController(self.bus, motor_id=motor1_id, main_can_id=main_can_id)
        self.motor2 = CANMotorController(self.bus, motor_id=motor2_id, main_can_id=main_can_id)
        
        # Setup motors
        self.motor1.set_run_mode(self.motor1.RunModes.POSITION_MODE)
        self.motor1.enable()
        print(f"Motor {motor1_id} initialized and enabled")
        
        self.motor2.set_run_mode(self.motor2.RunModes.POSITION_MODE)
        self.motor2.enable()
        print(f"Motor {motor2_id} initialized and enabled")
        
        # Home positions (current position as zero)
        self.home_position1 = 0.0
        self.home_position2 = 0.0
        
        # Data storage
        self.motor_data = []
        
        print("Dual motor controller initialized")
    
    def set_home_positions(self):
        """Set current positions as home positions."""
        print("Setting home positions to 0.0 rad for both motors")
        self.home_position1 = 0.0
        self.home_position2 = 0.0
    
    def go_home(self):
        """Move both motors to home position."""
        print("Moving both motors to home position...")
        self.motor1.set_motor_position_control(limit_spd=10, loc_ref=self.home_position1)
        self.motor2.set_motor_position_control(limit_spd=10, loc_ref=self.home_position2)
        time.sleep(2)  # Give motors time to reach position
        print("Homing commands sent")
    
    def execute_synchronized_trajectories(self, trajectory_func1, trajectory_func2, duration, frequency=50.0, speed_limit=50, save_data=True,save_path=""):
        """Execute synchronized trajectories on both motors.
        
        Args:
            trajectory_func1: Function for motor 1 (takes time, returns position relative to home)
            trajectory_func2: Function for motor 2 (takes time, returns position relative to home) 
            duration: Duration in seconds
            frequency: Control frequency in Hz
            speed_limit: Motor speed limit
            save_data: Whether to save trajectory data to CSV
            save_path: Path to save data file
        """
        print(f"Executing synchronized trajectories for {duration}s at {frequency}Hz...")
        
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
            
            # Calculate target positions for both motors
            relative_pos1 = trajectory_func1(t)
            relative_pos2 = trajectory_func2(t)
            target_pos1 = self.home_position1 + relative_pos1
            target_pos2 = self.home_position2 + relative_pos2
            
            # Send commands to both motors
            self.motor1.set_motor_position_control(limit_spd=speed_limit, loc_ref=target_pos1)
            self.motor2.set_motor_position_control(limit_spd=speed_limit, loc_ref=target_pos2)
            
            # Read motor feedback (simple approach - just log the commands)
            self.motor_data.append([t, target_pos1, target_pos2, None, None])
            
            loop_count += 1
            
            # Sleep until next loop time
            sleep_time = target_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        if save_data:
            self.save_data(save_path if save_path else None)
        
        print(f"Synchronized trajectory completed. Collected {len(self.motor_data)} data points.")
    
    def execute_motor1_trajectory(self, trajectory_func, duration, frequency=50.0, speed_limit=50):
        """Execute trajectory on motor 1 only."""
        print(f"Executing trajectory on motor 1 for {duration}s at {frequency}Hz...")
        
        dt = 1.0 / frequency
        start_time = time.perf_counter()
        loop_count = 0
        
        while True:
            target_time = start_time + loop_count * dt
            current_time = time.perf_counter()
            
            if current_time - start_time >= duration:
                break
                
            t = current_time - start_time
            relative_pos = trajectory_func(t)
            target_pos = self.home_position1 + relative_pos
            
            self.motor1.set_motor_position_control(limit_spd=speed_limit, loc_ref=target_pos)
            
            loop_count += 1
            
            sleep_time = target_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print("Motor 1 trajectory completed.")
    
    def execute_motor2_trajectory(self, trajectory_func, duration, frequency=50.0, speed_limit=50):
        """Execute trajectory on motor 2 only."""
        print(f"Executing trajectory on motor 2 for {duration}s at {frequency}Hz...")
        
        dt = 1.0 / frequency
        start_time = time.perf_counter()
        loop_count = 0
        
        while True:
            target_time = start_time + loop_count * dt
            current_time = time.perf_counter()
            
            if current_time - start_time >= duration:
                break
                
            t = current_time - start_time
            relative_pos = trajectory_func(t)
            target_pos = self.home_position2 + relative_pos
            
            self.motor2.set_motor_position_control(limit_spd=speed_limit, loc_ref=target_pos)
            
            loop_count += 1
            
            sleep_time = target_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print("Motor 2 trajectory completed.")
    
    def save_data(self, filename=None):
        """Save collected motor data to CSV file."""
        if filename is None:
            # Default behavior - save to data/ directory with timestamp
            os.makedirs("data", exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"data/dual_motor_data_{timestamp}.csv"
        
        # Use the full path as provided (no modification)
        print(f"Saving {len(self.motor_data)} data points to {filename}...")
        with open(filename, "w") as f:
            f.write("time,motor1_commanded,motor2_commanded,motor1_actual,motor2_actual\n")
            for data_point in self.motor_data:
                f.write(f"{data_point[0]:.6f},{data_point[1]:.6f},{data_point[2]:.6f},{data_point[3]},{data_point[4]}\n")
    
    def cleanup(self):
        """Disable motors and shutdown CAN bus."""
        print("Cleaning up dual motor controller...")
        
        try:
            self.motor1.disable()
            print("Motor 1 disabled")
        except Exception as e:
            print(f"Warning: Error disabling motor 1: {e}")
        
        try:
            self.motor2.disable()
            print("Motor 2 disabled")
        except Exception as e:
            print(f"Warning: Error disabling motor 2: {e}")
        
        try:
            self.bus.shutdown()
            print("CAN bus shut down")
        except Exception as e:
            print(f"Warning: Error shutting down CAN bus: {e}")
        
        print("Cleanup completed")

def main():
    # Create dual motor controller
    controller = DualMotorController(motor1_id=3, motor2_id=4)
    
    try:
        # Set home positions
        controller.set_home_positions()
        
        # Define trajectory functions
        def sine_trajectory(t):
            """Sine wave trajectory."""
            amplitude = 1.0
            frequency = 0.5
            return amplitude * math.sin(2 * math.pi * frequency * t)
        
        def cosine_trajectory(t):
            """Cosine wave trajectory (90 degrees out of phase)."""
            amplitude = 1.0
            frequency = 0.5
            return amplitude * math.cos(2 * math.pi * frequency * t)
        
        def triangle_wave(t):
            """Triangle wave trajectory."""
            period = 4.0
            return 2.0 * (2 * abs((t % period) / period - 0.5) - 0.5)
        def fast_trajectory(t):
            """Fast sine wave trajectory."""
            amplitude = 3.0
            frequency = 0.9
            return amplitude * math.sin(2 * math.pi * frequency * t)
        def slow_trajectory(t):
            """Slow sine wave trajectory."""
            amplitude = 1.0
            frequency = 0.1
            return amplitude * math.sin(2 * math.pi * frequency * t)
        # Demo 1: Synchronized trajectories
        # print("\n=== Demo 1: Synchronized Sine/Cosine Trajectories ===")
        # controller.execute_synchronized_trajectories(
        #     sine_trajectory, cosine_trajectory,
        #     duration=10.0, frequency=100.0, speed_limit=30
        # )
        
        # # Return to home
        # controller.go_home()
        # time.sleep(2)
        
        # # Demo 2: Motor 1 only
        # print("\n=== Demo 2: Motor 1 Triangle Wave ===")
        # controller.execute_motor1_trajectory(
        #     sine_trajectory, duration=5.0, frequency=700.0
        # )
        
        # # Demo 3: Motor 2 only  
        # print("\n=== Demo 3: Motor 2 Sine Wave ===")
        # controller.execute_motor2_trajectory(
        #     sine_trajectory, duration=5.0, frequency=50.0
        # )

        # #Demo 3: Synchronized trajectories
        # print("\n=== Demo 3: Synchronized Sine/Cosine Trajectories ===")
        # controller.execute_synchronized_trajectories(
        #     sine_trajectory, triangle_wave,
        #     duration=10.0, frequency=100.0, speed_limit=30, save_data=True, save_path="data/synchronized_trajectories.csv"
        # )   
       
        # # Demo 4: Fast trajectory
        # print("\n=== Demo 4: Fast Sine Wave Trajectory ===")
        # controller.execute_motor1_trajectory(
        #     fast_trajectory, duration=5.0, frequency=700.0
        # )
        
        # Final homing
        controller.go_home()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()
