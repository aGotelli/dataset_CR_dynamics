import os
import sys
sys.path.append(os.path.join(".", "cybergear"))

from pcan_cybergear import CANMotorController
import can
import time

def main():
    # Connect to CAN bus
    bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
    
    # Initialize motor
    motor = CANMotorController(bus, motor_id=3, main_can_id=254)
    
    try:
        # Set position mode and enable motor
        motor.set_run_mode(motor.RunModes.POSITION_MODE)
        motor.enable()
        
        # Perform sine trajectory for 5 seconds
        print("Starting sine trajectory for 5 seconds...")
        import math
        
        # Perform continuous rotation for 5 seconds at 200Hz
        print("Starting continuous rotation for 5 seconds at 200Hz...")
        import math
        
        duration = 5.0  # 5 seconds
        frequency = 200.0  # 200Hz
        dt = 1.0 / frequency  # 0.005 seconds per loop
        rotations_per_second = 0.5  # 0.5 rotations per second
        
        start_time = time.perf_counter()
        loop_count = 0
        
        while True:
            target_time = start_time + loop_count * dt
            current_time = time.perf_counter()
            
            if current_time - start_time >= duration:
                break
                
            t = current_time - start_time
            position = 2 * math.pi * rotations_per_second * t
            motor.set_motor_position_control(limit_spd=15, loc_ref=position)
            
            loop_count += 1
            
            # Sleep until next loop time
            sleep_time = target_time - current_time
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # Return to zero position
        print("Returning to zero position...")
        motor.set_motor_position_control(limit_spd=6, loc_ref=0)
        time.sleep(2)
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        try:
            motor.disable()
        except Exception as e:
            print(f"Warning: Error disabling motor: {e}")
        try:
            bus.shutdown()
        except Exception as e:
            print(f"Warning: Error shutting down CAN bus: {e}")
        print("Motor disabled and CAN bus shut down.")

if __name__ == "__main__":
    main()
