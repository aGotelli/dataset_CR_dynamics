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
        
        # Move to different positions
        positions = [0, 3.14, -3.14, 0]
        
        for pos in positions:
            print(f"Moving to position: {pos:.2f} rad")
            motor.set_motor_position_control(limit_spd=6, loc_ref=pos)
            time.sleep(3)
        
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
