import os
import sys
sys.path.append(os.path.join(".", "cybergear"))

from pcan_cybergear import CANMotorController

import can
import logging
import time
import threading

# Initialize logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def wait_for_user(message="Press Enter to continue..."):
    """Wait for user input before proceeding."""
    input(message)

def main():
    # Connect to CAN bus
    bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
    motor1 = CANMotorController(bus, motor_id=3, main_can_id=254)
    motor2 = CANMotorController(bus, motor_id=4, main_can_id=254)

    try:
        # reading motor status
        motor1.set_run_mode(motor1.RunModes.POSITION_MODE)
        motor1.enable()
        motor1.clear_can_rx(0)
        motor2.set_run_mode(motor1.RunModes.POSITION_MODE)
        motor2.enable()
        motor2.clear_can_rx(0)

        print("="*10, "reading motor 1")
        print(motor1.get_motor_status())
        print(motor1.get_motor_status())
        print(motor1.get_motor_status())
        print(motor1.get_motor_status())
        print(motor1.get_motor_status())
        
        print("="*10, "reading motor 2")
        print(motor2.get_motor_status())
        print(motor2.get_motor_status())
        print(motor2.get_motor_status())
        print(motor2.get_motor_status())
        print(motor2.get_motor_status())

        print("="*10, "mixed reading")
        print(motor1.get_motor_status())
        print(motor2.get_motor_status())
        print(motor2.get_motor_status())
        print(motor1.get_motor_status())
        print(motor2.get_motor_status())


    except Exception as e:
        logging.error(f"Error occurred: {e}")
    finally:
        logging.info("\nCleaning up...")
        motor1.disable()
        motor1.set_0_pos()
        motor2.disable()
        motor2.set_0_pos()
        bus.shutdown()
        logging.info("Demo completed. Motor disabled and CAN bus shut down.")

if __name__ == "__main__":
    main() 