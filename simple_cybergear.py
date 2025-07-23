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
    motor = CANMotorController(bus, motor_id=3, main_can_id=254)
    motor2 = CANMotorController(bus, motor_id=4, main_can_id=254)

    try:
        # Demo 1: Position Mode
        logging.info("\n=== Demo 1: Position Mode ===")
        logging.info("The motor will move to different positions: 0 -> π -> -π -> 0")
        wait_for_user("Press Enter to start position mode demo...")
        
        motor.set_run_mode(motor.RunModes.POSITION_MODE)
        motor.enable()
        
        # Move to different positions
        positions = [0, 3.14, -3.14, 0, 17]
        for pos in positions:
            logging.info(f"Moving to position: {pos:.2f} rad")
            motor.set_motor_position_control(limit_spd=6, loc_ref=pos)
            time.sleep(3)
        
        motor.disable()
        wait_for_user("\nPosition mode demo completed. Press Enter to continue...")

        # Demo 2: Speed Mode
        logging.info("\n=== Demo 2: Speed Mode ===")
        logging.info("The motor will run at different speeds: 2 -> -2 -> 4 -> -4 -> 0 rad/s")
        wait_for_user("Press Enter to start speed mode demo...")
        
        motor.set_run_mode(motor.RunModes.SPEED_MODE)
        motor.enable()

        motor2.set_run_mode(motor.RunModes.SPEED_MODE)
        motor2.enable()
        motor2.write_single_param("spd_ref", value=0.5)
        
        # Test different speeds
        speeds = [2, -2, 4, -4, 0]
        for speed in speeds:
            logging.info(f"\nSetting speed to: {speed} rad/s")
            motor.write_single_param("spd_ref", value=speed)
            time.sleep(2)
        
        motor.disable()
        motor2.disable()
        wait_for_user("\nSpeed mode demo completed. Press Enter to continue...")

        # Demo 3: Current Mode
        logging.info("\n=== Demo 3: Current Mode ===")
        logging.info("The motor will run with different current values: 0.5A -> -0.5A -> 1A -> -1A -> 0A")
        wait_for_user("Press Enter to start current mode demo...")
        
        motor.set_run_mode(motor.RunModes.CURRENT_MODE)
        motor.enable()
        
        # Test different currents
        currents = [0.5, -0.5, 1.0, -1.0, 0.0]
        for current in currents:
            logging.info(f"\nSetting current to: {current} A")
            motor.write_single_param("iq_ref", value=current)
            time.sleep(2)
        
        motor.disable()
        wait_for_user("\nCurrent mode demo completed. Press Enter to continue...")

        # Demo 4: Motion Control Mode
        logging.info("\n=== Demo 4: Motion Control Mode ===")
        logging.info("The motor will perform motion control with different parameters")
        wait_for_user("Press Enter to start motion control mode demo...")
        
        motor.set_run_mode(motor.RunModes.CONTROL_MODE)
        motor.enable()
        
        # Test motion control with different parameters
        for i in range(3):
            logging.info(f"\nMotion control cycle {i+1}/3")
            motor.send_motor_control_command(
                torque=0.0,
                target_angle=10.0,
                target_velocity=1.0,
                Kp=0.01,
                Kd=0.2
            )
            time.sleep(5)
            
            motor.send_motor_control_command(
                torque=0.0,
                target_angle=-10.0,
                target_velocity=1.0,
                Kp=0.01,
                Kd=0.2
            )
            time.sleep(5)
        
        motor.disable()
        wait_for_user("\nMotion control mode demo completed. Press Enter to continue...")

    except Exception as e:
        logging.error(f"Error occurred: {e}")
        import traceback
        logging.error(f"Full traceback: {traceback.format_exc()}")
    finally:
        logging.info("\nCleaning up...")
        try:
            motor.disable()
        except Exception as e:
            logging.warning(f"Error disabling motor: {e}")
        try:
            motor.set_0_pos()
        except Exception as e:
            logging.warning(f"Error setting motor to zero position: {e}")
        try:
            bus.shutdown()
        except Exception as e:
            logging.warning(f"Error shutting down CAN bus: {e}")
        logging.info("Demo completed. Motor disabled and CAN bus shut down.")

if __name__ == "__main__":
    main() 