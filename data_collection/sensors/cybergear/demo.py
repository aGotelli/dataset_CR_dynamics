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
        positions = [0, 3.14, -3.14, 0, 5]
        for i, pos in enumerate(positions):
            logging.info(f"Moving to position {i+1}/{len(positions)}: {pos:.2f} rad")
            motor.set_motor_position_control(limit_spd=6, loc_ref=pos)
            
            # Get motor status during movement
            for j in range(3):  # Check status 3 times during each movement
                time.sleep(1)
                status = motor.get_motor_status()
                if status[0] is not None:
                    logging.info(f"  → pos: {status[1]:.2f} rad, vel: {status[2]:.2f} rad/s, torque: {status[3]:.2f} Nm, temp: {status[4]:.1f}°C")
                else:
                    logging.info(f"  → No status update")
        
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
        for i, speed in enumerate(speeds):
            logging.info(f"Setting speed {i+1}/{len(speeds)}: {speed} rad/s")
            motor.write_single_param("spd_ref", value=speed)
            
            # Get motor status during speed changes
            for j in range(2):  # Check status 2 times during each speed
                time.sleep(1)
                status = motor.get_motor_status()
                if status[0] is not None:
                    logging.info(f"  → pos: {status[1]:.2f} rad, vel: {status[2]:.2f} rad/s, torque: {status[3]:.2f} Nm, temp: {status[4]:.1f}°C")
                else:
                    logging.info(f"  → No status update")
        
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
        for i, current in enumerate(currents):
            logging.info(f"Setting current {i+1}/{len(currents)}: {current} A")
            motor.write_single_param("iq_ref", value=current)
            
            # Get motor status during current changes
            for j in range(2):  # Check status 2 times during each current
                time.sleep(1)
                status = motor.get_motor_status()
                if status[0] is not None:
                    logging.info(f"  → pos: {status[1]:.2f} rad, vel: {status[2]:.2f} rad/s, torque: {status[3]:.2f} Nm, temp: {status[4]:.1f}°C")
                else:
                    logging.info(f"  → No status update")
        
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
            logging.info(f"Motion control cycle {i+1}/3")
            logging.info(f"  Moving to +10°...")
            motor.send_motor_control_command(
                torque=0.0,
                target_angle=10.0,
                target_velocity=1.0,
                Kp=0.01,
                Kd=0.2
            )
            
            # Get motor status during motion control
            for j in range(3):  # Check status 3 times during each motion
                time.sleep(1.5)
                status = motor.get_motor_status()
                if status[0] is not None:
                    logging.info(f"    → pos: {status[1]:.2f} rad, vel: {status[2]:.2f} rad/s, torque: {status[3]:.2f} Nm, temp: {status[4]:.1f}°C")
                else:
                    logging.info(f"    → No status update")
            
            logging.info(f"  Moving to -10°...")
            motor.send_motor_control_command(
                torque=0.0,
                target_angle=-10.0,
                target_velocity=1.0,
                Kp=0.01,
                Kd=0.2
            )
            
            # Get motor status during motion control
            for j in range(3):  # Check status 3 times during each motion
                time.sleep(1.5)
                status = motor.get_motor_status()
                if status[0] is not None:
                    logging.info(f"    → pos: {status[1]:.2f} rad, vel: {status[2]:.2f} rad/s, torque: {status[3]:.2f} Nm, temp: {status[4]:.1f}°C")
                else:
                    logging.info(f"    → No status update")
        
        motor.disable()
        wait_for_user("\nMotion control mode demo completed. Press Enter to continue...")

    except Exception as e:
        logging.error(f"Error occurred: {e}")
    finally:
        logging.info("\nCleaning up...")
        motor.disable()
        motor.set_0_pos()
        bus.shutdown()
        logging.info("Demo completed. Motor disabled and CAN bus shut down.")

if __name__ == "__main__":
    main() 