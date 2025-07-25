import os
import sys
sys.path.append(os.path.join(".", "cybergear"))

from pcan_cybergear import CANMotorController

import can
import logging
import time
import threading

# Initialize logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def wait_for_user(message="Press Enter to continue..."):
    """Wait for user input before proceeding."""
    input(message)

def main():
    # Connect to CAN bus
    bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
    
    # Revert to original working motor IDs
    motor = CANMotorController(bus, motor_id=3, main_can_id=254)
    motor2 = CANMotorController(bus, motor_id=4, main_can_id=254)
    
    # logging.info("=== CAN Message Analysis ===")
    # logging.info(f"Motor 1: motor_id={motor.MOTOR_ID}, main_can_id={motor.MAIN_CAN_ID}")
    # logging.info(f"Motor 2: motor_id={motor2.MOTOR_ID}, main_can_id={motor2.MAIN_CAN_ID}")
    # logging.info("Motor IDs are correct - focusing on message interpretation...")
    
    # Add comprehensive CAN monitoring
    def enhanced_can_monitor():
        logging.info("Starting enhanced CAN message monitoring...")
        while True:
            try:
                msg = bus.recv(timeout=0.1)
                if msg:
                    cmd = (msg.arbitration_id >> 24) & 0xFF
                    main_id = (msg.arbitration_id >> 8) & 0xFF  
                    motor_id = msg.arbitration_id & 0xFF
                    
                    # Decode command type
                    cmd_names = {
                        0: "GET_DEVICE_ID",
                        1: "MOTOR_CONTROL", 
                        2: "MOTOR_FEEDBACK",
                        3: "MOTOR_ENABLE",
                        4: "MOTOR_STOP",
                        6: "SET_MECHANICAL_ZERO",
                        7: "SET_MOTOR_CAN_ID",
                        8: "PARAM_TABLE_WRITE",
                        17: "SINGLE_PARAM_READ",
                        18: "SINGLE_PARAM_WRITE",
                        21: "FAULT_FEEDBACK"
                    }
                    cmd_name = cmd_names.get(cmd, f"UNKNOWN({cmd})")
                    
                    logging.info(f"CAN: {hex(msg.arbitration_id)} | {cmd_name} | Main:{main_id} Motor:{motor_id} | Data:{[hex(b) for b in msg.data]}")
                    
                    # Special analysis for your expected vs actual
                    if msg.arbitration_id == 0x20003fe:
                        logging.info("*** FOUND YOUR EXPECTED ID 0x20003fe! ***")
                    elif cmd == 2:  # MOTOR_FEEDBACK  
                        logging.info(f"*** MOTOR FEEDBACK from motor {motor_id} - This might be what you're looking for! ***")
                    elif cmd == 18:  # Your current received message
                        logging.info(f"*** PARAM WRITE RESPONSE from motor {motor_id} ***")
                        
            except Exception as e:
                logging.debug(f"CAN monitor exception: {e}")
                break
    
    monitor_thread = threading.Thread(target=enhanced_can_monitor, daemon=True)
    monitor_thread.start()
    
    logging.info("=== Starting Motor Demo with Enhanced Monitoring ===")
    
    # Let's try to get device ID first
    logging.info("Step 1: Getting device ID...")
    try:
        data, arb_id = motor.send_can_message(motor.CmdModes.GET_DEVICE_ID, [0]*8, 0)
        if data:
            logging.info(f"Device ID response: {[hex(b) for b in data]}")
    except Exception as e:
        logging.error(f"Error getting device ID: {e}")

    try:
        # Analysis of the message issue
        logging.info("\n=== Message Format Analysis ===")
        logging.info("Your EXPECTED:")
        logging.info("  ID: 0x20003fe = MOTOR_FEEDBACK (cmd=2) from motor 0xfe to main 0x03")
        logging.info("  Data: [0x80, 0x0, 0x80, 0x32, 0x7f, 0xff, 0x1, 0x1e] = Motor state feedback")
        logging.info("")
        logging.info("Your ACTUAL:")
        logging.info("  ID: 0x1200fe03 = SINGLE_PARAM_WRITE response (cmd=18) from motor 3 to main 254")  
        logging.info("  Data: [0x5, 0x70, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0] = Parameter write confirmation")
        logging.info("")
        logging.info("EXPLANATION: You're seeing the response to parameter writes, not motor feedback!")
        logging.info("Motor feedback (your expected format) comes automatically when motor is running.")
        logging.info("")

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