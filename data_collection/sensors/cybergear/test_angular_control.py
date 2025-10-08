import os
import sys
sys.path.append(os.path.join(".", "cybergear"))

from pcan_cybergear import CANMotorController
import can
import logging
import time
import math
import numpy as np

# Initialize logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
def get_current_angle(motor, max_retries=3, retry_delay=0.1):
    """Get current motor angle in degrees with retry mechanism"""
    for attempt in range(max_retries):
        try:
            status = motor.get_motor_status()
            if status[0] is not None:
                angle_rad = status[1]
                angle_deg = math.degrees(angle_rad)
                return angle_rad, angle_deg
            else:
                logging.warning(f"Status returned None (attempt {attempt+1}/{max_retries})")
        except Exception as e:
            logging.warning(f"Error reading angle (attempt {attempt+1}/{max_retries}): {e}")
        
        if attempt < max_retries - 1:  # Don't sleep on last attempt
            time.sleep(retry_delay)
    
    logging.error(f"Failed to read angle after {max_retries} attempts")
    return None, None

def get_current_angle_robust(motor, max_retries=5, retry_delay=0.2):
    """Get current motor angle with more aggressive retry for critical operations"""
    for attempt in range(max_retries):
        try:
            status = motor.get_motor_status()
            if status and len(status) > 1 and status[0] is not None:
                angle_rad = status[1]
                angle_deg = math.degrees(angle_rad)
                logging.debug(f"Successfully read angle: {angle_deg:.2f}° (attempt {attempt+1})")
                return angle_rad, angle_deg
            else:
                logging.warning(f"Invalid status format or None data (attempt {attempt+1}/{max_retries})")
        except Exception as e:
            logging.warning(f"Exception reading status (attempt {attempt+1}/{max_retries}): {e}")
        
        if attempt < max_retries - 1:
            time.sleep(retry_delay)
    
    logging.error(f"CRITICAL: Failed to read motor position after {max_retries} attempts")
    return None, None

# def get_current_angle(motor):
#     """Get current motor angle in degrees"""
#     try:
#         status = motor.get_motor_status()
#         if status[0] is not None:
#             angle_rad = status[1]
#             angle_deg = math.degrees(angle_rad)
#             return angle_rad, angle_deg
#     except Exception as e:
#         logging.error(f"Error reading angle: {e}")
#     return None, None

def sinusoidal_trajectory_simple(amplitude_deg=30, frequency=0.5, duration=10, sample_rate=10):
    """Generate simple sinusoidal trajectory starting from 0"""
    t = np.linspace(0, duration, int(duration * sample_rate))
    angles_deg = amplitude_deg * np.sin(2 * np.pi * frequency * t)
    return t, angles_deg

def sinusoidal_trajectory_offset(base_angle_deg, amplitude_deg=30, frequency=0.5, duration=10, sample_rate=10):
    """Generate sinusoidal trajectory starting from base angle"""
    t = np.linspace(0, duration, int(duration * sample_rate))
    angles_deg = base_angle_deg + amplitude_deg * np.sin(2 * np.pi * frequency * t)
    return t, angles_deg

def execute_trajectory(motor, time_points, angles_deg):
    """Execute trajectory and monitor motor status"""
    logging.info(f"Executing trajectory with {len(angles_deg)} points over {time_points[-1]:.1f} seconds")
    logging.info(f"Amplitude: ±{max(abs(angles_deg - np.mean(angles_deg))):.1f}°, Frequency: ~{1/(time_points[1]-time_points[0])/len(time_points)*2:.2f} Hz")
    
    start_time = time.time()
    
    for i, (t, target_angle_deg) in enumerate(zip(time_points, angles_deg)):
        # Wait for the right time
        while time.time() - start_time < t:
            time.sleep(0.01)
        
        # Send position command
        target_angle_rad = math.radians(target_angle_deg)
        motor.set_motor_position_control(limit_spd=6, loc_ref=target_angle_rad)
        
        # Read current status
        angle_rad, angle_deg = get_current_angle(motor)
        if angle_deg is not None:
            error = target_angle_deg - angle_deg
            elapsed = time.time() - start_time
            logging.info(f"t={elapsed:.2f}s | Target: {target_angle_deg:+6.1f}° | Current: {angle_deg:+6.1f}° | Error: {error:+5.1f}°")
        else:
            logging.warning(f"t={t:.2f}s | Target: {target_angle_deg:+6.1f}° | No status received")

def main():
    try:
        # Connect to CAN bus
        bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
        motor = CANMotorController(bus, motor_id=3, main_can_id=254)
        
        logging.info("Connected to motor (ID: 3)")
        
        # Set to position mode
        motor.set_run_mode(motor.RunModes.POSITION_MODE)
        motor.enable()
        
        # Read initial angle
        angle_rad, angle_deg = get_current_angle(motor)
        if angle_deg is None:
            logging.error("Cannot read initial angle!")
            return
        
        logging.info(f"Initial angle: {angle_deg:.2f}°")
        initial_angle_deg = angle_deg
        
        # Interactive menu
        while True:
            print("\n" + "="*50)
            print("SINUSOIDAL TRAJECTORY TEST")
            print("="*50)
            print("1. Manual angle increment")
            print("2. Simple sinusoidal (starts from 0°)")
            print("3. Sinusoidal from current position")
            print("4. Move to increment + sinusoidal from there")
            print("q. Quit")
            
            choice = input("\nEnter your choice: ").strip()
            
            if choice.lower() == 'q':
                break
            elif choice == '1':
                # Original manual control
                try:
                    increment = input(f"\nEnter angle increment in degrees: ")
                    increment_deg = float(increment)
                    target_angle_deg = initial_angle_deg + increment_deg
                    target_angle_rad = math.radians(target_angle_deg)
                    
                    logging.info(f"Moving to {target_angle_deg:.2f}° (increment: {increment_deg:+.1f}°)")
                    
                    motor.set_motor_position_control(limit_spd=3, loc_ref=target_angle_rad)
                    
                    # Monitor for 5 seconds
                    for i in range(10):
                        time.sleep(0.5)
                        angle_rad, angle_deg = get_current_angle(motor)
                        if angle_deg is not None:
                            error = target_angle_deg - angle_deg
                            logging.info(f"  Current: {angle_deg:.2f}° | Target: {target_angle_deg:.1f}° | Error: {error:.2f}°")
                    
                except ValueError:
                    logging.error("Invalid input! Please enter a number.")
                    
            elif choice == '2':
                # Simple sinusoidal trajectory
                try:
                    amplitude = float(input("Enter amplitude in degrees (default 30°): ") or "30")
                    frequency = float(input("Enter frequency in Hz (default 0.5): ") or "0.5")
                    duration = float(input("Enter duration in seconds (default 10): ") or "10")
                    
                    logging.info(f"\nGenerating simple sinusoidal trajectory:")
                    logging.info(f"  Amplitude: ±{amplitude}°")
                    logging.info(f"  Frequency: {frequency} Hz")
                    logging.info(f"  Duration: {duration}s")
                    
                    t, angles = sinusoidal_trajectory_simple(amplitude, frequency, duration)
                    
                    input("\nPress Enter to start trajectory...")
                    execute_trajectory(motor, t, angles)
                    
                except ValueError:
                    logging.error("Invalid input! Please enter valid numbers.")
                    
            elif choice == '3':
                # Sinusoidal from current position
                try:
                    # Read current angle again
                    angle_rad, current_angle_deg = get_current_angle(motor)
                    if current_angle_deg is None:
                        logging.error("Cannot read current angle!")
                        continue
                    
                    amplitude = float(input("Enter amplitude in degrees (default 30°): ") or "30")
                    frequency = float(input("Enter frequency in Hz (default 0.5): ") or "0.5")
                    duration = float(input("Enter duration in seconds (default 10): ") or "10")
                    
                    logging.info(f"\nGenerating sinusoidal trajectory from current position:")
                    logging.info(f"  Base angle: {current_angle_deg:.1f}°")
                    logging.info(f"  Amplitude: ±{amplitude}°")
                    logging.info(f"  Frequency: {frequency} Hz")
                    logging.info(f"  Duration: {duration}s")
                    
                    t, angles = sinusoidal_trajectory_offset(current_angle_deg, amplitude, frequency, duration)
                    
                    input("\nPress Enter to start trajectory...")
                    execute_trajectory(motor, t, angles)
                    
                except ValueError:
                    logging.error("Invalid input! Please enter valid numbers.")
                    
            elif choice == '4':
                # Move to increment + sinusoidal from there
                try:
                    increment = input(f"\nEnter angle increment in degrees: ")
                    increment_deg = float(increment)
                    target_angle_deg = initial_angle_deg + increment_deg
                    target_angle_rad = math.radians(target_angle_deg)
                    
                    logging.info(f"Step 1: Moving to {target_angle_deg:.2f}° (increment: {increment_deg:+.1f}°)")
                    
                    # Move to target position first
                    motor.set_motor_position_control(limit_spd=3, loc_ref=target_angle_rad)
                    
                    # Wait for motor to reach position (monitor for 3 seconds)
                    logging.info("Waiting for motor to reach target position...")
                    for i in range(6):
                        time.sleep(0.5)
                        angle_rad, angle_deg = get_current_angle(motor)
                        if angle_deg is not None:
                            error = target_angle_deg - angle_deg
                            logging.info(f"  Current: {angle_deg:.2f}° | Target: {target_angle_deg:.1f}° | Error: {error:.2f}°")
                    
                    # Read actual position before starting sinusoid
                    angle_rad, actual_angle_deg = get_current_angle_robust(motor)
                    if actual_angle_deg is None:
                        logging.error("Cannot read actual position!")
                        continue
                    
                    logging.info(f"Actual position reached: {actual_angle_deg:.2f}°")
                    
                    # Get sinusoidal parameters
                    amplitude = float(input("Enter sinusoidal amplitude in degrees (default 30°): ") or "30")
                    frequency = float(input("Enter frequency in Hz (default 0.5): ") or "0.5")
                    duration = float(input("Enter duration in seconds (default 10): ") or "10")
                    
                    logging.info(f"\nStep 2: Starting sinusoidal trajectory from {actual_angle_deg:.1f}°:")
                    logging.info(f"  Base angle: {actual_angle_deg:.1f}°")
                    logging.info(f"  Amplitude: ±{amplitude}°")
                    logging.info(f"  Frequency: {frequency} Hz")
                    logging.info(f"  Duration: {duration}s")
                    
                    # Generate sinusoidal trajectory from actual position
                    t, angles = sinusoidal_trajectory_offset(actual_angle_deg, amplitude, frequency, duration)
                    
                    input("\nPress Enter to start sinusoidal trajectory...")
                    execute_trajectory(motor, t, angles)
                    
                except ValueError:
                    logging.error("Invalid input! Please enter valid numbers.")
            else:
                print("Invalid choice!")
        
        motor.disable()
        
    except Exception as e:
        logging.error(f"Error: {e}")
    except KeyboardInterrupt:
        logging.info("Interrupted by user")
    finally:
        try:
            motor.disable()
            motor.set_0_pos()
            bus.shutdown()
        except:
            pass
        logging.info("Test completed.")

if __name__ == "__main__":
    main()



# # # # import os
# # # # import sys
# # # # sys.path.append(os.path.join(".", "cybergear"))

# # # # from pcan_cybergear import CANMotorController
# # # # import can
# # # # import logging
# # # # import time
# # # # import math

# # # # # Initialize logging
# # # # logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# # # # def get_current_angle(motor):
# # # #     """Get current motor angle in degrees"""
# # # #     try:
# # # #         status = motor.get_motor_status()
# # # #         if status[0] is not None:
# # # #             angle_rad = status[1]
# # # #             angle_deg = math.degrees(angle_rad)
# # # #             return angle_rad, angle_deg
# # # #     except Exception as e:
# # # #         logging.error(f"Error reading angle: {e}")
# # # #     return None, None

# # # # def main():
# # # #     try:
# # # #         # Connect to CAN bus
# # # #         bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
# # # #         motor = CANMotorController(bus, motor_id=3, main_can_id=254)
        
# # # #         logging.info("Connected to motor (ID: 3)")
        
# # # #         # Set to position mode
# # # #         motor.set_run_mode(motor.RunModes.POSITION_MODE)
# # # #         motor.enable()
        
# # # #         # Read initial angle
# # # #         angle_rad, angle_deg = get_current_angle(motor)
# # # #         if angle_deg is None:
# # # #             logging.error("Cannot read initial angle!")
# # # #             return
        
# # # #         logging.info(f"Initial angle: {angle_deg:.2f}°")
# # # #         initial_angle_deg = angle_deg
        
# # # #         # Interactive loop
# # # #         while True:
# # # #             try:
# # # #                 increment = input(f"\nEnter angle increment in degrees (or 'q' to quit): ")
# # # #                 if increment.lower() == 'q':
# # # #                     break
                    
# # # #                 increment_deg = float(increment)
# # # #                 target_angle_deg = initial_angle_deg + increment_deg
# # # #                 target_angle_rad = math.radians(target_angle_deg)
                
# # # #                 logging.info(f"Moving to {target_angle_deg:.2f}° (increment: {increment_deg:+.1f}°)")
                
# # # #                 # Send position command
# # # #                 motor.set_motor_position_control(limit_spd=3, loc_ref=target_angle_rad)
                
# # # #                 # Monitor for 5 seconds
# # # #                 for i in range(10):
# # # #                     time.sleep(0.5)
# # # #                     angle_rad, angle_deg = get_current_angle(motor)
# # # #                     if angle_deg is not None:
# # # #                         error = target_angle_deg - angle_deg
# # # #                         logging.info(f"  Current: {angle_deg:.2f}° | Target: {target_angle_deg:.1f}° | Error: {error:.2f}°")
                
# # # #             except ValueError:
# # # #                 logging.error("Invalid input! Please enter a number.")
# # # #             except KeyboardInterrupt:
# # # #                 break
        
# # # #         motor.disable()
        
# # # #     except Exception as e:
# # # #         logging.error(f"Error: {e}")
# # # #     finally:
# # # #         try:
# # # #             motor.disable()
# # # #             motor.set_0_pos()
# # # #             bus.shutdown()
# # # #         except:
# # # #             pass
# # # #         logging.info("Test completed.")

# # # # if __name__ == "__main__":
# # # #     main()