"""
Simple Motor Pretensioning for Synchronized Acquisition
Based on working motor_pretension.py - adapted to use existing bus and inputs
"""

import os
import sys
sys.path.append(os.path.join(".", "cybergear"))

from pcan_cybergear import CANMotorController
import can
import time
import serial

class MotorPretensioner:
    def __init__(self, motor_id, bus, mark10_com_port, target_tension, direction, speed=5):
        self.motor_id = motor_id
        self.bus = bus
        self.motor = CANMotorController(bus, motor_id=motor_id, main_can_id=254)
        self.motor.set_run_mode(self.motor.RunModes.POSITION_MODE)
        self.motor.enable()
        self.serial = serial.Serial(mark10_com_port, baudrate=115200, timeout=0.2)
        self.target_tension = target_tension
        self.direction = direction  # 1 for forward, -1 for backward
        self.speed = speed
        self.angle = 0.0
        self.stop_requested = False

    def read_tension(self):
        import re
        try:
            self.serial.write("?\r".encode())
            response = self.serial.readline().decode('utf-8', errors='ignore').strip()
            # print(f"[DEBUG] Raw Mark10 response: '{response}'")
            if not response:
                print("[ERROR] Mark10 returned empty response!")
                return 0.0
            match = re.search(r'-?\d+(\.\d+)?', response)
            if match:
                raw_value = float(match.group(0))
                # print(f"[DEBUG] Raw value from match: {raw_value}")
                return raw_value  # Return the raw value from match
            else:
                print(f"[ERROR] Could not parse tension value from: '{response}'")
                return 0.0
        except Exception as e:
            print(f"[ERROR] Exception in read_tension: {e}")
        return 0.0

    def pretension(self, max_angle_change=2.0, step_size=0.01, wait_time=0.2):
        current_angle = self.angle
        max_steps = int(max_angle_change / step_size)
        tension = self.read_tension()
        tolerance = 0.01
        for _ in range(max_steps):
            if self.stop_requested:
                print(f"Motor {self.motor_id}: pretension interrupted by user.")
                break
            abs_tension = abs(tension)  # Use absolute value for comparison
            abs_target = abs(self.target_tension)
            #print(f"\rMotor {self.motor_id}: angle = {current_angle:.4f} rad, raw_tension = {tension:.4f} N, |tension| = {abs_tension:.4f} N, target = {abs_target:.4f} N")
            if abs(abs_tension - abs_target) <= tolerance:
                self.angle = current_angle
                return current_angle, abs_tension  # Return the absolute tension used in comparison
            elif abs_tension < abs_target:
                current_angle += self.direction * step_size
                self.motor.set_motor_position_control(limit_spd=self.speed, loc_ref=current_angle)
            else:
                current_angle -= self.direction * step_size
                self.motor.set_motor_position_control(limit_spd=self.speed, loc_ref=current_angle)
            # Read new tension for next iteration
            tension = self.read_tension()
            time.sleep(wait_time)
        self.angle = current_angle
        # Return the last absolute tension reading we used
        return current_angle, abs(tension)

    def checkTensionReached(self, tolerance=0.01):
        """Check if current tension is within tolerance of target tension"""
        current_tension = abs(self.read_tension())
        target_tension = abs(self.target_tension)
        return abs(current_tension - target_tension) <= tolerance

    def simple_tension_control(self, step_size=0.01, wait_time=0.2, tolerance=0.01):
        """
        Simple tension control loop that runs until target tension is reached.
        
        Args:
            step_size: Small rotation increment (radians)
            wait_time: Delay between control steps (seconds)
            tolerance: Acceptable tension error (N)
        """
        while not self.checkTensionReached(tolerance):
            if self.stop_requested:
                print(f"Motor {self.motor_id}: tension control interrupted by user.")
                break
                
            current_tension = self.read_tension()
            abs_current = abs(current_tension)
            abs_target = abs(self.target_tension)
            
            print(f"Motor {self.motor_id}: tension = {abs_current:.4f} N, target = {abs_target:.4f} N")
            
            # Control actuator based on tension reading
            if abs_current < abs_target:
                # Tension too low - increase tension
                self.angle += self.direction * step_size
            else:
                # Tension too high - decrease tension
                self.angle -= self.direction * step_size
            
            # Apply the new position with slow motion
            self.motor.set_motor_position_control(limit_spd=self.speed, loc_ref=self.angle)
            time.sleep(wait_time)
        
        print(f"Motor {self.motor_id}: Target tension reached! Final angle = {self.angle:.4f} rad")
        return self.angle, abs(self.read_tension())

    def cleanup(self, close_bus=False, disable_motors=False):
        if disable_motors:  # Only disable motors if explicitly requested
            self.motor.disable()
        if close_bus:  # Only close bus if explicitly requested (not for embedded use)
            self.bus.shutdown()
        
        # Ensure serial port is properly closed
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                print(f"Serial port {self.serial.port} closed successfully")
            time.sleep(0.5)  # Give the OS time to release the port
        except Exception as e:
            print(f"Warning: Error closing serial port: {e}")


def dual_motor_tension_control(pretensioner_1, pretensioner_2, step_size=0.01, wait_time=0.2, tolerance=0.01):
    """
    Simple tension control for both actuators simultaneously.
    
    Args:
        pretensioner_1: First MotorPretensioner instance
        pretensioner_2: Second MotorPretensioner instance
        step_size: Small rotation increment (radians)
        wait_time: Delay between control steps (seconds)
        tolerance: Acceptable tension error (N)
    
    Returns:
        Tuple of (angle1, tension1, angle2, tension2)
    """
    print("Starting dual motor tension control...")
    
    def both_motors_reached_target():
        """Check if both motors have reached their target tensions"""
        return (pretensioner_1.checkTensionReached(tolerance) and 
                pretensioner_2.checkTensionReached(tolerance))
    
    while not both_motors_reached_target():
        if pretensioner_1.stop_requested or pretensioner_2.stop_requested:
            print("Dual motor tension control interrupted by user.")
            break
        
        # Read current tensions for both motors
        tension_1 = pretensioner_1.read_tension()
        tension_2 = pretensioner_2.read_tension()
        
        abs_tension_1 = abs(tension_1)
        abs_tension_2 = abs(tension_2)
        abs_target_1 = abs(pretensioner_1.target_tension)
        abs_target_2 = abs(pretensioner_2.target_tension)
        
        print(f"Motor {pretensioner_1.motor_id}: tension = {abs_tension_1:.4f} N, target = {abs_target_1:.4f} N")
        print(f"Motor {pretensioner_2.motor_id}: tension = {abs_tension_2:.4f} N, target = {abs_target_2:.4f} N")
        
        # Control motor 1 if not at target
        if not pretensioner_1.checkTensionReached(tolerance):
            if abs_tension_1 < abs_target_1:
                pretensioner_1.angle += pretensioner_1.direction * step_size
            else:
                pretensioner_1.angle -= pretensioner_1.direction * step_size
            pretensioner_1.motor.set_motor_position_control(
                limit_spd=pretensioner_1.speed, 
                loc_ref=pretensioner_1.angle
            )
        
        # Control motor 2 if not at target
        if not pretensioner_2.checkTensionReached(tolerance):
            if abs_tension_2 < abs_target_2:
                pretensioner_2.angle += pretensioner_2.direction * step_size
            else:
                pretensioner_2.angle -= pretensioner_2.direction * step_size
            pretensioner_2.motor.set_motor_position_control(
                limit_spd=pretensioner_2.speed, 
                loc_ref=pretensioner_2.angle
            )
        
        time.sleep(wait_time)
    
    print("Both motors have reached their target tensions!")
    final_tension_1 = abs(pretensioner_1.read_tension())
    final_tension_2 = abs(pretensioner_2.read_tension())
    
    print(f"Final Motor {pretensioner_1.motor_id}: angle = {pretensioner_1.angle:.4f} rad, tension = {final_tension_1:.4f} N")
    print(f"Final Motor {pretensioner_2.motor_id}: angle = {pretensioner_2.angle:.4f} rad, tension = {final_tension_2:.4f} N")
    
    return pretensioner_1.angle, final_tension_1, pretensioner_2.angle, final_tension_2


def run_embedded_pretensioning(motor_controller, mark10_sensors):
    """
    Run pretensioning using existing motor controller - adapted from motor_pretension.py
    Uses the exact same logic but with inputs from synchronized_acquisition
    
    Args:
        motor_controller: DualMotorController instance (has .bus attribute)
        mark10_sensors: List of SimpleMark10 instances (not used, we create our own serial connections)
        
    Returns:
        str: Summary string for README
    """
    print("\n🔧 Starting embedded motor pretensioning...")
    
    try:
        if not motor_controller:
            return "## Pretensioning Results\nNo motor controller available.\n"
        
        # Use the existing CAN bus from motor controller
        bus = motor_controller.bus
        
        # Create pretensioners exactly like motor_pretension.py
        # Motor ID 3 uses COM5, Motor ID 4 uses COM4
        pretensioner_3 = MotorPretensioner(motor_id=3, bus=bus, mark10_com_port="COM5", target_tension=2, direction=1, speed=5)
        pretensioner_4 = MotorPretensioner(motor_id=4, bus=bus, mark10_com_port="COM4", target_tension=2, direction=-1, speed=5)

        def pretension_and_print(pretensioner):
            try:
                angle, tension = pretensioner.pretension()
                print(f"\rPretension: angle={angle:.4f} rad, tension={tension:.4f} N")
                return angle, tension
            except Exception as e:
                print(f"Motor {pretensioner.motor_id}: Exception in pretension: {e}")
                return None, None

        # Execute pretensioning exactly like motor_pretension.py
        try:
            print("--- Pretensioning Motor 3 (Mark10 COM5) ---")
            angle3, tension3 = pretension_and_print(pretensioner_3)
            print("--- Pretensioning Motor 4 (Mark10 COM4) ---")
            angle4, tension4 = pretension_and_print(pretensioner_4)
        except KeyboardInterrupt:
            print("KeyboardInterrupt detected: stopping pretension...")
            pretensioner_3.stop_requested = True
            pretensioner_4.stop_requested = True
            angle3 = tension3 = angle4 = tension4 = None
        finally:
            # Cleanup but DON'T disable motors or close the shared bus
            pretensioner_3.cleanup(close_bus=False, disable_motors=False)
            pretensioner_4.cleanup(close_bus=False, disable_motors=False)
            
            # Give the OS time to fully release the COM ports
            print("Waiting for COM ports to be released...")
            time.sleep(2.0)

        # Generate summary
        if None not in (angle3, tension3, angle4, tension4):
            summary = (
                f"## Pretensioning Results\n"
                f"- Motor 3: angle = {angle3:.4f} rad, tension = {tension3:.4f} N\n"
                f"- Motor 4: angle = {angle4:.4f} rad, tension = {tension4:.4f} N\n"
            )
            pretension_data = {
                'motor3_angle': angle3,
                'motor3_tension': tension3,
                'motor4_angle': angle4,
                'motor4_tension': tension4
            }
            print("✅ Embedded pretensioning completed successfully")
        else:
            summary = "## Pretensioning Results\nPretensioning interrupted or failed.\n"
            pretension_data = None
            print("❌ Embedded pretensioning failed")
            
        return summary, pretension_data
        
    except Exception as e:
        print(f"❌ Embedded pretensioning error: {e}")
        import traceback
        traceback.print_exc()
        return f"## Pretensioning Results\nPretensioning failed: {e}\n", None



