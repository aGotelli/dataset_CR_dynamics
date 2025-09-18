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
            print(f"[DEBUG] Raw Mark10 response: '{response}'")
            if not response:
                print("[ERROR] Mark10 returned empty response!")
                return 0.0
            match = re.search(r'-?\d+(\.\d+)?', response)
            if match:
                raw_value = float(match.group(0))
                print(f"[DEBUG] Raw value from match: {raw_value}")
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
            print(f"Motor {self.motor_id}: angle = {current_angle:.4f} rad, raw_tension = {tension:.4f} N, |tension| = {abs_tension:.4f} N, target = {abs_target:.4f} N")
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

    def cleanup(self, close_bus=False, disable_motors=False):
        if disable_motors:  # Only disable motors if explicitly requested
            self.motor.disable()
        if close_bus:  # Only close bus if explicitly requested (not for embedded use)
            self.bus.shutdown()
        self.serial.close()


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
                print(f"Pretension: angle={angle:.4f} rad, tension={tension:.4f} N")
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
