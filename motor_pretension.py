import os
import sys
import threading
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
                value = float(match.group(0))
                return value
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
            abs_tension = abs(tension)
            abs_target = abs(self.target_tension)
            print(f"Motor {self.motor_id}: angle = {current_angle:.4f} rad, |tension| = {abs_tension:.4f} N, target = {abs_target:.4f} N")
            if abs(abs_tension - abs_target) <= tolerance:
                self.angle = current_angle
                return current_angle, tension
            elif abs_tension < abs_target:
                current_angle += self.direction * step_size
                self.motor.set_motor_position_control(limit_spd=self.speed, loc_ref=current_angle)
            else:
                current_angle -= self.direction * step_size
                self.motor.set_motor_position_control(limit_spd=self.speed, loc_ref=current_angle)
            tension = self.read_tension()
            time.sleep(wait_time)
        self.angle = current_angle
        # Se non si è mai stabilizzato, non restituisce nulla
        return None, None

    def export_string(self):
        return f"Pretension: angle={self.angle:.4f} rad, tension={self.read_tension():.4f} N"

    def cleanup(self):
        self.motor.disable()
        self.bus.shutdown()
        self.serial.close()
           

def main():
    # Shared CAN bus for both motors
    bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
    # Associa COM5 a motore ID3, COM4 a motore ID4
    pretensioner_3 = MotorPretensioner(motor_id=3, bus=bus, mark10_com_port="COM5", target_tension=2, direction=1, speed=5)
    pretensioner_4 = MotorPretensioner(motor_id=4, bus=bus, mark10_com_port="COM4", target_tension=2, direction=-1, speed=5)

    def pretension_and_print(pretensioner):
        try:
            angle, tension = pretensioner.pretension()
            print(pretensioner.export_string())
            return angle, tension
        except Exception as e:
            print(f"Motor {pretensioner.motor_id}: Exception in pretension: {e}")
            return None, None

    try:
        print("--- Pretensioning Motor 3 (Mark10 COM5) ---")
        pretension_and_print(pretensioner_3)
        print("--- Pretensioning Motor 4 (Mark10 COM4) ---")
        pretension_and_print(pretensioner_4)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected: stopping pretension and cleaning up...")
        pretensioner_3.stop_requested = True
        pretensioner_4.stop_requested = True
    finally:
        pretensioner_3.cleanup()
        pretensioner_4.cleanup()

if __name__ == "__main__":
    main()
