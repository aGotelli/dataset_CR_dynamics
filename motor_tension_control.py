import os
import sys
sys.path.append(os.path.join(".", "cybergear"))

from pcan_cybergear import CANMotorController
import can
import time
import math
import serial

class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        self.output_limit = output_limit
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
    
    def update(self, error, current_time):
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = current_time - self.previous_time
        
        if dt > 0:
            self.integral += error * dt
            derivative = (error - self.previous_error) / dt
        else:
            derivative = 0.0
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        if self.output_limit:
            output = max(-self.output_limit, min(self.output_limit, output))
        
        self.previous_error = error
        self.previous_time = current_time
        
        return output

class TensionController:
    
    def __init__(self, motor_id=3, mark10_com_port="COM5"):
        self.bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
        self.motor = CANMotorController(self.bus, motor_id=motor_id, main_can_id=254)
        self.motor.set_run_mode(self.motor.RunModes.CURRENT_MODE)
        self.motor.enable()
        
        self.serial = serial.Serial(mark10_com_port, baudrate=115200, timeout=0.01)

        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.05, output_limit=0.1)
        self.data_log = []
    
    def read_tension(self):
        try:
            self.serial.write("?\r".encode())
            response = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if response:
                force_str = response.replace('N', '').replace('lbF', '').replace('lb', '').strip()
                return float(force_str)
        except:
            pass
        return 0.0
    
    def run_tension_control(self, tension_func, duration):
        start_time = time.perf_counter()

        # Measure the tension at rest (zero current) to use as offset
        self.motor.write_single_param("iq_ref", value=0.0)
        time.sleep(1.0)  # Allow time for system to settle
        rest_tension_samples = []
        for _ in range(10):
            rest_tension_samples.append(self.read_tension())
            time.sleep(0.05)
        rest_tension = sum(rest_tension_samples) / len(rest_tension_samples)
        print(f"Measured rest tension: {rest_tension:.4f} N")
        while time.perf_counter() - start_time < duration:
            current_time = time.perf_counter()
            t = current_time - start_time
            
            desired_tension = tension_func(t)
            actual_tension = self.read_tension() - rest_tension

            if(actual_tension < 5.0):
                error = desired_tension - actual_tension


                current_command = rest_tension -self.pid.update(error, current_time)

                print(f"Time: {t:.3f} s, Actual Tension: {actual_tension:.4f} Current Command: {current_command:.4f}")

                self.motor.write_single_param("iq_ref", value=current_command)

            else:
                print(f"Time: {t:.3f} s, Actual Tension: {actual_tension:.4f} SETTING ZERO CURRENT")
                current_command = 0.0
                self.motor.write_single_param("iq_ref", value=current_command)

            
            # self.data_log.append([t, desired_tension, actual_tension, error, current_command])
    
    def save_data(self, filename="tension_control_log.csv"):
        import csv
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'desired_tension', 'actual_tension', 'error', 'current_command'])
            writer.writerows(self.data_log)
        print(f"Data saved to {filename}")
    
    def cleanup(self):
        self.motor.write_single_param("iq_ref", 0.0)
        self.motor.disable()
        self.bus.shutdown()
        self.serial.close()

def main():
    controller = TensionController(motor_id=3, mark10_com_port="COM5")
    
    def step_tension(t):
        return 2.0 if t > 2.0 else 1.0
    
    def sine_tension(t, min_val=1.0, max_val=2.0, freq=0.2):
        amplitude = (max_val - min_val) / 2.0
        offset = (max_val + min_val) / 2.0
        return offset + amplitude * math.sin(2 * math.pi * freq * t)
    
    def ramp_tension(t):
        return min(3.0, 0.5 * t)
    
    def linear_tension(t, max_tension=4, duration=10.0):
        return min(max_tension, (max_tension / duration) * t)
    
    def const_tension(t, max_tension=4):
        return max_tension
    
    duration = 10.0  # seconds

    print("Running tension control...")
    controller.run_tension_control(const_tension, duration)
    controller.save_data("data/tension_control_log.csv")
    controller.cleanup()

if __name__ == "__main__":
    main()
