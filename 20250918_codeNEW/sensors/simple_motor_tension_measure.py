import os
import sys
import can
import time
import math
from datetime import datetime
import threading
import numpy as np
import csv

# Import from local cybergear package
from cybergear import CANMotorController
from simple_mark10 import SimpleMark10



class MotorWithMark10:
    def __init__(self, motor_mk10, bus, main_can_id, duration, frequency):
        self.motor_id = motor_mk10[0]
        com_port = motor_mk10[1]

        self.function = motor_mk10[2]


        self.duration = duration

        extra_buffer = 1.3
        rows = int(extra_buffer*duration*frequency)
        cols = 4
        self.data = np.zeros((rows, cols))

        self.it_t = 0

        self.speed_limit = 50
        self.tension_threshold = 5
        
        print(f"Initializing motor ID: {self.motor_id}, COM: {com_port}, CAN ID: {main_can_id}")

        self.mark10 = SimpleMark10(com_port, sampling_rate=300, timeout=0.1)

        # Setup motor
        self.motor = CANMotorController(bus, motor_id=self.motor_id, main_can_id=main_can_id)

        self.motor.set_run_mode(self.motor.RunModes.POSITION_MODE)
        self.motor.enable()

        #   Otherwise it throws an error...
        self.motor.set_motor_position_control(limit_spd=0, loc_ref=0)
        time.sleep(0.5)
            
        
        self.home_position = 0
        feedback = self.motor.send_motor_control_command(
            torque=0.0, target_angle=0.0, target_velocity=0.0, Kp=0.0, Kd=0.0
        )


            
        if feedback and len(feedback) > 1:
            self.home_position = feedback[1]
            input("Press Enter to continue...")

            #   Make sure motor is stable at home position
            for i in range(10):

                feedback = self.motor.send_motor_control_command(
                    torque=0.0, target_angle=0.0, target_velocity=0.0, Kp=0.0, Kd=0.0
                )
                current_angle = feedback[1]

                self.motor.set_motor_position_control(limit_spd=5, loc_ref=self.home_position)

                print(f"Current angle: {current_angle}, Home position: {self.home_position}")
                input("Press Enter to continue...")
                time.sleep(0.1)


            print(f"✅Motor {self.motor_id} initialized and enabled")
        else:
            print("⚠️⚠️⚠️\tWarning: Could not read motor position")

        


    def run(self):

        it_t = 0
        start_time = time.perf_counter()
        time_elapsed = 0
        while time_elapsed <= self.duration:
            current_time = time.perf_counter()
            cable_tension = self.mark10.get_tension() 

            if cable_tension >= self.tension_threshold:
                desired_angle = self.home_position
                print(f"❌❌❌ MOTOR {self.motor_id} STOPPED FOR SAFETY LIMITS")
                break

            desired_angle = self.home_position + self.function(current_time)
            self.motor.set_motor_position_control(limit_spd=self.speed_limit, loc_ref=desired_angle)
            motor_feedback = self.motor.send_motor_control_command(
                torque=0.0, target_angle=0.0, target_velocity=0.0, Kp=0.0, Kd=0.0
            )

            time_elapsed = current_time - start_time

            current_angle = 0
            if motor_feedback and len(motor_feedback) > 1:
                current_angle = motor_feedback[1]  # position is second element

            self.data[it_t] = [current_time, cable_tension, current_angle, desired_angle]
            it_t = it_t + 1

            
    def save_data(self, filename):
        """
        Save data to CSV file
        
        Args:
            filename: Output filename
        """
        # Check if data is empty (works for both numpy arrays and lists)
        if self.data is None or len(self.data) == 0:
            print("No data to save")
            return False
        
        filename = os.path.join(filename, "motor" + self.motor_id + ".csv")
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            header = ['timestamp', 'cable_tension', 'current_angle', 'desired_angle']
            writer.writerow(header)
                
            # Write data
            writer.writerows(self.data)

        print(f"📁 Motor {self.motor_id} data saved to: {filename}")
    
    

    def cleanup(self):
        """Disable motor"""
        try:
            self.motor.disable()
            print(f"Motor {self.motor_id} disabled")
        except Exception as e:
            print(f"Warning: Error disabling motor : {self.motor_id}")
    
    def __del__(self):
        """Destructor - automatically cleanup when object is destroyed"""
        self.cleanup()
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensure cleanup is called"""
        self.cleanup()


class MotorController:
    """Simple controller for managing two motors without threading."""
    
    def __init__(self, motor_mk10_1, motor_mk10_2, duration, frequency, experiment_dir):
        """Initialize dual motor controller.
        
        Args:
            motor1_id: ID of first motor
            motor2_id: ID of second motor
            main_can_id: Main CAN ID
            interface: CAN interface type
            channel: CAN channel
            bitrate: CAN bitrate
        """
        main_can_id = 254


        self.filename = experiment_dir
        
        # Create shared CAN bus
        self.bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
        print("CAN bus created")
        
        # Initialize motors
        self.motor1 = MotorWithMark10(motor_mk10_1, self.bus, main_can_id, duration, frequency)
        self.motor2 = MotorWithMark10(motor_mk10_2, self.bus, main_can_id, duration, frequency)


        self.motor1_thread = threading.Thread(
                target=self.motor1.run
        )

        self.motor2_thread = threading.Thread(
                target=self.motor2.run
        )


    def start_acquisition(self):
        self.motor1_thread.start()
        self.motor2_thread.start()


    def save_data(self):

        self.motor1.save_data(self.filename)
        self.motor2.save_data(self.filename)


    def cleanup(self):
        """Disable motors and shutdown CAN bus."""
        try:
            self.bus.shutdown()
            print("CAN bus shut down")
        except Exception as e:
            print(f"Warning: Error shutting down CAN bus: {e}")
    
    def __del__(self):
        """Destructor - automatically cleanup when object is destroyed"""
        self.cleanup()
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensure cleanup is called"""
        self.cleanup()
        

def main():

    # Define trajectory functions
    def sine_trajectory(t):
        """Sine wave trajectory."""
        amplitude = 0.5
        frequency = 0.5
        return amplitude * math.sin(2 * math.pi * frequency * t)
    
    def msine_trajectory(t):
        """Sine wave trajectory."""
        amplitude = 0.5
        frequency = 0.5
        return -0*amplitude * math.sin(2 * math.pi * frequency * t)
    
    def cosine_trajectory(t):
        """Cosine wave trajectory (90 degrees out of phase)."""
        amplitude = 1.0
        frequency = 0.5
        return amplitude * math.cos(2 * math.pi * frequency * t)

    # Create dual motor controller
    motor_mk10_1 = [3, "COM5", sine_trajectory]  # Lista - accesso per indice
    motor_mk10_2 = [4, "COM4", msine_trajectory]  # Lista - accesso per indice
    controller = MotorController(motor_mk10_1, motor_mk10_2, 10, 50, "")

    controller.motor2.run()
    
if __name__ == "__main__":
    main()
