import os
import sys
sys.path.append(os.path.join(".", "cybergear"))

from pcan_cybergear import CANMotorController
import can
import time
import math
import serial
import numpy as np

# Optional imports for plotting
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Warning: PyQtGraph not available. Install pyqtgraph for plotting functionality.")

class OfflinePlotter:
    """Simple offline plotting from CSV data"""
    @staticmethod
    def plot_from_csv(csv_filename):
        """Create and display plot from saved CSV data"""
        if not PLOTTING_AVAILABLE:
            print("PyQtGraph not available")
            return
        
        import csv
        time_data, desired_data, actual_data = [], [], []
        
        with open(csv_filename, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            for row in reader:
                time_data.append(float(row[0]))
                desired_data.append(float(row[1]))
                actual_data.append(float(row[2]))
        
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        
        plot = pg.plot(title="Tension Control Results")
        plot.setLabel('left', 'Tension (N)')
        plot.setLabel('bottom', 'Time (s)')
        plot.plot(time_data, desired_data, pen='r', name='Desired')
        plot.plot(time_data, actual_data, pen='b', name='Actual')
        plot.addLegend()
        plot.show()
        
        app.exec_()
    
    @staticmethod
    def analyze_performance(csv_filename):
        """Analyze control performance from CSV data"""
        import csv
        time_data, desired_data, actual_data, error_data = [], [], [], []
        
        with open(csv_filename, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            for row in reader:
                time_data.append(float(row[0]))
                desired_data.append(float(row[1]))
                actual_data.append(float(row[2]))
                error_data.append(float(row[3]))
        
        # Calculate performance metrics
        import numpy as np
        errors = np.array(error_data)
        actual = np.array(actual_data)
        desired = np.array(desired_data)
        
        rmse = np.sqrt(np.mean(errors**2))
        mae = np.mean(np.abs(errors))
        max_error = np.max(np.abs(errors))
        steady_state_error = np.mean(errors[-int(len(errors)*0.1):])  # Last 10%
        
        print(f"\n=== PERFORMANCE ANALYSIS ===")
        print(f"RMSE (Root Mean Square Error): {rmse:.4f} N")
        print(f"MAE (Mean Absolute Error): {mae:.4f} N")
        print(f"Max Error: {max_error:.4f} N")
        print(f"Steady-state Error: {steady_state_error:.4f} N")
        
        # Check for oscillations
        if len(actual) > 10:
            # Simple oscillation detection
            actual_diff = np.diff(actual)
            sign_changes = np.sum(np.diff(np.sign(actual_diff)) != 0)
            oscillation_index = sign_changes / len(actual_diff)
            print(f"Oscillation Index: {oscillation_index:.3f} (lower is better)")
            
        return {'rmse': rmse, 'mae': mae, 'max_error': max_error, 'steady_state_error': steady_state_error}

class OnlinePlotter:
    """Simple real-time plotting"""
    def __init__(self, max_points=1000):
        self.time_data, self.desired_data, self.actual_data = [], [], []
        self.max_points = max_points
        self.plot = None
        
    def setup(self):
        if not PLOTTING_AVAILABLE:
            return False
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self.plot = pg.plot(title="Real-time Tension Control")
        self.plot.setLabel('left', 'Tension (N)')
        self.plot.setLabel('bottom', 'Time (s)')
        self.desired_curve = self.plot.plot(pen='r', name='Desired')
        self.actual_curve = self.plot.plot(pen='b', name='Actual')
        self.plot.addLegend()
        return True
    
    def update(self, time_val, desired, actual):
        if not self.plot:
            return
        self.time_data.append(time_val)
        self.desired_data.append(desired)
        self.actual_data.append(actual)
        
        if len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.desired_data.pop(0)
            self.actual_data.pop(0)
        
        self.desired_curve.setData(self.time_data, self.desired_data)
        self.actual_curve.setData(self.time_data, self.actual_data)
        QtWidgets.QApplication.processEvents()
    
    def close(self):
        if self.plot:
            self.plot.close()

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
        
        self.previous_error = error
        self.previous_time = current_time
        
        return output

class PositionTensionController:

    def __init__(self, motor_id, mark10_com_port, minimum_tension, pid):
        self.bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
        self.motor = CANMotorController(self.bus, motor_id=motor_id, main_can_id=254)
        self.motor.set_run_mode(self.motor.RunModes.POSITION_MODE)
        self.motor.enable()
        
        self.serial = serial.Serial(mark10_com_port, baudrate=115200, timeout=0.01)

        self.pid = pid
        
        # Position control parameters
        self.home_position = 0.0
        self.current_position = 0.0

        # Safety limits
        self.max_tension = 5.0  # N - maximum allowable tension to prevent cable damage
        self.min_tension = minimum_tension  # N - minimum allowable tension
        # Data logging
        self.data_log = []
        
        # Optional online plotter
        self.online_plotter = None
    
    def read_tension(self):
        try:
            self.serial.write("?\r".encode())
            response = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if response:
                force_str = response.replace('N', '').replace('lbF', '').replace('lb', '').strip()
                return -float(force_str)
        except:
            pass
        return 0.0
    
    def simulate_tension(self):
        """Simulate tension based on motor position (for testing without force sensor)"""
        # Simulate cable-spring system: tension = k * (position - home_position)
        # where k is spring constant and position displacement creates tension
        spring_constant = 2.0  # N/rad - adjust based on your system
        position_displacement = self.home_position - self.current_position  # Reversed to match motor direction
        
        # Only positive displacement creates tension (can't push cable)
        simulated_tension = max(0.0, spring_constant * position_displacement)
        
        # Add small amount of noise to make it realistic
        import random
        noise = random.uniform(-0.01, 0.01)
        
        return simulated_tension + noise
    
    def set_home_position(self):
        """Set current motor position as home position."""
        # Read current motor position and set as home
        try:
            motor_feedback = self.motor.send_motor_control_command(
                torque=0.0, target_angle=0.0, target_velocity=0.0, Kp=0.0, Kd=0.0
            )
            if motor_feedback and len(motor_feedback) > 1:
                current_motor_position = motor_feedback[1]  # position is second element
                self.home_position = current_motor_position
                self.current_position = current_motor_position
                print(f"Home position set to current motor angle: {self.home_position:.4f} rad")
            else:
                # Fallback if can't read position
                self.home_position = 0.0
                self.current_position = 0.0
                print("Warning: Could not read motor position, using 0.0 rad as home")
        except Exception as e:
            print(f"Error reading motor position: {e}")
            self.home_position = 0.0
            self.current_position = 0.0
            print("Using 0.0 rad as home position")
    
    def find_tension_position(self, max_position_change=2.0, step_size=0.05, wait_time=0.2):
        """
        Slowly move actuator counterclockwise until target tension is reached
        
        Args:
            target_tension: Target tension to reach (N)
            max_position_change: Maximum position change from home (rad)
            step_size: Position step size per iteration (rad)
            wait_time: Wait time between steps (s)
        """
        print(f"Finding position for target tension: {self.min_tension:.3f} N")
        
        # Start from home position
        current_pos = self.home_position
        self.motor.set_motor_position_control(limit_spd=5, loc_ref=current_pos)
        time.sleep(1.0)
        
        # Move counterclockwise (negative direction) in small steps
        step_count = 0
        max_steps = int(max_position_change / step_size)
        
        while step_count < max_steps:
            # Move one step counterclockwise
            current_pos -= step_size
            self.current_position = current_pos
            self.motor.set_motor_position_control(limit_spd=5, loc_ref=current_pos)
            time.sleep(wait_time)
            
            # Read current tension
            current_tension = self.read_tension()
            step_count += 1

            print(f"\rStep {step_count}: Position = {current_pos:.4f} rad, Tension = {current_tension:.4f} N", end="")
            sys.stdout.flush()

            # Check if we've reached target tension
            if current_tension >= self.min_tension:
                print(f"Target tension reached! Final position: {current_pos:.4f} rad, Tension: {current_tension:.4f} N")
                return current_pos, current_tension
                
            # Safety check - if tension gets too high, stop
            if current_tension > self.max_tension:
                print(f"Warning: Tension too high ({current_tension:.4f} N), stopping search")
                break
        
        # If we get here, we didn't reach target tension
        final_tension = self.read_tension()
        print(f"Maximum search range reached. Final position: {current_pos:.4f} rad, Tension: {final_tension:.4f} N")
        return current_pos, final_tension

    def run_tension_control(self, tension_func, duration, enable_online_plot=False):
        

        # Setup online plotting if enabled
        if enable_online_plot:
            self.online_plotter = OnlinePlotter()
            if not self.online_plotter.setup():
                self.online_plotter = None  # Disable if setup failed

        # Set home position
        self.set_home_position()
        
        # Find starting position with default tension
        print(f"\n=== FINDING STARTING POSITION ===")
        start_position, start_tension = self.find_tension_position()
        
        # Update home position to the tension position
        self.home_position = start_position
        self.current_position = start_position
        print(f"Updated home position to tension position: {self.home_position:.4f} rad")
        
        # Move to tension position and measure rest tension
        self.motor.set_motor_position_control(limit_spd=5, loc_ref=self.home_position)
        time.sleep(2.0)
        rest_tension_samples = []
        for _ in range(10):
            rest_tension_samples.append(self.read_tension())
            time.sleep(0.05)
        rest_tension = sum(rest_tension_samples) / len(rest_tension_samples)
        print(f"Measured rest tension at starting position: {rest_tension:.4f} N")
        
        # Wait for user input before starting control loop
        input("Press Enter to start the control loop...")
        
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < duration:
            current_time = time.perf_counter()
            t = current_time - start_time
            
            desired_tension = rest_tension + tension_func(t)
            actual_tension = self.read_tension()  # Use real tension sensor
         
            # Update online plot
            if self.online_plotter:
                self.online_plotter.update(t, desired_tension, actual_tension)

            if(actual_tension < self.max_tension):
                error = desired_tension - actual_tension

                # PID output is a position adjustment
                position_adjustment = self.pid.update(error, current_time)
                self.current_position = self.home_position - position_adjustment  # Reversed direction
                
                print(f"\rTime: {t:.3f} s, Desired: {desired_tension:.4f} N, Actual: {actual_tension:.4f} N, Position: {self.current_position:.4f} rad", end='', flush=True)

                self.motor.set_motor_position_control(limit_spd=30, loc_ref=self.current_position)

            else:
                print(f"\rTime: {t:.3f} s, Actual Tension: {actual_tension:.4f} RETURNING TO HOME", end='', flush=True)
                self.current_position = self.home_position
                self.motor.set_motor_position_control(limit_spd=30, loc_ref=self.current_position)

            
            self.data_log.append([t, desired_tension, actual_tension, position_adjustment, self.current_position])
        
        # Close online plot if it was used
        if self.online_plotter:
            self.online_plotter.close()
    
    def save_data(self, filename="position_tension_control_log.csv"):
        import csv
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'desired_tension', 'actual_tension', 'position_adjustment', 'position_command'])
            writer.writerows(self.data_log)
        print(f"Data saved to {filename}")
    
    def go_home(self):
        """Return to home position"""
        print("Returning to home position...")
        self.current_position = self.home_position
        self.motor.set_motor_position_control(limit_spd=5, loc_ref=self.home_position)  # Slower speed
        time.sleep(4)  # Wait longer to ensure motor reaches home
    
    def cleanup(self):
        self.go_home()
        self.motor.disable()
        self.bus.shutdown()
        self.serial.close()

def main():

    # Control parameters for tuning
    max_tension = 4.0  # N - set maximum tension limit
    min_tension = 0.3  # N - set minimum tension limit
    duration = 8.0  # seconds

    # PID tuning parameters (modify these for tuning)
    kp = 1.15    # Proportional gain - increase for faster response, decrease if oscillating
    ki = 0.95   # Integral gain - increase to eliminate steady-state error, decrease if overshooting
    kd = 0.008  # Derivative gain - increase to reduce overshoot, decrease if too noisy
    output_limit = 5.0  # Maximum position adjustment (rad)
    
    # Update PID controller with new parameters
    pid = PIDController(kp=kp, ki=ki, kd=kd)

    controller = PositionTensionController(motor_id=3, mark10_com_port="COM5", minimum_tension=min_tension, pid=pid)

    # Test profiles for different tuning scenarios
    def step_tension(t):
        """Step response - good for tuning overshoot and settling time"""
        return 1.0 if t > 2.0 else 0.0
    
    def sine_tension(t, min_val=1.0, max_val=2.0, freq=0.2):
        """Sine wave - good for testing frequency response and tracking"""
        amplitude = (max_val - min_val) / 2.0
        offset = (max_val + min_val) / 2.0
        return offset + amplitude * math.sin(2 * math.pi * freq * t)
    
    def ramp_tension(t):
        """Slow ramp - good for testing steady-state tracking"""
        return min(3.0, 0.5 * t)
    
    def linear_tension(t):
        """Linear ramp to max - tests full range tracking"""
        return min(max_tension, (max_tension / duration) * t)
    
    def const_tension(t, target=2.0):
        """Constant tension - good for testing steady-state performance"""
        return target
    

    
    print(f"Running tension control with PID gains: Kp={kp}, Ki={ki}, Kd={kd}")
    print(f"Output limit: {output_limit} rad, Max tension: {max_tension} N")
    
    # For real-time plotting: enable_online_plot=True
    controller.run_tension_control(step_tension, duration, enable_online_plot=False)
    
    # Save data to CSV
    csv_filename = "data/position_tension_control_log.csv"
    controller.save_data(csv_filename)
    
    controller.cleanup()

    # Show offline plot from saved CSV data
    print("Displaying offline plot from saved data...")
    OfflinePlotter.plot_from_csv(csv_filename)
    
    # # Analyze performance
    # OfflinePlotter.analyze_performance(csv_filename)
    
    # # Print tuning suggestions based on common patterns
    # print("\n=== PID TUNING GUIDELINES ===")
    # print("Look at the plot and adjust parameters:")
    # print("• If response is too slow: Increase Kp")
    # print("• If oscillating/overshooting: Decrease Kp, increase Kd")
    # print("• If steady-state error: Increase Ki (but watch for overshoot)")
    # print("• If noisy/jittery: Decrease Kd, check sensor noise")
    # print("• If saturating: Increase output_limit or decrease gains")
    # print("\nRecommended tuning order: Kp first, then Kd, finally Ki")

if __name__ == "__main__":
    main()
