"""
Synchronized Data Acquisition System
Single file with all sensor interfaces and coordination logic
"""

import os
import time
import threading
import socket
import json
import csv
import numpy as np
from datetime import datetime
import math
# Import all sensor classes
from simple_ati_ft import ThreadableATI_FT
from simple_mark10_clean import SimpleMark10
from simple_motor_position import DualMotorController
from simple_pretension import run_embedded_pretensioning
from tension_controlled_trajectories import create_tension_trajectories, cleanup_trajectories

class SimpleViconClient:
    def __init__(self, host="192.168.10.2", port=8080):
        """Connect to the Vicon TCP server in constructor"""
        self.host = host
        self.port = port
        self.socket = None
        self.duration = 0
        self.connect()
    
    def connect(self):
        """Connect to the TCP server"""
        try:
            self.socket = socket.socket()
            print(f"Connecting to {self.host}:{self.port}...")
            self.socket.connect((self.host, self.port))
            print(f"Connected to {self.host}:{self.port}")
        except ConnectionRefusedError:
            print(f"Could not connect to {self.host}:{self.port}. Make sure the Vicon TCP server is running.")
            raise
        except Exception as e:
            print(f"Connection error: {e}")
            raise
    
    def send_setup(self, duration=10):
        """Send setup message with specified duration"""
        try:
            self.duration = duration
            print(f"Setting up recording for {duration} seconds...")
            setup_cmd = {"command": "setup", "duration": duration}
            self.socket.sendall(json.dumps(setup_cmd).encode())
            response = self.socket.recv(1024).decode()
            print(f"Server response: {response}")
            return True
        except Exception as e:
            print(f"Setup error: {e}")
            return False
    
    def start_recording(self):
        """Start recording"""
        try:
            start_cmd = {"command": "start"}
            self.socket.sendall(json.dumps(start_cmd).encode())
            response = self.socket.recv(1024).decode()
            print(f"Server response: {response}")
            print(f"Recording started! Duration: {self.duration} seconds")
            return True
        except Exception as e:
            print(f"Start recording error: {e}")
            return False
    
    def get_data(self):
        """Get data from server and save to files"""
        try:
            print("Requesting data...")
            get_data_cmd = {"command": "get_data"}
            self.socket.sendall(json.dumps(get_data_cmd).encode())
            
            # Receive status message
            status_response = self.socket.recv(1024).decode()
            print(f"Server status: {status_response}")
            
            # Receive data size and shape information
            data_size = int.from_bytes(self.socket.recv(4), 'big')
            rows = int.from_bytes(self.socket.recv(4), 'big')
            cols = int.from_bytes(self.socket.recv(4), 'big')
            header_size = int.from_bytes(self.socket.recv(4), 'big')
            
            print(f"Receiving matrix of size {rows}x{cols} ({data_size} bytes)")
            
            # Receive column headers
            column_headers = json.loads(self.socket.recv(header_size).decode())
            
            # Receive data
            data = b''
            while len(data) < data_size:
                chunk = self.socket.recv(min(data_size - len(data), 4096))
                data += chunk
            
            # Convert to numpy matrix
            matrix = np.frombuffer(data, dtype=np.float64).reshape(rows, cols)
            
            print(f"Successfully received Vicon data:")
            print(f"Matrix shape: {matrix.shape}")
            print(f"Columns: {column_headers}")
            print(f"First 5 rows:")
            print(matrix[:5] if len(matrix) > 5 else matrix)
            
            # Save data
            filename = f"vicon_data_{rows}x{cols}.npy"
            np.save(filename, matrix)
            print(f"Data saved to {filename}")
            
            # Also save column headers
            headers_filename = f"vicon_headers_{rows}x{cols}.json"
            with open(headers_filename, 'w') as f:
                json.dump(column_headers, f, indent=2)
            print(f"Headers saved to {headers_filename}")
            
            return matrix, column_headers
            
        except Exception as e:
            print(f"Get data error: {e}")
            return None, None
    
    def close(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            print("Connection closed")

class SynchronizedDataAcquisition:
    """Main coordinator for all sensors and actuators"""
    
    def __init__(self, output_base_dir="data"):
        self.output_base_dir = output_base_dir
        self.experiment_dir = None
        
        # Sensor instances
        self.ati_sensor = None
        self.mark10_sensors = []
        self.vicon_client = None
        self.motor_controller = None
        
        # Trajectory instances (for cleanup)
        self.motor3_trajectory = None
        self.motor4_trajectory = None
        
        # Threading
        self.threads = []
        self.acquisition_started = False
        
        # Vicon settings
        self.vicon_host = "192.168.10.2"
        self.vicon_port = 8080
        
        print("Synchronized Data Acquisition System initialized")
    
    def setup_experiment_folder(self, experiment_name):
        """Create experiment folder with timestamp"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"{experiment_name}_{timestamp}"
        self.experiment_dir = os.path.join(self.output_base_dir, folder_name)
        os.makedirs(self.experiment_dir, exist_ok=True)
        print(f"Experiment folder created: {self.experiment_dir}")
        return self.experiment_dir
    
    def setup_ati_sensor(self, device_channels="Dev1/ai0:5", sampling_rate=1000, duration=10):
        """Setup ATI Force/Torque sensor"""
        try:
            self.ati_sensor = ThreadableATI_FT(
                device_channels=device_channels,
                sampling_rate=sampling_rate,
                name="ATI_FT"
            )
            print("✅ ATI F/T sensor ready")
            return True
        except Exception as e:
            print(f"❌ ATI setup failed: {e}")
            return False
    
    def setup_mark10_sensors(self, com_ports, sampling_rate=350):
        """Setup Mark-10 force gauges"""
        try:
            self.mark10_sensors = []
            for i, port in enumerate(com_ports):
                sensor = SimpleMark10(port, sampling_rate)
                self.mark10_sensors.append(sensor)
                print(f"✅ Mark-10 sensor {i+1} on {port} ready")
            return True
        except Exception as e:
            print(f"❌ Mark-10 setup failed: {e}")
            return False
    
    def setup_motor_controller(self, motor1_id=3, motor2_id=4):
        """Setup dual motor controller"""
        self.motor_controller = DualMotorController(
            motor1_id=motor1_id,
            motor2_id=motor2_id
        )
        self.motor_controller.set_home_positions()
        print("✅ Motor controller ready")
        return True
    
    def setup_vicon_sensor(self, duration, host=None, port=None):
        """Setup Vicon motion capture client"""
        try:
            # Use provided host/port or defaults
            vicon_host = host.split(':')[0] if host and ':' in host else (host or self.vicon_host)
            vicon_port = int(host.split(':')[1]) if host and ':' in host else (port or self.vicon_port)
            
            print(f"🔗 Setting up Vicon client connection to {vicon_host}:{vicon_port}...")
            self.vicon_client = SimpleViconClient(vicon_host, vicon_port)
            print("✅ Vicon client ready")
            return True
        except Exception as e:
            print(f"❌ Vicon setup failed: {e}")
            self.vicon_client = None
            return False
  
    def run_vicon_acquisition(self, duration):
        """Run Vicon data acquisition in a separate thread"""
        def vicon_thread():
            try:
                if self.vicon_client:
                    print("📡 Starting Vicon recording...")
                    self.vicon_client.send_setup(duration)
                    self.vicon_client.start_recording()
                    
                    # Wait for recording to complete
                    time.sleep(duration + 2)  # Extra time for completion
                    
                    # Get data and save it
                    matrix, headers = self.vicon_client.get_data()
                    if matrix is not None:
                        # Save as CSV in the experiment directory
                        csv_filename = os.path.join(self.experiment_dir, "vicon_data.csv")
                        self.save_vicon_as_csv(matrix, headers, csv_filename)
                        
                        # Move the original .npy and .json files to experiment directory if they exist
                        import glob
                        for file_pattern in ["vicon_data_*.npy", "vicon_headers_*.json"]:
                            for file_path in glob.glob(file_pattern):
                                import shutil
                                dest_path = os.path.join(self.experiment_dir, os.path.basename(file_path))
                                shutil.move(file_path, dest_path)
                                print(f"📁 Moved {file_path} to {dest_path}")
                        print("✅ Vicon data acquisition completed")
                    else:
                        print("❌ Failed to get Vicon data")
            except Exception as e:
                print(f"❌ Vicon acquisition error: {e}")
        
        return threading.Thread(target=vicon_thread) 
    
    def save_vicon_as_csv(self, matrix, headers, csv_filename):
        """Save Vicon data as CSV file with proper headers and optimized writing"""
        try:
            if matrix is None or matrix.size == 0:
                print("❌ No Vicon data to save")
                return False
            
            num_samples, num_cols = matrix.shape
            
            # Validate headers match data dimensions
            if len(headers) != num_cols:
                print(f"⚠️ Header count ({len(headers)}) doesn't match data columns ({num_cols})")
                # Create generic headers if mismatch
                headers = [f"col_{i}" for i in range(num_cols)]
                headers[0] = "timestamp"  # First column should be timestamp
            
            # Write CSV with optimized approach
            with open(csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
                
                # Write header row
                writer.writerow(headers)
                
                # Write data rows efficiently (avoid .tolist() call per row)
                writer.writerows(matrix)
            
            # Calculate and display statistics
            print(f"💾 Vicon CSV saved: {csv_filename}")
            print(f"📊 {num_samples:,} samples × {num_cols} columns")
            
            return True
            
        except Exception as e:
            print(f"❌ Error saving Vicon CSV: {e}")
            import traceback
            traceback.print_exc()
            return False

    def create_readme(self, experiment_name, duration, trajectory_func1, trajectory_func2, pretensioning_summary):
        """Create README file with experiment details"""
        readme_path = os.path.join(self.experiment_dir, "README.md")
        
        with open(readme_path, 'w') as f:
            f.write(f"# Experiment: {experiment_name}\n\n")
            f.write(f"**Date/Time:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write(f"**Duration:** {duration} seconds\n\n")
            
            f.write("## Trajectory Functions\n\n")
            f.write(f"**Motor 1:** {trajectory_func1.__doc__ if trajectory_func1.__doc__ else 'No description'}\n\n")
            f.write(f"**Motor 2:** {trajectory_func2.__doc__ if trajectory_func2.__doc__ else 'No description'}\n\n")
            
            f.write("## Active Sensors\n\n")
            if self.ati_sensor:
                f.write("- ATI Force/Torque Sensor\n")
            if self.mark10_sensors:
                f.write(f"- Mark-10 Force Gauges ({len(self.mark10_sensors)} sensors)\n")
            if self.vicon_client:
                f.write("- Vicon Motion Capture\n")
            if self.motor_controller:
                f.write("- Dual Motor Controller\n")
            
            # Add pretensioning results if available
            if pretensioning_summary:
                f.write(f"\n{pretensioning_summary}\n")
            
        print(f"README created: {readme_path}")

    def run_synchronized_acquisition(self, experiment_name, duration, trajectory_func1, trajectory_func2, additional_info=""):
        """Run synchronized data acquisition"""
        
        # Setup experiment folder
        self.setup_experiment_folder(experiment_name)
        
        # Run pretensioning if motors and Mark10 sensors are available
        pretensioning_summary = None
        if self.motor_controller and self.mark10_sensors:
            print("\n🔧 Starting motor pretensioning...")
            pretensioning_summary, _ = run_embedded_pretensioning(self.motor_controller, self.mark10_sensors)
        
        # Create README with pretensioning results
        self.create_readme(experiment_name, duration, trajectory_func1, trajectory_func2, pretensioning_summary)
        
        print(f"\n🎬 All systems ready. Press ENTER to start synchronized acquisition...")
        input()
        
        print(f"🚀 Starting synchronized acquisition in 3 seconds...")
        time.sleep(1)
        print("3...")
        time.sleep(1)
        print("2...")
        time.sleep(1)
        print("1...")
        time.sleep(1)
        print("GO!")
        
        # Start all sensors simultaneously
        start_time = time.time()
        
        # Start Vicon acquisition
        if self.vicon_client:
            vicon_thread = self.run_vicon_acquisition(duration)
            vicon_thread.start()
            self.threads.append(vicon_thread)

        # Start ATI sensor
        if self.ati_sensor:
            ati_thread = threading.Thread(
                target=self.ati_sensor.threaded_acquire,
                args=(duration, os.path.join(self.experiment_dir, "ati_data.csv"))
            )
            ati_thread.start()
            self.threads.append(ati_thread)
        
        # Start Mark-10 sensors
        for i, sensor in enumerate(self.mark10_sensors):
            mark10_thread = threading.Thread(
                target=sensor.acquire_data,
                args=(duration, os.path.join(self.experiment_dir, f"mark10_{i+1}.csv"))
            )
            mark10_thread.start()
            self.threads.append(mark10_thread)

        # Start motor trajectory execution
        if self.motor_controller:
            motor_thread = threading.Thread(
                target=self.motor_controller.execute_synchronized_trajectories,
                args=(trajectory_func1, trajectory_func2, duration, 50.0, 50, True, os.path.join(self.experiment_dir, "motor_data.csv"))
            )
            motor_thread.start()
            self.threads.append(motor_thread)
        
        # Wait for all threads to complete
        print(f"⏳ Acquisition running for {duration} seconds...")
        
        for thread in self.threads:
            thread.join()

        elapsed_time = time.time() - start_time
        print(f"✅ All acquisitions completed in {elapsed_time:.1f} seconds")
        print(f"📁 All data saved to: {self.experiment_dir}")
    
    def run_tension_controlled_acquisition(self, experiment_name, duration, additional_info=""):
        """Run synchronized data acquisition with tension-controlled trajectories"""
        
        # Setup experiment folder
        self.setup_experiment_folder(experiment_name)
        
        # Run pretensioning if motors and Mark10 sensors are available
        pretensioning_summary = None
        pretension_data = None
        if self.motor_controller and self.mark10_sensors:
            print("\n🔧 Starting motor pretensioning...")
            pretensioning_summary, pretension_data = run_embedded_pretensioning(self.motor_controller, self.mark10_sensors)
        
        # Create tension-controlled trajectories
        if pretension_data:
            print("\n🎯 Creating tension-controlled trajectories...")
            self.motor3_trajectory, self.motor4_trajectory = create_tension_trajectories(pretension_data)
            trajectory_func1 = self.motor3_trajectory
            trajectory_func2 = self.motor4_trajectory
            print("📈 Using: Tension-controlled trajectories")
            print("  - Motor 3: Ramp to 6N in 3s, then constant")
            print("  - Motor 4: Constant 3s, then sine wave")
        else:
            print("❌ No pretension data available, cannot create tension trajectories")
            return
        
        # Create README with pretensioning results and trajectory description
        self.create_readme(experiment_name, duration, trajectory_func1, trajectory_func2, pretensioning_summary)
        
        print(f"\n🎬 All systems ready. Press ENTER to start tension-controlled acquisition...")
        input()
        
        print(f"🚀 Starting tension-controlled acquisition in 3 seconds...")
        time.sleep(1)
        print("3...")
        time.sleep(1)
        print("2...")
        time.sleep(1)
        print("1...")
        time.sleep(1)
        print("GO!")
        
        # Start all sensors simultaneously
        start_time = time.time()
        
        # Start Vicon acquisition
        if self.vicon_client:
            vicon_thread = self.run_vicon_acquisition(duration)
            vicon_thread.start()
            self.threads.append(vicon_thread)

        # Start ATI sensor
        if self.ati_sensor:
            ati_thread = threading.Thread(
                target=self.ati_sensor.threaded_acquire,
                args=(duration, os.path.join(self.experiment_dir, "ati_data.csv"))
            )
            ati_thread.start()
            self.threads.append(ati_thread)
        
        # Start Mark-10 sensors
        for i, sensor in enumerate(self.mark10_sensors):
            mark10_thread = threading.Thread(
                target=sensor.acquire_data,
                args=(duration, os.path.join(self.experiment_dir, f"mark10_{i+1}.csv"))
            )
            mark10_thread.start()
            self.threads.append(mark10_thread)

        # Start motor trajectory execution with tension control
        if self.motor_controller:
            motor_thread = threading.Thread(
                target=self.motor_controller.execute_synchronized_trajectories,
                args=(trajectory_func1, trajectory_func2, duration, 50.0, 50, True, os.path.join(self.experiment_dir, "motor_data.csv"))
            )
            motor_thread.start()
            self.threads.append(motor_thread)
        
        # Wait for all threads to complete
        print(f"⏳ Tension-controlled acquisition running for {duration} seconds...")
        
        for thread in self.threads:
            thread.join()

        elapsed_time = time.time() - start_time
        print(f"✅ All tension-controlled acquisitions completed in {elapsed_time:.1f} seconds")
        print(f"📁 All data saved to: {self.experiment_dir}")
    
    def cleanup(self):
        """Cleanup all sensors and actuators"""
        # Cleanup tension trajectories first
        if self.motor3_trajectory or self.motor4_trajectory:
            cleanup_trajectories(self.motor3_trajectory, self.motor4_trajectory)
        
        if self.motor_controller:
            self.motor_controller.cleanup()
        
        if self.ati_sensor:
            self.ati_sensor.stop()
        
        for sensor in self.mark10_sensors:
            sensor.stop()
        
        if self.vicon_client:
            self.vicon_client.close()

def main():
    """Main function"""
    print("="*60)
    print("SYNCHRONIZED DATA ACQUISITION SYSTEM")
    print("="*60)
    
    # Configuration
    EXPERIMENT_NAME = "test_experiment"
    DURATION = 10  # seconds (increased for more motor turns)
    OUTPUT_DIR = "data"
    
    # ATI config
    ATI_CHANNELS = "Dev1/ai0:5"
    ATI_RATE = 1000
    
    # Mark10 config
    MARK10_PORTS = ["COM4", "COM5"]
    MARK10_RATE = 350
    
    # Vicon config
    VICON_HOST = "192.168.10.2"
    VICON_PORT = 8080
    
    # Motor config
    MOTOR1_ID = 3
    MOTOR2_ID = 4
    
    # Trajectory functions
    def fast_sine(t):
        """Fast sine alternative - Group 1"""
        frequency = 2.3
        amplitude = 0.45
        return amplitude * math.sin(2 * math.pi * frequency * t)
    def fast_cosine(t):
        """Fast cosine wave for motor 2 -  amplitude * math.cos(2 * math.pi * frequency * t) -- frequency = 0.8; amplitude = 0.75"""
        frequency = 0.5
        amplitude = 0.75
        return amplitude * math.cos(2 * math.pi * frequency * t)
    def slow_sine(t):
        """Slow sine wave for motor 1 - amplitude * math.sin(2 * math.pi * frequency * t) -- frequency = 0.3; amplitude = 0.7"""
        frequency = 0.3  # 0.3 Hz = 1 full cycle every 3.33 seconds
        amplitude = 0.9  # 
        return amplitude * math.sin(2 * math.pi * frequency * t)
    def slow_cosine(t):
        """Slow cosine wave for motor 2 - Group 2"""
        frequency = 0.3 
        amplitude = 0.3 
        return amplitude * math.cos(2 * math.pi * frequency * t)

    def sine_trajectory_motor1(t):
        """Sine wave trajectory for motor 1"""
        return math.sin(2 * math.pi * 0.5 * t)  # 0.5 Hz
    
    def cosine_trajectory_motor2(t):
        """Cosine wave trajectory for motor 2"""
        return math.cos(2 * math.pi * 0.3 * t)  # 0.3 Hz
    
    # Create acquisition system
    acquisition = SynchronizedDataAcquisition(output_base_dir=OUTPUT_DIR)
    acquisition.vicon_host = VICON_HOST
    acquisition.vicon_port = VICON_PORT
    
    try:
        print("Setting up sensors and actuators...")
        
        # Setup all systems
        ati_ok = acquisition.setup_ati_sensor(ATI_CHANNELS, ATI_RATE, DURATION)
        mark10_ok = acquisition.setup_mark10_sensors(MARK10_PORTS, MARK10_RATE)
        motor_ok = acquisition.setup_motor_controller(MOTOR1_ID, MOTOR2_ID)
        vicon_ok = acquisition.setup_vicon_sensor(DURATION, VICON_HOST, VICON_PORT)
        
        if not any([ati_ok, mark10_ok, vicon_ok, motor_ok]):
            print("❌ No systems ready. Check connections.")
            return
        
        # Show what's working
        working_systems = []
        if ati_ok: working_systems.append("ATI F/T")
        if mark10_ok: working_systems.append("Mark-10")
        if vicon_ok: working_systems.append("Vicon")
        if motor_ok: working_systems.append("Motors")
        
        print(f"✅ Working systems: {', '.join(working_systems)}")
        
        # Warn about failed systems
        failed_systems = []
        if not ati_ok: failed_systems.append("ATI F/T")
        if not mark10_ok: failed_systems.append("Mark-10")
        if not motor_ok: failed_systems.append("Motors")
        
        if failed_systems:
            print(f"⚠️  Failed systems: {', '.join(failed_systems)}")
            proceed = input("Continue with working systems? (y/n): ").lower().strip()
            if proceed != 'y':
                print("Experiment cancelled.")
                return
        
        # OPTION 1: Run synchronized acquisition with tension control
        acquisition.run_tension_controlled_acquisition(
            experiment_name=EXPERIMENT_NAME,
            duration=DURATION,
            additional_info="Test experiment with tension-controlled trajectories",
        )
        
        # # # # OPTION 2: Run synchronized acquisition with classic trajectories (DEFAULT)
        # # # acquisition.run_synchronized_acquisition(
        # # #     experiment_name=EXPERIMENT_NAME,
        # # #     duration=DURATION,
        # # #     trajectory_func1=slow_sine,  # sine_trajectory_motor1,
        # # #     trajectory_func2=fast_cosine,  # cosine_trajectory_motor2,
        # # #     additional_info="Test experiment with sine/cosine trajectories",
        # # # )
        
        print("🎉 Experiment completed successfully!")
        
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        acquisition.cleanup()

if __name__ == "__main__":
    main()
