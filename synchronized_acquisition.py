"""
Synchronized Data Acquisition System
Single file with all sensor interfaces and coordination logic
"""

import os
import time
import threading
import socket
import json
from datetime import datetime

# Import all sensor classes
from simple_ati_ft import ThreadableATI_FT
from simple_mark10_clean import SimpleMark10
from simple_vicon import ThreadableVicon
from simple_motor_position import DualMotorController

class SynchronizedDataAcquisition:
    """Main coordinator for all sensors and actuators"""
    
    def __init__(self, output_base_dir="data"):
        self.output_base_dir = output_base_dir
        self.experiment_dir = None
        
        # Sensor instances
        self.ati_sensor = None
        self.mark10_sensors = []
        self.vicon_sensor = None
        self.motor_controller = None
        
        # Threading
        self.threads = []
        self.acquisition_started = False
        
        # Vicon TCP settings
        self.vicon_tcp_address = "192.168.10.2"
        self.vicon_tcp_port = 8080
        self.vicon_data_port = 8081  # Port to receive data back from Vicon
        
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
    
    def setup_vicon_sensor(self, host="localhost:801", duration=10, remote_only=True):
        """Setup Vicon motion capture"""
        if remote_only:
            # Test TCP connection to Vicon client
            print("🔗 Testing Vicon TCP connection...")
            test_command = {"command": "status"}
            response = self.send_vicon_command(test_command)
            if response:
                print("✅ Vicon TCP connection OK")
                self.vicon_sensor = True
                return True
            else:
                print("❌ Vicon TCP connection failed")
                return False
        else:
            self.vicon_sensor = ThreadableVicon(
                host=host,
                duration=duration,
                name="Vicon"
            )
            print("✅ Vicon sensor ready")
            return True
    
    def setup_motor_controller(self, motor1_id=3, motor2_id=4):
        """Setup dual motor controller"""
        self.motor_controller = DualMotorController(
            motor1_id=motor1_id,
            motor2_id=motor2_id
        )
        self.motor_controller.set_home_positions()
        print("✅ Motor controller ready")
        return True
    
    def receive_vicon_data(self):
        """Start server to receive Vicon data back from client"""
        def data_receiver():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
                    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    server_socket.bind(("0.0.0.0", self.vicon_data_port))
                    server_socket.listen(1)
                    server_socket.settimeout(30)  # 30 second timeout
                    
                    print(f"📡 Waiting for Vicon data on port {self.vicon_data_port}...")
                    
                    client_socket, address = server_socket.accept()
                    print(f"📥 Receiving Vicon data from {address}")
                    
                    with client_socket:
                        # Receive file info first
                        info_length = int.from_bytes(client_socket.recv(4), 'big')
                        info_data = client_socket.recv(info_length)
                        file_info = json.loads(info_data.decode('utf-8'))
                        
                        print(f"📄 Receiving: {file_info['filename']} ({file_info['size']} bytes)")
                        
                        # Receive file data
                        received_data = b""
                        remaining = file_info['size']
                        
                        while remaining > 0:
                            chunk = client_socket.recv(min(remaining, 8192))
                            if not chunk:
                                break
                            received_data += chunk
                            remaining -= len(chunk)
                        
                        # Save file
                        vicon_file_path = os.path.join(self.experiment_dir, file_info['filename'])
                        with open(vicon_file_path, 'wb') as f:
                            f.write(received_data)
                        
                        print(f"✅ Vicon data saved: {vicon_file_path}")
                        
            except socket.timeout:
                print("⚠️ Timeout waiting for Vicon data")
            except Exception as e:
                print(f"❌ Error receiving Vicon data: {e}")
        
        # Start data receiver in background
        receiver_thread = threading.Thread(target=data_receiver)
        receiver_thread.daemon = True
        receiver_thread.start()
        return receiver_thread

    def send_vicon_command(self, command_data):
        """Send command to Vicon system via TCP"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(5.0)
                sock.connect((self.vicon_tcp_address, self.vicon_tcp_port))
                
                message = json.dumps(command_data).encode('utf-8')
                sock.sendall(message)
                
                response = sock.recv(1024).decode('utf-8')
                return json.loads(response)
        except Exception as e:
            print(f"❌ Vicon TCP communication failed: {e}")
            return None
    
    def create_readme(self, experiment_name, duration, trajectory_func1, trajectory_func2, additional_info=""):
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
            if self.vicon_sensor:
                f.write("- Vicon Motion Capture\n")
            if self.motor_controller:
                f.write("- Dual Motor Controller\n")
            
            if additional_info:
                f.write(f"\n## Additional Information\n\n{additional_info}\n")
        
        print(f"README created: {readme_path}")
    
    def run_synchronized_acquisition(self, experiment_name, duration, trajectory_func1, trajectory_func2, additional_info=""):
        """Run synchronized data acquisition"""
        
        # Setup experiment folder
        self.setup_experiment_folder(experiment_name)
        
        # Create README
        self.create_readme(experiment_name, duration, trajectory_func1, trajectory_func2, additional_info)
        
        # Send setup command to Vicon if available
        if self.vicon_sensor:
            # Start data receiver first
            data_receiver_thread = self.receive_vicon_data()
            
            vicon_command = {
                "command": "setup",
                "experiment_name": experiment_name,
                "duration": duration,
                "vicon_host": "localhost:801"
            }
            response = self.send_vicon_command(vicon_command)
            if response and response.get("status") == "ready":
                print("✅ Vicon system configured remotely")
            else:
                print("⚠️ Vicon remote setup failed, continuing with local only")
        
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
        
        # Start Vicon sensor
        if self.vicon_sensor:
            # Send start command to remote Vicon
            start_command = {"command": "start"}
            response = self.send_vicon_command(start_command)
            print(f"📡 Vicon start response: {response}")
            
            # Don't start local Vicon thread since remote is handling it
        
        # Start motor trajectory execution
        if self.motor_controller:
            motor_thread = threading.Thread(
                target=self.motor_controller.execute_synchronized_trajectories,
                args=(trajectory_func1, trajectory_func2, duration, 50.0, 50, True)
            )
            motor_thread.start()
            self.threads.append(motor_thread)
        
        # Wait for all threads to complete
        print(f"⏳ Acquisition running for {duration} seconds...")
        
        for thread in self.threads:
            thread.join()
        
        elapsed_time = time.time() - start_time
        print(f"✅ All acquisitions completed in {elapsed_time:.1f} seconds")
        
        # Send stop command to Vicon
        if self.vicon_sensor:
            stop_command = {"command": "stop"}
            self.send_vicon_command(stop_command)
        
        print(f"📁 All data saved to: {self.experiment_dir}")
    
    def cleanup(self):
        """Cleanup all sensors and actuators"""
        if self.motor_controller:
            self.motor_controller.cleanup()
        
        if self.ati_sensor:
            self.ati_sensor.stop()
        
        for sensor in self.mark10_sensors:
            sensor.stop()
        
        # Vicon sensor is just a boolean for remote mode, no cleanup needed

def main():
    """Main function"""
    print("="*60)
    print("SYNCHRONIZED DATA ACQUISITION SYSTEM")
    print("="*60)
    
    # Configuration
    EXPERIMENT_NAME = "test_experiment"
    DURATION = 10  # seconds
    OUTPUT_DIR = "data"
    
    # ATI config
    ATI_CHANNELS = "Dev1/ai0:5"
    ATI_RATE = 1000
    
    # Mark10 config
    MARK10_PORTS = ["COM4", "COM5"]
    MARK10_RATE = 350
    
    # Vicon config
    VICON_HOST = "localhost:801"
    VICON_TCP_ADDRESS = "192.168.10.2"
    
    # Motor config
    MOTOR1_ID = 3
    MOTOR2_ID = 4
    
    # Trajectory functions
    def sine_trajectory_motor1(t):
        """Sine wave trajectory for motor 1"""
        import math
        return math.sin(2 * math.pi * 0.5 * t)  # 0.5 Hz
    
    def cosine_trajectory_motor2(t):
        """Cosine wave trajectory for motor 2"""
        import math
        return math.cos(2 * math.pi * 0.3 * t)  # 0.3 Hz
    
    # Create acquisition system
    acquisition = SynchronizedDataAcquisition(output_base_dir=OUTPUT_DIR)
    acquisition.vicon_tcp_address = VICON_TCP_ADDRESS
    
    try:
        print("Setting up sensors and actuators...")
        
        # Setup all systems
        ati_ok = acquisition.setup_ati_sensor(ATI_CHANNELS, ATI_RATE, DURATION)
        mark10_ok = acquisition.setup_mark10_sensors(MARK10_PORTS, MARK10_RATE)
        vicon_ok = acquisition.setup_vicon_sensor(VICON_HOST, DURATION, remote_only=True)  # Use remote only
        # motor_ok = acquisition.setup_motor_controller(MOTOR1_ID, MOTOR2_ID)  # Temporarily disabled
        motor_ok = False  # Disabled for testing
        
        if not any([ati_ok, mark10_ok, vicon_ok, motor_ok]):
            print("❌ No systems ready. Check connections.")
            return
        
        # Show what's working
        working_systems = []
        if ati_ok: working_systems.append("ATI F/T")
        if mark10_ok: working_systems.append("Mark-10")
        if vicon_ok: working_systems.append("Vicon (remote)")
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
        
        # Run synchronized acquisition
        acquisition.run_synchronized_acquisition(
            experiment_name=EXPERIMENT_NAME,
            duration=DURATION,
            trajectory_func1=sine_trajectory_motor1,
            trajectory_func2=cosine_trajectory_motor2,
            additional_info="Test experiment with sine/cosine trajectories"
        )
        
        print("🎉 Experiment completed successfully!")
        
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        acquisition.cleanup()

if __name__ == "__main__":
    main()
