"""
Synchronized Data Acquisition System
Single file with all sensor interfaces and coordination logic
"""
import os
import sys
import time
from datetime import datetime
import threading

# Import all sensor classes
from sensor_ati_ft import ATI_FTSensor
from vicon_client import ViconClient
from sparkfun_ism330dhcx_interface.python_minimal.test_optimized import GYROSensor

class SensorContainer:
    """Main coordinator for all sensors and actuators 
        initialization of sensors
        run acquisition"""
    def __init__(self, experiment_dir, ati_channels, ati_rate, mark10_ports, mark10_rate, vicon_host, vicon_port, motor1_id, motor2_id):
        
        # Basic setup
        self.experiment_dir = experiment_dir
        self.threads = []
        
        # Store all parameters for future use
        self.ati_channels = ati_channels
        self.ati_rate = ati_rate
        self.mark10_ports = mark10_ports
        self.mark10_rate = mark10_rate
        self.vicon_host = vicon_host
        self.vicon_port = vicon_port
        self.motor1_id = motor1_id
        self.motor2_id = motor2_id
        
        # Initialize sensors
        self.ati_sensor = None
        self.mark10_sensors = []
        self.vicon_client = None
        self.motor_controller = None
        self.imu_sensor = None
        
        # ATI sensor initialization
        try:
            self.ati_sensor = ATI_FTSensor(
                device_channels=ati_channels,
                sampling_rate=ati_rate,
            )
            print("✅ ATI F/T sensor ready")
        except Exception as e:
            self.ati_sensor = None
            print(f"❌ ATI setup failed: {e}")
        
        # Mark-10 sensors initialization
        
        
        # Motor controller initialization (PLACEHOLDER)
              

        # Vicon client initialization
        try:
            self.vicon_client = ViconClient(vicon_host, vicon_port)
            print("✅ Vicon client ready")
        except Exception as e:
            print(f"❌ Vicon setup failed: {e}")
        
        # IMU sensor initialization  
        try:
            self.imu_sensor = GYROSensor()
            print("✅ IMU sensor ready")
        except Exception as e:
            self.imu_sensor = None
            print(f"❌ IMU setup failed: {e}")
        

    def create_readME(self):
        #temporary
        pass


    def vicon_thread(self,duration):
        try:
            if self.vicon_client:
                print("Vicon - Starting recording...")
                self.vicon_client.send_setup(duration)
                self.vicon_client.start_recording()                    
                # Wait for recording to complete
                time.sleep(duration + 2)  # Extra time for completion
                   
                # Get data and save it
                csv_filename = os.path.join(self.experiment_dir, "vicon_data.csv")
                self.vicon_client.get_data(csv_filename)
                matrix, headers = self.vicon_client.get_data(csv_filename)
                if matrix is not None: #---- !! no longer needed npy and json formats
                    # No longer moving .npy and .json files since they're not created
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

    def run_acquisition(self, duration):
        """Run synchronized data acquisition"""
        
        self.create_readME()
        #prepare vicon to receive data

        
        print(f"\n🎬 ATI sensor ready. Press ENTER to start acquisition...")
        input()
        
        print(f"🚀 Starting acquisition in 3 seconds...")
        time.sleep(1)
        print("3...")
        time.sleep(1)
        print("2...")
        time.sleep(1)
        print("1...")
        time.sleep(1)
        print("GO!")
                
        # ATI sensor thread
        if self.ati_sensor:
            ati_thread = threading.Thread(
                target=self.ati_sensor.acquire_data,
                args=(duration, os.path.join(self.experiment_dir, "ati_data.csv"))
            )
            ati_thread.start()
            self.threads.append(ati_thread)
        
        # gyro sensor thread
        if self.imu_sensor:
            gyro_thread = threading.Thread(
                target=self.imu_sensor.acquire_data,
                args=(duration, os.path.join(self.experiment_dir, "imu_data.csv"))
            )
            gyro_thread.start()
            self.threads.append(gyro_thread)
        # Vicon thread
        if self.vicon_client:
            vicon_thread = threading.Thread(
                target=self.vicon_thread,
                args=(duration)
            )
            vicon_thread.start()
            self.threads.append(vicon_thread)
            
        for thread in self.threads:
            thread.join()
        
        # Cleanup sensors
        if self.imu_sensor:
            self.imu_sensor.cleanup()

        
def setup_experiment_folder(experiment_name, output_base_dir):
        """Create experiment folder with timestamp"""
        experiment_dir=None
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"{experiment_name}_{timestamp}"
        experiment_dir = os.path.join(output_base_dir, folder_name)
        os.makedirs(experiment_dir, exist_ok=True)
        print(f"Experiment folder created: {experiment_dir}")
        return experiment_dir

def main():
    """Main"""
    print("="*60)
    print("MULTI-SENSOR DATA ACQUISITION SYSTEM")
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
    VICON_HOST = "192.168.10.2"
    VICON_PORT = 8080
    
    # Motor config
    MOTOR1_ID = 3
    MOTOR2_ID = 4
    
    EXPERIMENT_DIR = setup_experiment_folder(EXPERIMENT_NAME, OUTPUT_DIR)

    # Initialize acquisition system with all parameters
    sensor_container = SensorContainer(
        experiment_dir=EXPERIMENT_DIR,
        ati_channels=ATI_CHANNELS,
        ati_rate=ATI_RATE,
        mark10_ports=MARK10_PORTS,
        mark10_rate=MARK10_RATE,
        vicon_host=VICON_HOST,
        vicon_port=VICON_PORT,
        motor1_id=MOTOR1_ID,
        motor2_id=MOTOR2_ID
    )

    # Call the real acquisition
    sensor_container.run_acquisition(DURATION)

        # # # # OPTION 1: Run synchronized acquisition with tension control
        # # # acquisition.run_tension_controlled_acquisition(
        # # #     experiment_name=EXPERIMENT_NAME,
        # # #     duration=DURATION,
        # # #     additional_info="Test experiment with tension-controlled trajectories",
        # # # )
        
        # # # # OPTION 2: Run synchronized acquisition with classic trajectories (DEFAULT)
        # # # acquisition.run_synchronized_acquisition(
        # # #     experiment_name=EXPERIMENT_NAME,
        # # #     duration=DURATION,
        # # #     trajectory_func1=slow_sine,  # sine_trajectory_motor1,
        # # #     trajectory_func2=fast_cosine,  # cosine_trajectory_motor2,
        # # #     additional_info="Test experiment with sine/cosine trajectories",
        # # # )
        
    print("🎉 Experiment completed successfully!")

if __name__ == "__main__":
    main()