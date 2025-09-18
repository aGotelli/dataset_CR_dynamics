"""
Synchronized Data Acquisition System
Single file with all sensor interfaces and coordination logic
"""
import os
import time
from datetime import datetime
import threading
# Import all sensor classes
from simple_ati_ft import ThreadableATI_FT

class SynchronizedDataAcquisition:
    """Main coordinator for all sensors and actuators"""
    def __init__(self, output_base_dir="data"):
        # temporary initialized variables
        self.output_base_dir = output_base_dir
        self.experiment_dir = None
        self.threads = []  # Initialize threads list
        
        # Sensors will be retrieved from their classes after setup
        self.ati_sensor = None
        self.mark10_sensors = []
        self.vicon_sensor = None
        self.motor_controller = None
        
        self.vicon_host = "192.168.10.2" # CONTROLLA
        self.vicon_port = 8080 # CONTROLLA
        print("Synchronized Data Acquisition System initialized")

    def setup_experiment_folder(self, experiment_name):
        """Create experiment folder with timestamp"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"{experiment_name}_{timestamp}"
        self.experiment_dir = os.path.join(self.output_base_dir, folder_name)
        os.makedirs(self.experiment_dir, exist_ok=True)
        print(f"Experiment folder created: {self.experiment_dir}")
        return self.experiment_dir

    def create_readME(self):
        #temporary
        pass

    def run_acquisition(self, experiment_name, duration):
        """Run synchronized data acquisition"""
         # Setup experiment folder
        self.setup_experiment_folder(experiment_name)
                
        self.create_readME()
        
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
        
        for thread in self.threads:
            thread.join()

        elapsed_time = time.time() - start_time
        print(f"✅ All acquisitions completed in {elapsed_time:.1f} seconds")
        print(f"📁 All data saved to: {self.experiment_dir}")


    def cleanup(self):
        #temporary
        pass

def sensorsInit(acquisition, ati_channels, ati_rate, mark10_ports, mark10_rate, 
                vicon_host, vicon_port, motor1_id, motor2_id, duration):
    """Initialize all sensors"""
    
    # Setup ATI sensor
    try:
        acquisition.ati_sensor = ThreadableATI_FT(
            device_channels=ati_channels,
            sampling_rate=ati_rate,
        )
        print("✅ ATI F/T sensor ready")
    except Exception as e:
        acquisition.ati_sensor = None
        print(f"❌ ATI setup failed: {e}")
    
    # # # # Setup Mark-10 sensors
    # # # try:
    # # #     acquisition.setup_mark10_sensors(mark10_ports, mark10_rate)
    # # # except Exception as e:
    # # #     print(f"❌ Mark-10 setup failed: {e}")
    
    # # # # Setup Motor controller
    # # # try:
    # # #     acquisition.setup_motor_controller(motor1_id, motor2_id)
    # # # except Exception as e:
    # # #     print(f"❌ Motor setup failed: {e}")
    
    # # # # Setup Vicon sensor
    # # # try:
    # # #     acquisition.setup_vicon_sensor(duration, vicon_host, vicon_port)
    # # # except Exception as e:
    # # #     print(f"❌ Vicon setup failed: {e}")


def main():
    """Main"""
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

    # Create acquisition system
    acquisition = SynchronizedDataAcquisition(output_base_dir=OUTPUT_DIR)

    print("Setting up sensors and actuators...")
        
    # Setup all systems using the centralized function
    sensorsInit(
        acquisition=acquisition,
        ati_channels=ATI_CHANNELS,
        ati_rate=ATI_RATE,
        mark10_ports=MARK10_PORTS,
        mark10_rate=MARK10_RATE,
        vicon_host=VICON_HOST,
        vicon_port=VICON_PORT,
        motor1_id=MOTOR1_ID,
        motor2_id=MOTOR2_ID,
        duration=DURATION
    )

    # Call the real acquisition
    acquisition.run_acquisition(EXPERIMENT_NAME, DURATION)

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