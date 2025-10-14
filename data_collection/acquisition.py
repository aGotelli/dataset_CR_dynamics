"""
Synchronized Data Acquisition System
Single file with all sensor interfaces and coordination logic
"""
import os
import sys
import time
from datetime import datetime
import threading
import math

# Import all sensor classes
from sensors.sensor_ati_ft import ATI_FTSensor
from sensors.vicon_client import ViconClient
from sensors.sparkfun_tcp_client import SparkfunClient
from sensors.motor_controller import MotorControl

class SensorContainer:
    """Main coordinator for all sensors and actuators 
        initialization of sensors
        run acquisition"""
    def __init__(self, experiment_dir, duration, ati_channels, ati_rate, mark10_ports, mark10_rate, vicon_host, vicon_port, sparkfun_host, sparkfun_port, motor1_cfg, motor2_cfg, motors_frequency):
        
        # Basic setup
        self.experiment_dir = experiment_dir
        self.threads = []
        self.duration=duration

        # Store all parameters for future use
        self.ati_channels = ati_channels
        self.ati_rate = ati_rate
        self.mark10_ports = mark10_ports
        self.mark10_rate = mark10_rate
        self.vicon_host = vicon_host
        self.vicon_port = vicon_port
        self.sparkfun_host = sparkfun_host
        self.sparkfun_port = sparkfun_port
        self.motor1_cfg=motor1_cfg
        self.motor2_cfg=motor2_cfg
        self.motors_frequency=motors_frequency
        # self.motor1_id = motor1_id
        # self.motor2_id = motor2_id
        
        # Initialize sensors
        self.ati_sensor = None
        self.vicon_client = None
        self.sparkfun_client = None
        self.motor_control = None
        
        # ATI sensor initialization
        try:
            self.ati_sensor = ATI_FTSensor(
                device_channels=self.ati_channels,
                sampling_rate=self.ati_rate,
                duration=self.duration,
                output_file=os.path.join(self.experiment_dir, "ati_data.csv")
            )
            print("✅ ATI F/T sensor ready")
        except Exception as e:
            self.ati_sensor = None
            print(f"❌ ATI setup failed: {e}")
        
        # # Mark-10 sensors initialization
        

        # Motor controller with integrated Mark10 initialization
        try:
            self.motor_control = MotorControl(
                motor1_id=self.motor1_cfg,
                motor2_id=self.motor2_cfg, 
                mark10_ports=self.mark10_ports,
                mark10_rate=self.mark10_rate,
                motors_frequency=self.motors_frequency,
                experiment_dir=self.experiment_dir,
                target_tension=50.0  # Configure as needed ???????????????????
            )
            print("✅ Motor controller with Mark10 sensors ready")
        except Exception as e:
            self.motor_control = None
            print(f"❌ Motor controller setup failed: {e}")
        
              
        # Vicon client initialization
        # # try:
        # #     self.vicon_client = ViconClient(
        # #         host=vicon_host,
        # #         port=vicon_port,
        # #         duration=duration,
        # #         output_file=os.path.join(self.experiment_dir, "vicon_data.csv")
        # #     )
        # #     print("✅ Vicon client ready")
        # # except Exception as e:
        # #     self.vicon_client = None
        # #     print(f"❌ Vicon setup failed: {e}")

        # Sparkfun IMU client initialization
        try:
            self.sparkfun_client = SparkfunClient()
            # Setup with experiment folder and duration
            self.sparkfun_client.send_setup(self.experiment_dir, duration)
            print("✅ Sparkfun IMU client ready")
        except Exception as e:
            self.sparkfun_client = None
            print(f"❌ Sparkfun IMU setup failed: {e}")
        

    def create_readME(self):
        #temporary
        pass

    def run_acquisition(self, duration):
        """Run synchronized data acquisition"""
        
        self.create_readME()

        # ATI sensor thread
        if self.ati_sensor:
            ati_thread = threading.Thread(
                target=self.ati_sensor.acquire_data,
                name="ATI_Thread"
            )
            self.threads.append(ati_thread)
        
        # # # # Vicon sensor thread
        # # # if self.vicon_client:
        # # #     vicon_thread = threading.Thread(
        # # #         target=self.vicon_client.acquire_data,
        # # #         name="ViconThread"
        # # #     )
        # # #     self.threads.append(vicon_thread)
        
        # Sparkfun IMU sensor thread
        if self.sparkfun_client:
            sparkfun_thread = threading.Thread(
                target=self.sparkfun_client.acquire_data,
                name="SparkfunThread"
            )
            self.threads.append(sparkfun_thread)
        
        # Motor trajectory thread
        if self.motor_control:
            motor_thread = threading.Thread(
                target=self.motor_control.run_trajectory_acquisition,
                args=(duration, sine_trajectory, cosine_trajectory, 50),  # Use trajectory functions from main
                name="MotorThread"
            )
            self.threads.append(motor_thread)
        
        print(f"\n🎬Press ENTER to start acquisition...")
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

        # Start all sensor threads simultaneously
        for thread in self.threads:
            thread.start()

        # Wait for all threads to complete
        for thread in self.threads:
            thread.join()

        # Cleanup sensors
        if self.ati_sensor:
            self.ati_sensor.cleanup()
        if self.sparkfun_client:
            self.sparkfun_client.close()
        if self.vicon_client:
            self.vicon_client.close()
        if self.motor_control:
            self.motor_control.cleanup()

        print("🧹 All sensors cleaned up successfully")

        
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
    DURATION = 5  # seconds 
    OUTPUT_DIR = os.path.abspath("data")  # absolute path
    
    # ATI config
    ATI_CHANNELS = "Dev1/ai0:5"
    ATI_RATE = 1000
    
    # Mark10 config
    MARK10_PORTS = ["COM4", "COM5"]
    MARK10_RATE = 350
    
    # Vicon config
    VICON_HOST = "192.168.10.2"
    VICON_PORT = 8080
    
    # Sparkfun IMU config
    SPARKFUN_HOST = "localhost"
    SPARKFUN_PORT = 9999



    # Define trajectory functions
    def trajMotor1(t):
        """Sine wave trajectory."""
        amplitude = 0.5
        frequency = 0.5
        return amplitude * math.sin(2 * math.pi * frequency * t)
    
    def trajMotor2(t):
        """Cosine wave trajectory (90 degrees out of phase)."""
        amplitude = 1.0
        frequency = 0.5
        return amplitude * math.cos(2 * math.pi * frequency * t)
    
    # Create dual motor controller
    MOTOR1_CFG = [3, "COM5", trajMotor1]  # [motor_id, com_port, trajectory_func]
    MOTOR2_CFG = [4, "COM4", trajMotor2]  # [motor_id, com_port, trajectory_func]

    MOTOR_FREQUENCY = 50 # Hz
    

    
    EXPERIMENT_DIR = setup_experiment_folder(EXPERIMENT_NAME, OUTPUT_DIR)
    

    # Initialize acquisition system with all parameters
    sensor_container = SensorContainer(
        experiment_dir=EXPERIMENT_DIR,
        duration=DURATION,
        ati_channels=ATI_CHANNELS,
        ati_rate=ATI_RATE,
        mark10_ports=MARK10_PORTS,
        mark10_rate=MARK10_RATE,
        vicon_host=VICON_HOST,
        vicon_port=VICON_PORT,
        sparkfun_host=SPARKFUN_HOST,
        sparkfun_port=SPARKFUN_PORT,
        motor1_cfg=MOTOR1_CFG,
        motor2_cfg=MOTOR2_CFG,
        motors_frequency=MOTOR_FREQUENCY
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