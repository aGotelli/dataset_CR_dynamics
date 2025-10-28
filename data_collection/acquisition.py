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
    def __init__(self, experiment_dir, duration, ati_channels, ati_rate, mark10_ports, mark10_rate, vicon_host, vicon_port, sparkfun_host, sparkfun_port, motor1_cfg, motor2_cfg, motors_frequency, target_tension=None):
        
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
        self.target_tension = target_tension
        # self.motor1_id = motor1_id
        # self.motor2_id = motor2_id
        
        # Initialize sensors
        self.ati_sensor = None
        self.vicon_client = None
        self.sparkfun_client = None
        self.motor_control = None
        
        # # # ATI sensor initialization
        # # try:
        # #     self.ati_sensor = ATI_FTSensor(
        # #         device_channels=self.ati_channels,
        # #         sampling_rate=self.ati_rate,
        # #         duration=self.duration,
        # #         output_file=os.path.join(self.experiment_dir, "ati_data.csv")
        # #     )
        # #     print("✅ ATI F/T sensor ready")
        # # except Exception as e:
        # #     self.ati_sensor = None
        # #     print(f"❌ ATI setup failed: {e}")
        
        # Motor+Mark10 controller with integrated Mark10 initialization
        try:
            self.motor_control = MotorControl(
                motor1_cfg=self.motor1_cfg,
                motor2_cfg=self.motor2_cfg, 
                mark10_ports=self.mark10_ports,
                mark10_rate=self.mark10_rate,
                motors_frequency=self.motors_frequency,
                experiment_dir=self.experiment_dir,
                target_tension=self.target_tension
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

        # # # Sparkfun IMU client initialization
        # # try:
        # #     self.sparkfun_client = SparkfunClient()
        # #     # Setup with experiment folder and duration
        # #     self.sparkfun_client.send_setup(self.experiment_dir, duration)
        # #     print("✅ Sparkfun IMU client ready")
        # # except Exception as e:
        # #     self.sparkfun_client = None
        # #     print(f"❌ Sparkfun IMU setup failed: {e}")
        

    def create_readME(self):
        #temporary
        pass

    def run_acquisition(self, duration):
        """Run synchronized data acquisition"""
        
        self.create_readME()

        # Run motor pretension before starting acquisition so tensions are ready
        if self.motor_control:
            print("ℹ️  Running motor pretension before acquisition...")
            choice = input("Press 1 for running pretensioning, 2 to skip: ")
            # default to False so `flag` is always defined
            flag = False
            if choice.strip() == "1":
                flag = True
            elif choice.strip() == "2":
                flag = False
            successm1, successm2 = self.motor_control.pretension(flag)
            if successm1 and successm2:
                self.motor_control._build_relative_trajectory()
                print("✅ Pretension completed")
            else:
                print(f"❌ Pretension failed; Motor1: {successm1}, Motor2: {successm2}; aborting acquisition.")
            
        
        # ATI sensor thread
        # if self.ati_sensor:
        #     ati_thread = threading.Thread(
        #         target=self.ati_sensor.acquire_data,
        #         name="ATI_Thread"
        #     )
        #     self.threads.append(ati_thread)
        
        # Vicon sensor thread
        # if self.vicon_client:
        #     vicon_thread = threading.Thread(
        #         target=self.vicon_client.acquire_data,
        #         name="ViconThread"
        #     )
        #     self.threads.append(vicon_thread)
        
        # # Sparkfun IMU sensor thread
        # if self.sparkfun_client:
        #     sparkfun_thread = threading.Thread(
        #         target=self.sparkfun_client.acquire_data,
        #         name="SparkfunThread"
        #     )
        #     self.threads.append(sparkfun_thread)
       
       
       

        # Motor trajectory thread
        if self.motor_control:
            motor_thread = threading.Thread(
                target=self.motor_control.run_trajectory_acquisition,
                args=(duration,),
                name="MotorThread"
            )
            self.threads.append(motor_thread)
        #  #reading thread
        # if self.motor_control:
        #     motor_thread_read = threading.Thread(
        #         target=self.motor_control.readVal,
        #         args=(duration,),
        #         name="Readhread"
        #     )
        #     self.threads.append(motor_thread_read)
        

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
    DURATION = 2  # seconds 
    OUTPUT_DIR = os.path.abspath("data")  # absolute path
    
    # ATI config
    ATI_CHANNELS = "Dev1/ai0:5"
    ATI_RATE = 1000
    
    # Mark10 config
    MARK10_PORTS = ["COM4", "COM5"]
    MARK10_RATE = 350
    TARGET_TENSION = 5.0  # N
    
    # Vicon config
    VICON_HOST = "192.168.10.2"
    VICON_PORT = 8080
    
    # Sparkfun IMU config
    SPARKFUN_HOST = "localhost"
    SPARKFUN_PORT = 9999


    
    # Motor trajectories are described via dict; supported options:
    #   - type: "ramp", "sine", "circle"
    #   - axis: "x" / "y" (only for ramp/circle to select which motor moves)
    #   - duration: seconds (used by ramp)
    #   - max_deg: target angle in degrees (ramp)
    #   - target_deg: alternative key for ramp absolute target
    #   - amplitude_deg: amplitude of sine/cosine (degrees)
    #   - frequency_hz: frequency for oscillatory/circular motion
    #   - radius_deg: circle radius in degrees (circle)
    # MotorControl builds absolute trajectories after pretension from these params.
    MOTOR1_CFG = {
        "id": 3,
        "mark10_port": "COM5",
        "trajectory": {
            "type": "ramp",
            "axis": "y",
            "max_deg":1,# "type": "sine",
            # "amplitude_deg": 60.0,
            # "frequency_hz": 0.2,
        },
    }

    MOTOR2_CFG = {
        "id": 4,
        "mark10_port": "COM4",
        "trajectory": {
            "type": "ramp",
            "axis": "y",
            "max_deg":-100,
            # "type": "sine",
            # "amplitude_deg": 20.0,
            # "frequency_hz": 0.2,
            # "start_delay": 0.5, #motor 2 needs to start after motor 1 
        },
    }

    MOTOR_FREQUENCY = 60 # Hz
        
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
        motors_frequency=MOTOR_FREQUENCY,
        target_tension=TARGET_TENSION
    )

    # Call the real acquisition
    try:
        sensor_container.run_acquisition(DURATION)
        print("🎉 Experiment completed successfully!")
    except Exception as e:
        # Cleanup sensors
        if sensor_container.ati_sensor:
            sensor_container.ati_sensor.cleanup()
        if sensor_container.sparkfun_client:
            sensor_container.sparkfun_client.close()
        if sensor_container.vicon_client:
            sensor_container.vicon_client.close()
        if sensor_container.motor_control:
            sensor_container.motor_control.cleanup()
        print(f"❌ Acquisition failed: {e}")

if __name__ == "__main__":
    main()
