"""
Example of how to integrate the CentralizedSensorManager
"""

from sensor_manager import CentralizedSensorManager, Mark10SensorAdapter, Mark10DataLogger
import time

def example_usage():
    # Create the centralized manager
    sensor_manager = CentralizedSensorManager()
    
    # Add your Mark-10 sensors
    sensor_manager.add_mark10_sensor("COM4", sampling_rate=350)
    sensor_manager.add_mark10_sensor("COM5", sampling_rate=350)
    
    # Start sensor reading threads
    sensor_manager.start_sensors()
    
    try:
        # For pretensioning: use adapters
        mark10_com4 = Mark10SensorAdapter("COM4", sensor_manager)
        mark10_com5 = Mark10SensorAdapter("COM5", sensor_manager)
        
        # Read tension values (non-blocking, always available)
        tension_4 = mark10_com4.read_tension()
        tension_5 = mark10_com5.read_tension()
        print(f"Current tensions: COM4={tension_4:.3f}N, COM5={tension_5:.3f}N")
        
        # For data logging: start loggers
        logger_4 = Mark10DataLogger("COM4", sensor_manager)
        logger_5 = Mark10DataLogger("COM5", sensor_manager)
        
        # Start 10-second data acquisition
        print("Starting data acquisition...")
        logger_4.acquire_data(10, "mark10_1.csv")
        logger_5.acquire_data(10, "mark10_2.csv")
        
        # During acquisition, you can still read real-time values
        for i in range(10):
            tension_4 = mark10_com4.read_tension()
            tension_5 = mark10_com5.read_tension()
            print(f"Live: COM4={tension_4:.3f}N, COM5={tension_5:.3f}N")
            time.sleep(1)
    
    finally:
        # Clean shutdown
        sensor_manager.stop_sensors()
        print("Sensor manager stopped")

if __name__ == "__main__":
    example_usage()
