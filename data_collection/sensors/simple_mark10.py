import serial
import time
import numpy as np
import csv
import pandas as pd
import sys
import os

class SimpleMark10:
    """
    Simple Mark-10 force gauge data acquisition
    Clean interface similar to ATI sensor for easy integration
    """
    
    def __init__(self, com_port, sampling_rate=350, timeout=0.1):
        """
        Initialize the Mark-10 sensor
        
        Args:
            com_port: Serial port (e.g., "COM5")
            sampling_rate: Target sampling frequency in Hz
            timeout: Serial read timeout in seconds
        """
        self.com_port = com_port
        self.sampling_rate = sampling_rate
        self.timeout = timeout
        self.serial_connection = None
        
        # Establish serial connection in constructor
        try:
            self.serial_connection = serial.Serial(
                self.com_port, 
                baudrate=115200, 
                timeout=self.timeout
            )

            timing = self.measure_timing(50)
            print(f"Mark-10 initialized with {timing['data_rate']:.1f} Hz (desired {sampling_rate} Hz) on {com_port} ✅")
        except Exception as e:
            print(f"❌ Failed to connect to Mark-10 on {com_port}: {e}")
            self.serial_connection = None
            raise  # Re-raise exception so setup fails properly
    
    def measure_timing(self, num_samples=1000):
        """
        Measure timing performance using existing connection
        
        Args:
            num_samples: Number of samples to measure
            
        Returns:
            timing_stats: Dictionary with timing statistics
        """
        print(f"Measuring timing for {num_samples} data fetches...")
        
        if not self.serial_connection or not self.serial_connection.is_open:
            print("❌ Serial connection not available")
            return None
        
        timing_data = np.zeros(num_samples)
        
        for i in range(num_samples):
            start_time = time.perf_counter()
            self.serial_connection.write("?\r".encode())
            response = self.serial_connection.readline().decode().strip()
            end_time = time.perf_counter()
            
            timing_data[i] = (end_time - start_time) * 1000  # Convert to ms
        
        # Calculate statistics
        stats = {
            'avg_time': np.mean(timing_data),
            'min_time': np.min(timing_data),
            'max_time': np.max(timing_data),
            'std_time': np.std(timing_data),
            'data_rate': 1000 / np.mean(timing_data),
            'num_samples': num_samples
        }
        
        # print(f"Timing Results:")
        # print(f"  Average time: {stats['avg_time']:.2f} ms")
        # print(f"  Data rate: {stats['data_rate']:.1f} Hz")
        # print(f"  Min/Max: {stats['min_time']:.2f}/{stats['max_time']:.2f} ms")
        
        return stats
    

    def get_tension(self):  

        self.serial_connection.write("?\r".encode())
        response = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
        
        force_value = 0
        if response:
           
            # Parse force value
            force_str = response.replace('N', '').replace('lbF', '').replace('lb', '').strip()
            force_value = float(force_str)

        return force_value

    
    def close(self):
        """Close serial connection"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print(f"Serial connection to {self.com_port} closed")
    
    def __del__(self):
        """Destructor - ensure connection is closed"""
        self.close()
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensure connection is closed"""
        self.close()

def main():
    """Main function with usage examples"""
    print("=" * 60)
    print("Mark-10 Force Gauge Data Acquisition")
    print("=" * 60)
    
    
    
    # Example 2: Multiple sensors with synchronized acquisition and plotting
    # com_ports = ["COM4", "COM5"]  # Add "COM6", "COM7" for 4 sensors
    com_ports = ["/dev/ttyUSB0", "/dev/ttyUSB1"]  # For Linux
    
    sensor = SimpleMark10(com_ports[1])


    while True:
        force = sensor.get_tension()
        print(f"\rMeasured force: {force:.3f} N", end='', flush=True)

    

if __name__ == "__main__":
    main()
