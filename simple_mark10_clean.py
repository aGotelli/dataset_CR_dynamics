import serial
import time
import numpy as np
import csv

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
        self.is_running = False
        self.serial_connection = None
        
        print(f"Mark-10 initialized - {sampling_rate} Hz on {com_port}")
    
    def measure_timing(self, num_samples=1000):
        """
        Measure timing performance (from testMark10.py functionality)
        
        Args:
            num_samples: Number of samples to measure
            
        Returns:
            timing_stats: Dictionary with timing statistics
        """
        print(f"Measuring timing for {num_samples} data fetches...")
        
        timing_data = np.zeros(num_samples)
        
        with serial.Serial(self.com_port, baudrate=115200, timeout=1) as ser:
            for i in range(num_samples):
                start_time = time.perf_counter()
                ser.write("?\r".encode())
                response = ser.readline().decode().strip()
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
        
        print(f"Timing Results:")
        print(f"  Average time: {stats['avg_time']:.2f} ms")
        print(f"  Data rate: {stats['data_rate']:.1f} Hz")
        print(f"  Min/Max: {stats['min_time']:.2f}/{stats['max_time']:.2f} ms")
        
        return stats
    
    def acquire_data(self, duration, output_file="mark10_data.csv"):
        """
        Acquire data for specified duration
        
        Args:
            duration: Acquisition time in seconds
            output_file: Output CSV filename
            
        Returns:
            success: True if acquisition completed successfully
        """
        print(f"Starting Mark-10 acquisition for {duration} seconds...")
        print(f"Output file: {output_file}")
        
        samples = int(duration * self.sampling_rate)
        data = np.zeros((samples, 2))  # timestamp, force
        
        try:
            with serial.Serial(self.com_port, baudrate=115200, timeout=self.timeout) as ser:
                self.serial_connection = ser
                
                start_time = time.time()
                sample_count = 0
                target_interval = 1.0 / self.sampling_rate
                next_sample_time = start_time
                
                print("Starting data collection...")
                self.is_running = True
                
                # Main acquisition loop
                while self.is_running and (time.time() - start_time) < duration and sample_count < samples:
                    if time.time() >= next_sample_time:
                        ser.write("?\r".encode())
                        response = ser.readline().decode('utf-8', errors='ignore').strip()
                        
                        if response:
                            try:
                                # Parse force value
                                force_str = response.replace('N', '').replace('lbF', '').replace('lb', '').strip()
                                force_value = float(force_str)
                                
                                # Store data
                                data[sample_count, 0] = time.time()
                                data[sample_count, 1] = force_value
                                sample_count += 1
                            except:
                                pass  # Skip invalid readings
                        
                        next_sample_time += target_interval
                    
                    # # Progress update
                    # if sample_count % self.sampling_rate == 0 and sample_count > 0:
                    #     elapsed = time.time() - start_time
                    #     remaining = duration - elapsed
                    #     print(f"  Remaining: {remaining:.1f}s")
                
                self.is_running = False
                actual_duration = time.time() - start_time
                actual_rate = sample_count / actual_duration if actual_duration > 0 else 0
                
                print(f"Acquisition completed:")
                print(f"  Total samples: {sample_count}")
                print(f"  Actual duration: {actual_duration:.2f} seconds")
                print(f"  Actual rate: {actual_rate:.1f} Hz")
                
                # Save data
                actual_data = data[:sample_count]
                self.save_data(actual_data, output_file)
                
                return True
                
        except Exception as e:
            print(f"Error during acquisition on {self.com_port}: {e}")
            return False
    
    def save_data(self, data, filename):
        """
        Save data to CSV file
        
        Args:
            data: numpy array of data rows [timestamp, force]
            filename: Output filename
        """
        if len(data) == 0:
            print("No data to save")
            return False
        
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'force'])
                writer.writerows(data)
            
            print(f"Data saved to {filename}")
            print(f"  Rows: {len(data)}")
            
            return True
            
        except Exception as e:
            print(f"Error saving data: {e}")
            return False
    
    def stop(self):
        """Stop data acquisition"""
        self.is_running = False
        print("Stop signal sent")

class MultiMark10:
    """
    Multiple Mark-10 sensor management with synchronized data collection
    """
    
    def __init__(self, com_ports, sampling_rate=350):
        """
        Initialize multiple sensors
        
        Args:
            com_ports: List of COM port strings (e.g., ["COM4", "COM5", "COM6", "COM7"])
            sampling_rate: Sampling rate for all sensors
        """
        self.sensors = []
        
        for i, com_port in enumerate(com_ports):
            sensor = SimpleMark10(com_port, sampling_rate)
            self.sensors.append(sensor)
            print(f"Added sensor {i+1} on {com_port}")
    
    def acquire_synchronized_data(self, duration, output_dir="data"):
        """
        Acquire data from all sensors simultaneously
        
        Args:
            duration: Duration in seconds
            output_dir: Output directory
        """
        import threading
        import os
        
        os.makedirs(output_dir, exist_ok=True)
        
        def acquire_sensor_data(sensor_index, sensor):
            filename = f"{output_dir}/mark10_{sensor_index+1}.csv"
            success = sensor.acquire_data(duration, filename)
            if success:
                print(f"✓ Sensor {sensor_index+1} completed successfully")
            else:
                print(f"✗ Sensor {sensor_index+1} failed")
        
        # Start all threads at once
        threads = []
        for i, sensor in enumerate(self.sensors):
            thread = threading.Thread(target=acquire_sensor_data, args=(i, sensor))
            threads.append(thread)
        
        # Start all threads simultaneously
        print("Starting all sensors simultaneously...")
        for thread in threads:
            thread.start()
        
        # Wait for all to complete
        for thread in threads:
            thread.join()
        
        print("All sensors completed acquisition")

def main():
    """Main function with usage examples"""
    print("=" * 60)
    print("Mark-10 Force Gauge Data Acquisition")
    print("=" * 60)
    
    # Example 1: Single sensor
    # sensor = SimpleMark10("COM5", sampling_rate=350)
    # sensor.measure_timing(1000)
    # sensor.acquire_data(duration=10, output_file="data/single_mark10.csv")
    
    # Example 2: Multiple sensors - simple!
    com_ports = ["COM4", "COM5"]  # Add "COM6", "COM7" for 4 sensors
    
    multi_sensor = MultiMark10(com_ports)
    multi_sensor.acquire_synchronized_data(duration=10, output_dir="data")

if __name__ == "__main__":
    main()
