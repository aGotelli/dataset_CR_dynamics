import nidaqmx
from nidaqmx.constants import AcquisitionType
from nidaqmx.errors import DaqError
import numpy as np
import time
import threading
import queue
import csv
from datetime import datetime
import argparse

class ATI_FT_Sensor:
    """
    Optimized ATI Force/Torque sensor data acquisition class
    Supports multi-threading and pre-allocated memory
    """
    
    def __init__(self, device_channels="Dev1/ai0:5", sampling_rate=2000, 
                 calibration_matrix=None, sensor_name="FT_Sensor"):
        """
        Initialize ATI F/T sensor
        
        Args:
            device_channels: DAQ channels (e.g., "Dev1/ai0:5")
            sampling_rate: Sampling frequency in Hz (recommended: 1000-2000)
            calibration_matrix: 6x6 calibration matrix
            sensor_name: Name identifier for the sensor
        """
        self.device_channels = device_channels
        self.sampling_rate = sampling_rate
        self.sensor_name = sensor_name
        self.is_running = False
        self.data_queue = queue.Queue(maxsize=1000)  # Thread-safe queue
        
        # Default calibration matrix (replace with your sensor's matrix)
        if calibration_matrix is None:
            self.calibration_matrix = np.array([
                [0.13221, -0.07541, 0.07645, 6.18461, -0.15573, -6.10142],
                [-0.16220, -7.45852, 0.11342, 3.46326, 0.05746, 3.61610],
                [10.41723, 0.02199, 10.34789, -0.17272, 10.74723, -0.41960],
                [-0.00066, -0.04164, 0.15144, 0.01453, -0.14737, 0.02745],
                [-0.17053, -0.00023, 0.08313, -0.03642, 0.08890, 0.03090],
                [-0.00105, -0.09214, -0.00218, -0.08602, -0.00095, -0.08592]
            ])
        else:
            self.calibration_matrix = np.array(calibration_matrix)
        
        # Pre-allocated arrays for better performance
        self.buffer_size = 1000  # Adjust based on your needs
        self.voltage_buffer = np.zeros((self.buffer_size, 6))
        self.ft_buffer = np.zeros((self.buffer_size, 6))
        self.timestamp_buffer = np.zeros(self.buffer_size)
        self.buffer_index = 0
        
        print(f"Initialized {sensor_name} - Sampling at {sampling_rate} Hz")

    def _acquisition_thread(self, duration, bias_samples=100):
        """
        Thread function for data acquisition
        
        Args:
            duration: Acquisition duration in seconds
            bias_samples: Number of samples for bias calculation
        """
        try:
            with nidaqmx.Task() as task:
                # Configure analog input channels
                task.ai_channels.add_ai_voltage_chan(
                    self.device_channels, 
                    min_val=-10.0, 
                    max_val=10.0
                    # Using default terminal configuration (RSE or NRSE)
                )
                
                # Configure timing - use hardware timing for better precision
                task.timing.cfg_samp_clk_timing(
                    rate=self.sampling_rate,
                    sample_mode=AcquisitionType.CONTINUOUS,
                    samps_per_chan=self.sampling_rate  # 1 second buffer
                )
                
                print(f"{self.sensor_name}: Starting acquisition...")
                
                # Calculate bias (tare) values
                print(f"{self.sensor_name}: Calculating bias...")
                bias_data = []
                for _ in range(bias_samples):
                    raw_data = np.array(task.read(number_of_samples_per_channel=1))
                    bias_data.append(raw_data)
                    time.sleep(0.001)  # Small delay between samples
                
                bias_voltages = np.mean(bias_data, axis=0)
                bias_ft = np.dot(self.calibration_matrix, bias_voltages)
                
                print(f"{self.sensor_name}: Bias calculated - Starting data collection...")
                
                # Main acquisition loop
                start_time = time.time()
                sample_count = 0
                
                while self.is_running and (time.time() - start_time) < duration:
                    try:
                        # Read data (can read multiple samples for efficiency)
                        raw_data = np.array(task.read(number_of_samples_per_channel=10))
                        current_time = time.time()
                        
                        # Process each sample in the batch
                        if raw_data.ndim == 1:
                            raw_data = raw_data.reshape(1, -1)
                        
                        for i in range(raw_data.shape[0]):
                            if not self.is_running:
                                break
                                
                            # Apply calibration matrix (subtract bias)
                            voltage_sample = raw_data[i] - bias_voltages
                            ft_sample = np.dot(self.calibration_matrix, voltage_sample)
                            
                            # Calculate timestamp for this specific sample
                            sample_timestamp = current_time - (raw_data.shape[0] - 1 - i) / self.sampling_rate
                            
                            # Store in pre-allocated buffer
                            if self.buffer_index < self.buffer_size:
                                self.voltage_buffer[self.buffer_index] = voltage_sample
                                self.ft_buffer[self.buffer_index] = ft_sample
                                self.timestamp_buffer[self.buffer_index] = sample_timestamp
                                self.buffer_index += 1
                            
                            # When buffer is full, put data in queue and reset
                            if self.buffer_index >= self.buffer_size:
                                data_chunk = {
                                    'timestamp': self.timestamp_buffer.copy(),
                                    'voltage': self.voltage_buffer.copy(),
                                    'ft': self.ft_buffer.copy(),
                                    'sensor_name': self.sensor_name
                                }
                                
                                try:
                                    self.data_queue.put(data_chunk, timeout=0.1)
                                except queue.Full:
                                    print(f"{self.sensor_name}: Warning - Data queue full, dropping samples")
                                
                                self.buffer_index = 0
                            
                            sample_count += 1
                            
                            # Print progress every 1000 samples
                            if sample_count % 1000 == 0:
                                elapsed = time.time() - start_time
                                actual_rate = sample_count / elapsed if elapsed > 0 else 0
                                print(f"{self.sensor_name}: {sample_count} samples, "
                                     f"Rate: {actual_rate:.1f} Hz")
                    
                    except Exception as e:
                        print(f"{self.sensor_name}: Error in acquisition loop: {e}")
                        continue
                
                # Handle remaining data in buffer
                if self.buffer_index > 0:
                    data_chunk = {
                        'timestamp': self.timestamp_buffer[:self.buffer_index].copy(),
                        'voltage': self.voltage_buffer[:self.buffer_index].copy(),
                        'ft': self.ft_buffer[:self.buffer_index].copy(),
                        'sensor_name': self.sensor_name
                    }
                    try:
                        self.data_queue.put(data_chunk, timeout=1.0)
                    except queue.Full:
                        print(f"{self.sensor_name}: Warning - Could not save final data chunk")
                
                print(f"{self.sensor_name}: Acquisition completed. Total samples: {sample_count}")
                
        except DaqError as e:
            print(f"{self.sensor_name}: DAQmx Error: {e}")
        except Exception as e:
            print(f"{self.sensor_name}: Unexpected error: {e}")
        finally:
            self.is_running = False

    def start_acquisition(self, duration, bias_samples=100):
        """
        Start data acquisition in a separate thread
        
        Args:
            duration: Acquisition duration in seconds
            bias_samples: Number of samples for bias calculation
        """
        if self.is_running:
            print(f"{self.sensor_name}: Acquisition already running")
            return False
        
        self.is_running = True
        self.acquisition_thread = threading.Thread(
            target=self._acquisition_thread,
            args=(duration, bias_samples),
            daemon=True
        )
        self.acquisition_thread.start()
        return True

    def stop_acquisition(self):
        """Stop data acquisition"""
        self.is_running = False
        if hasattr(self, 'acquisition_thread'):
            self.acquisition_thread.join(timeout=5.0)

    def get_data(self, timeout=1.0):
        """
        Get data from the queue
        
        Returns:
            Dictionary with timestamp, voltage, ft data, or None if no data available
        """
        try:
            return self.data_queue.get(timeout=timeout)
        except queue.Empty:
            return None

class MultiSensorManager:
    """
    Manager class for multiple ATI F/T sensors
    """
    
    def __init__(self):
        self.sensors = {}
        self.data_writer = None
        self.output_filename = None
        
    def add_sensor(self, sensor_name, device_channels, sampling_rate=2000, calibration_matrix=None):
        """
        Add a sensor to the manager
        
        Args:
            sensor_name: Unique name for the sensor
            device_channels: DAQ channels for this sensor
            sampling_rate: Sampling frequency
            calibration_matrix: 6x6 calibration matrix
        """
        sensor = ATI_FT_Sensor(
            device_channels=device_channels,
            sampling_rate=sampling_rate,
            calibration_matrix=calibration_matrix,
            sensor_name=sensor_name
        )
        self.sensors[sensor_name] = sensor
        print(f"Added sensor: {sensor_name}")

    def start_all_sensors(self, duration, output_filename="multi_sensor_data.csv"):
        """
        Start acquisition for all sensors simultaneously
        
        Args:
            duration: Acquisition duration in seconds
            output_filename: Output CSV filename
        """
        if not self.sensors:
            print("No sensors configured")
            return False
        
        self.output_filename = output_filename
        
        # Start all sensors
        for sensor_name, sensor in self.sensors.items():
            success = sensor.start_acquisition(duration)
            if not success:
                print(f"Failed to start {sensor_name}")
                return False
        
        # Start data collection thread
        self.data_collection_thread = threading.Thread(
            target=self._collect_data,
            args=(duration,),
            daemon=True
        )
        self.data_collection_thread.start()
        
        print(f"Started acquisition for {len(self.sensors)} sensors")
        return True

    def _collect_data(self, duration):
        """
        Collect data from all sensors and save to file
        """
        # Prepare CSV file
        with open(self.output_filename, 'w', newline='') as csvfile:
            # Create header
            header = ['timestamp', 'sensor_name', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
            writer = csv.writer(csvfile)
            writer.writerow(header)
            
            start_time = time.time()
            total_samples = 0
            
            while (time.time() - start_time) < duration + 2:  # Small buffer time
                any_data_received = False
                
                # Check each sensor for data
                for sensor_name, sensor in self.sensors.items():
                    data_chunk = sensor.get_data(timeout=0.1)
                    if data_chunk is not None:
                        any_data_received = True
                        
                        # Write data chunk to file
                        timestamps = data_chunk['timestamp']
                        ft_data = data_chunk['ft']
                        
                        for i in range(len(timestamps)):
                            row = [timestamps[i], sensor_name] + ft_data[i].tolist()
                            writer.writerow(row)
                            total_samples += 1
                
                if not any_data_received:
                    time.sleep(0.01)  # Small sleep if no data
            
            print(f"Data collection completed. Total samples: {total_samples}")
            print(f"Data saved to: {self.output_filename}")

    def stop_all_sensors(self):
        """Stop all sensors"""
        for sensor_name, sensor in self.sensors.items():
            sensor.stop_acquisition()
            print(f"Stopped {sensor_name}")

def main():
    parser = argparse.ArgumentParser(description="Optimized ATI F/T sensor data acquisition")
    parser.add_argument('--duration', type=int, default=10, help="Duration in seconds")
    parser.add_argument('--rate', type=int, default=2000, help="Sampling rate in Hz")
    parser.add_argument('--output', type=str, default="ft_sensor_data.csv", help="Output filename")
    parser.add_argument('--multi', action='store_true', help="Enable multi-sensor mode")
    
    # Parse arguments, but provide defaults if running without parameters
    try:
        args = parser.parse_args()
    except SystemExit:
        # If no arguments provided, use defaults
        class DefaultArgs:
            duration = 10
            rate = 2000
            output = "ft_sensor_data.csv"
            multi = False
        args = DefaultArgs()
    
    print(f"Starting ATI F/T sensor acquisition:")
    print(f"  Duration: {args.duration} seconds")
    print(f"  Sampling rate: {args.rate} Hz")
    print(f"  Output file: {args.output}")
    print(f"  Multi-sensor mode: {args.multi}")
    print()
    
    if args.multi:
        # Multi-sensor example
        manager = MultiSensorManager()
        
        # Add sensors (customize these for your setup)
        manager.add_sensor("Sensor_1", "Dev1/ai0:5", sampling_rate=args.rate)
        # Uncomment the next line if you have a second DAQ device
        # manager.add_sensor("Sensor_2", "Dev2/ai0:5", sampling_rate=args.rate)
        
        try:
            manager.start_all_sensors(args.duration, args.output)
            
            # Wait for completion with progress updates
            for i in range(args.duration):
                time.sleep(1)
                print(f"Progress: {i+1}/{args.duration} seconds")
            
            time.sleep(1)  # Small buffer
            
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            manager.stop_all_sensors()
    
    else:
        # Single sensor mode
        sensor = ATI_FT_Sensor(
            device_channels="Dev1/ai0:5",
            sampling_rate=args.rate,
            sensor_name="ATI_FT_Main"
        )
        
        try:
            sensor.start_acquisition(args.duration)
            
            # Progress indicator
            print("Data collection in progress...")
            for i in range(args.duration):
                time.sleep(1)
                print(f"Progress: {i+1}/{args.duration} seconds")
            
            # Collect and save data
            all_data = []
            start_time = time.time()
            
            print("Collecting data from buffers...")
            while (time.time() - start_time) < 3:  # Wait up to 3 seconds for data
                data_chunk = sensor.get_data(timeout=1.0)
                if data_chunk is not None:
                    all_data.append(data_chunk)
                else:
                    break
            
            # Save to CSV
            if all_data:
                with open(args.output, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['timestamp', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
                    
                    total_samples = 0
                    for chunk in all_data:
                        timestamps = chunk['timestamp']
                        ft_data = chunk['ft']
                        
                        for i in range(len(timestamps)):
                            row = [timestamps[i]] + ft_data[i].tolist()
                            writer.writerow(row)
                            total_samples += 1
                
                print(f"Data saved to {args.output}")
                print(f"Total samples collected: {total_samples}")
                print(f"Actual sampling rate: {total_samples/args.duration:.1f} Hz")
            else:
                print("No data collected. Check your DAQ connection.")
            
        except KeyboardInterrupt:
            print("Interrupted by user")
        except Exception as e:
            print(f"Error during acquisition: {e}")
        finally:
            sensor.stop_acquisition()

# Simple function to run with default settings
def run_default():
    """Run with default settings - no command line arguments needed"""
    print("=" * 60)
    print("ATI Force/Torque Sensor Data Acquisition")
    print("=" * 60)
    print("Running with default settings:")
    print("  Duration: 10 seconds")
    print("  Sampling rate: 2000 Hz") 
    print("  Output: ft_sensor_data.csv")
    print("  Single sensor mode")
    print("=" * 60)
    
    # Create sensor with default settings
    sensor = ATI_FT_Sensor(
        device_channels="Dev1/ai0:5",
        sampling_rate=2000,
        sensor_name="ATI_FT_Default"
    )
    
    try:
        duration = 10
        sensor.start_acquisition(duration)
        
        # Progress indicator
        print("\nData collection in progress...")
        for i in range(duration):
            time.sleep(1)
            print(f"Progress: {i+1}/{duration} seconds", end="\r")
        print()  # New line after progress
        
        # Collect and save data
        all_data = []
        start_time = time.time()
        
        print("Collecting data from buffers...")
        while (time.time() - start_time) < 3:  # Wait up to 3 seconds for data
            data_chunk = sensor.get_data(timeout=1.0)
            if data_chunk is not None:
                all_data.append(data_chunk)
            else:
                break
        
        # Save to CSV
        output_file = "ft_sensor_data.csv"
        if all_data:
            with open(output_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['timestamp', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
                
                total_samples = 0
                for chunk in all_data:
                    timestamps = chunk['timestamp']
                    ft_data = chunk['ft']
                    
                    for i in range(len(timestamps)):
                        row = [timestamps[i]] + ft_data[i].tolist()
                        writer.writerow(row)
                        total_samples += 1
            
            print(f"\n✓ Data saved to {output_file}")
            print(f"✓ Total samples collected: {total_samples}")
            print(f"✓ Actual sampling rate: {total_samples/duration:.1f} Hz")
            print(f"✓ File size: {total_samples * 7 * 8 / 1024:.1f} KB (estimated)")
        else:
            print("\n❌ No data collected. Check your DAQ connection and device channels.")
            print("   Make sure 'Dev1/ai0:5' channels are available.")
            
    except Exception as e:
        print(f"\n❌ Error during acquisition: {e}")
        print("   Common issues:")
        print("   - DAQ device not connected")
        print("   - Wrong device name (try 'Dev2' instead of 'Dev1')")
        print("   - Channels already in use by another application")
    finally:
        sensor.stop_acquisition()
        print("\n" + "=" * 60)

if __name__ == "__main__":
    # Check if any command line arguments were provided
    import sys
    
    if len(sys.argv) == 1:
        # No arguments provided, run with defaults
        run_default()
    else:
        # Arguments provided, use full argument parser
        main()
