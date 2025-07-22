import nidaqmx
from nidaqmx.constants import AcquisitionType
from nidaqmx.errors import DaqError
import numpy as np
import time
import csv
from datetime import datetime

class SimpleATI_FT:
    """
    Simple ATI Force/Torque sensor data acquisition
    Easy to understand and modify for threading later
    """
    
    def __init__(self, device_channels="Dev1/ai0:5", sampling_rate=1000):
        """
        Initialize the sensor
        
        Args:
            device_channels: DAQ channels (e.g., "Dev1/ai0:5")
            sampling_rate: Sampling frequency in Hz
        """
        self.device_channels = device_channels
        self.sampling_rate = sampling_rate
        self.is_running = False
        
        # ATI calibration matrix (replace with your sensor's matrix)
        self.calibration_matrix = np.array([
            [0.13221, -0.07541, 0.07645, 6.18461, -0.15573, -6.10142],
            [-0.16220, -7.45852, 0.11342, 3.46326, 0.05746, 3.61610],
            [10.41723, 0.02199, 10.34789, -0.17272, 10.74723, -0.41960],
            [-0.00066, -0.04164, 0.15144, 0.01453, -0.14737, 0.02745],
            [-0.17053, -0.00023, 0.08313, -0.03642, 0.08890, 0.03090],
            [-0.00105, -0.09214, -0.00218, -0.08602, -0.00095, -0.08592]
        ])
        
        print(f"ATI F/T Sensor initialized - {sampling_rate} Hz on {device_channels}")
    
    def calculate_bias(self, task, num_samples=100):
        """
        Calculate bias (tare) values by averaging multiple samples
        
        Args:
            task: NI-DAQmx task object
            num_samples: Number of samples to average for bias calculation
            
        Returns:
            bias_voltages: Average voltage readings for bias correction
        """
        print("Calculating bias (tare)...")
        bias_data = []
        
        for i in range(num_samples):
            try:
                raw_voltages = task.read(number_of_samples_per_channel=1)
                bias_data.append(raw_voltages)
                
                # Progress indicator
                if (i + 1) % 20 == 0:
                    print(f"  Bias progress: {i+1}/{num_samples}")
                    
            except Exception as e:
                print(f"Error reading bias sample {i}: {e}")
                continue
        
        if not bias_data:
            print("Warning: No bias data collected, using zeros")
            return np.zeros(6)
        
        bias_voltages = np.mean(bias_data, axis=0)
        print(f"Bias calculated from {len(bias_data)} samples")
        print(f"Bias voltages: {bias_voltages}")
        
        return bias_voltages
    
    def acquire_data(self, duration, output_file="ati_data.csv", calculate_bias=True, save_raw_voltages=False):
        """
        Acquire data for specified duration
        
        Args:
            duration: Acquisition time in seconds
            output_file: Output CSV filename
            calculate_bias: Whether to calculate and subtract bias
            save_raw_voltages: Whether to save raw voltages alongside F/T data
            
        Returns:
            success: True if acquisition completed successfully
        """
        print(f"Starting data acquisition for {duration} seconds...")
        print(f"Output file: {output_file}")
        
        try:
            with nidaqmx.Task() as task:
                # Configure analog input
                task.ai_channels.add_ai_voltage_chan(
                    self.device_channels,
                    min_val=-10.0,
                    max_val=10.0
                )
                
                # Configure timing
                task.timing.cfg_samp_clk_timing(
                    rate=self.sampling_rate,
                    sample_mode=AcquisitionType.CONTINUOUS
                )
                
                # Calculate bias if requested
                bias_voltages = np.zeros(6)
                if calculate_bias:
                    bias_voltages = self.calculate_bias(task)
                
                # Prepare data storage
                all_data = []
                start_time = time.time()
                sample_count = 0
                
                print("Starting data collection...")
                self.is_running = True
                
                # Main acquisition loop
                while self.is_running and (time.time() - start_time) < duration:
                    try:
                        # Read one sample from all channels
                        raw_voltages = task.read(number_of_samples_per_channel=1)
                        timestamp = time.time()
                        
                        # Always compute both raw and bias-corrected data
                        corrected_voltages = np.array(raw_voltages) - bias_voltages
                        
                        # Apply calibration matrix to corrected voltages
                        ft_values = np.dot(self.calibration_matrix, corrected_voltages)
                        
                        # Ensure ft_values is flattened to avoid bracket issues in CSV
                        ft_values_flat = ft_values.flatten()
                        
                        # Store data based on options
                        if save_raw_voltages:
                            # Extended format: [timestamp, V0-V5, Fx, Fy, Fz, Mx, My, Mz]
                            data_row = [timestamp] + list(raw_voltages) + list(ft_values_flat)
                        else:
                            # Standard format: [timestamp, Fx, Fy, Fz, Mx, My, Mz]
                            data_row = [timestamp] + list(ft_values_flat)
                        
                        all_data.append(data_row)
                        
                        sample_count += 1

                        # Progress update every self.sampling_rate samples
                        if sample_count % self.sampling_rate == 0:
                            elapsed = time.time() - start_time
                            # rate = sample_count / elapsed if elapsed > 0 else 0
                            remaining = duration - elapsed
                            # print(f"  Samples: {sample_count}, Rate: {rate:.1f} Hz, Remaining: {remaining:.1f}s")
                            print(f"  Remaining: {remaining:.1f}s")
                    
                    except Exception as e:
                        print(f"Error in acquisition loop: {e}")
                        continue
                
                self.is_running = False
                actual_duration = time.time() - start_time
                actual_rate = sample_count / actual_duration if actual_duration > 0 else 0
                
                print(f"Acquisition completed:")
                print(f"  Total samples: {sample_count}")
                print(f"  Actual duration: {actual_duration:.2f} seconds")
                print(f"  Actual rate: {actual_rate:.1f} Hz")
                
                # Save data to CSV
                self.save_data(all_data, output_file, save_raw_voltages)
                
                return True
                
        except DaqError as e:
            print(f"DAQ Error: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error: {e}")
            return False
    
    def save_data(self, data, filename, include_raw_voltages=False):
        """
        Save data to CSV file
        
        Args:
            data: List of data rows 
            filename: Output filename
            include_raw_voltages: Whether data includes raw voltage columns
        """
        if not data:
            print("No data to save")
            return False
        
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write header
                if include_raw_voltages:
                    header = ['timestamp', 'V0', 'V1', 'V2', 'V3', 'V4', 'V5', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
                else:
                    header = ['timestamp', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
                writer.writerow(header)
                
                # Write data
                writer.writerows(data)
            
            # File info
            file_size_kb = len(data) * 7 * 8 / 1024  # Rough estimate
            print(f"Data saved to {filename}")
            print(f"  Rows: {len(data)}")
            print(f"  Estimated size: {file_size_kb:.1f} KB")
            
            return True
            
        except Exception as e:
            print(f"Error saving data: {e}")
            return False
    
    def stop(self):
        """Stop data acquisition"""
        self.is_running = False
        print("Stop signal sent")

# Threading-ready version (prepared for future use)
class ThreadableATI_FT(SimpleATI_FT):
    """
    Extended version ready for threading
    This inherits all functionality from SimpleATI_FT but adds threading support
    """
    
    def __init__(self, device_channels="Dev1/ai0:5", sampling_rate=1000, name="ATI_FT"):
        super().__init__(device_channels, sampling_rate)
        self.name = name
        self.data_buffer = []
        self.buffer_lock = None  # Will use threading.Lock() when needed
        
    def threaded_acquire(self, duration, output_file="threaded_ati.csv"):
        """
        Acquisition method ready for threading
        To use with threading later:
        
        import threading
        sensor = ThreadableATI_FT(name="Sensor1")
        thread = threading.Thread(target=sensor.threaded_acquire, args=(10, "sensor1.csv"))
        thread.start()
        """
        print(f"{self.name}: Starting threaded acquisition")
        success = self.acquire_data(duration, output_file)
        print(f"{self.name}: Threaded acquisition {'completed' if success else 'failed'}")
        return success

def main():
    """Main function with menu"""
    print("=" * 60)
    print("Simple ATI Force/Torque Sensor Data Acquisition")
    print("=" * 60)
   

    duration = 60
    rate = 2000

    output = "data/test_data.csv"

    sensor = SimpleATI_FT(sampling_rate=rate)
    return sensor.acquire_data(duration, output)
    


if __name__ == "__main__":
    main()
