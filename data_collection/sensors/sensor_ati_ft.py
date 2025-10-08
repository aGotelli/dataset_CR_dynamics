import nidaqmx
from nidaqmx.constants import AcquisitionType
from nidaqmx.errors import DaqError
import numpy as np
import time
import csv
import threading

class ATI_FTSensor:
    """
    Simple ATI Force/Torque sensor data acquisition with threading support
    """

    def __init__(self, device_channels, sampling_rate, duration, output_file):
        """
        Initialize the sensor and configure DAQ task
        Pre-allocate memory and setup everything needed for acquisition
        
        Args:
            device_channels: DAQ channels (e.g., "Dev1/ai0:5")
            sampling_rate: Sampling frequency in Hz
            name: Sensor name for logging
            duration: Data acquisition duration in seconds
            output_file: Output file path for data saving
        """
        print(f"🔧 Creating ATI sensor instance")

        self.device_channels = device_channels
        self.sampling_rate = sampling_rate
        self.duration = duration
        self.output_file = output_file
        
        # Thread management
        self.acquisition_thread = None
        self.task = None  # DAQ task 
        
        # ATI calibration matrix (replace with your sensor's matrix)
        self.calibration_matrix = np.array([
            [0.13221, -0.07541, 0.07645, 6.18461, -0.15573, -6.10142],
            [-0.16220, -7.45852, 0.11342, 3.46326, 0.05746, 3.61610],
            [10.41723, 0.02199, 10.34789, -0.17272, 10.74723, -0.41960],
            [-0.00066, -0.04164, 0.15144, 0.01453, -0.14737, 0.02745],
            [-0.17053, -0.00023, 0.08313, -0.03642, 0.08890, 0.03090],
            [-0.00105, -0.09214, -0.00218, -0.08602, -0.00095, -0.08592]
        ])
        
        # Initialize data acquisition constants
        self.num_channels = 6  # Fx, Fy, Fz, Mx, My, Mz
        
        # Pre-allocate memory 
        expected_samples = int(self.sampling_rate * duration)
        buffer_samples = int(expected_samples * 3.0) 
        self.all_data = np.zeros((buffer_samples, self.num_channels + 1))  # +1 for timestamp
        
        # Create and configure DAQ task
        self.task = nidaqmx.Task()
        self.task.ai_channels.add_ai_voltage_chan(self.device_channels,
            min_val=-10.0,
            max_val=10.0
        )
        self.task.timing.cfg_samp_clk_timing(
            rate=self.sampling_rate,
            sample_mode=AcquisitionType.CONTINUOUS
        )
        
        # Calculate initial bias (tare) using the configured task
        self.bias_voltages = self.calculate_bias(self.task)
            
        print(f"✅ ATI F/T Sensor initialized - {sampling_rate} Hz on {device_channels}")

    def calculate_bias(self, task, num_samples=100):
        """
        Calculate bias (tare) values by averaging multiple samples
        
        Args:
            task: NI-DAQmx task object
            num_samples: Number of samples to average for bias calculation
            
        Returns:
            bias_voltages: Average voltage readings for bias correction
        """
        print("Calculating bias...")
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
        print(f"Bias voltages: {bias_voltages}")
        
        return bias_voltages
    
    def acquire_data(self):
        """
        Acquire data for specified duration
        
        Args:
            duration: Acquisition time in seconds
            output_file: Output CSV filename
        """                
        try:
            # Use pre-allocated memory and pre-configured task
            all_data = self.all_data
           
            sample_count = 0
             
            print("ATI - Starting data collection...")
            start_time = time.time()

            # Main acquisition loop - direct use of persistent task
            while (time.time() - start_time) < self.duration:
                try:
                    # Read one sample from all channels using persistent task
                    raw_voltages = self.task.read(number_of_samples_per_channel=1)
                    timestamp = time.time()
                        
                    # Flatten raw_voltages to match bias shape (6,) instead of (6,1)
                    corrected_voltages = np.array(raw_voltages) - self.bias_voltages
                    ft_values = np.dot(self.calibration_matrix, corrected_voltages)
                    ft_values_flat = ft_values.flatten()

                    # Direct assignment to avoid list conversion overhead
                    all_data[sample_count, 0] = timestamp
                    all_data[sample_count, 1:] = ft_values_flat
                    
                    sample_count += 1

                    # Check if we've reached the buffer limit
                    if sample_count >= all_data.shape[0]:
                        print(f"Warning: ATI data buffer full, stopping acquisition early - buffer size {all_data.shape[0]} - samples collected {sample_count}")
                        break
                        
                    # # DEBUG --- Progress update every sampling_rate samples
                    # if sample_count % self.sampling_rate == 0:
                    #     elapsed = time.time() - start_time
                    #     remaining = duration - elapsed
                    #     print(f"  ATI - Remaining: {remaining:.1f}s")
                        
                except Exception as e:
                    print(f"Error in ATI acquisition loop: {e}")
                    continue
            
            # actual_duration = time.time() - start_time
            # actual_rate = sample_count / actual_duration if actual_duration > 0 else 0
            
            print(f"ATI Acquisition completed:")
            # print(f"  Total samples: {sample_count}")
            # print(f"  Actual duration: {actual_duration:.2f} seconds")
            # print(f"  Actual rate: {actual_rate:.1f} Hz")
            
            # Trim the data array to actual samples collected
            actual_data = all_data[:sample_count, :]
            
            # Save data to CSV
            self.save_data(actual_data, self.output_file)
            
            return True
            
        except DaqError as e:
            print(f"DAQ Error: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error: {e}")
            return False
    
    def save_data(self, data, filename):
        """
        Save data to CSV file
        
        Args:
            data: numpy array or list of data rows 
            filename: Output filename
        """
        # Check if data is empty (works for both numpy arrays and lists)
        if data is None or len(data) == 0:
            print("No data to save")
            return False
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            header = ['timestamp', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
            writer.writerow(header)
                
            # Write data
            writer.writerows(data)

        print(f"📁 ATIFT data saved to: {filename}")

    def cleanup(self):
        """
        Clean up resources - close DAQ task
        """
        if self.task:
            try:
                self.task.close()
                print("✅ ATI DAQ task closed successfully")
            except Exception as e:
                print(f"⚠️ Error closing ATI DAQ task: {e}")
            finally:
                self.task = None
    

def main():
    """Main function - simple ATI data acquisition"""
    print("=" * 60)
    print("Simple ATI Force/Torque Sensor Data Acquisition")
    print("=" * 60)
    ATI_CHANNELS = "Dev1/ai0:5"
    ATI_RATE = 1000
    duration = 10
    # rate = 2000
    output = "data/test_data.csv"

    sensor = ATI_FTSensor(ATI_CHANNELS, ATI_RATE, duration, output)
    success = sensor.acquire_data()
    
    if success:
        print(f"✅ Data acquisition completed successfully!")
        print(f"📁 Data saved to: {output}")
    else:
        print("❌ Data acquisition failed!")


if __name__ == "__main__":
    main()
