import nidaqmx
from nidaqmx.constants import AcquisitionType
from nidaqmx.errors import DaqError
import numpy as np
import time
import csv

class ATI_FTSensor:
    """
    Simple ATI Force/Torque sensor data acquisition with threading support
    """

    def __init__(self, device_channels="Dev1/ai0:5", sampling_rate=1000, name="ATI_FT"):
        """
        Initialize the sensor and configure DAQ task
        
        Args:
            device_channels: DAQ channels (e.g., "Dev1/ai0:5")
            sampling_rate: Sampling frequency in Hz
            name: Sensor name for logging
        """
        self.device_channels = device_channels
        self.sampling_rate = sampling_rate
        self.name = name
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
        
        # Initialize and configure DAQ task
        self.task = nidaqmx.Task()
        self.task.ai_channels.add_ai_voltage_chan(self.device_channels,
            min_val=-10.0,
            max_val=10.0
        )
        # Configure timing
        self.task.timing.cfg_samp_clk_timing(
            rate=self.sampling_rate,
            sample_mode=AcquisitionType.CONTINUOUS
        )
        
        # Initialize data acquisition constants
        self.num_channels = 6  # Fx, Fy, Fz, Mx, My, Mz
        
        # Calculate initial bias (tare)
        self.bias_voltages = self.calculate_bias()
            
        print(f"ATI F/T Sensor '{name}' initialized - {sampling_rate} Hz on {device_channels}")
    
    def calculate_bias(self, num_samples=100):
        """
        Calculate bias (tare) values by averaging multiple samples
        
        Args:
            num_samples: Number of samples to average for bias calculation
            
        Returns:
            bias_voltages: Average voltage readings for bias correction
        """
        print("Calculating bias (tare)...")
        bias_data = []
        
        for i in range(num_samples):
            try:
                raw_voltages = self.task.read(number_of_samples_per_channel=1)
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
    
    def recalculate_bias(self, num_samples=100):
        """Recalculate bias if sensor conditions have changed"""
        self.bias_voltages = self.calculate_bias(num_samples)
        print("Bias recalculated")
    
    def acquire_data(self, duration, output_file="ati_data.csv"):
        """
        Acquire data for specified duration
        
        Args:
            duration: Acquisition time in seconds
            output_file: Output CSV filename
        """        
        try:
            # Prepare data storage (depends on duration)
            expected_samples = int(self.sampling_rate * duration)
            # Add extra buffer to prevent index out of bounds
            all_data = np.zeros((expected_samples + 1000, self.num_channels + 1))  # +1 for timestamp
            start_time = time.time()
            sample_count = 0
            
            # Main acquisition loop
            while (time.time() - start_time) < duration:
                # Read one sample from all channels
                raw_voltages = self.task.read(number_of_samples_per_channel=1)
                timestamp = time.time()
                    
                # Always compute both raw and bias-corrected data
                corrected_voltages = np.array(raw_voltages) - self.bias_voltages
                
                # Apply calibration matrix to corrected voltages
                ft_values = np.dot(self.calibration_matrix, corrected_voltages)
                    
                # Ensure ft_values is flattened to avoid bracket issues in CSV
                ft_values_flat = ft_values.flatten()
                
                data_row = [timestamp] + list(ft_values_flat)
                    
                all_data[sample_count, :] = data_row
                
                sample_count += 1

                # Check if we've reached the buffer limit
                if sample_count >= all_data.shape[0]:
                    print("Warning: Data buffer full, stopping acquisition early")
                    break
            
            print(f"ATi Acquisition completed:")
            
            # Trim the data array to actual samples collected
            actual_data = all_data[:sample_count, :]
            
            # Save data to CSV
            self.save_data(actual_data, output_file)
            
            return True
                
        except DaqError as e:
            print(f"DAQ Error: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error: {e}")
            return False
    
    def save_data(self, data, filename, include_raw_voltages=False): ############## CHECK SE USIAMO O NO QUETA PARTE!
        """
        Save data to CSV file
        
        Args:
            data: numpy array or list of data rows 
            filename: Output filename
            include_raw_voltages: Whether data includes raw voltage columns
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
            

def main():
    """Main function - simple ATI data acquisition"""
    print("=" * 60)
    print("Simple ATI Force/Torque Sensor Data Acquisition")
    print("=" * 60)
   
    duration = 10
    rate = 2000
    output = "data/test_data.csv"

    sensor = SimpleATI_FT(sampling_rate=rate)
    success = sensor.acquire_data(duration, output)
    
    if success:
        print(f"✅ Data acquisition completed successfully!")
        print(f"📁 Data saved to: {output}")
    else:
        print("❌ Data acquisition failed!")


if __name__ == "__main__":
    main()
