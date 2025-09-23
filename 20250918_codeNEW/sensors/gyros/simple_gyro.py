"""
Optimized real-time streaming for ISM330DHCX sensor
Focus: Maximum performance with clean, simple code
"""
from sparkfun_ism330dhcx_interface.python_minimal.wire import TwoWire
import time
import sys
import csv
import os

class GYROSensor:
    """
    ISM330DHCX gyro sensor class - consistent with other sensor interfaces
    Hardware initialization in constructor, acquisition in separate method
    """
    
    def __init__(self, sensor_addr=0x6A, name="gyro"):
        """
        Initialize and configure the gyro sensor
        
        Args:
            sensor_addr: I2C address of the sensor (default 0x6A)
            name: Sensor name for logging
        """
        print(f"🔧 Initializing gyro sensor '{name}'...")
        
        self.sensor_addr = sensor_addr
        self.name = name
        
        # Initialize I2C interface
        self.wire = TwoWire()
        self.wire.begin()
        
        # Scaling factors for data conversion
        self.accel_scale = 0.000061  # g per LSB for ±2g
        self.gyro_scale = 0.00875    # dps per LSB for ±250 dps
        
        # Pre-allocate arrays for maximum performance
        self.accel = [0, 0, 0]
        self.gyro = [0, 0, 0]
        
        # Configure the sensor hardware
        # Verify sensor presence
        self.wire.begin_transmission(self.sensor_addr)
        self.wire.write(0x0F)  # WHO_AM_I register
        self.wire.end_transmission()
        self.wire.request_from(self.sensor_addr, 1)
        
        if self.wire.read() != 0x6B:
            raise Exception(f"gyro sensor not found at address 0x{self.sensor_addr:02X}")
        
        # Configure accelerometer: 416 Hz, ±2g range
        self.wire.begin_transmission(self.sensor_addr)
        self.wire.write(0x10)  # CTRL1_XL
        self.wire.write(0x60)  # 416 Hz, ±2g
        self.wire.end_transmission()
        
        # Configure gyroscope: 416 Hz, ±250 dps range
        self.wire.begin_transmission(self.sensor_addr)
        self.wire.write(0x11)  # CTRL2_G
        self.wire.write(0x60)  # 416 Hz, ±250 dps
        self.wire.end_transmission()
        
        # Configure control register: auto-increment + block data update
        self.wire.begin_transmission(self.sensor_addr)
        self.wire.write(0x12)  # CTRL3_C
        self.wire.write(0x44)  # Auto-increment + BDU
        self.wire.end_transmission()
        
        # Allow sensor startup time
        time.sleep(0.1)        
        print(f"✅ gyro sensor '{name}' ready at ~416 Hz")    
    
    def _read_accelerometer(self):
        """Read accelerometer data from sensor registers"""
        for i, reg in enumerate([0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D]):
            self.wire.begin_transmission(self.sensor_addr)
            self.wire.write(reg)
            self.wire.end_transmission()
            self.wire.request_from(self.sensor_addr, 1)
            
            if i % 2 == 0:  # Low byte
                low = self.wire.read()
            else:  # High byte
                high = self.wire.read()
                # Combine bytes and convert to signed 16-bit
                raw = (high << 8) | low
                if raw > 32767:
                    raw -= 65536
                self.accel[i // 2] = raw * self.accel_scale
    
    def _read_gyroscope(self):
        """Read gyroscope data from sensor registers"""
        for i, reg in enumerate([0x22, 0x23, 0x24, 0x25, 0x26, 0x27]):
            self.wire.begin_transmission(self.sensor_addr)
            self.wire.write(reg)
            self.wire.end_transmission()
            self.wire.request_from(self.sensor_addr, 1)
            
            if i % 2 == 0:  # Low byte
                low = self.wire.read()
            else:  # High byte
                high = self.wire.read()
                # Combine bytes and convert to signed 16-bit
                raw = (high << 8) | low
                if raw > 32767:
                    raw -= 65536
                self.gyro[i // 2] = raw * self.gyro_scale
    
    def acquire_data(self, duration, output_file):
        """
        Acquire gyro data for specified duration - OPTIMIZED FOR THREADING
        No setup overhead, starts immediately!
        
        Args:
            duration: Acquisition time in seconds
            output_file: Output CSV filename
            
        Returns:
            bool: Success status
        """
        print(f"gyro - Starting data acquisition for {duration} seconds...")
        
        try:
            all_data = []
            start_time = time.time()
            sample_count = 0
            
            # Main acquisition loop - maximum performance
            while (time.time() - start_time) < duration:
                try:
                    # Read both sensor types
                    self._read_accelerometer()
                    self._read_gyroscope() #fatti tornare un dato
                    
                    # Store timestamped data
                    timestamp = time.time()
                    data_row = [timestamp] + self.accel + self.gyro
                    all_data.append(data_row)
                    
                    sample_count += 1
                    
                    # ------ QUESTO IF E' UN DEBUG - DA TOGLIERE PER PERFORMANCE MIGLIONI
                    #  Progress update every ~400 samples (roughly every second)
                    # Useful for debugging and monitoring acquisition status
                    if sample_count % 400 == 0:
                        elapsed = time.time() - start_time
                        remaining = duration - elapsed
                        actual_rate = sample_count / elapsed if elapsed > 0 else 0
                        print(f"  gyro: {remaining:.1f}s remaining, {actual_rate:.0f}Hz")
                        
                except Exception as e:
                    print(f"Error in gyro reading: {e}")
                    continue
            
            # Final statistics
            actual_duration = time.time() - start_time
            actual_rate = sample_count / actual_duration if actual_duration > 0 else 0
            
            print(f"gyro Acquisition completed:")
            print(f"  Total samples: {sample_count}")
            print(f"  Actual duration: {actual_duration:.2f} seconds")
            print(f"  Actual rate: {actual_rate:.1f} Hz")
            
            # Save data to CSV
            success = self._save_data(all_data, output_file)
            return success
            
        except Exception as e:
            print(f"❌ gyro acquisition error: {e}")
            return False
    
    def _save_data(self, data, filename):
        """Save gyro data to CSV file"""
        if not data:
            print("No gyro data to save")
            return False
        
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write header
                header = ['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']
                writer.writerow(header)
                
                # Write data
                writer.writerows(data)
                
            print(f"📁 Gyro data saved to: {filename}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to save gyro data: {e}")
            return False
    
    def cleanup(self):
        """Clean up I2C resources"""
        if hasattr(self, 'wire') and self.wire:
            self.wire.end()
            print(f"🧹 Gyro sensor '{self.name}' cleaned up")


# Backward compatibility function for legacy code
def main(duration=None, output_file=None):
    """
    Legacy function - now uses the gyroSensor class internally
    Maintains compatibility with old code that calls main() directly
    """
    print("ISM330DHCX High-Performance Streaming (Legacy Mode)")
    print("===================================================")
    
    # Default values
    if duration is None:
        duration = float('inf')
    if output_file is None:
        output_file = "gyro_data.csv"
    
    try:
        # Use the new class internally
        sensor = GYROSensor(name="Legacy_gyro")
        success = sensor.acquire_data(duration, output_file)
        sensor.cleanup()
        return success
        
    except Exception as e:
        print(f"❌ Legacy gyro acquisition failed: {e}")
        return False


if __name__ == "__main__":
   main()