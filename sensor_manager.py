"""
Centralized Sensor Manager
Handles all sensor hardware access and provides thread-safe data sharing
"""

import threading
import time
import serial
import re
import csv
from collections import deque
import queue


class CentralizedSensorManager:
    """
    Single manager that owns all sensor hardware and provides thread-safe data access
    Prevents COM port conflicts by centralizing hardware access
    """
    
    def __init__(self):
        self.mark10_sensors = {}  # {port: sensor_info}
        self.data_locks = {}      # {port: threading.Lock()}
        self.sensor_threads = {}  # {port: threading.Thread()}
        self.running = False
        self.data_loggers = {}    # {port: data_logger_thread()}
        
    def add_mark10_sensor(self, port, buffer_size=1000, sampling_rate=350):
        """Add a Mark-10 sensor to be managed"""
        self.mark10_sensors[port] = {
            'buffer': deque(maxlen=buffer_size),
            'sampling_rate': sampling_rate,
            'total_samples': 0,
            'last_reading': None,
            'errors': 0
        }
        self.data_locks[port] = threading.Lock()
        print(f"Added Mark-10 sensor on {port}")
    
    def start_sensors(self):
        """Start all sensor reading threads"""
        self.running = True
        
        for port in self.mark10_sensors.keys():
            thread = threading.Thread(target=self._mark10_reader, args=(port,))
            thread.daemon = True
            thread.start()
            self.sensor_threads[port] = thread
            print(f"Started Mark-10 reader thread for {port}")
    
    def stop_sensors(self):
        """Stop all sensor reading threads"""
        self.running = False
        
        # Wait for all threads to finish
        for thread in self.sensor_threads.values():
            thread.join(timeout=2.0)
        
        print("All sensor threads stopped")
    
    def _mark10_reader(self, port):
        """Dedicated thread function for reading from one Mark-10 sensor"""
        sensor_info = self.mark10_sensors[port]
        target_interval = 1.0 / sensor_info['sampling_rate']
        
        try:
            with serial.Serial(port, baudrate=115200, timeout=0.1) as ser:
                print(f"Mark-10 reader for {port} started")
                next_sample_time = time.time()
                
                while self.running:
                    current_time = time.time()
                    
                    if current_time >= next_sample_time:
                        try:
                            # Read from sensor
                            ser.write("?\r".encode())
                            response = ser.readline().decode('utf-8', errors='ignore').strip()
                            
                            if response:
                                value = self._parse_mark10_response(response)
                                if value is not None:
                                    # Store in thread-safe buffer
                                    with self.data_locks[port]:
                                        sensor_info['buffer'].append((current_time, value))
                                        sensor_info['last_reading'] = (current_time, value)
                                        sensor_info['total_samples'] += 1
                                else:
                                    sensor_info['errors'] += 1
                            
                            next_sample_time += target_interval
                            
                        except Exception as e:
                            sensor_info['errors'] += 1
                            if sensor_info['errors'] % 100 == 0:  # Log every 100 errors
                                print(f"Mark-10 {port} error: {e}")
                    
                    time.sleep(0.001)  # Small sleep to prevent CPU spinning
                    
        except Exception as e:
            print(f"Fatal error in Mark-10 reader {port}: {e}")
    
    def _parse_mark10_response(self, response):
        """Parse Mark-10 response string to extract force value"""
        try:
            # Remove units and extract number
            force_str = response.replace('N', '').replace('lbF', '').replace('lb', '').strip()
            match = re.search(r'-?\d+(\.\d+)?', force_str)
            if match:
                return float(match.group(0))
        except:
            pass
        return None
    
    def get_latest_reading(self, port):
        """Get the most recent reading from a sensor"""
        if port not in self.mark10_sensors:
            return None
            
        with self.data_locks[port]:
            return self.mark10_sensors[port]['last_reading']
    
    def get_current_tension(self, port):
        """Get just the tension value (for pretensioning)"""
        reading = self.get_latest_reading(port)
        if reading:
            return reading[1]  # Return just the value
        return 0.0
    
    def start_data_logging(self, port, output_file, duration):
        """Start logging data from a sensor to CSV file"""
        if port not in self.mark10_sensors:
            print(f"Error: No sensor configured for {port}")
            return False
        
        def logger_thread():
            start_time = time.time()
            logged_data = []
            
            print(f"Starting data logging for {port} to {output_file}")
            
            while time.time() - start_time < duration and self.running:
                # Get all new data since last check
                with self.data_locks[port]:
                    buffer = list(self.mark10_sensors[port]['buffer'])
                
                # Add new data that's within our logging window
                for timestamp, value in buffer:
                    if timestamp >= start_time and (timestamp, value) not in logged_data:
                        logged_data.append((timestamp, value))
                
                time.sleep(0.1)  # Check every 100ms
            
            # Save all logged data
            try:
                with open(output_file, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['timestamp', 'force'])
                    writer.writerows(logged_data)
                
                print(f"Mark-10 {port}: Saved {len(logged_data)} samples to {output_file}")
                return True
                
            except Exception as e:
                print(f"Error saving Mark-10 data for {port}: {e}")
                return False
        
        # Start logger thread
        logger = threading.Thread(target=logger_thread)
        logger.daemon = True
        logger.start()
        self.data_loggers[port] = logger
        
        return True
    
    def get_sensor_stats(self, port):
        """Get statistics about a sensor"""
        if port not in self.mark10_sensors:
            return None
        
        sensor_info = self.mark10_sensors[port]
        with self.data_locks[port]:
            return {
                'total_samples': sensor_info['total_samples'],
                'buffer_size': len(sensor_info['buffer']),
                'errors': sensor_info['errors'],
                'last_reading': sensor_info['last_reading']
            }


# Adapter classes for existing code compatibility
class Mark10SensorAdapter:
    """Adapter to make CentralizedSensorManager compatible with existing pretensioning code"""
    
    def __init__(self, port, sensor_manager):
        self.port = port
        self.sensor_manager = sensor_manager
    
    def read_tension(self):
        """Compatible with existing pretensioning code"""
        return self.sensor_manager.get_current_tension(self.port)


class Mark10DataLogger:
    """Adapter for data logging that uses the centralized manager"""
    
    def __init__(self, port, sensor_manager):
        self.port = port
        self.sensor_manager = sensor_manager
    
    def acquire_data(self, duration, output_file):
        """Compatible with existing acquisition code"""
        return self.sensor_manager.start_data_logging(self.port, output_file, duration)
