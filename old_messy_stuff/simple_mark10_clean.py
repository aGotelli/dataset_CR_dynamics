import serial
import time
import numpy as np
import csv
import pandas as pd
import sys
import os

# Optional imports for plotting (only needed if using Mark10_Plotter)
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
    from scipy.signal import butter, filtfilt
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Warning: Plotting dependencies not available. Install pyqtgraph and scipy for plotting functionality.")

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

class Mark10_Plotter:
    """
    Standalone class for interactive plotting of Mark-10 Force data
    Used for post-processing and analysis of collected data from single or multiple sensors
    """
    
    def __init__(self):
        """Initialize the plotter"""
        if not PLOTTING_AVAILABLE:
            raise ImportError("Plotting dependencies not available. Install pyqtgraph and scipy.")
        
        self.app = None
        self.win = None
        self.data_files = []  # List of loaded data files
        self.relative_times = []  # List of relative times for each sensor
        self.sampling_rates = []  # List of sampling rates for each sensor
        
        print("Mark-10 Multi-Sensor Data Plotter initialized")
    
    def estimate_sampling_rate(self, timestamps):
        """Estimate sampling rate from timestamps"""
        dt = np.diff(timestamps)
        avg_dt = np.mean(dt)
        sampling_rate = 1.0 / avg_dt
        return sampling_rate, avg_dt
    
    def butterworth_filter(self, data, cutoff_freq, sampling_rate, order=4, filter_type='low'):
        """Apply Butterworth filter using filtfilt (zero-phase filtering)"""
        nyquist = sampling_rate / 2.0
        normal_cutoff = cutoff_freq / nyquist
        
        # Design the Butterworth filter
        b, a = butter(order, normal_cutoff, btype=filter_type, analog=False)
        
        # Apply zero-phase filtering
        filtered_data = filtfilt(b, a, data)
        
        return filtered_data
    
    def load_data(self, filenames):
        """
        Load CSV data from single file or multiple files
        
        Args:
            filenames: Single filename string or list of filenames
            
        Returns:
            success: True if data loaded successfully
        """
        # Convert single filename to list
        if isinstance(filenames, str):
            filenames = [filenames]
        
        self.data_files = []
        self.relative_times = []
        self.sampling_rates = []
        
        try:
            for i, filename in enumerate(filenames):
                # Load data
                data = pd.read_csv(filename)
                print(f"Sensor {i+1}: Loaded {len(data)} samples from {filename}")
                print(f"  Columns: {list(data.columns)}")
                
                # Extract timestamps and convert to relative time
                timestamps = data['timestamp'].values
                relative_time = timestamps - timestamps[0]
                duration = relative_time[-1]
                
                # Estimate sampling rate
                sampling_rate, avg_dt = self.estimate_sampling_rate(timestamps)
                
                print(f"  Duration: {duration:.2f} seconds")
                print(f"  Average sampling interval: {avg_dt*1000:.2f} ms")
                print(f"  Estimated sampling rate: {sampling_rate:.2f} Hz")
                
                # Store data
                self.data_files.append(data)
                self.relative_times.append(relative_time)
                self.sampling_rates.append(sampling_rate)
            
            print(f"\nLoaded data from {len(self.data_files)} sensor(s)")
            return True
            
        except Exception as e:
            print(f"Error loading data: {e}")
            return False
    
    def recommend_filter_parameters(self):
        """Recommend meaningful filter parameters based on sampling rate"""
        if not self.sampling_rates:
            print("No data loaded. Load data first.")
            return 10.0
        
        # Use the minimum sampling rate for conservative filtering
        min_sampling_rate = min(self.sampling_rates)
        nyquist = min_sampling_rate / 2
        
        print(f"\nRecommended Filter Parameters:")
        print(f"  Sampling rates: {[f'{sr:.1f}' for sr in self.sampling_rates]} Hz")
        print(f"  Min sampling rate: {min_sampling_rate:.2f} Hz (used for filter design)")
        print(f"  Nyquist frequency: {nyquist:.2f} Hz")
        
        # For force sensors, typical recommendations:
        if min_sampling_rate > 100:
            # High sampling rate - can use higher cutoff
            low_cutoff = min(10, nyquist * 0.1)
            medium_cutoff = min(25, nyquist * 0.25)
            high_cutoff = min(50, nyquist * 0.4)
        else:
            # Lower sampling rate - be more conservative
            low_cutoff = min(5, nyquist * 0.1)
            medium_cutoff = min(15, nyquist * 0.3)
            high_cutoff = min(25, nyquist * 0.4)
        
        print(f"  Low-pass options:")
        print(f"    Conservative: {low_cutoff:.1f} Hz (removes high-freq noise)")
        print(f"    Moderate: {medium_cutoff:.1f} Hz (good balance)")
        print(f"    Aggressive: {high_cutoff:.1f} Hz (preserves more signal)")
        print(f"  Recommended order: 4 (good balance of rolloff and phase)")
        
        return medium_cutoff
    
    def plot_interactive(self, cutoff_freq=None, filter_order=4):
        """Create an interactive PyQtGraph plot with real-time controls"""
        if not self.data_files:
            print("No data loaded. Use load_data() first.")
            return None, None
        
        if cutoff_freq is None:
            cutoff_freq = self.recommend_filter_parameters()
        
        num_sensors = len(self.data_files)
        
        # Create PyQtGraph application
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication(sys.argv)
        
        # Create main window
        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle(f'Interactive Mark-10 Force Data Viewer ({num_sensors} Sensors)')
        self.win.resize(1400, 800)
        
        # Create central widget and layout
        central_widget = QtWidgets.QWidget()
        self.win.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)
        
        # Create control panel
        control_panel = QtWidgets.QWidget()
        control_layout = QtWidgets.QHBoxLayout(control_panel)
        
        # Filter controls
        filter_group = QtWidgets.QGroupBox("Filter Controls")
        filter_layout = QtWidgets.QGridLayout(filter_group)
        
        # Cutoff frequency control
        cutoff_label = QtWidgets.QLabel("Cutoff Freq (Hz):")
        cutoff_spinbox = QtWidgets.QDoubleSpinBox()
        min_sampling_rate = min(self.sampling_rates)
        cutoff_spinbox.setRange(0.1, min(min_sampling_rate/2 * 0.8, 100))
        cutoff_spinbox.setValue(cutoff_freq)
        cutoff_spinbox.setDecimals(1)
        cutoff_spinbox.setSingleStep(0.5)
        
        # Filter order control
        order_label = QtWidgets.QLabel("Filter Order:")
        order_spinbox = QtWidgets.QSpinBox()
        order_spinbox.setRange(1, 10)
        order_spinbox.setValue(filter_order)
        
        filter_layout.addWidget(cutoff_label, 0, 0)
        filter_layout.addWidget(cutoff_spinbox, 0, 1)
        filter_layout.addWidget(order_label, 1, 0)
        filter_layout.addWidget(order_spinbox, 1, 1)
        
        # Visibility controls
        visibility_group = QtWidgets.QGroupBox("Visibility")
        visibility_layout = QtWidgets.QVBoxLayout(visibility_group)
        
        raw_checkbox = QtWidgets.QCheckBox("Show Raw Data")
        raw_checkbox.setChecked(True)
        filtered_checkbox = QtWidgets.QCheckBox("Show Filtered Data")
        filtered_checkbox.setChecked(True)
        
        visibility_layout.addWidget(raw_checkbox)
        visibility_layout.addWidget(filtered_checkbox)
        
        # Statistics display
        stats_group = QtWidgets.QGroupBox("Statistics")
        stats_layout = QtWidgets.QVBoxLayout(stats_group)
        stats_label = QtWidgets.QLabel("Noise reduction info will appear here")
        stats_layout.addWidget(stats_label)
        
        # Add groups to control layout
        control_layout.addWidget(filter_group)
        control_layout.addWidget(visibility_group)
        control_layout.addWidget(stats_group)
        control_layout.addStretch()
        
        # Create plot widget - multiple plots for multiple sensors
        plot_widget = pg.GraphicsLayoutWidget()
        plot_widget.setBackground('w')  # White background
        
        # Create plots for each sensor
        plots = []
        raw_curves = []
        filtered_curves = []
        
        # Define colors for different sensors
        colors = [
            (255, 0, 0),    # Red
            (0, 0, 255),    # Blue
            (0, 128, 0),    # Green
            (255, 165, 0),  # Orange
            (128, 0, 128),  # Purple
            (255, 192, 203) # Pink
        ]
        
        # Determine grid layout
        if num_sensors == 1:
            rows, cols = 1, 1
        elif num_sensors == 2:
            rows, cols = 1, 2
        elif num_sensors <= 4:
            rows, cols = 2, 2
        elif num_sensors <= 6:
            rows, cols = 2, 3
        else:
            rows, cols = 3, 3
        
        for i in range(num_sensors):
            row = i // cols
            col = i % cols
            
            # Create plot
            plot = plot_widget.addPlot(row=row, col=col)
            plot.setLabel('left', 'Force', units='N')
            plot.setLabel('bottom', 'Time', units='s')
            plot.setTitle(f'Mark-10 Sensor {i+1}')
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.enableAutoRange()
            
            # Create curves
            color = colors[i % len(colors)]
            # Raw data as shadow: lighter color, thinner line
            shadow_color = (*color, 80)  # Add alpha for transparency
            raw_curve = plot.plot(name='Raw', pen=pg.mkPen(color=shadow_color, width=1))
            # Filtered data: solid color, thicker line
            filtered_curve = plot.plot(name='Filtered', pen=pg.mkPen(color=color, width=2))
            
            plots.append(plot)
            raw_curves.append(raw_curve)
            filtered_curves.append(filtered_curve)
        
        # Add widgets to main layout
        layout.addWidget(control_panel)
        layout.addWidget(plot_widget)
        
        # Function to update plots
        def update_plots():
            current_cutoff = cutoff_spinbox.value()
            current_order = order_spinbox.value()
            show_raw = raw_checkbox.isChecked()
            show_filtered = filtered_checkbox.isChecked()
            
            noise_reductions = []
            sensor_stats = []
            
            for i in range(num_sensors):
                # Get force data
                raw_data = self.data_files[i]['force'].values
                relative_time = self.relative_times[i]
                sampling_rate = self.sampling_rates[i]
                
                # Apply filter
                try:
                    filtered_data = self.butterworth_filter(
                        raw_data, current_cutoff, sampling_rate, order=current_order
                    )
                except Exception as e:
                    print(f"Filter error for sensor {i+1}: {e}")
                    filtered_data = raw_data
                
                # Update curves
                if show_raw:
                    raw_curves[i].setData(relative_time, raw_data)
                    raw_curves[i].show()
                else:
                    raw_curves[i].hide()
                
                if show_filtered:
                    filtered_curves[i].setData(relative_time, filtered_data)
                    filtered_curves[i].show()
                else:
                    filtered_curves[i].hide()
                
                # Calculate noise reduction
                raw_std = np.std(raw_data)
                filtered_std = np.std(filtered_data)
                noise_reduction = (1 - filtered_std/raw_std) * 100 if raw_std > 0 else 0
                noise_reductions.append(noise_reduction)
                
                sensor_stats.append({
                    'raw_std': raw_std,
                    'filtered_std': filtered_std,
                    'noise_reduction': noise_reduction,
                    'samples': len(raw_data),
                    'duration': relative_time[-1]
                })
            
            # Update statistics
            avg_noise_reduction = np.mean(noise_reductions)
            stats_text = f"Multi-Sensor Statistics:\n"
            stats_text += f"Average Noise Reduction: {avg_noise_reduction:.1f}%\n"
            stats_text += f"Cutoff: {current_cutoff:.1f} Hz, Order: {current_order}\n\n"
            
            for i, stats in enumerate(sensor_stats):
                stats_text += f"Sensor {i+1}: {stats['noise_reduction']:.1f}% reduction\n"
                stats_text += f"  Raw/Filt STD: {stats['raw_std']:.3f}/{stats['filtered_std']:.3f} N\n"
            
            stats_label.setText(stats_text)
        
        # Connect signals
        cutoff_spinbox.valueChanged.connect(update_plots)
        order_spinbox.valueChanged.connect(update_plots)
        raw_checkbox.stateChanged.connect(update_plots)
        filtered_checkbox.stateChanged.connect(update_plots)
        
        # Initial plot
        update_plots()
        
        # Show window
        self.win.show()
        
        print(f"\nPyQtGraph Interactive Features ({num_sensors} sensors):")
        print("- Real-time filter adjustment with spinboxes")
        print("- Toggle raw/filtered data visibility with checkboxes")
        print("- Zoom: Mouse wheel or right-click drag")
        print("- Pan: Left-click drag")
        print("- Auto-range: Right-click -> View All")
        print("- Export: Right-click -> Export...")
        print("- Each plot has independent zoom/pan")
        print("- Close the window to exit")
        
        return self.app, self.win
    
    def run(self):
        """Run the application event loop"""
        if self.app is None:
            print("No application created. Use plot_interactive() first.")
            return
        
        # Run the application
        if hasattr(self.app, 'exec'):
            self.app.exec()  # PyQt6/PySide6
        else:
            self.app.exec_()  # PyQt5/PySide2
    
    def plot_data_file(self, filenames, cutoff_freq=None, filter_order=4):
        """
        Convenience method to load and plot data in one call
        
        Args:
            filenames: Single CSV filename or list of CSV filenames
            cutoff_freq: Filter cutoff frequency (auto-recommended if None)
            filter_order: Filter order (default 4)
        """
        if not self.load_data(filenames):
            return False
        
        self.plot_interactive(cutoff_freq, filter_order)
        self.run()
        return True
    
    def plot_multi_sensor_directory(self, data_dir="data", pattern="mark10_*.csv", cutoff_freq=None, filter_order=4):
        """
        Convenience method to load and plot all Mark-10 files from a directory
        
        Args:
            data_dir: Directory containing CSV files
            pattern: File pattern to match (default: "mark10_*.csv")
            cutoff_freq: Filter cutoff frequency (auto-recommended if None)
            filter_order: Filter order (default 4)
        """
        import glob
        
        # Find all matching files
        file_pattern = os.path.join(data_dir, pattern)
        filenames = sorted(glob.glob(file_pattern))
        
        if not filenames:
            print(f"No files found matching pattern: {file_pattern}")
            return False
        
        print(f"Found {len(filenames)} files matching pattern:")
        for filename in filenames:
            print(f"  {filename}")
        
        return self.plot_data_file(filenames, cutoff_freq, filter_order)

def main():
    """Main function with usage examples"""
    print("=" * 60)
    print("Mark-10 Force Gauge Data Acquisition")
    print("=" * 60)
    
    # Example 1: Single sensor
    # sensor = SimpleMark10("COM5", sampling_rate=350)
    # success = sensor.acquire_data(duration=10, output_file="data/single_mark10.csv")
    # 
    # # Plot single sensor data
    # if success and PLOTTING_AVAILABLE:
    #     plotter = Mark10_Plotter()
    #     plotter.plot_data_file("data/single_mark10.csv")
    
    # Example 2: Multiple sensors with synchronized acquisition and plotting
    com_ports = ["COM4", "COM5"]  # Add "COM6", "COM7" for 4 sensors
    
    multi_sensor = MultiMark10(com_ports)
    multi_sensor.acquire_synchronized_data(duration=10, output_dir="data")
    
    # Plot all sensor data together
    if PLOTTING_AVAILABLE:
        plotter = Mark10_Plotter()
        # Option A: Plot from directory (automatic file discovery)
        plotter.plot_multi_sensor_directory(data_dir="data", pattern="mark10_*.csv")
        
        # Option B: Plot specific files
        # sensor_files = ["data/mark10_1.csv", "data/mark10_2.csv"]
        # plotter.plot_data_file(sensor_files)

if __name__ == "__main__":
    main()
