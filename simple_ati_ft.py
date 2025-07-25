import nidaqmx
from nidaqmx.constants import AcquisitionType
from nidaqmx.errors import DaqError
import numpy as np
import time
import csv
from datetime import datetime
import pandas as pd
import sys
import os

# Optional imports for plotting (only needed if using ATI_FT_Plotter)
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
    from scipy.signal import butter, filtfilt
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Warning: Plotting dependencies not available. Install pyqtgraph and scipy for plotting functionality.")

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
    
    def acquire_data(self, duration, output_file="ati_data.csv", calculate_bias=True):
        """
        Acquire data for specified duration
        
        Args:
            duration: Acquisition time in seconds
            output_file: Output CSV filename
            calculate_bias: Whether to calculate and subtract bias
            
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
                # Preallocate memory for all_data as a numpy array for efficiency
                num_channels = 6  # Fx, Fy, Fz, Mx, My, Mz
                expected_samples = int(self.sampling_rate * duration)
                # Add extra buffer to prevent index out of bounds
                all_data = np.zeros((expected_samples + 1000, num_channels + 1))  # +1 for timestamp
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
                        
                        # # Store data based on options
                        # if save_raw_voltages:
                        #     # Extended format: [timestamp, V0-V5, Fx, Fy, Fz, Mx, My, Mz]
                        #     data_row = [timestamp] + list(raw_voltages) + list(ft_values_flat)
                        # else:
                        #     # Standard format: [timestamp, Fx, Fy, Fz, Mx, My, Mz]
                        data_row = [timestamp] + list(ft_values_flat)
                        
                        all_data[sample_count, :] = data_row
                        
                        sample_count += 1

                        # Check if we've reached the buffer limit
                        if sample_count >= all_data.shape[0]:
                            print("Warning: Data buffer full, stopping acquisition early")
                            break

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
    
    def save_data(self, data, filename, include_raw_voltages=False):
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
        
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Write header
                # if include_raw_voltages:
                #     header = ['timestamp', 'V0', 'V1', 'V2', 'V3', 'V4', 'V5', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
                # else:
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

class ATI_FT_Plotter:
    """
    Standalone class for interactive plotting of ATI Force/Torque data
    Used for post-processing and analysis of collected data
    """
    
    def __init__(self):
        """Initialize the plotter"""
        if not PLOTTING_AVAILABLE:
            raise ImportError("Plotting dependencies not available. Install pyqtgraph and scipy.")
        
        self.app = None
        self.win = None
        self.data = None
        self.relative_time = None
        self.sampling_rate = None
        
        # Define channel information
        self.force_channels = ['Fx', 'Fy', 'Fz']
        self.torque_channels = ['Mx', 'My', 'Mz']
        self.all_channels = self.force_channels + self.torque_channels
        
        print("ATI F/T Data Plotter initialized")
    
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
    
    def load_data(self, filename):
        """Load CSV data and perform initial analysis"""
        try:
            # Load data
            self.data = pd.read_csv(filename)
            print(f"Loaded {len(self.data)} samples from {filename}")
            print(f"Columns: {list(self.data.columns)}")
            
            # Extract timestamps and convert to relative time
            timestamps = self.data['timestamp'].values
            self.relative_time = timestamps - timestamps[0]
            duration = self.relative_time[-1]
            
            # Estimate sampling rate
            self.sampling_rate, avg_dt = self.estimate_sampling_rate(timestamps)
            
            print(f"Duration: {duration:.2f} seconds")
            print(f"Average sampling interval: {avg_dt*1000:.2f} ms")
            print(f"Estimated sampling rate: {self.sampling_rate:.2f} Hz")
            
            return True
            
        except Exception as e:
            print(f"Error loading data: {e}")
            return False
    
    def recommend_filter_parameters(self):
        """Recommend meaningful filter parameters based on sampling rate"""
        if self.sampling_rate is None:
            print("No data loaded. Load data first.")
            return 10.0
        
        nyquist = self.sampling_rate / 2
        
        print(f"\nRecommended Filter Parameters:")
        print(f"  Sampling rate: {self.sampling_rate:.2f} Hz")
        print(f"  Nyquist frequency: {nyquist:.2f} Hz")
        
        # For force/torque sensors, typical recommendations:
        if self.sampling_rate > 100:
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
        if self.data is None:
            print("No data loaded. Use load_data() first.")
            return None, None
        
        if cutoff_freq is None:
            cutoff_freq = self.recommend_filter_parameters()
        
        # Create PyQtGraph application
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication(sys.argv)
        
        # Create main window
        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle('Interactive ATI F/T Data Viewer')
        self.win.resize(1400, 900)
        
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
        cutoff_spinbox.setRange(0.1, min(self.sampling_rate/2 * 0.8, 100))
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
        
        # Raw data transparency control
        transparency_label = QtWidgets.QLabel("Raw Data Transparency:")
        transparency_dial = QtWidgets.QDial()
        transparency_dial.setRange(0, 100)
        transparency_dial.setValue(30)  # Default 30% transparency (70% alpha)
        transparency_dial.setNotchesVisible(True)
        transparency_dial.setFixedSize(60, 60)
        
        transparency_value_label = QtWidgets.QLabel("30%")
        transparency_value_label.setAlignment(QtCore.Qt.AlignCenter)
        
        visibility_layout.addWidget(raw_checkbox)
        visibility_layout.addWidget(filtered_checkbox)
        visibility_layout.addWidget(transparency_label)
        visibility_layout.addWidget(transparency_dial)
        visibility_layout.addWidget(transparency_value_label)
        
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
        
        # Create plot widget with 2x3 grid
        plot_widget = pg.GraphicsLayoutWidget()
        plot_widget.setBackground('w')  # White background
        
        # Create subplots
        plots = []
        raw_curves = []
        filtered_curves = []
        
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]  # RGB colors
        
        for i, channel in enumerate(self.all_channels):
            row = i // 3
            col = i % 3
            
            # Create plot
            plot = plot_widget.addPlot(row=row, col=col)
            plot.setLabel('left', f'{channel}', units='N' if channel in self.force_channels else 'Nm')
            plot.setLabel('bottom', 'Time', units='s')
            plot.setTitle(f'{"Force" if channel in self.force_channels else "Torque"} {channel}')
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.enableAutoRange()
            
            # Create curves
            color = colors[i % 3]
            # Raw data as shadow: will use dial-controlled transparency
            raw_curve = plot.plot(name='Raw', pen=pg.mkPen(color=(*color, 76), width=1))  # Default 30% transparency
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
            transparency_percent = transparency_dial.value()
            
            # Update transparency value label
            transparency_value_label.setText(f"{transparency_percent}%")
            
            # Calculate alpha value (0-255) from transparency percentage
            # transparency_percent: 0% = fully opaque (alpha=255), 100% = fully transparent (alpha=0)
            alpha = int(255 * (100 - transparency_percent) / 100)
            
            noise_reductions = []
            
            for i, channel in enumerate(self.all_channels):
                # Get data
                raw_data = self.data[channel].values
                
                # Apply filter
                try:
                    filtered_data = self.butterworth_filter(
                        raw_data, current_cutoff, self.sampling_rate, order=current_order
                    )
                except Exception as e:
                    print(f"Filter error for {channel}: {e}")
                    filtered_data = raw_data
                
                # Update curves with new transparency for raw data
                color = colors[i % 3]
                
                if show_raw:
                    # Update raw curve with new transparency
                    raw_pen = pg.mkPen(color=(*color, alpha), width=1)
                    raw_curves[i].setPen(raw_pen)
                    raw_curves[i].setData(self.relative_time, raw_data)
                    raw_curves[i].show()
                else:
                    raw_curves[i].hide()
                
                if show_filtered:
                    filtered_curves[i].setData(self.relative_time, filtered_data)
                    filtered_curves[i].show()
                else:
                    filtered_curves[i].hide()
                
                # Calculate noise reduction
                raw_std = np.std(raw_data)
                filtered_std = np.std(filtered_data)
                noise_reduction = (1 - filtered_std/raw_std) * 100 if raw_std > 0 else 0
                noise_reductions.append(noise_reduction)
            
            # Update statistics
            avg_noise_reduction = np.mean(noise_reductions)
            stats_text = f"Average Noise Reduction: {avg_noise_reduction:.1f}%\n"
            stats_text += f"Cutoff: {current_cutoff:.1f} Hz, Order: {current_order}\n"
            stats_text += f"Data points: {len(self.relative_time)}, Duration: {self.relative_time[-1]:.1f}s"
            stats_label.setText(stats_text)
        
        # Connect signals
        cutoff_spinbox.valueChanged.connect(update_plots)
        order_spinbox.valueChanged.connect(update_plots)
        raw_checkbox.stateChanged.connect(update_plots)
        filtered_checkbox.stateChanged.connect(update_plots)
        transparency_dial.valueChanged.connect(update_plots)
        
        # Initial plot
        update_plots()
        
        # Show window
        self.win.show()
        
        print("\nPyQtGraph Interactive Features:")
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
    
    def plot_data_file(self, filename, cutoff_freq=None, filter_order=4):
        """
        Convenience method to load and plot data in one call
        
        Args:
            filename: CSV file to plot
            cutoff_freq: Filter cutoff frequency (auto-recommended if None)
            filter_order: Filter order (default 4)
        """
        if not self.load_data(filename):
            return False
        
        self.plot_interactive(cutoff_freq, filter_order)
        self.run()
        return True

def main():
    """Main function with menu"""
    print("=" * 60)
    print("Simple ATI Force/Torque Sensor Data Acquisition")
    print("=" * 60)
   
    duration = 10
    rate = 2000
    output = "data/test_data.csv"

    sensor = SimpleATI_FT(sampling_rate=rate)
    success = sensor.acquire_data(duration, output)


    plotting = ATI_FT_Plotter()
    plotting.plot_data_file(output)


if __name__ == "__main__":
    main()
