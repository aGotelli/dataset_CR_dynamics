from vicon_dssdk import ViconDataStream
import numpy as np
import time
import csv
import os
import sys
import json
from datetime import datetime

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from scipy.signal import butter, filtfilt
import pandas as pd

# Optional imports for plotting (only needed if using Vicon_Plotter)
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets
    from scipy.signal import butter, filtfilt
    import pandas as pd
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Warning: Plotting dependencies not available. Install pyqtgraph and scipy for plotting functionality.")

class SimpleVicon:
    """
    Simple Vicon motion capture data acquisition
    Focus on performance with preallocated memory and constant timestep
    """
    
    def __init__(self, host="localhost:801", duration=10.0):
        """
        Initialize the Vicon client and prepare for data acquisition
        
        Args:
            host: Vicon server address (e.g., "localhost:801")
            duration: Expected recording duration for buffer allocation
        """
        self.host = host
        self.duration = duration
        self.is_running = False
        self.client = ViconDataStream.Client()
        
        print(f"Vicon initializing - Host: {host}")
        
        # Connect and setup immediately
        self.connect_and_setup()
        
        # Initialize data buffer based on expected duration
        expected_samples = int(self.duration * self.vicon_frame_rate) + 100  # Buffer
        num_columns = len(self.column_names)
        self.data_buffer = np.zeros((expected_samples, num_columns))
        self.sample_count = 0
        
        print(f"Preallocated memory for {expected_samples} samples x {num_columns} columns")
        print(f"Vicon ready for data acquisition!")
    
    def connect_and_setup(self):
        """Connect to Vicon and configure data streams"""
        print(f"Connecting to {self.host}...")
        self.client.Connect(self.host)
        
        # Configure data streams for maximum performance
        self.client.EnableSegmentData()
        self.client.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
        self.client.SetBufferSize(1)  # Minimize latency
        
        # Wait for first frame
        print("Waiting for data...")
        timeout = 50
        while not self.client.GetFrame() and timeout > 0:
            time.sleep(0.02)
            timeout -= 1
        
        if timeout == 0:
            raise RuntimeError("Failed to get initial frame from Vicon")
        
        # Get system frame rate and setup timing
        self.vicon_frame_rate = self.client.GetFrameRate()
        self.target_dt = 1.0 / self.vicon_frame_rate
        print(f"Vicon system frame rate: {self.vicon_frame_rate:.2f} Hz")
        
        # Build data structure: [timestamp, pos_x, pos_y, pos_z, euler_x, euler_y, euler_z, quality, occluded] per segment
        self.column_names = ['timestamp']
        self.data_structure = []
        
        subject_names = self.client.GetSubjectNames()
        if not subject_names:
            raise RuntimeError("No subjects found in Vicon")
        
        for subject_name in subject_names:
            segment_names = self.client.GetSegmentNames(subject_name)
            print(f"Subject '{subject_name}' has segments: {segment_names}")
            
            for segment_name in segment_names:
                # Use just segment name if it differs from subject, otherwise use subject name
                if segment_name == subject_name:
                    name_prefix = subject_name
                else:
                    name_prefix = f"{subject_name}_{segment_name}"
                
                # Position (X, Y, Z)
                for axis in ['X', 'Y', 'Z']:
                    self.column_names.append(f"{name_prefix}_pos_{axis}")
                    self.data_structure.append((subject_name, segment_name, 'position', axis))
                
                # Euler angles (X, Y, Z)  
                for axis in ['X', 'Y', 'Z']:
                    self.column_names.append(f"{name_prefix}_euler_{axis}")
                    self.data_structure.append((subject_name, segment_name, 'euler', axis))
                
                # Quality metrics
                self.column_names.extend([
                    f"{name_prefix}_quality",
                    f"{name_prefix}_occluded"
                ])
                self.data_structure.extend([
                    (subject_name, segment_name, 'quality', None),
                    (subject_name, segment_name, 'occluded', None)
                ])
        
        print(f"Found {len(subject_names)} subjects with {len(self.column_names)} data columns")
    
    def collect_frame_data(self):
        """Collect data for one frame - optimized for performance, and store pose/supplementary vectors for debugging"""
        # Record timestamp for this frame
        self.data_buffer[self.sample_count, 0] = time.time()

        # --- Per-frame vector for debugging (mimics sample code, but uses euler angles) ---
        timestamp = self.data_buffer[self.sample_count, 0]
        subjectNames = self.client.GetSubjectNames()
        try:
            subjectNames.sort(key=lambda name: int(name[-1]))
        except Exception:
            pass  # fallback if subject names are not numeric
        row_data = [timestamp]
        for subjectName in subjectNames:
            segmentNames = self.client.GetSegmentNames(subjectName)
            for segmentName in segmentNames:
                translation, translation_occluded = self.client.GetSegmentGlobalTranslation(subjectName, segmentName)
                translation_m = [t / 1000.0 for t in translation]  # convert mm to meters
                euler, euler_occluded = self.client.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName)
                subject_index = subjectName[-1] if subjectName else ''
                # row_data: subject_index, translation (m), euler angles (deg), occlusion flags
                row_data.extend([
                    subject_index,
                    translation_m[0], translation_m[1], translation_m[2],
                    euler[0], euler[1], euler[2],
                    int(translation_occluded), int(euler_occluded)
                ])

        # Store per-frame row_data for debugging
        if not hasattr(self, 'all_frame_vectors'):
            self.all_frame_vectors = []
        self.all_frame_vectors.append(row_data)

        # --- Original data buffer logic for main CSV output ---
        frame_poses = []
        frame_supplementary = []
        col_index = 1
        for subject_name, segment_name, data_type, axis in self.data_structure:
            if data_type == 'position':
                translation, occluded = self.client.GetSegmentGlobalTranslation(subject_name, segment_name)
                axis_idx = ['X', 'Y', 'Z'].index(axis)
                self.data_buffer[self.sample_count, col_index] = translation[axis_idx] if not occluded else 0.0
                if axis == 'Z':
                    t, t_occ = self.client.GetSegmentGlobalTranslation(subject_name, segment_name)
                    r, r_occ = self.client.GetSegmentGlobalRotationEulerXYZ(subject_name, segment_name)
                    frame_poses.append({
                        'subject': subject_name,
                        'segment': segment_name,
                        'translation': t,
                        'translation_occluded': t_occ,
                        'euler': r,
                        'euler_occluded': r_occ
                    })
                if self.sample_count < 3 and axis == 'X':
                    print(f"  {subject_name}.{segment_name}: pos={translation}, occluded={occluded}")
            elif data_type == 'euler':
                rotation, occluded = self.client.GetSegmentGlobalRotationEulerXYZ(subject_name, segment_name)
                axis_idx = ['X', 'Y', 'Z'].index(axis)
                self.data_buffer[self.sample_count, col_index] = rotation[axis_idx] if not occluded else 0.0
            elif data_type == 'quality':
                try:
                    quality = self.client.GetObjectQuality(subject_name)
                    self.data_buffer[self.sample_count, col_index] = quality
                    frame_supplementary.append({
                        'subject': subject_name,
                        'segment': segment_name,
                        'quality': quality
                    })
                    if self.sample_count < 3:
                        print(f"  {subject_name} quality: {quality}")
                except Exception as e:
                    self.data_buffer[self.sample_count, col_index] = 0.0
                    if self.sample_count < 3:
                        print(f"  {subject_name} quality error: {e}")
            elif data_type == 'occluded':
                _, occluded = self.client.GetSegmentGlobalTranslation(subject_name, segment_name)
                self.data_buffer[self.sample_count, col_index] = 1.0 if occluded else 0.0
                if frame_supplementary:
                    frame_supplementary[-1]['occluded'] = occluded
            col_index += 1

        if not hasattr(self, 'all_frame_poses'):
            self.all_frame_poses = []
        if not hasattr(self, 'all_frame_supplementary'):
            self.all_frame_supplementary = []
        self.all_frame_poses.append(frame_poses)
        self.all_frame_supplementary.append(frame_supplementary)

        self.sample_count += 1
    
    def acquire_data(self, output_file="vicon_data.csv"):
        """
        Acquire data for the preset duration by capturing frames as soon as available
        
        Args:
            output_file: Output CSV filename
            
        Returns:
            success: True if acquisition completed successfully
        """
        print(f"Starting Vicon acquisition for {self.duration} seconds...")
        print(f"Output file: {output_file}")
        
        # Reset sample count for new acquisition
        self.sample_count = 0
        
        # Timing control
        start_time = time.time()
        
        print("Starting data collection...")
        self.is_running = True
        
        # Main acquisition loop - capture frames as soon as available
        while self.is_running and (time.time() - start_time) < self.duration:
            # Get fresh frame - this is the primary condition
            if self.client.GetFrame():
                # Collect data for this frame
                self.collect_frame_data()
                
                # Progress indicator
                if self.sample_count % (int(self.vicon_frame_rate) * 2) == 0:  # Every 2 seconds
                    elapsed = time.time() - start_time
                    actual_rate = self.sample_count / elapsed if elapsed > 0 else 0
                    print(f"  {elapsed:.1f}s - {self.sample_count} samples - {actual_rate:.1f} Hz")
            else:
                # Small sleep to prevent CPU overload when no frame is available
                # Sleep for a fraction of the frame period
                time.sleep(self.target_dt * 0.1)  # Sleep for 10% of frame period
        
        self.is_running = False
        
        # Trim data to actual samples collected
        final_data = self.data_buffer[:self.sample_count]
        
        # Calculate actual performance
        actual_duration = time.time() - start_time
        actual_rate = self.sample_count / actual_duration
        
        print(f"Collection completed:")
        print(f"  Samples: {self.sample_count}")
        print(f"  Duration: {actual_duration:.2f}s")
        print(f"  Actual rate: {actual_rate:.2f} Hz")
        print(f"  Vicon rate: {self.vicon_frame_rate:.2f} Hz")
        print(f"  Rate efficiency: {(actual_rate/self.vicon_frame_rate)*100:.1f}%")
        
        # Save data
        success = self.save_data(final_data, output_file)
        
        return success
    
    def save_data(self, data, filename):
        """
        Save data to CSV file
        
        Args:
            data: numpy array of data
            filename: Output filename
        """
        if len(data) == 0:
            print("No data to save")
            return False
        
        os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            writer.writerow(self.column_names)
            
            # Write data
            for row in data:
                writer.writerow(row)
        
        # File info
        file_size_kb = os.path.getsize(filename) / 1024
        print(f"Data saved to {filename}")
        print(f"  Rows: {len(data)}")
        print(f"  Columns: {len(self.column_names)}")
        print(f"  File size: {file_size_kb:.1f} KB")
        
        return True
    

    
    def stop(self):
        """Stop data acquisition"""
        self.is_running = False
        print("Stop signal sent")



class ThreadableVicon(SimpleVicon):
    """
    Extended version ready for threading
    """
    
    def __init__(self, host="localhost:801", name="Vicon", duration=10.0):
        super().__init__(host, duration)
        self.name = name
        self.buffer_lock = None  # Will use threading.Lock() when needed
    
    def threaded_acquire(self, output_file="threaded_vicon.csv"):
        """
        Acquisition method ready for threading
        """
        print(f"{self.name}: Starting threaded acquisition")
        success = self.acquire_data(output_file)
        print(f"{self.name}: Threaded acquisition {'completed' if success else 'failed'}")
        return success

class Vicon_Plotter:
    """
    Standalone class for interactive plotting of Vicon motion capture data
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
        
        print("Vicon Motion Capture Data Plotter initialized")
    
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
            print(f"Columns: {len(self.data.columns)}")
            
            # Extract timestamps and convert to relative time
            timestamps = self.data['timestamp'].values
            self.relative_time = timestamps - timestamps[0]
            duration = self.relative_time[-1]
            
            # Estimate sampling rate
            self.sampling_rate, avg_dt = self.estimate_sampling_rate(timestamps)
            
            print(f"Duration: {duration:.2f} seconds")
            print(f"Average sampling interval: {avg_dt*1000:.2f} ms")
            print(f"Estimated sampling rate: {self.sampling_rate:.2f} Hz")
            
            # Identify position and angle columns
            self.position_columns = [col for col in self.data.columns if '_pos_' in col]
            self.euler_columns = [col for col in self.data.columns if '_euler_' in col]
            self.quality_columns = [col for col in self.data.columns if '_quality' in col]
            
            print(f"Position channels: {len(self.position_columns)}")
            print(f"Euler angle channels: {len(self.euler_columns)}")
            print(f"Quality channels: {len(self.quality_columns)}")
            
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
        
        # For motion capture, typical recommendations:
        if self.sampling_rate > 100:
            # High sampling rate - can use higher cutoff
            low_cutoff = min(5, nyquist * 0.05)
            medium_cutoff = min(15, nyquist * 0.15)
            high_cutoff = min(30, nyquist * 0.3)
        else:
            # Lower sampling rate - be more conservative
            low_cutoff = min(2, nyquist * 0.05)
            medium_cutoff = min(8, nyquist * 0.2)
            high_cutoff = min(15, nyquist * 0.3)
        
        print(f"  Low-pass options:")
        print(f"    Conservative: {low_cutoff:.1f} Hz (smooth motion)")
        print(f"    Moderate: {medium_cutoff:.1f} Hz (good balance)")
        print(f"    Aggressive: {high_cutoff:.1f} Hz (preserves dynamics)")
        print(f"  Recommended order: 4 (good balance of rolloff and phase)")
        
        return medium_cutoff
    
    def plot_interactive(self, cutoff_freq=None, filter_order=4):
        """Create an interactive PyQtGraph plot with real-time controls"""
        if self.data is None:
            print("No data loaded. Use load_data() first.")
            return
        
        if cutoff_freq is None:
            cutoff_freq = self.recommend_filter_parameters()
        
        # Create PyQtGraph application
        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication(sys.argv)
        
        # Create main window
        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle('Interactive Vicon Motion Capture Data Viewer')
        self.win.resize(1600, 1000)
        
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
        cutoff_spinbox.setRange(0.1, min(self.sampling_rate/2 * 0.8, 50))
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
        
        # Data type selection
        show_position_checkbox = QtWidgets.QCheckBox("Show Positions")
        show_position_checkbox.setChecked(True)
        show_euler_checkbox = QtWidgets.QCheckBox("Show Euler Angles")
        show_euler_checkbox.setChecked(True)
        
        visibility_layout.addWidget(raw_checkbox)
        visibility_layout.addWidget(filtered_checkbox)
        visibility_layout.addWidget(show_position_checkbox)
        visibility_layout.addWidget(show_euler_checkbox)
        
        # Statistics display
        stats_group = QtWidgets.QGroupBox("Statistics")
        stats_layout = QtWidgets.QVBoxLayout(stats_group)
        stats_label = QtWidgets.QLabel("Motion analysis info will appear here")
        stats_layout.addWidget(stats_label)
        
        # Add groups to control layout
        control_layout.addWidget(filter_group)
        control_layout.addWidget(visibility_group)
        control_layout.addWidget(stats_group)
        control_layout.addStretch()
        
        # Create plot widget with 2x2 grid (positions and angles)
        plot_widget = pg.GraphicsLayoutWidget()
        plot_widget.setBackground('w')  # White background
        
        # Create subplots
        plots = {'pos_X': None, 'pos_Y': None, 'pos_Z': None, 'euler_X': None, 'euler_Y': None, 'euler_Z': None}
        raw_curves = {}
        filtered_curves = {}
        
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]  # RGB colors
        
        # Position plots (top row)
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plot = plot_widget.addPlot(row=0, col=i)
            plot.setLabel('left', f'Position {axis}', units='mm')
            plot.setLabel('bottom', 'Time', units='s')
            plot.setTitle(f'Position {axis}')
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.enableAutoRange()
            
            color = colors[i]
            shadow_color = (*color, 80)
            raw_curves[f'pos_{axis}'] = plot.plot(name='Raw', pen=pg.mkPen(color=shadow_color, width=1))
            filtered_curves[f'pos_{axis}'] = plot.plot(name='Filtered', pen=pg.mkPen(color=color, width=2))
            plots[f'pos_{axis}'] = plot
        
        # Euler angle plots (bottom row)
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plot = plot_widget.addPlot(row=1, col=i)
            plot.setLabel('left', f'Euler {axis}', units='deg')
            plot.setLabel('bottom', 'Time', units='s')
            plot.setTitle(f'Euler Angle {axis}')
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.enableAutoRange()
            
            color = colors[i]
            shadow_color = (*color, 80)
            raw_curves[f'euler_{axis}'] = plot.plot(name='Raw', pen=pg.mkPen(color=shadow_color, width=1))
            filtered_curves[f'euler_{axis}'] = plot.plot(name='Filtered', pen=pg.mkPen(color=color, width=2))
            plots[f'euler_{axis}'] = plot
        
        # Add widgets to main layout
        layout.addWidget(control_panel)
        layout.addWidget(plot_widget)
        
        # Function to update plots
        def update_plots():
            current_cutoff = cutoff_spinbox.value()
            current_order = order_spinbox.value()
            show_raw = raw_checkbox.isChecked()
            show_filtered = filtered_checkbox.isChecked()
            show_positions = show_position_checkbox.isChecked()
            show_euler = show_euler_checkbox.isChecked()

            motion_stats = []

            # Only plot the last marker (last set of columns for each axis)
            for axis in ['X', 'Y', 'Z']:
                plot_key = f'pos_{axis}'
                if show_positions and self.position_columns:
                    axis_columns = [col for col in self.position_columns if f'_pos_{axis}' in col]
                    if axis_columns:
                        # Use last available column for this axis (last marker)
                        raw_data = self.data[axis_columns[-1]].values
                        filtered_data = self.butterworth_filter(raw_data, current_cutoff, self.sampling_rate, current_order)
                        raw_range = np.ptp(raw_data)
                        filtered_range = np.ptp(filtered_data)
                        noise_reduction = ((raw_range - filtered_range) / raw_range * 100) if raw_range > 0 else 0
                        motion_stats.append(f"Pos {axis}: {noise_reduction:.1f}% noise reduction")
                        if show_raw:
                            raw_curves[plot_key].setData(self.relative_time, raw_data)
                        else:
                            raw_curves[plot_key].setData([], [])
                        if show_filtered:
                            filtered_curves[plot_key].setData(self.relative_time, filtered_data)
                        else:
                            filtered_curves[plot_key].setData([], [])
                        plots[plot_key].setVisible(True)
                    else:
                        plots[plot_key].setVisible(False)
                else:
                    plots[plot_key].setVisible(False)
                    raw_curves[plot_key].setData([], [])
                    filtered_curves[plot_key].setData([], [])

            for axis in ['X', 'Y', 'Z']:
                plot_key = f'euler_{axis}'
                if show_euler and self.euler_columns:
                    axis_columns = [col for col in self.euler_columns if f'_euler_{axis}' in col]
                    if axis_columns:
                        # Use last available column for this axis (last marker)
                        raw_data = self.data[axis_columns[-1]].values
                        filtered_data = self.butterworth_filter(raw_data, current_cutoff, self.sampling_rate, current_order)
                        raw_range = np.ptp(raw_data)
                        filtered_range = np.ptp(filtered_data)
                        noise_reduction = ((raw_range - filtered_range) / raw_range * 100) if raw_range > 0 else 0
                        motion_stats.append(f"Euler {axis}: {noise_reduction:.1f}% noise reduction")
                        if show_raw:
                            raw_curves[plot_key].setData(self.relative_time, raw_data)
                        else:
                            raw_curves[plot_key].setData([], [])
                        if show_filtered:
                            filtered_curves[plot_key].setData(self.relative_time, filtered_data)
                        else:
                            filtered_curves[plot_key].setData([], [])
                        plots[plot_key].setVisible(True)
                    else:
                        plots[plot_key].setVisible(False)
                else:
                    plots[plot_key].setVisible(False)
                    raw_curves[plot_key].setData([], [])
                    filtered_curves[plot_key].setData([], [])

            avg_noise_reduction = np.mean([float(s.split(':')[1].split('%')[0]) for s in motion_stats]) if motion_stats else 0
            stats_text = f"Average Noise Reduction: {avg_noise_reduction:.1f}%\n"
            stats_text += f"Cutoff: {current_cutoff:.1f} Hz, Order: {current_order}\n"
            stats_text += f"Data points: {len(self.relative_time)}, Duration: {self.relative_time[-1]:.1f}s\n"
            stats_text += f"Subjects: 1 (last marker only)"
            stats_label.setText(stats_text)
        
        # Connect signals
        cutoff_spinbox.valueChanged.connect(update_plots)
        order_spinbox.valueChanged.connect(update_plots)
        raw_checkbox.stateChanged.connect(update_plots)
        filtered_checkbox.stateChanged.connect(update_plots)
        show_position_checkbox.stateChanged.connect(update_plots)
        show_euler_checkbox.stateChanged.connect(update_plots)
        
        # Initial plot
        update_plots()
        
        # Show window
        self.win.show()
        
        print("\nPyQtGraph Interactive Features:")
        print("- Real-time filter adjustment with spinboxes")
        print("- Toggle raw/filtered data visibility")
        print("- Toggle position/angle data types")
        print("- Zoom: Mouse wheel or right-click drag")
        print("- Pan: Left-click drag")
        print("- Auto-range: Right-click -> View All")
        print("- Export: Right-click -> Export...")
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
    """Main function with usage examples"""
    print("=" * 60)
    print("Simple Vicon Motion Capture Data Acquisition")
    print("=" * 60)
    
    # Configuration
    duration = 10  # seconds
    output = "data/vicon_data.csv"
    host = "localhost:801"  # Change to your Vicon server
    
    # Data acquisition
    vicon = SimpleVicon(host=host, duration=duration)
    success = vicon.acquire_data(output)
    
    if success:
        print("\nData acquisition completed successfully!")
        
        # Plot data if plotting is available
        if PLOTTING_AVAILABLE:
            print("\nStarting interactive plotter...")
            plotter = Vicon_Plotter()
            plotter.plot_data_file(output)
        else:
            print("Install pyqtgraph and scipy for interactive plotting")
    else:
        print("Data acquisition failed!")

if __name__ == "__main__":
    main()
