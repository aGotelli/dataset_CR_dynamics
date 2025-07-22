import numpy as np
import pandas as pd
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from scipy.signal import butter, filtfilt
import argparse
import os
import sys

# Set up PyQtGraph configuration
pg.setConfigOptions(antialias=True)
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

def estimate_sampling_rate(timestamps):
    """
    Estimate sampling rate from timestamps
    """
    dt = np.diff(timestamps)
    avg_dt = np.mean(dt)
    sampling_rate = 1.0 / avg_dt
    return sampling_rate, avg_dt

def butterworth_filter(data, cutoff_freq, sampling_rate, order=4, filter_type='low'):
    """
    Apply Butterworth filter using filtfilt (zero-phase filtering like MATLAB)
    """
    nyquist = sampling_rate / 2.0
    normal_cutoff = cutoff_freq / nyquist
    
    # Design the Butterworth filter
    b, a = butter(order, normal_cutoff, btype=filter_type, analog=False)
    
    # Apply zero-phase filtering (equivalent to MATLAB's filtfilt)
    filtered_data = filtfilt(b, a, data)
    
    return filtered_data

def load_and_analyze_data(filename):
    """
    Load CSV data and perform initial analysis
    """
    if not os.path.exists(filename):
        raise FileNotFoundError(f"File '{filename}' not found")
    
    # Load data
    data = pd.read_csv(filename)
    print(f"Loaded {len(data)} samples")
    print(f"Columns: {list(data.columns)}")
    
    # Extract timestamps and convert to relative time
    timestamps = data['timestamp'].values
    relative_time = timestamps - timestamps[0]
    duration = relative_time[-1]
    
    # Estimate sampling rate
    sampling_rate, avg_dt = estimate_sampling_rate(timestamps)
    
    print(f"Duration: {duration:.2f} seconds")
    print(f"Average sampling interval: {avg_dt*1000:.2f} ms")
    print(f"Estimated sampling rate: {sampling_rate:.2f} Hz")
    
    return data, relative_time, sampling_rate

class SixDoFPlotter(QtGui.QMainWindow):
    def __init__(self, data, relative_time, sampling_rate, cutoff_freq=10.0, filter_order=4):
        super().__init__()
        
        self.data = data
        self.relative_time = relative_time
        self.sampling_rate = sampling_rate
        self.cutoff_freq = cutoff_freq
        self.filter_order = filter_order
        
        self.setup_ui()
        self.plot_data()
        
    def setup_ui(self):
        """Set up the user interface"""
        self.setWindowTitle('6DoF Force/Torque Data Viewer')
        self.setGeometry(100, 100, 1400, 900)
        
        # Create central widget and layout
        central_widget = QtGui.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtGui.QVBoxLayout(central_widget)
        
        # Add control panel
        control_panel = self.create_control_panel()
        layout.addWidget(control_panel)
        
        # Create graphics layout widget
        self.graphics_widget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics_widget)
        
        # Create plots
        self.create_plots()
        
    def create_control_panel(self):
        """Create control panel with filter settings"""
        panel = QtGui.QWidget()
        layout = QtGui.QHBoxLayout(panel)
        
        # Filter controls
        layout.addWidget(QtGui.QLabel("Cutoff Freq (Hz):"))
        self.cutoff_spinbox = QtGui.QDoubleSpinBox()
        self.cutoff_spinbox.setRange(0.1, self.sampling_rate/2)
        self.cutoff_spinbox.setValue(self.cutoff_freq)
        self.cutoff_spinbox.setSingleStep(0.5)
        self.cutoff_spinbox.valueChanged.connect(self.update_filter)
        layout.addWidget(self.cutoff_spinbox)
        
        layout.addWidget(QtGui.QLabel("Filter Order:"))
        self.order_spinbox = QtGui.QSpinBox()
        self.order_spinbox.setRange(1, 10)
        self.order_spinbox.setValue(self.filter_order)
        self.order_spinbox.valueChanged.connect(self.update_filter)
        layout.addWidget(self.order_spinbox)
        
        # Toggle buttons
        self.show_raw_btn = QtGui.QCheckBox("Show Raw Data")
        self.show_raw_btn.setChecked(True)
        self.show_raw_btn.stateChanged.connect(self.toggle_raw_data)
        layout.addWidget(self.show_raw_btn)
        
        self.show_filtered_btn = QtGui.QCheckBox("Show Filtered Data")
        self.show_filtered_btn.setChecked(True)
        self.show_filtered_btn.stateChanged.connect(self.toggle_filtered_data)
        layout.addWidget(self.show_filtered_btn)
        
        # Statistics label
        self.stats_label = QtGui.QLabel()
        layout.addWidget(self.stats_label)
        
        layout.addStretch()
        return panel
        
    def create_plots(self):
        """Create the 6 subplot layout"""
        # Define channel information
        self.channels = [
            ('Fx', 'Force X (N)', 'red'),
            ('Fy', 'Force Y (N)', 'green'), 
            ('Fz', 'Force Z (N)', 'blue'),
            ('Mx', 'Torque X (Nm)', 'red'),
            ('My', 'Torque Y (Nm)', 'green'),
            ('Mz', 'Torque Z (Nm)', 'blue')
        ]
        
        self.plots = []
        self.raw_curves = []
        self.filtered_curves = []
        
        # Create 2x3 grid of plots
        for i, (channel, ylabel, color) in enumerate(self.channels):
            row = i // 3
            col = i % 3
            
            # Create plot
            plot = self.graphics_widget.addPlot(row=row, col=col, title=channel)
            plot.setLabel('left', ylabel)
            plot.setLabel('bottom', 'Time (s)')
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend()
            
            # Add crosshair
            crosshair_v = pg.InfiniteLine(angle=90, movable=False, pen='k')
            crosshair_h = pg.InfiniteLine(angle=0, movable=False, pen='k')
            plot.addItem(crosshair_v, ignoreBounds=True)
            plot.addItem(crosshair_h, ignoreBounds=True)
            
            # Connect mouse events for crosshair
            plot.scene().sigMouseMoved.connect(
                lambda pos, p=plot, v=crosshair_v, h=crosshair_h: self.update_crosshair(pos, p, v, h)
            )
            
            self.plots.append(plot)
            
        # Link all x-axes for synchronized zooming
        for i in range(1, len(self.plots)):
            self.plots[i].setXLink(self.plots[0])
            
    def update_crosshair(self, pos, plot, crosshair_v, crosshair_h):
        """Update crosshair position and show values"""
        if plot.sceneBoundingRect().contains(pos):
            mouse_point = plot.vb.mapSceneToView(pos)
            crosshair_v.setPos(mouse_point.x())
            crosshair_h.setPos(mouse_point.y())
            
            # Update status with current values
            time_val = mouse_point.x()
            y_val = mouse_point.y()
            self.stats_label.setText(f"Time: {time_val:.3f}s, Value: {y_val:.4f}")
            
    def plot_data(self):
        """Plot all channels with raw and filtered data"""
        for i, (channel, _, color) in enumerate(self.channels):
            plot = self.plots[i]
            
            # Get raw data
            raw_data = self.data[channel].values
            
            # Apply filter
            filtered_data = butterworth_filter(
                raw_data, self.cutoff_freq, self.sampling_rate, order=self.filter_order
            )
            
            # Clear existing curves
            plot.clear()
            
            # Plot raw data
            raw_curve = plot.plot(
                self.relative_time, raw_data, 
                pen=pg.mkPen(color=color, width=1, style=QtCore.Qt.DotLine, alpha=150),
                name='Raw'
            )
            
            # Plot filtered data
            filtered_curve = plot.plot(
                self.relative_time, filtered_data,
                pen=pg.mkPen(color=color, width=2),
                name='Filtered'
            )
            
            self.raw_curves.append(raw_curve)
            self.filtered_curves.append(filtered_curve)
            
            # Calculate and display statistics
            raw_std = np.std(raw_data)
            filtered_std = np.std(filtered_data)
            noise_reduction = (1 - filtered_std/raw_std) * 100
            
            # Add text item for statistics
            text_item = pg.TextItem(
                f'Noise reduction: {noise_reduction:.1f}%',
                anchor=(0, 1), border='k', fill='w'
            )
            text_item.setPos(self.relative_time[len(self.relative_time)//20], 
                           np.max(raw_data) * 0.9)
            plot.addItem(text_item)
            
    def update_filter(self):
        """Update filter parameters and replot"""
        self.cutoff_freq = self.cutoff_spinbox.value()
        self.filter_order = self.order_spinbox.value()
        self.plot_data()
        
    def toggle_raw_data(self, state):
        """Toggle visibility of raw data"""
        for curve in self.raw_curves:
            curve.setVisible(state == QtCore.Qt.Checked)
            
    def toggle_filtered_data(self, state):
        """Toggle visibility of filtered data"""
        for curve in self.filtered_curves:
            curve.setVisible(state == QtCore.Qt.Checked)

def analyze_frequency_content_pyqt(data, sampling_rate, channel='Fx'):
    """
    Analyze frequency content using PyQtGraph
    """
    signal = data[channel].values
    
    # Compute FFT
    fft = np.fft.fft(signal)
    freqs = np.fft.fftfreq(len(signal), 1/sampling_rate)
    
    # Only positive frequencies
    positive_freqs = freqs[:len(freqs)//2]
    magnitude = np.abs(fft[:len(fft)//2])
    power = magnitude**2
    
    # Create frequency analysis window
    app = QtGui.QApplication.instance()
    if app is None:
        app = QtGui.QApplication(sys.argv)
        
    win = pg.GraphicsLayoutWidget(show=True, title=f"Frequency Analysis - {channel}")
    win.resize(800, 600)
    
    # Magnitude plot
    p1 = win.addPlot(title="Frequency Spectrum")
    p1.plot(positive_freqs, magnitude, pen='b')
    p1.setLabel('left', 'Magnitude')
    p1.setLabel('bottom', 'Frequency (Hz)')
    p1.setXRange(0, min(100, sampling_rate/2))
    p1.showGrid(x=True, y=True)
    
    # Power spectral density plot
    win.nextRow()
    p2 = win.addPlot(title="Power Spectral Density")
    p2.plot(positive_freqs, power, pen='r')
    p2.setLabel('left', 'Power')
    p2.setLabel('bottom', 'Frequency (Hz)')
    p2.setLogMode(False, True)  # Log scale on y-axis
    p2.setXRange(0, min(100, sampling_rate/2))
    p2.showGrid(x=True, y=True)
    
    # Find and print dominant frequencies
    peak_indices = np.argsort(magnitude)[-10:]  # Top 10 peaks
    dominant_freqs = positive_freqs[peak_indices]
    print(f"\nDominant frequencies in {channel}:")
    for freq in sorted(dominant_freqs[dominant_freqs > 0]):
        print(f"  {freq:.2f} Hz")
    
    return win

def main():
    parser = argparse.ArgumentParser(description="Interactive 6DoF force/torque data viewer with PyQtGraph")
    parser.add_argument('filename', type=str, nargs='?', default='test_data.csv',
                       help="CSV file to plot (default: test_data.csv)")
    parser.add_argument('--cutoff', type=float, default=10.0,
                       help="Initial cutoff frequency for Butterworth filter in Hz (default: 10.0)")
    parser.add_argument('--order', type=int, default=4,
                       help="Initial filter order (default: 4)")
    parser.add_argument('--analyze-freq', action='store_true',
                       help="Show frequency analysis")
    
    args = parser.parse_args()
    
    try:
        # Load and analyze data
        data, relative_time, sampling_rate = load_and_analyze_data(args.filename)
        
        # Recommend filter parameters based on sampling rate
        nyquist = sampling_rate / 2
        recommended_cutoff = min(args.cutoff, nyquist * 0.4)  # Conservative cutoff
        
        print(f"\nFilter Parameters:")
        print(f"  Sampling rate: {sampling_rate:.2f} Hz")
        print(f"  Nyquist frequency: {nyquist:.2f} Hz")
        print(f"  Requested cutoff: {args.cutoff:.2f} Hz")
        print(f"  Recommended cutoff: {recommended_cutoff:.2f} Hz")
        print(f"  Filter order: {args.order}")
        
        # Create QApplication
        app = QtGui.QApplication.instance()
        if app is None:
            app = QtGui.QApplication(sys.argv)
        
        # Show frequency analysis if requested
        if args.analyze_freq:
            freq_win = analyze_frequency_content_pyqt(data, sampling_rate)
        
        # Create main plotter window
        plotter = SixDoFPlotter(data, relative_time, sampling_rate, 
                               cutoff_freq=recommended_cutoff, filter_order=args.order)
        plotter.show()
        
        print("\nInteractive features:")
        print("- Use mouse wheel to zoom")
        print("- Click and drag to pan")
        print("- Adjust filter parameters in real-time")
        print("- Toggle raw/filtered data visibility")
        print("- Hover over plots to see exact values")
        print("- All plots are linked for synchronized zooming")
        
        # Start the application
        sys.exit(app.exec_())
        
    except FileNotFoundError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
