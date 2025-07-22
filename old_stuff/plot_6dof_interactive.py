import numpy as np
import pandas as pd
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from scipy.signal import butter, filtfilt
import argparse
import os
import sys

def estimate_sampling_rate(timestamps):
    """Estimate sampling rate from timestamps"""
    dt = np.diff(timestamps)
    avg_dt = np.mean(dt)
    sampling_rate = 1.0 / avg_dt
    return sampling_rate, avg_dt

def butterworth_filter(data, cutoff_freq, sampling_rate, order=4, filter_type='low'):
    """Apply Butterworth filter using filtfilt (zero-phase filtering like MATLAB)"""
    nyquist = sampling_rate / 2.0
    normal_cutoff = cutoff_freq / nyquist
    
    # Design the Butterworth filter
    b, a = butter(order, normal_cutoff, btype=filter_type, analog=False)
    
    # Apply zero-phase filtering (equivalent to MATLAB's filtfilt)
    filtered_data = filtfilt(b, a, data)
    
    return filtered_data

def load_and_analyze_data(filename):
    """Load CSV data and perform initial analysis"""
    # Simple load - data should be clean now
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

def plot_6dof_interactive(data, relative_time, sampling_rate, cutoff_freq=10.0, filter_order=4):
    """Create an interactive PyQtGraph plot with real-time controls"""
    
    # Define channel information
    force_channels = ['Fx', 'Fy', 'Fz']
    torque_channels = ['Mx', 'My', 'Mz']
    all_channels = force_channels + torque_channels
    
    # Create PyQtGraph application
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)
    
    # Create main window
    win = QtWidgets.QMainWindow()
    win.setWindowTitle('Interactive 6DoF Force/Torque Data Viewer')
    win.resize(1400, 900)
    
    # Create central widget and layout
    central_widget = QtWidgets.QWidget()
    win.setCentralWidget(central_widget)
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
    cutoff_spinbox.setRange(0.1, min(sampling_rate/2 * 0.8, 100))
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
    
    # Create plot widget with 2x3 grid
    plot_widget = pg.GraphicsLayoutWidget()
    plot_widget.setBackground('w')  # White background
    
    # Create subplots
    plots = []
    raw_curves = []
    filtered_curves = []
    
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]  # RGB colors
    
    for i, channel in enumerate(all_channels):
        row = i // 3
        col = i % 3
        
        # Create plot
        plot = plot_widget.addPlot(row=row, col=col)
        plot.setLabel('left', f'{channel}', units='N' if channel in force_channels else 'Nm')
        plot.setLabel('bottom', 'Time', units='s')
        plot.setTitle(f'{"Force" if channel in force_channels else "Torque"} {channel}')
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.enableAutoRange()
        
        # Create curves (no legend)
        color = colors[i % 3]
        # Raw data as shadow: lighter color, thinner line, no dots
        shadow_color = (*color, 10)  # Add alpha for transparency
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
        
        for i, channel in enumerate(all_channels):
            # Get data
            raw_data = data[channel].values
            
            # Apply filter
            try:
                filtered_data = butterworth_filter(
                    raw_data, current_cutoff, sampling_rate, order=current_order
                )
            except Exception as e:
                print(f"Filter error for {channel}: {e}")
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
        
        # Update statistics
        avg_noise_reduction = np.mean(noise_reductions)
        stats_text = f"Average Noise Reduction: {avg_noise_reduction:.1f}%\n"
        stats_text += f"Cutoff: {current_cutoff:.1f} Hz, Order: {current_order}\n"
        stats_text += f"Data points: {len(relative_time)}, Duration: {relative_time[-1]:.1f}s"
        stats_label.setText(stats_text)
    
    # Connect signals
    cutoff_spinbox.valueChanged.connect(update_plots)
    order_spinbox.valueChanged.connect(update_plots)
    raw_checkbox.stateChanged.connect(update_plots)
    filtered_checkbox.stateChanged.connect(update_plots)
    
    # Initial plot
    update_plots()
    
    # Show window
    win.show()
    
    return app, win

def recommend_filter_parameters(sampling_rate):
    """Recommend meaningful filter parameters based on sampling rate"""
    nyquist = sampling_rate / 2
    
    print(f"\nRecommended Filter Parameters:")
    print(f"  Sampling rate: {sampling_rate:.2f} Hz")
    print(f"  Nyquist frequency: {nyquist:.2f} Hz")
    
    # For force/torque sensors, typical recommendations:
    if sampling_rate > 100:
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

def main():
    # Simple version - just load test_data.csv directly
    filename = 'test_data.csv'
    cutoff_freq = 10.0  # Default cutoff frequency
    filter_order = 4    # Default filter order
    
    print(f"Loading {filename}...")
    
    # Load and analyze data
    data, relative_time, sampling_rate = load_and_analyze_data(filename)
    
    # Get recommended cutoff frequency
    recommended_cutoff = recommend_filter_parameters(sampling_rate)
    
    print(f"\nUsing cutoff frequency: {recommended_cutoff:.2f} Hz")
    print(f"Using filter order: {filter_order}")
    
    # Create interactive plot
    app, win = plot_6dof_interactive(data, relative_time, sampling_rate, 
                                    cutoff_freq=recommended_cutoff, filter_order=filter_order)
    
    print("\nPyQtGraph Interactive Features:")
    print("- Real-time filter adjustment with spinboxes")
    print("- Toggle raw/filtered data visibility with checkboxes")
    print("- Zoom: Mouse wheel or right-click drag")
    print("- Pan: Left-click drag")
    print("- Auto-range: Right-click -> View All")
    print("- Export: Right-click -> Export...")
    print("- Each plot has independent zoom/pan")
    print("- Close the window to exit")
    
    # Run the application
    if hasattr(app, 'exec'):
        app.exec()  # PyQt6/PySide6
    else:
        app.exec_()  # PyQt5/PySide2

# Convenience function to plot data from simple_ati_ft.py
def plot_ati_data(filename=None, cutoff_freq=None):
    """
    Convenient function to plot data from simple_ati_ft.py
    
    Args:
        filename: CSV file to plot (if None, searches for common ATI files)
        cutoff_freq: Filter cutoff frequency (if None, uses recommended value)
    """
    
    # Auto-find data file if not specified
    if filename is None:
        possible_files = [
            'ati_data.csv',
            'quick_test.csv', 
            'ati_highspeed.csv',
            'test_data.csv',
            'ft_sensor_data.csv'
        ]
        
        filename = None
        for file in possible_files:
            if os.path.exists(file):
                filename = file
                break
        
        if filename is None:
            print("No ATI data files found. Run simple_ati_ft.py first to collect data.")
            return
    
    print(f"Plotting data from: {filename}")
    
    try:
        # Load data
        data, relative_time, sampling_rate = load_and_analyze_data(filename)
        
        # Get recommended cutoff if not specified
        if cutoff_freq is None:
            cutoff_freq = recommend_filter_parameters(sampling_rate)
        
        # Create interactive plot
        app, win = plot_6dof_interactive(data, relative_time, sampling_rate, 
                                        cutoff_freq=cutoff_freq, filter_order=4)
        
        # Run the application
        if hasattr(app, 'exec'):
            app.exec()  # PyQt6/PySide6
        else:
            app.exec_()  # PyQt5/PySide2
        
    except Exception as e:
        print(f"Error plotting data: {e}")
        import traceback
        traceback.print_exc()

# Quick plotting functions for different ATI data files
def plot_quick_test():
    """Plot quick test data"""
    plot_ati_data('quick_test.csv', cutoff_freq=20)

def plot_standard_data():
    """Plot standard acquisition data"""
    plot_ati_data('ati_data.csv', cutoff_freq=25)

def plot_highspeed_data():
    """Plot high-speed acquisition data"""
    plot_ati_data('ati_highspeed.csv', cutoff_freq=50)

if __name__ == "__main__":
    # Simple execution - just load test_data.csv and plot
    print("=" * 50)
    print("6DoF Force/Torque Data Plotter")
    print("=" * 50)
    main()

# Example usage functions that can be imported
def quick_plot_demo():
    """
    Quick demonstration of plotting ATI data
    Run this after collecting data with simple_ati_ft.py
    """
    print("Quick ATI Data Plotting Demo")
    print("=" * 40)
    
    # Try different plotting options
    if os.path.exists('quick_test.csv'):
        print("Plotting quick test data...")
        plot_quick_test()
    elif os.path.exists('ati_data.csv'):
        print("Plotting standard ATI data...")
        plot_standard_data()
    else:
        print("No ATI data found. Run simple_ati_ft.py first!")
