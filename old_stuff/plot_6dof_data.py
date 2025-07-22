import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
import argparse
import os

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
    
    Args:
        data: Input signal
        cutoff_freq: Cutoff frequency in Hz
        sampling_rate: Sampling rate in Hz
        order: Filter order (default 4)
        filter_type: 'low', 'high', 'band', 'bandstop'
    
    Returns:
        Filtered signal
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

def plot_6dof_data(data, relative_time, sampling_rate, cutoff_freq=10.0, filter_order=4):
    """
    Plot 6DoF force/torque data with and without filtering
    """
    # Define channel names and units
    force_channels = ['Fx', 'Fy', 'Fz']  # Forces in N
    torque_channels = ['Mx', 'My', 'Mz']  # Torques in Nm
    
    # Create subplots
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle(f'6DoF Force/Torque Data (Cutoff: {cutoff_freq} Hz)', fontsize=16)
    
    # Plot forces
    for i, channel in enumerate(force_channels):
        ax = axes[0, i]
        raw_data = data[channel].values
        
        # Apply Butterworth filter
        filtered_data = butterworth_filter(raw_data, cutoff_freq, sampling_rate, order=filter_order)
        
        # Plot both raw and filtered data
        ax.plot(relative_time, raw_data, 'b-', alpha=0.3, linewidth=0.5, label='Raw')
        ax.plot(relative_time, filtered_data, 'r-', linewidth=1.5, label=f'Filtered ({cutoff_freq} Hz)')
        
        ax.set_title(f'{channel} (Force)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Force (N)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Add statistics
        raw_std = np.std(raw_data)
        filtered_std = np.std(filtered_data)
        noise_reduction = (1 - filtered_std/raw_std) * 100
        ax.text(0.02, 0.98, f'Noise reduction: {noise_reduction:.1f}%', 
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Plot torques
    for i, channel in enumerate(torque_channels):
        ax = axes[1, i]
        raw_data = data[channel].values
        
        # Apply Butterworth filter
        filtered_data = butterworth_filter(raw_data, cutoff_freq, sampling_rate, order=filter_order)
        
        # Plot both raw and filtered data
        ax.plot(relative_time, raw_data, 'b-', alpha=0.3, linewidth=0.5, label='Raw')
        ax.plot(relative_time, filtered_data, 'r-', linewidth=1.5, label=f'Filtered ({cutoff_freq} Hz)')
        
        ax.set_title(f'{channel} (Torque)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Torque (Nm)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        # Add statistics
        raw_std = np.std(raw_data)
        filtered_std = np.std(filtered_data)
        noise_reduction = (1 - filtered_std/raw_std) * 100
        ax.text(0.02, 0.98, f'Noise reduction: {noise_reduction:.1f}%', 
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    return fig

def analyze_frequency_content(data, sampling_rate, channel='Fx'):
    """
    Analyze frequency content of the signal to help determine filter parameters
    """
    signal = data[channel].values
    
    # Compute FFT
    fft = np.fft.fft(signal)
    freqs = np.fft.fftfreq(len(signal), 1/sampling_rate)
    
    # Only positive frequencies
    positive_freqs = freqs[:len(freqs)//2]
    magnitude = np.abs(fft[:len(fft)//2])
    
    # Plot frequency spectrum
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(positive_freqs, magnitude)
    plt.title(f'Frequency Spectrum of {channel}')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.grid(True)
    plt.xlim(0, min(100, sampling_rate/2))  # Show up to 100 Hz or Nyquist frequency
    
    # Plot power spectral density (log scale)
    plt.subplot(2, 1, 2)
    power = magnitude**2
    plt.semilogy(positive_freqs, power)
    plt.title(f'Power Spectral Density of {channel}')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Power')
    plt.grid(True)
    plt.xlim(0, min(100, sampling_rate/2))
    
    plt.tight_layout()
    plt.show()
    
    # Find dominant frequencies
    peak_indices = np.argsort(magnitude)[-10:]  # Top 10 peaks
    dominant_freqs = positive_freqs[peak_indices]
    print(f"\nDominant frequencies in {channel}:")
    for freq in sorted(dominant_freqs[dominant_freqs > 0]):
        print(f"  {freq:.2f} Hz")

def main():
    parser = argparse.ArgumentParser(description="Plot 6DoF force/torque data with Butterworth filtering")
    parser.add_argument('filename', type=str, nargs='?', default='test_data.csv',
                       help="CSV file to plot (default: test_data.csv)")
    parser.add_argument('--cutoff', type=float, default=10.0,
                       help="Cutoff frequency for Butterworth filter in Hz (default: 10.0)")
    parser.add_argument('--order', type=int, default=4,
                       help="Filter order (default: 4)")
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
        
        # Show frequency analysis if requested
        if args.analyze_freq:
            analyze_frequency_content(data, sampling_rate)
        
        # Plot the data
        fig = plot_6dof_data(data, relative_time, sampling_rate, 
                           cutoff_freq=recommended_cutoff, filter_order=args.order)
        plt.show()
        
        # Optionally save filtered data
        save_filtered = input("\nSave filtered data to CSV? (y/n): ").lower().strip() == 'y'
        if save_filtered:
            filtered_data = data.copy()
            channels = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
            
            for channel in channels:
                filtered_data[f'{channel}_filtered'] = butterworth_filter(
                    data[channel].values, recommended_cutoff, sampling_rate, order=args.order)
            
            output_filename = f"filtered_{args.filename}"
            filtered_data.to_csv(output_filename, index=False)
            print(f"Filtered data saved to {output_filename}")
        
    except FileNotFoundError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
