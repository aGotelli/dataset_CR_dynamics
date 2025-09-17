"""
ATI Sensor Noise Analysis
Script to analyze noise characteristics in ATI Force/Torque sensor data
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import glob
from scipy import signal
from scipy.fft import fft, fftfreq

def load_ati_data(experiment_folder):
    """Load ATI data from an experiment folder"""
    ati_file = os.path.join(experiment_folder, "ati_data.csv")
    if os.path.exists(ati_file):
        data = pd.read_csv(ati_file)
        print(f"✅ ATI data loaded: {len(data)} samples")
        return data
    else:
        print(f"❌ No ATI data found in {experiment_folder}")
        return None

def analyze_ati_noise_spectrum(ati_data, experiment_name, save_path=None):
    """Analyze noise spectrum in ATI sensor data"""
    if ati_data is None or ati_data.empty:
        print("❌ No ATI data available for noise analysis")
        return None
    
    # Calculate sampling rate
    time_diff = np.diff(ati_data['timestamp'])
    sampling_rate = 1.0 / np.mean(time_diff)
    print(f"📊 ATI Sampling rate: {sampling_rate:.1f} Hz")
    
    # Create figure with subplots for forces and torques
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f'ATI Sensor Noise Analysis - {experiment_name}', fontsize=16, fontweight='bold')
    
    # Force channels
    force_channels = ['Fx', 'Fy', 'Fz']
    torque_channels = ['Mx', 'My', 'Mz']
    
    # Colors for different channels
    force_colors = ['red', 'green', 'blue']
    torque_colors = ['orange', 'purple', 'brown']
    
    # Create relative time for plotting
    time_rel = ati_data['timestamp'] - ati_data['timestamp'].iloc[0]
    
    # Plot 1: Force time series
    ax1 = axes[0, 0]
    for i, channel in enumerate(force_channels):
        if channel in ati_data.columns:
            ax1.plot(time_rel, ati_data[channel], 
                    color=force_colors[i], label=channel, linewidth=0.8, alpha=0.8)
    ax1.set_title('Forces - Time Domain', fontweight='bold')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Force (N)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Force spectrum
    ax2 = axes[0, 1]
    for i, channel in enumerate(force_channels):
        if channel in ati_data.columns:
            # Remove DC component and calculate FFT
            signal_data = ati_data[channel] - np.mean(ati_data[channel])
            N = len(signal_data)
            
            # Apply window to reduce spectral leakage
            windowed_signal = signal_data * np.hanning(N)
            
            # Calculate FFT
            fft_data = fft(windowed_signal)
            freqs = fftfreq(N, 1/sampling_rate)
            
            # Only positive frequencies
            pos_freqs = freqs[:N//2]
            magnitude = np.abs(fft_data[:N//2]) * 2/N
            
            # Plot in dB scale
            magnitude_db = 20 * np.log10(magnitude + 1e-12)  # Add small value to avoid log(0)
            
            ax2.plot(pos_freqs, magnitude_db, color=force_colors[i], label=channel, linewidth=1)
    
    ax2.set_title('Forces - Frequency Spectrum', fontweight='bold')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylabel('Magnitude (dB)')
    ax2.set_xlim(0, min(100, sampling_rate/2))  # Show up to 100Hz or Nyquist
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Add vertical lines for common interference frequencies
    ax2.axvline(x=50, color='red', linestyle='--', alpha=0.5, linewidth=2, label='50Hz (EU)')
    ax2.axvline(x=60, color='orange', linestyle='--', alpha=0.5, linewidth=2, label='60Hz (US)')
    ax2.axvline(x=100, color='gray', linestyle=':', alpha=0.5, linewidth=1, label='100Hz')
    
    # Plot 3: Torque time series
    ax3 = axes[1, 0]
    for i, channel in enumerate(torque_channels):
        if channel in ati_data.columns:
            ax3.plot(time_rel, ati_data[channel], 
                    color=torque_colors[i], label=channel, linewidth=0.8, alpha=0.8)
    ax3.set_title('Torques - Time Domain', fontweight='bold')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Torque (Nm)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Torque spectrum
    ax4 = axes[1, 1]
    for i, channel in enumerate(torque_channels):
        if channel in ati_data.columns:
            # Remove DC component and calculate FFT
            signal_data = ati_data[channel] - np.mean(ati_data[channel])
            N = len(signal_data)
            
            # Apply window
            windowed_signal = signal_data * np.hanning(N)
            
            # Calculate FFT
            fft_data = fft(windowed_signal)
            freqs = fftfreq(N, 1/sampling_rate)
            
            # Only positive frequencies
            pos_freqs = freqs[:N//2]
            magnitude = np.abs(fft_data[:N//2]) * 2/N
            
            # Plot in dB scale
            magnitude_db = 20 * np.log10(magnitude + 1e-12)
            
            ax4.plot(pos_freqs, magnitude_db, color=torque_colors[i], label=channel, linewidth=1)
    
    ax4.set_title('Torques - Frequency Spectrum', fontweight='bold')
    ax4.set_xlabel('Frequency (Hz)')
    ax4.set_ylabel('Magnitude (dB)')
    ax4.set_xlim(0, min(100, sampling_rate/2))
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # Add vertical lines for common interference frequencies
    ax4.axvline(x=50, color='red', linestyle='--', alpha=0.5, linewidth=2, label='50Hz (EU)')
    ax4.axvline(x=60, color='orange', linestyle='--', alpha=0.5, linewidth=2, label='60Hz (US)')
    ax4.axvline(x=100, color='gray', linestyle=':', alpha=0.5, linewidth=1, label='100Hz')
    
    plt.tight_layout()
    
    # Print detailed noise statistics
    print(f"\n📊 ATI Noise Statistics for {experiment_name}:")
    print("-" * 60)
    
    for channel in force_channels + torque_channels:
        if channel in ati_data.columns:
            data_vals = ati_data[channel]
            mean_val = np.mean(data_vals)
            rms = np.sqrt(np.mean(data_vals**2))
            std = np.std(data_vals)
            peak_to_peak = np.max(data_vals) - np.min(data_vals)
            snr_db = 20 * np.log10(abs(mean_val) / (std + 1e-12))
            
            print(f"   {channel:2s}: Mean={mean_val:8.4f}, RMS={rms:8.4f}, STD={std:8.4f}")
            print(f"       P2P={peak_to_peak:8.4f}, SNR={snr_db:6.1f}dB")
    
    # Analyze frequency content for each channel
    print(f"\n🔍 Frequency Analysis:")
    print("-" * 60)
    
    for channel in ['Fx', 'Fy', 'Fz']:  # Focus on forces first
        if channel in ati_data.columns:
            signal_data = ati_data[channel] - np.mean(ati_data[channel])
            N = len(signal_data)
            windowed_signal = signal_data * np.hanning(N)
            fft_data = fft(windowed_signal)
            freqs = fftfreq(N, 1/sampling_rate)
            pos_freqs = freqs[:N//2]
            magnitude = np.abs(fft_data[:N//2]) * 2/N
            
            # Find peak frequencies
            peak_indices = signal.find_peaks(magnitude, height=np.max(magnitude)*0.1)[0]
            peak_freqs = pos_freqs[peak_indices]
            peak_mags = magnitude[peak_indices]
            
            print(f"   {channel} - Top frequency components:")
            for freq, mag in zip(peak_freqs[:5], peak_mags[:5]):  # Top 5 peaks
                if freq > 0.1:  # Skip DC component
                    print(f"      {freq:6.1f} Hz: {mag:8.4f}")
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n📊 Noise analysis saved to: {save_path}")
    
    plt.show()
    return fig

def compare_channels_noise(ati_data, experiment_name):
    """Compare noise levels between different channels"""
    if ati_data is None or ati_data.empty:
        return
    
    print(f"\n🔬 Channel Noise Comparison for {experiment_name}:")
    print("=" * 70)
    
    channels = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
    noise_stats = {}
    
    for channel in channels:
        if channel in ati_data.columns:
            data_vals = ati_data[channel]
            noise_stats[channel] = {
                'std': np.std(data_vals),
                'rms': np.sqrt(np.mean(data_vals**2)),
                'p2p': np.max(data_vals) - np.min(data_vals)
            }
    
    # Compare Fz with Fx and Fy
    if all(ch in noise_stats for ch in ['Fx', 'Fy', 'Fz']):
        fz_std = noise_stats['Fz']['std']
        fx_std = noise_stats['Fx']['std']
        fy_std = noise_stats['Fy']['std']
        
        print(f"Force Channel Noise (Standard Deviation):")
        print(f"   Fx: {fx_std:.4f} N")
        print(f"   Fy: {fy_std:.4f} N") 
        print(f"   Fz: {fz_std:.4f} N")
        print(f"\nFz vs Fx ratio: {fz_std/fx_std:.2f}x")
        print(f"Fz vs Fy ratio: {fz_std/fy_std:.2f}x")
        
        if fz_std > 2 * max(fx_std, fy_std):
            print("⚠️  Fz has significantly higher noise than Fx/Fy!")
        elif fz_std > 1.5 * max(fx_std, fy_std):
            print("⚠️  Fz has moderately higher noise than Fx/Fy")
        else:
            print("✅ Fz noise level is comparable to Fx/Fy")

def analyze_experiment_folder(experiment_folder):
    """Analyze ATI noise for a single experiment"""
    print(f"\n{'='*60}")
    print(f"ATI NOISE ANALYSIS: {os.path.basename(experiment_folder)}")
    print(f"{'='*60}")
    
    # Load ATI data
    ati_data = load_ati_data(experiment_folder)
    
    if ati_data is None:
        return
    
    experiment_name = os.path.basename(experiment_folder)
    
    # Perform noise spectrum analysis
    noise_path = os.path.join(experiment_folder, f"{experiment_name}_ati_noise_analysis.png")
    analyze_ati_noise_spectrum(ati_data, experiment_name, noise_path)
    
    # Compare channel noise levels
    compare_channels_noise(ati_data, experiment_name)

def main():
    """Main function"""
    print("🔬 ATI SENSOR NOISE ANALYSIS")
    print("=" * 50)
    
    # Find all experiment folders
    data_dir = "data"
    experiment_folders = []
    
    if os.path.exists(data_dir):
        for item in os.listdir(data_dir):
            item_path = os.path.join(data_dir, item)
            if os.path.isdir(item_path) and item.startswith("test_experiment"):
                experiment_folders.append(item_path)
    
    if not experiment_folders:
        print("❌ No experiment folders found in data directory")
        return
    
    # Sort by date (newest first)
    experiment_folders.sort(reverse=True)
    
    print(f"📁 Found {len(experiment_folders)} experiment folders:")
    for i, folder in enumerate(experiment_folders):
        print(f"   {i+1}. {os.path.basename(folder)}")
    
    # Ask user to select experiment or analyze all
    choice = input(f"\nEnter experiment number (1-{len(experiment_folders)}) or 'all': ").strip()
    
    if choice.lower() == 'all':
        # Analyze all experiments
        for folder in experiment_folders:
            try:
                analyze_experiment_folder(folder)
            except Exception as e:
                print(f"❌ Error analyzing {os.path.basename(folder)}: {e}")
    else:
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(experiment_folders):
                analyze_experiment_folder(experiment_folders[idx])
            else:
                print("❌ Invalid experiment number")
        except ValueError:
            print("❌ Invalid input")

if __name__ == "__main__":
    main()
