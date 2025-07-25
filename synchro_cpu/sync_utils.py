"""
Enhanced time synchronization utilities for high-precision data acquisition

This module provides utilities for improving timestamp accuracy beyond basic NTP,
including performance monitoring and synchronization validation.
"""

import time
import threading
import queue
import statistics
from typing import List, Dict, Tuple, Optional
import ctypes
from ctypes import wintypes
import sys

class HighPrecisionTimer:
    """
    Windows high-precision timer using QueryPerformanceCounter
    
    This provides microsecond-level precision for relative timing measurements
    while still being synchronized to system time via NTP.
    """
    
    def __init__(self):
        self.kernel32 = ctypes.windll.kernel32
        self.freq = wintypes.LARGE_INTEGER()
        self.kernel32.QueryPerformanceFrequency(ctypes.byref(self.freq))
        self.frequency = float(self.freq.value)
        
        # Get initial sync between QPC and system time
        self._sync_qpc_to_system_time()
        
    def _sync_qpc_to_system_time(self):
        """Synchronize QPC timer to system time"""
        # Take multiple samples to get best sync
        samples = []
        for _ in range(10):
            qpc_time = self._get_qpc_time()
            sys_time = time.time()
            samples.append((qpc_time, sys_time))
            time.sleep(0.001)  # Small delay between samples
        
        # Use the sample with minimum delay between measurements
        min_delay = float('inf')
        best_sample = None
        
        for i in range(len(samples)-1):
            qpc1, sys1 = samples[i]
            qpc2, sys2 = samples[i+1]
            delay = (qpc2 - qpc1) - (sys2 - sys1)
            if abs(delay) < min_delay:
                min_delay = abs(delay)
                best_sample = samples[i]
        
        self.qpc_offset = best_sample[1] - best_sample[0]
        
    def _get_qpc_time(self) -> float:
        """Get QPC time in seconds"""
        counter = wintypes.LARGE_INTEGER()
        self.kernel32.QueryPerformanceCounter(ctypes.byref(counter))
        return float(counter.value) / self.frequency
    
    def time(self) -> float:
        """Get current time synchronized to system clock with high precision"""
        return self._get_qpc_time() + self.qpc_offset
    
    def resync(self):
        """Re-synchronize QPC to system time (call periodically)"""
        self._sync_qpc_to_system_time()


class SynchronizedDataCollector:
    """
    Base class for synchronized data collection across multiple computers
    
    Provides common timing and synchronization utilities that can be used
    by your existing data collection classes.
    """
    
    def __init__(self, use_high_precision=True, sync_interval=300):
        """
        Initialize synchronized data collector
        
        Args:
            use_high_precision: Use QueryPerformanceCounter for higher precision
            sync_interval: How often to re-sync high precision timer (seconds)
        """
        self.use_high_precision = use_high_precision
        self.sync_interval = sync_interval
        
        if use_high_precision:
            self.timer = HighPrecisionTimer()
            self.last_sync = time.time()
        else:
            self.timer = None
    
    def get_timestamp(self) -> float:
        """Get synchronized timestamp"""
        if self.use_high_precision:
            # Periodically re-sync to account for clock drift
            current_time = time.time()
            if current_time - self.last_sync > self.sync_interval:
                self.timer.resync()
                self.last_sync = current_time
            
            return self.timer.time()
        else:
            return time.time()
    
    def validate_synchronization(self) -> Dict[str, float]:
        """
        Validate current time synchronization quality
        
        Returns:
            Dictionary with synchronization metrics
        """
        # Compare high-precision timer with system time
        samples = []
        for _ in range(100):
            if self.use_high_precision:
                hp_time = self.timer.time()
            sys_time = time.time()
            
            if self.use_high_precision:
                diff = hp_time - sys_time
                samples.append(diff)
            
            time.sleep(0.001)
        
        if self.use_high_precision and samples:
            return {
                'mean_offset_ms': statistics.mean(samples) * 1000,
                'std_dev_ms': statistics.stdev(samples) * 1000,
                'max_offset_ms': max(samples) * 1000,
                'min_offset_ms': min(samples) * 1000,
                'sample_count': len(samples)
            }
        else:
            return {'status': 'high_precision_timer_not_available'}


class TimestampedDataBuffer:
    """
    Thread-safe buffer for timestamped data with synchronization tracking
    """
    
    def __init__(self, max_size=10000):
        self.buffer = queue.Queue(maxsize=max_size)
        self.lock = threading.Lock()
        self.collector = SynchronizedDataCollector()
        
    def add_sample(self, data):
        """Add a data sample with synchronized timestamp"""
        timestamp = self.collector.get_timestamp()
        sample = {
            'timestamp': timestamp,
            'data': data
        }
        
        try:
            self.buffer.put_nowait(sample)
        except queue.Full:
            # Remove oldest sample to make room
            try:
                self.buffer.get_nowait()
                self.buffer.put_nowait(sample)
            except queue.Empty:
                pass
    
    def get_all_samples(self) -> List[Dict]:
        """Get all samples from buffer"""
        samples = []
        while True:
            try:
                sample = self.buffer.get_nowait()
                samples.append(sample)
            except queue.Empty:
                break
        return samples
    
    def get_sync_quality(self) -> Dict[str, float]:
        """Get current synchronization quality metrics"""
        return self.collector.validate_synchronization()


def create_sync_test_data(duration=10, sample_rate=100):
    """
    Create test data with high-precision timestamps for validation
    
    Args:
        duration: Test duration in seconds
        sample_rate: Samples per second
        
    Returns:
        List of (timestamp, sample_number) tuples
    """
    collector = SynchronizedDataCollector(use_high_precision=True)
    data = []
    
    start_time = collector.get_timestamp()
    sample_interval = 1.0 / sample_rate
    sample_count = 0
    
    while True:
        current_time = collector.get_timestamp()
        if current_time - start_time >= duration:
            break
        
        # Check if it's time for next sample
        expected_time = start_time + (sample_count * sample_interval)
        if current_time >= expected_time:
            data.append((current_time, sample_count))
            sample_count += 1
        
        # Small sleep to prevent 100% CPU usage
        time.sleep(sample_interval * 0.1)
    
    return data


def analyze_timing_accuracy(timestamps: List[float], expected_rate: float) -> Dict[str, float]:
    """
    Analyze timing accuracy of timestamp data
    
    Args:
        timestamps: List of timestamps
        expected_rate: Expected sampling rate in Hz
        
    Returns:
        Dictionary with timing analysis metrics
    """
    if len(timestamps) < 2:
        return {'error': 'insufficient_data'}
    
    # Calculate intervals
    intervals = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
    expected_interval = 1.0 / expected_rate
    
    # Calculate metrics
    mean_interval = statistics.mean(intervals)
    std_interval = statistics.stdev(intervals) if len(intervals) > 1 else 0
    actual_rate = 1.0 / mean_interval if mean_interval > 0 else 0
    
    # Calculate jitter and rate accuracy
    jitter_ms = std_interval * 1000
    rate_error_percent = abs(actual_rate - expected_rate) / expected_rate * 100
    
    return {
        'expected_rate_hz': expected_rate,
        'actual_rate_hz': actual_rate,
        'rate_error_percent': rate_error_percent,
        'mean_interval_ms': mean_interval * 1000,
        'expected_interval_ms': expected_interval * 1000,
        'jitter_ms': jitter_ms,
        'min_interval_ms': min(intervals) * 1000,
        'max_interval_ms': max(intervals) * 1000,
        'total_samples': len(timestamps),
        'total_duration_s': timestamps[-1] - timestamps[0]
    }


if __name__ == "__main__":
    # Example usage and testing
    print("Testing high-precision timing...")
    
    # Test basic functionality
    collector = SynchronizedDataCollector(use_high_precision=True)
    
    print("Collecting test data...")
    test_data = create_sync_test_data(duration=5, sample_rate=50)
    timestamps = [item[0] for item in test_data]
    
    print(f"Collected {len(test_data)} samples")
    
    # Analyze timing
    analysis = analyze_timing_accuracy(timestamps, expected_rate=50.0)
    print("\nTiming Analysis:")
    for key, value in analysis.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")
    
    # Test synchronization quality
    sync_quality = collector.validate_synchronization()
    print("\nSynchronization Quality:")
    for key, value in sync_quality.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")
