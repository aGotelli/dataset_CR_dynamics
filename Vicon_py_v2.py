from vicon_dssdk import ViconDataStream
import numpy as np
import time
import csv

class ViconV2:
    def __init__(self, host="localhost:801"):
        self.host = host
        self.duration = None
        self.client = ViconDataStream.Client()
        self.data_buffer = None
        self.column_names = []
        self.sample_count = 0
        self.frame_rate = None
        self.extra_buffer = 1.3  # Extra buffer to avoid overflow
        
        print(f"ViconV2 initialized with host: {host}")
        
    def prepare(self, duration=10.0):
        """Connect, setup, and preallocate memory for acquisition"""
        self.duration = duration
        print(f"Preparing Vicon for {duration}s acquisition...")
        
        # Connect and configure
        print(f"Connecting to {self.host}...")
        self.client.Connect(self.host)
        self.client.EnableSegmentData()
        self.client.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
        
        # Get first frame to build structure and verify connectivity
        print("Waiting for data...")
        timeout = 50
        while not self.client.GetFrame() and timeout > 0:
            time.sleep(0.02)
            timeout -= 1
        
        if timeout == 0:
            raise RuntimeError("Failed to get data from Vicon - check connection")
        
        self.frame_rate = self.client.GetFrameRate()
        print(f"Frame rate: {self.frame_rate:.1f} Hz")
        
        # Build column structure
        self.column_names = ['timestamp']
        subjects = self.client.GetSubjectNames()
        
        if not subjects:
            raise RuntimeError("No subjects found in Vicon system")
        
        for subject in subjects:
            segments = self.client.GetSegmentNames(subject)
            for segment in segments:
                prefix = f"{subject}_{segment}" if segment != subject else subject
                # Position XYZ first, then rotation XYZ
                for axis in ['X', 'Y', 'Z']:
                    self.column_names.append(f"{prefix}_pos_{axis}")
                for axis in ['X', 'Y', 'Z']:
                    self.column_names.append(f"{prefix}_rot_{axis}")
        
        print(f"Found {len(subjects)} subjects, {len(self.column_names)} columns")
        
        # Test data collection for all markers
        print("Testing data collection...")
        test_data = self._test_data_collection()
        if test_data:
            print("✅ All markers responding correctly")
        else:
            print("⚠️ Some markers may have issues")
        
        # Allocate buffer
        expected_samples = int(self.extra_buffer * self.duration * self.frame_rate)
        self.data_buffer = np.zeros((expected_samples, len(self.column_names)))
        print(f"Memory allocated: {expected_samples} samples x {len(self.column_names)} columns")
        
        print("Vicon ready for acquisition!")
        return True
    
    def _test_data_collection(self):
        """Test data collection for a few frames to verify all markers work"""
        test_frames = 5
        good_frames = 0
        
        for i in range(test_frames):
            if self.client.GetFrame():
                all_ok = True
                for subject in self.client.GetSubjectNames():
                    for segment in self.client.GetSegmentNames(subject):
                        pos, pos_occluded = self.client.GetSegmentGlobalTranslation(subject, segment)
                        rot, rot_occluded = self.client.GetSegmentGlobalRotationEulerXYZ(subject, segment)
                        if pos_occluded and rot_occluded:
                            all_ok = False
                            print(f"  Warning: {subject}.{segment} occluded")
                
                if all_ok:
                    good_frames += 1
            time.sleep(0.01)
        
        return good_frames >= test_frames // 2
    
    def collect_frame(self):
        self.data_buffer[self.sample_count, 0] = time.time()
        col = 1
        
        for subject in self.client.GetSubjectNames():
            for segment in self.client.GetSegmentNames(subject):
                # Get position and rotation
                pos, pos_occluded = self.client.GetSegmentGlobalTranslation(subject, segment)
                rot, rot_occluded = self.client.GetSegmentGlobalRotationEulerXYZ(subject, segment)
                
                # Fill position XYZ first
                for i in range(3):  # X, Y, Z
                    self.data_buffer[self.sample_count, col] = pos[i] if not pos_occluded else 0
                    col += 1
                
                # Then fill rotation XYZ
                for i in range(3):  # X, Y, Z
                    self.data_buffer[self.sample_count, col] = rot[i] if not rot_occluded else 0
                    col += 1
        
        self.sample_count += 1
    
    def _show_frame_rate(self, start_time):
        """Simple function to display actual frame rate during acquisition"""
        elapsed = time.time() - start_time
        if elapsed > 0:
            actual_fps = self.sample_count / elapsed
            print(f"\r{elapsed:.1f}s - {self.sample_count} frames - {actual_fps:.1f} fps", end="", flush=True)
    
    def acquire(self):
        print(f"Recording for {self.duration}s...")
        start_time = time.time()
        
        while (time.time() - start_time) < self.duration:
            if self.client.GetFrame():
                self.collect_frame()
                #self._show_frame_rate(start_time)
        
        print(f"\nCollected {self.sample_count} samples")
        
        # Return data matrix and column info
        self.data_matrix = self.data_buffer[:self.sample_count]
        self.column_info = self.column_names.copy()

        print(f"Matrix shape: {self.data_matrix.shape}")
        print(f"Columns: {len(self.column_info)}")


def main():
    # Initialize without duration
    vicon = ViconV2()

    # Later, prepare for acquisition with specific duration
    vicon.prepare(duration=5.0)  # Test connectivity and allocate memory

    vicon.acquire()

    print(f"\nData matrix shape: {vicon.data_matrix.shape}")
    print(f"Column info: {vicon.column_info}")
    print(f"First few samples:")
    print(vicon.data_matrix[:3, :5])  # Show first 3 rows, 5 columns

if __name__ == "__main__":
    main()