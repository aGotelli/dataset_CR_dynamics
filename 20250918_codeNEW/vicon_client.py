# simple_vicon_client.py
"""
Simple Vicon TCP Client
Connects to a Vicon TCP server, sends commands, and retrieves motion capture data.
Date: 2024-06-20
"""

import socket
import json
import numpy as np
import csv


class ViconClient:
    def __init__(self, host, port):
        print(f"🔧 Creating ViconClient instance for {host}:{port}")
        self.host = host
        self.port = port
        self.socket = None
        """Try to connect to the TCP server"""
        try:
            self.socket = socket.socket()
            self.socket.connect((self.host, self.port))
            print(f"Connected to {self.host}:{self.port}")
        except ConnectionRefusedError:
            print(f"Could not connect to {self.host}:{self.port}. Make sure the Vicon TCP server is running.")
            raise
        except Exception as e:
            print(f"Connection error: {e}")
            raise
    
    def send_setup(self, duration=10):
        """Send setup message with specified duration"""
        try:
            self.duration = duration
            print(f"Vicon - Setting up recording for {duration} seconds...")
            setup_cmd = {"command": "setup", "duration": duration}
            self.socket.sendall(json.dumps(setup_cmd).encode())
            response = self.socket.recv(1024).decode()
            # print(f"Server response: {response}")
            return True
        except Exception as e:
            print(f"Setup error: {e}")
            return False
    
    def start_recording(self):
        """Start recording"""
        try:
            start_cmd = {"command": "start"}
            self.socket.sendall(json.dumps(start_cmd).encode())
            response = self.socket.recv(1024).decode()
            # print(f"Server response: {response}")
            print(f"Vicon - Recording started! Duration: {self.duration} seconds")
            return True
        except Exception as e:
            print(f"Start recording error: {e}")
            return False
    
    def get_data(self, csv_filename=None):
        """Get data from server and save to files"""
        try:
            print("Requesting data...")
            get_data_cmd = {"command": "get_data"}
            self.socket.sendall(json.dumps(get_data_cmd).encode())
            
            # Receive status message - double check connection
            # status_response = self.socket.recv(1024).decode()
            # print(f"Server status: {status_response}")
            
            # Receive data size and shape information
            data_size = int.from_bytes(self.socket.recv(4), 'big')
            rows = int.from_bytes(self.socket.recv(4), 'big')
            cols = int.from_bytes(self.socket.recv(4), 'big')
            header_size = int.from_bytes(self.socket.recv(4), 'big')
            
            print(f"Receiving matrix of size {rows}x{cols} ({data_size} bytes)")
            
            # Receive column headers
            column_headers = json.loads(self.socket.recv(header_size).decode())
            
            # Receive data
            data = b''
            while len(data) < data_size:
                chunk = self.socket.recv(min(data_size - len(data), 4096))
                data += chunk
            
            # Convert to numpy matrix
            matrix = np.frombuffer(data, dtype=np.float64).reshape(rows, cols)
            
            print(f"Successfully received Vicon data:")
            print(f"Matrix shape: {matrix.shape}")
            print(f"Columns: {column_headers}")
            print(f"First 5 rows:")
            print(matrix[:5] if len(matrix) > 5 else matrix)
            
            # ------ Save data in npy and json format
            filename = f"vicon_data_{rows}x{cols}.npy"
            np.save(filename, matrix)  # Commented: CSV is sufficient
            
            # ------ Also save column headers
            headers_filename = f"vicon_headers_{rows}x{cols}.json"
            with open(headers_filename, 'w') as f:
                json.dump(column_headers, f, indent=2)  # Commented: CSV is sufficient
            print(f"Headers saved to {headers_filename}")
            
            # Save as CSV if filename provided
            if csv_filename:
                self.save_as_csv(matrix, column_headers, csv_filename)
            
            return matrix, column_headers
            
        except Exception as e:
            print(f"Get data error: {e}")
            return None, None
    
    def save_as_csv(self, matrix, headers, csv_filename):
        """Save Vicon data as CSV file with proper headers and optimized writing"""
        if matrix is None or matrix.size == 0:
            print("❌ No Vicon data to save")
            return False
            
        num_samples, num_cols = matrix.shape
            
        # Validate headers match data dimensions
        if len(headers) != num_cols:
            print(f"⚠️ Header count ({len(headers)}) doesn't match data columns ({num_cols})")
            # Create generic headers if mismatch
            headers = [f"col_{i}" for i in range(num_cols)]
            headers[0] = "timestamp"  # First column should be timestamp
            
        # Write CSV with optimized approach
        with open(csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(headers)
            writer.writerows(matrix)
        print(f"📁 Vicon CSV saved: {csv_filename}")            
        return True
    
    def close(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            print("Connection closed")
