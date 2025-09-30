import socket
import json
import numpy as np
import signal
import sys
import time

class SimpleViconClient:
    def __init__(self, host="192.168.10.2", port=8080):
        """Connect to the Vicon TCP server in constructor"""
        self.host = host
        self.port = port
        self.socket = None
        self.duration = 0
        self.connect()
    
    def connect(self):
        """Connect to the TCP server"""
        try:
            self.socket = socket.socket()
            print(f"Connecting to {self.host}:{self.port}...")
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
            print(f"Setting up recording for {duration} seconds...")
            setup_cmd = {"command": "setup", "duration": duration}
            self.socket.sendall(json.dumps(setup_cmd).encode())
            response = self.socket.recv(1024).decode()
            print(f"Server response: {response}")
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
            print(f"Server response: {response}")
            print(f"Recording started! Duration: {self.duration} seconds")
            return True
        except Exception as e:
            print(f"Start recording error: {e}")
            return False
    
    def get_data(self):
        """Get data from server and save to files"""
        try:
            print("Requesting data...")
            get_data_cmd = {"command": "get_data"}
            self.socket.sendall(json.dumps(get_data_cmd).encode())
            
            # Receive status message
            status_response = self.socket.recv(1024).decode()
            print(f"Server status: {status_response}")
            
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
            
            # Save data
            filename = f"vicon_data_{rows}x{cols}.npy"
            np.save(filename, matrix)
            print(f"Data saved to {filename}")
            
            # Also save column headers
            headers_filename = f"vicon_headers_{rows}x{cols}.json"
            with open(headers_filename, 'w') as f:
                json.dump(column_headers, f, indent=2)
            print(f"Headers saved to {headers_filename}")
            
            return matrix, column_headers
            
        except Exception as e:
            print(f"Get data error: {e}")
            return None, None
    
    def close(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            print("Connection closed")

def signal_handler(sig, frame):
    print("\nStopping...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    # Configuration
    host = "192.168.10.2"  # Vicon server IP
    port = 8080
    duration = 10  # Recording duration in seconds
    extra_wait = 2  # Extra time to wait after recording completes

    client = None
    try:
        # Create client and connect (done in constructor)
        client = SimpleViconClient(host, port)
        
        # Send setup message
        client.send_setup(duration)
        
        # Ask for Enter to start recording
        input("Press Enter to start recording...")
        
        # Start recording
        client.start_recording()
        
        # Wait for recording completion in main loop
        total_wait = duration + extra_wait
        print(f"Waiting {total_wait} seconds for recording to complete...")
        time.sleep(total_wait)
        
        # Get data
        matrix, headers = client.get_data()
        
        if matrix is not None:
            print("Recording and data retrieval completed successfully!")
        else:
            print("Failed to retrieve data")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if client:
            client.close()

if __name__ == "__main__":
    main()