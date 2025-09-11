import socket
import json
import numpy as np
import signal
import sys

def signal_handler(sig, frame):
    print("\nStopping...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    host = "192.168.10.5"
    port = 8080
    duration = 5  # seconds


    with socket.socket() as s:
        s.connect((host, port))
        print(f"Connected to {host}:{port}")
        
        # Start recording
        s.sendall(json.dumps({"command": "start", "duration": duration}).encode())
        print(s.recv(1024).decode())
        print(f"Recording for {duration}s...")
        
        input("Press Enter to save data...")
        
        # Save and receive data
        s.sendall(json.dumps({"command": "save"}).encode())
        print(s.recv(1024).decode())
        
        # Get data
        data_size = int.from_bytes(s.recv(4), 'big')
        rows = int.from_bytes(s.recv(4), 'big')
        cols = int.from_bytes(s.recv(4), 'big')
        info_size = int.from_bytes(s.recv(4), 'big')
        
        column_info = json.loads(s.recv(info_size).decode())
        
        data = b''
        while len(data) < data_size:
            data += s.recv(data_size - len(data))
        
        matrix = np.frombuffer(data, dtype=np.float64).reshape(rows, cols)
        
        print(f"Matrix {rows}x{cols}:")
        print(matrix)
        print(f"Columns: {column_info}")
        
        np.save(f"matrix_{rows}x{cols}.npy", matrix)
        print("Saved!")

if __name__ == "__main__":
    main()
