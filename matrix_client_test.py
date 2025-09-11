import socket
import json
import numpy as np

def main():
    host = input("Server IP (localhost): ") or "localhost"
    port = int(input("Port (8080): ") or "8080")
    duration = int(input("Duration (5): ") or "5")
    
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
