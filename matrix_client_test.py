import socket
import numpy as np

def main():
    server_host = input("Server IP (localhost): ") or "localhost"
    server_port = int(input("Server port (8080): ") or "8080")
    
    with socket.socket() as s:
        s.connect((server_host, server_port))
        print(f"Connected to {server_host}:{server_port}")
        
        # Receive data size, rows, and cols
        size_bytes = s.recv(4)
        data_size = int.from_bytes(size_bytes, 'big')
        
        rows_bytes = s.recv(4)
        rows = int.from_bytes(rows_bytes, 'big')
        
        cols_bytes = s.recv(4)
        cols = int.from_bytes(cols_bytes, 'big')
        
        print(f"Receiving {rows}x{cols} matrix ({data_size} bytes)...")
        
        # Receive all matrix data
        data = b''
        while len(data) < data_size:
            chunk = s.recv(min(4096, data_size - len(data)))
            data += chunk
        
        # Convert back to numpy array
        matrix = np.frombuffer(data, dtype=np.float64).reshape(rows, cols)
        
        print(f"Matrix shape: {matrix.shape}")
        print(f"Matrix data:")
        print(matrix)
        
        # Save to file
        filename = f"received_matrix_{rows}x{cols}.npy"
        np.save(filename, matrix)
        print(f"Saved matrix to: {filename}")
        print(f"Total elements: {matrix.size}, Shape: {rows}x{cols}")

if __name__ == "__main__":
    main()
