import socket
import numpy as np

def main():
    server_host = input("Server IP (localhost): ") or "localhost"
    server_port = int(input("Server port (8080): ") or "8080")
    
    with socket.socket() as s:
        s.connect((server_host, server_port))
        print(f"Connected to {server_host}:{server_port}")
        
        # Receive data size first
        size_bytes = s.recv(4)
        data_size = int.from_bytes(size_bytes, 'big')
        print(f"Receiving {data_size} bytes...")
        
        # Receive all matrix data
        data = b''
        while len(data) < data_size:
            chunk = s.recv(min(4096, data_size - len(data)))
            data += chunk
        
        # Convert back to numpy array (assuming float64)
        matrix = np.frombuffer(data, dtype=np.float64)
        
        # Reshape to 2D (you may need to adjust based on your matrix dimensions)
        # For now, let's assume it's a square matrix or ask user
        total_elements = len(matrix)
        rows = int(input(f"Matrix rows ({int(np.sqrt(total_elements))}): ") or int(np.sqrt(total_elements)))
        cols = total_elements // rows
        
        matrix = matrix.reshape(rows, cols)
        
        print(f"Matrix shape: {matrix.shape}")
        print(f"Matrix data (first 5x5):")
        print(matrix[:5, :5])
        
        # Save to file
        filename = f"received_matrix_{rows}x{cols}.npy"
        np.save(filename, matrix)
        print(f"Saved matrix to: {filename}")
        print(f"Total elements: {matrix.size}, Shape: {rows}x{cols}")

if __name__ == "__main__":
    main()
