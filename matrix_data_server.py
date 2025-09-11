import socket
import json
import numpy as np
import csv
import signal
import sys

class MatrixServer:
    def __init__(self, port=8080, rows=10, cols=5):
        self.port = port
        self.rows = rows
        self.cols = cols

        print(f"Generating {self.rows}x{self.cols} matrix...")
        # Create predictable matrix with linspace values
        self.data = np.linspace(1, self.rows * self.cols, self.rows * self.cols).reshape(self.rows, self.cols)
        print(f"Matrix preview:\n{self.data}")
        
    def start(self):
        with socket.socket() as s:
            s.bind(("0.0.0.0", self.port))
            s.listen()
            s.settimeout(1.0)
            print(f"Matrix server on port {self.port}")

            while True:
                try:
                    conn, addr = s.accept()
                    print(f"Connection from {addr}")
                    with conn:
                        # Send matrix dimensions and data
                        data_bytes = self.data.tobytes()
                        
                        print(f"Sending matrix: {self.rows}x{self.cols} ({len(data_bytes)} bytes)")
                        
                        # Send: data_size, rows, cols, then matrix data
                        conn.sendall(len(data_bytes).to_bytes(4, 'big'))
                        conn.sendall(self.rows.to_bytes(4, 'big'))
                        conn.sendall(self.cols.to_bytes(4, 'big'))
                        conn.sendall(data_bytes)
                        
                        print("Data sent!")
                except socket.timeout:
                    continue

def signal_handler(sig, frame):
    print("\nStopping...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    MatrixServer().start()

if __name__ == "__main__":
    main()

