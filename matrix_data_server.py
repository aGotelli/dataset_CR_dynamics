import socket
import json
import numpy as np
import csv
import signal
import sys

class MatrixServer:
    def __init__(self, port=8080, rows=1000, cols=50):
        self.port = port
        self.ready = False

        self.rows = rows
        self.cols = cols

        print(f"Generating {self.rows}x{self.cols} matrix...")
        self.data = np.random.randn(self.rows, self.cols)
        
    def handle_command(self, cmd, sock):
        if cmd["command"] == "setup":
            self.ready = True
            return {"status": "ready"}
            
        elif cmd["command"] == "start" and self.ready:
            # Send numpy data directly back through the same connection
            data_bytes = self.data.tobytes()
            print(f"Sending {len(data_bytes)} bytes matrix data")

            sock.sendall(len(data_bytes).to_bytes(4, 'big'))  # Send size first
            sock.sendall(data_bytes)  # Send raw numpy data
            print("Data sent!")
            
            return {"status": "completed"}
        
        return {"status": "error"}
    
    def start(self):
        with socket.socket() as s:
            s.bind(("0.0.0.0", self.port))
            s.listen()
            s.settimeout(1.0)  # 1 second timeout
            print(f"Matrix server on port {self.port}")

            while True:
                try:
                    conn, addr = s.accept()
                    print(f"Connection from {addr}")
                    with conn:
                        # Send matrix data immediately
                        data_bytes = self.data.tobytes()
                        print(f"Sending {len(data_bytes)} bytes matrix data")
                        conn.sendall(len(data_bytes).to_bytes(4, 'big'))  # Send size first
                        conn.sendall(data_bytes)  # Send raw numpy data
                        print("Data sent!")
                except socket.timeout:
                    continue  # Keep listening

def signal_handler(sig, frame):
    print("\nStopping...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    MatrixServer().start()

if __name__ == "__main__":
    main()

