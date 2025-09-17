import socket
import json
import numpy as np
import signal
import sys
import time

class MatrixServer:
    def __init__(self, port=8080):
        self.port = port
        self.duration = 0
        self.data = None
        self.column_info = None
        
    def external_operation(self):
        print(f"Recording for {self.duration}s...")
        time.sleep(self.duration)
        self.data = np.linspace(1, 50, 50).reshape(10, 5)
        self.column_info = [f"sensor_{i+1}" for i in range(5)]
        print("Recording complete")
        
    def start(self):
        with socket.socket() as s:
            s.bind(("0.0.0.0", self.port))
            s.listen()
            s.settimeout(1.0)  # 1 second timeout
            print(f"Server on port {self.port}")

            while True:
                try:
                    conn, addr = s.accept()
                    print(f"Connected: {addr}")
                    
                    with conn:
                        while True:
                            data = conn.recv(1024)
                            if not data:
                                break
                                
                            cmd = json.loads(data.decode())
                            command = cmd.get("command")
                            
                            if command == "start":
                                self.duration = cmd.get("duration", 5)
                                conn.sendall(b'{"status": "started"}')
                                self.external_operation()
                                
                            elif command == "save":
                                conn.sendall(b'{"status": "sending"}')
                                
                                data_bytes = self.data.tobytes()
                                column_bytes = json.dumps(self.column_info).encode()
                                
                                conn.sendall(len(data_bytes).to_bytes(4, 'big'))
                                conn.sendall((10).to_bytes(4, 'big'))  # rows
                                conn.sendall((5).to_bytes(4, 'big'))   # cols
                                conn.sendall(len(column_bytes).to_bytes(4, 'big'))
                                conn.sendall(column_bytes)
                                conn.sendall(data_bytes)
                                print("Data sent")
                except socket.timeout:
                    continue  # Check for Ctrl+C and continue listening

def signal_handler(sig, frame):
    print("\nStopping server...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    MatrixServer().start()

if __name__ == "__main__":
    main()