import socket
import json
import numpy as np
import signal
import sys
import time

from Vicon_py_v2 import ViconV2

class ViconTCPServer:
    def __init__(self, port=8080):
        self.port = port
        self.duration = 0
        self.data = None
        self.column_info = None

        self.vicon = ViconV2()
        
    def setup(self):
        self.vicon.prepare(duration=self.duration)
        
        
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
                            
                            if command == "setup":
                                self.duration = cmd.get("duration", 0)
                                self.setup()
                                conn.sendall(b'{"status": "Memory allocated"}')

                            if command == "start":
                                conn.sendall(b'{"status": "Recording started"}')
                                self.vicon.acquire()

                            elif command == "get_data":
                                conn.sendall(b'{"status": "preparing data\nsending data number of bytes, rows and columns.\nFollowing by header bytes and finally the header and data bytes"}')

                                data = self.vicon.data_matrix
                                header = self.vicon.column_names

                                data_bytes = data.tobytes()
                                column_bytes = json.dumps(header).encode()

                                conn.sendall(len(data_bytes).to_bytes(4, 'big'))
                                conn.sendall((data.shape[0]).to_bytes(4, 'big'))  # rows
                                conn.sendall((data.shape[1]).to_bytes(4, 'big'))  # cols
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
    ViconTCPServer().start()

if __name__ == "__main__":
    main()