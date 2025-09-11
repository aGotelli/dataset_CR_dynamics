import socket
import json
import numpy as np
import csv
import signal
import sys

class MatrixClient:
    def __init__(self, port=8080):
        self.port = port
        self.ready = False
        
    def handle_command(self, cmd, sock):
        if cmd["command"] == "setup":
            self.rows = cmd.get("matrix_rows", 1000)
            self.cols = cmd.get("matrix_cols", 50)
            self.ready = True
            return {"status": "ready"}
            
        elif cmd["command"] == "start" and self.ready:
            # Generate big matrix instantly
            print(f"Generating {self.rows}x{self.cols} matrix...")
            data = np.random.randn(self.rows, self.cols)
            filename = f"matrix_{self.rows}x{self.cols}.csv"
            
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(data)
            
            # Send file back
            with open(filename, 'rb') as f:
                file_data = f.read()
            
            # Send to server
            server_addr = sock.getpeername()[0]
            data_port = cmd.get("data_port", 12345)
            print(f"Sending {len(file_data)} bytes to {server_addr}:{data_port}")
            with socket.socket() as s:
                s.connect((server_addr, data_port))
                s.sendall(json.dumps({"filename": filename, "size": len(file_data)}).encode())
                s.sendall(file_data)
            print("Data sent!")
            
            return {"status": "completed"}
        
        return {"status": "error"}
    
    def start(self):
        with socket.socket() as s:
            s.bind(("0.0.0.0", self.port))
            s.listen()
            s.settimeout(1.0)  # 1 second timeout
            print(f"Matrix client on port {self.port}")
            
            while True:
                try:
                    conn, addr = s.accept()
                    print(f"Connection from {addr}")
                    with conn:
                        data = conn.recv(1024)
                        cmd = json.loads(data.decode())
                        print(f"Command: {cmd['command']}")
                        resp = self.handle_command(cmd, conn)
                        conn.sendall(json.dumps(resp).encode())
                except socket.timeout:
                    continue  # Keep listening

def signal_handler(sig, frame):
    print("\nStopping...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    MatrixClient().start()

if __name__ == "__main__":
    main()

