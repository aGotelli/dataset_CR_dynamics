import socket
import json
import threading

def data_receiver(port=12345):
    """Listen for incoming data files"""
    with socket.socket() as s:
        s.bind(("0.0.0.0", port))
        s.listen()
        print(f"Data receiver on port {port}")
        
        conn, addr = s.accept()
        with conn:
            # Get file info
            info_len = int.from_bytes(conn.recv(4), 'big')
            info = json.loads(conn.recv(info_len).decode())
            print(f"Receiving: {info['filename']} ({info['size']} bytes)")
            
            # Get file data
            data = conn.recv(info['size'])
            with open(info['filename'], 'wb') as f:
                f.write(data)
            print(f"Saved: {info['filename']}")

def main():
    server_host = input("Server IP (localhost): ") or "localhost"
    server_port = int(input("Server port (8080): ") or "8080")
    
    # Start data receiver in background
    threading.Thread(target=data_receiver, daemon=True).start()
    
    with socket.socket() as s:
        s.connect((server_host, server_port))
        
        # Setup
        setup_cmd = {
            "command": "setup",
            "matrix_rows": int(input("Matrix rows (1000): ") or "1000"),
            "matrix_cols": int(input("Matrix cols (50): ") or "50"),
            "data_port": 12345
        }
        s.sendall(json.dumps(setup_cmd).encode())
        resp = json.loads(s.recv(1024).decode())
        print(f"Setup: {resp['status']}")
    
    input("Press Enter to start...")
    
    with socket.socket() as s:
        s.connect((server_host, server_port))
        
        # Start
        start_cmd = {"command": "start"}
        s.sendall(json.dumps(start_cmd).encode())
        resp = json.loads(s.recv(1024).decode())
        print(f"Start: {resp['status']}")
    
    input("Press Enter to exit...")

if __name__ == "__main__":
    main()
