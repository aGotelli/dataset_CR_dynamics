import socket
import json
import sys
import time

class SparkfunClient:

    def __init__(self):
        print(f"🔧 Creating SparkfunClient instance")
        self.host = 'localhost'
        self.port = 9999
        self.socket = None
        """Try to connect to the TCP server"""
        try:
            self.socket = socket.socket()
            self.socket.connect((self.host, self.port))
            print(f"Connected to {self.host}:{self.port}")
        except ConnectionRefusedError:
            print(f"Could not connect to {self.host}:{self.port}. Make sure the Sparkfun TCP server is running.")
            raise
        except Exception as e:
            print(f"Connection error: {e}")
            raise

    def send_setup(self, folder_path, duration=10):
        """Send setup message with folder path and duration"""
        try:
            self.duration = duration
            self.folder_path = folder_path
            print(f"Sparkfun - Setting up recording: folder='{folder_path}', duration={duration} seconds...")
            setup_cmd = {"command": "setup", "folder": folder_path, "duration": duration}
            self.socket.sendall(json.dumps(setup_cmd).encode())
            response = self.socket.recv(1024).decode()
            print(f"Server response: {response}")
            return True
        except Exception as e:
            print(f"Setup error: {e}")
            return False
    
    def start_recording(self):
        """Start recording"""
        try:
            start_cmd = {"command": "start"}
            self.socket.sendall(json.dumps(start_cmd).encode())
            response = self.socket.recv(1024).decode()
            print(f"Server response: {response}")
            print(f"Sparkfun - Recording started! Duration: {self.duration} seconds")
            return True
        except Exception as e:
            print(f"Start recording error: {e}")
            return False
    
    def acquire_data(self):
        """
        Complete acquisition workflow:
        1. Start recording
        2. Wait for completion 
        3. Server handles file saving automatically
        """
        print(f"Sparkfun - Starting recording now...")
        
        # Start recording
        self.start_recording()
        # Wait for recording to complete (server saves files automatically)
        time.sleep(self.duration + 1)  # Extra time for completion
        
        print(f"Sparkfun - Recording completed!")
    
    def close(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            print("Connection closed")

def main():
    """Example usage of the SparkfunClient"""
    folder = "client_test_data_saving"
    duration = 5
    
    try:
        client = SparkfunClient()
        
        # Setup
        if client.send_setup(folder, duration):
            print("Setup successful!")
            
            # Start recording
            if client.start_recording():
                print("Recording started successfully!")
            else:
                print("Failed to start recording")
        else:
            print("Setup failed")
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'client' in locals():
            client.close()

if __name__ == "__main__":
    main()