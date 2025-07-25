"""
Vicon Client for Remote Data Acquisition
Runs on the Vicon PC (192.168.10.2) and waits for TCP commands
"""

import socket
import json
import time
import threading
import os
from datetime import datetime

# Import your Vicon interface
from simple_vicon import SimpleVicon

class ViconTCPClient:
    """TCP client that runs Vicon data acquisition based on remote commands"""
    
    def __init__(self, host="0.0.0.0", port=8080):
        self.host = host
        self.port = port
        self.vicon_sensor = None
        self.experiment_config = {}
        self.acquisition_thread = None
        self.is_ready = False
        self.data_file_path = None
        
    def setup_vicon(self, duration, vicon_host="localhost:801"):
        """Setup Vicon sensor with specified duration"""
        try:
            self.vicon_sensor = SimpleVicon(host=vicon_host, duration=duration)
            self.vicon_sensor.connect_and_setup()
            self.is_ready = True
            print(f"✅ Vicon sensor ready for {duration}s acquisition")
            return True
        except Exception as e:
            print(f"❌ Vicon setup failed: {e}")
            self.is_ready = False
            return False
    
    def start_acquisition(self, output_file):
        """Start Vicon data acquisition in a thread"""
        if not self.is_ready or not self.vicon_sensor:
            print("❌ Vicon not ready for acquisition")
            return False
        
        self.data_file_path = output_file
        
        def acquire_data():
            try:
                print(f"🚀 Starting Vicon acquisition...")
                success = self.vicon_sensor.acquire_data(output_file)
                if success:
                    print(f"✅ Vicon data saved to: {output_file}")
                else:
                    print(f"❌ Vicon acquisition failed")
            except Exception as e:
                print(f"❌ Vicon acquisition error: {e}")
        
        self.acquisition_thread = threading.Thread(target=acquire_data)
        self.acquisition_thread.start()
        return True
    
    def wait_for_completion(self):
        """Wait for acquisition to complete"""
        if self.acquisition_thread:
            self.acquisition_thread.join()
    
    def send_data_back(self, server_address, server_port=8081):
        """Send acquired data back to server PC"""
        if not self.data_file_path or not os.path.exists(self.data_file_path):
            print("❌ No data file to send back")
            return False
        
        try:
            with open(self.data_file_path, 'rb') as f:
                data = f.read()
            
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((server_address, server_port))
                
                # Send file info first
                file_info = {
                    "filename": os.path.basename(self.data_file_path),
                    "size": len(data),
                    "experiment": self.experiment_config.get("experiment_name", "unknown")
                }
                
                info_msg = json.dumps(file_info).encode('utf-8')
                sock.sendall(len(info_msg).to_bytes(4, 'big'))  # Send length first
                sock.sendall(info_msg)
                
                # Send file data
                sock.sendall(data)
                
                print(f"✅ Data sent back to server: {len(data)} bytes")
                return True
                
        except Exception as e:
            print(f"❌ Failed to send data back: {e}")
            return False
    
    def handle_command(self, command_data, client_socket):
        """Handle incoming TCP command"""
        command = command_data.get("command")
        response = {"status": "error", "message": "Unknown command"}
        
        if command == "setup":
            # Setup Vicon with specified parameters
            experiment_name = command_data.get("experiment_name", "vicon_experiment")
            duration = command_data.get("duration", 10)
            vicon_host = command_data.get("vicon_host", "localhost:801")
            
            self.experiment_config = {
                "experiment_name": experiment_name,
                "duration": duration,
                "vicon_host": vicon_host,
                "server_address": client_socket.getpeername()[0]  # Get server IP
            }
            
            success = self.setup_vicon(duration, vicon_host)
            if success:
                response = {"status": "ready", "message": f"Vicon ready for {duration}s"}
            else:
                response = {"status": "error", "message": "Vicon setup failed"}
        
        elif command == "start":
            # Start data acquisition
            if not self.is_ready:
                response = {"status": "error", "message": "Vicon not ready"}
            else:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"vicon_data_{timestamp}.csv"
                
                success = self.start_acquisition(filename)
                if success:
                    response = {"status": "started", "message": "Acquisition started"}
                else:
                    response = {"status": "error", "message": "Failed to start acquisition"}
        
        elif command == "stop":
            # Stop and send data back
            if self.vicon_sensor:
                self.vicon_sensor.stop()
            
            # Wait for acquisition to complete
            self.wait_for_completion()
            
            # Send data back to server
            server_address = self.experiment_config.get("server_address")
            if server_address and self.data_file_path:
                success = self.send_data_back(server_address)
                if success:
                    response = {"status": "completed", "message": "Data sent back to server"}
                else:
                    response = {"status": "completed", "message": "Acquisition done, but failed to send data back"}
            else:
                response = {"status": "completed", "message": "Acquisition completed"}
        
        elif command == "status":
            # Return current status
            status = "ready" if self.is_ready else "not_ready"
            response = {"status": status, "experiment": self.experiment_config}
        
        return response
    
    def start_server(self):
        """Start TCP server to listen for commands"""
        print(f"🔧 Starting Vicon TCP server on {self.host}:{self.port}")
        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((self.host, self.port))
            server_socket.listen(5)
            
            print(f"✅ Vicon client listening on {self.host}:{self.port}")
            print("Waiting for commands from main acquisition system...")
            
            while True:
                try:
                    client_socket, address = server_socket.accept()
                    print(f"📡 Connection from {address}")
                    
                    with client_socket:
                        # Receive command
                        data = client_socket.recv(1024)
                        if data:
                            try:
                                command_data = json.loads(data.decode('utf-8'))
                                print(f"📨 Received command: {command_data}")
                                
                                # Handle command
                                response = self.handle_command(command_data, client_socket)
                                
                                # Send response
                                response_data = json.dumps(response).encode('utf-8')
                                client_socket.sendall(response_data)
                                
                                print(f"📤 Sent response: {response}")
                                
                            except json.JSONDecodeError:
                                error_response = {"status": "error", "message": "Invalid JSON"}
                                client_socket.sendall(json.dumps(error_response).encode('utf-8'))
                
                except KeyboardInterrupt:
                    print("\n⚠️ Server interrupted by user")
                    break
                except Exception as e:
                    print(f"❌ Server error: {e}")
                    continue
                    
        print("🔌 Vicon TCP server stopped")

def main():
    """Main function for Vicon client"""
    print("="*60)
    print("VICON REMOTE DATA ACQUISITION CLIENT")
    print("="*60)
    print("This runs on the Vicon PC and waits for TCP commands")
    print("from the main acquisition system.")
    print("")
    
    # Configuration
    SERVER_HOST = "0.0.0.0"  # Listen on all interfaces
    SERVER_PORT = 8080
    
    client = ViconTCPClient(host=SERVER_HOST, port=SERVER_PORT)
    
    try:
        client.start_server()
    except KeyboardInterrupt:
        print("\n⚠️ Vicon client interrupted by user")
    except Exception as e:
        print(f"❌ Vicon client error: {e}")
    finally:
        if client.vicon_sensor:
            client.vicon_sensor.stop()
        print("🧹 Cleanup completed")

if __name__ == "__main__":
    main()
