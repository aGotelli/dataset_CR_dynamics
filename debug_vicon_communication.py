#!/usr/bin/env python3
"""
Debug the exact difference between test_connection and synchronized_acquisition
"""

import socket
import json
import time

def test_status_command():
    """Test the simple status command like test_connection does"""
    print("🧪 Testing simple status command...")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(10.0)
            sock.connect(("192.168.10.2", 8080))
            
            test_command = {'command': 'status'}
            command_str = json.dumps(test_command)
            sock.send(command_str.encode('utf-8'))
            print(f"✅ Sent: {test_command}")
            
            response = sock.recv(1024).decode('utf-8')
            print(f"✅ Received: {response}")
            return True
    except Exception as e:
        print(f"❌ Failed: {e}")
        return False

def test_setup_command():
    """Test the setup command like synchronized_acquisition does"""
    print("\n🧪 Testing setup command...")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(10.0)
            sock.connect(("192.168.10.2", 8080))
            
            setup_command = {
                "command": "setup",
                "experiment_name": "test_experiment",
                "duration": 10,
                "vicon_host": "localhost:801",
                "server_time": time.time(),
                "data_port": 12345
            }
            
            message = json.dumps(setup_command).encode('utf-8')
            print(f"📤 Sending: {setup_command}")
            sock.sendall(message)
            
            response = sock.recv(1024).decode('utf-8')
            print(f"📥 Received: {response}")
            return json.loads(response)
    except Exception as e:
        print(f"❌ Failed: {e}")
        return None

def test_connection_like_sync_acquisition():
    """Test exactly like synchronized_acquisition does"""
    print("\n🧪 Testing like synchronized_acquisition...")
    
    def send_vicon_command(command_data, timeout=10.0):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(timeout)
                print(f"🔗 Connecting to Vicon at 192.168.10.2:8080 (timeout: {timeout}s)")
                sock.connect(("192.168.10.2", 8080))
                
                message = json.dumps(command_data).encode('utf-8')
                print(f"📤 Sending command: {command_data}")
                sock.sendall(message)
                
                response = sock.recv(1024).decode('utf-8')
                print(f"📥 Received response: {response}")
                return json.loads(response)
        except socket.timeout:
            print(f"❌ Vicon TCP communication timed out after {timeout}s")
            return None
        except ConnectionRefusedError:
            print(f"❌ Vicon connection refused - client not running?")
            return None
        except Exception as e:
            print(f"❌ Vicon TCP communication failed: {e}")
            return None
    
    # Test status
    print("Testing status command...")
    response = send_vicon_command({"command": "status"})
    if not response:
        return False
        
    # Test setup
    print("\nTesting setup command...")
    vicon_command = {
        "command": "setup",
        "experiment_name": "test_experiment", 
        "duration": 10,
        "vicon_host": "localhost:801",
        "server_time": time.time(),
        "data_port": 12345
    }
    response = send_vicon_command(vicon_command)
    print(f"Setup response: {response}")
    
    return response and response.get("status") == "ready"

if __name__ == "__main__":
    print("🔍 DEBUGGING VICON COMMUNICATION DIFFERENCES")
    print("=" * 60)
    
    print("1. Testing simple method (like test_connection)...")
    if test_status_command():
        print("✅ Simple method works")
    else:
        print("❌ Simple method fails")
    
    print("\n2. Testing setup method...")
    result = test_setup_command()
    if result:
        print("✅ Setup method works")
    else:
        print("❌ Setup method fails")
    
    print("\n3. Testing exactly like synchronized_acquisition...")
    if test_connection_like_sync_acquisition():
        print("✅ Synchronized acquisition method works")
    else:
        print("❌ Synchronized acquisition method fails")
