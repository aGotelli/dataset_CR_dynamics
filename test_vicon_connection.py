#!/usr/bin/env python3
"""
Test script to diagnose Vicon TCP connection issues
"""

import socket
import time

def test_vicon_connection():
    """Test connection to Vicon client"""
    vicon_ip = "192.168.10.2"
    vicon_port = 8080
    
    print(f"🔍 Testing connection to Vicon at {vicon_ip}:{vicon_port}")
    
    try:
        # First, try to ping the IP address
        import subprocess
        result = subprocess.run(['ping', '-n', '1', vicon_ip], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print(f"✅ Ping to {vicon_ip} successful")
        else:
            print(f"❌ Ping to {vicon_ip} failed")
            print(f"Output: {result.stdout}")
            return False
    except Exception as e:
        print(f"⚠️ Could not ping: {e}")
    
    # Test TCP connection
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(5.0)
            print(f"⏳ Attempting TCP connection...")
            sock.connect((vicon_ip, vicon_port))
            print(f"✅ TCP connection to {vicon_ip}:{vicon_port} successful!")
            
            # Try to send a simple status command (proper JSON format)
            import json
            test_command = {
                'command': 'status',
                'timestamp': time.time()
            }
            
            command_str = json.dumps(test_command)
            sock.send(command_str.encode('utf-8'))
            print(f"✅ Sent test command: {test_command}")
            
            # Try to receive response
            sock.settimeout(2.0)
            response = sock.recv(1024).decode('utf-8')
            print(f"✅ Received response: {response}")
            
            return True
            
    except socket.timeout:
        print(f"❌ TCP connection timeout - server not responding")
        return False
    except ConnectionRefusedError:
        print(f"❌ Connection refused - Vicon client not running or port blocked")
        return False
    except Exception as e:
        print(f"❌ TCP connection failed: {e}")
        return False

def check_network_config():
    """Check network configuration"""
    print("\n🔍 Checking network configuration...")
    
    try:
        import subprocess
        
        # Check network adapters
        result = subprocess.run(['ipconfig'], capture_output=True, text=True)
        print("Network adapters:")
        for line in result.stdout.split('\n'):
            if 'IPv4' in line or 'Ethernet' in line:
                print(f"  {line.strip()}")
                
        # Check if we can reach the subnet
        print(f"\n🔍 Checking if 192.168.10.x subnet is reachable...")
        result = subprocess.run(['route', 'print', '192.168.10.0'], 
                              capture_output=True, text=True)
        if '192.168.10' in result.stdout:
            print("✅ Route to 192.168.10.x subnet found")
        else:
            print("❌ No route to 192.168.10.x subnet")
            
    except Exception as e:
        print(f"⚠️ Could not check network config: {e}")

if __name__ == "__main__":
    print("🧪 Vicon Connection Diagnostic Tool")
    print("=" * 50)
    
    check_network_config()
    print("\n" + "=" * 50)
    
    if test_vicon_connection():
        print("\n✅ Vicon connection test PASSED")
        print("The Vicon client is running and reachable!")
    else:
        print("\n❌ Vicon connection test FAILED")
        print("\nTroubleshooting steps:")
        print("1. Make sure the Vicon client (vicon_client.py) is running on 192.168.10.2")
        print("2. Check if both computers are on the same network")
        print("3. Check firewall settings on both computers")
        print("4. Try running: telnet 192.168.10.2 8080")
        print("5. Verify the IP address is correct")
