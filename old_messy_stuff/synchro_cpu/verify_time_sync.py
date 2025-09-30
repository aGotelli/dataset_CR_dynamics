#!/usr/bin/env python3
"""
Time Synchronization Verification Script

This script helps verify that time synchronization between computers is working correctly.
Run this on both computers simultaneously to check sync quality.
"""

import time
import socket
import json
from datetime import datetime
import threading
import subprocess
import sys

class TimeSyncVerifier:
    def __init__(self, role='client', server_ip='192.168.1.100', port=12345):
        """
        Initialize time sync verifier
        
        Args:
            role: 'server' or 'client'
            server_ip: IP address of the time server computer
            port: Port for communication between computers
        """
        self.role = role
        self.server_ip = server_ip
        self.port = port
        self.running = False
        
    def get_windows_time_status(self):
        """Get Windows Time service status and offset"""
        try:
            # Get W32Time status
            result = subprocess.run(['w32tm', '/query', '/status'], 
                                  capture_output=True, text=True, timeout=10)
            status_output = result.stdout
            
            # Parse offset from output
            offset = None
            for line in status_output.split('\n'):
                if 'Last Successful Sync Time:' in line:
                    last_sync = line.split(':', 1)[1].strip()
                elif 'Offset:' in line:
                    offset_str = line.split(':', 1)[1].strip()
                    # Extract numeric offset (remove 's' suffix)
                    if 's' in offset_str:
                        offset = float(offset_str.replace('s', ''))
            
            return {
                'status': 'success' if result.returncode == 0 else 'error',
                'offset_seconds': offset,
                'full_output': status_output
            }
        except Exception as e:
            return {
                'status': 'error',
                'error': str(e),
                'offset_seconds': None
            }
    
    def run_server(self, duration=30):
        """Run as time server - send timestamps to client"""
        print(f"Starting time sync verification server on port {self.port}")
        print(f"Will run for {duration} seconds")
        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('', self.port))
            server_socket.listen(1)
            server_socket.settimeout(1.0)  # Non-blocking accept
            
            print("Waiting for client connection...")
            
            start_time = time.time()
            client_socket = None
            
            while time.time() - start_time < duration:
                try:
                    client_socket, addr = server_socket.accept()
                    print(f"Client connected from {addr}")
                    break
                except socket.timeout:
                    continue
            
            if client_socket is None:
                print("No client connected. Exiting.")
                return
            
            # Send timestamps
            sample_count = 0
            with client_socket:
                while time.time() - start_time < duration:
                    timestamp = time.time()
                    message = {
                        'timestamp': timestamp,
                        'sample': sample_count,
                        'computer': 'server'
                    }
                    
                    try:
                        client_socket.send(json.dumps(message).encode() + b'\n')
                        sample_count += 1
                        time.sleep(0.1)  # 10 Hz sampling
                    except Exception as e:
                        print(f"Error sending data: {e}")
                        break
        
        print(f"Server finished. Sent {sample_count} samples.")
    
    def run_client(self, duration=30):
        """Run as client - receive timestamps and compare"""
        print(f"Connecting to time server at {self.server_ip}:{self.port}")
        
        time_differences = []
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                client_socket.settimeout(5.0)
                client_socket.connect((self.server_ip, self.port))
                print("Connected to server")
                
                start_time = time.time()
                buffer = ""
                
                while time.time() - start_time < duration:
                    try:
                        data = client_socket.recv(1024).decode()
                        if not data:
                            break
                        
                        buffer += data
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            if line:
                                try:
                                    message = json.loads(line)
                                    server_time = message['timestamp']
                                    client_time = time.time()
                                    
                                    # Calculate time difference
                                    diff = client_time - server_time
                                    time_differences.append(diff)
                                    
                                    if len(time_differences) % 50 == 0:  # Print every 5 seconds
                                        avg_diff = sum(time_differences) / len(time_differences)
                                        print(f"Samples: {len(time_differences)}, "
                                              f"Avg difference: {avg_diff*1000:.3f} ms")
                                        
                                except json.JSONDecodeError:
                                    continue
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(f"Error receiving data: {e}")
                        break
        
        except Exception as e:
            print(f"Connection error: {e}")
            return
        
        # Analyze results
        if time_differences:
            avg_diff = sum(time_differences) / len(time_differences)
            max_diff = max(time_differences)
            min_diff = min(time_differences)
            std_dev = (sum((x - avg_diff)**2 for x in time_differences) / len(time_differences))**0.5
            
            print(f"\n=== Time Synchronization Analysis ===")
            print(f"Samples collected: {len(time_differences)}")
            print(f"Average time difference: {avg_diff*1000:.3f} ms")
            print(f"Standard deviation: {std_dev*1000:.3f} ms")
            print(f"Min difference: {min_diff*1000:.3f} ms")
            print(f"Max difference: {max_diff*1000:.3f} ms")
            print(f"Range: {(max_diff-min_diff)*1000:.3f} ms")
            
            # Quality assessment
            if abs(avg_diff*1000) < 1.0 and std_dev*1000 < 0.5:
                print("✅ EXCELLENT: Time sync is very good (< 1ms average, < 0.5ms jitter)")
            elif abs(avg_diff*1000) < 5.0 and std_dev*1000 < 2.0:
                print("✅ GOOD: Time sync is acceptable (< 5ms average, < 2ms jitter)")
            elif abs(avg_diff*1000) < 10.0:
                print("⚠️  FAIR: Time sync has noticeable offset (< 10ms average)")
            else:
                print("❌ POOR: Time sync is not working well (> 10ms average)")
        else:
            print("No time data collected")
        
        # Get Windows Time status
        print(f"\n=== Windows Time Service Status ===")
        status = self.get_windows_time_status()
        if status['status'] == 'success':
            if status['offset_seconds'] is not None:
                print(f"W32Time offset: {status['offset_seconds']*1000:.3f} ms")
            else:
                print("Could not parse W32Time offset")
        else:
            print(f"Error getting W32Time status: {status.get('error', 'Unknown')}")
    
    def check_network_connectivity(self):
        """Check if computers can communicate"""
        print(f"Testing network connectivity to {self.server_ip}...")
        
        try:
            # Test ping
            result = subprocess.run(['ping', '-n', '3', self.server_ip], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                print("✅ Ping successful")
                # Extract average ping time
                for line in result.stdout.split('\n'):
                    if 'Average' in line:
                        print(f"Network latency: {line.strip()}")
            else:
                print(f"❌ Ping failed: {result.stderr}")
                return False
        except Exception as e:
            print(f"❌ Ping error: {e}")
            return False
        
        # Test NTP connectivity (if client)
        if self.role == 'client':
            try:
                result = subprocess.run(['w32tm', '/stripchart', f'/computer:{self.server_ip}', '/samples:3'], 
                                      capture_output=True, text=True, timeout=15)
                if result.returncode == 0:
                    print("✅ NTP connectivity successful")
                    print("NTP response summary:")
                    for line in result.stdout.split('\n'):
                        if 'offset' in line.lower() or 'error' in line.lower():
                            print(f"  {line.strip()}")
                else:
                    print(f"⚠️  NTP test had issues: {result.stderr}")
            except Exception as e:
                print(f"⚠️  NTP test error: {e}")
        
        return True

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  Server computer: python verify_time_sync.py server")
        print("  Client computer: python verify_time_sync.py client [server_ip]")
        print("\nExample:")
        print("  Computer A: python verify_time_sync.py server")
        print("  Computer B: python verify_time_sync.py client 192.168.1.100")
        return
    
    role = sys.argv[1].lower()
    server_ip = sys.argv[2] if len(sys.argv) > 2 else '192.168.1.100'
    
    if role not in ['server', 'client']:
        print("Role must be 'server' or 'client'")
        return
    
    verifier = TimeSyncVerifier(role=role, server_ip=server_ip)
    
    # Test network connectivity first
    if not verifier.check_network_connectivity():
        print("Network connectivity test failed. Check your network configuration.")
        return
    
    print(f"\nStarting time synchronization verification as {role}")
    
    if role == 'server':
        verifier.run_server(duration=60)
    else:
        verifier.run_client(duration=60)

if __name__ == "__main__":
    main()
