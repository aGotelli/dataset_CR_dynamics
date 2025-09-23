"""
Linux implementation of TwoWire using native I2C
"""
import os
import fcntl
from typing import Optional
import struct

# I2C constants
I2C_SLAVE = 0x0703

class TwoWire:
    """Linux TwoWire implementation using native I2C"""
    
    def __init__(self, device_path: str = "/dev/i2c-1"):
        self.device_path = device_path
        self.fd = -1
        self.target_address = 0
        self.tx_buffer = bytearray()
        self.rx_buffer = bytearray()
    
    def begin(self) -> bool:
        """Initialize I2C interface"""
        try:
            if self.fd >= 0:
                os.close(self.fd)
            
            self.fd = os.open(self.device_path, os.O_RDWR)
            if self.fd < 0:
                print(f"Failed to open I2C device: {self.device_path}")
                return False
            
            self.tx_buffer.clear()
            self.rx_buffer.clear()
            print(f"Linux I2C interface initialized: {self.device_path}")
            return True
        except Exception as e:
            print(f"Failed to initialize I2C: {e}")
            return False
    
    def end(self):
        """Close I2C interface"""
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1
        self.tx_buffer.clear()
        self.rx_buffer.clear()
    
    def begin_transmission(self, address: int):
        """Start I2C transmission to device"""
        self.target_address = address
        self.tx_buffer.clear()
    
    def write(self, data):
        """Write data to transmission buffer"""
        if isinstance(data, int):
            self.tx_buffer.append(data)
        elif isinstance(data, (bytes, bytearray)):
            self.tx_buffer.extend(data)
        elif isinstance(data, list):
            self.tx_buffer.extend(data)
    
    def end_transmission(self, stop: bool = True) -> int:
        """Send buffered data"""
        if self.fd < 0:
            return -1
        
        try:
            # Set I2C slave address
            fcntl.ioctl(self.fd, I2C_SLAVE, self.target_address)
            
            # Write data
            if len(self.tx_buffer) > 0:
                written = os.write(self.fd, bytes(self.tx_buffer))
                if written != len(self.tx_buffer):
                    print("Failed to write all bytes")
                    return -1
            
            return 0  # Success
        except Exception as e:
            print(f"I2C write error: {e}")
            return -1
    
    def request_from(self, address: int, num_bytes: int, stop: bool = True) -> int:
        """Request data from I2C device"""
        if self.fd < 0:
            return 0
        
        try:
            # Set I2C slave address
            fcntl.ioctl(self.fd, I2C_SLAVE, address)
            
            # Read data
            data = os.read(self.fd, num_bytes)
            self.rx_buffer.extend(data)
            
            return len(data)
        except Exception as e:
            print(f"I2C read error: {e}")
            return 0
    
    def read(self) -> int:
        """Read byte from receive buffer"""
        if len(self.rx_buffer) == 0:
            return 0xFF  # Return -1 as unsigned byte
        
        return self.rx_buffer.pop(0)
    
    def available(self) -> int:
        """Number of bytes available to read"""
        return len(self.rx_buffer)
