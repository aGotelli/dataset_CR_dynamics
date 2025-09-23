"""
Windows implementation of TwoWire using CH341 DLL
"""
import ctypes
from ctypes import wintypes
import os
from typing import Optional, List

# Define UCHAR if not available in wintypes
if not hasattr(wintypes, 'UCHAR'):
    wintypes.UCHAR = ctypes.c_ubyte

class CH341Wrapper:
    """Python wrapper for CH341 DLL functions"""
    
    def __init__(self):
        self.dll = None
        self.device_index = 0
        
    def load_dll(self, dll_path: str = "CH341DLLA64.dll") -> bool:
        """Load the CH341 DLL"""
        try:
            self.dll = ctypes.WinDLL(dll_path)
            
            # Define function signatures
            self.dll.CH341OpenDevice.argtypes = [wintypes.ULONG]
            self.dll.CH341OpenDevice.restype = wintypes.BOOL
            
            self.dll.CH341CloseDevice.argtypes = [wintypes.ULONG]
            self.dll.CH341CloseDevice.restype = None
            
            self.dll.CH341GetVersion.argtypes = []
            self.dll.CH341GetVersion.restype = wintypes.ULONG
            
            self.dll.CH341SetStream.argtypes = [wintypes.ULONG, wintypes.ULONG]
            self.dll.CH341SetStream.restype = wintypes.BOOL
            
            self.dll.CH341ReadI2C.argtypes = [
                wintypes.ULONG,  # device index
                wintypes.UCHAR,  # device address
                wintypes.UCHAR,  # register address
                ctypes.POINTER(wintypes.UCHAR),  # buffer
                wintypes.ULONG   # length
            ]
            self.dll.CH341ReadI2C.restype = wintypes.BOOL
            
            self.dll.CH341WriteI2C.argtypes = [
                wintypes.ULONG,  # device index
                wintypes.UCHAR,  # device address
                wintypes.UCHAR,  # register address
                ctypes.POINTER(wintypes.UCHAR),  # buffer
                wintypes.ULONG   # length
            ]
            self.dll.CH341WriteI2C.restype = wintypes.BOOL
            
            return True
        except Exception as e:
            print(f"Failed to load CH341 DLL: {e}")
            return False
    
    def open_device(self, index: int = 0) -> bool:
        """Open CH341 device"""
        if not self.dll:
            return False
        return bool(self.dll.CH341OpenDevice(index))
    
    def close_device(self, index: int = 0):
        """Close CH341 device"""
        if self.dll:
            self.dll.CH341CloseDevice(index)
    
    def get_version(self) -> int:
        """Get driver version"""
        if not self.dll:
            return 0
        return self.dll.CH341GetVersion()
    
    def set_stream(self, index: int, mode: int) -> bool:
        """Set communication mode (1 = I2C)"""
        if not self.dll:
            return False
        return bool(self.dll.CH341SetStream(index, mode))
    
    def read_i2c(self, index: int, device_addr: int, reg_addr: int, length: int) -> Optional[bytes]:
        """Read from I2C device"""
        if not self.dll:
            return None
        
        buffer = (wintypes.UCHAR * length)()
        success = self.dll.CH341ReadI2C(index, device_addr, reg_addr, buffer, length)
        
        if success:
            return bytes(buffer)
        return None
    
    def write_i2c(self, index: int, device_addr: int, reg_addr: int, data: bytes) -> bool:
        """Write to I2C device"""
        if not self.dll:
            return False
        
        buffer = (wintypes.UCHAR * len(data))(*data)
        return bool(self.dll.CH341WriteI2C(index, device_addr, reg_addr, buffer, len(data)))


class TwoWire:
    """Windows TwoWire implementation using CH341"""
    
    def __init__(self, device_name: str = "CH341"):
        self.device_name = device_name
        self.ch341 = CH341Wrapper()
        self.is_initialized = False
        self.target_address = 0
        self.last_register_address = 0
        self.tx_buffer = bytearray()
        self.rx_buffer = bytearray()
    
    def begin(self) -> bool:
        """Initialize I2C interface"""
        if not self.ch341.load_dll():
            print("Failed to load CH341 DLL")
            return False
        
        print(f"CH341 Driver Version: 0x{self.ch341.get_version():x}")
        
        if not self.ch341.open_device(0):
            print("Failed to open CH341 device 0")
            return False
        
        if not self.ch341.set_stream(0, 1):  # Set I2C mode
            print("Failed to set I2C mode")
            return False
        
        self.is_initialized = True
        self.tx_buffer.clear()
        self.rx_buffer.clear()
        print("CH341 I2C interface initialized successfully")
        return True
    
    def end(self):
        """Close I2C interface"""
        if self.is_initialized:
            self.ch341.close_device(0)
            self.is_initialized = False
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
        if not self.is_initialized:
            return -1
        
        if len(self.tx_buffer) == 0:
            return 0
        
        if len(self.tx_buffer) == 1:
            # Just setting register address
            self.last_register_address = self.tx_buffer[0]
            return 0
        elif len(self.tx_buffer) >= 2:
            # Write data to register
            reg_addr = self.tx_buffer[0]
            data = bytes(self.tx_buffer[1:])
            
            success = self.ch341.write_i2c(0, self.target_address, reg_addr, data)
            return 0 if success else -1
        
        return 0
    
    def request_from(self, address: int, num_bytes: int, stop: bool = True) -> int:
        """Request data from I2C device"""
        if not self.is_initialized:
            return 0
        
        data = self.ch341.read_i2c(0, address, self.last_register_address, num_bytes)
        
        if data is None:
            return 0
        
        self.rx_buffer.extend(data)
        return len(data)
    
    def read(self) -> int:
        """Read byte from receive buffer"""
        if len(self.rx_buffer) == 0:
            return 0xFF  # Return -1 as unsigned byte
        
        return self.rx_buffer.pop(0)
    
    def available(self) -> int:
        """Number of bytes available to read"""
        return len(self.rx_buffer)
