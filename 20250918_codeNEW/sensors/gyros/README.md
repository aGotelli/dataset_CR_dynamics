# Python Minimal Working Example - ISM330DHCX Interface

This directory contains a Python implementation of the cross-platform I2C interface for the ISM330DHCX sensor.

## Files

- **`wire.py`** - Main module with platform detection
- **`twowire_windows.py`** - Windows implementation using CH341 DLL via ctypes
- **`twowire_linux.py`** - Linux implementation using native I2C
- **`test_simple.py`** - Minimal working example (basic I2C test)
- **`test_sensor.py`** - Advanced example with actual sensor data reading

## Requirements

### Windows
- Python 3.6+
- CH341DLLA64.dll in the same directory or system PATH
- CH341 driver installed

### Linux  
- Python 3.6+
- I2C permissions (add user to i2c group or run with sudo)
- CH341 driver or native I2C interface

## Usage

### Simple Test (Minimal Working Example)
```python
python test_simple.py
```

### Advanced Sensor Test
```python
python test_sensor.py
```

### Using in Your Code
```python
from wire import TwoWire

# Initialize (platform automatically detected)
wire = TwoWire()
wire.begin()

# Read WHO_AM_I register
wire.begin_transmission(0x6A)
wire.write(0x0F)  # WHO_AM_I register
wire.end_transmission()

if wire.request_from(0x6A, 1) == 1:
    who_am_i = wire.read()
    print(f"WHO_AM_I: 0x{who_am_i:02x}")

wire.end()
```

## API Compatibility

The Python TwoWire class provides the same methods as the C++ version:

| Method | Description |
|--------|-------------|
| `begin()` | Initialize I2C interface |
| `end()` | Close I2C interface |
| `begin_transmission(addr)` | Start I2C transaction |
| `write(data)` | Write data to buffer |
| `end_transmission()` | Send buffered data |
| `request_from(addr, length)` | Request data from device |
| `read()` | Read byte from receive buffer |
| `available()` | Number of bytes available |

## Platform Detection

The code automatically detects the platform:
- **Windows**: Uses `ctypes` to call CH341 DLL functions
- **Linux**: Uses native I2C file operations via `os` module

## Error Handling

- Returns appropriate error codes (-1 for errors, 0 for success)
- Prints diagnostic messages for debugging
- Handles exceptions gracefully
