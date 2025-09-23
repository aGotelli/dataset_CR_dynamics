"""
Cross-platform TwoWire I2C interface for Python
Supports both Windows (CH341 DLL) and Linux (native I2C)
"""
import platform
import os

# Platform detection and import appropriate implementation
if platform.system() == "Windows":
    try:
        from .twowire_windows import TwoWire
    except ImportError:
        # Fall back to absolute import when running as script
        from twowire_windows import TwoWire
elif platform.system() == "Linux":
    try:
        from .twowire_linux import TwoWire
    except ImportError:
        # Fall back to absolute import when running as script
        from twowire_linux import TwoWire
else:
    raise RuntimeError(f"Unsupported platform: {platform.system()}. Only Windows and Linux are supported.")

__all__ = ['TwoWire']
