"""
Test Mark10 reading in the same way as synchronized_acquisition
"""
from simple_mark10_clean import SimpleMark10
import time

def test_mark10_reading():
    print("Testing Mark10 sensors...")
    
    # Setup Mark10 sensors like synchronized_acquisition
    MARK10_PORTS = ["COM4", "COM5"]
    mark10_sensors = []
    
    for i, port in enumerate(MARK10_PORTS):
        try:
            sensor = SimpleMark10(port, 350)
            mark10_sensors.append(sensor)
            print(f"✅ Mark-10 sensor {i+1} on {port} ready")
        except Exception as e:
            print(f"❌ Mark-10 sensor {i+1} on {port} failed: {e}")
            return
    
    # Test reading like simple_pretension
    print("\nTesting direct serial reading...")
    for i, sensor in enumerate(mark10_sensors):
        port = MARK10_PORTS[i]
        print(f"\nTesting sensor {i+1} on {port}:")
        try:
            import re
            print(f"  Serial port open: {sensor.serial.is_open}")
            sensor.serial.write("?\r".encode())
            response = sensor.serial.readline().decode('utf-8', errors='ignore').strip()
            print(f"  Raw response: '{response}'")
            
            match = re.search(r'-?\d+(\.\d+)?', response)
            if match:
                value = float(match.group(0))
                print(f"  Parsed value: {value} N")
            else:
                print(f"  Could not parse value from: '{response}'")
                
        except Exception as e:
            print(f"  Error: {e}")
            import traceback
            traceback.print_exc()
    
    # Cleanup
    for sensor in mark10_sensors:
        try:
            sensor.stop()
        except:
            pass

if __name__ == "__main__":
    test_mark10_reading()
