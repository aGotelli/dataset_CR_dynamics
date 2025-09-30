#!/usr/bin/env python3
"""
Minimal Motor Test - Test the -12.5 fix
"""
import sys
sys.path.append('sensors')

from sensors.simple_motor_tension_measure import MotorWithMark10
import can

def test_motor():
    """Test single motor initialization"""
    print("🔧 Testing motor fixes...")
    
    try:
        # Create CAN bus
        bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
        
        # Define simple test trajectory
        def test_trajectory(t):
            return 0.0  # No movement for test
        
        # Test motor configuration
        motor_cfg = [3, "COM5", test_trajectory]
        
        # Create motor (this will test the fixes)
        motor = MotorWithMark10(motor_cfg, bus, 254, 5, 50)
        
        print(f"✅ Motor initialized with home position: {motor.home_position}")
        
        # Test getting angle a few times
        for i in range(3):
            angle = motor.get_motor_angle()
            print(f"Attempt {i+1} - Current angle: {angle}")
        
        # Cleanup
        motor.cleanup()
        bus.shutdown()
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_motor()