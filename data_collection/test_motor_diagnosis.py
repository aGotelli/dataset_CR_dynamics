#!/usr/bin/env python3
"""
Motor Diagnosis Tool
Test script to diagnose the -12.5 position issue
"""
import can
import time
import sys
import os

# Add the sensors directory to the path
sys.path.append('sensors')
sys.path.append('sensors/cybergear')
from sensors.cybergear.pcan_cybergear import CANMotorController

def diagnose_motor(motor_id=3, main_can_id=254):
    """
    Diagnose motor communication and position reading
    """
    print(f"🔧 Starting motor diagnosis for Motor ID: {motor_id}")
    
    try:
        # Create CAN bus
        bus = can.interface.Bus(interface="candle", channel=0, bitrate=1000000)
        print("✅ CAN bus created successfully")
        
        # Create motor controller
        motor = CANMotorController(bus, motor_id=motor_id, main_can_id=main_can_id)
        print(f"✅ Motor controller created (ID: {motor_id})")
        
        # Check motor limits
        print(f"📊 Motor position limits: {motor.P_MIN} to {motor.P_MAX} radians")
        
        # Try to read device ID first
        print("\n🔍 Testing device ID request...")
        try:
            motor.clear_can_rx()
            data, arb_id = motor.send_receive_can_message(
                motor.CmdModes.GET_DEVICE_ID, main_can_id, [0]*8
            )
            if data is not None:
                print(f"✅ Device ID response: data={data}, arb_id={hex(arb_id) if arb_id else None}")
            else:
                print("❌ No device ID response")
        except Exception as e:
            print(f"❌ Device ID request failed: {e}")
        
        # Try motor feedback multiple times
        print("\n🔍 Testing motor feedback requests...")
        for i in range(5):
            try:
                motor.clear_can_rx()
                print(f"  Attempt {i+1}:")
                data, arb_id = motor.send_receive_can_message(
                    motor.CmdModes.MOTOR_FEEDBACK, main_can_id, [0]*8
                )
                
                if data is not None:
                    raw_pos_uint = (data[0] << 8) + data[1]
                    print(f"    ✅ Raw data: {data}")
                    print(f"    ✅ Raw position uint: {raw_pos_uint}")
                    print(f"    ✅ Arbitration ID: {hex(arb_id) if arb_id else None}")
                    
                    # Parse the message
                    motor_id_parsed, position, velocity, torque, temperature = motor.parse_received_msg(data, arb_id)
                    print(f"    ✅ Parsed - Motor ID: {motor_id_parsed}, Position: {position:.4f} rad")
                    print(f"    ✅ Velocity: {velocity:.4f} rad/s, Torque: {torque:.4f} Nm, Temp: {temperature:.1f}°C")
                    
                    # Check if it's the problematic -12.5
                    if abs(position + 12.5) < 0.001:  # Close to -12.5
                        print(f"    ⚠️  DETECTED -12.5 ISSUE! Raw uint was: {raw_pos_uint}")
                        if raw_pos_uint == 0:
                            print(f"    ⚠️  Raw uint is 0 - this suggests no valid motor response")
                    
                else:
                    print(f"    ❌ No data received on attempt {i+1}")
                    
                time.sleep(0.1)  # Small delay between attempts
                
            except Exception as e:
                print(f"    ❌ Feedback request {i+1} failed: {e}")
        
        # Test enabling the motor
        print("\n🔍 Testing motor enable...")
        try:
            motor.enable()
            print("✅ Motor enable command sent")
            time.sleep(0.5)
            
            # Try feedback again after enabling
            print("🔍 Testing feedback after enable...")
            motor.clear_can_rx()
            data, arb_id = motor.send_receive_can_message(
                motor.CmdModes.MOTOR_FEEDBACK, main_can_id, [0]*8
            )
            
            if data is not None:
                motor_id_parsed, position, velocity, torque, temperature = motor.parse_received_msg(data, arb_id)
                print(f"✅ After enable - Position: {position:.4f} rad")
            else:
                print("❌ No feedback after enable")
                
        except Exception as e:
            print(f"❌ Motor enable failed: {e}")
        
        # Cleanup
        try:
            motor.disable()
            print("✅ Motor disabled")
        except:
            pass
            
        try:
            bus.shutdown()
            print("✅ CAN bus shutdown")
        except:
            pass
            
    except Exception as e:
        print(f"❌ Overall diagnosis failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("🔧 Motor Position Diagnosis Tool")
    print("=" * 50)
    
    # Test both motors
    print("\n🎯 Testing Motor 3...")
    diagnose_motor(motor_id=3)
    
    print("\n" + "="*50)
    print("\n🎯 Testing Motor 4...")
    diagnose_motor(motor_id=4)
    
    print("\n🎉 Diagnosis complete!")