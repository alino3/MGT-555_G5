import InverseKinematics as IK
import serial
import struct
import time
import math
import Utilities as utl

def send_servo_command(servo1_angle=None, servo2_angle=None):
    """
    Send servo commands to Arduino 3
    Angles should be between 0-180, or None to skip
    """
    try:
        # Create command packet for servos
        # Use a different command byte (0x02 for servos)
        command = b'\x02'
        
        # Add servo data
        if servo1_angle is not None:
            command += struct.pack('B', min(180, max(0, servo1_angle)))
        else:
            command += b'\xFF'  # No change marker
            
        if servo2_angle is not None:
            command += struct.pack('B', min(180, max(0, servo2_angle)))
        else:
            command += b'\xFF'  # No change marker
        
        ser.write(command)
        
        print(f"Sent servo command: Servo1={servo1_angle}, Servo2={servo2_angle}")
        
        # Read responses
        time.sleep(0.5)
        responses = []
        while ser.in_waiting > 0:
            msg = ser.readline().decode().strip()
            if msg:
                print(f"Arduino: {msg}")
                responses.append(msg)
        
        return responses
        
    except Exception as e:
        print(f"Error sending servo command: {e}")
        return []

def move_with_servo(x, y, current_angles, phi_desired=0.0, servo1_angle=None, servo2_angle=None):
    """
    Move to point and control servos simultaneously
    """
    # First move the SCARA robot
    success, new_angles = move_to_point(x, y, current_angles, phi_desired, auto_home=False)
    
    # Then control servos if specified
    if success and (servo1_angle is not None or servo2_angle is not None):
        servo_responses = send_servo_command(servo1_angle, servo2_angle)
        if servo_responses:
            print("✅ Servo movement completed")
        else:
            print("⚠️ No servo response")
    
    return success, new_angles

# Example usage in your main test:
def test_with_servos():
    """Test sequence with servo movements"""
    current_angles = (0, 0, 0)
    
    test_sequence = [
        # (x, y, phi, servo1_angle, servo2_angle)
        (0.1, 0.0, 0.0, 0, 180),    # Move right, servos extreme
        (0.1, 0.1, 0.0, 90, 90),    # Move diagonal, servos center
        (0.0, 0.1, 0.0, 180, 0),    # Move forward, servos opposite
        (0.0, 0.0, 0.0, 45, 135),   # Return home, servos intermediate
    ]
    
    for x, y, phi, s1, s2 in test_sequence:
        print(f"\n🎯 Moving to ({x}, {y}) with servos: {s1}°, {s2}°")
        success, current_angles = move_with_servo(x, y, current_angles, phi, s1, s2)
        
        if success:
            print("✅ Movement with servos completed")
        else:
            print("❌ Movement failed")
        
        time.sleep(2)

# Uncomment to test:
test_with_servos()