import InverseKinematics as IK
import serial
import struct
import time
import math
import Utilities as utl

# -------------------------
# CONFIGURATION
# -------------------------
PORT = "COM5"
BAUDRATE = 115200
TIMEOUT = 1

# Home position (angles in radians)
HOME_ANGLES = (0, 0, 0, 0)

# Global variable to track current position
current_angles = (0, 0, 0, 0)  # Start at home position

# -------------------------
# SERIAL INITIALIZATION
# -------------------------
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    time.sleep(2)
    print(f"Connected to {PORT} successfully")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

# -------------------------
# SEND FUNCTION (2 MOTORS)
# -------------------------
def send_and_listen(pulses1, dir1, pulses2, dir2, pulses3, dir3, servo1, servo2):
    """Send motor commands for 2 motors and listen for Arduino responses"""
    try:
        # Convert direction values to 0 or 1
        dir1 = 1 if dir1 else 0
        dir2 = 1 if dir2 else 0
        dir3 = 1 if dir3 else 0
  
        # Pack data: 2 pulses (32-bit int) + 2 directions (byte)
        data = struct.pack('>iiiBBBBB', pulses1, pulses2, pulses3, dir1, dir2, dir3,  servo1, servo2)
        ser.write(b'\x01' + data)
        
        # Wait for and read responses
        print("Arduino responses:")
        time.sleep(1)
        
        responses = []
        start_time = time.time()
        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                msg = ser.readline().decode().strip()
                if msg:
                    print("  ", msg)
                    responses.append(msg)
            time.sleep(0.1)
            
        return responses
        
    except Exception as e:
        print(f"Error in send_and_listen: {e}")
        return []


# -------------------------
# MAIN MOVEMENT FUNCTION
# -------------------------
def move_to_point(x, y, current_angles, z, phi=0, gripper_open=False, auto_home=True):
        """
        Move to specified point with constraints and optional homing
        Returns: (success, current_angles)
        """
        print("\n--- MOVE TO POINT ---")
        print("Current angles:")
        print(current_angles)
        print(f"\n🎯 MOVING TO: x={x}, y={y}, z={z}")
        print(f"Starting from: θ1={utl.radians_to_degrees(current_angles[0]):.1f}°, θ2={utl.radians_to_degrees(current_angles[1]):.1f}°")
        
        # Use the utility function for IK calculation (pass current_angles)
        target_angles = utl.safe_ik_calculation(x, y, current_angles, z, phi)
        
        if target_angles is None:
            print("❌ IK failed - skipping this point")
            return False, current_angles  # ← FIXED: Return tuple with current angles

        print("\n--- TARGET ANGLES ---")
        print(f"θ1 = {target_angles[0]:.3f} rad ({utl.radians_to_degrees(target_angles[0]):.1f}°)")
        print(f"θ2 = {target_angles[1]:.3f} rad ({utl.radians_to_degrees(target_angles[1]):.1f}°)")
        print(f"θ3 = {target_angles[2]:.3f} rad ({utl.radians_to_degrees(target_angles[2]):.1f}°)")

        # Calculate relative steps from current position to target (pass current_angles)
        rel_steps1, dir1, rel_steps2, dir2, rel_steps3, dir3 = utl.calculate_relative_steps(target_angles, current_angles)


        servo1 = target_angles[2]  # Assuming servo1 corresponds to theta3

        if gripper_open:
            servo2 = 0  # Open position
        else:
            servo2 = 90  # Closed position

        # Send movement
        print("\nSending movement command...")
        print(f"Relative Steps: M1={rel_steps1}(d:{dir1}), M2={rel_steps2}(d:{dir2}), M3={rel_steps3}(d:{dir3})")
        responses = send_and_listen(rel_steps1, dir1, rel_steps2, dir2, rel_steps3, dir3, servo1, servo2=90)
        
        if not responses:
            print("⚠️ No response from Arduino!")
            return False, current_angles  # ← FIXED: Return tuple with current angles
        
        # Update current position if movement was successful
        current_angles = target_angles
        print("✅ Movement completed successfully!")
        print(f"New position: θ1={utl.radians_to_degrees(current_angles[0]):.1f}°, θ2={utl.radians_to_degrees(current_angles[1]):.1f}°")
        
        # Auto-home if requested
        if auto_home:
            time.sleep(5)
            # For homing, we need to pass current_angles and get the steps needed
            home_steps1, home_dir1, home_steps2, home_dir2, _, _ = utl.calculate_relative_steps(HOME_ANGLES, current_angles)
            responses = send_and_listen(home_steps1, home_dir1, home_steps2, home_dir2, pulses3=0, dir3=0, servo1=90, servo2=90)
            if responses:
                current_angles = HOME_ANGLES
                print("✅ Home position reached")
        print("----------c1---------------")
        print(current_angles)
        return True, current_angles
    

# -------------------------
# MANUAL CONTROL FUNCTIONS
# -------------------------
def manual_move(rel_steps1, rel_steps2):
    """Manually move relative steps (for testing)"""
    global current_angles
    
    dir1 = 1 if rel_steps1 >= 0 else 0
    dir2 = 1 if rel_steps2 >= 0 else 0
    
    rel_steps1 = abs(rel_steps1)
    rel_steps2 = abs(rel_steps2)
    
    print(f"\n🔧 MANUAL MOVE: M1={rel_steps1}(d:{dir1}), M2={rel_steps2}(d:{dir2})")
    
    responses = send_and_listen(rel_steps1, dir1, rel_steps2, dir2, pulses3=0, dir3=0, servo1=90, servo2=90)
    
    if responses:
        print("✅ Manual move completed")
    else:
        print("❌ No response during manual move")
    
    return responses

# -------------------------
# MAIN TEST
# -------------------------
if __name__ == "__main__":
    try:
        print("🤖 SCARA Robot Controller Started")
        print(f"Current position: θ1={utl.radians_to_degrees(current_angles[0]):.1f}°, θ2={utl.radians_to_degrees(current_angles[1]):.1f}°")
        
        # Test sequence
        test_points = [
            #(0.05, 0.0, 0.0),   # Point 2
            (0.1, 0.0, -10),   # Point 1  ###wrong?
            (0.2, 0.0, -15),   # Point 3
            (0.36, 0.0, 0)
        ]
        for i, (x, y, z) in enumerate(test_points, 1):
            print(f"\n{'='*50}")
            print(f"MOVEMENT {i}/{len(test_points)}")
            print(current_angles)
            success, current_angles = move_to_point(x, y, current_angles, z, auto_home=False)

            if not success:
                print(f"⚠️ Movement {i} failed - continuing to next point...")
            
            """             if i < len(test_points):
                cont = input(f"\nContinue to point {i+1}? (y/N): ")
                if cont.lower() != 'y':
                    break """
        
        print("\n🎉 All movements completed!")
        
    except KeyboardInterrupt:
        print("\n🛑 Program interrupted by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed")