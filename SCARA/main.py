import InverseKinematics as IK
import serial
import struct
import time
import math
import Utilities as utl
import numpy as np

# -------------------------
# CONFIGURATION
# -------------------------
PORT = "COM6"
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
# SEND FUNCTION 
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
        time.sleep(0.1)
        
        responses = []
        start_time = time.time()
        while time.time() - start_time < 5: # Increased timeout for safety (in case move is long)
            if ser.in_waiting > 0:
                # Use errors='ignore' to prevent crashes like before
                msg = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if msg:
                    print("  ", msg)
                    responses.append(msg)
                    
                    # --- THE FIX: Stop waiting if Arduino says it's done ---
                    if "Movement Done" in msg:
                        return responses
            
            # Short sleep to save CPU, but loop checks often
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
        
        # FIX: Convert absolute phi (degrees) to radians here
        phi_rad = utl.degrees_to_radians(phi)
        
        # Pass radians to the safe_ik_calculation
        target_angles = utl.safe_ik_calculation(x, y, current_angles, z, phi_rad)
        
        if target_angles is None:
            print("❌ IK failed - skipping this point")
            return False, current_angles  # ← FIXED: Return tuple with current angles

        print("\n--- TARGET ANGLES ---")
        print(f"θ1 = {target_angles[0]:.3f} rad ({utl.radians_to_degrees(target_angles[0]):.1f}°)")
        print(f"θ2 = {target_angles[1]:.3f} rad ({utl.radians_to_degrees(target_angles[1]):.1f}°)")
        print(f"θ3 = {target_angles[2]:.3f} rad ({utl.radians_to_degrees(target_angles[2]):.1f}°)")

        # Calculate relative steps from current position to target (pass current_angles)
        rel_steps1, dir1, rel_steps2, dir2, rel_steps3, dir3 = utl.calculate_relative_steps(target_angles, current_angles)

        # --- FIX STARTS HERE ---
        # Convert the IK radian output to degrees for the servo
        theta3_rad = target_angles[2]
        theta3_deg = utl.radians_to_degrees(theta3_rad)
        
        # Ensure it's a positive integer for the servo command
        servo1 = int(4*abs(theta3_deg)/5)
        # --- FIX ENDS HERE ---

        if gripper_open:
            servo2 = 0  # Open position
        else:
            servo2 = 90  # Closed position

        # Send movement
        print("\nSending movement command...")
        print(f"Relative Steps: M1={rel_steps1}(d:{dir1}), M2={rel_steps2}(d:{dir2}), M3={rel_steps3}(d:{dir3})")
        responses = send_and_listen(rel_steps1, dir1, rel_steps2, dir2, rel_steps3, dir3, servo1, servo2)
        
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


def pick_and_place(pick_x, pick_y, place_x, place_y, z_pick, z_place, phi_p=0):
    """Perform a pick-and-place operation"""
    global current_angles
    
    print("\n--- PICK AND PLACE OPERATION ---")
    rest_point = (0.36, 0.0, 0, 0, False)  # Safe rest point above the table
    
        # Test sequence
    test_points = [
            #(0.05, 0.0, 0.0),   # Point 2
            (pick_x, pick_y, 0, 0, False),   # Point 1 go above pick 
            (pick_x, pick_y, z_pick, phi_p, True),   # Point 2 pick
            (pick_x, pick_y, z_pick, phi_p, False),   # Point 3 grip
            (pick_x, pick_y, 0, 90, False),   # Point 4 lift up
            (place_x, place_y, 0, 0, False), # Point 5 go above place
            (place_x, place_y, z_place, 90, False),   # Point 6 go down to place
            (place_x, place_y, z_place, 90, True),   # Point 7 release
            (place_x, place_y, 0, 0, False), # Point 8 go up
            rest_point
        ]
    for i, (x, y, z, phi, gripper_state) in enumerate(test_points, 1):
            print(f"\n{'='*50}")
            print(f"Pick and place {i}/{len(test_points)}")
            print(current_angles)
            success, current_angles = move_to_point(x, y, current_angles, z, phi, gripper_open=gripper_state, auto_home=False)
            print('---------------------//()-------------------')
            print(phi)
            time.sleep(2)  # Short pause between moves
            if not success:
                print(f"⚠️ Movement {i} failed - continuing to next point...")
                break
    

        
    
    print("✅ Pick and place operation completed")


# -------------------------
# MAIN TEST
# -------------------------
if __name__ == "__main__":
    try:
        print("🤖 SCARA Robot Controller Started")
        pick_and_place(0.28, -0.09, 0.2, 0.2, z_pick=-30, z_place=-15, phi_p=135)
        
        print("\n🎉 All movements completed!")
        
    except KeyboardInterrupt:
        print("\n🛑 Program interrupted by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed")