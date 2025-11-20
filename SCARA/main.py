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
HOME_ANGLES = (0, 0, 0)

# Global variable to track current position
current_angles = (0, 0, 0)  # Start at home position

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
def send_and_listen(pulses1, dir1, pulses2, dir2):
    """Send motor commands for 2 motors and listen for Arduino responses"""
    try:
        # Convert direction values to 0 or 1
        dir1 = 1 if dir1 else 0
        dir2 = 1 if dir2 else 0
        
        # Pack data: 2 pulses (32-bit int) + 2 directions (byte)
        data = struct.pack('>iiBB', pulses1, pulses2, dir1, dir2)
        ser.write(b'\x01' + data)
        
        print(f"Sent: M1={pulses1}(d:{dir1}), M2={pulses2}(d:{dir2})")
        
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
# SAFETY CHECKS
# -------------------------
def validate_movement(steps1, steps2):
    """Check if movement is within safe limits"""
    MAX_STEPS = 10000
    
    if abs(steps1) > MAX_STEPS or abs(steps2) > MAX_STEPS:
        print(f"⚠️ Large movement detected! Steps: {steps1}, {steps2}")
        return False
    return True

# -------------------------
# MAIN MOVEMENT FUNCTION
# -------------------------
def move_to_point(x, y, phi_desired=0.0, auto_home=True):
    """
    Move to specified point with constraints and optional homing
    """
    global current_angles
    
    try:
        print(f"\n🎯 MOVING TO: x={x}, y={y}, phi={phi_desired}")
        print(f"Starting from: θ1={utl.radians_to_degrees(current_angles[0]):.1f}°, θ2={utl.radians_to_degrees(current_angles[1]):.1f}°")
        
        # Use the utility function for IK calculation (pass current_angles)
        target_angles = utl.safe_ik_calculation(x, y, current_angles, phi_desired)
        
        if target_angles is None:
            print("❌ IK failed - skipping this point")
            return False

        print("\n--- TARGET ANGLES ---")
        print(f"θ1 = {target_angles[0]:.3f} rad ({utl.radians_to_degrees(target_angles[0]):.1f}°)")
        print(f"θ2 = {target_angles[1]:.3f} rad ({utl.radians_to_degrees(target_angles[1]):.1f}°)")
        print(f"θ3 = {target_angles[2]:.3f} rad ({utl.radians_to_degrees(target_angles[2]):.1f}°)")

        # Calculate relative steps from current position to target (pass current_angles)
        rel_steps1, dir1, rel_steps2, dir2, rel_steps3, dir3 = utl.calculate_relative_steps(target_angles, current_angles)

        # Safety validation
        if not validate_movement(rel_steps1, rel_steps2):
            confirm = input("Large movement detected. Continue anyway? (y/N): ")
            if confirm.lower() != "y":
                print("Movement cancelled.")
                return False

        # Send movement
        print("\nSending movement command...")
        responses = send_and_listen(rel_steps1, dir1, rel_steps2, dir2)
        
        if not responses:
            print("⚠️ No response from Arduino!")
            return False
        
        # Update current position if movement was successful
        current_angles = target_angles
        print("✅ Movement completed successfully!")
        print(f"New position: θ1={utl.radians_to_degrees(current_angles[0]):.1f}°, θ2={utl.radians_to_degrees(current_angles[1]):.1f}°")
        
        # Auto-home if requested
        if auto_home:
            time.sleep(1)
            # For homing, we need to pass current_angles and get the steps needed
            home_steps1, home_dir1, home_steps2, home_dir2, _, _ = utl.calculate_relative_steps(HOME_ANGLES, current_angles)
            responses = send_and_listen(home_steps1, home_dir1, home_steps2, home_dir2)
            if responses:
                current_angles = HOME_ANGLES
                print("✅ Home position reached")
        
        return True
        
    except Exception as e:
        print(f"❌ Unexpected error in move_to_point: {e}")
        return False

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
    
    responses = send_and_listen(rel_steps1, dir1, rel_steps2, dir2)
    
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
            (0.0, -0.15, 0.0),   # Point 1  ###wrong?
            (0.10, 0.0, 0.0),   # Point 2  
            (0.20, 0.0, 0.0),   # Point 3
            (0.10, -0.25, 0.0),   # Point 4
            (0.0, 0.15, 0.0),   # Point 5  
            (0.10, 0.3, 0.0),   # Point 6
        ]
        
        for i, (x, y, phi) in enumerate(test_points, 1):
            print(f"\n{'='*50}")
            print(f"MOVEMENT {i}/{len(test_points)}")
            
            success = move_to_point(x, y, phi, auto_home=True)
            
            if not success:
                print(f"⚠️ Movement {i} failed - continuing to next point...")
            
            if i < len(test_points):
                cont = input(f"\nContinue to point {i+1}? (y/N): ")
                if cont.lower() != 'y':
                    break
        
        print("\n🎉 All movements completed!")
        
    except KeyboardInterrupt:
        print("\n🛑 Program interrupted by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed")