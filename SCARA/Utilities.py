# -------------------------
# UTILITY FUNCTIONS
# -------------------------

import math
import InverseKinematics as IK


# Motor angle constraints (degrees)
MOTOR1_ABS_MIN = -80   # degrees
MOTOR1_ABS_MAX = 80    # degrees  
MOTOR2_ABS_MIN = -160  # degrees
MOTOR2_ABS_MAX = 160   # degrees

COUPLING_RATIO = 0.5  # Motor2 moves half the angle of Motor1 due to coupling

def get_effective_limits(current_theta1, current_theta2):
    """
    Calculate effective limits for motor2 based on current motor1 position
    The coupling only affects how much motor2 MOVES RELATIVELY when motor1 moves
    """
    current_theta1_deg = radians_to_degrees(current_theta1)
    current_theta2_deg = radians_to_degrees(current_theta2)
    
    # The key insight: coupling affects RELATIVE MOVEMENT, not absolute limits
    # When motor1 moves by Δθ1, motor2 is dragged by Δθ2_coupling = Δθ1 * COUPLING_RATIO
    # So the NET movement we command to motor2 should be: Δθ2_desired - Δθ2_coupling
    
    print(f"   Coupling analysis:")
    print(f"   - Motor1 current: {current_theta1_deg:.1f}°")
    print(f"   - Motor2 current: {current_theta2_deg:.1f}°")
    print(f"   - When Motor1 moves, Motor2 is dragged by {COUPLING_RATIO} of that movement")
    
    # Absolute limits still apply to both motors independently
    # The coupling only affects how we achieve the movement
    effective_min = MOTOR2_ABS_MIN
    effective_max = MOTOR2_ABS_MAX
    
    print(f"   - Motor2 absolute range: [{effective_min:.1f}°, {effective_max:.1f}°]")
    
    return effective_min, effective_max

def validate_angles_with_coupling(theta1, theta2, current_angles):
    """
    Check if angles are within constraints considering mechanical coupling
    The coupling means we need to compensate motor2 commands
    """
    current_theta1, current_theta2, _ = current_angles
    
    theta1_deg = radians_to_degrees(theta1)
    theta2_deg = radians_to_degrees(theta2)
    current_theta1_deg = radians_to_degrees(current_theta1)
    current_theta2_deg = radians_to_degrees(current_theta2)
    
    print(f"\n🔍 VALIDATING ANGLES:")
    print(f"   Current: θ1={current_theta1_deg:.1f}°, θ2={current_theta2_deg:.1f}°")
    print(f"   Target:  θ1={theta1_deg:.1f}°, θ2={theta2_deg:.1f}°")
    
    # Check motor1 absolute limits
    if not (MOTOR1_ABS_MIN <= theta1_deg <= MOTOR1_ABS_MAX):
        reason = f"Motor 1 angle {theta1_deg:.1f}° outside absolute limits [{MOTOR1_ABS_MIN}, {MOTOR1_ABS_MAX}]"
        print(f"❌ {reason}")
        return False, reason
    
    # Check motor2 absolute limits
    if not (MOTOR2_ABS_MIN <= theta2_deg <= MOTOR2_ABS_MAX):
        reason = f"Motor 2 angle {theta2_deg:.1f}° outside absolute limits [{MOTOR2_ABS_MIN}, {MOTOR2_ABS_MAX}]"
        print(f"❌ {reason}")
        return False, reason
    
    # The coupling doesn't limit the target position, it affects how we get there
    # We need to calculate the COMPENSATION needed for motor2
    
    delta_theta1 = theta1_deg - current_theta1_deg
    delta_theta2_desired = theta2_deg - current_theta2_deg
    
    # Due to coupling, when motor1 moves by delta_theta1, motor2 moves by delta_theta1 * COUPLING_RATIO
    delta_theta2_coupling = delta_theta1 * COUPLING_RATIO
    
    # The actual motor2 command should compensate for this coupling
    delta_theta2_command = delta_theta2_desired + delta_theta2_coupling
    
    print(f"   Movement analysis:")
    print(f"   - Motor1 desired Δ: {delta_theta1:.1f}°")
    print(f"   - Motor2 desired Δ: {delta_theta2_desired:.1f}°")
    print(f"   - Coupling effect: Motor2 would move {delta_theta2_coupling:.1f}° automatically")
    print(f"   - Motor2 command needed: {delta_theta2_command:.1f}° (to compensate coupling)")
    
    # Check if the compensated motor2 movement is within reasonable bounds
    # (we don't want enormous compensation values)
    compensation_tolerance = MOTOR2_ABS_MAX 
    if abs(delta_theta2_command) > compensation_tolerance:
        reason = f"Coupling compensation too large: {delta_theta2_command:.1f}° (max: {compensation_tolerance}°)"
        print(f"❌ {reason}")
        return False, reason
    
    print("✅ All constraints satisfied")
    return True, "OK"

def calculate_relative_steps_with_coupling(target_angles, current_angles):
    """
    Calculate relative steps needed to move from current position to target position
    WITH COUPLING COMPENSATION
    """
  
    try:
        current_theta1, current_theta2, current_theta3 = current_angles
        target_theta1, target_theta2, target_theta3 = target_angles
        
        # Convert angles to degrees for coupling calculation
        current_theta1_deg = radians_to_degrees(current_theta1)
        current_theta2_deg = radians_to_degrees(current_theta2)
        target_theta1_deg = radians_to_degrees(target_theta1)
        target_theta2_deg = radians_to_degrees(target_theta2)
        
        # Calculate desired deltas
        delta_theta1_deg = target_theta1_deg - current_theta1_deg
        delta_theta2_desired_deg = target_theta2_deg - current_theta2_deg
        
        # Calculate coupling effect
        delta_theta2_coupling_deg = delta_theta1_deg * COUPLING_RATIO
        
        # Calculate compensated motor2 movement
        delta_theta2_command_deg = delta_theta2_desired_deg - delta_theta2_coupling_deg
        
        print(f"\n--- COUPLING COMPENSATION CALCULATION ---")
        print(f"Desired movement:")
        print(f"  Motor1: {current_theta1_deg:.1f}° → {target_theta1_deg:.1f}° (Δ={delta_theta1_deg:.1f}°)")
        print(f"  Motor2: {current_theta2_deg:.1f}° → {target_theta2_deg:.1f}° (Δ={delta_theta2_desired_deg:.1f}°)")
        print(f"Coupling effect: Motor2 auto-moves {delta_theta2_coupling_deg:.1f}° when Motor1 moves")
        print(f"Compensated command: Motor2 needs Δ={delta_theta2_command_deg:.1f}°")
        
        # Convert compensated deltas back to radians for step calculation
        delta_theta1_rad = degrees_to_radians(delta_theta1_deg)
        delta_theta2_rad = degrees_to_radians(delta_theta2_command_deg)
        
        # Calculate target angles for step conversion (using compensated values)
        compensated_theta1 = current_theta1 + delta_theta1_rad
        compensated_theta2 = current_theta2 + delta_theta2_rad
        
        # Convert both current and compensated target positions to steps
        current_steps1, current_dir1, current_steps2, current_dir2, current_steps3, current_dir3 = IK.angles_to_steps(
            current_theta1, current_theta2, current_theta3
        )
        
        target_steps1, target_dir1, target_steps2, target_dir2, target_steps3, target_dir3 = IK.angles_to_steps(
            compensated_theta1, compensated_theta2, target_theta3
        )
        
        # Calculate relative steps needed
        rel_steps1 = target_steps1 - current_steps1
        rel_steps2 = target_steps2 - current_steps2
        
        # Determine directions based on sign of relative steps
        dir1 = 1 if rel_steps1 >= 0 else 0
        dir2 = 1 if rel_steps2 >= 0 else 0
        
        # Use absolute values for step counts
        rel_steps1 = abs(rel_steps1)
        rel_steps2 = abs(rel_steps2)
        
        print(f"Final relative steps: M1={rel_steps1}(d:{dir1}), M2={rel_steps2}(d:{dir2})")
        
        return rel_steps1, dir1, rel_steps2, dir2, 0, 0
        
    except Exception as e:
        print(f"❌ Error in calculate_relative_steps_with_coupling: {e}")
        return 0, 0, 0, 0, 0, 0
    
def degrees_to_radians(deg):
    """Convert degrees to radians"""
    return deg * math.pi / 180.0

def radians_to_degrees(rad):
    """Convert radians to degrees"""
    return rad * 180.0 / math.pi


def choose_best_solution(solA, solB, current_angles=None):
    """
    Choose the best IK solution based on constraints and coupling
    Returns (theta1, theta2) or None if no valid solution
    """
    try:
        theta1A, theta2A = solA
        theta1B, theta2B = solB
        
        print(f"\n🎯 ANALYZING IK SOLUTIONS:")
        print(f"Solution A: θ1={radians_to_degrees(theta1A):.1f}°, θ2={radians_to_degrees(theta2A):.1f}°")
        print(f"Solution B: θ1={radians_to_degrees(theta1B):.1f}°, θ2={radians_to_degrees(theta2B):.1f}°")
        
        # Check which solutions satisfy constraints WITH COUPLING
        validA, reasonA = validate_angles_with_coupling(theta1A, theta2A, current_angles)
        validB, reasonB = validate_angles_with_coupling(theta1B, theta2B, current_angles)
        
        print(f"\n--- SOLUTION ANALYSIS RESULTS ---")
        print(f"Solution A: {'VALID' if validA else 'INVALID'} - {reasonA}")
        print(f"Solution B: {'VALID' if validB else 'INVALID'} - {reasonB}")
        
        if not validA and not validB:
            print("❌ No IK solution satisfies motor constraints with coupling!")
            print("💡 Possible issues:")
            print("   - Target position may be out of workspace")
            print("   - Coupling constraints too restrictive") 
            print("   - Current position limits movement options")
            return None
        
        # If only one valid, choose it
        if validA and not validB:
            print("✅ Selected Solution A (only valid option)")
            return solA
        if validB and not validA:
            print("✅ Selected Solution B (only valid option)")
            return solB
        
        # If both valid, choose based on proximity to current position
        if current_angles:
            current_theta1, current_theta2, _ = current_angles
            
            # Calculate angular distance for each solution
            distA = abs(theta1A - current_theta1) + abs(theta2A - current_theta2)
            distB = abs(theta1B - current_theta1) + abs(theta2B - current_theta2)
            
            if distA <= distB:
                print("✅ Selected Solution A (closer to current position)")
                return solA
            else:
                print("✅ Selected Solution B (closer to current position)")
                return solB
        else:
            # No current position, default to solution A
            print("✅ Selected Solution A (default)")
            return solA
            
    except Exception as e:
        print(f"❌ Error in choose_best_solution: {e}")
        return None

def safe_ik_calculation(x, y, current_angles, phi_desired=0.0):
    """
    Safely compute IK with comprehensive error handling
    Returns (theta1, theta2, theta3) or None if failed
    """
    try:
        print(f"🔍 Computing IK for x={x}, y={y}, phi={phi_desired}")
        
        # Compute IK solutions
        solA, solB = IK.ik_scara(x, y)
        
        # Debug: Check if IK returned valid solutions
        if solA is None or solB is None:
            print("❌ IK function returned None solutions")
            return None
            
        print(f"✅ IK computed solutions:")
        print(f"   Solution A: θ1={radians_to_degrees(solA[0]):.1f}°, θ2={radians_to_degrees(solA[1]):.1f}°")
        print(f"   Solution B: θ1={radians_to_degrees(solB[0]):.1f}°, θ2={radians_to_degrees(solB[1]):.1f}°")
        
        # Choose best solution based on constraints
        best_solution = choose_best_solution(solA, solB, current_angles)
        
        if best_solution is None:
            print("❌ No valid IK solution found")
            print("💡 Try:")
            print("   - Moving to a different position first")
            print("   - Checking if target is within workspace")
            print("   - Relaxing coupling constraints if possible")
            return None
            
        theta1, theta2 = best_solution
        theta3 = IK.end_effector(theta1, theta2, phi_desired)
        
        print("✅ IK computation successful")
        return (theta1, theta2, theta3)
        
    except ValueError as e:
        print(f"❌ IK mathematical error: {e}")
        return None
    except Exception as e:
        print(f"❌ Unexpected IK error: {e}")
        return None
 
def calculate_relative_steps(target_angles, current_angles):
    """
    Calculate relative steps needed to move from current position to target position
    Returns: (steps1, dir1, steps2, dir2, steps3, dir3) for relative movement
    """
     
    
    current_theta1, current_theta2, current_theta3 = current_angles
    target_theta1, target_theta2, target_theta3 = target_angles
    
    # Convert both current and target positions to steps
    current_steps1, current_dir1, current_steps2, current_dir2, current_steps3, current_dir3 = IK.angles_to_steps(
        current_theta1, current_theta2, current_theta3
    )
    
    target_steps1, target_dir1, target_steps2, target_dir2, target_steps3, target_dir3 = IK.angles_to_steps(
        target_theta1, target_theta2, target_theta3
    )
    
    # Calculate relative steps needed
    rel_steps1 = target_steps1 - current_steps1
    rel_steps2 = target_steps2 - current_steps2
    rel_steps3 = target_steps3 - current_steps3
    
    # Determine directions based on sign of relative steps
    dir1 = 1 if rel_steps1 >= 0 else 0
    dir2 = 0 if rel_steps2 >= 0 else 1
    dir3 = 1 if rel_steps3 >= 0 else 0
    
    # Use absolute values for step counts
    rel_steps1 = abs(rel_steps1)
    rel_steps2 = abs(rel_steps2)
    rel_steps3 = abs(rel_steps3)
    
    print(f"\n--- RELATIVE MOVEMENT CALCULATION ---")
    print(f"Current: θ1={radians_to_degrees(current_theta1):.1f}°, θ2={radians_to_degrees(current_theta2):.1f}°")
    print(f"Target:  θ1={radians_to_degrees(target_theta1):.1f}°, θ2={radians_to_degrees(target_theta2):.1f}°")
    print(f"Relative steps: M1={rel_steps1}(d:{dir1}), M2={rel_steps2}(d:{dir2})")
    
    return rel_steps1, dir1, rel_steps2, dir2, rel_steps3, dir3

def go_home():
    """Move all motors to home position from current position"""
    global current_angles
    
    print("\n🏠 RETURNING TO HOME POSITION...")
    print(f"Current position: θ1={radians_to_degrees(current_angles[0]):.1f}°, θ2={radians_to_degrees(current_angles[1]):.1f}°")
    
    # Calculate relative steps needed to get to home
    rel_steps1, dir1, rel_steps2, dir2, rel_steps3, dir3 = calculate_relative_steps(HOME_ANGLES)
    
    # Send movement command
    responses = send_and_listen(rel_steps1, dir1, rel_steps2, dir2)
    
    if responses:
        # Update current position to home
        current_angles = HOME_ANGLES
        print("✅ Home position reached")
        print(f"New position: θ1={radians_to_degrees(current_angles[0]):.1f}°, θ2={radians_to_degrees(current_angles[1]):.1f}°")
    else:
        print("❌ No response during homing")
    
    return responses