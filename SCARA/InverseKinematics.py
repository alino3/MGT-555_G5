import math

# Parameters (example — set to your real values)
L1 = 0.18   # meters
L2 = 0.18   # meters
STEPS_PER_REV = 200       # full steps per revolution (motor)
MICROSTEPS = 16           # e.g., 16 microsteps
GEAR_RATIO = 1.0          # motor revs per joint rev (include pulley ratio)
STEP_SIGN = [1, 1, 1]        # sign correction for motor wiring [motor1, motor2]
HOME_OFFSETS = [0, 0, 0]     # steps to add for zero calibration
STEPS_PER_JOINT_REV = STEPS_PER_REV * MICROSTEPS * GEAR_RATIO

def ik_scara(x, y, L1=L1, L2=L2):
    r2 = x*x + y*y
    r = math.sqrt(r2)
    # reachable?
    if r > (L1 + L2) + 1e-12 or r < abs(L1 - L2) - 1e-12:
        raise ValueError("Target unreachable: r = {:.4f} m".format(r))
    cos_theta2 = (r2 - L1*L1 - L2*L2) / (2 * L1 * L2)
    # clamp for numerical errors
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    # two elbow solutions
    sin_pos = math.sqrt(max(0.0, 1 - cos_theta2*cos_theta2))
    sin_neg = -sin_pos

    theta2_a = math.atan2(sin_pos, cos_theta2)   # elbow-down (or up depending on convention)
    theta2_b = math.atan2(sin_neg, cos_theta2)   # the other configuration

    # compute theta1 for both
    def theta1_from_theta2(theta2):
        k1 = L1 + L2 * math.cos(theta2)
        k2 = L2 * math.sin(theta2)
        return math.atan2(y, x) - math.atan2(k2, k1)

    theta1_a = theta1_from_theta2(theta2_a)
    theta1_b = theta1_from_theta2(theta2_b)

    return (theta1_a, theta2_a), (theta1_b, theta2_b)

def end_effector(theta1, theta2, phi_desired):

    theta3 = phi_desired - (theta1 + theta2)
    return theta3

def angles_to_steps(theta1, theta2, theta3=0,
                    steps_per_rev=STEPS_PER_REV, microsteps=MICROSTEPS,
                    gear_ratio=GEAR_RATIO, step_sign=STEP_SIGN, home_offsets=HOME_OFFSETS):
    steps_per_joint_rev = steps_per_rev * microsteps * gear_ratio
    s1 = int(round((theta1 / (2*math.pi)) * steps_per_joint_rev)) * step_sign[0] + home_offsets[0]
    s2 = int(round((theta2 / (2*math.pi)) * steps_per_joint_rev)) * step_sign[1] + home_offsets[1]
    s3 = int(round((theta3 / (2*math.pi)) * steps_per_joint_rev)) * step_sign[2] + home_offsets[2]
    d1 = int(math.copysign(1, s1))
    d2 = int(math.copysign(1, s2))
    d3 = int(math.copysign(1, s3))
    return s1, d1, s2, d2, s3, d3

# Example usage:
if __name__ == "__main__":
    x, y = 0.11, 0.06   # example target in meters
    try:
        sol1, sol2 = ik_scara(x, y)
        print("Solution A (theta1, theta2) in deg:", tuple(math.degrees(a) for a in sol1))
        print("Solution B (theta1, theta2) in deg:", tuple(math.degrees(a) for a in sol2))
        s1, s2 = angles_to_steps(*sol1)
        print("Steps (motor1, motor2):", s1, s2)
    except ValueError as e:
        print("IK error:", e)
