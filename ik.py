import math
import time
from adafruit_servokit import ServoKit

# Initialize PCA9685 with 16 channels per board
kit1 = ServoKit(channels=16, address=0x40)
kit2 = ServoKit(channels=16, address=0x41)

# Servo angle limits
ANGLE_MIN = 0
ANGLE_MAX = 180

def set_servo_angle(kit, channel, angle):
    """Set servo to a specific angle, ensuring it stays within bounds."""
    angle = max(ANGLE_MIN, min(ANGLE_MAX, angle))
    if kit == 1: 
        kit1.servo[channel].angle = angle
    elif kit == 2:
        kit2.servo[channel].angle = angle

def pwm(v1, v2, v3, leg):
    """Set angles for servos based on leg number."""
    if leg == 1:
        set_servo_angle(1, 15, v1)
        set_servo_angle(1, 14, v2)
        set_servo_angle(1, 13, v3)
    elif leg == 2:
        set_servo_angle(1, 12, v1)
        set_servo_angle(1, 11, v2)
        set_servo_angle(1, 10, v3)
    elif leg == 3:
        set_servo_angle(1, 9, v1)
        set_servo_angle(1, 8, v2)
        set_servo_angle(2, 15, v3)
    elif leg == 4:
        set_servo_angle(2, 6, v1)
        set_servo_angle(2, 5, v2)
        set_servo_angle(2, 11, v3)

def ik(x, y, z, L1, L2):
    """Inverse kinematics calculations."""
    theta1 = math.degrees(math.atan2(y, x)) + 90
    theta1 = max(45, min(135, theta1))
    
    r = math.sqrt(x**2 + y**2)
    h = z
    d = math.sqrt(r**2 + h**2)
    
    cos_theta3 = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    if abs(cos_theta3) > 1:
        raise ValueError("Target is out of reach")
    theta3 = math.degrees(math.acos(cos_theta3))
    
    cos_theta2 = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    if abs(cos_theta2) > 1:
        raise ValueError("Target is out of reach")
    theta2 = math.degrees(math.atan2(h, r) + math.acos(cos_theta2))
    
    return theta1, theta2, theta3

def servo(inleg, j1, j2, j3):
    """Map leg numbers and apply angles."""
    leg_mapping = {
        1: (1,), 2: (2,), 3: (3,), 4: (4,),
        12: (1, 2), 13: (1, 3), 14: (1, 4),
        21: (2, 1), 23: (2, 3), 24: (2, 4),
        31: (3, 1), 32: (3, 2), 34: (3, 4),
        41: (4, 1), 42: (4, 2), 43: (4, 3),
        44: (4, 4)
    }
    outleg = leg_mapping.get(inleg, (72,))
    print(f"Setting leg(s): {outleg} to J1: {j1}, J2: {j2}, J3: {j3}")
    if 1 in outleg: pwm(j1, j2, j3, 1)
    if 2 in outleg: pwm(j1, j2, j3, 2)
    if 3 in outleg: pwm(j1, j2, j3, 3)
    if 4 in outleg: pwm(j1, j2, j3, 4)

def trot(length, height, rest):
    """Simple trot gait sequence."""
    trot_sequence = [
        (14, 0, 0, ((L1 + L2) / 2) + height),
        (14, 0, 0, (L1 + L2) / 2),
        (14, length, 0, ((L1 + L2) / 2) + height),
        (14, length, 0, (L1 + L2) / 2),
        (23, 0, 0, ((L1 + L2) / 2) + height),
        (23, 0, 0, (L1 + L2) / 2),
        (23, length, 0, ((L1 + L2) / 2) + height),
        (23, length, 0, (L1 + L2) / 2)
    ]

    for leg, x, y, z in trot_sequence:
        angles = ik(x, y, z, L1, L2)
        j1, j2, j3 = angles
        servo(leg, j1, j2, j3)
        time.sleep(rest)

# Example usage
L1 = 72  # Upper leg length (mm)
L2 = 115  # Lower leg length (mm)

trot(40, 40, 0.2) 
