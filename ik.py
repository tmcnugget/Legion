import math
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import time

# Initialize I2C bus
i2c = busio.I2C(SCL, SDA)

# Set custom I2C addresses
I2C_ADDRESS_1 = 0x40  # First PCA9685
I2C_ADDRESS_2 = 0x41  # Second PCA9685

# Initialize PCA9685 devices
pca1 = PCA9685(i2c, address=I2C_ADDRESS_1)
pca2 = PCA9685(i2c, address=I2C_ADDRESS_2)
pca1.frequency = 50  # MG996R operates at 50Hz
pca2.frequency = 50

# Servo pulse range
MIN_PULSE = 500   # Min pulse width
MAX_PULSE = 2500  # Max pulse width
ANGLE_RANGE = 180 # Max servo angle range

def angle_to_pulse(angle):
    return int(MIN_PULSE + (angle / ANGLE_RANGE) * (MAX_PULSE - MIN_PULSE))

def pwm(v1, v2, v3, leg)
    pulse1 = angle_to_pulse(v1)
    pulse2 = angle_to_pulse(v2)
    pulse3 = angle_to_pulse(v3)
    if leg == 1:
        pca1.channels[15].duty_cycle = int((pulse1 / 20000) * 0xFFFF)
        pca1.channels[14].duty_cycle = int((pulse2 / 20000) * 0xFFFF)
        pca1.channels[13].duty_cycle = int((pulse3 / 20000) * 0xFFFF)
   if leg == 2:
        pca1.channels[12].duty_cycle = int((pulse1 / 20000) * 0xFFFF)
        pca1.channels[11].duty_cycle = int((pulse2 / 20000) * 0xFFFF)
        pca1.channels[10].duty_cycle = int((pulse3 / 20000) * 0xFFFF)
    if leg == 3:
        pca1.channels[9].duty_cycle = int((pulse1 / 20000) * 0xFFFF)
        pca1.channels[8].duty_cycle = int((pulse2 / 20000) * 0xFFFF)
        pca2.channels[15].duty_cycle = int((pulse3 / 20000) * 0xFFFF)
    if leg == 4:
        pca2.channels[6].duty_cycle = int((pulse1 / 20000) * 0xFFFF)
        pca2.channels[5].duty_cycle = int((pulse2 / 20000) * 0xFFFF)
        pca2.channels[11].duty_cycle = int((pulse3 / 20000) * 0xFFFF)

def ik(x, y, z, L1, L2):
    # Base rotation (theta1) with Y defaulting to 90 degrees and constrained between 45 and 135
    theta1 = math.degrees(math.atan2(y, x)) + 90
    theta1 = max(45, min(135, theta1))
    
    # Distance from base to target in the XY plane
    r = math.sqrt(x**2 + y**2)
    
    # Adjusted target height
    h = z
    
    # Distance from shoulder to target
    d = math.sqrt(r**2 + h**2)
    
    # Law of cosines to find the elbow angle (theta3)
    cos_theta3 = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    if abs(cos_theta3) > 1:
        raise ValueError("Target is out of reach")
    theta3 = math.degrees(math.acos(cos_theta3))
    
    # Law of cosines to find the shoulder angle (theta2)
    cos_theta2 = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    if abs(cos_theta2) > 1:
        raise ValueError("Target is out of reach")
    theta2 = math.degrees(math.atan2(h, r) + math.acos(cos_theta2))
    
    return theta1, theta2, theta3

# Example usage
L1 = 72  # Upper arm length
L2 = 115  # Lower arm length

def servo(inleg, j1, j2, j3):
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

def trot(length, height, rest):
    trot = [
        (14, 0, 0, ((L1 + L2)/2)+height),
        (14, 0, 0, (L1 + L2)/2),
        (14, length, 0, ((L1 + L2)/2)+height),
        (14, length, 0, (L1 + L2)/2 ),
        (23, 0, 0, ((L1 + L2)/2)+height),
        (23, 0, 0, (L1 + L2)/2),
        (23, length, 0, ((L1 + L2)/2)+height),
        (23, length, 0, (L1 + L2)/2 )
    ]

    for leg, x, y, z in trot:
        angles = ik(x, y, z, L1, L2)
        j1, j2, j3 = angles
        servo(leg, j1, j2, j3)
        time.sleep(rest)

def deinit():
    bus.close()

trot(40, 40, 0.2)
