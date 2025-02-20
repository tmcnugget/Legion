from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

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

# Convert angle to pulse width
def angle_to_pulse(angle):
    return int(MIN_PULSE + (angle / ANGLE_RANGE) * (MAX_PULSE - MIN_PULSE))

# Lengths of arm segments
L1 = 72  # Upper arm length in mm
L2 = 115 # Lower arm length in mm

# Inverse Kinematics function for calculating angles
def ik(Z):
    # Compute theta2 (elbow angle)
    d = Z
    cos_theta2 = (d**2 - L1**2 - L2**2) / (-2 * L1 * L2)
    if abs(cos_theta2) > 1:
        return None  # No solution, target too far

    theta2 = math.acos(cos_theta2)

    # Compute theta1 (base angle)
    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(d, k1)

    return math.degrees(theta1), math.degrees(theta2)

def servo(channel, angle):
    pulse = angle_to_pulse(angle)
    if channel > 15:
        pca1.channels[channel].duty_cycle = int((pulse / 20000) * 0xFFFF)
    elif channel < 32 :
        pca2.channels[channel - 16].duty_cycle = int((pulse / 20000) * 0xFFFF)

# Function to set all servos to a user-defined angle
def set_servos():
    while True:
          input = float(input("Enter Z height "))
          angles = ik(input)
          j1 = angles[0]
          j2 = angles[1]
          servo(14, j1)
          servo(13, j2)

try:
    set_servos()
except KeyboardInterrupt:
    print("Stopping...")
    pca1.deinit()
    pca2.deinit()
