import math
import time
import smbus
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

def angle_to_pulse(angle):
    return int(MIN_PULSE + (angle / ANGLE_RANGE) * (MAX_PULSE - MIN_PULSE))

def twos_complement(value):
    """Convert raw two's complement data to signed integer"""
    if value & (1 << 15):  # Check if the sign bit is set
        value -= 1 << 16
    return value

def ik(x, y, z, L1=72, L2=72):
    """
    Calculate the joint angles for a 3DOF quadruped leg.
    """
    theta_hip_rotation = math.atan2(y, x)
    proj_x = math.sqrt(x**2 + y**2)
    d = math.sqrt(proj_x**2 + z**2)
    
    if d > (L1 + L2):
        raise ValueError("Target position out of reach")
    
    cos_knee = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    theta_knee = math.acos(cos_knee)
    
    cos_hip_lift = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    theta_hip_lift = math.acos(cos_hip_lift) + math.atan2(z, proj_x)

    theta_hip_rotation = max(0, min(270, theta_hip_rotation))
    theta_hip_lift = max(0, min(270, theta_hip_lift))
    theta_knee = max(0, min(270, theta_knee))
    
    return math.degrees(theta_hip_rotation), math.degrees(theta_hip_lift), math.degrees(theta_knee)

def read_mpu(bus, address=0x68):
    """
    Read accelerometer and gyroscope data from the MPU6050 sensor.
    """
    bus.write_byte_data(address, 0x6B, 0)  # Wake up MPU6050
    
    accel_x = twos_complement((bus.read_byte_data(address, 0x3B) << 8) | bus.read_byte_data(address, 0x3C))
    accel_y = twos_complement((bus.read_byte_data(address, 0x3D) << 8) | bus.read_byte_data(address, 0x3E))
    accel_z = twos_complement((bus.read_byte_data(address, 0x3F) << 8) | bus.read_byte_data(address, 0x40))
    
    gyro_x = twos_complement((bus.read_byte_data(address, 0x43) << 8) | bus.read_byte_data(address, 0x44))
    gyro_y = twos_complement((bus.read_byte_data(address, 0x45) << 8) | bus.read_byte_data(address, 0x46))
    gyro_z = twos_complement((bus.read_byte_data(address, 0x47) << 8) | bus.read_byte_data(address, 0x48))
    
    return {
        "accel": (accel_x, accel_y, accel_z),
        "gyro": (gyro_x, gyro_y, gyro_z)
    }

def adjust_gait(mpu_data, x, y, z):
    """
    Adjust gait based on MPU6050 sensor data.
    """
    gyro_x, gyro_y, gyro_z = mpu_data["gyro"]
    correction_x = gyro_x * 0.0001  # Small correction factor
    correction_y = gyro_y * 0.0001
    
    return x + correction_x, y + correction_y, z

def servo(channel, angle):
    pulse = angle_to_pulse(angle)
    if channel <= 15:
        pca1.channels[channel].duty_cycle = int((pulse / 20000) * 0xFFFF)
    else:
        pca2.channels[channel].duty_cycle = int((pulse / 20000) * 0xFFFF)

def update_servos(leg, angles):
    """
    Update the servo positions based on calculated angles.
    """
    global l1j1, l1j2, l1j3, l2j1, l2j2, l2j3, l3j1, l3j2, l3j3, l4j1, l4j2, l4j3
    
    if leg == "FL":
        l1j1, l1j2, l1j3 = angles
    elif leg == "FR":
        l2j1, l2j2, l2j3 = angles
    elif leg == "BL":
        l3j1, l3j2, l3j3 = angles
    elif leg == "BR":
        l4j1, l4j2, l4j3 = angles

    l1j1 = min(max(l1j1, 45), 135)
    l2j1 = min(max(l2j1, 45), 135) 
    l3j1 = min(max(l3j1, 45), 135) 
    l4j1 = min(max(l4j1, 45), 135)

    servo(15, l1j1)
    servo(14, l1j2)
    servo(13, l2j3)
    servo(12, l2j1)
    servo(11, l2j2)
    servo(10, l2j3)
    servo(9, l3j1)
    servo(8, l3j2)
    servo(31, l3j3)
    servo(22, l4j1)
    servo(23, l4j2)
    servo(27, l4j3)

def trot(step_len, step_h, cycle_t, bus):
    """
    Generate a simple trotting gait with MPU6050 balance correction.
    """
    time_step = cycle_t / 4
    trot_pos = [
        (step_len / 2, 0, -72),
        (0, 0, -72 + step_h),
        (-step_len / 2, 0, -72),
        (0, 0, -72)
    ]
    
    legs = ['FL', 'BR', 'FR', 'BL']
    
    for phase in range(4):
        mpu_data = read_mpu(bus)
        
        for i, leg in enumerate(legs):
            if (phase % 2 == 0 and i % 2 == 0) or (phase % 2 == 1 and i % 2 == 1):
                x, y, z = trot_pos[phase]
                x, y, z = adjust_gait(mpu_data, x, y, z)
                angles = ik(x, y, z)
                update_servos(leg, angles)
                print(f"{leg} -> Hip Rotation: {angles[0]:.2f}, Hip Lift: {angles[1]:.2f}, Knee: {angles[2]:.2f}")
        time.sleep(time_step)

def sit():
    """
    Make the robot sit by adjusting leg angles.
    """
    sit_pos = {
        "FL": (0, 0, -50),  # Front left leg slightly bent
        "FR": (0, 0, -50),  # Front right leg slightly bent
        "BL": (0, 0, -30),  # Back left leg more bent
        "BR": (0, 0, -30),  # Back right leg more bent
    }
    
    for leg, pos in sit_pos.items():
        angles = ik(*pos)
        update_servos(leg, angles)
        print(f"{leg} -> Hip Rotation: {angles[0]:.2f}, Hip Lift: {angles[1]:.2f}, Knee: {angles[2]:.2f}")

# Define servo positions
global l1j1, l1j2, l1j3, l2j1, l2j2, l2j3, l3j1, l3j2, l3j3, l4j1, l4j2, l4j3
l1j1 = l1j2 = l1j3 = 0
l2j1 = l2j2 = l2j3 = 0
l3j1 = l3j2 = l3j3 = 0
l4j1 = l4j2 = l4j3 = 0

bus = smbus.SMBus(1)
step_len = 30
step_h = 15
cycle_t = 1.0
trot(step_len, step_h, cycle_t, bus)
sit()
