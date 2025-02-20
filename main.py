import numpy as np
import math
import time
import smbus
import sys
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import unicornhat as unicorn
import scrollphat as scroll

bus = smbus.SMBus(1)

def detect_i2c(addr):
    try:
        bus.write_byte(addr, 0)
        return True
    except:
        return False

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

# Initialize Unicorn HAT
unicorn.clear()

scroll.set_brightness(20)

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

def analog_read_pcf8591(chn):
    """Read analog value from PCF8591 (4 channels: 0-3)."""
    return bus.read_byte_data(0x4F, 0x40 + chn)

def analog_write_pcf8591(value):
    """Write DAC value to PCF8591."""
    bus.write_byte_data(0x4F, 0x40, value)

def analog_read_ads7830(chn):
    """Read analog value from ADS7830 (8 channels: 0-7)."""
    cmd = 0x84 | (((chn << 2 | chn >> 1) & 0x07) << 4)
    return bus.read_byte_data(0x48, cmd)

def get_analog(chn):
    """Read analog value based on detected ADC."""
    return analog_read_ads7830(chn) if adc_flag else analog_read_pcf8591(chn)

def get_voltage(bat):
    """Calculate battery voltage from ADC readings."""
    if adc_flag:
        val0 = get_analog(0)
        val1 = get_analog(4)
    else:
        val0 = get_analog(0)
        val1 = get_analog(1)

    battery1 = round(val0 / 255 * 5 * 3, 2)
    battery2 = round(val1 / 255 * 5 * 3, 2)

    return battery1 if bat == 1 else battery2

def scroll_sine():
    i = 0
    buf = [0] * 11
    
    for x in range(0, 11):
        y = (math.sin((i + (x * 10)) / 10.0) + 1)  # Produces range from 0 to 2
        y *= 2.5                                   # Scale to 0 to 5
        buf[x] = 1 << int(y)

    scroll.set_buffer(buf)
    scroll.update()

    time.sleep(0.005)

    i += 1

# Detect which ADC is available
adc_flag = None
if detect_i2c(0x4F):  # PCF8591 detected
    adc_flag = False
elif detect_i2c(0x48):  # ADS7830 detected
    adc_flag = True

j1, j2, j3 = 90, 45, 90  # Default joint angles for (x, y, z) = (0, 0, 0)

def ik(x, y, z, L1=72, L2=87):
    """
    Solve IK for a 3DOF quadruped leg in 3D (x, y, z space).
    x, y, z: Desired foot position relative to the hip joint.
    L1, L2: Lengths of upper and lower leg segments.
    """
    global j1, j2, j3
    
    # Base rotation (J1) to align with X-Z plane
    j1 = np.clip(np.degrees(np.arctan2(z, x)), 45, 135)  # Limit J1 between 45° and 135°
    print(f"J1 calculation: atan2(z, x) = atan2({z}, {x}) -> J1 = {j1}°")
    
    # Projected distance in YZ plane
    x_proj = np.sqrt(x**2 + z**2)  # Effective x when rotated
    dist = np.sqrt(x_proj**2 + y**2)  # Distance from hip to foot
    print(f"Projected x (x_proj): {x_proj}")
    print(f"Total distance (dist): {dist}")
    
    if dist == 0:
        print("Foot position is at the hip! Invalid IK solution.")
        return None
    
    if dist > (L1 + L2):
        print(f"Target out of reach! Dist: {dist}, L1+L2: {L1 + L2}")
        return None
    
    # Law of Cosines to find knee angle (J3)
    cos_knee = (L1**2 + L2**2 - dist**2) / (2 * L1 * L2)
    cos_knee = np.clip(cos_knee, -1, 1)  # Clamping to prevent invalid values
    knee_angle = np.arccos(cos_knee)
    print(f"Knee angle calculation: cos_knee = {cos_knee}, knee_angle = {np.degrees(knee_angle)}°")
    
    # Law of Cosines for hip-lift angle (J2)
    cos_hip = (L1**2 + dist**2 - L2**2) / (2 * L1 * dist)
    cos_hip = np.clip(cos_hip, -1, 1)  # Clamping to prevent invalid values
    hip_angle = np.arccos(cos_hip)
    print(f"Hip angle calculation: cos_hip = {cos_hip}, hip_angle = {np.degrees(hip_angle)}°")
    
    # Hip joint rotation to reach (x, y)
    theta_hip = np.arctan2(y, x_proj) - hip_angle
    theta_knee = np.pi - knee_angle  # Convert to servo-friendly angle
    print(f"Theta hip: {np.degrees(theta_hip)}°, Theta knee: {np.degrees(theta_knee)}°")
    
    # Convert angles to 0-180 range
    j2 = np.clip(np.degrees(theta_hip) + 90, 0, 180)  # Adjusted to make 90° point straight down
    j3 = np.clip(np.degrees(theta_knee), 0, 180)
    
    print(f"Final joint angles -> J2: {j2}°, J3: {j3}°")
    
    return j1, j2, j3

# Generate smooth semi-circle trajectory
def smooth_traj(points, n_steps=10):
    path = []
    for i in range(len(points) - 1):
        x1, y1, z1 = points[i]
        x2, y2, z2 = points[i+1]
        t = np.linspace(0, 1, n_steps)
        x_interp = (1 - t) * x1 + t * x2
        y_interp = (1 - np.cos(np.pi * t)) / 2 * (y2 - y1) + y1
        z_interp = (1 - t) * z1 + t * z2
        path.extend(zip(x_interp, y_interp, z_interp))
    return path

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

def servo(channel, angle):
    pulse = angle_to_pulse(angle)
    if channel <= 15:
        pca1.channels[channel].duty_cycle = int((pulse / 20000) * 0xFFFF)
    else:
        pca2.channels[channel - 16].duty_cycle = int((pulse / 20000) * 0xFFFF)

def scaleRGB(value):
    # Normalize to the range [0, 1] where 7.2 maps to 0 and 8.2 maps to 1
    normalized = (value - 7.2) / (8.5 - 7.2)
    # Scale to 0-255 and convert to int
    return int(round(normalized * 255))

def scaleHeight(value):
    # Normalize to the range [0, 1] where 7.2 maps to 0 and 8.2 maps to 1
    normalized = (value - 7.2) / (8.2 - 7.2)
    # Scale to 0-255 and convert to int
    return int(round(normalized * 8))

def display_battery(red1, green1, h1, red2, green2, h2):
    # Clear the screen
    unicorn.clear()

    # Filling the battery (green bar inside the 8x16 grid)
    bar1 = [
        (x, y, red1, green1, 0) for y in range(0, 4) for x in range(0, h1)
    ]

    bar2 = [
        (x, y, red2, green2, 0) for y in range(4, 8) for x in range(0, h2)
    ]

    # Set the pixels
    for pixel in bar1:
        unicorn.set_pixel(*pixel)

    for pixel in bar2:
        unicorn.set_pixel(*pixel)

# Trot gait using semi-circle trajectory
def trot_leg(leg, n_steps=10):
    trajectory = [(0, 0, 0), (1, 1, 1), (2, 0, 2), (0, 0, 0)]
    smooth_path = smooth_traj(trajectory, n_steps)
    
    for x, y, z in smooth_path:
        angles = ik(x, y, z)
        if angles:
            j1, j2, j3 = angles
            print(f"Foot Pos: ({x:.1f}, {y:.1f}, {z:.1f}) -> J1: {j1:.1f}°, J2: {j2:.1f}°, J3: {j3:.1f}°")
            globals()[f"l{leg}j1"] = j1
            globals()[f"l{leg}j2"] = j2
            globals()[f"l{leg}j3"] = j3

            if leg == 1:
                servo(15, l1j1)
                servo(14, l1j2)
                servo(13, l1j3)
            if leg == 2:
                servo(12, l2j1)
                servo(11, l2j2)
                servo(10, l2j3)
            if leg == 3:
                servo(9, l3j1)
                servo(8, l3j2)
                servo(31, l3j3)
            if leg == 4:
                servo(22, l4j1)
                servo(23, l4j2)
                servo(27, l4j3)

def trot(wait=0.5, res=10):
    trot_leg(1, res)
    trot_leg(4, res)
    time.sleep(wait)
    trot_leg(2, res)
    trot_leg(3, res)
    time.sleep(wait)

# Define servo positions
global l1j1, l1j2, l1j3, l2j1, l2j2, l2j3, l3j1, l3j2, l3j3, l4j1, l4j2, l4j3
l1j1 = l1j2 = l1j3 = 0
l2j1 = l2j2 = l2j3 = 0
l3j1 = l3j2 = l3j3 = 0
l4j1 = l4j2 = l4j3 = 0

if __name__ == '__main__':
    try:
        while True:
            display_battery(255 - scaleRGB(get_voltage(1)), scaleRGB(get_voltage(1)), scaleHeight(get_voltage(1)), 255 - scaleRGB(get_voltage(2)), scaleRGB(get_voltage(2)), scaleHeight(get_voltage(2)))
            unicorn.show()
            scroll_sine()
            trot()
    except KeyboardInterrupt:
        bus.close()
        unicorn.clear
        scroll.clear
        print("\nEnd of program")
