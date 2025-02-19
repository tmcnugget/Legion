import math
import time
import smbus
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import unicornhat as unicorn

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

# Detect which ADC is available
adc_flag = None
if detect_i2c(0x4F):  # PCF8591 detected
    adc_flag = False
elif detect_i2c(0x48):  # ADS7830 detected
    adc_flag = True

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

    theta_hip_rotation = max(0, min(180, theta_hip_rotation))
    theta_hip_lift = max(0, min(180, theta_hip_lift))
    theta_knee = max(0, min(180, theta_knee))
    
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
    correction_x = gyro_x * 0.01  # Small correction factor
    correction_y = gyro_y * 0.01
    
    return x + correction_x, y + correction_y, z

def servo(channel, angle):
    pulse = angle_to_pulse(angle)
    if channel <= 15:
        pca1.channels[channel].duty_cycle = int((pulse / 20000) * 0xFFFF)
    else:
        pca2.channels[channel - 16].duty_cycle = int((pulse / 20000) * 0xFFFF)

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
    Move the quadruped into a sitting position by lowering the back legs 
    while keeping the front legs more extended.
    """
    # Front legs more upright
    front_leg_angles = ik(0, 0, -60)  # Adjust z for desired upright posture
    update_servos("FL", front_leg_angles)
    update_servos("FR", front_leg_angles)
    
    # Back legs bent to simulate sitting
    back_leg_angles = ik(0, 0, -30)  # Adjust z for a more bent position
    update_servos("BL", back_leg_angles)
    update_servos("BR", back_leg_angles)

    print("Quadruped is now sitting.")

def jump():
    crouch_angles = ik(0, 0, -60)
    jump_angles = ik(0, 0, 60)
    brace_angles = ik(0, 0, 1)
    impact_angles = ik(0, 0, -30)
    return_angles = ik(0, 0, 1)
    
    update_servos("FL", crouch_angles)
    update_servos("FR", crouch_angles)
    update_servos("BL", crouch_angles)
    update_servos("BR", crouch_angles)
    time.sleep(0.3)
    update_servos("FL", jump_angles)
    update_servos("FR", jump_angles)
    update_servos("BL", jump_angles)
    update_servos("BR", jump_angles)
    time.sleep(0.1)
    update_servos("FL", brace_angles)
    update_servos("FR", brace_angles)
    update_servos("BL", brace_angles)
    update_servos("BR", brace_angles)
    time.sleep(0.2)
    update_servos("FL", impact_angles)
    update_servos("FR", impact_angles)
    update_servos("BL", impact_angles)
    update_servos("BR", impact_angles)
    time.sleep(0.7)
    update_servos("FL", return_angles)
    update_servos("FR", return_angles)
    update_servos("BL", return_angles)
    update_servos("BR", return_angles)

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

# Define servo positions
global l1j1, l1j2, l1j3, l2j1, l2j2, l2j3, l3j1, l3j2, l3j3, l4j1, l4j2, l4j3
l1j1 = l1j2 = l1j3 = 0
l2j1 = l2j2 = l2j3 = 0
l3j1 = l3j2 = l3j3 = 0
l4j1 = l4j2 = l4j3 = 0

step_len = 30
step_h = 15
cycle_t = 1.0

if __name__ == '__main__':
    try:
        # trot(step_len, step_h, cycle_t, bus)
        sit()
        jump()
        while True:
            display_battery(255 - scaleRGB(get_voltage(1)), scaleRGB(get_voltage(1)), scaleHeight(get_voltage(1)), 255 - scaleRGB(get_voltage(2)), scaleRGB(get_voltage(2)), scaleHeight(get_voltage(2)))
            unicorn.show()
            time.sleep(0.1)
    except KeyboardInterrupt:
        bus.close()
        print("\nEnd of program")
