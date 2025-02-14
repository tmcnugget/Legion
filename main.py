import math
import time
from mpu6050 import mpu6050
import smbus

# Initialize MPU6050 sensor
sensor = mpu6050(0x68)

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
    
    return math.degrees(theta_hip_rotation), math.degrees(theta_hip_lift), math.degrees(theta_knee)

def read_mpu():
    """
    Read accelerometer and gyroscope data using the mpu6050 library.
    """
    data = sensor.get_all_data()
    
    accel_x, accel_y, accel_z = data[0]  # Accelerometer values (ax, ay, az)
    gyro_x, gyro_y, gyro_z = data[1]     # Gyroscope values (gx, gy, gz)
    
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
        mpu_data = read_mpu()  # Get sensor data
        
        for i, leg in enumerate(legs):
            if (phase % 2 == 0 and i % 2 == 0) or (phase % 2 == 1 and i % 2 == 1):
                x, y, z = trot_pos[phase]
                x, y, z = adjust_gait(mpu_data, x, y, z)  # Adjust based on gyro
                angles = ik(x, y, z)  # Calculate joint angles
                update_servos(leg, angles)  # Update servo positions
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
