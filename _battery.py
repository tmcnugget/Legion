import smbus
import time

bus = smbus.SMBus(1)

def detect_i2c(addr):
    try:
        bus.write_byte(addr, 0)
        return True
    except:
        return False

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

# Detect which ADC is available
adc_flag = None
if detect_i2c(0x4F):  # PCF8591 detected
    adc_flag = False
elif detect_i2c(0x48):  # ADS7830 detected
    adc_flag = True
else:
    print("No correct I2C address found.\n"
          "Please use 'i2cdetect -y 1' to check the I2C address!\n"
          "Program exiting.")
    exit(-1)

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


import unicornhat as unicorn

# Initialize Unicorn HAT
unicorn.clear()

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

if __name__ == '__main__':
    try:
        while True:
            display_battery(255 - scaleRGB(get_voltage(1)), scaleRGB(get_voltage(1)), scaleHeight(get_voltage(1)), 255 - scaleRGB(get_voltage(2)), scaleRGB(get_voltage(2)), scaleHeight(get_voltage(2)))
            unicorn.show()
            time.sleep(0.1)
    except KeyboardInterrupt:
        bus.close()
        print("\nEnd of program")


