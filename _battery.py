import smbus
import time

class ADCDevice(object):
    def __init__(self):
        self.cmd = 0
        self.address = 0
        self.bus = smbus.SMBus(1)

    def detectI2C(self, addr):
        try:
            self.bus.write_byte(addr, 0)
            return True
        except:
            return False

    def close(self):
        self.bus.close()

class PCF8591(ADCDevice):
    def __init__(self):
        super(PCF8591, self).__init__()
        self.cmd = 0x40     # The default command for PCF8591 is 0x40.
        self.address = 0x4f # 0x48 is the default I2C address for PCF8591 Module.

    def analogRead(self, chn): # PCF8591 has 4 ADC input pins, chn: 0,1,2,3
        value = self.bus.read_byte_data(self.address, self.cmd + chn)
        return value

    def analogWrite(self, value): # Write DAC value
        self.bus.write_byte_data(self.address, self.cmd, value)

class ADS7830(ADCDevice):
    def __init__(self):
        super(ADS7830, self).__init__()
        self.cmd = 0x84
        self.address = 0x48 # 0x4b is the default I2C address for ADS7830 Module.

    def analogRead(self, chn): # ADS7830 has 8 ADC input pins, chn: 0,1,2,3,4,5,6,7
        value = self.bus.read_byte_data(self.address, self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4))
        return value

class ADC:
    def __init__(self):
        self.adcFlag = None
        self.adc = ADCDevice()
        if self.adc.detectI2C(0x4f):  # Detect the PCF8591.
            self.adcFlag = False
            self.adc = PCF8591()
        elif self.adc.detectI2C(0x48):  # Detect the ADS7830.
            self.adcFlag = True
            self.adc = ADS7830()
        else:
            print("No correct I2C address found, \n"
                  "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
                  "Program Exit. \n")
            exit(-1)

    def batteryValue(self, chn):
        return self.adc.analogRead(chn)

    def batteryPower(self):
        if self.adcFlag:
            val0 = self.batteryValue(0)
            val1 = self.batteryValue(4)
        else:
            val0 = self.batteryValue(0)
            val1 = self.batteryValue(1)

        battery1 = round(val0 / 255 * 5 * 3, 2)
        battery2 = round(val1 / 255 * 5 * 3, 2)
        return battery1, battery2

import unicornhat as unicorn

# Initialize Unicorn HAT
unicorn.set_layout(unicorn.PHAT)
unicorn.rotation(0)
unicorn.brightness(1.0)

def color_gradient(value):
    # Map the value to a range from 0 to 255 for red and green channels
    value = max(7.00, min(8.00, value))  # Clamp value between 7.00 and 8.00
    green = int((8.00 - value) * 255)  # Green goes up as value decreases
    red = int((value - 7.00) * 255)    # Red goes up as value increases
    return (red, green, 0)

def draw_bar(value, start_col):
    # Convert value to height for the bar, based on the range 7.00 to 8.00
    height = int(((value - 7.00) / 1.00) * 8)  # Map to 0 to 8 rows
    for row in range(8):
        color = color_gradient(value) if row < height else (0, 0, 0)  # Color for bar or black if below height
        for col in range(start_col, start_col + 3):  # Draw a bar 3 columns wide
            unicorn.set_pixel(col, row, *color)

def display_data(input_string):
    # Parse the input string
    values = input_string.strip('()').split(',')
    value1 = float(values[0].strip())
    value2 = float(values[1].strip())

    # Clear Unicorn HAT screen
    unicorn.clear()

    # Draw the first bar (left half)
    draw_bar(value1, 0)
    # Draw the second bar (right half)
    draw_bar(value2, 3)

    # Update the display
    unicorn.show()

if __name__ == '__main__':
    adc = ADC()
    try:
        while True:
            input_data = adc.batteryPower()
            print(input_data)
            display_data(input_data)
            time.sleep(0.1)
    except KeyboardInterrupt:
        adc.adc.close()
        print("\nEnd of program")


