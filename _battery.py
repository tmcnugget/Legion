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

    def getA(self, chn): # getA stands for get analogue
        return self.adc.analogRead(chn)

    def getV(self, bat): # getV stands for get voltage
        if self.adcFlag:
            val0 = self.getA(0)
            val1 = self.getA(4)
        else:
            val0 = self.getA(0)
            val1 = self.getA(1)

        battery1 = round(val0 / 255 * 5 * 3, 2)
        battery2 = round(val1 / 255 * 5 * 3, 2)
        if bat == 1:
            return battery1
        if bat == 2:
            return battery2

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
    adc = ADC()
    try:
        while True:
            bat1 = adc.getV(1)
            bat2 = adc.getV(2)
            
            # Show the battery icon
            display_battery(255 - scaleRGB(bat1), scaleRGB(bat1), scaleHeight(bat1), 255 - scaleRGB(bat2), scaleRGB(bat2), scaleHeight(bat2))
            unicorn.show()
            time.sleep(0.1)
    except KeyboardInterrupt:
        adc.adc.close()
        print("\nEnd of program")


