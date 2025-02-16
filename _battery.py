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


if __name__ == '__main__':
    adc = ADC()
    try:
        while True:
            power = adc.batteryPower()
            print(power)
            time.sleep(0.1)
    except KeyboardInterrupt:
        adc.adc.close()
        print("\nEnd of program")
