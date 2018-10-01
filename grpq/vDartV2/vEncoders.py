import time
import sys
import imp

class EncodersIO ():
    def __init__(self):
        i2c = imp.load_source('i2c', '../vDartV2/vI2C.py')
        self.__bus_nb = 2
        self.__addr = 0x14 
        self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb)


    def battery_voltage(self):
        v = 0.0
        # insert your code here
        return v

if __name__ == "__main__":
    enc = EncodersIO()
    # example test battery voltage
    print ("Battery : %5.2f V"%(enc. battery_voltage()))
