import time
import sys
try:
    import i2c
except:
    import drivers.i2c as i2c

class EncodersIO ():
    def __init__(self):
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
