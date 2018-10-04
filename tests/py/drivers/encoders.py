import time
import sys
try:
    import i2c
except:
    import drivers.i2c as i2c

class EncodersIO ():
    def __init__(self, bus_nb = 2, addr = 0x14):
        self.__bus_nb = 2
        self.__addr = 0x14 
        self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb)
        self.enc_left = -1
        self.enc_right = -1

    def battery_voltage(self):
        time.sleep(0.001)
        cmd = 5
        v = self.__read(cmd)
        v = 5.0*v/1024.0
        #print (v)
        return v/0.23

    def read_encoders(self):
        time.sleep(0.001)
        cmd = 1
        self.enc_left = self.__read(cmd)
        time.sleep(0.001)
        cmd = 2
        self.enc_right = self.__read(cmd)
        return [self.enc_left, self.enc_right]

    def read_motors_direction (self):
        time.sleep(0.001)
        cmd = 3
        self.motor_dir_left = self.__read(cmd)
        time.sleep(0.001)
        cmd = 4
        self.motor_dir_right = self.__read(cmd)
        return [self.motor_dir_left, self.motor_dir_right]
       
    def __debug(self,offs,nbytes):
        v = self.__dev_i2c.read(offs,nbytes)
        return v
       
    def __read(self,cmd):
        v=0
        try:
            v = self.__dev_i2c.read(cmd,2)
            v = v[0] + (v[1] << 8)
        except:
            v = -1
        return v        


if __name__ == "__main__":
    enc = EncodersIO()
    print ("Encoders (L,R) : ",enc.read_encoders())
    print ("Motor directions (L,R) : ",enc.read_motors_direction())
    print ("Battery : %5.2f V"%(enc. battery_voltage()))
