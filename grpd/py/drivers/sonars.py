import time
import sys
try:
    import i2c
except:
    import drivers.i2c as i2c

class SonarsIO ():
    def __init__(self):
        self.__bus_nb = 2
        self.__addr_4_sonars = 0x21
        self.__addr_front_left = 0x070
        self.__addr_front_right = 0x072
        self.__dev_i2c_4_sonars=i2c.i2c(self.__addr_4_sonars,self.__bus_nb)
        self.__dev_i2c_front_left=i2c.i2c(self.__addr_front_left,self.__bus_nb)
        self.__dev_i2c_front_right=i2c.i2c(self.__addr_front_right,self.__bus_nb)
        #
        # setup the sonar

    # function to read the distance to nearest obstacle in front of the front sonar
    def read_right(self):
        dist_front = -1
        # insert your code here 
        return dist_front

    # write the other functions here ...

if __name__ == "__main__":
    sonars = SonarsIO()
    # do some tests here, example distance to obstacle for sonar right
    print (sonars.read_right())
