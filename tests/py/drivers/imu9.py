import struct
import time
try:
    import i2c
except:
    import drivers.i2c as i2c

# LIS3DML 0x1e  (mag sensor)
# LSM6    0x6b  (accelero - gyro)



class Imu9IO():
    def __init__(self):
        self.__bus_nb = 2
        self.__addr_mg = 0x1e  # mag sensor
        self.__addr_ag = 0x6b  # accelero - gyro
        self.__dev_i2c_mg=i2c.i2c(self.__addr_mg,self.__bus_nb)
        self.__dev_i2c_ag=i2c.i2c(self.__addr_ag,self.__bus_nb)
        self.__mag_raw = [0.0,0.0,0.0]
        self.__accel_raw = [0.0,0.0,0.0]
        self.__gyro_raw = [0.0,0.0,0.0]
        #
        # configure mag sensor
        #
        # configure accelero + gyro

if __name__ == "__main__":
    imu = Imu9IO()
