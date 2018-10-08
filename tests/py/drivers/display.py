import smbus
import time

class DisplayIO ():
    def __init__(self, bus_nb = 2, addr = 0x40):
        self.bus=smbus.SMBus(bus_nb)
        self.addr = addr
        self.numjog = 8 # numjog should be read from somewhere in the system !
        self.val_dec = 0  # val decade  (in fact decade is 16 not 10 !)
        self.val_unit = self.numjog  # val unit
        self.on_dec = True
        self.on_unit = True
        self.__write(3)


    def off(self):
        self.val_dec = 0
        self.val_unit = 0
        self.on_dec = False
        self.on_unit = False
        self.__write(0)

    def both(self,val_dec,val_unit):
        self.val_dec = val_dec
        self.val_unit = val_unit
        self.on_dec = True
        self.on_unit = True
        self.__write(3)

    def decade(self,val_dec):
        self.val_dec = val_dec
        self.on_dec = True
        self.on_unit = False
        self.__write(2)

    def unit(self,val_unit):
        self.val_unit = val_unit
        self.on_dec = False
        self.on_unit = True
        self.__write(1)

    def __write(self,cmd):
        val = self.val_dec*16+self.val_unit
        self.bus.write_i2c_block_data(self.addr,0,[cmd , val])
        

if __name__ == "__main__":
    displ = DisplayIO()
    time.sleep(2)
    displ.off()
    time.sleep(2)
    displ.unit(8)
    time.sleep(2)
    displ.decade(0)
    time.sleep(2)
    displ.both(0,8)
