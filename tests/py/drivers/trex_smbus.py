import smbus
import time
import crcmod
import struct

class TrexSmbusIO ():
    def __init__(self, bus_nb = 2, addr = 0x07):
        self.bus=smbus.SMBus(bus_nb)
        self.addr = addr
        self.cmd_left = 0
        self.cmd_right = 0
        self.use_pid = 0
        self.compute_crc = crcmod.mkCrcFun(0x131, initCrc=0) #Dallas polynom
        self.stop()

    def set_speed(self,cmd_left,cmd_right):
        self.cmd_left = cmd_left
        self.cmd_right = cmd_right
        self.write()
        

    def stop(self):
        self.set_speed(0,0)

    def status(self):
        val = self.bus.read_i2c_block_data(self.addr,0,6) # offset , number
        return val

    def write(self):
        cmd_left_low =  self.cmd_left & 0x0FF
        cmd_left_high =  (self.cmd_left & 0x0FF00) >> 8
        cmd_right_low =  self.cmd_right & 0x0FF
        cmd_right_high =  (self.cmd_right & 0x0FF00) >> 8
        data_packet = struct.pack("<BBBBB",
                                  cmd_left_high,cmd_left_low,
                                  cmd_right_high,cmd_right_low,
                                  self.use_pid)
        crc = self.compute_crc(data_packet, 0)
        val = [cmd_left_high,cmd_left_low,cmd_right_high,cmd_right_low,self.use_pid,crc]
        print (val)
        self.bus.write_i2c_block_data(self.addr,0x0F,val)
 

if __name__ == "__main__":
    trx = TrexSmbusIO()
    trx.set_speed(100,100)
    time.sleep(1)
    trx.stop()
    print (trx.status())
