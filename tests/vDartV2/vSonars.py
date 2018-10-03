import struct
import time
import imp

class SonarsIO():
    def __init__(self):
        i2c = imp.load_source('i2c', '../vDartV2/vI2C.py')
        self.__bus_nb = 2
        self.__addr_4_sonars = 0x21
        self.__addr_front_left = 0x070
        self.__addr_front_right = 0x072
        self.__dev_i2c_4_sonars=i2c.i2c(self.__addr_4_sonars,self.__bus_nb)
        self.__dev_i2c_front_left=i2c.i2c(self.__addr_front_left,self.__bus_nb)
        self.__dev_i2c_front_right=i2c.i2c(self.__addr_front_right,self.__bus_nb)
        #
        # setup the sonar here
        self.period = 0x20  # 200 ms period 
        #self.config = 0x65  # front sonar
        self.config = 0xFF  # all sonars
        self.__dev_i2c_4_sonars.write(0,[0, self.period])
        self.__dev_i2c_4_sonars.write(0,[0, self.config])

    # function to read the distance to nearest obstacle in front of the front sonar
    def read_diag_right(self):
        #self.bus.write_i2c_block_data(self.addr_r,0,[0x51])
        try:
            self.__dev_i2c_front_right.write(0,[0x51])
            time.sleep(0.065)
            try:
                vms = self.__dev_i2c_front_right.read_byte(2)
                vls = self.__dev_i2c_front_right.read_byte(3)
                #print (vms,vls)
                self.diag_right = float(vls + (vms << 8))/100.0        
            except:
                self.diag_right = -1
        except:
            self.diag_right = -1
        return self.diag_right

    def read_diag_all(self):
        self.read_diag()
        return [self.diag_left,self.diag_right]

    def read_diag(self):
        #self.bus.write_i2c_block_data(self.addr_l,0,[0x51])
        #self.bus.write_i2c_block_data(self.addr_r,0,[0x51])
        try:
            self.__dev_i2c_front_left.write(0,[0x51])
            self.__dev_i2c_front_right.write(0,[0x51])
            time.sleep(0.065)
            try:
                vms = self.__dev_i2c_front_left.read_byte(2)
                vls = self.__dev_i2c_front_left.read_byte(3)
                #vms = self.bus.read_byte_data(self.addr_l,2)
                #vls = self.bus.read_byte_data(self.addr_l,3)
                #print (vms,vls)
                self.diag_left = float(vls + (vms << 8))/100.0
            except:
                self.diag_left = -1
            try:
                vms = self.__dev_i2c_front_right.read_byte(2)
                vls = self.__dev_i2c_front_right.read_byte(3)
                #vms = self.bus.read_byte_data(self.addr_r,2)
                #vls = self.bus.read_byte_data(self.addr_r,3)
                #print (vms,vls)
                self.diag_right = float(vls + (vms << 8))/100.0        
            except:
                self.diag_right = -1
        except:
            self.diag_left = -1
            self.diag_right = -1
        return [self.diag_left,self.diag_right]

    def read_4_sonars(self):
        vf = self.read_front()
        vl = self.read_left()
        vb = self.read_rear()
        vr = self.read_right()
        return [vf, vl, vb, vr]

    def read_front(self):
        self.front = self.__read(1)
        return self.front

    def read_left(self):
        self.left = self.__read(3)
        return self.left

    def read_rear(self):
        self.rear = self.__read(2)
        return self.rear
    
    def read_right(self):
        self.right = self.__read(4)
        return self.right


    def __read(self,num_sonar):
        try:
            #v = self.bus.read_i2c_block_data(self.addr,num_sonar,2)
            v = self.__dev_i2c_4_sonars.read(num_sonar,2)
            print (v)
            v = v[0] + (v[1] << 8)
        except:
            v = -1
        return v

    def __write(self,cmd):
        #self.bus.write_i2c_block_data(self.addr,0,cmd)
        self.__dev_i2c_4_sonars.write(0,cmd)

    # write the other functions here ...
        
    def get_distance(self, sonar_key):
        """
        Return distance measured "sonar_key" sonar in meters
        
        Parameters:
            sonar_key: string, one of "front", "rear", "left", "right"
            
        Return value:
            distance: float, -1 on error, 0 on timeOut
        """
        v = -1
        n = 0
        if sonar_key == "front":
            v = self.read_front()
            n = 1
        elif sonar_key == "rear":
            v = self.read_rear()
            n = 1
        elif sonar_key == "left":
            v = self.read_left()
            n = 1
        elif sonar_key == "right":
            v = self.read_right()
            n = 1
        elif sonar_key == "front_left":
            v = self.read_diag_left()*100.0
            n = 1
        elif sonar_key == "front_right":
            v = self.read_diag_right()*100.0
            n = 1
        elif sonar_key == "front_diag":
            v = self.read_diag_all()
            n = 2
        if n==1:
            if v>0:
                v = float(v)/100.0

        return v

if __name__ == "__main__":
    sonars = SonarsIO()
    # do some tests here, example distance to obstacle for sonar right
    print (sonars.read_right())
