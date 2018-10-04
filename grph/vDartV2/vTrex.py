import struct
import time
from collections import OrderedDict

# from stackoverflow, it's safer to proxy dict rather than subclassing it
# this way we are sure to give access to methods we know we want to provide
# rather than falling back to parent class' methods

# provide a student-proof dictionary with immutable keys 
class FixedDict():
        def __init__(self, dictionary):
            self._dictionary = dictionary
        def __setitem__(self, key, item):
                if key not in self._dictionary:
                    raise KeyError("The key {} is not defined.".format(key))
                self._dictionary[key] = item
        def __getitem__(self, key):
            return self._dictionary[key]
        def values(self):
            return self._dictionary.values()
        def keys(self):
            return self._dictionary.keys()

def high_byte(integer):
    '''
    Get the high byte from a int
    '''
    return integer >> 8


def low_byte(integer):
    '''
    Get the low byte from a int
    '''
    return integer & 0xFF


class TrexIO():

    __command_dict_strings = ('left_motor_speed', 
                              'right_motor_speed', 'use_pid', 'crc')

    __status_dict_strings = (           
                'left_encoder', 'right_encoder',
                '__dont_use_this_padding_byte', 'crc'
                )

    def __init__(self):
        self.command_bytes = [0,0,0,0,0,0]
        self.status_bytes = [0,0,0,0,0,0]
        self.status = dict(
                    zip(
                        TrexIO.__status_dict_strings, 
                        [0]*len(TrexIO.__status_dict_strings)
                        )
                    )
        self.__reset_bytes = (0,0,0,0)
        self.update_motor = False
        self.reset()

    def reset(self):
        self.command = FixedDict(OrderedDict(
                zip(
                TrexIO.__command_dict_strings,
                self.__reset_bytes)
                ))
        self.leftEnco=0
        self.rightEnco=0
        self.i2c_write()

    def i2c_write(self):
        #print ("i2c_write")
        val=list(self.command.values())
        key=list(self.command.keys())
        self.update_motor = True
        #for i in range(len(val)):
        #    print (key[i],":",val[i])
                
    def i2c_read(self):
        #print ("i2c_read")
        val=list(self.status.values())
        key=list(self.status.keys())
        #for i in range(len(val)):
        #    print (key[i],":",val[i])
        self.leftEnco = 1
        self.rightEnco = 1
        return 0

