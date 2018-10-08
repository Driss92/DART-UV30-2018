import sys
import time
import dartv2 as dartv2

class MyCmdDartV2():

    def __init__(self):
        # initialize the robot
        self.drt = dartv2.DartV2()

    # get battery voltage
    def battery_level(self):
        return self.drt.encoders.battery_voltage()

    def set_speed(self,speed_left,speed_right):
        """
        Set speed using left and right motor command
        Commands are in [0, 255]
        Warning : DART robot actually moves when commands are 
        higher than 80 (depending on the robot)
        """
        #print ("motors",speed_left,speed_right)
        self.drt.trex.command["left_motor_speed"] = int(speed_left)
        self.drt.trex.command["right_motor_speed"] = int(speed_right)
        self.drt.trex.i2c_write()

    def distance(self,nom):
        return self.drt.sonars.get_distance(nom)

    def all_distances(self):
        return self.drt.sonars.get_all_distances()

    #def odos(self):
    #    return self.drt.encoders.read_encoders()

    def stop(self):
        self.drt.stop()

if __name__ == "__main__": 

    my_drt = MyCmdDartV2()

    # test a function (ex. Battery level)
    v = my_drt.battery_level ()
    print ("Battery level %5.2f V "%(v))

    my_drt.set_speed(80,-80)
    for i in range(5):
        #print (my_drt.distance("front"),my_drt.distance("right"),
        #       my_drt.distance("left"),my_drt.distance("right"),
        #       my_drt.distance("front_left"),my_drt.distance("front_right"))
        print(my_drt.all_distances())
        time.sleep(0.2)
   #print (my_drt.odos())

    # stop the robot
    my_drt.stop()
