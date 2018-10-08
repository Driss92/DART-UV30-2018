#!/usr/bin/python
# -*- coding: utf-8 -*-

# Second Year ENSTA Bretagne SPID Project
#    D : Brian Detourbet
#    A : Elouan Autret
#    A : Fahad Al Shaik
#    R : Corentin Rifflart
#    R : Clément Rodde
#    T : Rémi Terrien

# Code modified in May 2016, by BenBlop (small changes to conform with
#  new DART)

# Code modified in May 2017, by BenBlop (big changes to adapt to Irvin's
# drivers and to communicate with V-REP using sockets

# Code modified in April 2018 to adapt to Dart V2 (BenBlop)
#    4 sonars changed (accessed with I2C)
#    2 diagonal sonars added (I2C)
#    Razor IMU (serial) changed to POLOLU IMU (I2C)
#    7 segments display added (I2C)
#    Front encoders added (I2C)

# v-rep 3.3.2, path :
# /home/newubu/MyApps/Nao/v-rep/nao-new-model/build/vnao/V-REP_PRO_EDU_V3_3_2_64_Linux
# 2017 simulator build path :
# /home/newubu/Lecture/robmob/dart/Simulateur/build/2017-05-10

import os
import time
import math
import struct
import sys
import signal
import random
import pickle

class DartV2():
    # interrupt handler
    def interrupt_handler(self,signal, frame):
        print ('You pressed Ctrl+C! DART V2 will stop immediatly')
        self.stop()
        sys.exit(0)

    def realMissionTime(self):
        #print ("real simulation time",self.__init_mission_time)
        if self.__init_mission_time:
            self.__start_mission_time = time.time()
            self.__init_mission_time = False
        return time.time() - self.__start_mission_time
        
    def virtualMissionTime(self):
        #print ("virtual simulation time",self.__init_mission_time)
        if self.__init_mission_time:
            self.__start_mission_time = self.vSimTime
            self.__init_mission_time = False
        return self.vSimTime - self.__start_mission_time
            

    def __init__(self):

        self.__dartSim = False

        # trap hit of ctrl-x to stop robot and exit (emergency stop!)
        signal.signal(signal.SIGINT, self.interrupt_handler)
        
        # test if on real robot, if yes load the drivers
        if os.access("/sys/class/gpio/gpio266", os.F_OK) :
            print ("Real DART is being created")

            # Import modules
            from drivers.trex import TrexIO
            from drivers.sonars import SonarsIO
            from drivers.encoders import EncodersIO
            from drivers.imu9 import Imu9IO

            self.__trex = TrexIO()
            self.__sonars = SonarsIO()
            self.__encoders = EncodersIO()
            self.__trex.command["use_pid"] = 0 # 1
            self.__imu = Imu9IO()

            self.__init_mission_time = True
            self.__start_mission_time = 0.0
            self.__mission_time = self.realMissionTime

        # if not create virtual drivers for the robot running in V-REP
        else :
            self.__dartSim = True
            print ("Virtual DART is being created")
            import threading
            import socket
            import sys
            import struct
            import time

            self.vdart_ready = threading.Event()
            self.vdart_ready.clear()
 
            # socket connection to V-REP
            self.__HOST = 'localhost'  # IP of the sockect
            self.__PORT = 30100 # port (set similarly in v-rep)

            self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print ('Socket created')

            # bind socket to local host and port
            try:
                # prevent to wait for  timeout for reusing the socket after crash
                # from :  http://stackoverflow.com/questions/29217502/socket-error-address-already-in-use
                self.__s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.__s.bind((self.__HOST, self.__PORT))
            except socket.error as msg:
                print (msg)
                sys.exit()
            print ('Socket bind completed')
             
            # start listening on socket
            self.__s.listen(10)
            print ('Socket now listening')

            self.vSimTime = 0.0

            self.__vLocation = [0.,0.,0.]
            self.__vAccel = [0.,0.,0.]
            self.__vGyro = [0.,0.,0.]

            # reset variables 
            #self.__vMotorLeft_Mem = 0
            #self.__vMotorRight_Mem = 0
            #self.__vMotorLeft =  0
            #self.__vMotorRight = 0
            self.__vEncoderFrontLeft = 0
            self.__vEncoderFrontRight = 0
            self.__vEncoderRearLeft = 0
            self.__vEncoderRearRight = 0
            self.__vSonarFrontLeft = 0.0
            self.__vSonarFrontRight = 0.0
            self.__vSonarLeft = 0.0
            self.__vSonarRight = 0.0
            self.__vSonarFront = 0.0
            self.__vSonarRear = 0.0
            self.__vHeading =  0.0
            self.__vVoltage =  0.0

            # left wheels motion control
            self.__vMotorDirectionLeft = 1
            self.__vMotorCmdLeft = 0
            #self.__vMotorCmdLastLeft = 0
            #self.__vMotorAngSpeedLeft = 0.0
            self.__vMotorCmdWithInertiaLeft = 0.0
            self.__vMotorAngSpeedWithInertiaLeft = 0.0
            self.__vMotorAngSpeedVrepLeft = 0.0
            self.__vMotorAngSpeedDriftVrepLeft = 0.0
            self.__vMotorCmdActualLastLeft = 0
            self.__vMotorInertiaLeft = 0.0

            # right wheels motion control
            self.__vMotorDirectionRight = 1
            self.__vMotorCmdRight = 0
            #self.__vMotorCmdLastRight = 0
            #self.__vMotorAngSpeedRight = 0.0
            self.__vMotorCmdWithInertiaRight = 0.0
            self.__vMotorAngSpeedWithInertiaRight = 0.0
            self.__vMotorAngSpeedVrepRight = 0.0
            self.__vMotorAngSpeedDriftVrepRight = 0.0
            self.__vMotorCmdActualLastRight = 0
            self.__vMotorInertiaRight = 0.0

            # control and simulation parameters 
            #self.__vInertiaLeft = "none"
            #self.__vInertiaRight = "none"
            self.__vInertiaCoefUp = 0.05 # 0.1 1st order LP filter (speed up)
            self.__vInertiaCoefDown = 0.1 # 0.1 1st order LP filter (speed down)
            self.__vDeadZoneUp = 70.0 # dead zone when starting
            self.__vDeadZoneDown = 30.0 # dead zone when stopping
            self.__vDeadZoneLeft = self.__vDeadZoneUp
            self.__vDeadZoneRight = self.__vDeadZoneUp            
            self.__vMotorLeftRightDiff = 1.01 # +/- % assymetric motor speed
            self.__vCmdToAngSpeed = 1./7.8 # speed cmd to actual angular speed

            # load and start virtual drivers 
            #vTrex = imp.load_source('vTrex', '../vDartV2/vTrex.py')
            #vSonars = imp.load_source('vSonars', '../vDartV2/vSonars.py')
            #vImu = imp.load_source('vImu', '../vDartV2/vImu.py')
            #vI2cHead = imp.load_source('vI2c', '../vDartV2/vI2cHead.py')
            sys.path.append("../vDartV2")
            import vSonars as vSonars
            import vTrex as vTrex
            import vImu9 as vImu9
            import vEncoders as vEncoders
            import vSimVar as vSimVar
            #import vI2C as vI2C

            self.vSimVar = vSimVar
            self.__trex = vTrex.TrexIO()
            self.__imu = vImu9.Imu9IO()
            self.__sonars = vSonars.SonarsIO()
            self.__encoders = vEncoders.EncodersIO()

            self.__init_mission_time = True
            self.__start_mission_time = 0.0
            self.__mission_time = self.virtualMissionTime

            # initiate communication thread with V-Rep
            self.__simulation_alive = True
            sk = self.__s
            ev = self.vdart_ready
            vrep = threading.Thread(target=self.vrep_com_socket,args=(sk,ev,))
            vrep.start()

            # debug log
            self.tlog=[]

            # wait for vdart to be ready
            self.vdart_ready.wait()
            print ("Dart ready ...")

    # realistic wheel motion (using tests from dart03)
    def motor_angular_speed(self, y):
        a = 13.6
        b = 91.0
        wv = a*(1.0-math.exp(-abs(y)/b))
        if y < 0:
            wv = -wv
        return wv

    def motor_angular_speed_drift_force(self, wvl, wvr):
        if wvl*wvr < 0:
            wvl *= 0.75
            wvr *= 0.75
        return wvl,wvr

    def motor_angular_speed_bound(self,  wvl, wvr):
        dz =  self.motor_angular_speed(self.__vDeadZoneDown)
        if abs(wvl) < dz:
            wvl = 0.0
        if abs(wvr) < dz:
            wvr = 0.0
        return wvl,wvr

    def angular_speed_with_inertia (self,side):
        if side == "left":
            cmd = self.__vMotorCmdLeft
            cmdi = self.__vMotorCmdWithInertiaLeft
            cmd_act_last = self.__vMotorCmdActualLastLeft
            w = self.__vMotorAngSpeedWithInertiaLeft
            motor = self.__vMotorDirectionLeft
            cin_last = self.__vMotorInertiaLeft
        elif side == "right":
            cmd = self.__vMotorCmdRight
            cmdi = self.__vMotorCmdWithInertiaRight
            cmd_act_last = self.__vMotorCmdActualLastRight
            w = self.__vMotorAngSpeedWithInertiaRight
            motor = self.__vMotorDirectionRight
            cin_last = self.__vMotorInertiaRight
        dz_up = self.__vDeadZoneUp
        dz_dn = self.__vDeadZoneDown
        # define actual command using dead zones and motor rotation 
        #if side == "left":
        #    print ("av (cmd,cmdi,w,motor)",cmd,cmdi,w,motor)
        cmd_act = 0
        if motor == 0:  # if motor not turning , cmd must be >= dz_up
            if cmd <= -dz_up:
                cmd_act = cmd
            elif cmd >= dz_up:
                cmd_act = cmd
        elif motor < 0: # if motor turns backwards
            if cmd <= -dz_dn:  # it continues turning if cmd <= -dz_down
                cmd_act = cmd
            elif cmd >= dz_up: # or if it changes direction with cmd >= dz_up
                cmd_act = cmd
        else:  # same but reversed cmd with motor turning forwards
            if cmd >= dz_dn:
                cmd_act = cmd
            elif cmd <= -dz_up:
                cmd_act = cmd

        # set inertia coef
        cin_up = self.__vInertiaCoefUp
        cin_dn = self.__vInertiaCoefDown
        if cmd_act == cmd_act_last:
            cin = cin_last
        else:
            if cmd_act >= 0:
                if cmd_act_last >= 0:
                    if cmd_act >= cmd_act_last:
                        cin = cin_up
                    else:
                        cin = cin_dn
                else:
                    if cmd_act <= cmd_act_last:
                        cin = cin_up
                    else:
                        cin = cin_dn
            else:
                if cmd_act_last <= 0:
                    if cmd_act <= cmd_act_last:
                        cin = cin_up
                    else:
                        cin = cin_dn
                else:
                    if cmd_act >= cmd_act_last:
                        cin = cin_up
                    else:
                        cin = cin_dn
            #print ("cin",cmd_act,cmd_act_last,cin)
        cmd_act_last = cmd_act # save for next step
        cin_last = cin
        
        # update command
        cmdi_last = cmdi
        cmdi = cin * cmd_act + (1.0 -cin) * cmdi
        dcmdi = cmdi-cmdi_last

        # find if motor is turning and in which direction
        motor = 1
        if cmdi < 0:
            motor = -1
        if abs(cmdi) < dz_dn:            
            motor = 0
        w = self.motor_angular_speed(cmdi)



        if side == "left":
            log={"t":0.0, "ts":0.0, "cmd" : 0, "cmdi" :0.0, "w" : 0.0,
                 "cmd_act" : 0, "motor" :0, "dz_up" : 0.0 , "dz_dn": 0.0,
                 "cin" : 0.0}
            log["t"] = time.time()
            log["ts"] = self.vSimTime
            log["cmd"] = cmd
            log["cmd_act"] = cmd_act
            log["cmdi"] = cmdi
            log["motor"] = motor
            log["w"] = w
            log["dz_up"] = dz_up
            log["dz_dn"] = dz_dn
            log["cin"] = cin
            self.tlog.append(log)

        if side == "left":
            self.__vMotorCmdWithInertiaLeft = cmdi
            self.__vMotorDerivCmdWithInertiaLeft = dcmdi
            self.__vMotorAngSpeedWithInertiaLeft = w
            self.__vMotorDirectionLeft = motor
            self.__vMotorAngSpeedVrepLeft = w * self.__vMotorLeftRightDiff
            self.__vMotorCmdActualLastLeft = cmd_act_last
            self.__vMotorInertiaLeft = cin_last
            #print (cmd,cmd_act,cmdi,cin,motor,w,self.__vMotorAngSpeedVrepLeft)
        elif side == "right":
            self.__vMotorCmdWithInertiaRight = cmdi
            self.__vMotorDerivCmdWithInertiaRight = dcmdi
            self.__vMotorAngSpeedWithInertiaRight = w
            self.__vMotorDirectionRight = motor
            self.__vMotorAngSpeedVrepRight = w / self.__vMotorLeftRightDiff
            self.__vMotorCmdActualLastRight = cmd_act_last
            self.__vMotorInertiaRight = cin_last

    # communication socket with V-REP
    def vrep_com_socket(vdart,s,ev):
        #print (s)
        #print (ev)
        RECV_BUFFER = 4096 # buffer size 
        while True:
            #wait to accept a connection - blocking call
            conn, addr =  s.accept()
            print ('Connected with ' + addr[0] + ':' + str(addr[1]))
            #print (conn)

            cnt=0
            while True:
                tsock0 = time.time()
                #print ("socket read",conn)
                data = conn.recv(RECV_BUFFER)
                #print (len(data))
                hd0,hd1,sz,lft,simulationTime,vSonarFront, vSonarRear,vSonarLeft, vSonarRight, vEncoderFrontLeft, vEncoderFrontRight, vHeading,vSonarFrontLeft,vSonarFrontRight, vEncoderRearLeft, vEncoderRearRight,vXRob,vYRob,vZRob,vXAcc,vYAcc,vZAcc,vXGyro,vYGyro,vZGyro = struct.unpack('<ccHHfffffffffffffffffffff',data)
                #print ("Encoders",vEncoderFrontLeft, vEncoderFrontRight, vEncoderRearLeft, vEncoderRearRight)
                #print ("Sonars",vSonarFront, vSonarRear,vSonarLeft, vSonarRight,vSonarFrontLeft,vSonarFrontRight)
                vVoltage = 6.4 - cnt*0.0002
                cnt+=1
                vMotorDirectionLeft = 1  # 1: forwards, 0: backwards
                vMotorDirectionRight = 1

                vdart.vrep_update_sim_param (simulationTime,
                                             vEncoderFrontLeft,
                                             vEncoderFrontRight,
                                             vEncoderRearLeft,
                                             vEncoderRearRight,
                                             vSonarFrontLeft,
                                             vSonarFrontRight,
                                             vSonarLeft,
                                             vSonarRight,
                                             vSonarFront,
                                             vSonarRear,
                                             vHeading,vVoltage,
                                             vMotorDirectionLeft,
                                             vMotorDirectionRight,
                                             vXRob,vYRob,vZRob,
                                             vXAcc,vYAcc,vZAcc,
                                             vXGyro,vYGyro,vZGyro)

                vdart.angular_speed_with_inertia("left")
                vdart.angular_speed_with_inertia("right")
                vMotorLeft = vdart.__vMotorAngSpeedVrepLeft
                vMotorRight = vdart.__vMotorAngSpeedVrepRight
                vMotorLeft, vMotorRight = vdart.motor_angular_speed_drift_force(vMotorLeft,vMotorRight)
                vMotorLeft, vMotorRight = vdart.motor_angular_speed_bound(vMotorLeft,vMotorRight)
                #print ("vdart",vMotorLeft,vMotorRight)
                vdart.__vMotorAngSpeedDriftVrepLeft = vMotorLeft
                vdart.__vMotorAngSpeedDriftVrepRight = vMotorRight
                #vdart.angular_speed_with_inertia("left")
                #vdart.angular_speed_with_inertia("right")
                #vdart.angular_speed_vrep("left")
                #vdart.angular_speed_vrep("right")
                #vMotorLeft = vdart.__vMotorAngSpeedVrepLeft
                #vMotorRight = vdart.__vMotorAngSpeedVrepRight
                #print ("vdart",vdart.__vMotorCmdLeft, vdart.__vMotorCmdRight,
                #       vMotorLeft,vMotorRight)
                strSend = struct.pack('<BBHHff',data[0],data[1],8,0,vMotorLeft,vMotorRight)
                conn.send(strSend)
                if not vdart.simulation_alive:
                    break
                #time.sleep(0.010)
                #time.sleep(0.002)
                ev.set()

                #print ("sock duration ",time.time()-tsock0)

            if not vdart.simulation_alive:
                break

    # actual sonar functions
    def actual_sonar(self,v):
        if random.random() < 0.005: # i2c failure probability of 0.5 %
            v = -1
        return v

    # actual encoder functions
    def actual_front_encoders(self,v): # 16 bits signed
        iv = int(round(v+0.5)) % 65536  # put on  16 bits
        if iv > 32767:   # add the sign
            iv -= 65536
        return iv
    def actual_rear_encoders(self,v): # 16 bits unsigned
        iv = int(round(v+0.5)) % 65536  # put on  16 bits
        #if random.random() < 0.005: # i2c failure probability of 0.5 %
        #    iv = -1
        return iv

    # update parameters (from new vales given by V-REP simulator)
    def vrep_update_sim_param (self,simulationTime,
                               vEncoderFrontLeft,vEncoderFrontRight,
                               vEncoderRearLeft,vEncoderRearRight,
                               vSonarFrontLeft,vSonarFrontRight,
                               vSonarLeft,vSonarRight,vSonarFront,vSonarRear,
                               vHeading,vVoltage,
                               vMotorDirectionLeft,vMotorDirectionRight,
                               vXRob,vYRob,vZRob,
                               vXAcc,vYAcc,vZAcc,
                               vXGyro,vYGyro,vZGyro):
        #print ("update params from vrep ...")
        self.vSimTime = simulationTime
        self.__vLocation = [vXRob,vYRob,vZRob]
        self.__vAccel = [vXAcc,vYAcc,vZAcc]
        self.__vGyro = [vXGyro,vYGyro,vZGyro]
        
        #print(self.__mission_time())

        self.__vEncoderFrontLeft = self.actual_front_encoders(vEncoderFrontLeft)
        self.__vEncoderFrontRight = self.actual_front_encoders(vEncoderFrontRight)
        self.__vEncoderRearLeft =  self.actual_rear_encoders(vEncoderRearLeft)
        self.__vEncoderRearRight =  self.actual_rear_encoders(vEncoderRearRight)
        self.__vSonarFrontLeft = self.actual_sonar(vSonarFrontLeft)
        self.__vSonarFrontRight = self.actual_sonar(vSonarFrontRight)
        self.__vSonarLeft = self.actual_sonar(vSonarLeft)
        self.__vSonarRight = self.actual_sonar(vSonarRight)
        self.__vSonarFront = self.actual_sonar(vSonarFront)
        self.__vSonarRear = self.actual_sonar(vSonarRear)
        #print ("trex enc ",vEncoderFrontLeft,vEncoderFrontRight)
        self.__trex.status["left_encoder"] =  self.actual_front_encoders(vEncoderFrontLeft)
        self.__trex.status["right_encoder"] = self.actual_front_encoders(vEncoderFrontRight)
        #print (self.__trex.status)
        if self.__trex.update_motor:
            actual_update = False
            if  self.__vMotorCmdLeft != self.__trex.command["left_motor_speed"]:
                actual_update = True
            if  self.__vMotorCmdRight != self.__trex.command["right_motor_speed"]:
                actual_update = True
            if actual_update:
                print ("dartv2 - update motors cmd",
                       self.__trex.command["left_motor_speed"],
                       self.__trex.command["right_motor_speed"])
                self.__vMotorCmdLeft = self.__trex.command["left_motor_speed"]
                self.__vMotorCmdRight = self.__trex.command["right_motor_speed"]
                #self.__vMotorCmdLastLeft = self.__vMotorLeft
                #self.__vMotorCmdLastRight = self.__vMotorRight
            self.__trex.update_motor = False
        #else:
        #    self.__vMotorLeft = self.__vMotorLeft_Mem
        #    self.__vMotorRight = self.__vMotorRight_Mem
        self.__vHeading = vHeading
        self.__vVoltage = vVoltage

        # update
        #print ("update")
        self.vSimVar.heading = vHeading
        self.vSimVar.sonar_front_left = vSonarFrontLeft
        self.vSimVar.sonar_front_right = vSonarFrontRight
        self.vSimVar.sonar_left = vSonarLeft
        self.vSimVar.sonar_right = vSonarRight
        self.vSimVar.sonar_front = vSonarFront
        self.vSimVar.sonar_rear = vSonarRear
        self.vSimVar.heading = vHeading
        self.vSimVar.voltage = vVoltage
        self.vSimVar.odo_front_left = vEncoderFrontLeft
        self.vSimVar.odo_front_right = vEncoderFrontRight
        self.vSimVar.odo_rear_left = vEncoderRearLeft
        self.vSimVar.odo_rear_right = vEncoderRearRight
        self.vSimVar.motor_direction_right = vMotorDirectionRight
        self.vSimVar.motor_direction_left = vMotorDirectionLeft


    # give acces to the communication socket with V-REP
    #   (for debug)
    @property
    def s(self):
        return self.__s

    # simulation running when True
    @property
    def simulation_alive(self):
        return self.__simulation_alive

    # real (False) or virtual (True) Dart V2 
    @property
    def dartSim(self):
        return self.__dartSim

    # give access to location of the robot in virtual world
    @property
    def vLocation(self):
        return self.__vLocation

    
    
    # give access to the sensors and actuators 
    @property
    def trex(self):
        return self.__trex
    @property
    def sonars(self):
        return self.__sonars
    @property
    def encoders(self):
        return self.__encoders
    @property
    def imu(self):
        return self.__imu
    @property
    def missionTime(self):
        return self.__mission_time()

    # stop the robot (end of mission)
    def stop(self):
        """
        Fully stop the DART robot , motor speeds to 0
        and also stops the virtual DART (in case of simulation)
        """
        self.__trex.command["left_motor_speed"] = 0
        self.__trex.command["right_motor_speed"] = 0
        self.__trex.i2c_write()
        if self.__dartSim:
            self.__vMotorCmdLeft = 0
            self.__vMotorCmdRight = 0
            wait_for_end = True
            t_end = time.time()+1.0
            dz = self.motor_angular_speed(self.__vDeadZoneDown)
            while wait_for_end:
                time.sleep(0.05)
                cmdLeft = self.__vMotorAngSpeedDriftVrepLeft
                cmdRight = self.__vMotorAngSpeedDriftVrepRight
                wait_for_end = (abs(cmdLeft) > dz) and (abs(cmdRight) > dz)
                #print ("stop",dz,cmdLeft,cmdRight)
                if time.time() > t_end:
                    break
            self.__vMotorAngSpeedVrepLeft = 0.0
            self.__vMotorAngSpeedVrepRightt = 0.0
            time.sleep(0.1)     
            # save logged data : uncomment for debug 
            #log_name  = "dartv2.log"
            #pickle.dump(self.tlog,open(log_name,"wb"))
            print ("stops virtual DART v2")
            self.__simulation_alive = False


if __name__ == "__main__":
    print ("start")
    if len(sys.argv) < 3:
        print ("needs to fix left and right speeds :")
        print ("python3 dart speedLeft speedRight")
        print ("exit!")
        exit()

    myDart = DartV2()

    speedL = int(sys.argv[1])
    speedR = int(sys.argv[2])

    myDart.trex.command["left_motor_speed"] = speedL
    myDart.trex.command["right_motor_speed"]= speedR
    myDart.trex.i2c_write()

    for i in range(5):
        print ("-----------------------------------")
        print (myDart.encoders.battery_voltage())
        print (myDart.encoders.read_encoders())
        print (myDart.sonars.read_4_sonars())
        print (myDart.sonars.read_diag())
        print ([myDart.trex.status['left_encoder'],
                myDart.trex.status['right_encoder']])
        time.sleep(0.25)

    myDart.trex.command["left_motor_speed"] = -speedL
    myDart.trex.command["right_motor_speed"]= -speedR
    myDart.trex.i2c_write()

    for i in range(5):
        print ("-----------------------------------")
        print (myDart.encoders.battery_voltage())
        print (myDart.encoders.read_encoders())
        print (myDart.sonars.read_4_sonars())
        print (myDart.sonars.read_diag())
        print ([myDart.trex.status['left_encoder'],
                myDart.trex.status['right_encoder']])
        time.sleep(0.25)

    print (myDart.imu.read_mag_raw())

    myDart.stop()





