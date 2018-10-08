# -*- coding: utf-8 -*-
import sys
import time
import os
import math
import signal
import numpy as np
import dartv2 as dartv2
import pickle


class DartControl ():
    
    # Init DART (virtual/real)
    def __init__(self,dart0=None):
        #global drt4interrupt
        """
        DartBasicFun class contains basic low level commands 
        for sensors and motors of DartV2.
        """
        self.drt = None
        self.t0=time.time()
        self.logFileName = "dartv2_control.log"
        self.t0Mission = time.time()
        self.tLog = []
        
        self.t0Control = 0.0 # control parameters
        self.odorl0 = 0
        self.odorr0 = 0
        self.move = False
        self.dodorLast = -1
        self.dodolLast = -1
         
        self.tBoost = 0.05
        self.spdBoost = 100
        self.tBrake = 0.05
       
        self.doWallFollow = False
        self.leftWall = False
        self.rightWall = False
        self.distToWallSonar= 99.9
        self.wallFollow = True
        self.wallSide = "none"
        self.distToWall = 0.5
        self.distToWallMax = 1.0
        self.kpOdor = 0.8
        
        self.frontObstacle = False
        self.rearObstacle = False
        self.distObstacle = 99.9
        self.sonarFront = 99.9
        self.sonarLeft = 99.9
        self.sonarRear = 99.9
        self.sonarRight = 99.9
        self.dt = 0.020
        self.motion = "idle"
        self.motionImplemented=["idle","line_fwd","line_bwd","turn_inplace"]
        self.spdl = 0.0
        self.spdr = 0.0
        self.t_sonar = 0.2
        
        if dart0 is None:
            self.drt = dartv2.DartV2()
        else:
            self.drt=dart0

    def stop(self):
        """
        Fully stop the DART robot , motor speeds to 0
        and also stops the virtual DART (in case of simulation)

        Note :
        In simulation, use stop only at the full end of the programm, as it 
        deconnects the program form the simulator
        """
        self.drt.stop()
 
    def get_battery(self):
        """
        Get the tension of the battery.

        Warning :
        If less than 6.0 Volts, charger must be plugged.
        """
        vbat=self.drt.encoders.battery_voltage()
        return vbat

    def get_distance(self, key):
        """
        get distance to obstacle from a given sonar

        input prms:
        key : defines the sonar, it can be : 
               "front", "rear", "left" , "right", 
               "front_left", "front_right" , "front_diag"
        return :
        distance in meters from the sonar to the fisrt 
        detected obstacle
                 return inf if no obstacles, -1 if problem
        """
        return self.drt.sonars.get_distance(key)  # sonar key

    def get_distance_until_ok(self, key,cntmax=10):
        """
        key defines the sonar, it can be : 
           "front", "rear", "left" , "right", 
           "front_left", "front_right" , "front_diag"

        if sonar return is -1 try again cntmax times
        default value for cntmax is 10 retries

        return : 
        d : distance in meters from the sonar to the fisrt detected obstacle
            return inf if no obstacles, -1 if problem
        """
        cnt = 0
        while cnt<cntmax:
            d = self.drt.sonars.get_distance(key)  # sonar key
            if d > 0:
                break
            cnt+=1
            time.sleep(0.0005)
        return d

    def get_cardinal_sonars(self):
        """
            return the distance detected by the 4  cardinal sonars
              front,left,rear,right
            distances are in meters
            if nothing detected sonar return is  99.9
            if I2C error sonar return is -1
        """
        df,dl,db,dr = self.drt.sonars.read_4_sonars() 
        return [df/100.0,dl/100.0,db/100.0,dr/100.0]
        
    def  get_diagonal_sonars(self):
        """
            return the distance detected by the 2  diagonal sonars
              front left, front right
            distances are in meters
            if nothing detected sonar return is  99.9
            if I2C error sonar return is -1
        """
        return self.drt.sonars.read_diag() # front left, fropnt right

    def get_4_encoders(self): 
        """
        return : left and right encoders (front)
                 left and right encoders (rear)
        """
        f = self.get_front_odos()
        r = self.get_rear_odos()
        encs = []
        for i in range(2):
            encs.append(f[i])
        for i in range(2):
            encs.append(r[i])
        return encs

    def get_front_odos(self):
        """
        DART wheels are 12 cm diameter
        Virtual DART makes 200 ticks per wheel revolution
        Read from T-REX (xx ticks per revolution)
        return : odometer values (left and right) on 16 bits signed (-32768 to - 32767)
        """
        return self.drt.trex.status["left_encoder"],self.drt.trex.status["right_encoder"]

    def get_rear_odos(self):
        """
        DART wheels are 12 cm diameter
        Virtual DART makes 200 ticks per wheel revolution
        Read from PIC (xx ticks per revolution)
        return : odometer values (left and right) on 16 bits unsigned (0 to 65535)
        """
        return ( self.drt.encoders.read_encoders()) 

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

    def deltaRearOdo (self,odo1,odo0):
        """
        Return the difference betwenn two odometric mesurments
        taking into account the jump at 16 bits signed overflow
        """
        dOdo = -(odo1-odo0)
        #print ("deltaodo 0: ",odo1,odo0,dOdo)
        if dOdo < -32768:
            dOdo += 65536
        #print ("deltaodo 1: ",odo1,odo0,dOdo)
        if dOdo > 32768:
            dOdo -= 65536
        #print ("deltaodo 2: ",odo1,odo0,dOdo)
        return dOdo

    def dataLogV1(self,t0,spdl,spdr):
        """
        Acquire values from the 4 encoders and log them with time tags

        input prms :
        t0 : mission staring time
        spdl : speed left (command)
        spdr : speed right (command)
        
        return :
        log : a dictionnary with the logged values and keys
        """
        log = {"ts": 0.0, "t": 0.0, "odofl": 0, "odofr" : 0,
               "odorl": 0 , "odorr": 0 , "spdl" : spdl , "spdr" : spdr }
        odofl,odofr = self.get_front_odos()
        odorl,odorr = self.get_rear_odos()
        while odorl<0:
            time.sleep(0.0005)
            print ("------ error I2C odorl")
            odorl,dummy = self.get_rear_odos()
        while odorr<0:
            time.sleep(0.0005)
            print ("------ error I2C odorr")
            dummy,odorr = self.get_rear_odos()
        log["t"] = time.time()-t0
        log["ts"] = self.drt.missionTime 
        log["odofl"] = odofl
        log["odofr"] = odofr
        log["odorl"] = odorl
        log["odorr"] = odorr
        log["spdl"] = spdl
        log["spdr"] = spdr
        #print ("log",log["odorl"],log["odorr"])
        return log

    
    def waitForLoopTime(self,dt,t0l):
        """
        Usefull function to terminate a control loop with e given 
        execution time (to take into account the calculation already done)

        input prms:
        dt : delta time to wait for (in s)
        t0l : starting time (in s)
        """
        tw = dt - (time.time()-t0l)
        if tw > 0:
            time.sleep(tw)

    def setDistToObstacle(self,dobst,t1,spd1,spd,kp,error_max=0.1):
        """
        Set the robot at a given distance to the obstacle with a given 
        maximum error (tolerance) of 10 cm by default
        To start motors, speed is set to a higher value (spd1) for a little
        time (t1) at beginning. The speed (spd) can be lower, once the motors
        have started.
        
        input prms:
        dobst : distance to obstacle (in m)
        t1 : startup time (in s), time using speed spd1 to start the motors
        spd1 : startup speed (unit less : 0 - 255) 
        spd : speed for the motion (unit less : 0 - 255)
        kp : proprotinal controller constant (unit less)
        error_max : maximum error to reach the distance to obtacle (def 0.1 m)

        return:
        tleg : time tagged log data
        """
        tleg=[]
        while True:
            df,dl,db,dr = self.get_cardinal_sonars()
            minidist = 0.8*(df-dobst)
            print ("minidist",minidist,dobst,df)
            if abs(minidist) < error_min:
                break
            cnt = self.convertDistanceInTicks(minidist)
            sgn = 1
            if cnt < 0:
                cnt = -cnt
                sgn = -1
            print ("--------------------------- Mini Move")
            print (minidist,sgn,cnt)
            t0=time.time()
            dobst_m,t_log = self.goLineRearOdo (t1,spd1*sgn,spd*sgn,cnt,kp)
            tleg.append([t0,t_log])
            time.sleep(1.0)
        return tleg


    def convertDistanceInTicks(self,dist):
        """
        Convert a distance into a number of ticks
        Use a wheel diameter of 12.5 cm and 300 ticks per revolution

        input prms:
        dist : distance in m
        
        return
        ticks : number of ticks (unit less)
        """
        diam = 0.125
        ticks_per_revol =  300.0
        ticks = int(round(ticks_per_revol*dist/(math.pi*diam)))
        return ticks
            
    def goLineRearOdo (self,ts1,spd1,spd,cnt,kp,dfront=None):
        """
        Do a line using odometric control (rear odometers)
        Input params:
           ts1 : duration at speed spd1 at start and at 0 at end
           spd1 : speed at start of the motion
           spd : speed during the modtion
           cnt : number of ticks
           kp : coef of prop controler
           dfront : distance min to front obstacle (default None)
        Output params:
           dobst : dist to obstacle , -1 if no obstacle
           tlog : an array with the logged data (a dict for each 
                  time stamped data)
        If spd1 and spd < 0, motion goes backwards
        """
        t0 = time.time()
        tLog = []
        dt = 0.020  # 0.005
        spdl = 0 # very little time at 0 for safety (may be removed)
        spdr = 0
        self.set_speed(spdl,spdr)
        while (time.time()-t0) < 0.2*ts1:
            pass
        log = self.dataLogV1(t0,spdl,spdr) # log the data
        tLog.append(log)
        # setup obstacle avoidance
        if dfront is None:
            dt_obst = False
        else:
            dt_obst = True
            t_obst = 0.2
        # ckeck if obstacle before starting
        move = True
        if dt_obst:
            df,dl,db,dr = self.get_cardinal_sonars()
            if spd1 > 0 and df < dfront:
                dobst = df
                move = False
            if spd1 < 0 and db < dfront:
                dobst = db
                move = False
        if move:
            # init left and right counters
            curcntl=0  
            curcntr=0
            # set memory odo values to compute delta odo
            odorl0 = log["odorl"]
            odorr0 = log["odorr"]
            # set speed to spd1 up to ts1
            spdl = spd1
            spdr = spd1
            self.set_speed(spdl,spdr)
            if dt_obst:
                tcur_obst = time.time()
            while (time.time()-t0) < ts1:
                t0l = time.time()
                log = self.dataLogV1(t0,spdl,spdr)
                tLog.append(log)
                odorl=log["odorl"]
                odorr=log["odorr"]
                dodol = self.deltaRearOdo(odorl,odorl0)
                dodor = self.deltaRearOdo(odorr,odorr0)
                curcntl += dodol
                curcntr += dodor
                errdodo = (dodor-dodol)
                spdl = spdl + kp * errdodo
                spdr = spdr - kp * errdodo
                # debug (detect unusual jumps on odos)
                #if abs(dodol) > 50 or abs(dodor) > 50 :
                #    print ("ph1",dodol,dodor,curcntl,curcntr,
                #           odorl,odorl0,odorr,odorr0,
                #           errdodo,spdl,spdr)
                self.set_speed(spdl,spdr)
                odorl0 = odorl
                odorr0 = odorr
                if dt_obst:
                    if (time.time()-tcur_obst) > t_obst:
                        df,dl,db,dr = self.get_cardinal_sonars()
                        if spd1 > 0 and df < dfront:
                            dobst = df
                            move = False
                            break
                        if spd1 < 0 and db < dfront:
                            dobst = db
                            move = False
                            break
                        tcur_obst = time.time()
                self.waitForLoopTime(dt,t0l)
        if move:
            # 
            # set the control speed (but keep the corrections of previous loop)
            dspd = spd-spd1
            spdl = spdl + dspd
            spdr = spdr + dspd
            self.set_speed(spdl,spdr)
            while True:
                t0l = time.time()
                log = self.dataLogV1(t0,spdl,spdr)
                tLog.append(log)
                odorl=log["odorl"]
                odorr=log["odorr"]
                dodol = self.deltaRearOdo(odorl,odorl0)
                dodor = self.deltaRearOdo(odorr,odorr0)
                curcntl += dodol
                curcntr += dodor
                curcnt = (curcntl+curcntr)/2
                if abs(curcnt) > abs(cnt):
                    print ("end cnt",cnt,curcnt,curcntl,curcntr)
                    break
                errdodo = (dodor-dodol)
                spdl = spdl + kp * errdodo
                spdr = spdr - kp * errdodo
                # debug (detect unusual jumps on odos)
                #if abs(dodol) > 50 or abs(dodor) > 50 :
                #    print ("ph2",dodol,dodor,curcntl,curcntr,
                #           odorl,odorl0,odorr,odorr0,
                #           errdodo,spdl,spdr)
                self.set_speed(spdl,spdr)
                odorl0 = odorl
                odorr0 = odorr
                if dt_obst:
                    if (time.time()-tcur_obst) > t_obst:
                        df,dl,db,dr = self.get_cardinal_sonars()
                        if spd > 0 and df < dfront:
                            dobst = df
                            move = False
                            break
                        if spd < 0 and db < dfront:
                            dobst = db
                            move = False
                            break
                        tcur_obst = time.time()
                self.waitForLoopTime(dt,t0l)
        # for safety, put motors to 0 for a durantion ts1
        spdl = 0
        spdr = 0
        self.set_speed(spdl,spdr)
        t01 = time.time()
        while (time.time()-t01) < ts1:
            t0l = time.time()
            log = self.dataLogV1(t0,spdl,spdr)
            tLog.append(log)
            self.waitForLoopTime(dt,t0l)
        dist_obst = -1
        if dt_obst and (not move):
            dist_obst = dobst
        return dist_obst,tLog

    def wall_follow_bang(self,fwd,side,dw,dwcur):
        """
        compute correction for wall following
        """
        verbose = False
        whl=0.0
        whr=0.0
        bang = 15.0
        if side == "right":
            err = dw - dwcur
            if err > 0.02:
                whl=-bang
                whr=bang
            elif err < -0.02:
                whl=bang
                whr=-bang
        elif side == "left":
            pass
        if not fwd:
            whl = -whl
            whr = -whr            
        if verbose:
            print ("wall",whl,whr)
        return whl,whr

    def wall_follow(self,fwd,side,dw,dwcur):
        """
        compute correction for wall following
        """
        verbose = False
        whl=0.0
        whr=0.0
        kp=5.0
        err = (dw - dwcur) * 100
        ker = round(kp*err)
        cor = False
        cormx=40
        if (abs(err) < 100):  # do not follow if error is > 1 m 
            if ker > cormx:
                ker = cormx
            if ker < -cormx:
                ker = -cormx
            if side == "right":
                whl = -ker
                whr = ker
            elif side == "left":
                whl = ker
                whr = -ker
            if not fwd:
                whl = -whl
                whr = -whr
            cor = True
        if verbose:
            print ("wall",whl,whr,err,ker,cor)
        return whl,whr

    def goLineRearOdoAndSonar (self,ts1,spd1,spd,cnt,kp,dfront,dwall,side,dwall_mx=1000.0):
        """
        Do a line using odometric control (rear odometers)
        Input params:
           ts1 : duration at speed spd1 at start and at 0 at end
           spd1 : speed at start of the motion
           spd : speed during the modtion
           cnt : number of ticks
        if not fwd:
            whl = -whl
            whr = -whr
           kp : coef of prop controler
           dfront : distance min to front obstacle
           dwall : distance to the wall
           side : side of the wall , left or right
           dwall_mx : maximum distance to the wall
        Output params:
           dobst : dist to obstacle , -1 if no obstacle
           dwall_ok : dist to wall, -1 if not detected
           tlog : an array with the logged data (a dict for each 
                  time stamped data)
        If spd1 and spd < 0, motion goes backwards
        """
        t0 = time.time()
        verbose = False
        verbose = True
        tLog = []
        dobst = -1
        dwall_ok = -1
        dt = 0.020  # 0.005
        fwd = True
        if spd < 0:
            fwd = False
        wall_l = 0.0
        wall_r = 0.0
        spdl = 0 # very little time at 0 for safety (may be removed)
        spdr = 0
        self.set_speed(spdl,spdr)
        while (time.time()-t0) < 0.2*ts1:
            pass
        log = self.dataLogV1(t0,spdl,spdr) # log the data
        tLog.append(log)
        # setup sonar time refresh
        t_sonar = 0.2
        # ckeck if obstacle before starting
        move = True
        df,dl,db,dr = self.get_cardinal_sonars()
        if spd1 > 0 and df < dfront: # test is obstacle forwards
            dobst = df
            move = False
        if spd1 < 0 and db < dfront: # test if obstacle backwards
            dobst = db
            move = False
        doWallFollow = False
        if side == "right":
            if abs(dr-dwall) > dwall_mx:
                dwall_ok = -1
                doWallFollow = False
            else:
                dwall_ok = dr
                doWallFollow = True
        elif side == "left":
            if abs(dl-dwall) > dwall_mx:
                dwall_ok = -1
                doWallFollow = False
            else:
                dwall_ok = dl
                doWallFollow = True
        if move: # no obstacle , move is possible
            # init left and right counters
            curcntl=0  
            curcntr=0
            # set memory odo values to compute delta odo
            odorl0 = log["odorl"]
            odorr0 = log["odorr"]
            # set speed to spd1 up to ts1
            spdl = spd1
            spdr = spd1
            self.set_speed(spdl,spdr)
            tcur_sonar = time.time()
            while (time.time()-t0) < ts1:
                t0l = time.time()
                log = self.dataLogV1(t0,spdl,spdr)
                tLog.append(log)
                odorl=log["odorl"]
                odorr=log["odorr"]
                dodol = self.deltaRearOdo(odorl,odorl0)
                dodor = self.deltaRearOdo(odorr,odorr0)
                curcntl += dodol
                curcntr += dodor
                errdodo = (dodor-dodol)
                spdl = spdl + kp * errdodo
                spdr = spdr - kp * errdodo
                if verbose:
                    print ("spd l,r",spdl+wall_l,spdr+wall_r)
                self.set_speed(spdl+wall_l,spdr+wall_r)
                odorl0 = odorl
                odorr0 = odorr
                # acquire and process sonar at slower rate than odos
                if (time.time()-tcur_sonar) > t_sonar: 
                    df,dl,db,dr = self.get_cardinal_sonars()
                    if spd1 > 0 and df < dfront:
                        dobst = df
                        move = False
                        break
                    if spd1 < 0 and db < dfront:
                        dobst = db
                        move = False
                        break
                    if side == "right":
                        if abs(dr-dwall) > dwall_mx:
                            dwall_ok = -1
                            doWallFollow = False
                        else:
                            dwall_ok = dr
                            doWallFollow = True
                        dwall_cur = dr
                        if verbose:
                            print ("distance to right wall , follow, move : ",dwall_cur,doWallFollow,move)
                    elif side == "left":
                        if abs(dr-dwall) > dwall_mx:
                            dwall_ok = -1
                            doWallFollow = False
                        else:
                            dwall_ok = dr
                            doWallFollow = True
                        dwall_cur = dl
                        if verbose:
                            print ("distance to left wall , follow, move : ",dwall_cur,doWallFollow,move)
                    tcur_sonar = time.time()
                self.waitForLoopTime(dt,t0l)
        if verbose:
            print ("Main move,follow,df,dr",move,doWallFollow,df,dr)
        if move:
            # 
            # set the control speed (but keep the corrections of previous loop)
            dspd = spd-spd1
            spdl = spdl + dspd
            spdr = spdr + dspd
            self.set_speed(spdl,spdr)
            while True:
                if verbose:
                    print ("(1) move,follow",move,doWallFollow)
                t0l = time.time()
                log = self.dataLogV1(t0,spdl,spdr)
                tLog.append(log)
                odorl=log["odorl"]
                odorr=log["odorr"]
                dodol = self.deltaRearOdo(odorl,odorl0)
                dodor = self.deltaRearOdo(odorr,odorr0)
                curcntl += dodol
                curcntr += dodor
                curcnt = (curcntl+curcntr)/2
                if abs(curcnt) > abs(cnt):
                    print ("end cnt",cnt,curcnt,curcntl,curcntr)
                    break
                errdodo = (dodor-dodol)
                spdl = spdl + kp * errdodo
                spdr = spdr - kp * errdodo
                if doWallFollow:
                    if verbose:
                        print ("spd l,r, wall follow",round(spdl+wall_l),round(spdr+wall_r),doWallFollow)
                    self.set_speed(spdl+wall_l,spdr+wall_r)
                else:
                    if verbose:
                        print ("spd l,r, wall follow",round(spdl),round(spdr),doWallFollow)
                    self.set_speed(spdl,spdr)
                odorl0 = odorl
                odorr0 = odorr
                if (time.time()-tcur_sonar) > t_sonar:                    
                    df,dl,db,dr = self.get_cardinal_sonars()
                    if verbose:
                        print ("sonars",df,dl,db,dr)
                    if spd > 0 and df < dfront:
                        dobst = df
                        if verbose:
                            print ("front obstacle at : ",dobst)
                        move = False
                        break
                    if spd < 0 and db < dfront:
                        dobst = db
                        if verbose:
                            print ("rear obstacle at : ",dobst)
                        move = False
                        break
                    if side == "right":
                        dwall_cur = dr
                        if verbose:
                            print ("distance to right wall , follow, move : ",dwall_cur,doWallFollow,move)
                        if abs(dr-dwall) > dwall_mx:
                            dwall_ok = -1
                            if doWallFollow:
                                move=False
                                break
                            doWallFollow = False
                        else:
                            doWallFollow = True
                            dwall_ok = dr
                    elif side == "left":
                        dwall_cur = dl
                        if verbose:
                            print ("distance to left wall : ",dwall_cur)
                        if abs(dl-dwall) > dwall_mx:
                            dwall_ok = -1
                            if doWallFollow:
                                move = False
                                break
                            doWallFollow = False
                        else:
                            doWallFollow = True
                            dwall_ok = dl
                    if doWallFollow:
                        wall_l,wall_r = self.wall_follow(fwd,side,dwall,dwall_cur)
                        if verbose:
                            print (" -----------  wall follow correction",wall_l,wall_r)
                    #wall_l,wall_r = self.wall_follow(fwd,side,dwall,dwall_cur)
                    tcur_obst = time.time()
                self.waitForLoopTime(dt,t0l)
        # for safety, put motors to 0 for a durantion ts1
        spdl = 0
        spdr = 0
        self.set_speed(spdl,spdr)
        t01 = time.time()
        while (time.time()-t01) < ts1:
            t0l = time.time()
            log = self.dataLogV1(t0,spdl,spdr)
            tLog.append(log)
            self.waitForLoopTime(dt,t0l)
        dist_obst = -1
        if not move:
            dist_obst = dobst
        return dist_obst,dwall_ok,tLog
        
    def convertAngleAlongRadiusInTicks(self,angle,radius):
        verbose = True
        ticks_per_revol =  300.0
        diam = 0.125
        dlen = 0.16
        dwid = 0.26
        dlen2 = dlen/2.0
        dwid2 = dwid/2.0
        ang_drift_intern = math.atan(dlen2/(radius-dwid2))
        ang_drift_extern = math.atan(dlen2/(radius+dwid2))
        ang_drift_radius = math.atan(dlen2/radius)
        delta_perim_intern = angle*2*math.pi*(radius-dwid2)/360.0
        nb_turn_intern = delta_perim_intern/(math.pi*diam)
        ticks_intern = nb_turn_intern * ticks_per_revol
        ticks_intern /= math.cos(ang_drift_intern)
        delta_perim_extern = angle*2*math.pi*(radius+dwid2)/360.0
        nb_turn_extern = delta_perim_extern/(math.pi*diam)
        ticks_extern = nb_turn_extern * ticks_per_revol
        ticks_extern /= math.cos(ang_drift_extern)
        delta_perim_radius = angle*2*math.pi*(radius)/360.0
        nb_turn_radius = delta_perim_radius/(math.pi*diam)
        ticks_radius = nb_turn_radius * ticks_per_revol
        ticks_radius /= math.cos(ang_drift_radius)
        if verbose:
            print ("ticks int,ext,radius",ticks_intern,ticks_extern,ticks_radius)
        return ticks_intern,ticks_extern,ticks_radius
        
    # turnAlongRadiusRearOdo
    def turnAlongRadiusRearOdo(self,ts1,spd1,spd0,spdmx,angle,radius,kp,side,dfront=None):
        t0 = time.time()
        verbose = True
        tLog = []
        #dt = 0.005
        dt = 0.020
        curcntl=0
        curcntr=0
        move = True
        spdmn = 40
        
        spdl = 0.0
        spdr = 0.0
        ticks_int,ticks_ext,ticks_mid = self.convertAngleAlongRadiusInTicks(angle,radius)
        coef_ext = float(ticks_ext)/float(ticks_mid)
        coef_int = float(ticks_int)/float(ticks_mid)
        coef_mid = float(ticks_ext)/float(ticks_int)
        
        spdl = spd1*coef_mid
        spdr = spd1
        if spdl > 250:
            spdl = 250
        self.set_speed(spdl,spdr)
        log = self.dataLogV1(t0,spdl,spdr)
        tLog.append(log)
        odorl0 = log["odorl"]
        odorr0 = log["odorr"]

        while (time.time()-t0) < ts1:
            t0l = time.time()
            log = self.dataLogV1(t0,spdl,spdr)
            tLog.append(log)
            odorl=log["odorl"]
            odorr=log["odorr"]
            dodol = self.deltaRearOdo(odorl,odorl0)
            dodor = self.deltaRearOdo(odorr,odorr0)
            curcntl += dodol
            curcntr += dodor
            odorl0 = odorl
            odorr0 = odorr
            if verbose:
                if curcntr != 0:
                    print (coef_mid,curcntl/curcntr,round(spdl),round(spdr))
            #self.set_speed(spdl,spdr)
            self.waitForLoopTime(dt,t0l)
        
     
        spdl = spd0*coef_mid
        spdr = spd0
        if verbose:
            print ("before theo - spdl,spdr",round(spdl),round(spdr))
        if spdl > spdmx:
            spdl = spdmx
        if spdr < spdmn:
            spdr = spdmn       
        if verbose:
            print ("before actu - spdl,spdr",round(spdl),round(spdr))
        self.set_speed(spdl,spdr)
        t0o = time.time()
        tupdo = 0.02
        doTune = True
        while True:
            t0l = time.time()
            log = self.dataLogV1(t0,spdl,spdr)
            tLog.append(log)
            odorl=log["odorl"]
            odorr=log["odorr"]
            dodol = self.deltaRearOdo(odorl,odorl0)
            dodor = self.deltaRearOdo(odorr,odorr0)
            curcntl += dodol
            curcntr += dodor
            err_ext = ticks_ext-curcntl
            err_int = ticks_int-curcntr
            stopl = False
            if err_ext <= 0:
                stopl = True
            stopr = False
            if err_int <= 0:
                stopr = True
            if stopl and stopr:
            #if curcnt>cnt:
            #if curcntl>cnt or curcntr>cnt:
                print ("end cnt",ticks_ext,ticks_int,curcntl,curcntr)
                break
            odorl0 = odorl
            odorr0 = odorr
            """
            correct_l = 0.0
            correct_r = 0.0
            if curcntl > 100:
                coef_m = curcntl/curcntr
                correct = (coef_mid/coef_m) -1
                correct_l = 50 * coef_mid * correct 
                correct_r = -50 * correct
                print (coef_mid,coef_m,curcntl,curcntr,correct,correct_l,correct_r)
            spdl = spdl+correct_l
            spdr = spdr+correct_r
            """
            if curcntr > 10 and time.time()-t0o > tupdo:
                t0o = time.time()
                coef_m = curcntl/curcntr
                if verbose:
                    print ("curcntl,curcntr,spdl,spdr,coef_m",curcntl,curcntr,round(spdl),round(spdr),coef_m)
                err_ext_rel = curcntl/ticks_ext
                err_int_rel = curcntr/ticks_int
                #err_rel = (err_ext_rel+err_int_rel)/2.0
                #correct_l = 500.0* (err_rel - err_ext_rel)
                #correct_r = 0.0
                #if verbose:
                #    print (coef_mid,coef_m,err_ext_rel,err_int_rel,err_rel,correct_l,correct_r)
                   
                err_ext = coef_mid - coef_m
                correct_l = 5.0*err_ext
                correct_r = 0                    
                if verbose:
                    print (coef_mid,coef_m,err_ext,err_ext_rel,err_int_rel,correct_l,correct_r)
                if doTune:
                    spdl = spdl+correct_l
                    spdr = spdr+correct_r
                if spdl > spdmx:
                    spdl = spdmx
                if spdr > spdmx:
                    spdr = spdmx
                if spdl < spdmn:
                    spdl = spdmn
                if spdr < spdmn:
                    spdr = spdmn
                #if stopl:
                #    spdl = 0
                #if stopr:
                #    spdr = 0
                #print (cnt,curcnt,curcntl,curcntr,
                #       dodol,dodor,err,
                #       round(spdl),
                #       round(spdl),round(spdr))
                #if abs(spdl) > 250 or  abs(spdr) > 250:
                #    print ("break cnt",cnt,curcntl,curcntr)
                #    break
                if verbose:
                    print ("spdl,spdr",round(spdl),round(spdr))
                #if abs(err_ext) < 0.05:
                #    doTune=False
            self.set_speed(spdl,spdr)
            self.waitForLoopTime(dt,t0l)

        # for safety, put motors to 0 for a durantion ts1
        spdl = 0
        spdr = 0
        self.set_speed(spdl,spdr)
        t01 = time.time()
        while (time.time()-t01) < ts1:
            t0l = time.time()
            log = self.dataLogV1(t0,spdl,spdr)
            tLog.append(log)
            self.waitForLoopTime(dt,t0l)
        dist_obst = -1
        if not move:
            dist_obst = dobst
        return dist_obst,tLog
        

    def convertTurnAngleInTicks(self,angle):
        dlen = 0.16
        dwid = 0.26
        dlen2 = dlen/2.0
        dwid2 = dwid/2.0
        ang_drift = math.atan(dlen/dwid)
        diam = 0.125
        r = math.sqrt(dlen2*dlen2+dwid2*dwid2)
        ticks_per_revol =  300.0
        ticks_per_360 = (2*r/diam)*ticks_per_revol
        ticks_per_360_drift = ticks_per_360/math.cos(ang_drift)
        #print (ang_drift*180.0/math.pi,r,ticks_per_360,ticks_per_360_drift,
        #       1/math.cos(ang_drift))
        ticks = int(round(ticks_per_360_drift*angle/360.0))
        return ticks

    def turnAngleRearOdo (self,ts1,spd1,spd0,spdmx,angle,kp,side):
        """
        turn on place using rear odometers
        input params:
            ts1 : time as start speed spd1 and time at 0 at the end
            spd1 : start speed
            spd0 : fine control speed
            spdmx : max control speed
            angle : rotation angle 
            kp : coef for proportional control
            side : "left' or "right"
        output params:
            tlog : logged data
        the rotation angle is always positive, to change rotation direction
        use the side parameter (left ou right)
        """
        cnt = self.convertTurnAngleInTicks(angle)
        tlog = self.turnRearOdo(ts1,spd1,spd0,spdmx,cnt,kp,side)
        return tlog

        
    def turnRearOdo (self,ts1,spd1,spd0,spdmx,cnt,kp,side):
        """
        turn on place using rear odometers
        input params:
            ts1 : time as start speed spd1 and time at 0 at the end
            spd1 : start speed
            spd0 : fine control speed
            spdmx : max control speed
            cnt : odometer count to reach
            kp : coef for proportional control
            side : "left' or "right"
        output params:
            tlog : logged data
        """
        t0 = time.time()
        verbose = False
        tLog = []
        #dt = 0.005
        dt = 0.010
        if side == "left":
            sgnl = -1
            sgnr = 1
        else:
            sgnl = 1
            sgnr = -1
        #spdl = 0
        #spdr = 0
        #self.set_speed(spdl,spdr)
        #log = self.dataLogV1(t0,spdl,spdr)
        #tLog.append(log)
        curcntl=0
        curcntr=0
        spdl = spd1*sgnl
        spdr = spd1*sgnr
        self.set_speed(spdl,spdr)
        log = self.dataLogV1(t0,spdl,spdr)
        tLog.append(log)
        odorl0 = log["odorl"]
        odorr0 = log["odorr"]
        while (time.time()-t0) < ts1:
            t0l = time.time()
            log = self.dataLogV1(t0,spdl,spdr)
            tLog.append(log)
            odorl=log["odorl"]
            odorr=log["odorr"]
            dodol = self.deltaRearOdo(odorl,odorl0)
            dodor = self.deltaRearOdo(odorr,odorr0)
            dodols = dodol*sgnl
            dodors = dodor*sgnr
            
            curcntl += dodols
            curcntr += dodors
            if verbose:
                print (side,sgnl,sgnr,curcntl,curcntr,
                       odorl0,odorr0,odorl,odorr,
                       dodols,dodors)
            odorl0 = odorl
            odorr0 = odorr
            #errdodo = (dodor-dodol)
            #spdl = spdl + kp * errdodo
            #spdr = spdr - kp * errdodo
            self.set_speed(spdl,spdr)
            self.waitForLoopTime(dt,t0l)
        #dspd = spd-spd1
        #spdl = spdl + dspd*sgnl
        #spdr = spdr + dspd*sgnr
        #self.set_speed(spdl,spdr)
        while True:
            t0l = time.time()
            log = self.dataLogV1(t0,spdl,spdr)
            tLog.append(log)
            odorl=log["odorl"]
            odorr=log["odorr"]
            dodol = self.deltaRearOdo(odorl,odorl0)*sgnl
            dodor = self.deltaRearOdo(odorr,odorr0)*sgnr
            curcntl += dodol
            curcntr += dodor
            curcnt = (curcntl+curcntr)/2
            err = cnt-curcnt
            sgne = 1
            if err<0:
                sgne=-1
            if abs(err) < 3:
            #if curcnt>cnt:
            #if curcntl>cnt or curcntr>cnt:
                print ("end cnt",cnt,curcnt,curcntl,curcntr)
                break
            odorl0 = odorl
            odorr0 = odorr
            spd = spd0 + (spd1-spd0) * err * kp
            spdl = spd * sgnl * sgne
            spdr = spd * sgnr * sgne
            if spdl > spdmx:
                spdl = spdmx
            if spdr > spdmx:
                spdr = spdmx
            if spdl < -spdmx:
                spdl = -spdmx
            if spdr < -spdmx:
                spdr = -spdmx
            if verbose:
                print (cnt,curcnt,curcntl,curcntr,
                       dodol,dodor,err,
                       round(spdl),
                       round(spdl),round(spdr))
            #if abs(spdl) > 250 or  abs(spdr) > 250:
            #    print ("break cnt",cnt,curcntl,curcntr)
            #    break
            self.set_speed(spdl,spdr)
            self.waitForLoopTime(dt,t0l)
        spdl = 0
        spdr = 0
        self.set_speed(spdl,spdr)
        t01 = time.time()
        while (time.time()-t01) < ts1:
            t0l = time.time()
            log = self.dataLogV1(t0,spdl,spdr)
            tLog.append(log)
            self.waitForLoopTime(dt,t0l)
        return tLog
        
    """
     new functions for data log
    """
    
    def dataLogInit(self,logFileName):
        self.logFileName = logFileName
        self.t0Mission = time.time()
        self.tLog = []

    def dataLogSpeedAndFourOdos(self):
        """
        Acquire values from the 4 encoders
        Log the data in a dict
        """
        log = {"ts": 0.0, "t": 0.0, "odofl": 0, "odofr" : 0,
               "odorl": 0 , "odorr": 0 , "spdl" : self.spdl , "spdr" : self.spdr }
        odofl,odofr = self.get_front_odos()
        odorl,odorr = self.get_rear_odos()
        while odorl<0:
            time.sleep(0.0005)
            print ("------ error I2C odorl")
            odorl,dummy = self.get_rear_odos()
        while odorr<0:
            time.sleep(0.0005)
            print ("------ error I2C odorr")
            dummy,odorr = self.get_rear_odos()
        log["t"] = time.time()-self.t0Mission
        log["ts"] = self.drt.missionTime
        log["odofl"] = odofl
        log["odofr"] = odofr
        log["odorl"] = odorl
        log["odorr"] = odorr
        log["spdl"] = self.spdl
        log["spdr"] = self.spdr
        self.tLog.append(log)
        return odofl,odofr,odorl,odorr

    def dataLogSpeedAndRearOdos(self):
        """
        Acquire values from the 4 encoders
        Log the data in a dict
        """
        log = {"ts": 0.0, "t": 0.0, 
               "odorl": 0 , "odorr": 0 , "spdl" : self.spdl , "spdr" : self.spdr }
        odorl,odorr = self.get_rear_odos()
        while odorl<0:
            time.sleep(0.0005)
            print ("------ error I2C odorl")
            odorl,dummy = self.get_rear_odos()
        while odorr<0:
            time.sleep(0.0005)
            print ("------ error I2C odorr")
            dummy,odorr = self.get_rear_odos()
        log["t"] = time.time()-self.t0Mission
        log["ts"] = self.drt.missionTime
        log["odorl"] = odorl
        log["odorr"] = odorr
        log["spdl"] = self.spdl
        log["spdr"] = self.spdr
        self.tLog.append(log)
        return odorl,odorr

    def dataLogSonars(self):
        """
        Acquire values from the 4 cardinal sonars
        Log the data in a dict
        """
        log = {"ts": 0.0, "t": 0.0, "front": 0.0, "left" : 0.0,
               "rear": 0.0 , "right": 0.0  }
        df,dl,db,dr = self.get_cardinal_sonars()
        log["t"] = time.time()-self.t0Mission
        log["ts"] = self.drt.missionTime
        log["front"] = df
        log["left"] = dl
        log["rear"] = db 
        log["right"] = dr
        self.tLog.append(log)
        return df,dl,db,dr

        
    def setBoostParams(self,tBoost,spdBoost):            
        self.tBoost = tBoost
        self.spdBoost = spdBoost

    """
        self.t0Control = 0.0 # control parameters
        self.odorl0 = 0
        self.odorr0 = 0
        self.move = False
        
        self.tBoost = 0.05
        self.spdBoost = 100
        self.tBrake = 0.05

        self.doWallFollow = False
        self.leftWall = False
        self.rightWall = False
        self.distToWallSonar= 99.9
        self.wallFollow = True
        self.wallSide = "none"
        self.distToWall = 0.5
        self.distToWallMax = 1.0
        self.kpOdor = 0.8
        
        self.frontObstacle = False
        self.rearObstacle = False
        self.distObstacle = 99.9
        self.sonarFront = 99.9
        self.sonarLeft = 99.9
        self.sonarRear = 99.9
        self.sonarRight = 99.9
        self.dt = 0.020
        self.motion = "idle"
        self.motionImplemented=["idle","line_fwd","line_bwd","turn_inplace"]
        self.spdl = 0.0
        self.spdr = 0.0
        self.t_sonar = 0.2
        
    """
    
    def initControl(self,motion,wallFollow=None,distToObstacle=None,kpOdor=None):
        """
         return False if motion is not implemented
        """
        self.t0Control = time.time()
        verbose = False
        verbose = True
        self.frontObstacle = False
        self.wallOk = False
        self.dt = 0.020
        if motion in self.motionImplemented:
            self.motion = motion
        else:
            return False   # motion non implemented
        if not (kpOdor is None):
            self.kpOdor = kpOdor  # prop coef for odometric control
        self.odol0,self.odor0 = self.dataLogSpeedAndRearOdos()  


        self.curcntl=0  # reset counters
        self.curcntr=0
        
        self.spdl = 0 # very little time at 0 for safety (may be removed)
        self.spdr = 0
        self.set_speed(self.spdl,self.spdr)
        while (time.time()-self.t0Control) < 2.0*self.dt:
            pass
        self.odorl0,self.odorr0 = self.dataLogSpeedAndRearOdos()  

        # setup sonar time refresh
        self.t_sonar = 0.2
        # ckeck if obstacle before starting
        self.move = True
        df,dl,db,dr = self.dataLogSonars()
        if not (distToObstacle is None):
            if self.motion == "line_fwd" and df < distToObstacle:
                self.distObstacle = df
                self.move = False
            if self.motion == "line_bwd" and df < distToObstacle:
                self.distObstacle = df
                self.move = False
        self.doWallFollow = False
        if not (wallFollow is None):
            self.wallFollow = True
            self.wallSide = wallFollow[0]
            self.distToWall = wallFollow[1]
            self.distToWallMax = wallFollow[2]
            if self.wallSide == "right":
                if abs(dr-self.distToWall) > self.distToWallMax:
                    self.doWallFollow = False
                else:
                    self.distToWallSonar = dr
                    self.doWallFollow = True
            elif self.wallSide == "left":
                if abs(dl-self.distToWall) > self.distToWallMax:
                    self.doWallFollow = False
                else:
                    self.distToWallSonar = dl
                    self.doWallFollow = True
        return True

            
    def lineBoost(self):
        """
        No check for obstacles during move  as front obstacle has been checked 
        before in initControl())
        Odometers counts are updated and odometric control is applied
        """
        verbose = False
        verbose = True
        if verbose:
            print ("t and spd line boost",self.tBoost,self.spdBoost)
        if self.move: # no obstacle , move is possible
            # set speed to spd1 up to ts1
            spdl = self.spdBoost
            spdr = self.spdBoost
            self.set_speed(spdl,spdr)
            t0 = time.time()
            while (time.time()-t0) < self.tBoost:
                t0l = time.time()
                odorl,odorr = self.dataLogSpeedAndRearOdos() 
                dodol = self.deltaRearOdo(odorl,self.odorl0)
                dodor = self.deltaRearOdo(odorr,self.odorr0)
                self.curcntl += dodol
                self.curcntr += dodor
                errdodo = (dodor-dodol)
                self.spdl = self.spdl + self.kpOdor * errdodo
                self.spdr = self.spdr - self.kpOdor * errdodo
                if verbose:
                    print ("spd line boost l,r",self.spdl,self.spdr)
                self.set_speed(self.spdl,self.spdr)
                self.odorl0 = odorl
                self.odorr0 = odorr
                # acquire and process sonar at slower rate than odos
                self.waitForLoopTime(self.dt,t0l)
                
    def turnBoost(self,angle):
        """
        only apply boost speed for boost duration
        update odometric counters
        """
        verbose = False
        verbose = True
        if verbose:
            print ("t and spd turn boost",self.tBoost,self.spdBoost)
        if self.move: # no obstacle , move is possible
            # set speed to spd1 up to ts1
            if angle>0:
                spdl = self.spdBoost
                spdr = -self.spdBoost
            elif angle<0:
                spdl = -self.spdBoost
                spdr = self.spdBoost                
            self.set_speed(spdl,spdr)
            t0 = time.time()
            while (time.time()-t0) < self.tBoost:
                t0l = time.time()
                odorl,odorr = self.dataLogSpeedAndRearOdos() 
                dodol = self.deltaRearOdo(odorl,self.odorl0)
                dodor = self.deltaRearOdo(odorr,self.odorr0)
                self.curcntl += dodol
                self.curcntr += dodor
                self.odorl0 = odorl
                self.odorr0 = odorr
                # acquire and process sonar at slower rate than odos
                self.waitForLoopTime(self.dt,t0l)
                
    def brakes(self,leftFwd,rightFwd,tBrake=None):
        odorl,odorr = self.dataLogSpeedAndRearOdos()  
        if not (tBrake is None):
            self.tBrake = tBrake
        spdl=255
        spdr=255
        if leftFwd:
            spdl=-255
        if rightFwd:
            spdr=-255
        self.set_speed(spdl,spdr)
        t0=time.time()
        while (time.time()-self.t0) < self.tBrake:
            pass
        self.spdl = 0.0
        self.spdr = 0.0
        self.set_speed(spdl,spdr)
        odorl,odorr = self.dataLogSpeedAndRearOdos()  
        
    def inPlaceTurn(self,angle,spd):
        cnt = self.convertTurnAngleInTicks(abs(angle))
        if cnt == 0:
            return
        if angle>0: # turn right
            cntl = cnt
            cntr = -cnt
        elif angle<0:
            cntl = -cnt
            cntr = +cnt
        self.initControl("turn_inplace")
        self.setBoostParams(0.05,120)
        self.move = True  # in place turn always possible 
        # in future check if not too close to walls before turning
        self.turnBoost(angle)
        self.move = True # in place motion always possible
        print ("odo before : ",self.get_rear_odos())
        finish,actualCntl,actualCntr = self.odometricControlSimple(spd,cntl,cntr)
        print ("odo end of control ",self.get_rear_odos())
        self.brakes(cntl,cntr)
        print ("odo end of brake ",self.get_rear_odos())
        
        
    def odometricControlSimple(self,spd0,cntl,cntr):
        """
        stops when curcntl reaches cntl or when curcntr reaches cntr 
        """
        verbose = False
        verbose = True
        finish=False
        if verbose:
            print ("odo control simple : spd,cnt l and r",spd0,cntl,cntr)
        # slow down 0.5 -> 100% , 1.0 -> 30 % , cmd min = 35
        spdMin=80
        ratioSpd = float(spdMin)/float(spd0)
        aSlowDown = (1.0-ratioSpd)/(0.5-1.0)
        bSlowDown = 1.0-aSlowDown*0.5
        # should be improved with with speed regulation

        if cntl>0:
            self.spdl=spd0
            sgnl=1.0
        else:
            self.spdl=-spd0
            sgnl=-1.0
        if cntr>0:
            self.spdr=spd0
            sgnr=1.0
        else:
            self.spdr=-spd0
            sgnr=-1.0
        self.set_speed(self.spdl,self.spdr)
        
        if self.move: # no obstacle , move is possible
            while True:
                t0l = time.time()
                odorl,odorr = self.dataLogSpeedAndRearOdos() 
                dodol = self.deltaRearOdo(odorl,self.odorl0)
                dodor = self.deltaRearOdo(odorr,self.odorr0)
                self.curcntl += dodol
                self.curcntr += dodor               
                percl = float(self.curcntl)/float(cntl)         
                percr = float(self.curcntr)/float(cntr)

                if dodol == 0 and self.dodolLast == 0:
                    if verbose:
                        print ("motor left stalled",cntl,cntr,self.curcntl,self.curcntr)    
                    break
                if dodor == 0 and self.dodorLast == 0:
                    if verbose:
                        print ("motor right stalled",cntl,cntr,self.curcntl,self.curcntr)    
                    break
                self.dodorLast = dodor
                self.dodolLast = dodol
                
                if percl>1:
                    if verbose:
                        print ("odo control (end on left)",cntl,cntr,self.curcntl,self.curcntr)    
                    finish=True
                    break
                if percr>1:
                    if verbose:
                        print ("odo control (end on right)",cntl,cntr,self.curcntl,self.curcntr)
                    finish=True
                    break
                
                errdodo = abs(dodor)-abs(dodol)
                self.spdl = self.spdl + sgnl * self.kpOdor * errdodo
                self.spdr = self.spdr - sgnr * self.kpOdor * errdodo
                                
                slowDownl = aSlowDown * percl + bSlowDown
                if slowDownl > 1.0:
                    slowDownl = 1.0
                slowDownr = aSlowDown * percr + bSlowDown
                if slowDownr > 1.0:
                    slowDownr = 1.0
                if verbose:
                    print ("slowdown l,r",slowDownl,slowDownr)
                    print ("spd l,r",self.spdl*slowDownl,self.spdr*slowDownr)
                self.set_speed(self.spdl*slowDownl,self.spdr*slowDownr)

                self.odorl0 = odorl
                self.odorr0 = odorr
                # acquire and process sonar at slower rate than odos
                self.waitForLoopTime(self.dt,t0l)
        if verbose:
            print ("finish ok",finish)
        return finish,self.curcntl,self.curcntr
                
                

if __name__ == "__main__":
    print ("start")
    """
    if len(sys.argv) < 5:
        print ("needs to fix left and right speeds, dist to obstacle, loop count :")
        print ("python3 dart speedLeft speedRight dist2Obstacle cntMax")
        print ("exit!")
        exit()
    """
    myDart = dartv2.DartV2()
    drt = DartControl(myDart)

    print ("Battery Level : ",drt.get_battery())
    time.sleep(0.5)

    ts1 = 0.2
    spd1 = 120
    spd = 60
    cnt = 300
    kp = 0.8
    dist_obst_a, t_log_a = drt.goLineRearOdo (ts1,spd1,spd,cnt,kp,dfront=0.6)
    dist_obst_b, t_log_b = drt.goLineRearOdo (ts1,-spd1,-spd,cnt,kp,dfront=0.6)


    ts1 = 0.2
    spd1 = 120
    spd0 = 100
    spdmx = 160
    angle = 180.0
    kp = 0.01
    t_log_c = drt.turnAngleRearOdo (ts1,spd1,spd0,spdmx,angle,kp,"left")

    print ("Obst front and back :" ,dist_obst_a, dist_obst_b)

    print ("Battery Level :",drt.get_battery())
    time.sleep(0.5)

    print ("Card Sonars :",drt.get_cardinal_sonars())
    print ("Diag Sonars :",drt.get_diagonal_sonars())
    
    print ("Ticks for 90 deg turn :",drt.convertTurnAngleInTicks(90.0))
    drt.stop()
