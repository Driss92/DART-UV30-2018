# -*- coding: utf-8 -*-
import sys
import time
import os
import math
import signal
import numpy as np
import dartv2_control_v1
import random
import pickle
import select

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def getKey():
    #tty.setcbreak(sys.stdin.fileno())
    c='s'
    cok=False
    if isData():
        c = sys.stdin.read(1)
        cok=True
    return cok,c


t0circuit = time.time()
drt = dartv2_control_v1.DartControl()

if drt.drt.dartSim:
    p0 = np.asarray( drt.drt.vLocation)
    print (p0)

dart_name = sys.argv[1]
log_id = sys.argv[2]
debug = True
if int(sys.argv[3]) == 0:
    debug = False

dobst = 0.3
 
# parameters for line
t1=0.05
spd1=120 # 160
spd=70 # 120
kp=0.8

# parameters for turn
tr1 = t1
spdr1 = 140
spdr0 = 100
spdrmx = 160
kpr = 0.01

drt.setDistToObstacle(dobst,t1,spd1,spd,kp)

drt.stop()
