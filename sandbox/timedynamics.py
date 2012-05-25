#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Simple timing for dynamics computations
"""
from openravepy import *
from numpy import *
import time

env=Environment()
env.Load('robots/wam7.kinbody.xml')
body = env.GetBodies()[0]

N = 10000
for itry in range(10):
    lower,upper = body.GetDOFLimits()
    vellimits = body.GetDOFVelocityLimits()
    dofvaluesnew = lower+random.rand(len(lower))*(upper-lower)
    dofvelnew = -vellimits+2*random.rand(len(vellimits))*vellimits
    link0vel = [random.rand(3)-0.5,random.rand(3)-0.5]
    dofaccel = 10*random.rand(body.GetDOF())-5
    starttime=time.time()
    for itry in range(N):
        body.SetDOFValues(dofvaluesnew)
        body.SetDOFVelocities(dofvelnew)
        torques = body.ComputeInverseDynamics(dofaccel)
    print (time.time()-starttime)/N
    
RaveDestroy()
