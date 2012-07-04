#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Simple timing for dynamics computations
"""
from openravepy import *
from numpy import *
import time

env=Environment()
env.Load('robots/barrettwam.robot.xml')#wam7.kinbody.xml')
body = env.GetBodies()[0]
N = 10000

def timedynamics():
    for itry in range(10):
        env.GetPhysicsEngine().SetGravity([0,0,-10])
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
        print 'ComputeInverseDynamics all', (time.time()-starttime)/N
        starttime=time.time()
        for itry in range(N):
            body.SetDOFValues(dofvaluesnew)
            body.SetDOFVelocities(dofvelnew)
            tm,tc,te = body.ComputeInverseDynamics(dofaccel,None,returncomponents=True)
        print 'ComputeInverseDynamics components', (time.time()-starttime)/N
        starttime=time.time()
        for itry in range(N):
            body.SetDOFValues(dofvaluesnew)
            body.SetDOFVelocities(dofvelnew)
            tm,tc,te = body.ComputeInverseDynamics(dofaccel,None,returncomponents=True)
            tother = body.ComputeInverseDynamics(dofvelnew)-tc-te
            # slower
            #env.GetPhysicsEngine().SetGravity([0,0,-10])
            #body.SetDOFVelocities(zeros(body.GetDOF()))
            #tother = body.ComputeInverseDynamics(dofvelnew)

        print 'ComputeInverseDynamics special', (time.time()-starttime)/N

def timehessian():
    for itry in range(10):
        lower,upper = body.GetDOFLimits()
        dofvaluesnew = lower+random.rand(len(lower))*(upper-lower)
        starttime=time.time()
        for itry in range(N):
            body.SetDOFValues(dofvaluesnew)
            body.ComputeHessianTranslation(len(body.GetLinks())-1, random.rand(3)-0.5)
        print 'ComputeHessianTranslation/SetDOFValues all', (time.time()-starttime)/N
        starttime=time.time()
        for itry in range(N):
            body.ComputeHessianTranslation(len(body.GetLinks())-1, random.rand(3)-0.5)
        print 'ComputeHessianTranslation', (time.time()-starttime)/N

#timehessian()
timedynamics()
RaveDestroy()
