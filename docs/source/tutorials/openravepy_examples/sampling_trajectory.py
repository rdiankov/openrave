"""Show how to sample the trajectory objects and move the robot
"""
from openravepy import *
from numpy import arange
import time
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
env.Load('data/lab1.env.xml') # load a scene
robot = env.GetRobots()[0] # get the first robot
manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
traj=manipprob.MoveManipulator(goal=[-0.75,1.24,-0.064,2.33,-1.16,-1.548,1.19],outputtrajobj=True,execute=False) # call motion planner with goal joint angles
spec=traj.GetConfigurationSpecification() # get the configuration specification of the trajrectory
for i in range(5):
    starttime = time.time()
    while time.time()-starttime < traj.GetDuration():
        curtime = time.time()-starttime
        with env: # have to lock environment since accessing robot
            trajdata=traj.Sample(curtime)
            values=spec.ExtractJointValues(trajdata,robot,range(robot.GetDOF()),0)
            robot.SetDOFValues(values)
        time.sleep(0.01)
