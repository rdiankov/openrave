"""Loads the grasping model and moves the robot to the first grasp found
"""
from openravepy import *
import numpy, time
env=Environment()
env.Load('data/lab1.env.xml')
env.SetViewer('qtcoin')
robot = env.GetRobots()[0]
target = env.GetKinBody('mug1')
gmodel = databases.grasping.GraspingModel(robot,target)
if not gmodel.load():
    gmodel.autogenerate()

validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=1)
gmodel.moveToPreshape(validgrasps[0])
Tgoal = gmodel.getGlobalGraspTransform(validgrasps[0],collisionfree=True)
basemanip = interfaces.BaseManipulation(robot)
basemanip.MoveToHandPosition(matrices=[Tgoal])
robot.WaitForController(0)
taskmanip = interfaces.TaskManipulation(robot)
taskmanip.CloseFingers()
robot.WaitForController(0)
