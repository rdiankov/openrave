"""Get a trajectory to a grasp before executing it.
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
basemanip = interfaces.BaseManipulation(robot)
with robot:
    grasp = validgrasps[0]
    gmodel.setPreshape(grasp)
    T = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
    traj = basemanip.MoveToHandPosition(matrices=[T],execute=False,outputtrajobj=True)

raveLogInfo('traj has %d waypoints, last waypoint is: %s'%(traj.GetNumWaypoints(),repr(traj.GetWaypoint(-1))))
robot.GetController().SetPath(traj)
robot.WaitForController(0)
