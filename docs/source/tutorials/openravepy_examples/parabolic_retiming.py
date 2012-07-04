"""Do parabolic segment retiming to a goal position without checking collisions.
"""
from openravepy import *
import numpy
env = Environment() # create openrave environment
env.SetViewer('qtcoin')
env.Load('robots/barrettwam.robot.xml') # load a simple scene
robot=env.GetRobots()[0]
with env:  
    lower,upper = robot.GetActiveDOFLimits()
    goalvalues = numpy.random.rand(len(lower))*(upper-lower)+lower
    traj = RaveCreateTrajectory(env,'')
    traj.Init(robot.GetActiveConfigurationSpecification())
    traj.Insert(0,robot.GetActiveDOFValues())
    traj.Insert(1,goalvalues)
    planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1,plannername='ParabolicTrajectoryRetimer')
    print 'duration',traj.GetDuration()

robot.GetController().SetPath(traj)
robot.WaitForController(0)
