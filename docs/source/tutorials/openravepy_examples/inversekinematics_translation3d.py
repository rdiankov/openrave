"""Moves a robot in a random position, gets the end effector transform, and calls IK on it.
"""
from openravepy import *
import time
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
env.Load('data/katanatable.env.xml') # load a scene
robot = env.GetRobots()[0] # get the first robot

manip = robot.GetActiveManipulator()
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
if not ikmodel.load():
    ikmodel.autogenerate()

with robot: # lock environment and save robot state
    robot.SetDOFValues([2.58, 0.547, 1.5, -0.7],[0,1,2,3]) # set the first 4 dof values
    Tee = manip.GetEndEffectorTransform() # get end effector
    ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
    sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions

h = env.plot3(Tee[0:3,3],10) # plot one point
with robot: # save robot state
    raveLogInfo('%d solutions'%len(sols))
    for sol in sols: # go through every solution
        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
        env.UpdatePublishedBodies() # allow viewer to update new robot
        time.sleep(10.0/len(sols))

raveLogInfo('restored dof values: '+repr(robot.GetDOFValues())) # robot state is restored to original
