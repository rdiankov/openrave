"""Shows how to get all 6D IK solutions.
"""
from openravepy import *
import numpy, time
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
env.Load('data/pr2test1.env.xml') # load a scene
robot = env.GetRobots()[0] # get the first robot

manip = robot.SetActiveManipulator('leftarm_torso') # set the manipulator to leftarm
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()

with env: # lock environment
    Tgoal = numpy.array([[0,-1,0,-0.21],[-1,0,0,0.04],[0,0,-1,0.92],[0,0,0,1]])
    sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    with robot: # save robot state
        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
        Tee = manip.GetEndEffectorTransform()
        env.UpdatePublishedBodies() # allow viewer to update new robot
        time.sleep(10)
    
    raveLogInfo('Tee is: '+repr(Tee))
