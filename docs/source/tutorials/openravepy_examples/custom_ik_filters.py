"""Set a custom IK filter to abort computation after 100ms.
"""
from openravepy import *
import numpy, time
env=Environment()
env.Load('data/pr2test1.env.xml')

robot=env.GetRobots()[0]
manip = robot.SetActiveManipulator('leftarm_torso')
lower,upper = robot.GetDOFLimits(manip.GetArmIndices()) # get the limits of just the arm indices
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()

maxtime = 0.1 # 100ms
while True:
    with env:
        robot.SetDOFValues(lower+numpy.random.rand(len(lower))*(upper-lower),manip.GetArmIndices()) # set a random values to just the arm
        incollision = not env.CheckCollision(robot) and not robot.CheckSelfCollision()
        starttime = time.time()
        def timeoutfilter(values, manip, ikparam):
            return IkFilterReturn.Quit if time.time()-starttime > maxtime else IkFilterReturn.Success
        
        handle=manip.GetIkSolver().RegisterCustomFilter(0,timeoutfilter)
        success = manip.FindIKSolution(manip.GetIkParameterization(IkParameterization.Type.Transform6D),IkFilterOptions.CheckEnvCollisions)
        raveLogInfo('in collision: %d, real success: %d, time passed: %f'%(incollision,success is not None,time.time()-starttime))
