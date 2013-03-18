"""Shows how to solve IK for two robot arms  simultaneously
"""
from openravepy import *
from numpy import *
from itertools import izip
import time
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
env.Load('data/dualarmmanipulation.env.xml')

# get the first robot
robot=env.GetRobots()[0]

# create the dual-arm ik solver
dualsolver = misc.MultiManipIKSolver([robot.GetManipulator('leftarm'), robot.GetManipulator('rightarm')])

body=env.GetKinBody('Object1')
for itry in range(5):
    with env:
        Tbody=body.GetTransform()
        ab = body.ComputeAABB().extents()
        halfwidth= ab[1] #this is y
        #.04 is just half the thickness of the EEF
        TRightGrasp= dot(Tbody,array([[0, 0, -1, 0],[1, 0, 0, (halfwidth+.04)],[0, -1, 0, 0 ],[0, 0, 0, 1]]))
        # to determine the grasp for the eef given the transform of the object
        TLeftGrasp= dot(Tbody,array([[0, 0, -1, 0],[-1, 0, 0, -(halfwidth+.04)],[0, 1, 0, 0],[0, 0, 0, 1]]))
        solutions = dualsolver.findMultiIKSolution(Tgrasps=[TLeftGrasp,TRightGrasp],filteroptions=IkFilterOptions.CheckEnvCollisions)
        if solutions is not None:
            for manip,solution in izip(dualsolver.manips, solutions):
                robot.SetDOFValues(solution,manip.GetArmIndices())
            
    time.sleep(0.2)
