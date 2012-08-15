"""Launch a planner directly by creating its interface and configuring the PlannerParameters structures.
"""
from openravepy import *
from numpy import pi
env = Environment() # create openrave environment
env.SetViewer('qtcoin')
env.Load('data/lab1.env.xml')
robot = env.GetRobots()[0]

robot.SetActiveDOFs(range(4)) # set joints the first 4 dofs
params = Planner.PlannerParameters()
params.SetRobotActiveJoints(robot)
params.SetGoalConfig([0,pi/2,pi/2,pi/2]) # set goal to all ones
# forces parabolic planning with 40 iterations
params.SetExtraParameters("""<_postprocessing planner="parabolicsmoother">
    <_nmaxiterations>40</_nmaxiterations>
</_postprocessing>""")

planner=RaveCreatePlanner(env,'birrt')
planner.InitPlan(robot, params)

traj = RaveCreateTrajectory(env,'')
planner.PlanPath(traj)

for i in range(traj.GetNumWaypoints()):
    # get the waypoint values, this holds velocites, time stamps, etc
    data=traj.GetWaypoint(i)
    # extract the robot joint values only
    dofvalues = traj.GetConfigurationSpecification().ExtractJointValues(data,robot,robot.GetActiveDOFIndices())
    raveLogInfo('waypint %d is %s'%(i,dofvalues))
