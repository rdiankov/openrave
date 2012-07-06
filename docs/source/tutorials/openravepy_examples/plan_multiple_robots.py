"""Set multiple robots in one configuration to allow for simultaneously planning
"""
from openravepy import *
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
with env:
    # init scene
    robot1 = env.ReadRobotURI('robots/barrettwam.robot.xml')
    env.Add(robot1,True)
    robot2 = env.ReadRobotURI('robots/barrettwam.robot.xml')
    env.Add(robot2,True)
    Trobot = robot2.GetTransform()
    Trobot[0,3] += 0.5
    robot2.SetTransform(Trobot)
    RaveSetDebugLevel(DebugLevel.Debug) # set output level to debug
    
    # create planner parmaeters
    params = Planner.PlannerParameters()
    params.SetConfigurationSpecification(env, robot1.GetActiveManipulator().GetArmConfigurationSpecification() + robot2.GetActiveManipulator().GetArmConfigurationSpecification())
    params.SetGoalConfig([  2.16339636e+00,  -3.67548731e-01,  -1.84983003e+00, 1.76388705e+00,  -1.27624984e-07,   7.65325147e-09, 0.00000000e+00,  -7.27862052e-01,  -6.52626197e-01, -8.10210670e-09,   1.34978628e+00,  -1.21644879e-08, 2.77047240e-08,   0.00000000e+00])
    
    # start planner
    traj = RaveCreateTrajectory(env,'')
    planner = RaveCreatePlanner(env,'birrt')
    planner.InitPlan(None,params)
    status = planner.PlanPath(traj)

# set new traj to robot controllers
robot1.GetController().SetPath(traj)
robot2.GetController().SetPath(traj)
robot1.WaitForController(0) # wait
robot2.WaitForController(0) # wait
