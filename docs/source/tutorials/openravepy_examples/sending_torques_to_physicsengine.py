"""Shows how to set a physics engine and send torque commands to the robot
"""
from openravepy import *
import numpy, time
env=Environment()
env.Load('data/lab1.env.xml')
env.SetViewer('qtcoin')
with env:
    # set a physics engine
    physics = RaveCreatePhysicsEngine(env,'ode')
    env.SetPhysicsEngine(physics)
    physics.SetGravity(numpy.array((0,0,-9.8)))
    
    robot = env.GetRobots()[0]
    robot.GetLinks()[0].SetStatic(True)
    env.StopSimulation()
    env.StartSimulation(timestep=0.001)

for itry in range(5):
    torques = 100*(numpy.random.rand(robot.GetDOF())-0.5)
    for i in range(100):
        # have to lock environment when calling robot methods
        with env:
            robot.SetDOFTorques(torques,True)
        time.sleep(0.01)

# reset the physics engine
env.SetPhysicsEngine(None)

