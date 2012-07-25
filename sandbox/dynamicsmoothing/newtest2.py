from openravepy import *
from numpy import *
import time
import Dynamics
import Trajectory
import MinimumTime
import Shortcutting
set_printoptions(precision=5)



################# Loading the environment ########################

env = Environment() # create openrave environment
#env.SetViewer('qtcoin') # attach viewer (optional)
#env.Load('robots/wam4.robot.xml')
env.Load('robots/arm.robot.xml')
#env.Load('robots/twodof.robot.xml')


robot=env.GetRobots()[0]
env.GetPhysicsEngine().SetGravity([0,0,0])
params=Dynamics.GetParams(robot)

#robot.SetDOFValues([5,-2,1,0])
robot.SetDOFValues([5,-2])

robot.SetDOFVelocities(zeros(robot.GetDOF()))

Mc = []
Mp = []
for i in range(robot.GetDOF()):
    print '********************************'
    print i
    accelerations = zeros(robot.GetDOF())
    accelerations[i] = 1
    # use c++
    torques = robot.ComputeInverseDynamics(accelerations)
    Mc.append(torques)
    # use python
    q=robot.GetDOFValues()
    self=Dynamics.RobotDynamics(params,q,grav=env.GetPhysicsEngine().GetGravity())
    velocities = robot.GetDOFVelocities()
    torques2 = self.rne(velocities,accelerations,env.GetPhysicsEngine().GetGravity())
    Mp.append(torques2)



print 'cpp torques',array(Mc)

print 'python torques: ',array(Mp)




starttime=time.time()
for i in range(1000):
    robot.SetDOFValues([0.5,-0.2,0.1,0.2])
    robot.SetDOFVelocities([0.1,0.2,0.3,0.4])
    torques = robot.ComputeInverseDynamics([1,2,3,6])


print 'average time for inv dynamics computation', (time.time()-starttime)/1000


