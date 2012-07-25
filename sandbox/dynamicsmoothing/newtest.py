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
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('/home/cuong/Dropbox/Code/mintime/robots/twodof.robot.xml')
#env.Load('/home/cuong/Dropbox/Code/mintime/robots/arm.robot.xml')
#env.Load('robots/wam4.robot.xml')
robot=env.GetRobots()[0]
env.GetPhysicsEngine().SetGravity([0,0,0])
params=Dynamics.GetParams(robot)



robot.SetDOFValues(zeros(robot.GetDOF()))
robot.SetDOFVelocities(zeros(robot.GetDOF()))

robot.SetDOFValues(zeros(robot.GetDOF()))
m=robot.GetLinks()[1].GetMass() #mass of the links
l=robot.GetLinks()[1].GetGlobalCOM()[0]*2 #length of the links
coef=1/2.*m*l*l

theta1=0
theta2=1.1
ctheta2=cos(theta2)
robot.SetDOFValues([theta1,theta2])
M_ref=coef*array([[2*(5/3.+ctheta2),2/3.+ctheta2],[2/3.+ctheta2,2/3.]])

accelerations1=[1,0]
accelerations2=[0,1]

# use c++
torques1c = robot.ComputeInverseDynamics(accelerations1)
torques2c = robot.ComputeInverseDynamics(accelerations2)

# use python
q=robot.GetDOFValues()
self=Dynamics.RobotDynamics(params,q,grav=env.GetPhysicsEngine().GetGravity())
velocities = robot.GetDOFVelocities()
torques1p = self.rne(velocities,accelerations1,env.GetPhysicsEngine().GetGravity())
torques2p = self.rne(velocities,accelerations2,env.GetPhysicsEngine().GetGravity())


print 'manipulator inertia matrix'
print M_ref
print 'cpp torques'
print transpose(array([torques1c,torques2c]))
print 'python torques'
print transpose(array([torques1p,torques2p]))


