from openravepy import *
from numpy import *
from pylab import *
import scipy
import time
import Image
import ZMP
import Trajectory
import MinimumTimeZMP
import HRP4

set_printoptions(precision=5)



################# Loading the environment ########################

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('/home/cuong/Dropbox/Code/mintime/robots/hrp4r.dae')
env.Load('/home/cuong/Dropbox/Code/mintime/robots/floorwalls.xml')

collisionChecker = RaveCreateCollisionChecker(env,'pqp')
env.SetCollisionChecker(collisionChecker)

robot=env.GetRobots()[0]

for i in range(1800):                                                
    Trajectory.Set(robot,config_vect[:,i])
    time.sleep(0.01)



