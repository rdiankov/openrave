# -*- coding: utf-8 -*-
# Copyright (C) 2011-2012 Quang-Cuong Pham <cuong.pham@normalesup.org>
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 



from openravepy import *
from numpy import *
from pylab import *
import scipy
import time
import thread
import PinAndDrag
import HRP4
import Trajectory
import FollowTrajectory
set_printoptions(precision=5)



################# Loading the environment ########################

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
#env.Load('/home/cuong/Dropbox/Code/mintime/robots/arm.robot.xml')
env.Load('/home/cuong/Dropbox/Code/mintime/robots/hrp4r.dae')
env.Load('/home/cuong/Dropbox/Code/mintime/robots/floorwalls.xml')

exclude_list=[1,2,17,5,6,8,9,12,13,15,17,19,20,23,24,26,27,28,29,30,31,32,33,34,35,36,37,40,41,43,44,45,46,47,48,49]


xminf=-0.085
xmaxf=0.16
yminf=-0.16
ymaxf=0.16
zz=0.001
p1=array([xminf,yminf,zz])
p2=array([xmaxf,yminf,zz])
p3=array([xmaxf,ymaxf,zz])
p4=array([xminf,ymaxf,zz])
handle_base=env.drawlinestrip(array([p1,p2,p3,p4,p1]),5)

collisionChecker = RaveCreateCollisionChecker(env,'pqp')
env.SetCollisionChecker(collisionChecker)

robot=env.GetRobots()[0]
[lower_lim,upper_lim]=robot.GetDOFLimits()
hrp=HRP4.HRP4robot(robot)
n=robot.GetDOF()
robot.SetDOFLimits(-1e10*ones(n),1e10*ones(n))

Q0=eye(n+6)
Q0[0,0]=3
Q0[1,11]=3
Q0[2,2]=3
Q0[3,3]=1
Q0[4,4]=1
Q0[5,5]=1
params={'x':1}
params['robot']=robot
params['exclude_list']=exclude_list
params['p_step']=0.01
params['timestep']=0.001
params['Q0']=Q0
params['Q0inv']=linalg.inv(Q0)
params['lower_lim']=lower_lim
params['upper_lim']=upper_lim
params['K_p']=0.1
params['K_li']=0.01
params['K_sr']=0.00001
params['priority']='pin'
params['activate_base_translations']=False
params['activate_base_rotations']=False
params['draw_com']=True


config=array([  3.33897e-01,   3.60542e-01,   8.00304e-01,   8.35608e-08,
         3.31824e-08,  -3.13480e+00,   2.64733e-17,  -1.64237e-16,
        -2.44140e-16,  -1.38744e-16,  -2.46371e-16,  -2.65448e-02,
        -1.11867e-16,   3.52050e-18,  -1.60872e-16,  -1.07424e-16,
        -6.02101e-17,  -1.16830e-16,  -1.72097e-16,  -1.01949e-16,
        -4.88103e-17,  -1.75168e-16,  -1.45074e-17,   7.35638e-18,
         6.67107e-21,  -2.03958e-22,  -9.36739e-22,  -9.41175e-24,
         2.72573e-27,  -4.44707e-27,   9.01336e-27,   1.03314e-27,
         6.81670e-27,  -1.28606e-27,   3.77427e-01,  -5.97724e-01,
         1.05678e-02,  -2.67625e-03,   6.18971e-01,   1.73002e-01,
        -1.24488e+00,  -1.41274e+00,   1.39935e-26,  -5.13813e-18,
        -3.40759e-18,   1.36842e-16,   0.00000e+00,  -6.28591e-01,
         8.13047e-01,  -4.56412e-01,  -1.86819e+00,   0.00000e+00,
         0.00000e+00,   0.00000e+00,  -1.61425e-16,   0.00000e+00])


PinAndDrag.stop()
HRP4.SetConfig(robot,config)
PinAndDrag.start(params)








PinAndDrag.thread_data['drag']=[7,'translation']






robot.GetLinks()[0].GetTransform()
robot.GetDOFValues()

PinAndDrag.thread_params=params
baselink=robot.GetLinks()[0]


# Initial
hrp.halfsit()
config=HRP4.GetConfig(robot)
p_init=robot.GetLinks()[18].GetGlobalCOM()
center=baselink.GetTransform()[0:3,3]
u=p_init-center
R0=HRP4.rpy2mat(config[3:6])


#Prediction
J=PinAndDrag.Jacobian(config[3:6],18,p_init)
delta=zeros(56)
delta[3:6]=1e-3*array([1,-1,2])
p_predicted=p_init+dot(J,delta)
config2=config+delta
R1=HRP4.rpy2mat(config2[3:6])
shift=dot(R1,u)
p_predicted2=center+shift


#Final
HRP4.SetConfig(robot,config2)
p_real=robot.GetLinks()[18].GetGlobalCOM()

print p_predicted
print p_predicted2
print p_real
print linalg.norm(p_predicted-p_real)/linalg.norm(p_init-p_real)*100





