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


"""
Test file for RRT_Smooth
"""


from openravepy import *
from numpy import *
from pylab import *
import time
import MintimeProblemTorque
import RRT_Smooth




################# Loading the environment ########################

set_printoptions(precision=5)
ion()

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('robots/arm.robot.xml')
env.Load('robots/table.kinbody.xml')
env.Load('robots/ikeashelf.kinbody.xml')

collisionChecker = RaveCreateCollisionChecker(env,'pqp')
env.SetCollisionChecker(collisionChecker)

robot=env.GetRobots()[0]
table=env.GetKinBody('table').GetLinks()[0]
shelf=env.GetKinBody('ikeashelf').GetLinks()[0]
table.SetTransform(array([[0,0,0,-1],[0,0,0,0.4],[0,0,0,0.4],[0,0,0,1]]))
shelf.SetTransform(array([[  2.21862e-02,   9.99754e-01,  -2.77376e-08,   2.68396e-01],
       [ -9.99754e-01,   2.21862e-02,   1.00355e-08,   6.68360e-01],
       [  1.06484e-08,   2.75081e-08,   1.00000e+00,  -3.89345e-01],
       [  0.00000e+00,   0.00000e+00,   0.00000e+00,   1.00000e+00]]))


grav=[0,0,-9.8]

# DOF and velocity limits
n=robot.GetDOF()
dof_lim=robot.GetDOFLimits()
vel_lim=robot.GetDOFVelocityLimits()
robot.SetDOFLimits(-10*ones(n),10*ones(n))
robot.SetDOFVelocityLimits(100*vel_lim)



V=robot.GetEnv().GetViewer()
V.SetCamera(array([[ 0.9946 , -0.01639,  0.10243, -0.55002],
       [-0.1024 , -0.31308,  0.94419, -2.02975],
       [ 0.01659, -0.94958, -0.31307,  1.16889],
       [ 0.     ,  0.     ,  0.     ,  1.     ]]))



##################### RRT to find a collision-free path ########################

robot.SetDOFValues(zeros(4))
goal_config=[ 2.32883,  1.61082,  0.97706,  1.94169]

deb=time.time()

print 'Searching a collision-free path using bi-rrt...'
rave_traj1=RRT_Smooth.RRT(robot,goal_config)





################# Time parameterize the RRT trajectory ###############


tunings=RRT_Smooth.Tunings()

# Torque limits
m0=10 
m1=15
m2=6
m3=6

tunings.grav=grav
tunings.tau_min=[-m0,-m1,-m2,-m3] # Torque limits
tunings.tau_max=[m0,m1,m2,m3]
tunings.qd_max=4*vel_lim # Velocity limits

# Tunings for the time parameterization algorithm
tunings.t_step=0.005 # time step to sample the dynamics
tunings.dt_integ=tunings.t_step/10 # time step to integrate the limiting curves
tunings.width=20 # window to test if we can get through a switching point
tunings.palier=20 #length of the palier around zero inertia points
tunings.tolerance_ends=1e-2 # 


print 'Reparameterizing the RRT path using optimal-time algorithm...'
[rave_traj2,traj2]=RRT_Smooth.Retime(robot,rave_traj1,tunings)
# MintimeProblemTorque.Execute(robot,traj2,0.01)





################# Run the shortcutting algorithm ########################


# Tunings for the smoothing repetition
tunings.n_runs=5 # Number of runs of IterateSmooth
tunings.max_time=15 # Time limit (in seconds) for one run of IterateSmooth

# Tunings for IterateSmooth
tunings.mean_len=traj2.duration/4 # Mean length of each shortcut
tunings.std_len=traj2.duration/2 # Std of the length of the shortcut
tunings.coef_tolerance=1.2

# Tuning parameters for one Shortcut
tunings.dt_sample=0.005 # time step to sample the shortcut
tunings.dt_integ=0.001 # time step to integrate the limiting curves
tunings.threshold_waive=1e-2

print 'Running the shortcutting algorithm...'
time.sleep(1)
[rave_traj3,traj3]=RRT_Smooth.RepeatIterateSmooth(robot,traj2,tunings)


print '\n*****************************************************************\n'
print 'Total execution time (including RRT): '+str(int(time.time()-deb))+'s'
print '\n*****************************************************************\n'



#################### Executing and plotting ################################


# Run the trajectories
raw_input('Press Enter to execute the trajectory /before/ shortcutting (duration='+str(traj2.duration)+'s)')
MintimeProblemTorque.Execute(robot,traj2,0.02)
raw_input('Press Enter to execute the trajectory /after/ shortcutting (duration='+str(traj3.duration)+'s)')
MintimeProblemTorque.Execute(robot,traj3,0.02)


# Can also do that
#robot.GetController().SetPath(rave_traj1)
#robot.WaitForController(0)
#robot.GetController().SetPath(rave_traj2)
#robot.WaitForController(0)
#robot.GetController().SetPath(rave_traj3)
#robot.WaitForController(0)




# Inverse dynamics and plot

tau_min=tunings.tau_min
tau_max=tunings.tau_max
qd_max=tunings.qd_max


tau2=MintimeProblemTorque.ComputeTorques(robot,traj2,grav)
mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(1)
MintimeProblemTorque.PlotVelocities(traj2.t_vect,traj2.qd_vect,qd_max)
title('Velocity profile before shortcutting')
figure(2)
MintimeProblemTorque.PlotTorques(traj2.t_vect,tau2,tau_min,tau_max)
title('Torques profile before shortcutting')

tau3=MintimeProblemTorque.ComputeTorques(robot,traj3,grav)
mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(3)
MintimeProblemTorque.PlotVelocities(traj3.t_vect,traj3.qd_vect,qd_max)
title('Velocity profile after shortcutting')
figure(4)
MintimeProblemTorque.PlotTorques(traj3.t_vect,tau3,tau_min,tau_max)
title('Torques profile after shortcutting')



