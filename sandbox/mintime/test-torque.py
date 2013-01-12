# -*- coding: utf-8 -*-
# Copyright (C) 2012 Quang-Cuong Pham <cuong.pham@normalesup.org>
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
Test file for MintimeProblemTorque
"""



from openravepy import *
from numpy import *
from pylab import *
import time
import MintimeTrajectory
import MintimeProblemTorque
import MintimeProfileIntegrator





################# Loading the environment ########################

set_printoptions(precision=5)
ion()



env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('robots/arm.robot.xml')

collisionChecker = RaveCreateCollisionChecker(env,'pqp')
env.SetCollisionChecker(collisionChecker)

robot=env.GetRobots()[0]

grav=[0,0,-9.8]

# DOF and velocity limits
n=robot.GetDOF()
dof_lim=robot.GetDOFLimits()
vel_lim=robot.GetDOFVelocityLimits()
robot.SetDOFLimits(-10*ones(n),10*ones(n))
robot.SetDOFVelocityLimits(100*vel_lim)


V=robot.GetEnv().GetViewer()
V.SetCamera(array([[ 0.56202,  0.12676,  0.81736, -1.81434],
       [-0.82712,  0.08233,  0.55596, -1.2069 ],
       [ 0.00318, -0.98851,  0.15111, -0.05104],
       [ 0.     ,  0.     ,  0.     ,  1.     ]]))


################## Define a test trajectory ##############


q0=[0,0,0,0]
q1=[ 2.32883,  1.61082,  0.97706,  1.94169]
v=1e-2
qd0=[v,v,v,v]
qd1=[v,v,v,v]

q_list=[q0,q1]
qd_list=[qd0,qd1]
T_list=[1.5]
pwp_traj=MintimeTrajectory.Interpolate(q_list,qd_list,T_list)

T=sum(T_list)
n_discr=200.
t_step=T/n_discr
a=time.time()
traj=pwp_traj.GetSampleTraj(T,t_step)

m0=6
m1=15
m2=5
m3=4
tau_min=array([-m0,-m1,-m2,-m3])
tau_max=array([m0,m1,m2,m3])
qd_max=array([3,3,3,3])





################### Run the algorithm ########################

deb=time.time()


print 'Running the time parameterization algorithm...'

# Set up a minimum time problem under torque limits

pb=MintimeProblemTorque.MintimeProblemTorque(robot,traj)
pb.set_dynamics_limits([tau_min,tau_max])
pb.set_velocity_limits(qd_max)
pb.disc_thr=10 # Threshold in the discontinuity point search
pb.preprocess()
#pb.plot_maxvel_curves()


# Integrate the limiting curves and the final profile

algo=MintimeProfileIntegrator.MintimeProfileIntegrator(pb)
algo.dt_integ=t_step/10 # time step to integrate the limiting curves
algo.width=5 # window to test if we can get through a switching point
algo.palier=10 # length of the palier around zero inertia points
algo.tolerance_ends=1e-2 # threshold at the ends
algo.sdot_init=1e-4 # initial value of sdot
algo.sdot_final=1e-4 # final value of sdot

algo.integrate_all_profiles()
algo.integrate_final()




# Reparameterize the path using the new velocity profile

s_res=algo.s_res
sdot_res=algo.sdot_res
undersample_coef=int(round(t_step/algo.dt_integ))
s_res_u=s_res[range(1,len(s_res),undersample_coef)]
sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]
traj2=pwp_traj.ResampleTraj(s_res_u,sdot_res_u,t_step)





###################### Plotting ############################


print '\n*****************************************************************\n'
print 'Total execution time: '+str(time.time()-deb)+'s'
print '\n*****************************************************************\n'


raw_input('Press Enter to execute the trajectory /before/ time-reparameterization (duration='+str(traj.duration)+'s)')
MintimeProblemTorque.Execute(robot,traj,0.02)
raw_input('Press Enter to execute the trajectory /after/ time-reparameterization (duration='+str(traj2.duration)+'s)')
MintimeProblemTorque.Execute(robot,traj2,0.02)


# Inverse dynamics of the retimed trajectory and plot
tau=MintimeProblemTorque.ComputeTorques(robot,traj,grav)
tau2=MintimeProblemTorque.ComputeTorques(robot,traj2,grav)


mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(1)
MintimeProblemTorque.PlotVelocities(traj.t_vect,traj.qd_vect,qd_max)
title('Velocity profiles before reparameterization')
figure(2)
MintimeProblemTorque.PlotTorques(traj.t_vect,tau,tau_min,tau_max)
title('Torque profiles before reparameterization')
figure(3)
algo.plot_profiles()
axis([0,traj.duration,0,10])
title('Phase space')
figure(4)
MintimeProblemTorque.PlotVelocities(traj2.t_vect,traj2.qd_vect,qd_max)
title('Velocity profiles after reparameterization')
figure(5)
MintimeProblemTorque.PlotTorques(traj2.t_vect,tau2,tau_min,tau_max)
title('Torque profiles after reparameterization')


