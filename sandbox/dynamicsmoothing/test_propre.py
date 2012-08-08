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
import Image
import Trajectory
import MinimumTime
import Shortcutting

set_printoptions(precision=5)
interactive(True)

################# Loading the environment ########################

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)

xmldata = """<Robot name="Arm">
  <KinBody file="robots/wam4.kinbody.xml">
  </KinBody>
</Robot>"""
robot=env.ReadRobotData(xmldata)
env.Add(robot)

env.Load('robots/table.kinbody.xml')

collisionChecker = RaveCreateCollisionChecker(env,'pqp')
collisionChecker.SetCollisionOptions(0) 
env.SetCollisionChecker(collisionChecker)



robot=env.GetRobots()[0]
table=env.GetKinBody('table').GetLinks()[0]
table.SetTransform(array([[0,0,0,-1],[0,0,0,0.4],[0,0,0,0.4],[0,0,0,1]]))

grav=[0,0,-9.8]

# DOF and velocity limits
n=robot.GetDOF()
dof_lim=robot.GetDOFLimits()
vel_lim=robot.GetDOFVelocityLimits()
robot.SetDOFLimits(-10*ones(n),10*ones(n))
robot.SetDOFVelocityLimits(100*vel_lim)


################# Initialize a rave trajectory ########################

q0=[0,0,0,0]
q1=[ 0.8345 , 0,  0,  0]
q2=[ 0.8345 , -0.13497,  0.97706,  1.94169]
q3=[ 0.8345 ,  1.61082,  0.97706,  1.94169]
q4=[ 2.32883,  1.61082,  0.97706,  1.94169]
dt_vect=array([0,1,1,2,1])
via_points=array([q0,q1,q2,q3,q4])
rave_traj=RaveCreateTrajectory(env,'')
spec = robot.GetActiveConfigurationSpecification('linear')
spec.AddDeltaTimeGroup()
rave_traj.Init(spec)
rave_traj.Insert(0,c_[via_points,dt_vect].flatten())

# Execute the trajectory
#robot.GetController().SetPath(rave_traj)


############################ Settings #################################

# Torques limits
m0=10
m1=15
m2=6
m3=6
m=max([m0,m1,m2,m3])
tau_min=[-m0,-m1,-m2,-m3]
tau_max=[m0,m1,m2,m3]
qd_max=vel_lim
#qd_max=None

# Time step for the discretization
t_step=0.005

# Some tuning parameters
tunings={'':0}
tunings['slope_threshold']=10 # threshold in the tangent points search

tunings['t_step_integrate']=t_step/20 # time step to integrate the limiting curves
tunings['tolerance']=0.01 #tolerance above the max_curve
tunings['width']=20 # window to test if we can get through a switching point
tunings['palier']=10 #length of the palier around zero inertia points
tunings['threshold_final']=1e-2
tunings['threshold_waive']=1e-2


############# Compute a feasible timing for each segment #################

traj_list=[]
pb_list=[]
for i_way in range(1,rave_traj.GetNumWaypoints()):
    a_prev=rave_traj.GetWaypoints(i_way-1,i_way)
    a=rave_traj.GetWaypoints(i_way,i_way+1)
    T=a[-1]
    pieces_list=[]
    for i_dof in range(len(a)-1):
        pieces_list.append(poly1d([(a[i_dof]-a_prev[i_dof])/T,a_prev[i_dof]]))
    pwp_traj_linear=Trajectory.PieceWisePolyTrajectory([pieces_list],[T])
    traj=pwp_traj_linear.GetSampleTraj(T,t_step)
    pb=MinimumTime.RobotMinimumTime(robot,traj,tunings,grav,tau_min,tau_max,qd_max,1e-2,1e-2)
    pb_list.append(pb)
    s_res=pb.s_res
    sdot_res=pb.sdot_res
    undersample_coef=int(round(t_step/tunings['t_step_integrate']))
    s_res_u=s_res[range(1,len(s_res),undersample_coef)]
    sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]
    traj2=pwp_traj_linear.ResampleTraj(s_res_u,sdot_res_u,t_step)
    traj_list.append(traj2)

traj2=Trajectory.Concat(traj_list)

# Inverse dynamics of the retimed trajectory and plot
tau2=Trajectory.ComputeTorques(robot,traj2,grav)
mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(7)
Trajectory.PlotTorques(traj2.t_vect,tau2,tau_min,tau_max)
figure(8)
clf()
plot(transpose(traj2.qd_vect))



################# Run the shortcutting algorithm ########################

# Tuning parameters for the shortcutting algorithm
tunings['t_step_sample']=0.001 # time step to sample the shortcut
tunings['t_step_integrate']=0.0005 # time step to integrate the limiting curves

max_time=40 # Upper bound for the execution of the algorithm in seconds
mean_len=traj2.duration/4 # Mean length of each shortcut
std_len=traj2.duration/2 # Std of the length of the shortcut

# Run the shortcutting algorithm
n_runs=5 # Number of runs of the algorithm
trajlist=[]
ls=[]
la=[]
lc=[]
coeft=1.2 #Tolerance above the torque max
i=0
while i <n_runs:
    print i
    [traj3,d_list_s,d_list_all,n_collisions]=Shortcutting.IterateSmooth(robot,traj2,tunings,grav,max_time,mean_len,std_len,tau_min,tau_max,qd_max)
    tau3=Trajectory.ComputeTorques(robot,traj3,grav)
    print [max(abs(tau3[k,:])) for k in range(4)]
    if True not in [max(abs(tau3[k,:]))/coeft>tau_max[k] for k in range(4)]:  
        trajlist.append(traj3)
        ls.append(d_list_s)
        la.append(d_list_all)
        lc.append(n_collisions)
        i+=1

dd=[k[-1] for k in la]
i=dd.index(min(dd))
traj3=trajlist[i]

# Inverse dynamics of the shortcutted trajectory and plot
tau3=Trajectory.ComputeTorques(robot,traj3,grav)
mpl.axes.set_default_color_cycle(['r','b','g','m'])
figure(100)
Trajectory.PlotTorques(traj3.t_vect,tau3,tau_min,tau_max)
figure(101)
clf()
plot(traj3.t_vect,transpose(traj3.qd_vect))


print ' '
print '********************************************'
print '*************** Final result ***************'
print ' '
print 'Trajectory duration before shortcutting: '+str(traj2.duration)
print 'Trajectory duration after shortcutting: '+str(traj3.duration) + ' ('+ str(int(traj3.duration/traj2.duration*100))+'% of original)'
print ' '
print ' '
print ' '



######################### Return a rave traj ################################

dt_vect_res=zeros(len(traj3.t_vect))
dt_vect_res[1:len(traj3.t_vect)]=diff(traj3.t_vect)
via_points_res=transpose(traj3.q_vect)
rave_traj_res=RaveCreateTrajectory(env,'')
rave_traj_res.Init(spec)
rave_traj_res.Insert(0,c_[via_points_res,dt_vect_res].flatten())

# Execute the trajectory
robot.GetController().SetPath(rave_traj_res)
