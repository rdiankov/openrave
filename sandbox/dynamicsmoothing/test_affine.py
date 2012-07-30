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
import Affine
import AffineDemo
import copy

set_printoptions(precision=5)



################# Loading the environment ########################

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('/home/cuong/Dropbox/Code/mintime/robots/hrp4r.dae')
env.Load('/home/cuong/Dropbox/Code/mintime/robots/hrp4r.dae')
env.Load('/home/cuong/Dropbox/Code/mintime/robots/floorwalls.xml')

exclude_list=[1,2,17,5,6,8,9,12,13,15,17,19,20,23,24,26,27,28,29,30,31,32,33,34,35,36,37,40,41,43,44,45,46,47,48,49]

robot=env.GetRobots()[0]
robot2=env.GetRobots()[1]
[lower_lim_o,upper_lim_o]=robot.GetDOFLimits()
lower_lim=lower_lim_o/1.2
upper_lim=upper_lim_o/1.2
hrp=HRP4.HRP4robot(robot)
n=robot.GetDOF()
robot.SetDOFLimits(-1e10*ones(n),1e10*ones(n))
robot2.SetDOFLimits(-1e10*ones(n),1e10*ones(n))



xminf=-0.085
xmaxf=0.15
yminf=-0.16
ymaxf=0.16
zz=0.001
p1=array([xminf,yminf,zz])
p2=array([xmaxf,yminf,zz])
p3=array([xmaxf,ymaxf,zz])
p4=array([xminf,ymaxf,zz])
#handle_base=env.drawlinestrip(array([p1,p2,p3,p4,p1]),5)

border=0.007
xmin=xminf+border
xmax=xmaxf-border
ymin=yminf+border
ymax=ymaxf-border
bounds=[xmin,xmax,ymin,ymax]


############ Obtain the original movement #########################
file1='hrp4test/swing_table_tennis1-mod.csv'
c=1.15
q=loadtxt(file1,delimiter=',',skiprows=6)/1000./c
config=array([  3.33897e-01,   3.60542e-01,   8.00304e-01,   8.35608e-08,
         3.31824e-08,  -3.13480e+00,   0.00000e+00,   0.00000e+00,
         0.00000e+00,   0.00000e+00,   0.00000e+00,   0.00000e+00,
         0.00000e+00,   0.00000e+00,   0.00000e+00,   0.00000e+00,
         0.00000e+00,   0.00000e+00,   0.00000e+00,   0.00000e+00,
         0.00000e+00,   0.00000e+00,   0.00000e+00,   0.00000e+00,
         0.00000e+00,   0.00000e+00,   0.00000e+00,   0.00000e+00,
         0.00000e+00,   0.00000e+00,   0.00000e+00,   0.00000e+00,
         0.00000e+00,   0.00000e+00,   0.00000e+00,   0.00000e+00,
         0.00000e+00,   0.00000e+00,   2.83020e-01,  -1.16476e-01,
         0.00000e+00,  -6.88414e-01,   0.00000e+00,   0.00000e+00,
         0.00000e+00,  -1.48802e-09,   0.00000e+00,   2.95042e-01,
         1.05355e-01,   0.00000e+00,  -6.89127e-01,   0.00000e+00,
         0.00000e+00,   0.00000e+00,   7.01535e-09,   0.00000e+00])
HRP4.SetConfig(robot,config)
FollowTrajectory.compute_local_positions(robot,q,100,1,FollowTrajectory.markers_list)

Q0=eye(n+6)
Q0[0,0]=3
Q0[1,11]=3
Q0[2,2]=3
Q0[3,3]=2
Q0[4,4]=2
Q0[5,5]=2
params={'x':1}
params['robot']=robot
params['exclude_list']=exclude_list
params['dt']=0.1
params['Q0']=Q0
params['Q0inv']=linalg.inv(Q0)
params['lower_lim']=lower_lim
params['upper_lim']=upper_lim
params['K_v']=0
params['K_p']=1e-1
params['K_li']=1
params['K_sr']=1e-7
params['activate_base_translations']=True
params['activate_base_rotations']=False




HRP4.SetConfig(robot,config)
milestones=range(0,q.shape[0],10)
traj_orig=FollowTrajectory.Reach(FollowTrajectory.markers_list,milestones,20,params)
traj_immo=Trajectory.ImmobilizeBaselink(traj_orig)
#Trajectory.Execute(robot,traj_orig,0.1,stepsize=10)

duration=5
#sub=range(1300,1401,10)
sub=range(1251,1401,10)
q_vect2=traj_immo.q_vect[:,sub]
t_vect=linspace(0,duration,len(sub))

spline_traj=Trajectory.SplineInterpolateTrajectory(t_vect,q_vect2,s=1)
#traj_mod=spline_traj.GetSampleTraj(duration-1.25,0.005)
traj_mod=spline_traj.GetSampleTraj(duration-1,0.005)
#Trajectory.Execute(robot,traj_mod,0.002)

active_dofs=[]
for k in range(28,56):
    if abs(traj_mod.q_vect[k,-1]-traj_mod.q_vect[k,0])>1e-5:
        active_dofs.append(k)




################## Set the parameters ############################

offset=[0,1,0]
params['activate_base_translations']=False
params['activate_base_rotations']=False
params['sphere_radius']=0.08
params['n_steps']=20
params['linkindex']=22
params['linkindex2']=31
params['traj']=traj_mod
params['tau']=int(traj_mod.n_steps*0.25)
params['T']=int(traj_mod.n_steps*0.75)
params['active_dofs']=active_dofs
params['robot2']=robot2
params['offset']=offset
params['dist']=100
params['limits']=[lower_lim,upper_lim,lower_lim_o,upper_lim_o]





###################### Interactive #######################


q2_vect=copy.copy(traj_mod.q_vect)
for k in range(traj_mod.n_steps):
    q2_vect[[0,1,2],k]+=params['offset']


traj_mod_shift=Trajectory.SampleTrajectory(q2_vect)


def run():
    HRP4.SetConfig(robot,traj_mod.q_vect[:,params['T']])
    Trajectory.Execute(robot2,traj_mod_shift,0.001)


def run1():
    params['with_velocity']=False
    HRP4.SetConfig(robot,traj_mod.q_vect[:,params['T']])
    AffineDemo.start(params)

def run2():
    params['with_velocity']=True
    HRP4.SetConfig(robot,traj_mod.q_vect[:,params['T']])
    AffineDemo.start(params)

def stop():
    AffineDemo.stop(params)





# traj_def=AffineDemo.thread_data['traj_def']
# com=Trajectory.ComputeCOM(traj_def,{'robot':robot2,'exclude_list':[]})


# xmin=xminf+offset[0]
# xmax=xmaxf+offset[0]
# ymin=yminf+offset[1]
# ymax=ymaxf+offset[1]
# p1=array([xmin,ymin,zz])
# p2=array([xmax,ymin,zz])
# p3=array([xmax,ymax,zz])
# p4=array([xmin,ymax,zz])
# handle_base=env.drawlinestrip(array([p1,p2,p3,p4,p1]),5)

# figure(2)
# clf()
# plot([xmin,xmin,xmax,xmax,xmin],[ymin,ymax,ymax,ymin,ymin],'k--',linewidth=2)
# plot(com[0,:],com[1,:],'g',linewidth=2)
# axis('equal')
# grid('on')



# figure(3)
# clf()
# AffineDemo.plot_limits(AffineDemo.thread_data['traj_def'],'r')






# ############### Real experiment ######################


# n_steps=100
# linkindex=params['linkindex']
# linkindex2=params['linkindex2']
# tau=params['tau']
# T=params['T']
# q=traj_mod.q_vect[:,T]
# qd=traj_mod.q_vect[:,T]-traj_mod.q_vect[:,T-1]
# HRP4.SetConfig(robot,q)
# v=dot(AffineDemo.compute_Jacobian_speed(params),qd)

# #First traj
# p_sphere_pos=robot.GetLinks()[linkindex2].GetGlobalCOM()
# p_sphere_pos2=p_sphere_pos+array([0,0,-0.1])
# q2=AffineDemo.Reach(linkindex,linkindex2,p_sphere_pos2,n_steps,params)
# traj_defA=Affine.deform_traj(traj_mod,tau,T,q2,active_dofs)

# #Second traj
# with robot:
#     HRP4.SetConfig(robot,q2)
#     J=AffineDemo.compute_Jacobian_speed(params)

# qd2=qd+dot(linalg.pinv(J),v-dot(J,qd))
# traj_defB=Affine.deform_traj(traj_mod,tau,T,q2,active_dofs,qd,qd2)





# [j1x,traj_goto1]=Trajectory.goto(traj_mod.q_vect[:,T],800)
# [j2x,traj_goto2]=Trajectory.goto(traj_defA.q_vect[:,T],800)
# [j3x,traj_goto3]=Trajectory.goto(traj_defB.q_vect[:,T],800)
# [j1,traj_mod_pad]=Trajectory.prepare(traj_mod,800)
# [j2,traj_defA_pad]=Trajectory.prepare(traj_defA,800)
# [j3,traj_defB_pad]=Trajectory.prepare(traj_defB,800)



# savetxt('hrp4test/mov1x.pos',transpose(j1x))
# savetxt('hrp4test/mov2x.pos',transpose(j2x))
# savetxt('hrp4test/mov3x.pos',transpose(j3x))
# savetxt('hrp4test/mov1.pos',transpose(j1))
# savetxt('hrp4test/mov2.pos',transpose(j2))
# savetxt('hrp4test/mov3.pos',transpose(j3))




# figure(3)
# clf()
# AffineDemo.plot_limits(traj_goto,'y',params)
# AffineDemo.plot_limits(traj_mod_pad,'r',params)
# AffineDemo.plot_limits(traj_defA_pad,'b',params)
# AffineDemo.plot_limits(traj_defB_pad,'g',params)









# ###################### Snapshots ####################


# N=5
# for i in range(N-1):
#     env.Load('/home/cuong/Dropbox/Code/mintime/robots/hrp4r.dae')




# traj=traj_mod
# for i in range(N+1):
#     HRP4.SetConfig(env.GetRobots()[i],traj_mod.q_vect[:,i*120])



# robotN=env.GetRobots()[4]
# p=robotN.GetLinks()[linkindex2].GetGlobalCOM()
# p2=p+[0,0,0.1]
# AffineDemo.CreateSphere(p,[0,1,0],params)
# AffineDemo.CreateSphere(p2,[1,0,0],params)
















# #Test
# traj=traj_defB
# q=traj.q_vect[:,T]
# qd=traj.q_vect[:,T]-traj.q_vect[:,T-1]
# HRP4.SetConfig(robot,q)
# J=AffineDemo.compute_Jacobian_speed(params)
# p=robot.GetLinks()[linkindex2].GetGlobalCOM()
# v=dot(J,qd)
# print p
# print v















# traj_mod=spline_traj.GetSampleTraj(duration-1,0.01)

# tau=100
# T=300
# q_T=array(traj_mod.q_vect[:,T])
# q_Td=q_T
# q_Td[11]=q_T[11]+0.05
# qd_T=array(traj_mod.q_vect[:,T])-array(traj_mod.q_vect[:,T-1])
# qd_Td=v_T



# traj_mod_def=Affine.deform_traj(traj_mod,tau,T,q_Td,active_dofs)
# traj_mod_def2=Affine.deform_traj(traj_mod,tau,T,q_Td,active_dofs,qd_T,qd_Td)







# n_robots=len(env.GetRobots())
# for i in range(n_robots):
#     HRP4.SetConfig(env.GetRobots()[i],traj_mod.q_vect[:,100*i])





# n_steps=100

# hrp.halfsit()
# i=7
# p1=robot.GetLinks()[i].GetGlobalCOM()
# p1_vect=transpose(array(n_steps*[p1]))
# p1a=p1+array([1,0,0])
# p1a_vect=transpose(array(n_steps*[p1a]))
# p1b=p1+array([0,1,0])
# p1b_vect=transpose(array(n_steps*[p1b]))
# p1m={'link_index':i,'local_pos':robot.GetLinks()[i].GetLocalCOM(),'p_vect':p1_vect}
# p1am={'link_index':i,'local_pos':robot.GetLinks()[i].GetLocalCOM()+array([1,0,0]),'p_vect':p1a_vect}
# p1bm={'link_index':i,'local_pos':robot.GetLinks()[i].GetLocalCOM()+array([0,1,0]),'p_vect':p1b_vect}

# i=14
# incr=zeros((3,n_steps))
# incr[0,:]=linspace(0,0.5,n_steps)
# p2=robot.GetLinks()[i].GetGlobalCOM()
# p2_vect=transpose(array(n_steps*[p2]))+incr
# p2a=robot.GetLinks()[i].GetGlobalCOM()+array([1,0,0])
# p2a_vect=transpose(array(n_steps*[p2a]))+incr
# p2b=robot.GetLinks()[i].GetGlobalCOM()+array([0,1,0])
# p2b_vect=transpose(array(n_steps*[p2b]))+incr
# p2m={'link_index':i,'local_pos':robot.GetLinks()[i].GetLocalCOM(),'p_vect':p2_vect}
# p2am={'link_index':i,'local_pos':robot.GetLinks()[i].GetLocalCOM()+array([1,0,0]),'p_vect':p2a_vect}
# p2bm={'link_index':i,'local_pos':robot.GetLinks()[i].GetLocalCOM()+array([0,1,0]),'p_vect':p2b_vect}


# hrp.halfsit()
# traj=FollowTrajectory.Follow([p1m,p1am,p1bm,p2m,p2am,p2bm],params)
# Trajectory.Execute(robot,traj,0.01)














# ##### Test
# HRP4.SetConfig(robot,config)
# milestones=range(0,q.shape[0],10)
# res=zeros((56,len(milestones)))

# for i in range(len(milestones)):
#     trajx=FollowTrajectory.Reach(FollowTrajectory.markers_list,[milestones[i]],milestones[i],params)    
#     res[:,i]=trajx.q_vect[:,-1]

# traj=Trajectory.SampleTrajectory(array(res))
# Trajectory.Execute(robot,traj,0.1)
  
