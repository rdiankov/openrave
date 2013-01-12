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
Test file for MintimeProblemZMP
"""



from openravepy import *
from numpy import *
from pylab import *
import time
import MintimeTrajectory
import MintimeProblemZMP
import MintimeProfileIntegrator
import HRP4
import ZMP

#import scipy
#import Image



################# Loading the environment ########################

set_printoptions(precision=5)
ion()

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('robots/hrp4r.dae')
env.Load('robots/floorwalls.xml')

collisionChecker = RaveCreateCollisionChecker(env,'pqp')
env.SetCollisionChecker(collisionChecker)

robot=env.GetRobots()[0]
hrp=HRP4.HRP4robot(robot)
n=robot.GetDOF()

g=9.8
env.GetPhysicsEngine().SetGravity([0,0,-g])


# DOF and velocity limits
dof_lim=robot.GetDOFLimits()
vel_lim=robot.GetDOFVelocityLimits()
robot.SetDOFLimits(-3*ones(n),3*ones(n))
robot.SetDOFVelocityLimits(100*vel_lim)


exclude_list=[]
for k in range(n):
    if robot.GetLinks()[k].GetMass()<0.01:
        exclude_list.append(k)

# Parameters for ZMP computation
params={'robot':robot,'gravity':g,'moment_coef':1,'exclude_list':exclude_list}


V=robot.GetEnv().GetViewer()
M=array([[ 0.05117,  0.09089, -0.99455,  2.74898],
       [ 0.99847,  0.01639,  0.05287, -0.17906],
       [ 0.02111, -0.99573, -0.08991,  0.98455],
       [ 0.     ,  0.     ,  0.     ,  1.     ]])
V.SetCamera(M)



##################### Define a test trajectory ###########################

# Initial config
T_init=array([[ 1.     ,  0.     ,  0.     , -0.00169 ],
       [ 0.     ,  1.     ,  0.     , -0.16625],
       [ 0.     ,  0.     ,  1.     , 0.54949 ],
       [ 0.     ,  0.     ,  0.     ,  1.     ]])
j_init=array([  1.04662e-16,   1.10352e-15,   1.04662e-16,   1.10352e-15,
         1.04662e-16,   1.10352e-15,   1.04662e-16,   1.10352e-15,
         0.00000e+00,   7.06569e-17,   5.28097e-17,   7.06569e-17,
        -4.60806e-15,   7.06569e-17,   0.00000e+00,   7.06569e-17,
        -3.55255e-03,   3.27627e-01,  -8.17893e-01,   1.59803e+00,
        -7.70391e-01,  -3.50313e-01,  -3.66871e-03,   3.75582e-01,
        -8.26181e-01,   1.61937e+00,  -7.82397e-01,  -3.74728e-01,
         5.23599e-01,   5.34707e-01,  -1.22174e+00,   4.36337e-01,
        -4.43160e-01,  -1.09261e+00,  -1.33531e-01,  -2.29060e-01,
        -9.15966e-02,  -9.40728e-02,   3.69845e-01,   2.78079e-15,
         5.28139e-17,   5.63246e-02,   8.95126e-01,   1.56273e-01,
        -3.35607e-01,   2.89823e-01,  -2.54493e-01,  -4.46619e-01,
        -4.80704e-08,  -1.04609e-16])
a1=-0.6
a2=-0.5
j_init[8]=a1
j_init[9]=a2
j_init[10]=a1
j_init[11]=a2
j_init[12]=a1
j_init[13]=a2
j_init[14]=a1
j_init[15]=a2
j_init[39]=1.1 #Thumb
j_init[40]=0.7 #Four fingers
j_init[36]=-0.91 #Wrist yaw
j_init[37]=0.66  #Wrist pitch
j_init[31]=0.227 #Neck
q_init=concatenate([T_init[0:3,3],array([0,0,0]),j_init])


# Final config
T_final=array([[ 1.     ,  0.     ,  0.     , -0.00169],
       [ 0.     ,  1.     ,  0.     ,  0.12328],
       [ 0.     ,  0.     ,  1.     ,  0.65617],
       [ 0.     ,  0.     ,  0.     ,  1.     ]])

j_final=array([ -6.03447e-16,  -1.62853e-15,   8.08812e-18,  -2.18690e-15,
         3.10914e-16,   4.15906e-18,   3.10914e-16,   4.15906e-18,
         7.17591e-16,   5.12392e-17,   3.58762e-16,  -1.93760e-15,
         7.17591e-16,   5.12392e-17,   7.17591e-16,   5.12392e-17,
         1.81557e-03,  -2.38055e-01,  -6.37043e-01,   1.23217e+00,
        -5.85675e-01,   2.15393e-01,   2.30682e-03,  -1.98688e-01,
        -6.26159e-01,   1.21459e+00,  -5.78231e-01,   1.99572e-01,
         5.23667e-01,  -6.10923e-01,  -1.22173e+00,  -5.23599e-01,
        -1.53525e+00,  -1.28782e+00,  -1.01680e-01,  -1.09716e+00,
        -6.89126e-03,  -4.62218e-01,   8.00249e-01,   2.20941e-15,
         0.00000e+00,  -2.08712e+00,   2.16326e-01,   5.76610e-02,
        -7.47006e-01,   2.89823e-01,  -2.54493e-01,  -4.46619e-01,
        -4.80704e-08,  -1.63475e-13])
j_final[8]=a1
j_final[9]=a2
j_final[10]=a1
j_final[11]=a2
j_final[12]=a1
j_final[13]=a2
j_final[14]=a1
j_final[15]=a2
j_final[39]=1.1 #Thumb
j_final[40]=0.7 #Four fingers
j_final[37]=-0.9  #Wrist pitch
q_final=concatenate([T_final[0:3,3],array([0,0,0]),j_final])


# Trajectory

qd0=zeros(len(q_init))
qd1=zeros(len(q_init))
v=0.01

for i in range(len(q_init)):
    if abs(q_init[i]-q_final[i])>1e-2:
        if q_init[i]<q_final[i]:
            qd0[i]=v
            qd1[i]=v
        else:
            qd0[i]=-v
            qd1[i]=-v


q_list=[q_init,q_final]
qd_list=[qd0,qd1]

T_list=[1.4]
pwp_traj=MintimeTrajectory.Interpolate(q_list,qd_list,T_list)
T=sum(T_list)


n_discr=200.
t_step=T/n_discr
traj_1=pwp_traj.GetSampleTraj(T,t_step)

traj_2=MintimeProblemZMP.DynamicShift(robot,traj_1,7)
spline_traj=MintimeTrajectory.SplineInterpolateTrajectory(traj_2.t_vect,traj_2.q_vect,k=3,s=0)
traj=spline_traj.GetSampleTraj(T,t_step)


# Bounds
xminf=-0.085
xmaxf=0.15
yminf=-0.16
ymaxf=0.16
bounds=[xminf,xmaxf,yminf,ymaxf]
zz=0.001
p1=array([xminf,yminf,zz])
p2=array([xmaxf,yminf,zz])
p3=array([xmaxf,ymaxf,zz])
p4=array([xminf,ymaxf,zz])
handle_base=env.drawlinestrip(array([p1,p2,p3,p4,p1]),5)





##################### Run the algorithm ###########################


deb=time.time()

# Set up a minimum time problem under ZMP constraints

print 'Running the time parameterization algorithm...'


pb=MintimeProblemZMP.MintimeProblemZMP(robot,traj)
pb.set_dynamics_limits(bounds)
pb.disc_thr=100 # Threshold in the discontinuity point search
pb.zmp_params=params # Parameters for ZMP computations
pb.preprocess()
#clf()
#pb.plot_maxvel_curves()


# Integrate the limiting curves and the final profile


algo=MintimeProfileIntegrator.MintimeProfileIntegrator(pb)
algo.dt_integ=t_step/10 # time step to integrate the limiting curves
algo.width=10 # window to test if we can get through a switch point
algo.palier=20 # length of the palier around zero inertia points
algo.tolerance_ends=1e-2 # threshold at the end
algo.sdot_init=1 # initial value of sdot
algo.sdot_final=1 # final value of sdot

algo.possible=True
algo.integrate_all_profiles()
if algo.possible:
    algo.integrate_final()


# Reparameterize the path using the new velocity profile

s_res=algo.s_res
sdot_res=algo.sdot_res
undersample_coef=int(round(t_step/algo.dt_integ))
s_res_u=s_res[range(1,len(s_res),undersample_coef)]
sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]
traj2=spline_traj.ResampleTraj(s_res_u,sdot_res_u,t_step)





##################### Plotting ###########################

print '\n*****************************************************************\n'
print 'Total execution time: '+str(time.time()-deb)+'s'
print '\n*****************************************************************\n'

raw_input('Press Enter to execute the trajectory /before/ time-reparameterization (duration='+str(traj.duration)+'s)')
MintimeProblemZMP.Execute(robot,traj,0.02,drawcom=2)
raw_input('Press Enter to execute the trajectory /after/ time-reparameterization (duration='+str(traj2.duration)+'s)')
MintimeProblemZMP.Execute(robot,traj2,0.02,drawcom=2)


zmp=ZMP.ComputeZMPTraj(traj,params)
zmp2=ZMP.ComputeZMPTraj(traj2,params)
com=ZMP.ComputeCOMTraj(traj,params)



figure(1)
clf()
algo.plot_profiles()
axis([0,1.4,0,10])
title('Phase plane')
xlabel('$s$',fontsize=15)
ylabel('$\dot s$',fontsize=15)
#savefig('../../Reda/mintime/fig/zmp-phase.eps')



figure(2)
clf()
plot([xminf,xminf,xmaxf,xmaxf,xminf],[yminf,ymaxf,ymaxf,yminf,yminf],'k',linewidth=2)

plot(zmp[0,0],zmp[1,0],'rs',markersize=7)
plot(zmp[0,-1],zmp[1,-1],'ro',markersize=7)
plot(zmp[0,:],zmp[1,:],'r',linewidth=2)

plot(zmp2[0,0],zmp2[1,0],'bs',markersize=7)
plot(zmp2[0,-1],zmp2[1,-1],'bo',markersize=7)
plot(zmp2[0,:],zmp2[1,:],'b',linewidth=2)

plot(com[0,0],com[1,0],'gs',markersize=7)
plot(com[0,-1],com[1,-1],'go',markersize=7)
plot(com[0,:],com[1,:],'g',linewidth=2)

axis('equal')
grid('on')
title('2D position of the ZMP')
xlabel('X-axis (antero-posterior) (m)',fontsize=15)
ylabel('Y-axis (medio-lateral) (m)',fontsize=15)
#savefig('../../Reda/mintime/fig/zmp-2dpos.eps')


figure(3)
clf()
plot([0,traj.duration],[xminf,xminf],'m--',linewidth=2)
plot([0,traj.duration],[xmaxf,xmaxf],'m--',linewidth=2)
plot([0,traj.duration],[yminf,yminf],'c--',linewidth=2)
plot([0,traj.duration],[ymaxf,ymaxf],'c--',linewidth=2)
plot(traj.t_vect,zmp[0,:],'m-.',linewidth=2)
plot(traj.t_vect,zmp[1,:],'c-.',linewidth=2)
plot(traj2.t_vect,zmp2[0,:],'m',linewidth=2)
plot(traj2.t_vect,zmp2[1,:],'c',linewidth=2)
xlabel('Time (s)',fontsize=15)
ylabel('ZMP coordinates (m)',fontsize=15)
title('ZMP coordinates as a function of time')
#savefig('../../Reda/mintime/fig/zmp-intime.eps')



######################## Images ##################################


# n_snaps=20
# box=[130,0,480,480]
# cur_traj=traj2
# color=[0,0,1]
# name='after'
# ni=0
# for i in [int(round(k)) for k in linspace(0,cur_traj.n_steps-1,n_snaps)]:
#     with robot:
#         q=cur_traj.q_vect[:,i]
#         qd=cur_traj.qd_vect[:,i]
#         qdd=cur_traj.qdd_vect[:,i]
#         robot.GetLinks()[0].SetTransform(HRP4.v2t(q[0:6]))
#         robot.SetDOFValues(q[6:cur_traj.dim])
#         zmp=ZMP.ComputeZMP([q[0:3],qd[0:3],qdd[0:3],q[6:len(q)],qd[6:len(q)],qdd[6:len(q)]],{'robot':robot,'exclude_list':[],'gravity':9.8,'moment_coef':1})
#         zmp_proj=array([zmp[0],zmp[1],0])
#         ss=MintimeTrajectory.CreateSphere(env,zmp_proj,0.05,color)
#         I=V.GetCameraImage(640,480,M,[640,640,320,240])
#         env.Remove(ss)
#         scipy.misc.imsave('tmp.jpg',I)
#         im=Image.open('tmp.jpg')
#         im2=im.crop(box)
#         im2.save('../../Reda/mintime/fig/hrp4-'+name+'-'+str(ni)+'.jpg')
#         ni+=1


