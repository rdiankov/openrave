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
from scipy.interpolate import *
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


params={'robot':robot,'gravity':g,'moment_coef':1,'exclude_list':exclude_list}
params_all={'robot':robot,'gravity':g,'moment_coef':1,'exclude_list':[]}


##################### Initial config ###########################

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


################# Final config ###########################

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
pwp_traj=Trajectory.Interpolate(q_list,qd_list,T_list)
T=sum(T_list)


n_discr=500
t_step=T/n_discr
traj_1=pwp_traj.GetSampleTraj(T,t_step)

traj_2=Trajectory.DynamicShift(robot,traj_1,7)
spline_traj=Trajectory.SplineInterpolateTrajectory(traj_2.t_vect,traj_2.q_vect,k=3,s=0)
traj=spline_traj.GetSampleTraj(T,t_step)


Trajectory.Execute(robot,traj,0.01)




xminf=-0.085
xmaxf=0.15
yminf=-0.16
ymaxf=0.16
zz=0.001
p1=array([xminf,yminf,zz])
p2=array([xmaxf,yminf,zz])
p3=array([xmaxf,ymaxf,zz])
p4=array([xminf,ymaxf,zz])
handle_base=env.drawlinestrip(array([p1,p2,p3,p4,p1]),5)

border=0.007
xmin=xminf+border
xmax=xmaxf-border
ymin=yminf+border
ymax=ymaxf-border
bounds=[xmin,xmax,ymin,ymax]

tunings={'':0}
tunings['i_threshold']=10 # threshold in the tangent points search
tunings['slope_threshold']=1 # threshold in the tangent points search
tunings['sdot_threshold']=0.05 # threshold in the discontinuity points search
tunings['a_threshold']=0.05 # threshold in the zero inertia points search
tunings['width']=10 # window to test if we can get through a switching point
tunings['tolerance']=0.005 #tolerance above the max_curve
tunings['t_step_integrate']=t_step/20 # time step to integrate the limiting curves
tunings['sdot_init']=1
tunings['sdot_final']=1
tunings['threshold_final']=1e-2
tunings['threshold_waive']=1e-2



deb=time.time()
pb=MinimumTimeZMP.RobotMinimumTime(robot,traj,bounds,tunings,params)

figure(1)
pb.plot_limiting_curves()

s_res=pb.s_res
sdot_res=pb.sdot_res
undersample_coef=int(round(t_step/tunings['t_step_integrate']))
s_res_u=s_res[range(1,len(s_res),undersample_coef)]
sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]
traj2=spline_traj.ResampleTraj(s_res_u,sdot_res_u,t_step)

print time.time()-deb

zmp=Trajectory.ComputeZMP(traj,params)
zmp2=Trajectory.ComputeZMP(traj2,params)
#zmp3=Trajectory.ComputeZMP(traj3,params)
com=Trajectory.ComputeCOM(traj,params)


figure(2)
clf()
plot([xmin,xmin,xmax,xmax,xmin],[ymin,ymax,ymax,ymin,ymin],'k--',linewidth=2)
plot([xminf,xminf,xmaxf,xmaxf,xminf],[yminf,ymaxf,ymaxf,yminf,yminf],'k',linewidth=2)

plot(zmp[0,0],zmp[1,0],'rs',markersize=7)
plot(zmp[0,-1],zmp[1,-1],'ro',markersize=7)
plot(zmp[0,:],zmp[1,:],'r',linewidth=2)

plot(zmp2[0,0],zmp2[1,0],'bs',markersize=7)
plot(zmp2[0,-1],zmp2[1,-1],'bo',markersize=7)
plot(zmp2[0,:],zmp2[1,:],'b',linewidth=2)

#plot(zmp3[0,0],zmp3[1,0],'ys',markersize=7)
#plot(zmp3[0,-1],zmp3[1,-1],'yo',markersize=7)
#plot(zmp3[0,:],zmp3[1,:],'y',linewidth=2)

plot(com[0,0],com[1,0],'gs',markersize=7)
plot(com[0,-1],com[1,-1],'go',markersize=7)
plot(com[0,:],com[1,:],'g',linewidth=2)

#for k in gca().get_xticklabels():
#    k.set_fontsize(18)

#for k in gca().get_yticklabels():
#    k.set_fontsize(18)

axis('equal')
grid('on')
xlabel('X-axis (antero-posterior) (m)',fontsize=15)
ylabel('Y-axis (medio-lateral) (m)',fontsize=15)


figure(3)
clf()
plot([0,traj2.duration],[xmin,xmin],'m--',linewidth=2)
plot([0,traj2.duration],[xmax,xmax],'m--',linewidth=2)
plot([0,traj2.duration],[ymin,ymin],'c--',linewidth=2)
plot([0,traj2.duration],[ymax,ymax],'c--',linewidth=2)
plot(traj.t_vect,zmp[0,:],'m-.',linewidth=2)
plot(traj.t_vect,zmp[1,:],'c-.',linewidth=2)
plot(traj2.t_vect,zmp2[0,:],'m',linewidth=2)
plot(traj2.t_vect,zmp2[1,:],'c',linewidth=2)
xlabel('Time (s)',fontsize=15)
ylabel('ZMP coordinates (m)',fontsize=15)


figure(3)
clf()
plot(pb.sample_t_vect,pb.sample_max_curve,'b',linewidth=2)

for i in range(len(pb.s_traj_list)):
    plot(pb.s_traj_list[i],pb.sdot_traj_list[i],'r',linewidth=2)

for i in range(len(pb.s_list)):
    if pb.type_list[i]=='t':
        plot(pb.s_list[i],pb.sdot_list[i],'ro',markersize=10)
    if pb.type_list[i]=='d':
        plot(pb.s_list[i],pb.sdot_list[i],'bo',markersize=10)
    if pb.type_list[i]=='z':
        plot(pb.s_list[i],pb.sdot_list[i],'go',markersize=10)

plot(pb.s_res,pb.sdot_res,'k--',linewidth=2)
xlabel('Path parameter $s$',fontsize=15)
ylabel('Velocity $\dot s$',fontsize=15)
axis([0,T,0,10])
grid('on')






############ Record images #####################

###################### Images #####################

V=env.GetViewer()
M=array([[ 0.21974,  0.24695, -0.94379,  2.6838 ],
       [ 0.97527, -0.03198,  0.2187 , -0.59939],
       [ 0.02382, -0.9685 , -0.24787,  1.52736],
       [ 0.     ,  0.     ,  0.     ,  1.     ]])

V.SetCamera(M)

n_snaps=11
box=[130,0,480,480]
ct=traj
ni=0
for i in [int(round(k)) for k in linspace(0,ct.n_steps-1,n_snaps)]:
    with robot:
        robot.GetLinks()[0].SetTransform(Trajectory.v2t(ct.q_vect[0:3,i]))
        robot.SetDOFValues(ct.q_vect[3:ct.dim,i])
        I=V.GetCameraImage(640,480,M,[640,640,320,240])
        scipy.misc.imsave('tmp.jpg',I)
        im=Image.open('tmp.jpg')
        im2=im.crop(box)
        im2.save('../../Reda/mintime/fig/hrp4-'+str(ni)+'.eps')
        ni+=1












#### Test the basic ZMP computation


sd=1.5
sdd=20
i=300

base_T=traj.q_vect[0:3,i]
base_s=traj.qd_vect[0:3,i]
base_ss=traj.qdd_vect[0:3,i]

q=traj.q_vect[3:traj.n_steps,i]
qs=traj.qd_vect[3:traj.n_steps:,i]
qss=traj.qdd_vect[3:traj.n_steps:,i]

qd=sd*qs
qdd=sdd*qs+sd*sd*qss
base_vel=sd*base_s
base_acc=sdd*base_s+sd*sd*base_ss


config=[base_T,base_vel,base_acc,q,qd,qdd]
config_pure=[base_T,base_s,base_ss,q,qs,qss]

ZMP.ComputeCOM([base_T,q],params)
zmp=ZMP.ComputeZMP(config,params)
[ax,bx,cx,ay,by,cy,d,e,f]=ZMP.ComputeCoefsFractionZMP(config_pure,params_all)

(ax*sdd+bx*sd*sd+cx)/(d*sdd+e*sd*sd+f)-zmp[0]
(ay*sdd+by*sd*sd+cy)/(d*sdd+e*sd*sd+f)-zmp[1]



deb=time.time()
for i in range(1000):
    HRP4.SetConfig(robot,q_init)
    robot.CalculateJacobian(0,[0,0,0]);

print (time.time()-deb)/1000






