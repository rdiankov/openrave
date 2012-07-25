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

hrp.halfsit()
T_init=robot.GetLinks()[0].GetTransform()[0:3,3]

q_init=array([  3.67439e-16,   1.21099e-16,   3.67439e-16,   1.21099e-16,
         3.67439e-16,   1.21099e-16,   3.67439e-16,   1.21099e-16,
         0.00000e+00,   4.59517e-16,   0.00000e+00,   4.59517e-16,
         0.00000e+00,   4.59517e-16,   0.00000e+00,   4.59517e-16,
         4.99177e-02,   9.18430e-02,  -7.83180e-01,   7.41884e-01,
        -1.88047e-01,   5.90535e-01,  -7.38187e-17,   2.00713e-02,
        -3.82053e-01,   7.19250e-01,  -3.27075e-01,  -1.91986e-02,
        -1.92704e-01,   5.67716e-01,   4.47993e-01,   4.36332e-01,
         6.18659e-02,  -1.28791e-01,  -5.31544e-02,  -1.41869e+00,
         9.87720e-02,   2.45604e-01,   1.15574e+00,   3.46240e-08,
         8.51790e-16,  -1.22022e+00,   8.99898e-01,   1.67790e-01,
        -1.65623e-01,   2.64022e-02,  -3.18204e-02,  -9.97006e-02,
         1.81067e-15,   1.00386e-15])


q_final=array([  7.84663e-17,  -6.98415e-17,   7.84663e-17,  -6.98415e-17,
         7.84663e-17,  -6.98415e-17,   7.84663e-17,  -6.98415e-17,
         5.05029e-17,  -6.16735e-17,   5.05029e-17,  -6.16735e-17,
         5.05029e-17,  -6.16735e-17,   5.05029e-17,  -6.16735e-17,
         3.12201e-01,   4.36332e-01,  -1.34390e+00,   1.30163e+00,
        -3.96849e-01,   4.23480e-01,  -3.34873e-17,   2.00713e-02,
        -3.82053e-01,   7.19250e-01,  -3.27075e-01,  -1.91986e-02,
         5.56841e-01,   6.28098e-01,   0.00000e+00,   0.00000e+00,
        -2.24163e+00,  -7.91972e-02,   5.74750e-02,  -2.14675e+00,
        -6.71243e-03,   4.68733e-02,   1.00602e-01,   1.69128e-08,
        -1.20655e-16,  -1.35723e+00,   1.11558e+00,   1.71919e-01,
        -1.57907e+00,  -7.67522e-03,  -9.38702e-02,   1.98582e-01,
        -8.08711e-08,   6.89982e-18])



q_init=concatenate([T_init,q_init])
q_final=concatenate([T_init,q_final])

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

T_list=[1.1]
pwp_traj=Trajectory.Interpolate(q_list,qd_list,T_list)
T=sum(T_list)
n_discr=500.
t_step=T/n_discr
traj=pwp_traj.GetSampleTraj(T,t_step)

T_list3=[2.1]
pwp_traj3=Trajectory.Interpolate(q_list,qd_list,T_list3)
T3=sum(T_list3)
n_discr=500.
t_step=T3/n_discr
traj3=pwp_traj3.GetSampleTraj(T3,t_step)


#figure(1)
#clf()
#plot(transpose(traj.q_vect))
#plot(transpose(traj.qd_vect))
#grid('on')


Trajectory.Execute(robot,traj,0.01)


xminf=-0.085
xmaxf=0.16
yminf=0.02
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
tunings['a_threshold']=0.01 # threshold in the zero inertia points search
tunings['width']=5 # window to test if we can get through a switching point
tunings['tolerance']=0.005 #tolerance above the max_curve
tunings['t_step_integrate']=t_step/40 # time step to integrate the limiting curves
tunings['sdot_init']=1
tunings['sdot_final']=1
tunings['threshold_final']=1e-2
tunings['threshold_waive']=1e-2

deb=time.time()
pb=MinimumTimeZMP.RobotMinimumTime(robot,traj,bounds,tunings,params)
s_res=pb.s_res
sdot_res=pb.sdot_res
undersample_coef=int(round(t_step/tunings['t_step_integrate']))
s_res_u=s_res[range(1,len(s_res),undersample_coef)]
sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]
traj2=pwp_traj.ResampleTraj(s_res_u,sdot_res_u,t_step)

print time.time()

zmp=Trajectory.ComputeZMP(traj,params)
zmp2=Trajectory.ComputeZMP(traj2,params)
zmp3=Trajectory.ComputeZMP(traj3,params)
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

plot(zmp3[0,0],zmp3[1,0],'ms',markersize=7)
plot(zmp3[0,-1],zmp3[1,-1],'mo',markersize=7)
plot(zmp3[0,:],zmp3[1,:],'m',linewidth=2)

plot(com[0,0],com[1,0],'gs',markersize=7)
plot(com[0,-1],com[1,-1],'go',markersize=7)
plot(com[0,:],com[1,:],'g',linewidth=2)

for k in gca().get_xticklabels():
    k.set_fontsize(18)

for k in gca().get_yticklabels():
    k.set_fontsize(18)

xlabel('Antero-posterior axis (m)',fontsize=20)
ylabel('Medio-lateral axis (m)',fontsize=20)
axis('equal')
grid('on')



figure(3)
pb.plot_limiting_curves()






############ Record images #####################

###################### Images #####################

V=env.GetViewer()
M=array([[ -5.61713e-02,   1.80862e-01,  -9.81903e-01,   2.77983e+00],
       [  9.98354e-01,  -1.21427e-03,  -5.73361e-02,   9.60523e-02],
       [ -1.15622e-02,  -9.83508e-01,  -1.80496e-01,   1.35762e+00],
       [  0.00000e+00,   0.00000e+00,   0.00000e+00,   1.00000e+00]])

V.SetCamera(M)

n_snaps=11
box=[250,0,530,480]
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


sd=1
sdd=1
i=0

base_T=array(robot.GetLinks()[0].GetTransform())
base_s=array([4,10,-2])
base_ss=array([-40,100,3])

q=traj.q_vect[:,i]
qs=traj.qd_vect[:,i]
qss=traj.qdd_vect[:,i]

qd=sd*qs
qdd=sdd*qs+sd*sd*qss
base_vel=sd*base_s
base_acc=sdd*base_s+sd*sd*base_ss


config=[q,qd,qdd,base_T,base_vel,base_acc]
config_pure=[q,qs,qss,base_T,base_s,base_ss]

ZMP.ComputeCOM(q,params_all)
ZMP.ComputeZMP(config,params_all)
[ax,bx,cx,ay,by,cy,d,e,f]=ZMP.ComputeCoefsFractionZMP(config_pure,params_all)
(ax*sdd+bx*sd*sd+cx)/(d*sdd+e*sd*sd+f)
(ay*sdd+by*sd*sd+cy)/(d*sdd+e*sd*sd+f)



deb=time.time()
for i in range(10):
    
print time.time()-deb




