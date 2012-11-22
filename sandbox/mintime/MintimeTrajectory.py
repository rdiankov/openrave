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
Defines and initializes various types of trajectories
Utilities for processing trajectories
"""


from openravepy import *
from numpy import *
from pylab import *
from scipy.interpolate import *
import time
import copy
import bisect
import HRP4
import ZMP



##############################################################
####################### Utilities ############################
##############################################################


################## Execution and Collision checking #########################


def Execute(robot,sample_traj,t_sleep,stepsize=1,drawcom=0,wait_end=0):
    global com_handle
    n=robot.GetDOF()
    for i in range(0,sample_traj.n_steps,stepsize):
        if sample_traj.dim==n+6:
            robot.GetLinks()[0].SetTransform(HRP4.v2t(sample_traj.q_vect[0:6,i]))
            robot.SetDOFValues(sample_traj.q_vect[6:sample_traj.dim,i])
        else:
            robot.SetDOFValues(sample_traj.q_vect[:,i])
    
        if drawcom==1:
            CoM=ZMP.ComputeCOM([robot.GetLinks()[0].GetTransform()[0:3,3],robot.GetDOFValues()],{'robot':robot,'exclude_list':[]})
            CoM_proj=zeros(3)
            CoM_proj[0]=CoM[0]
            CoM_proj[1]=CoM[1]
            com_handle=robot.GetEnv().drawlinestrip(array([CoM,CoM_proj]),5)
        elif drawcom==2:
            q=sample_traj.q_vect[:,i]
            qd=sample_traj.qd_vect[:,i]
            qdd=sample_traj.qdd_vect[:,i]
            zmp=ZMP.ComputeZMP([q[0:3],qd[0:3],qdd[0:3],q[6:len(q)],qd[6:len(q)],qdd[6:len(q)]],{'robot':robot,'exclude_list':[],'gravity':9.8,'moment_coef':1})
            zmp_proj=array([zmp[0],zmp[1],0])
            zmp_high=array([zmp[0],zmp[1],0.5])
            zmp_handle=robot.GetEnv().drawlinestrip(array([zmp_proj,zmp_high]),5)
            
        time.sleep(t_sleep)

    time.sleep(wait_end)


def CheckCollisionTraj(robot,sample_traj):
    n=robot.GetDOF()
    for i in range(sample_traj.n_steps):
        with robot:
            if sample_traj.dim==n+6:
                robot.GetLinks()[0].SetTransform(v2t(sample_traj.q_vect[0:6,i]))
                robot.SetDOFValues(sample_traj.q_vect[6:sample_traj.dim,i])
            else:
                robot.SetDOFValues(sample_traj.q_vect[:,i])
            if robot.GetEnv().CheckCollision(robot):
                return [True,'env',i]
            if robot.CheckSelfCollision():
                return [True,'self',i]
    return [False,None,None]


def ImmobilizeBaselink(traj):

    q_vect2=zeros(traj.q_vect.shape)

    for i in range(traj.n_steps):
        q_vect2[0:6,i]=[0,0,0.75,0,0,0]
        q_vect2[22:34,i]=HRP4.halfsitPose[0:12]
        q_vect2[34:56,i]=traj.q_vect[34:56,i]

    return SampleTrajectory(q_vect2)


def ramp(q0,qd0,q1,qd1,n):
    res=zeros(n)
    n2=n*n
    n3=n2*n
    M=array([[n3,n2],
             [3*n2,2*n]])
    B=array([q1-q0-qd0*n,qd1-qd0])
    A=linalg.solve(M,B)
    for i in range(n):
        i2=i*i
        i3=i*i2
        res[i]=A[0]*i3+A[1]*i2+qd0*i+q0
    return res


def prepare(traj,npad):
    N=34
    q_vect=traj.q_vect[22:56,:]
    q_vect2_red=zeros((N,traj.n_steps+npad))
    for i in range(N):
        q_vect2_red[i,0:npad]=ramp(HRP4.halfsitPose2[i],0,q_vect[i,0],q_vect[i,1]-q_vect[i,0],npad)
        q_vect2_red[i,npad:npad+traj.n_steps]=q_vect[i,:]

    q_vect2_full=zeros((56,traj.n_steps+npad))
    q_vect2_full[22:56,:]=q_vect2_red
    q_vect2_full[2,:]=[0.75]*(traj.n_steps+npad)

    q_vect2_red2=zeros((N+1,traj.n_steps+npad))
    q_vect2_red2[0,:]=arange(0,(traj.n_steps+npad-0.999999999)*0.005,0.005)
    q_vect2_red2[1:N+1,:]=q_vect2_red 

    return [q_vect2_red2,SampleTrajectory(q_vect2_full)]



def goto(q,npad):
    N=34
    q_red=q[22:56]
    q_vect2_red=zeros((N,2*npad))
    for i in range(N):
        q_vect2_red[i,0:npad]=ramp(HRP4.halfsitPose[i],0,HRP4.halfsitPose2[i],0,npad)
        q_vect2_red[i,npad:2*npad]=ramp(HRP4.halfsitPose2[i],0,q_red[i],0,npad)

    q_vect2_full=zeros((56,2*npad))
    q_vect2_full[22:56,:]=q_vect2_red
    q_vect2_full[2,:]=[0.75]*(2*npad)

    q_vect2_red2=zeros((N+1,2*npad))
    q_vect2_red2[0,:]=arange(0,(2*npad-0.999999999)*0.005,0.005)
    q_vect2_red2[1:N+1,:]=q_vect2_red 

    return [q_vect2_red2,SampleTrajectory(q_vect2_full)]



################## Torques computation ################################

def ComputeTorques(robot,sample_traj,grav):

    robot.GetEnv().GetPhysicsEngine().SetGravity(grav)

    n_steps=sample_traj.n_steps
    t_vect=sample_traj.t_vect
    q_vect=sample_traj.q_vect
    qd_vect=sample_traj.qd_vect
    qdd_vect=sample_traj.qdd_vect
    tau_vect=zeros(shape(q_vect))

    for i in range(n_steps):
        q=q_vect[:,i]
        qd=qd_vect[:,i]
        qdd=qdd_vect[:,i]
        with robot:
            robot.SetDOFValues(q)
            robot.SetDOFVelocities(qd)
            tau = robot.ComputeInverseDynamics(qdd,None,returncomponents=False)

        tau_vect[:,i]=tau

    return tau_vect


def PlotTorques(t_vect,tau,tau_min,tau_max):
    T=t_vect[-1]
    clf()
    hold('on')
    plot(t_vect,transpose(tau),linewidth=2)
    plot([0,T],array([tau_min,tau_min]),'--',linewidth=2)
    plot([0,T],array([tau_max,tau_max]),'--',linewidth=2)
    for k in gca().get_xticklabels():
        k.set_fontsize(18)
    for k in gca().get_yticklabels():
        k.set_fontsize(18)
    xlabel('Time (s)',fontsize=20)
    ylabel('Torque (Nm)',fontsize=20)
    axis([0,T,min(tau_min)*1.2,max(tau_max)*1.2])
    grid('on')


def PlotVelocities(t_vect,qd_vect,qd_max):
    T=t_vect[-1]
    clf()
    hold('on')
    plot(t_vect,transpose(qd_vect),linewidth=2)
    plot([0,T],array([-qd_max,-qd_max]),'k--',linewidth=2)
    plot([0,T],array([qd_max,qd_max]),'k--',linewidth=2)
    for k in gca().get_xticklabels():
        k.set_fontsize(18)
    for k in gca().get_yticklabels():
        k.set_fontsize(18)
    xlabel('Time (s)',fontsize=20)
    ylabel('Velocity (rad.s-1)',fontsize=20)
    axis([0,T,-max(qd_max)*1.2,max(qd_max)*1.2])
    grid('on')




################## ZMP computation ################################


def ComputeZMP(sample_traj,params_init):
    n_steps=sample_traj.n_steps
    t_vect=sample_traj.t_vect
    q_vect=sample_traj.q_vect
    qd_vect=sample_traj.qd_vect
    qdd_vect=sample_traj.qdd_vect
    zmp_vect=zeros((2,n_steps))
    for i in range(n_steps):
        q=q_vect[:,i]
        qd=qd_vect[:,i]
        qdd=qdd_vect[:,i]
        # Here we assume there is no base link rotation
        zmp=ZMP.ComputeZMP([q[0:3],qd[0:3],qdd[0:3],q[6:len(q)],qd[6:len(q)],qdd[6:len(q)]],params_init)
        zmp_vect[:,i]=zmp
    return zmp_vect

def ComputeCOM(sample_traj,params_init):
    n_steps=sample_traj.n_steps
    t_vect=sample_traj.t_vect
    q_vect=sample_traj.q_vect
    com_vect=zeros((3,n_steps))
    for i in range(n_steps):
        q=q_vect[:,i]
        # Here we assume there is no base link rotation
        com_vect[:,i]=ZMP.ComputeCOM([q[0:3],q[6:len(q)]],params_init)

    return com_vect

    


################## Interpolation ################################
         
        
def Interpolate(q_list,qd_list,T_list):
    P_list=[]
    for i in range(len(q_list)-1):
        t=[]
        for j in range(len(q_list[0])):
            t.append(poly1d(SimpleInterpolate(q_list[i][j],q_list[i+1][j],qd_list[i][j],qd_list[i+1][j],T_list[i])))
        P_list.append(t)
    return PieceWisePolyTrajectory(P_list,T_list)


def SimpleInterpolate(q0,q1,qd0,qd1,T):
    a=((qd1-qd0)*T-2*(q1-q0-qd0*T))/T**3
    b=(3*(q1-q0-qd0*T)-(qd1-qd0)*T)/T**2
    c=qd0
    d=q0
    return poly1d([a,b,c,d])

def SimpleInterpolate5(q0,q1,T):
    M=array([[0,0,0,0,0,1],
             [0,0,0,0,1,0],
             [0,0,0,2,0,0],
             [T**5,T**4,T**3,T**2,T,1],
             [5*T**4,4*T**3,3*T**2,2*T,1,0],
             [20*T**3,12*T**2,6*T,2,0,0]])
    C=array([q0,0,0,q1,0,0])
    return poly1d(linalg.solve(M,C))


def InterpolateParabola(xi,vi,xf,vf,T):
    M=array([[0,0,1,0,0,0],
             [0,1,0,0,0,0],
             [T*T/4,T/2,1,0,0,-1],
             [T,1,0,0,-1,0],
             [0,0,0,T*T/4,T/2,1],
             [0,0,0,T,1,0]])
    P1=[]
    P2=[]
    for i in range(len(xi)):
        C=array([xi[i],vi[i],0,0,xf[i],vf[i]])
        X=linalg.solve(M,C)
        P1.append(poly1d([X[0],X[1],X[2]]))
        P2.append(poly1d([X[3],X[4],X[5]]))

    return([P1,P2])


#################### Trajectory utilities ###########################


def CastTrajRave(traj,dof_list,t_step):

    duration=traj.GetDuration()
    t_vect=arange(0,duration+1e-10,t_step)
    dim=len(dof_list)
    n=len(t_vect)
    t_step=t_vect[1]-t_vect[0]
    t_step2=2*t_step

    q_vect=zeros((dim,n))
    qd_vect=zeros((dim,n))
    qdd_vect=zeros((dim,n))

    for i in range(n):
        q_vect[:,i]=traj.Sample(t_vect[i])[dof_list]

    for i in range(1,n-1):
        qd_vect[:,i]=(q_vect[:,i+1]-q_vect[:,i-1])/t_step2
    qd_vect[:,0]=(q_vect[:,1]-q_vect[:,0])/t_step
    qd_vect[:,n-1]=(q_vect[:,n-1]-q_vect[:,n-2])/t_step

    for i in range(1,n-1):
        qdd_vect[:,i]=(qd_vect[:,i+1]-qd_vect[:,i-1])/t_step2
    qdd_vect[:,0]=(qd_vect[:,1]-qd_vect[:,0])/t_step
    qdd_vect[:,n-1]=(qd_vect[:,n-1]-qd_vect[:,n-2])/t_step

    new_traj=SampleTrajectory()
    new_traj.duration=duration
    new_traj.dim=dim
    new_traj.t_vect=t_vect
    new_traj.t_step=t_step
    new_traj.n_steps=n
    new_traj.q_vect=q_vect
    new_traj.qd_vect=qd_vect
    new_traj.qdd_vect=qdd_vect

    return new_traj


def DynamicShift(robot,traj,fixed_link_i):
    fixed_link=robot.GetLinks()[fixed_link_i]
    base_link=robot.GetLinks()[0]
    traj2=copy.deepcopy(traj)
    with robot:
        HRP4.SetConfig(robot,traj.q_vect[:,0])
        fixed_link_init=fixed_link.GetGlobalCOM()
    for i in range(traj.n_steps):
        with robot:
            HRP4.SetConfig(robot,traj.q_vect[:,i])
            fixed_link_cur=fixed_link.GetGlobalCOM()
            shift=fixed_link_init-fixed_link_cur
        traj2.q_vect[0:3,i]=traj2.q_vect[0:3,i]+shift
    return traj2


def Concat(traj_list):
    traj=traj_list.pop(0)
    while traj_list!=[]:
        traj=Glue(traj,traj_list.pop(0))
    return traj
    

def Glue(traj1,traj2):
    new_traj=SampleTrajectory()
    new_traj.dim=traj1.dim
    new_traj.n_steps=traj1.n_steps+traj2.n_steps-1
    new_traj.t_step=traj1.t_step
    new_traj.duration=new_traj.t_step*(new_traj.n_steps-1)
    new_traj.t_vect=r_[traj1.t_vect[0:-1],traj2.t_vect+traj1.t_vect[-1]]
    new_traj.q_vect=c_[traj1.q_vect[:,0:-1],traj2.q_vect]
    new_traj.qd_vect=c_[traj1.qd_vect[:,0:-1],traj2.qd_vect]
    new_traj.qdd_vect=c_[traj1.qdd_vect[:,0:-1],traj2.qdd_vect]
    return new_traj

def Sub(traj,t1,t2=None):
    new_traj=SampleTrajectory()
    new_traj.dim=traj.dim
    if t2==None:
        t2=traj.n_steps
    new_traj.n_steps=t2-t1
    new_traj.t_step=traj.t_step
    new_traj.duration=new_traj.t_step*(new_traj.n_steps-1)
    new_traj.t_vect=traj.t_vect[range(t1,t2)]-traj.t_vect[t1]
    new_traj.q_vect=traj.q_vect[:,range(t1,t2)]
    new_traj.qd_vect=traj.qd_vect[:,range(t1,t2)]
    new_traj.qdd_vect=traj.qdd_vect[:,range(t1,t2)]
    return new_traj    


def Insert(traj,i1,i2,trajx):
    n_steps=traj.n_steps    
    t_vect=traj.t_vect
    q_vect=traj.q_vect
    t_step=traj.t_step
    qd_vect=traj.qd_vect
    qdd_vect=traj.qdd_vect
    n_stepsx=trajx.n_steps
    n=i1+(n_steps-i2-1)+n_stepsx
    
    t_vect_new=zeros(n)
    q_vect_new=zeros((traj.dim,n))
    qd_vect_new=zeros((traj.dim,n))
    qdd_vect_new=zeros((traj.dim,n))

    t_vect_new[range(i1)]=t_vect[range(i1)]
    t_vect_new[range(i1,i1+n_stepsx)]=t_vect[i1]+trajx.t_vect
    t_vect_new[range(i1+n_stepsx,n)]=trajx.t_vect[-1]-trajx.t_vect[0]-(t_vect[i2]-t_vect[i1])+t_vect[range(i2+1,n_steps)]

    q_vect_new[:,range(i1)]=q_vect[:,range(i1)]
    q_vect_new[:,range(i1,i1+n_stepsx)]=trajx.q_vect
    q_vect_new[:,range(i1+n_stepsx,n)]=q_vect[:,range(i2+1,n_steps)]

    qd_vect_new[:,range(i1)]=qd_vect[:,range(i1)]    
    qd_vect_new[:,range(i1,i1+n_stepsx)]=trajx.qd_vect
    qd_vect_new[:,range(i1+n_stepsx,n)]=qd_vect[:,range(i2+1,n_steps)]
    
    qdd_vect_new[:,range(i1)]=qdd_vect[:,range(i1)]
    qdd_vect_new[:,range(i1,i1+n_stepsx)]=trajx.qdd_vect
    qdd_vect_new[:,range(i1+n_stepsx,n)]=qdd_vect[:,range(i2+1,n_steps)]

    new_traj=SampleTrajectory()
    new_traj.n_steps=n
    new_traj.t_vect=t_vect_new
    new_traj.t_step=t_step
    new_traj.q_vect=q_vect_new
    new_traj.qd_vect=qd_vect_new
    new_traj.qdd_vect=qdd_vect_new
    new_traj.dim=traj.dim
    new_traj.duration=t_step*(n-1)

    return new_traj


# Smooth a trajectory given by RRT by dichotomy
def linear_smooth_dichotomy(robot,vp_list,coll_check_step):
    if len(vp_list)==2:
        return vp_list
    else:
        p1=vp_list[0]
        p2=vp_list[-1]
        d=norm(p2-p1)
        v_unit=(p2-p1)/d
        for t in linspace(0,d,d/coll_check_step+1):
            p=p1+t*v_unit
            with robot:
                robot.SetDOFValues(p)
                if robot.GetEnv().CheckCollision(robot):
                    l1=linear_smooth_dichotomy(robot,vp_list[0:len(vp_list)/2+1],coll_check_step)
                    l2=linear_smooth_dichotomy(robot,vp_list[len(vp_list)/2:len(vp_list)],coll_check_step)
                    l1.extend(l2[1:])
                    return l1
        return [p1,p2]





##############################################################
####################### Classes ##############################
##############################################################


class MintimeTrajectory():    
    def GetSampleTraj(self,duration,t_step):
        sample_traj=SampleTrajectory()
        t_vect=arange(0,duration+1e-10,t_step)
        sample_traj.dim=self.dim
        sample_traj.t_vect=t_vect
        sample_traj.n_steps=len(t_vect)
        sample_traj.t_step=t_step
        [r_val,r_vel,r_acc]=self.val_vel_acc_vect(t_vect)
        sample_traj.q_vect=r_val
        sample_traj.qd_vect=r_vel
        sample_traj.qdd_vect=r_acc
        sample_traj.duration=duration
        sample_traj.t_step=t_step

        return sample_traj


    def ResampleTraj(self,s_vect,sdot_vect,t_step):
        n=len(s_vect)
        q_vect=zeros((self.dim,n))
        qd_vect=zeros((self.dim,n))
        qdd_vect=zeros((self.dim,n))

        for i in range(n):
            q_vect[:,i]=self.value(s_vect[i])

        for i in range(0,n-1):
            qd_vect[:,i]=(q_vect[:,i+1]-q_vect[:,i])/t_step
        qd_vect[:,n-1]=(q_vect[:,n-1]-q_vect[:,n-2])/t_step

        for i in range(0,n-1):
            qdd_vect[:,i]=(qd_vect[:,i+1]-qd_vect[:,i])/t_step
        qdd_vect[:,n-1]=(qd_vect[:,n-1]-qd_vect[:,n-2])/t_step

        new_traj=SampleTrajectory()
        new_traj.n_steps=n
        new_traj.t_vect=arange(0,(n-1)*t_step+1e-10,t_step)
        new_traj.q_vect=q_vect
        new_traj.qd_vect=qd_vect
        new_traj.qdd_vect=qdd_vect
        new_traj.t_step=t_step
        new_traj.duration=t_step*(n-1)
        new_traj.dim=self.dim

        return new_traj


class SampleTrajectory(MintimeTrajectory):
    def __init__(self,t_vect=None,q_vect=None,qd_vect=None,qdd_vect=None):
        if t_vect!=None:
            self.t_vect=t_vect
            self.t_step=t_vect[1]-t_vect[0]
            self.duration=t_vect[-1]-t_vect[0]
        if q_vect!=None:
            (dim,n_steps)=shape(q_vect)
            self.q_vect=q_vect
            self.dim=dim
            self.n_steps=n_steps            
        self.qd_vect=qd_vect
        self.qdd_vect=qdd_vect

    def value(self,s):
        t_vect=self.t_vect
        q_vect=self.q_vect
        i=bisect.bisect_left(t_vect,s)
        if i==0: 
            return q_vect[:,i]
        r=(s-t_vect[i-1])/(t_vect[i]-t_vect[i-1])
        return (1-r)*q_vect[:,i-1]+r*q_vect[:,i] 
       


class SplineInterpolateTrajectory(MintimeTrajectory):

    def __init__(self,t_vect,q_vect,k=4,s=0):
        self.dim=shape(q_vect)[0]
        self.splines_list=[]
        for i in range(self.dim):
            self.splines_list.append(UnivariateSpline(t_vect,q_vect[i,:],k=k,s=s))

    def value(self,t):
        q_vect=zeros(self.dim)
        for i in range(self.dim):
            der=self.splines_list[i].derivatives(t)
            q_vect[i]=der[0]
        return q_vect
        

    def val_vel_acc_vect(self,t_vect):
        n_steps=len(t_vect)
        new_q_vect=zeros((self.dim,n_steps))
        new_qd_vect=zeros((self.dim,n_steps))
        new_qdd_vect=zeros((self.dim,n_steps))
        for i in range(self.dim):
            for j in range(n_steps):
                der=self.splines_list[i].derivatives(t_vect[j])
                new_q_vect[i,j]=der[0]
                new_qd_vect[i,j]=der[1]
                new_qdd_vect[i,j]=der[2]
        return [new_q_vect,new_qd_vect,new_qdd_vect]
 

class PieceWisePolyTrajectory(MintimeTrajectory):

    def evaluate_list(self,l,t):
        r=zeros(self.dim)
        for i in range(self.dim):
            r[i]=l[i](t)
        return r

    def __init__(self,pieces_list,durations_list):
        self.n_pieces=len(pieces_list)
        self.dim=len(pieces_list[0])
        self.pieces_list=pieces_list
        self.velocities_list=map(lambda x:map(polyder,x),self.pieces_list)
        self.accelerations_list=map(lambda x:map(polyder,x),self.velocities_list)
        self.durations_list=durations_list
        self.duration=sum(durations_list)

    def find_piece(self,t):
        t_cur=0
        for i in range(self.n_pieces):
            t_cur+=self.durations_list[i]
            if t_cur>=t-1e-10:
                return [i,t_cur-self.durations_list[i]]
        raise NameError('t larger than total duration')

    def val_vel_acc_vect(self,t_vect):
        n_samples=t_vect.shape[0]
        r_val=zeros((self.dim,n_samples))
        r_vel=zeros((self.dim,n_samples))
        r_acc=zeros((self.dim,n_samples))
        for i in range(n_samples):
            [val,vel,acc]=self.val_vel_acc(t_vect[i])
            r_val[:,i]=val
            r_vel[:,i]=vel
            r_acc[:,i]=acc
        return [r_val,r_vel,r_acc]

    ########## Not mandatory ##########
    def value(self,t):
        [i,t_deb]=self.find_piece(t)
        return self.evaluate_list(self.pieces_list[i],t-t_deb)

    def velocity(self,t):
        [i,t_deb]=self.find_piece(t)
        return self.evaluate_list(self.velocities_list[i],t-t_deb)
    
    def acceleration(self,t):
        [i,t_deb]=self.find_piece(t)
        return self.evaluate_list(self.accelerations_list[i],t-t_deb)

    def val_vel_acc(self,t):
        [i,t_deb]=self.find_piece(t)
        val=self.evaluate_list(self.pieces_list[i],t-t_deb)
        vel=self.evaluate_list(self.velocities_list[i],t-t_deb)
        acc=self.evaluate_list(self.accelerations_list[i],t-t_deb)
        return [val,vel,acc]

    def value_vect(self,t_vect):
        n_samples=t_vect.shape[0]
        r=zeros((self.dim,n_samples))
        for i in range(n_samples):
            r[:,i]=self.value(t_vect[i])
        return r

    def velocity_vect(self,t_vect):
        n_samples=t_vect.shape[0]
        r=zeros((self.dim,n_samples))
        for i in range(n_samples):
            r[:,i]=self.velocity(t_vect[i])
        return r

    def acceleration_vect(self,t_vect):
        n_samples=t_vect.shape[0]
        r=zeros((self.dim,n_samples))
        for i in range(n_samples):
            r[:,i]=self.acceleration(t_vect[i])
        return r






