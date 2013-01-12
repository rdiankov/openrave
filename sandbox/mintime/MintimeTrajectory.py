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


from numpy import *
from scipy.interpolate import *
import bisect



#################### Interpolation utilities ################################

         
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




#################### Trajectory manipulations ###########################


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


def reverse_array(a):
    b=array(a)
    if len(shape(a))>1:
        for i in range(a.shape[0]):
            for j in range(a.shape[1]):
                b[i,j]=a[i,a.shape[1]-j-1]
    else:
        for j in range(len(a)):
            b[j]=a[len(a)-j-1]
    return b


def Reverse(traj):
    new_traj=SampleTrajectory()
    new_traj.dim=traj.dim
    new_traj.n_steps=traj.n_steps
    new_traj.t_step=traj.t_step
    new_traj.duration=traj.duration
    new_traj.t_vect=reverse_array(traj.t_vect)
    new_traj.q_vect=reverse_array(traj.q_vect)
    new_traj.qd_vect=reverse_array(traj.qd_vect)
    new_traj.qdd_vect=reverse_array(traj.qdd_vect)
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



####################### Trajectory classes ##############################


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





