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
Child class of MintimeProblem which considers the case of a
manipulator under torque limits
"""



from openravepy import *
from numpy import *
from pylab import *
import time
import MintimeProblemGeneric


class MintimeProblemTorque(MintimeProblemGeneric.MintimeProblemGeneric):



############################# Initialization ############################


    def __init__(self,robot,traj):        
        MintimeProblemGeneric.MintimeProblemGeneric.__init__(self,robot,traj)

    def set_dynamics_limits(self,limits):
        self.tau_min=limits[0]
        self.tau_max=limits[1]
        self.isset_dynamics_limits=True


################################ Dynamics ################################


    def sample_dynamics(self):
        """Sample the dynamics coefficients along the trajectory"""
        a_vect=zeros((self.dim,self.n_steps))
        b_vect=zeros((self.dim,self.n_steps))
        c_vect=zeros((self.dim,self.n_steps))

        for i in range(self.n_steps):
            q=self.q_vect[:,i]
            qd=self.qd_vect[:,i]
            qdd=self.qdd_vect[:,i]
            with self.robot:
                self.robot.SetDOFValues(q)
                self.robot.SetDOFVelocities(qd)
                tm,tc,tg = self.robot.ComputeInverseDynamics(qdd,None,returncomponents=True)
                to = self.robot.ComputeInverseDynamics(qd) - tc - tg

            a_vect[:,i]=to
            b_vect[:,i]=tm+tc
            c_vect[:,i]=tg

        self.a_vect=a_vect
        self.b_vect=b_vect
        self.c_vect=c_vect   

 

    def dynamics_coefficients(self,s):
        """Compute the dynamics coefficients at a given point by interpolation

        s -- point on the trajectory

        """
        return self.linear_interpolate_multi(s,[self.a_vect,self.b_vect,self.c_vect])




############################ Accel limits ################################


    def accel_limits(self,s,sdot):
        """Compute the acceleration limits caused by torque limits 

        (s,sdot) -- point of the phase plane

        """
        tau_min=self.tau_min
        tau_max=self.tau_max
        [a,b,c]=self.dynamics_coefficients(s)
        alpha=-1e15
        beta=1e15
        ialpha=0
        ibeta=0
        for i in range(self.dim):
            if a[i]>0:
                tau_min_i=tau_min[i]
                tau_max_i=tau_max[i]
            else:
                tau_min_i=tau_max[i]
                tau_max_i=tau_min[i]
            alpha_i=(tau_min_i-b[i]*sdot**2-c[i])/a[i]
            beta_i=(tau_max_i-b[i]*sdot**2-c[i])/a[i]
            if alpha_i>alpha:
                alpha=alpha_i
                ialpha=i
            if beta_i<beta:
                beta=beta_i
                ibeta=i
        return [alpha,beta,ialpha,ibeta]


    def maxvel_accel(self,s):
        """Compute the maximum velocity caused by torque limits 
        
        s -- point on the trajectory
        
        """
        tau_min=self.tau_min
        tau_max=self.tau_max
        [a,b,c]=self.dynamics_coefficients(s)
        tau_alpha=zeros(self.dim)
        tau_beta=zeros(self.dim)
        for i in range(self.dim):
            if a[i]>0:
                tau_alpha[i]=tau_min[i]
                tau_beta[i]=tau_max[i]
            else:
                tau_alpha[i]=tau_max[i]
                tau_beta[i]=tau_min[i]

        [alpha,beta,ialpha,ibeta]=self.accel_limits(s,0)
        if alpha>beta:
            return 0

        sdot_min=1e15
        for k in range(self.dim):
            for m in range(k+1,self.dim):
                r=(a[k]*(tau_alpha[m]-c[m])-a[m]*(tau_beta[k]-c[k]))/(a[k]*b[m]-a[m]*b[k])
                if r>=0:
                    sdot=sqrt(r)
                    if sdot<sdot_min:
                        sdot_min=sdot
                r=(a[m]*(tau_alpha[k]-c[k])-a[k]*(tau_beta[m]-c[m]))/(a[m]*b[k]-a[k]*b[m])
                if r>=0:
                    sdot=sqrt(r)
                    if sdot<sdot_min:
                        sdot_min=sdot
        return(sdot_min)



########################## Zero-inertia points ############################


    def find_zero_inertia_points(self):
        """Find all zero-inertia points and assign to the list self.sw_zero_inertia"""
        if self.n_steps<1:
            self.sw_zero_inertia=[]
            return
        s=self.t_vect[0]
        [ap,bp,cp]=self.dynamics_coefficients(s)
        i_list=[]
        for i in range(1,self.n_steps-1):
            s=self.t_vect[i]            
            [a,b,c]=self.dynamics_coefficients(s)
            for j in range(self.dim):
                aj=a[j]
                ajp=ap[j]
                if aj*ajp<0:
                    if (not self.isset_velocity_limits) or (self.maxvel_accel_curve[i]<self.maxvel_velocity_curve[i]):
                        if abs(aj)<abs(ajp):
                            if not (i in i_list):
                                i_list.append(i)
                            if self.maxvel_curve[i-1]< self.maxvel_curve[i]:
                                if not ((i-1) in i_list):
                                    i_list.append(i-1)
                        else:
                            if not ((i-1) in i_list):
                                i_list.append(i-1)
                            if self.maxvel_curve[i]< self.maxvel_curve[i-1]:
                                if not (i in i_list):
                                    i_list.append(i)
            ap=a

        self.sw_zero_inertia=i_list


    def correct_accel_zi(self,s):
        """Compute the correct acceleration at a zero-inertia point

        s -- a zero-inertia point on the trajectory

        """
        sdot=self.maxvel_interp(s)
        [a,b,c]=self.dynamics_coefficients(s)        
        a_abs=list(abs(a))
        i_sat=a_abs.index(min(a_abs))
        delta=0.01

        [a2,b2,c2]=self.dynamics_coefficients(s-delta)
        a2=a2[i_sat]
        b2=b2[i_sat]
        c2=c2[i_sat]

        a=a[i_sat]
        b=b[i_sat]
        c=c[i_sat]
        ap=-(a2-a)/delta
        bp=-(b2-b)/delta
        cp=-(c2-c)/delta
        return ((-bp*sdot*sdot-cp)/(2*b*sdot+ap*sdot))*sdot
        




################## Trajectory utilities ################################

def ComputeTorques(robot,traj,grav):

    robot.GetEnv().GetPhysicsEngine().SetGravity(grav)

    n_steps=traj.n_steps
    t_vect=traj.t_vect
    q_vect=traj.q_vect
    qd_vect=traj.qd_vect
    qdd_vect=traj.qdd_vect
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

    #Smooth out the first steps
    if n_steps>2:
        tau_vect[:,0]=tau_vect[:,2]
        tau_vect[:,1]=tau_vect[:,2]

    #Smooth out the last two steps
    if n_steps>2:
        tau_vect[:,n_steps-1]=tau_vect[:,n_steps-3]
        tau_vect[:,n_steps-2]=tau_vect[:,n_steps-3]

    return tau_vect


def PlotTorques(t_vect,tau,tau_min,tau_max,offset=0,reverse=False):
    T=t_vect[-1]
    hold('on')
    plot([offset,T+offset],array([tau_min,tau_min]),'--',linewidth=2)
    plot([offset,T+offset],array([tau_max,tau_max]),'--',linewidth=2)
    if not reverse:
        plot(t_vect+offset,transpose(tau),linewidth=2)
    else:
        plot(t_vect+offset,transpose(reverse_array(tau)),linewidth=2)
    for k in gca().get_xticklabels():
        k.set_fontsize(18)
    for k in gca().get_yticklabels():
        k.set_fontsize(18)
    xlabel('Time (s)',fontsize=20)
    ylabel('Torque (Nm)',fontsize=20)
    grid('on')


def Execute(robot,traj,t_sleep,stepsize=1):
    for i in range(0,traj.n_steps,stepsize):
        robot.SetDOFValues(traj.q_vect[:,i])
        time.sleep(t_sleep)


def CheckCollisionTraj(robot,traj):
    for i in range(traj.n_steps):
        with robot:
            robot.SetDOFValues(traj.q_vect[:,i])
            if robot.GetEnv().CheckCollision(robot):
                return [True,'env',i]
            if robot.CheckSelfCollision():
                return [True,'self',i]
    return [False,None,None]



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

