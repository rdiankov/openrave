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
Compute the minimum time trajectory under torque limits
Based on Bobrow, Dubowsky, Schiller, Slotine, Yang
"""

from openravepy import *
from numpy import *
from pylab import *
import bisect
import Trajectory


def respect_bounds(tau,tau_min,tau_max):
    (k,n)=shape(tau)
    for i in range(k):
        for j in range(n):
            if tau[i,j]>tau_max[i] or tau[i,j]<tau_min[i]:
                return False
    return True
        





def ApproximateIntegrate(robot,sample_traj,tau_min,tau_max,grav,ibeg,iend,margin_coef):
    
    print '----------------'
    print ibeg
    print iend




def ApproximateDichotomy(robot,sample_traj,tau_min,tau_max,grav,ibeg,iend,margin_coef):


    print '----------------'
    print ibeg
    print iend
    #print sample_traj.t_vect[ibeg]
    #print sample_traj.t_vect[iend]
    
    #if sample_traj.t_vect[iend]-sample_traj.t_vect[ibeg]<tunings['switchtime']:
    #    return ([],[])

    T=(sample_traj.t_vect[iend]-sample_traj.t_vect[ibeg])*margin_coef

    if iend-ibeg<4:
        P_list=Trajectory.InterpolateParabola(sample_traj.q_vect[:,ibeg],sample_traj.qd_vect[:,ibeg],sample_traj.q_vect[:,iend],sample_traj.qd_vect[:,iend],T)
        return (P_list,[T/2,T/2])


    P_list=Trajectory.InterpolateParabola(sample_traj.q_vect[:,ibeg],sample_traj.qd_vect[:,ibeg],sample_traj.q_vect[:,iend],sample_traj.qd_vect[:,iend],T)
    subtraj=Trajectory.PieceWisePolyTrajectory(P_list,[T/2,T/2]).GetSampleTraj(0.001)
    tau=Trajectory.ComputeTorques(robot,subtraj,grav)

    if respect_bounds(tau,tau_min,tau_max) and (not Trajectory.CheckCollisionTraj(robot,subtraj)[0]):
        return (P_list,[T/2,T/2])
    else:
        (P1_list,T1_list)=ApproximateDichotomy(robot,sample_traj,tau_min,tau_max,grav,ibeg,(iend+ibeg)/2,margin_coef)
        (P2_list,T2_list)=ApproximateDichotomy(robot,sample_traj,tau_min,tau_max,grav,(iend+ibeg)/2,iend,margin_coef)
        P1_list.extend(P2_list)
        T1_list.extend(T2_list)
        return (P1_list,T1_list)




class RobotMinimumTime():

    ##########################################
    def __init__(self,robot,sample_traj,tau_min,tau_max,tunings,grav):

        # Set the robot parameters
        self.robot=robot
        self.env=robot.GetEnv()

        # Trajectory-related definitions
        self.sample_traj=sample_traj
        self.dim=sample_traj.dim
        self.sample_t_step=sample_traj.t_step
        self.sample_n_steps=sample_traj.n_steps
        self.sample_duration=sample_traj.duration
        self.sample_t_vect=sample_traj.t_vect
        self.sample_q_vect=sample_traj.q_vect
        self.sample_qd_vect=sample_traj.qd_vect
        self.sample_qdd_vect=sample_traj.qdd_vect

        self.tau_min=tau_min
        self.tau_max=tau_max

        self.tunings=tunings

        self.grav=grav

        self.sample_dynamics()
        self.sample_max_velocity_curve()
        self.find_switching_points()

        self.sdot_init=1
        self.sdot_final=1
        self.possible=True
        self.compute_limiting_curves()
        if self.possible:
            self.integrate_final()


    ##########################################
    # Compute the dynamics along the sample trajectory
    def sample_dynamics(self):
        a_vect=zeros((self.dim,self.sample_n_steps))
        b_vect=zeros((self.dim,self.sample_n_steps))
        c_vect=zeros((self.dim,self.sample_n_steps))
        self.env.GetPhysicsEngine().SetGravity(self.grav)

        for i in range(self.sample_n_steps):
            q=self.sample_q_vect[:,i]
            qd=self.sample_qd_vect[:,i]
            qdd=self.sample_qdd_vect[:,i]

            with self.robot:
                self.robot.SetDOFValues(q)
                self.robot.SetDOFVelocities(qd)
                tm,tc,tg = self.robot.ComputeInverseDynamics(qdd,None,returncomponents=True)
                to = self.robot.ComputeInverseDynamics(qd) - tc - tg

            a_vect[:,i]=to
            b_vect[:,i]=tm+tc
            c_vect[:,i]=tg

        self.sample_a_vect=a_vect
        self.sample_b_vect=b_vect
        self.sample_c_vect=c_vect   


    ##########################################
    # Compute the max velocity curve
    def sample_max_velocity_curve(self):
        max_curve=zeros(self.sample_n_steps)
        for i in range(self.sample_n_steps):
            max_curve[i]=self.max_velocity(self.sample_t_vect[i])
        self.sample_max_curve=max_curve


    def sample_max_velocity_curve_vellim(self,qd_max):        
        n_steps=self.sample_n_steps
        max_curve=ones(n_steps)*1e10
        for i in range(n_steps):
            qd=self.sample_qd_vect[:,i]
            for j in range(self.dim):
                max_curve[i]=min(max_curve[i],qd_max[j]/abs(qd[j]))

        self.sample_max_curve_vellim=max_curve
       


    ##########################################
    # Find the switching points
    def find_switching_points(self):

        tunings=self.tunings
        dt=tunings['t_step_integrate']
        i_threshold=tunings['i_threshold']
        slope_threshold=tunings['slope_threshold']
        sdot_threshold=tunings['sdot_threshold']
        a_threshold=tunings['a_threshold']

        # Find the tangent points
        [i_list,diff_list]=self.find_tangent_points(i_threshold,slope_threshold)
        #print(i_list)
        # Find the discontinuous points        
        #i_list2=self.find_discontinuous_points(sdot_threshold)
        i_list2=[]
        # Find the zero inertia points        
        i_list3=self.find_zero_inertia_points(a_threshold)
        # Merge the lists
        type_list=['t']*len(i_list)
        for x in i_list2:
            index=bisect.bisect_left(i_list,x)
            i_list.insert(index,x)
            type_list.insert(index,'d')
        for x in i_list3:
            index=bisect.bisect_left(i_list,x)
            i_list.insert(index,x)
            type_list.insert(index,'z')
        self.i_list=i_list
        self.type_list=type_list

        # Find the corresponding values of sdot
        s_list=[]
        sdot_list=[]
        for j in range(len(i_list)):
            s_list.append(self.sample_t_vect[i_list[j]])
            sdot=self.sample_max_curve[i_list[j]]
            sdot_list.append(sdot)
        self.s_list=s_list
        self.sdot_list=sdot_list


    ##########################################
    # Compute the limiting curves
    def compute_limiting_curves(self):

        tunings=self.tunings
        sdot_init=tunings['sdot_init']
        sdot_final=tunings['sdot_final']
        tolerance=tunings['tolerance']
        dt=tunings['t_step_integrate']
        width=tunings['width']

        s_traj_list=[]
        sdot_traj_list=[]
        cat_list=[]
        duration=self.sample_duration

        #Final part
        [s_final_backward,sdot_final_backward]=self.integrate_backward(duration,sdot_final,[],[])
        s_traj_list.append(s_final_backward)
        sdot_traj_list.append(sdot_final_backward)
        cat_list.append('alpha')

        #Initial part
        [s_forward,sdot_forward]=self.integrate_forward(0,sdot_init,[s_final_backward],[sdot_final_backward])
        s_traj_list.append(s_forward)
        sdot_traj_list.append(sdot_forward)
        cat_list.append('beta')

        if len(s_final_backward)==0 or len(s_forward)==0:
            self.possible=False
            return

        s_target= s_final_backward[0]
        s_farthest=s_forward[-1]
        s_list_copy=list(self.s_list)
        sdot_list_copy=list(self.sdot_list)
        type_list_copy=list(self.type_list)

        while True:
            if s_farthest>s_target: break
            if s_farthest>duration: break
            if len(s_list_copy)==0: break
            si=s_list_copy.pop(0)
            sdoti=sdot_list_copy.pop(0)
            typei=type_list_copy.pop(0)

            if si<s_farthest: continue

            if  typei=='z' or  typei=='d':
                sdoti=self.find_sdot_min(si,sdoti)

            #Backward part            
            [s_backward,sdot_backward]=self.integrate_backward(si,sdoti,s_traj_list,sdot_traj_list,acc=self.compute_acc(si))
            if s_backward[0]>s_farthest: continue

            #Forward part            
            [s_forward,sdot_forward]=self.integrate_forward(si,sdoti,[s_final_backward],[sdot_final_backward],acc=self.compute_acc(si))
            s_farthest=s_forward[-1]                

            #Save the trajectories
            s_traj_list.append(s_backward)
            sdot_traj_list.append(sdot_backward)
            cat_list.append('alpha')                
            s_traj_list.append(s_forward)
            sdot_traj_list.append(sdot_forward)
            cat_list.append('beta')
 
        if len(s_traj_list)==0:
            self.possible=False
            return
           
        self.s_traj_list=s_traj_list
        self.sdot_traj_list=sdot_traj_list
        self.cat_list=cat_list


    ##########################################
    # Integrate the final trajectory
    def integrate_final(self):

        s_traj_list=self.s_traj_list
        sdot_traj_list=self.sdot_traj_list
        dt=self.tunings['t_step_integrate']

        s_curr=0
        s_res=[]
        sdot_res=[]
        while True:
            if s_curr>self.sample_duration: break
            s_res.append(s_curr)
            [index_min,sdot_min]=self.compute_index(s_curr,s_traj_list,sdot_traj_list)
            s_curr=s_curr+sdot_min*dt            
            sdot_res.append(sdot_min)

        self.s_res=array(s_res)
        self.sdot_res=array(sdot_res)
        if s_res[-1]<self.sample_duration-self.tunings['threshold_final']:
            self.possible=False


    ##########################################
    # Find the zero inertia points
    def find_zero_inertia_points(self,a_threshold):
        i_list=[]
        for i in range(self.sample_n_steps-1):
            s=self.sample_t_vect[i]
            sn=self.sample_t_vect[i+1]
            [a,b,c]=self.dynamics_coefficients(s)
            [an,bn,cn]=self.dynamics_coefficients(sn)
            for j in range(self.dim):
                aj=a[j]
                ajn=an[j]
                if aj*ajn<0:
                    if abs(aj)<abs(ajn):
                        i_list.append(i)    
                    else:
                        i_list.append(i+1)    
        return i_list


    ##########################################
    # Find the discontinuous points
    def find_discontinuous_points(self,sdot_threshold):
        i_list=[]
        max_curve=self.sample_max_curve
        for i in range(1,self.sample_n_steps-1):
            if max_curve[i+1]-max_curve[i]>sdot_threshold:
                if max_curve[i]-max_curve[i-1]<sdot_threshold/2:
                    i_list.append(i)
        return i_list


    ##########################################
    # Find the tangent points
    def find_tangent_points(self,i_threshold,slope_threshold):
        max_curve=self.sample_max_curve
        i_list=[]
        diff_list=[]
        for i in range(self.sample_n_steps-1):
            s=self.sample_t_vect[i]
            sdot=max_curve[i]
            [alpha,beta,k_alpha,k_beta]=self.compute_limits(s,sdot)
            diff=abs(alpha/sdot-(max_curve[i+1]-max_curve[i])/self.sample_t_step)
            if diff<slope_threshold:
                i_list.append(i)
                diff_list.append(diff)
        #print(i_list)
        """Pruning the list"""
        i_list_list=[]
        diff_list_list=[]
        i=0
        while i<len(i_list):
            #create a new list
            i_list_list.append([])
            diff_list_list.append([])
            cont=True
            while cont:
                i_list_list[-1].append(i_list[i])
                diff_list_list[-1].append(diff_list[i])
                i+=1
                if i<len(i_list):
                    if i_list[i]-i_list[i-1]>i_threshold:
                        cont=False
                else:
                    cont=False
        #print(i_list_list)
        i_list2=[]
        diff_list2=[]
        for i in range(len(i_list_list)):
            i_best=0
            diff_best=1e15
            for j in range(len(i_list_list[i])):
                if diff_list_list[i][j]<diff_best:
                    diff_best=diff_list_list[i][j]
                    i_best=i_list_list[i][j]
            i_list2.append(i_best)
            diff_list2.append(diff_best)
        return [i_list2,diff_list2]


    ##########################################
    # Find the highest sdot that allows passing through switching point
    def find_sdot_min(self,s,sdot):

        #print '-------'
        #print s
        #print sdot

        tunings=self.tunings
        tolerance=tunings['tolerance']
        dt=tunings['t_step_integrate']
        width=tunings['width']

        bound_top=sdot
        bound_bottom=0.2
        min_sep=0.01
        if bound_top<bound_bottom: return 0
        sdot=bound_top
        [s_forward,sdot_forward]=self.integrate_forward(s,sdot,[],[],width,acc=self.compute_acc(s))
        [s_backward,sdot_backward]=self.integrate_backward(s,sdot,[],[],width,acc=self.compute_acc(s))
        #print s_forward
        #print s_backward
        if (len(s_forward)>=width or s_forward[-1]>self.sample_duration) and (len(s_backward)>=width or s_backward[0]<-1):
            return bound_top
        sdot=bound_bottom
        [s_forward,sdot_forward]=self.integrate_forward(s,sdot,[],[],width,acc=self.compute_acc(s))
        [s_backward,sdot_backward]=self.integrate_backward(s,sdot,[],[],width,acc=self.compute_acc(s))
        #print s_forward
        #print s_backward
        if not ((len(s_forward)>=width or s_forward[-1]>self.sample_duration) and (len(s_backward)>=width or s_backward[0]<-1)):
            return 0
        while bound_top-bound_bottom>min_sep:
            sdot=(bound_top+bound_bottom)/2
            [s_forward,sdot_forward]=self.integrate_forward(s,sdot,[],[],width,acc=self.compute_acc(s))
            [s_backward,sdot_backward]=self.integrate_backward(s,sdot,[],[],width,acc=self.compute_acc(s))
            if (len(s_forward)>=width or s_forward[-1]>self.sample_duration) and (len(s_backward)>=width or s_backward[0]<-1):
                bound_bottom=sdot
            else:
                bound_top=sdot
        return bound_bottom


    ##########################################
    # Compute the dynamics coefficient at a given point s
    # by interpolating along the sample trajectory
    def dynamics_coefficients(self,s):
        t_vect=self.sample_t_vect
        n_steps=self.sample_n_steps
        a_vect=self.sample_a_vect
        b_vect=self.sample_b_vect
        c_vect=self.sample_c_vect
        a_threshold=self.tunings['a_threshold']
        if s<t_vect[0]: s=t_vect[0]+1e-5
        if s>t_vect[n_steps-1]: s=t_vect[n_steps-1]-1e-5
        i=bisect.bisect_left(t_vect,s)
        if i>=n_steps-1: 
            j=n_steps-1
            return [a_vect[:,j],b_vect[:,j],c_vect[:,j]]
        r=(s-t_vect[i])/(t_vect[i+1]-t_vect[i])        
        # if min(abs(a_vect[:,i]))<a_threshold or min(abs(a_vect[:,i+1]))<a_threshold:
        #     q=(1-r)*self.sample_q_vect[:,i]+r*self.sample_q_vect[:,i+1]
        #     qd=(1-r)*self.sample_qd_vect[:,i]+r*self.sample_qd_vect[:,i+1]
        #     qdd=(1-r)*self.sample_qdd_vect[:,i]+r*self.sample_qdd_vect[:,i+1]
        #     dynamics=Dynamics.RobotDynamics(self.params,q,self.grav)
        #     a=dynamics.inertiaTerm(qd)
        #     b=dynamics.inertiaTerm(qdd)+dynamics.coriolisTerm(qd)
        #     c=dynamics.gravTerm()
        #     return [a,b,c]
        a=(1-r)*a_vect[:,i]+r*a_vect[:,i+1]
        b=(1-r)*b_vect[:,i]+r*b_vect[:,i+1]
        c=(1-r)*c_vect[:,i]+r*c_vect[:,i+1]
        return [a,b,c]


    ##########################################
    # Compute the max velocity at a given point s
    def max_velocity(self,s):
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


    ##########################################
    # Compute the acceleration limits at a point (s,sdot)
    def compute_limits(self,s,sdot):
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


    ##########################################
    # Compute the acceleration limits at a point (s,sdot)
    def compute_acc(self,s):
        sdot=self.max_velocity(s)
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
        


    ##########################################
    # Integrate a trajectory forward
    def integrate_forward(self,s_start,sdot_start,s_traj_list_bw,sdot_traj_list_bw,width=1e15,acc=-1e10):

        tunings=self.tunings
        tolerance=tunings['tolerance']
        dt=tunings['t_step_integrate']

        s_curr=s_start
        sdot_curr=sdot_start
        s_res=[]
        sdot_res=[]
        n_list=len(s_traj_list_bw)
        cont=True
        start=0
        while cont:
            if len(s_res)>width: break
            if sdot_curr<0: break
            if s_curr>self.sample_duration: break
            if sdot_curr>self.max_velocity(s_curr)+tolerance: break
            if start<tunings['palier'] and acc>-1e5:
                beta=acc                
            else:
                [alpha,beta,ialpha,ibeta]=self.compute_limits(s_curr,sdot_curr)
            start+=1
            s_res.append(s_curr)
            sdot_res.append(sdot_curr)
            sdot_next=sdot_curr+beta*dt
            s_next=s_curr+sdot_curr*dt
            s_curr=s_next
            sdot_curr=sdot_next
            for j in range(n_list-1,-1,-1):
                s_traj_bw=s_traj_list_bw[j]
                sdot_traj_bw=sdot_traj_list_bw[j]
                if self.is_above(s_curr,sdot_curr,s_traj_bw,sdot_traj_bw):
                    s_res.append(s_curr)
                    sdot_res.append(sdot_curr)
                    cont=False  # Stop when crosses a backward trajectory
                    break
 
        return(array(s_res),array(sdot_res))


    ##########################################
    # Integrate a trajectory backward
    def integrate_backward(self,s_start,sdot_start,s_traj_list_fw,sdot_traj_list_fw,width=1e15,acc=-1e10):

        tunings=self.tunings
        tolerance=tunings['tolerance']
        dt=tunings['t_step_integrate']

        s_curr=s_start
        sdot_curr=sdot_start
        s_res=[]
        sdot_res=[]
        n_list=len(s_traj_list_fw)
        start=0
        cont=True
        while cont:
            if len(s_res)>width: break
            if sdot_curr<0: break
            if s_curr<0: break
            if sdot_curr>self.max_velocity(s_curr)+tolerance: break
            if start<tunings['palier'] and acc>1e-5:
                alpha=acc
            else:
                [alpha,beta,ialpha,ibeta]=self.compute_limits(s_curr,sdot_curr)
            start+=1
            s_res.append(s_curr)
            sdot_res.append(sdot_curr)
            sdot_next=sdot_curr-alpha*dt
            s_next=s_curr-sdot_curr*dt
            s_curr=s_next
            sdot_curr=sdot_next
            for j in range(n_list-1,-1,-1):
                s_traj_fw=s_traj_list_fw[j]
                sdot_traj_fw=sdot_traj_list_fw[j]
                if self.is_above(s_curr,sdot_curr,s_traj_fw,sdot_traj_fw):
                    s_res.append(s_curr)
                    sdot_res.append(sdot_curr)
                    cont=False # Stop when crosses a forward trajectory
                    break
        return(array(s_res)[::-1],array(sdot_res)[::-1])


    ##########################################
    # Test whether the point (s,sdot) is above the trajectory (s_traj,sdot_traj)
    def is_above(self,s,sdot,s_traj,sdot_traj):
        if len(s_traj)<1:
            return False
        if s<=s_traj[0] or s>=s_traj[-1]:
            return False
        else:
            i=bisect.bisect_left(s_traj,s)
            sa=s_traj[i-1]
            sb=s_traj[i]
            sdota=sdot_traj[i-1]
            sdotb=sdot_traj[i]
            r=(s-sa)/(sb-sa)
            return sdot>(1-r)*sdota+r*sdotb


    ##########################################
    # Test whether the point (s,sdot) is above the a list of trajectories
    def is_underneath_list(self,s,sdot,s_traj_list,sdot_traj_list):
        for i in range(len(s_traj_list)-1,-1,-1):
            s_traj=s_traj_list[i]
            sdot_traj=sdot_traj_list[i]
            if self.is_above(s,sdot,s_traj,sdot_traj):
                return False
        return True
            

    ##########################################
    # Find sdot
    def find_sdot(self,s,s_traj,sdot_traj):
        n=len(s_traj)
        if n==0: return 1e15
        if s==s_traj[0]: return sdot_traj[0]
        if s==s_traj[n-1]: return sdot_traj[n-1]
        if s<s_traj[0] or s>s_traj[n-1]: return 1e15
        for i in range(1,n):
            if s_traj[i]>s:
                r=(s-s_traj[i-1])/(s_traj[i]-s_traj[i-1])
                return sdot_traj[i-1]+r*(s_traj[i]-s_traj[i-1])


    ##########################################
    # Compute the index of the curve which is on top
    def compute_index(self,s,s_traj_list,sdot_traj_list):

        sdot_min=1e15
        index_min=0
        for j in range(len(s_traj_list)):
            sdot=self.find_sdot(s,s_traj_list[j],sdot_traj_list[j])
            if sdot<sdot_min:
                sdot_min=sdot
                index_min=j
        return [index_min,sdot_min]



#############################################################################
# Plotting stuffs

    def plot_limiting_curves(self):
        clf()
        hold('on')
        T=self.sample_duration
        plot(self.sample_t_vect,self.sample_max_curve,'k')
        plot([0,T],array([1,1]),'m:')
        for i in range(len(self.s_list)):
            if self.type_list[i]=='t':
                plot(self.s_list[i],self.sdot_list[i],'ro')
            if self.type_list[i]=='d':
                plot(self.s_list[i],self.sdot_list[i],'bo')
            if self.type_list[i]=='z':
                plot(self.s_list[i],self.sdot_list[i],'go')

        for i in range(len(self.s_traj_list)):
            plot(self.s_traj_list[i],self.sdot_traj_list[i],'r')

        plot(self.s_res,self.sdot_res,'k--')
        axis([0,T,0,max(self.sample_max_curve)])
        grid('on')
        show()       


    def plot_arrows(self,si,sdoti,ns,nsdot,dt,dtdot):
        i=1
        sv=linspace(si-ns*dt,si+ns*dt,2*ns+1)
        sdotv=linspace(sdoti-nsdot*dtdot,sdoti+nsdot*dtdot,2*nsdot+1)
        self.plot_limiting_curves()
        dtf=dt*0.7
        for s in sv:
            for sdot in sdotv:
                [alpha,beta,ialpha,ibeta]=self.compute_limits(s,sdot)
                plot([s,s+dtf],[sdot,sdot+dtf*beta/sdot],'r')
                plot([s,s+dtf],[sdot,sdot+dtf*alpha/sdot],'b')
                plot([s],[sdot],'k*')


    def compute_flow(self,s_curr,sdot_curr,dt,n_int,alphabeta):
        s_res=[]
        sdot_res=[]
        for i in range(n_int):
            if sdot_curr<0: break
            if s_curr>self.sample_duration: break
            if sdot_curr>self.max_velocity(s_curr): break
            [alpha,beta,ialpha,ibeta]=self.compute_limits(s_curr,sdot_curr)
            s_res.append(s_curr)
            sdot_res.append(sdot_curr)
            if alphabeta=='alpha':
                sdot_next=sdot_curr+alpha*dt
            else:                
                sdot_next=sdot_curr+beta*dt               
            s_next=s_curr+sdot_curr*dt
            s_curr=s_next
            sdot_curr=sdot_next

        return [s_res,sdot_res]









