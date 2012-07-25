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
import ZMP


class RobotMinimumTime():

    ##########################################
    def __init__(self,robot,sample_traj,bounds,tunings,params):

        # Set the robot parameters
        self.robot=robot
        self.env=robot.GetEnv()
        self.params=params
        self.tunings=tunings
        self.xmin=bounds[0]
        self.xmax=bounds[1]
        self.ymin=bounds[2]
        self.ymax=bounds[3]

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

        # Run functions
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
        ax_vect=zeros(self.sample_n_steps)
        bx_vect=zeros(self.sample_n_steps)
        cx_vect=zeros(self.sample_n_steps)
        ay_vect=zeros(self.sample_n_steps)
        by_vect=zeros(self.sample_n_steps)
        cy_vect=zeros(self.sample_n_steps)
        d_vect=zeros(self.sample_n_steps)
        e_vect=zeros(self.sample_n_steps)
        f_vect=zeros(self.sample_n_steps)
        self.env.GetPhysicsEngine().SetGravity([0,0,-self.params['gravity']])

        for i in range(self.sample_n_steps):
            q=self.sample_q_vect[:,i]
            qd=self.sample_qd_vect[:,i]
            qdd=self.sample_qdd_vect[:,i]

            # Here we assume there is no baselink rotation
            [ax,bx,cx,ay,by,cy,d,e,f]=ZMP.ComputeCoefsFractionZMP([q[0:3],qd[0:3],qdd[0:3],q[6:len(q)],qd[6:len(q)],qdd[6:len(q)]],self.params)

            ax_vect[i]=ax
            bx_vect[i]=bx
            cx_vect[i]=cx
            ay_vect[i]=ay
            by_vect[i]=by
            cy_vect[i]=cy
            d_vect[i]=d
            e_vect[i]=e
            f_vect[i]=f

        self.sample_ax_vect=ax_vect
        self.sample_bx_vect=bx_vect
        self.sample_cx_vect=cx_vect   
        self.sample_ay_vect=ay_vect
        self.sample_by_vect=by_vect
        self.sample_cy_vect=cy_vect   
        self.sample_d_vect=d_vect
        self.sample_e_vect=e_vect
        self.sample_f_vect=f_vect   


    ##########################################
    # Compute the max velocity curve
    def sample_max_velocity_curve(self):
        max_curve=zeros(self.sample_n_steps)
        min_curve=zeros(self.sample_n_steps)
        for i in range(self.sample_n_steps):
            [sdotmin,sdotmax]=self.max_velocity(self.sample_t_vect[i])
            min_curve[i]=sdotmin
            max_curve[i]=sdotmax
        self.sample_min_curve=min_curve
        self.sample_max_curve=max_curve


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
        i_list2=self.find_discontinuous_points(sdot_threshold)
        # Find the zero inertia points        
        i_list3=self.find_zero_inertia_points(a_threshold)
        #i_list3=[]
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
                print si
                print sdoti

            #Backward part            
            [s_backward,sdot_backward]=self.integrate_backward(si,sdoti,s_traj_list,sdot_traj_list)
            if s_backward[0]>s_farthest: continue

            #Forward part            
            [s_forward,sdot_forward]=self.integrate_forward(si,sdoti,[s_final_backward],[sdot_final_backward])
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
        xmin=self.xmin
        xmax=self.xmax
        ymin=self.ymin
        ymax=self.ymax
        i_list=[]
        for i in range(1,self.sample_n_steps-2):
            s=self.sample_t_vect[i]
            sp=self.sample_t_vect[i-1]
            sn=self.sample_t_vect[i+1]
            [ax,bx,cx,ay,by,cy,d,e,f]=self.dynamics_coefficients(s)
            [axp,bxp,cxp,ayp,byp,cyp,dp,ep,fp]=self.dynamics_coefficients(sp)
            [axn,bxn,cxn,ayn,byn,cyn,dn,en,fn]=self.dynamics_coefficients(sn)
            if abs(ax-xmin*d)<a_threshold and abs(ax-xmin*d)<abs(axp-xmin*dp) and abs(ax-xmin*d)<abs(axn-xmin*dn):
                i_list.append(i)
            if abs(ax-xmax*d)<a_threshold and abs(ax-xmax*d)<abs(axp-xmax*dp) and abs(ax-xmax*d)<abs(axn-xmax*dn):
                i_list.append(i)
            if abs(ay-ymin*d)<a_threshold and abs(ay-ymin*d)<abs(ayp-ymin*dp) and abs(ay-ymin*d)<abs(ayn-ymin*dn):
                i_list.append(i)
            if abs(ay-ymax*d)<a_threshold and abs(ay-ymax*d)<abs(ayp-ymax*dp) and abs(ay-ymax*d)<abs(ayn-ymax*dn):
                i_list.append(i)

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
        [s_forward,sdot_forward]=self.integrate_forward(s,sdot,[],[],width)
        [s_backward,sdot_backward]=self.integrate_backward(s,sdot,[],[],width)
        #print s_forward
        #print s_backward
        if (len(s_forward)>=width or s_forward[-1]>self.sample_duration) and (len(s_backward)>=width or s_backward[0]<-1):
            return bound_top
        sdot=bound_bottom
        [s_forward,sdot_forward]=self.integrate_forward(s,sdot,[],[],width)
        [s_backward,sdot_backward]=self.integrate_backward(s,sdot,[],[],width)
        #print s_forward
        #print s_backward
        if not ((len(s_forward)>=width or s_forward[-1]>self.sample_duration) and (len(s_backward)>=width or s_backward[0]<-1)):
            return 0
        while bound_top-bound_bottom>min_sep:
            sdot=(bound_top+bound_bottom)/2
            [s_forward,sdot_forward]=self.integrate_forward(s,sdot,[],[],width)
            [s_backward,sdot_backward]=self.integrate_backward(s,sdot,[],[],width)
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

        ax_vect=self.sample_ax_vect
        bx_vect=self.sample_bx_vect
        cx_vect=self.sample_cx_vect
        ay_vect=self.sample_ay_vect
        by_vect=self.sample_by_vect
        cy_vect=self.sample_cy_vect
        d_vect=self.sample_d_vect
        e_vect=self.sample_e_vect
        f_vect=self.sample_f_vect

        if s<t_vect[0]: s=t_vect[0]+1e-5
        if s>t_vect[n_steps-1]: s=t_vect[n_steps-1]-1e-5
        i=bisect.bisect_left(t_vect,s)
        if i>=n_steps-1: 
            j=n_steps-1
            return [ax_vect[j],bx_vect[j],cx_vect[j],ay_vect[j],by_vect[j],cy_vect[j],d_vect[j],e_vect[j],f_vect[j]]

        r=(s-t_vect[i])/(t_vect[i+1]-t_vect[i])        

        ax=(1-r)*ax_vect[i]+r*ax_vect[i+1]
        bx=(1-r)*bx_vect[i]+r*bx_vect[i+1]
        cx=(1-r)*cx_vect[i]+r*cx_vect[i+1]
        ay=(1-r)*ay_vect[i]+r*ay_vect[i+1]
        by=(1-r)*by_vect[i]+r*by_vect[i+1]
        cy=(1-r)*cy_vect[i]+r*cy_vect[i+1]
        d=(1-r)*d_vect[i]+r*d_vect[i+1]
        e=(1-r)*e_vect[i]+r*e_vect[i+1]
        f=(1-r)*f_vect[i]+r*f_vect[i+1]

        return [ax,bx,cx,ay,by,cy,d,e,f]


    # Compute the lists of A and B
    def alpha_beta(self,s):
        xmin=self.xmin
        xmax=self.xmax
        ymin=self.ymin
        ymax=self.ymax
        [ax,bx,cx,ay,by,cy,d,e,f]=self.dynamics_coefficients(s)
        
        alpha_list=[]
        beta_list=[]
    
        #xmin
        denum=ax-xmin*d
        A=(xmin*e-bx)/denum
        B=(xmin*f-cx)/denum
        if denum>0:
            alpha_list.append([A,B])
        else:
            beta_list.append([A,B])

        #xmax
        denum=ax-xmax*d
        A=(xmax*e-bx)/denum
        B=(xmax*f-cx)/denum
        if denum>0:
            beta_list.append([A,B])
        else:
            alpha_list.append([A,B])
           
        #ymin
        denum=ay-ymin*d
        A=(ymin*e-by)/denum
        B=(ymin*f-cy)/denum
        if denum>0:
            alpha_list.append([A,B])
        else:
            beta_list.append([A,B])

        #ymax
        denum=ay-ymax*d
        A=(ymax*e-by)/denum
        B=(ymax*f-cy)/denum
        if denum>0:
            beta_list.append([A,B])
        else:
            alpha_list.append([A,B])
        
        return [alpha_list,beta_list]



    ##########################################
    # Compute the max velocity at a given point s
    def max_velocity(self,s):

        [alpha_list,beta_list]=self.alpha_beta(s)
        low_bounds=[]
        up_bounds=[]

        for [Aalpha,Balpha] in alpha_list:
            for [Abeta,Bbeta] in beta_list:
                if Aalpha<Abeta:
                    r=(Bbeta-Balpha)/(Aalpha-Abeta)
                    if r>=0:
                        low_bounds.append(sqrt(r))
                else:
                    r=(Bbeta-Balpha)/(Aalpha-Abeta)
                    if r<0:
                        up_bounds.append(-1)
                    else:
                        up_bounds.append(sqrt(r))
                    
        # Conclusion
        if len(low_bounds)==0:
            sdotmin=0
        else:
            sdotmin=max(low_bounds)
        if len(up_bounds)==0:
            sdotmax=20
        else:
            sdotmax=min(up_bounds)

        return [sdotmin,sdotmax]





    ##########################################
    # Compute the acceleration limits at a point (s,sdot)
    def compute_limits(self,s,sdot):
        [alpha_list,beta_list]=self.alpha_beta(s)
        alpha=-1e15
        beta=1e15
        for [Aalpha,Balpha] in alpha_list:
            alpha=max(alpha,Aalpha*sdot*sdot+Balpha)
        for [Abeta,Bbeta] in beta_list:
             beta=min(beta,Abeta*sdot*sdot+Bbeta)

        return [alpha,beta,0,0]


    ##########################################
    # Integrate a trajectory forward
    def integrate_forward(self,s_start,sdot_start,s_traj_list_bw,sdot_traj_list_bw,width=1e15):

        tunings=self.tunings
        tolerance=tunings['tolerance']
        dt=tunings['t_step_integrate']

        s_curr=s_start
        sdot_curr=sdot_start
        s_res=[]
        sdot_res=[]
        n_list=len(s_traj_list_bw)
        cont=True
        while cont:
            if len(s_res)>width: break
            if sdot_curr<0: break
            if s_curr>self.sample_duration: break
            if sdot_curr>self.max_velocity(s_curr)[1]+tolerance: break
            [alpha,beta,ialpha,ibeta]=self.compute_limits(s_curr,sdot_curr)
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
    def integrate_backward(self,s_start,sdot_start,s_traj_list_fw,sdot_traj_list_fw,width=1e15):

        tunings=self.tunings
        tolerance=tunings['tolerance']
        dt=tunings['t_step_integrate']

        s_curr=s_start
        sdot_curr=sdot_start
        s_res=[]
        sdot_res=[]
        n_list=len(s_traj_list_fw)
        cont=True
        while cont:
            if len(s_res)>width: break
            if sdot_curr<0: break
            if s_curr<0: break
            if sdot_curr>self.max_velocity(s_curr)[1]+tolerance: break
            [alpha,beta,ialpha,ibeta]=self.compute_limits(s_curr,sdot_curr)
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


    def plot_min_max_curves(self):
        clf()
        hold('on')
        T=self.sample_duration
        plot(self.sample_t_vect,self.sample_min_curve,'g')
        plot(self.sample_t_vect,self.sample_max_curve,'r')
        axis('auto')
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


