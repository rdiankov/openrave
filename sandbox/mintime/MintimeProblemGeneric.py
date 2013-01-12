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
General class of time-optimal path parameterization problems
All new problem classes must be derived from this class
For examples, see MintimeProblemManip.py and MintimeProblemZMP.py
"""


from numpy import *
from pylab import *
import bisect



class MintimeProblemGeneric():

    def __init__(self,robot,traj):
        self.robot=robot
        self.traj=traj
        self.dim=traj.dim
        self.t_step=traj.t_step
        self.n_steps=traj.n_steps
        self.duration=traj.duration
        self.t_vect=traj.t_vect
        self.q_vect=traj.q_vect
        self.qd_vect=traj.qd_vect
        self.qdd_vect=traj.qdd_vect
        self.isset_dynamics_limits=False
        self.isset_velocity_limits=False
       

    def preprocess(self):
        """Preprocess, must be called before running the ProfileIntegrator"""
 
       # Sample the dynamics
        self.sample_dynamics()

        # Compute the max velocity curve caused by accelerations limits
        if(self.isset_dynamics_limits):
            self.compute_maxvel_accel_curve()
            self.maxvel_curve=array(self.maxvel_accel_curve)
        else:
            raise NameError('Second order (torques, zmp,...) limits are required')

        # Compute the max velocity curve caused by velocity limits
        if(self.isset_velocity_limits):
            self.compute_maxvel_velocity_curve()
            for i in range(self.n_steps):
                self.maxvel_curve[i]=min(self.maxvel_accel_curve[i],self.maxvel_velocity_curve[i])


        # Compute the switch points
        self.find_tangent_disc_points()
        self.find_zero_inertia_points()
        self.merge_switch_points_lists()
        





############################ Velocity limits ##############################

    def set_velocity_limits(self,limits):
        self.qd_max=limits
        self.isset_velocity_limits=True


    def compute_maxvel_velocity_curve(self):
        """Compute the max velocity curve caused by velocity limits"""
        qd_max=self.qd_max        
        n_steps=self.n_steps
        self.maxvel_velocity_curve=ones(n_steps)*1e5
        for i in range(n_steps):
            qd=self.qd_vect[:,i]
            for j in range(self.dim):
                self.maxvel_velocity_curve[i]=min(self.maxvel_velocity_curve[i],qd_max[j]/abs(qd[j]))


    def compute_maxvel_accel_curve(self):
        """Compute the max velocity curve caused by torque limits"""
        self.maxvel_accel_curve=zeros(self.n_steps)
        for i in range(self.n_steps):
            self.maxvel_accel_curve[i]=self.maxvel_accel(self.t_vect[i])


    def maxvel_velocity_interp(self,s):
        """Compute the max velocity caused by velocity limits

        s -- point on the trajectory

        """
        return self.linear_interpolate(s,self.maxvel_velocity_curve)


    def maxvel_accel_interp(self,s):
        """Compute the max velocity caused by accel limits

        s -- point on the trajectory

        """
        return self.linear_interpolate(s,self.maxvel_accel_curve)


    def maxvel_interp(self,s):
        """Compute the overall max velocity limits

        s -- point on the trajectory

        """
        return self.linear_interpolate(s,self.maxvel_curve)



############################ Prototypes ##############################


    def set_dynamics_limits(self,limits):
        """Set dynamics limits"""
        raise NameError('Some virtual methods need be implemented')


    def sample_dynamics(self):
        """Sample the dynamics coefficients along the trajectory"""
        raise NameError('Some virtual methods need be implemented')


    def dynamics_coefficients(self,s):
        """Compute the dynamics coefficients at a given point by interpolation

        s -- point on the trajectory

        """
        raise NameError('Some virtual methods need be implemented')


    def accel_limits(self,s,sdot):
        """Compute the acceleration limits caused by torque limits 

        (s,sdot) -- point of the phase plane

        """
        raise NameError('Some virtual methods need be implemented')


    def maxvel_accel(self,s):
        """Compute the maximum velocity caused by torque limits 
        
        s -- point on the trajectory
        
        """
        raise NameError('Some virtual methods need be implemented')


    def find_zero_inertia_points(self):
        """Find all zero-inertia points and assign to the list self.sw_zero_inertia"""
        raise NameError('Some virtual methods need be implemented')


    def correct_accel_zi(self,s):
        """Compute the correct acceleration at a zero-inertia point

        s -- a zero-inertia point on the trajectory

        """
        raise NameError('Some virtual methods need be implemented')
      



############################ Switch points ##############################

# The search for tangent and discontinuity points is the same for everyone
# The search for zero-inertia points depends on the dynamics, so this is
# computed in the children classes


    def find_tangent_disc_points(self):
        """Find all tangent and discontinuity points and assign to the list self.sw_tangent_disc"""
        if self.n_steps<3:
            self.sw_tangent_disc=[]
            return
        maxvel_curve=self.maxvel_curve
        i=1
        s=self.t_vect[i]
        sdot=maxvel_curve[i]
        [alpha,beta,k_alpha,k_beta]=self.accel_limits(s,sdot)
        diffp=alpha/sdot-(maxvel_curve[i+1]-maxvel_curve[i])/self.t_step
        i_list=[]
        for i in range(1,self.n_steps-1):
            # Test whether i is a discontinuity points
            if abs(maxvel_curve[i+1]-maxvel_curve[i])>self.disc_thr:
                if maxvel_curve[i+1]>maxvel_curve[i]:
                    i_list.append(i)
                else:
                    i_list.append(i+1)
            # Test whether i is a tangent points
            s=self.t_vect[i]
            sdot=maxvel_curve[i]
            [alpha,beta,k_alpha,k_beta]=self.accel_limits(s,sdot)
            diff=alpha/sdot-(maxvel_curve[i+1]-maxvel_curve[i])/self.t_step
            if diff*diffp<0:
                if i>2 and i<self.n_steps-3:
                    # A switch point cannot be a local maximum
                    if maxvel_curve[i-2]>sdot or maxvel_curve[i+2]>sdot:
                        if maxvel_curve[i]<maxvel_curve[i-1]:
                            i_list.append(i)
                        else:
                            i_list.append(i-1)
                else:
                    if maxvel_curve[i]<maxvel_curve[i-1]:
                        i_list.append(i)
                    else:
                        i_list.append(i-1)
            diffp=diff

        self.sw_tangent_disc=i_list


    def merge_switch_points_lists(self):
        """Find all switch points and assign to the list self.sw_s_list
        by merging the zero-inertia list and tangent-disc list

        """

        i_list=self.sw_zero_inertia # Zero-inertia points 'z'
        i_list2=self.sw_tangent_disc # Tangent or discontinuity points 't'

        # Merge the lists
        type_list=['z']*len(i_list)
        for x in i_list2:
            if not (x in i_list):
                index=bisect.bisect_left(i_list,x)
                i_list.insert(index,x)
                type_list.insert(index,'t')


        # Find the corresponding values s and sdot
        s_list=[]
        sdot_list=[]
        for j in range(len(i_list)):
            s_list.append(self.t_vect[i_list[j]])
            sdot=self.maxvel_curve[i_list[j]]
            sdot_list.append(sdot)

        self.sw_i_list=i_list
        self.sw_type_list=type_list
        self.sw_s_list=s_list
        self.sw_sdot_list=sdot_list




############################ Interpolation ##############################


    def linear_interpolate(self,s,value_vect,t_vect=None,elim_out=False):
        if t_vect==None:
            t_vect=self.t_vect
            n_steps=self.n_steps
        else:
            n_steps=len(t_vect)
        
        if n_steps==0:
            return 1e15

        if s<t_vect[0]: 
            if elim_out:
                return 1e15
            else:
                s=t_vect[0]+1e-5
        if s>t_vect[n_steps-1]: 
            if elim_out:
                return 1e15
            else:
                s=t_vect[n_steps-1]-1e-5
        i=bisect.bisect_left(t_vect,s)
        if i==0: 
            return value_vect[i]
        r=(s-t_vect[i-1])/(t_vect[i]-t_vect[i-1])

        return (1-r)*value_vect[i-1]+r*value_vect[i] 


    def linear_interpolate_multi(self,s,value_vect_list,t_vect=None):
        if t_vect==None:
            t_vect=self.t_vect
            n_steps=self.n_steps
        else:
            n_steps=len(t_vect)
        if s<t_vect[0]: s=t_vect[0]+1e-5
        if s>t_vect[n_steps-1]: s=t_vect[n_steps-1]-1e-5
        i=bisect.bisect_left(t_vect,s)
        if i==0: 
            return [k[:,i] for k in value_vect_list]
        r=(s-t_vect[i-1])/(t_vect[i]-t_vect[i-1])
        return [(1-r)*k[:,i-1]+r*k[:,i] for k in value_vect_list]
   


########################## Plotting ############################

    def plot_maxvel_curves(self,h_offset=0):
        plot(self.t_vect+h_offset,self.maxvel_curve,'c',linewidth=3)
        plot(self.t_vect+h_offset,self.maxvel_accel_curve,'r')
        if(self.isset_velocity_limits):
            plot(self.t_vect+h_offset,self.maxvel_velocity_curve,'g')
        plot([h_offset,h_offset],[0,1e2],'r--')
        for i in range(len(self.sw_s_list)):
            if self.sw_type_list[i]=='t':
                plot(self.sw_s_list[i]+h_offset,self.sw_sdot_list[i],'ro')
            if self.sw_type_list[i]=='z':
                plot(self.sw_s_list[i]+h_offset,self.sw_sdot_list[i],'go')

