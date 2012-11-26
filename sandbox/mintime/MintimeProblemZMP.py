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
humanoid robot under ZMP constraints
Cf. Pham and Nakamura, Humanoids 2012, http://www.normalesup.org/~pham/docs/zmp.pdf
"""



from openravepy import *
import MintimeProblemGeneric
import ZMP
from numpy import *
from pylab import *
import bisect


class MintimeProblemZMP(MintimeProblemGeneric.MintimeProblemGeneric):



############################# Initialization ############################


    def __init__(self,robot,traj):        
        MintimeProblemGeneric.MintimeProblemGeneric.__init__(self,robot,traj)


    def set_zmp_limits(self,bounds):
        self.xmin=bounds[0]
        self.xmax=bounds[1]
        self.ymin=bounds[2]
        self.ymax=bounds[3]        
        self.isset_accel_limits=True




################################ Dynamics ################################


    def sample_dynamics(self):
        """Sample the dynamics coefficients along the trajectory"""
        self.ax_vect=zeros(self.n_steps)
        self.bx_vect=zeros(self.n_steps)
        self.cx_vect=zeros(self.n_steps)
        self.ay_vect=zeros(self.n_steps)
        self.by_vect=zeros(self.n_steps)
        self.cy_vect=zeros(self.n_steps)
        self.d_vect=zeros(self.n_steps)
        self.e_vect=zeros(self.n_steps)
        self.f_vect=zeros(self.n_steps)

        for i in range(self.n_steps):
            q=self.q_vect[:,i]
            qd=self.qd_vect[:,i]
            qdd=self.qdd_vect[:,i]

            # Here we assume there is no baselink rotation
            [ax,bx,cx,ay,by,cy,d,e,f]=ZMP.ComputeCoefsFractionZMP([q[0:3],qd[0:3],qdd[0:3],q[6:len(q)],qd[6:len(q)],qdd[6:len(q)]],self.zmp_params)

            self.ax_vect[i]=ax
            self.bx_vect[i]=bx
            self.cx_vect[i]=cx
            self.ay_vect[i]=ay
            self.by_vect[i]=by
            self.cy_vect[i]=cy
            self.d_vect[i]=d
            self.e_vect[i]=e
            self.f_vect[i]=f


    def dynamics_coefficients(self,s):
        """Compute the dynamics coefficients at a given point by interpolation

        s -- point on the trajectory

        """
        return self.linear_interpolate(s,transpose(array([self.ax_vect,self.bx_vect,self.cx_vect,self.ay_vect,self.by_vect,self.cy_vect,self.d_vect,self.e_vect,self.f_vect])))



############################ Accel limits ################################


    def alpha_beta(self,s):
        """"Compute the lists A and B, cf Pham Asian-MMS"""
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


    def accel_limits(self,s,sdot):
        """Compute the acceleration limits caused by ZMP constraints

        (s,sdot) -- point of the phase plane

        """
        [alpha_list,beta_list]=self.alpha_beta(s)
        alpha=-1e15
        beta=1e15
        for [Aalpha,Balpha] in alpha_list:
            alpha=max(alpha,Aalpha*sdot*sdot+Balpha)
        for [Abeta,Bbeta] in beta_list:
             beta=min(beta,Abeta*sdot*sdot+Bbeta)

        return [alpha,beta,0,0]


    def maxvel_accel(self,s):
        """Compute the maximum velocity caused by ZMP constraints
        
        s -- point on the trajectory
        
        """
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

        return(sdotmax)



########################## Zero-inertia points ############################


    def find_zero_inertia_points(self):
        """Find all zero-inertia points and assign to the list self.sw_zero_inertia"""
        xmin=self.xmin
        xmax=self.xmax
        ymin=self.ymin
        ymax=self.ymax
        if self.n_steps<1:
            self.sw_zero_inertia=[]
            return
        s=self.t_vect[0]
        [axp,bxp,cxp,ayp,byp,cyp,dp,ep,fp]=self.dynamics_coefficients(s)
        denomp=[axp-xmin*dp,axp-xmax*dp,ayp-ymin*dp,ayp-ymax*dp]
        i_list=[]
        for i in range(1,self.n_steps-1):
            s=self.t_vect[i]
            [ax,bx,cx,ay,by,cy,d,e,f]=self.dynamics_coefficients(s)
            denom=[ax-xmin*d,ax-xmax*d,ay-ymin*d,ay-ymax*d]
            for j in range(4):
                if denom[j]*denomp[j]<0:
                    if abs(denom[j])<abs(denomp[j]):
                        i_list.append(i)
                    else:
                        i_list.append(i-1)
            denomp=list(denom)
       
        self.sw_zero_inertia=i_list


    def correct_accel_zi(self,s):
        """Compute the correct acceleration at a zero-inertia point

        s -- a zero-inertia point on the trajectory

        """
        return 0

