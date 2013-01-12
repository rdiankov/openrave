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
Core algorithm for time-optimal path parameterization under second-order limits
Based on Bobrow, Dubowsky, Johanni, Pfeiffer, Slotine, Shiller, Pham, etc.
Cf. Pham, Asian-MMS 2012, http://www.normalesup.org/~pham/docs/shortcuts.pdf
"""



from numpy import *
from pylab import *
import bisect
import MintimeProblemGeneric





class MintimeProfileIntegrator():
    

############################## Initialization ################################

    def __init__(self,pb):
        self.pb=pb
        

####################### Integrate the limiting curves #########################

    def compute_limiting_curves(self):

        sdot_init=self.sdot_init
        sdot_final=self.sdot_final
        dt_integ=self.dt_integ
        width=self.width

        s_traj_list=[]
        sdot_traj_list=[]
        cat_list=[]
        duration=self.pb.traj.duration

        self.sws=list(self.pb.sw_s_list)
        self.swsd=list(self.pb.sw_sdot_list)
        self.swt=list(self.pb.sw_type_list)


        #Integrate backward from the end
        [s_final_backward,sdot_final_backward]=self.integrate_backward(duration,sdot_final,[],[])
        s_traj_list.append(s_final_backward)
        sdot_traj_list.append(sdot_final_backward)
        cat_list.append('alpha')


        #Integrate forward from the start
        [s_forward,sdot_forward]=self.integrate_forward(0,sdot_init,[s_final_backward],[sdot_final_backward])
        s_traj_list.append(s_forward)
        sdot_traj_list.append(sdot_forward)
        cat_list.append('beta')

        if len(s_final_backward)==0 or len(s_forward)==0:
            self.possible=False
            return

        s_target= s_final_backward[0]
        s_farthest=s_forward[-1]

        #Integrate backward and forward from switching points
        while True:
            if s_farthest>s_target: break
            if s_farthest>duration: break
            if len(self.sws)==0: break
            si=self.sws.pop(0)
            sdoti=self.swsd.pop(0)
            typei=self.swt.pop(0)

            if si<s_farthest: continue

            if  typei=='z' or  typei=='t' or typei=='v':
                sdoti=self.find_sdot_min(si,sdoti)
            if typei=='z':
                acc_sw=self.pb.correct_accel_zi(si)
            else:
                acc_sw=nan

            #Backward part
            [s_backward,sdot_backward]=self.integrate_backward(si,sdoti,s_traj_list,sdot_traj_list,acc=acc_sw)

            if s_backward[0]>s_farthest: continue

            #Forward part            
            [s_forward,sdot_forward]=self.integrate_forward(si,sdoti,[s_final_backward],[sdot_final_backward],acc=acc_sw)
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




########################### Integrate forward #############################

    def integrate_forward(self,s_start,sdot_start,s_traj_list_bw,sdot_traj_list_bw,width=1e15,acc=nan):

        dt_integ=self.dt_integ

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
            if s_curr>self.pb.duration: break
            # If sdot_cur > combined max sdot curve
            if sdot_curr>self.pb.maxvel_interp(s_curr):
                if(not self.pb.isset_velocity_limits):
                    break
                # If sdot_cur < sdot_vel, it means that sdot_cur > sdot_acc
                if sdot_curr<self.pb.maxvel_velocity_interp(s_curr):
                    break
                # Else: sdot_vel < sdot_cur < sdot_acc
                else:
                    cont2=True
                    while cont2:
                        # Last step before going above the velocity limit curve
                        if len(s_res)>=1:
                            s_curr=s_res[-1]
                            sdot_curr=sdot_res[-1]
                        
                        [alpha,beta,ialpha,ibeta]=self.pb.accel_limits(s_curr,sdot_curr)
                        s_next=s_curr+sdot_curr*dt_integ
                        sdot_next_vel=self.pb.maxvel_velocity_interp(s_next)

                        # If vel max curve comprised between min and max acc
                        # then follows vel max curve
                        if sdot_next_vel<sdot_curr+beta*dt_integ and sdot_next_vel>sdot_curr+alpha*dt_integ and s_curr<self.pb.duration:
                            s_curr=s_next
                            sdot_curr=sdot_next_vel                            
                            s_res.append(s_curr)
                            sdot_res.append(sdot_curr)
                        # If vel max curve above max acc
                        # then follows max acc (going downwards)
                        elif sdot_next_vel > sdot_curr+beta*dt_integ:
                            s_curr=s_next
                            sdot_curr=sdot_curr+beta*dt_integ
                            s_res.append(s_curr)
                            sdot_res.append(sdot_curr)
                            # Come back to the main loop
                            cont2=False
                            break
                        # Here the vel max curve is below min acc
                        # In this case, go along the max vel curve
                        # until the min acc becomes smaller than the max vel curve
                        # and add this point to the switch point list
                        # This is taken from the Zlajpah ICRA 1996 paper
                        else:
                            while s_curr+sdot_curr*dt_integ<self.pb.duration:
                                print 'Zlajpah'
                                s_next=s_curr+sdot_curr*dt_integ
                                sdot_next_vel=self.pb.maxvel_velocity_interp(s_next)
                                [alpha,beta,ialpha,ibeta]=self.pb.accel_limits(s_curr,sdot_curr)
                                if sdot_next_vel>sdot_curr+alpha*dt_integ:
                                    self.sws.insert(0,s_curr)
                                    self.swsd.insert(0,sdot_curr)
                                    self.swt.insert(0,'v')
                                    #print self.sws
                                    cont2=False
                                    cont=False
                                    break
                                s_curr=s_next
                                sdot_curr=sdot_next_vel
                            cont2=False
                            cont=False
                            
                             
                            
            # Here sdot_cur < combined max sdot curve, so integrate the max acc
            else:                    
                if start<self.palier and not isnan(acc):
                    beta=acc                
                else:
                    [alpha,beta,ialpha,ibeta]=self.pb.accel_limits(s_curr,sdot_curr)
                start+=1
                s_res.append(s_curr)
                sdot_res.append(sdot_curr)
                sdot_next=sdot_curr+beta*dt_integ
                s_next=s_curr+sdot_curr*dt_integ
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




########################### Integrate backward ############################

    def integrate_backward(self,s_start,sdot_start,s_traj_list_fw,sdot_traj_list_fw,width=1e15,acc=nan):

        dt_integ=self.dt_integ

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
            if sdot_curr>self.pb.maxvel_interp(s_curr): break
            if start<self.palier and not isnan(acc):
                alpha=acc
            else:
                [alpha,beta,ialpha,ibeta]=self.pb.accel_limits(s_curr,sdot_curr)
            start+=1
            s_res.append(s_curr)
            sdot_res.append(sdot_curr)
            sdot_next=sdot_curr-alpha*dt_integ
            s_next=s_curr-sdot_curr*dt_integ
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




    
#################### Integrate the final velocity profile ######################

    def integrate_final(self):

        s_traj_list=self.s_traj_list
        sdot_traj_list=self.sdot_traj_list
        dt_integ=self.dt_integ

        s_curr=0
        s_res=[]
        sdot_res=[]
        while True:
            if s_curr>self.pb.duration: break
            s_res.append(s_curr)
            [index_min,sdot_min]=self.compute_index(s_curr,s_traj_list,sdot_traj_list)
            s_curr=s_curr+sdot_min*dt_integ
            if sdot_min<1e-5: break
            sdot_res.append(sdot_min)

        self.s_res=array(s_res)
        self.sdot_res=array(sdot_res)
        if len(s_res)==0 or s_res[-1]<self.pb.duration-self.threshold_final:
            self.possible=False



#################################### Utilities #################################


    # Find the highest sdot that allows passing through a switching point
    def find_sdot_min(self,s,sdot):

        dt_integ=self.dt_integ
        width=self.width

        bound_top=sdot
        bound_bottom=0.001
        min_sep=0.01
        if bound_top<bound_bottom: return 0
        sdot=bound_top
        [s_forward,sdot_forward]=self.integrate_forward(s,sdot,[],[],width,acc=self.pb.correct_accel_zi(s))
        [s_backward,sdot_backward]=self.integrate_backward(s,sdot,[],[],width,acc=self.pb.correct_accel_zi(s))
        if (len(s_forward)>=width or (len(s_forward)>0 and s_forward[-1]>self.pb.duration)) and (len(s_backward)>=width or (len(s_backward)>0 and s_backward[0]<-1)):
            return bound_top
        sdot=bound_bottom
        [s_forward,sdot_forward]=self.integrate_forward(s,sdot,[],[],width,acc=self.pb.correct_accel_zi(s))
        [s_backward,sdot_backward]=self.integrate_backward(s,sdot,[],[],width,acc=self.pb.correct_accel_zi(s))
        if not ((len(s_forward)>=width or (len(s_forward)>0 and s_forward[-1]>self.pb.duration)) and (len(s_backward)>=width or (len(s_backward)>0 and s_backward[0]<-1))):
            return 0
        while bound_top-bound_bottom>min_sep:
            sdot=(bound_top+bound_bottom)/2
            [s_forward,sdot_forward]=self.integrate_forward(s,sdot,[],[],width,acc=self.pb.correct_accel_zi(s))
            [s_backward,sdot_backward]=self.integrate_backward(s,sdot,[],[],width,acc=self.pb.correct_accel_zi(s))
            if (len(s_forward)>=width or (len(s_forward)>0 and s_forward[-1]>self.pb.duration)) and (len(s_backward)>=width or (len(s_backward)>0 and s_backward[0]<-1)):
                bound_bottom=sdot
            else:
                bound_top=sdot
        return bound_bottom


    # Test whether the point (s,sdot) is above the trajectory (s_traj,sdot_traj)
    def is_above(self,s,sdot,s_traj,sdot_traj):
        if len(s_traj)<1:
            return False
        if s<=s_traj[0] or s>=s_traj[-1]:
            return False
        else:
            return sdot>self.pb.linear_interpolate(s,sdot_traj,s_traj)


    # Test whether the point (s,sdot) is above the a list of trajectories
    def is_underneath_list(self,s,sdot,s_traj_list,sdot_traj_list):
        for i in range(len(s_traj_list)-1,-1,-1):
            s_traj=s_traj_list[i]
            sdot_traj=sdot_traj_list[i]
            if self.is_above(s,sdot,s_traj,sdot_traj):
                return False
        return True
            

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


    # Plot the limiting curves and the final velocity profile
    def plot_limiting_curves(self):
        clf()
        hold('on')
        T=self.pb.duration
        self.pb.plot_maxvel_curves()

        for i in range(len(self.s_traj_list)):
            plot(self.s_traj_list[i],self.sdot_traj_list[i],'k',linewidth=2)

        plot(self.s_res,self.sdot_res,'b--',linewidth=4)
        axis([0,T,0,1.1*max(self.pb.maxvel_curve)])
        grid('on')
