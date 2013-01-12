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
        self.s_res=[]

        self.s_traj_list=[]
        self.sdot_traj_list=[]

        self.cur_s_traj_fw=[]
        self.cur_sdot_traj_fw=[]

        self.cur_s_traj_bw=[]
        self.cur_sdot_traj_bw=[]

        self.sws=[]
        self.swsd=[]
        self.swt=[]

        self.possible=True




####################### Integrate the profiles #########################

    def integrate_all_profiles(self):

        sdot_init=self.sdot_init
        sdot_final=self.sdot_final

        duration=self.pb.traj.duration        

        #Integrate backward from the end
        [s_backward,sdot_backward,status]=self.integrate_backward(duration,sdot_final)
        stop=False

        if len(s_backward)==0:
            self.main_status="BwTrajVoid"
            self.possible=False
            stop=True
        if status=="TouchedBottom":
            self.main_status=status
            self.possible=False
            stop=True
        if status=="ReachedBeginning" and sdot_backward[0]<sdot_init:
            self.main_status="LowerThanSdotInit"
            self.possible=False
            stop=True

        if stop:
            return

        self.s_traj_list.append(s_backward)
        self.sdot_traj_list.append(sdot_backward)
        self.cur_s_traj_bw=s_backward
        self.cur_sdot_traj_bw=sdot_backward

        self.sws=list(self.pb.sw_s_list)
        self.swsd=list(self.pb.sw_sdot_list)
        self.swt=list(self.pb.sw_type_list)

        #Integrate forward from the start
        [s_forward,sdot_forward,status]=self.integrate_forward(0,sdot_init)

        if len(s_forward)==0:
            self.main_status='FwTrajVoid'
            self.possible=False
            stop=True
        if status=="TouchedBottom":
            self.main_status=status
            self.possible=False
            stop=True

        self.s_traj_list.append(s_forward)
        self.sdot_traj_list.append(sdot_forward)
        self.cur_s_traj_fw=s_forward
        self.cur_sdot_traj_fw=sdot_forward

        if stop:
            return

        # Integrate backward and forward from the switch points
        self.compute_limiting_curves()



########## Integrate the limiting curves from the switch points #############

    def compute_limiting_curves(self):

        main_status="NothingToReport"
        width=self.width
        duration=self.pb.traj.duration

        if len(self.sws)==0:
            self.sws=list(self.pb.sw_s_list)
            self.swsd=list(self.pb.sw_sdot_list)
            self.swt=list(self.pb.sw_type_list)


       #Integrate backward and forward from switching points
        while len(self.sws)>0:


            si=self.sws.pop(0)
            sdoti=self.swsd.pop(0)
            typei=self.swt.pop(0)


            if self.is_above(si,sdoti,self.cur_s_traj_fw,self.cur_sdot_traj_fw) or self.is_above(si,sdoti,self.cur_s_traj_bw,self.cur_sdot_traj_bw):
                continue

            if  typei=='v' or typei=='t':
                sdoti=self.find_sdot_max(si,sdoti)
                if sdoti<1e-5:
                    main_status="CouldnotCrossSwitch/T"
                    self.possible=False
                    break
                [sb,sf]=[si,si]
                [sdotb,sdotf]=[sdoti,sdoti]

            elif typei=='z':
                [sb,sdotb,sf,sdotf]=self.find_sdot_max_zi(si,sdoti)
                if sdotb<0:
                    main_status="CouldnotCrossSwitch/Z"
                    self.possible=False
                    break                   

                self.s_traj_list.append(array([sb,sf]))
                self.sdot_traj_list.append(array([sdotb,sdotf]))


            stop=False

            #Backward part
            [s_backward,sdot_backward,status]=self.integrate_backward(sb,sdotb)
            if status=="TouchedBottom":
                main_status=status
                self.possible=False
                stop=True

            if len(s_backward)>0:
                self.s_traj_list.append(s_backward)
                self.sdot_traj_list.append(sdot_backward)

            #Forward part            
            [s_forward,sdot_forward,status]=self.integrate_forward(sf,sdotf)
            if status=="TouchedBottom":
                main_status=status
                self.possible=False
                stop=True

            if len(s_forward)>0:
                self.s_traj_list.append(s_forward)
                self.sdot_traj_list.append(sdot_forward)
                self.cur_s_traj_fw=s_forward
                self.cur_sdot_traj_fw=sdot_forward

            if stop:
                break

        return main_status



########################### Integrate forward #############################

    def integrate_forward(self,s_start,sdot_start,width=1e15,test_list=False):

        dt_integ=self.dt_integ
        s_curr=s_start
        sdot_curr=sdot_start
        s_res=[]
        sdot_res=[]
        start=0
        status="NothingToReport"
        cont=True
        while cont:
            #print [s_curr,sdot_curr]
            if len(s_res)>width:
                status="OverpassedPalier"
                break
            if s_curr>self.pb.duration: 
                status="ReachedEnd"
                break
            if sdot_curr<0: 
                [alpha,beta,ialpha,ibeta]=self.pb.accel_limits(s_curr,sdot_curr)
                if alpha>beta: #Double check because of possible discretization errors
                    status="CrossedMaxvel"
                else:
                    status="TouchedBottom"
                break
            if isnan(sdot_curr): 
                status="TouchedBottom"
                break
           # If sdot_cur > combined max sdot curve
            if sdot_curr>self.pb.maxvel_interp(s_curr):
                if(not self.pb.isset_velocity_limits):
                    status="CrossedMaxvel"
                    break
                # If sdot_cur < sdot_vel, it means that sdot_cur > sdot_acc
                if sdot_curr<self.pb.maxvel_velocity_interp(s_curr):
                    status="CrossedMaxvel"
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
                                    status="Zlajpah"
                                    break
                                s_curr=s_next
                                sdot_curr=sdot_next_vel
                            cont2=False
                            cont=False
                            
            # Here sdot_cur < combined max sdot curve, so integrate the max acc
            else:                    
                [alpha,beta,ialpha,ibeta]=self.pb.accel_limits(s_curr,sdot_curr)
                start+=1
                s_res.append(s_curr)
                sdot_res.append(sdot_curr)
                sdot_next=sdot_curr+beta*dt_integ
                s_next=s_curr+sdot_curr*dt_integ
                s_curr=s_next
                sdot_curr=sdot_next
            if self.is_above(s_curr,sdot_curr,self.cur_s_traj_bw,self.cur_sdot_traj_bw) or (test_list and self.is_above_list(s_curr,sdot_curr)):
                s_res.append(s_curr)
                sdot_res.append(sdot_curr)
                status="CrossedBwTraj"
                break
 
        return(array(s_res),array(sdot_res),status)




########################### Integrate backward ############################

    def integrate_backward(self,s_start,sdot_start,width=1e15,test_list=False):

        dt_integ=self.dt_integ
        s_curr=s_start
        sdot_curr=sdot_start
        s_res=[]
        sdot_res=[]
        start=0
        while True:
            #print [s_curr,sdot_curr]
            if len(s_res)>width: 
                status="OverpassedPalier"
                break
            if s_curr<0: 
                status="ReachedBeginning"
                break
            if sdot_curr<0: 
                [alpha,beta,ialpha,ibeta]=self.pb.accel_limits(s_curr,sdot_curr)
                if alpha>beta: #Double check because of possible discretization errors
                    status="CrossedMaxvel"
                else:
                    status="TouchedBottom"
                break
            if isnan(sdot_curr): 
                status="TouchedBottom"
                break
            if sdot_curr>self.pb.maxvel_interp(s_curr): 
                status="CrossedMaxvel"
                break

            [alpha,beta,ialpha,ibeta]=self.pb.accel_limits(s_curr,sdot_curr)
            start+=1
            s_res.append(s_curr)
            sdot_res.append(sdot_curr)
            sdot_next=sdot_curr-alpha*dt_integ
            s_next=s_curr-sdot_curr*dt_integ
            s_curr=s_next
            sdot_curr=sdot_next
            if self.is_above(s_curr,sdot_curr,self.cur_s_traj_fw,self.cur_sdot_traj_fw) or (test_list and self.is_above_list(s_curr,sdot_curr)):
                s_res.append(s_curr)
                sdot_res.append(sdot_curr)
                status='CrossedFwTraj'
                return(array(s_res)[::-1],array(sdot_res)[::-1],status)

        return(array(s_res)[::-1],array(sdot_res)[::-1],status)



    
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
        if len(s_res)==0 or s_res[-1]<self.pb.duration-self.tolerance_ends:
            self.possible=False




############################## Dealing with switch points ################################


    # Find the highest sdot that allows passing through a tangent/disc point
    def find_sdot_max(self,s,sdot_top):

        dt_integ=self.dt_integ
        width=self.width

        n_search=20
        
        for sdot in linspace(sdot_top,0,n_search):
            [s_forward,sdot_forward,status]=self.integrate_forward(s,sdot,width)
            [s_backward,sdot_backward,status]=self.integrate_backward(s,sdot,width)
            if (len(s_forward)>=width or (len(s_forward)>0 and s_forward[-1]>self.pb.duration)) and (len(s_backward)>=width or (len(s_backward)>0 and s_backward[0]<-1)):
                break

        if sdot==0:
            return 0
        if sdot==sdot_top:
            return sdot_top
        else:
            #Here sdot is OK and sdot+sdot_top/(n_search-1) is not OK, we perform a dichotomy search
            top=sdot+sdot_top/(n_search-1)
            bot=sdot
            while n_search>0:
                st=(top+bot)/2
                [s_forward,sdot_forward,status]=self.integrate_forward(st,sdot,width)
                [s_backward,sdot_backward,status]=self.integrate_backward(st,sdot,width)
                if (len(s_forward)>=width or (len(s_forward)>0 and s_forward[-1]>self.pb.duration)) and (len(s_backward)>=width or (len(s_backward)>0 and s_backward[0]<-1)):
                    bot=st
                else:
                    top=st
                n_search-=1
                return bot



    # Find the highest sdot that allows passing through a zero-inertia point
    def find_sdot_max_zi(self,s,sdot_top):

        smart_mode=False
        #if smart_mode=True, we use a smart algorithm to compute the exact slope at the zi point
        if smart_mode:
            slope=self.pb.correct_accel_zi(s)/sdot_top
            tige=self.pb.t_step*4
            sb=s-tige
            sdotb=sdot_top-tige*slope
            sf=s+tige
            sdotf=sdot_top+tige*slope
            return [sb,sdotb,sf,sdotf]
        #if smart_mode=False, we integrate some steps from the zi point and compute the resulting slope
        else:
            n_search=20
            tolerance=20
            palier=int(self.palier/2)

            for sdot in linspace(sdot_top,0,n_search):
                s1=s-palier*sdot*self.dt_integ
                s2=s+palier*sdot*self.dt_integ
                [s_backward,sdot_backward,status]=self.integrate_backward(s1,sdot,palier)
                [s_forward,sdot_forward,status]=self.integrate_forward(s2,sdot,palier)
                if len(s_backward)==0 or len(s_forward)==0:
                    continue
                else:
                    sb=s_backward[0]
                    sdotb=sdot_backward[0]
                    sf=s_forward[-1]
                    sdotf=sdot_forward[-1]
                    if not self.is_above(s,sdot_top,[sb,sf],[sdotb,sdotf]):
                        continue

                    slope=(sdotf-sdotb)/(sf-sb)
                    [alphab,betab,ialpha,ibeta]=self.pb.accel_limits(sb,sdotb)
                    [alphaf,betaf,ialpha,ibeta]=self.pb.accel_limits(sf,sdotf)
                    if slope<max(alphab/sdotb,alphaf/sdotf)-tolerance or slope>min(betab/sdotb,betaf/sdotf)+tolerance:
                        continue

                return [sb,sdotb,sf,sdotf]

            return [s,-1,s,-1]





############################## Utilities ################################


    # Test whether the point (s,sdot) is above the trajectory (s_traj,sdot_traj)
    def is_above(self,s,sdot,s_traj,sdot_traj):
        return sdot>=self.pb.linear_interpolate(s,sdot_traj,s_traj,elim_out=True)


    # Test whether the point (s,sdot) is above the list of traj
    def is_above_list(self,s,sdot):
        for i in range(len(self.s_traj_list)):
            s_traj=self.s_traj_list[i]
            sdot_traj=self.sdot_traj_list[i]
            if self.is_above(s,sdot,s_traj,sdot_traj):
                return True
        return False
            

    # Compute the index of the curve which is to the bottom and its value at s
    def compute_index(self,s,s_traj_list,sdot_traj_list):
        sdot_min=1e15
        index_min=0
        for j in range(len(s_traj_list)):
            sdot=self.pb.linear_interpolate(s,sdot_traj_list[j],s_traj_list[j],elim_out=True)
            if sdot<sdot_min:
                sdot_min=sdot
                index_min=j
        return [index_min,sdot_min]





#################################### Plotting #################################


    # Plot the limiting curves and the final velocity profile
    def plot_profiles(self,h_offset=0):
        T=self.pb.duration
        self.pb.plot_maxvel_curves(h_offset)
        for i in range(len(self.s_traj_list)):
            plot(self.s_traj_list[i]+h_offset,self.sdot_traj_list[i],'k',linewidth=2)        
        if len(self.s_res)>0:
            plot(self.s_res+h_offset,self.sdot_res,'b--',linewidth=4)


    def plot_fw(self,s,sdot,h_offset=0):
        [s_res,sdot_res,status]=self.integrate_forward(s,sdot)
        if len(s_res)>0:
            plot(s_res+h_offset,sdot_res,'k',linewidth=2)

    def plot_bw(self,s,sdot,h_offset=0):
        [s_res,sdot_res,status]=self.integrate_backward(s,sdot)
        if len(s_res)>0:
            plot(s_res+h_offset,sdot_res,'k',linewidth=2)

