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
Shortcutting algorithm and integration with OpenRave trajectory class
Cf. Pham, Asian-MMS 2012, http://www.normalesup.org/~pham/docs/shortcuts.pdf
"""




from openravepy import *
from pylab import *
import time
import MintimeTrajectory
import MintimeProfileIntegrator
import MintimeProblemTorque




############################### Misc #################################


class Tunings():    
    def __init__(self):
        pass

def traj_to_rave_traj(robot,traj):
    """Converts an instance of MintimeTrajectory into a rave trajectory"""
    dt_vect_res=zeros(len(traj.t_vect))
    dt_vect_res[1:len(traj.t_vect)]=diff(traj.t_vect)
    via_points_res=transpose(traj.q_vect)
    rave_traj_res=RaveCreateTrajectory(robot.GetEnv(),'')
    spec = robot.GetActiveConfigurationSpecification('linear')
    spec.AddDeltaTimeGroup()
    rave_traj_res.Init(spec)
    rave_traj_res.Insert(0,c_[via_points_res,dt_vect_res].flatten())

    return rave_traj_res


def linear_smooth_dichotomy(robot,vp_list,coll_check_step):
    """Recursively smooth a list of via points"""
    if len(vp_list)==2:
        return vp_list
    else:
        p1=vp_list[0]
        p2=vp_list[-1]
        d=linalg.norm(p2-p1)
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




#################### RRT to find a collision-free path ####################

def RRT(robot,goal_config):
    """Find a RRT trajectory using OpenRAVE's bi-rrt
    Linear smoothing by dichotomy to reduce the number of via-points
    Return a rave trajectory
    
    robot -- the robot
    goal_config -- goal configuration
    
    """
    params = Planner.PlannerParameters()
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(goal_config)
    params.SetExtraParameters('<_postprocessing planner="shortcut_linear"><_nmaxiterations>40</_nmaxiterations></_postprocessing>')

    env=robot.GetEnv()
    planner=RaveCreatePlanner(env,'birrt')
    planner.InitPlan(robot, params)

    traj = RaveCreateTrajectory(env,'')
    planner.PlanPath(traj)

    traj_rrt=[]
    for i in range(traj.GetNumWaypoints()):
        traj_rrt.append(traj.GetWaypoint(i))

    # Linear smoothing by dichotomy

    via_points=linear_smooth_dichotomy(robot,traj_rrt,0.001)
    dt_vect=array([1]*len(via_points))
    dt_vect[0]=0

    rave_traj=RaveCreateTrajectory(env,'')
    spec = robot.GetActiveConfigurationSpecification('linear')
    spec.AddDeltaTimeGroup()
    rave_traj.Init(spec)
    rave_traj.Insert(0,c_[via_points,dt_vect].flatten())
    return rave_traj





########## Time-optimal parameterization of a RaveTrajectory ############

def Retime(robot,rave_traj,tunings):
    """Optimal-time parameterization of a trajectory under torque constraints
    Return a couple (rave_traj,traj) where traj is an instance of MintimeTrajectory class

    robot -- the robot
    rave_traj -- an openrave trajectory
    tunings -- set of tuning parameter for the time-parameterization algorithm (see test-RRT.py for explanations)
    
    """
    t_step=tunings.t_step
    tau_min=tunings.tau_min
    tau_max=tunings.tau_max
    qd_max=tunings.qd_max
    
    algo_list=[]
    traj2_list=[]

    for i_way in range(1,rave_traj.GetNumWaypoints()):
        a_prev=rave_traj.GetWaypoints(i_way-1,i_way)
        a=rave_traj.GetWaypoints(i_way,i_way+1)
        T=a[-1]
        pieces_list=[]
        for i_dof in range(len(a)-1):
            pieces_list.append(poly1d([(a[i_dof]-a_prev[i_dof])/T,a_prev[i_dof]]))
        pwp_traj_linear=MintimeTrajectory.PieceWisePolyTrajectory([pieces_list],[T])
        traj=pwp_traj_linear.GetSampleTraj(T,t_step)

        # Run the algorithm
        pb=MintimeProblemTorque.MintimeProblemTorque(robot,traj)
        pb.set_dynamics_limits([tau_min,tau_max])
        pb.set_velocity_limits(qd_max)
        pb.disc_thr=1
        pb.preprocess()
        algo=MintimeProfileIntegrator.MintimeProfileIntegrator(pb)
        algo.dt_integ=tunings.dt_integ
        algo.width=tunings.width
        algo.palier=tunings.palier
        algo.tolerance_ends=tunings.tolerance_ends
        algo.sdot_init=1e-4
        algo.sdot_final=1e-4
        algo.integrate_all_profiles()
        algo.integrate_final()
        algo_list.append(algo)

        # Reparameterize using the velocity profile
        s_res=algo.s_res
        sdot_res=algo.sdot_res
        undersample_coef=int(round(t_step/algo.dt_integ))
        s_res_u=s_res[range(1,len(s_res),undersample_coef)]
        sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]
        traj2=pwp_traj_linear.ResampleTraj(s_res_u,sdot_res_u,t_step)
        traj2_list.append(traj2)

    traj2_list_copy=list(traj2_list)
    traj2=MintimeTrajectory.Concat(traj2_list)

    rave_traj=traj_to_rave_traj(robot,traj2)
    
    return [rave_traj,traj2]






##################  Repeatedly apply IterateSmooth ######################


def RepeatIterateSmooth(robot,traj,tunings):
    """Find a global time-optimal trajectory by applying n_runs times the shortcutting algorithm and taking the best result
    Return a couple (rave_traj,traj) where traj is an instance of MintimeTrajectory class

    robot -- the robot
    rave_traj -- an openrave trajectory
    tunings -- set of tuning parameter for the time-parameterization algorithm and of the shortcutting algorithm (see test-RRT.py for explanations)
    
    """
    n_runs=tunings.n_runs
    tau_max=tunings.tau_max
    tau_min=tunings.tau_min
    coef_tolerance=tunings.coef_tolerance

    trajlist=[]
    traj_list_list=[]
    ends_list_list=[]
    ls=[]
    la=[]
    lc=[]
    i=0
    while i <n_runs:
        print i
        [traj3,d_list_s,d_list_all,n_collisions,traj_list_one,ends_list]=IterateSmooth(robot,traj,tunings)
        tau3=MintimeProblemTorque.ComputeTorques(robot,traj3,tunings.grav)
        print [max(abs(tau3[k,:])) for k in range(4)]
        if (True not in [max(tau3[k,:])/coef_tolerance>tau_max[k] for k in range(4)]) and (True not in [min(tau3[k,:])/coef_tolerance<tau_min[k] for k in range(4)]):  
            trajlist.append(traj3)
            ls.append(d_list_s)
            la.append(d_list_all)
            lc.append(n_collisions)
            traj_list_list.append(traj_list_one)
            ends_list_list.append(ends_list)
            i+=1

    dd=[k[-1] for k in la]
    i=dd.index(min(dd))
    traj3=trajlist[i]

    rave_traj=traj_to_rave_traj(robot,traj3)


    txt = '\n'
    txt+= '\n'
    txt+= '\n'
    txt+= '********************************************************************\n'
    txt+= '********************** Final result ********************************\n'
    txt+= '\n'
    txt+= 'Trajectory duration before shortcutting: '+str(traj.duration) +'s\n'
    txt+= 'Trajectory duration after shortcutting: '+str(traj3.duration) + 's ('+ str(int(traj3.duration/traj.duration*100))+'% of original)'+'\n'
    txt+= 'Number of attempted shortcuts: '+str(len(la[i])) +'\n'
    txt+= 'Number of effective shortcuts: '+str(len(ls[i])) +'\n'
    txt+= '\n'
    txt+= '********************************************************************\n'
    txt+= '********************************************************************\n'
    txt+= '\n'
    print txt

    return [rave_traj,traj3]






##################  Repeatedly apply shortcuts ######################

def IterateSmooth(robot,traj_orig,tunings):
    """Find a global time-optimal trajectory by repeatedly applying time-optimal random shortcuts
    Return a couple (rave_traj,traj) where traj is an instance of MintimeTrajectory class

    robot -- the robot
    rave_traj -- an openrave trajectory
    tunings -- set of tuning parameter for the time-parameterization algorithm and of the shortcutting algorithm (see test-RRT.py for explanations)
    
    """
    t_step=tunings.t_step
    max_time=tunings.max_time
    mean_len=tunings.mean_len
    std_len=tunings.std_len

    traj_list_one=[traj_orig]
    ends_list=[]
    n_shortcuts=0
    n_collisions=0
    d_list_s=[]
    d_list_all=[]
    traj=traj_orig
    reps=0
    deb=time.time()
    while time.time()-deb<max_time: 
        T=traj.duration
        rand_len=mean_len+randn()*std_len
        if rand_len>T or rand_len<0: 
            continue
        t1=rand()*(T-rand_len)
        t2=t1+rand_len
        #if t1<0 or t2>T or t1>t2: continue
        reps+=1
        print '\n***** '+str(reps)+' *****'
        [success,traj2]=Smooth(robot,traj,t1,t2,tunings)
        d_list_all.append(traj2.duration)
        if success=='great':
            traj_list_one.append(traj2)
            ends_list.append((traj2.i1,traj2.i2))
            traj=traj2
            n_shortcuts+=1
            d_list_s.append(traj2.duration)
        elif success=='collision':
            n_collisions+=1

    duration=traj_orig.t_step*(traj_orig.n_steps-1)
    duration2=traj2.t_step*(traj2.n_steps-1)
    print '\n\n---------------------------------\n\n'
    print 'Original duration: '+str(duration)+' s'
    print 'Final duration: '+str(duration2)+' s (saves '+str(int((duration-duration2)/duration*100))+'%)'
    print 'Computation time: '+str(time.time()-deb)
    print 'Number of shortcuts made: '+str(n_shortcuts)
    print '\n\n---------------------------------\n\n'

    return [traj2,d_list_s,d_list_all,n_collisions,traj_list_one,ends_list]





########################  Apply one shortcut ###################################

def Smooth(robot,traj_orig,t1,t2,tunings):
    """Try to apply a time-optimal shortcut between two time instants of a trajectory
    Return a couple (rave_traj,traj) where traj is an instance of MintimeTrajectory class

    robot -- the robot
    traj_orig -- the original trajectory (instance of MintimeTrajectory)
    (t1,t2) -- start and end time instants of the candidate shortcut
    tunings -- set of tuning parameter for the time-parameterization algorithm and of the shortcutting algorithm
    
    """
    tau_min=tunings.tau_min
    tau_max=tunings.tau_max
    qd_max=tunings.qd_max
    dt_sample=tunings.dt_sample
    dt_integ=tunings.dt_integ
    threshold_waive=tunings.threshold_waive


    deb=time.time()
    i1=0
    t_vect=traj_orig.t_vect
    n_steps=traj_orig.n_steps
    t_step=traj_orig.t_step
    q_vect=traj_orig.q_vect
    qd_vect=traj_orig.qd_vect

    for i in range(n_steps):
        if t1<t_vect[i]:
            i1=i
            break
    for i in range(i1,n_steps):
        if t2<t_vect[i]:
            i2=i
            break

    print 'Initial time: '+str(t1)
    print 'Final time: '+str(t2)
    print 'Duration: '+str(t2-t1)

    c_sdot=1



    # Generate a shortcut path

    q_list=[q_vect[:,i1],q_vect[:,i2]]
    qd_list=[qd_vect[:,i1]/c_sdot,qd_vect[:,i2]/c_sdot]
    T_list=[t2-t1]
    pwp_traj_shortcut=MintimeTrajectory.Interpolate(q_list,qd_list,T_list)
    sample_traj_shortcut=pwp_traj_shortcut.GetSampleTraj(pwp_traj_shortcut.duration,dt_sample)

    if MintimeProblemTorque.CheckCollisionTraj(robot,sample_traj_shortcut)[0]:
        print 'Shortcut collides, returning'
        print 'Computation time was: '+str(time.time()-deb)
        return ['collision',traj_orig]



    # Run the optimal-time parameterization algorithm

    pb=MintimeProblemTorque.MintimeProblemTorque(robot,sample_traj_shortcut)
    pb.set_dynamics_limits([tau_min,tau_max])
    pb.set_velocity_limits(qd_max)
    pb.disc_thr=1
    pb.preprocess()
    algo=MintimeProfileIntegrator.MintimeProfileIntegrator(pb)
    algo.dt_integ=dt_integ
    algo.width=tunings.width
    algo.palier=tunings.palier
    algo.tolerance_ends=tunings.tolerance_ends
    algo.sdot_init=c_sdot
    algo.sdot_final=c_sdot
    algo.integrate_all_profiles()



    # Various tests to see if we should keep the shortcut

    if not algo.possible:
        print 'Shortcut is dynamically imposible, returning'
        print 'Computation time was: '+str(time.time()-deb)
        return ['impossible',traj_orig]

    algo.integrate_final()

    if not algo.possible:
        print 'Shortcut is dynamically imposible, returning'
        print 'Computation time was: '+str(time.time()-deb)
        return ['impossible',traj_orig]

    s_res=algo.s_res
    sdot_res=algo.sdot_res
    t_shortcut=len(s_res)*algo.dt_integ
    
    if t_shortcut>=t2-t1:
        print 'Shortcut time ('+str(t_shortcut)+') is longer than original, returning'
        print 'Computation time was: '+str(time.time()-deb)
        return ['too_long',traj_orig]
    if t2-t1-t_shortcut<threshold_waive:
        print 'Gain is not significant ('+str(t2-t1-t_shortcut)+'), returning'
        print 'Computation time was: '+str(time.time()-deb)
        return ['not_signif',traj_orig]
        
    print 'Great! Shortcut time is: '+str(t_shortcut)+' ('+str(int(t_shortcut/(t2-t1)*100))+'% of original, saves '+str(t2-t1-t_shortcut)+' seconds)'
    print 'Computation time was: '+str(time.time()-deb)+' seconds'    

    undersample_coef=t_step/algo.dt_integ
    undersample_coef=int(round(undersample_coef))
    s_res_u=s_res[range(1,len(s_res),undersample_coef)]
    sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]

    if len(s_res_u)<2:
        return ['impossible',traj_orig]



    # If OK then replace

    resampled_traj_u=pwp_traj_shortcut.ResampleTraj(s_res_u,sdot_res_u,t_step)
    tau_u=MintimeProblemTorque.ComputeTorques(robot,resampled_traj_u,tunings.grav)
    traj2=MintimeTrajectory.Insert(traj_orig,i1,i2,resampled_traj_u)
    traj2.i1=i1
    traj2.i2=i2


    return ['great',traj2]
