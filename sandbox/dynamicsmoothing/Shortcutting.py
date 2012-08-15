from openravepy import *
from numpy import *
from pylab import *
import Trajectory
import MinimumTime
import time



def IterateSmooth(robot,traj_orig,tunings,grav,max_time,mean_len,std_len,tau_min,tau_max,qd_max):

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
        [success,traj2]=Smooth(robot,traj,t1,t2,tunings,grav,tau_min,tau_max,qd_max)
        d_list_all.append(traj2.duration)
        if success=='great':
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

    return [traj2,d_list_s,d_list_all,n_collisions]




def Smooth(robot,traj_orig,t1,t2,tunings,grav,tau_min,tau_max,qd_max):

    global coco
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

    q_list=[q_vect[:,i1],q_vect[:,i2]]
    qd_list=[qd_vect[:,i1]/c_sdot,qd_vect[:,i2]/c_sdot]
    T_list=[t2-t1]
    pwp_traj_shortcut=Trajectory.Interpolate(q_list,qd_list,T_list)
    sample_traj_shortcut=pwp_traj_shortcut.GetSampleTraj(pwp_traj_shortcut.duration,tunings['t_step_sample'])

    if Trajectory.CheckCollisionTraj(robot,sample_traj_shortcut)[0]:
        print 'Shortcut collides, returning'
        print 'Computation time was: '+str(time.time()-deb)
        return ['collision',traj_orig]

    pb=MinimumTime.RobotMinimumTime(robot,sample_traj_shortcut,tunings,grav,tau_min,tau_max,qd_max,c_sdot,c_sdot)
    if not pb.possible:
        print 'Shortcut is dynamically imposible, returning'
        print 'Computation time was: '+str(time.time()-deb)
        return ['impossible',traj_orig]

    s_res=pb.s_res
    sdot_res=pb.sdot_res
    t_shortcut=len(s_res)*tunings['t_step_integrate']
    
    if t_shortcut>=t2-t1:
        print 'Shortcut time ('+str(t_shortcut)+') is longer than original, returning'
        print 'Computation time was: '+str(time.time()-deb)
        return ['too_long',traj_orig]
    if t2-t1-t_shortcut<tunings['threshold_waive']:
        print 'Gain is not significant ('+str(t2-t1-t_shortcut)+'), returning'
        print 'Computation time was: '+str(time.time()-deb)
        return ['not_signif',traj_orig]
        
    print 'Great! Shortcut time is: '+str(t_shortcut)+' ('+str(int(t_shortcut/(t2-t1)*100))+'% of original, saves '+str(t2-t1-t_shortcut)+' seconds)'
    print 'Computation time was: '+str(time.time()-deb)+' seconds'    

    undersample_coef=t_step/tunings['t_step_integrate']
    undersample_coef=int(round(undersample_coef))
    s_res_u=s_res[range(1,len(s_res),undersample_coef)]
    sdot_res_u=sdot_res[range(1,len(s_res),undersample_coef)]

    if len(s_res_u)<2:
        return ['impossible',traj_orig]

    resampled_traj_u=pwp_traj_shortcut.ResampleTraj(s_res_u,sdot_res_u,t_step)
    tau_u=Trajectory.ComputeTorques(robot,resampled_traj_u,grav)
    traj2=Trajectory.Insert(traj_orig,i1,i2,resampled_traj_u)


    return ['great',traj2]
