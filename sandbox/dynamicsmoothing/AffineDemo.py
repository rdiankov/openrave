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



#http://openrave.org/latest_stable/en/main/_modules/openravepy/examples/testviewercallback.html#itemselectioncb
#g=dragsphere.GetLinks()[0].GetGeometries()[0]
#g.SetTransparency(0.5)
#g.SetAmbientColor([0,1,0])



from openravepy import *
from numpy import *
import HRP4
import ZMP
import time
import thread
import FollowTrajectory
import Affine
import Trajectory
from pylab import *

norm=linalg.norm

def start(params):
    global thread_data
    global thread_lock

    robot=params['robot']
    robot2=params['robot2']
    traj=params['traj']
    linkindex=params['linkindex']
    linkindex2=params['linkindex2']

    #Robot 2
    final_config=array(traj.q_vect[:,-1])
    final_config[[0,1,2]]+=params['offset']
    HRP4.SetConfig(robot2,final_config)

    
    #Find the initial speed of the sphere
    T=params['T']
    initial_q=traj.q_vect[:,T]
    initial_qd=traj.q_vect[:,T]-traj.q_vect[:,T-1]

    J_link2=compute_Jacobian_speed(params)
    initial_sphere_speed=dot(J_link2,initial_qd)
    
    #Initial positions of the spheres
    p_sphere_pos=robot.GetLinks()[linkindex2].GetGlobalCOM()
    v_sphere_pos=p_sphere_pos+initial_sphere_speed*params['dist']
    p_sphere=CreateSphere(p_sphere_pos,[0,1,0],params)
    v_sphere=CreateSphere(v_sphere_pos,[1,0,0],params)

    thread_data={'continue':True}
    thread_data['initial_q']=initial_q
    thread_data['initial_qd']=initial_qd
    thread_data['p_sphere']=p_sphere
    thread_data['v_sphere']=v_sphere
    thread_data['p_sphere_pos']=p_sphere_pos
    thread_data['v_sphere_pos']=v_sphere_pos
    thread_data['cur_vit']=initial_sphere_speed
    thread_data['state']='Rest'


    thread_lock=thread.allocate_lock()

    thread.start_new_thread(update,(params,))


def compute_Jacobian_speed(params):
    robot=params['robot']
    linkindex=params['linkindex']
    linkindex2=params['linkindex2']
    euler=HRP4.mat2euler(robot.GetLinks()[0].GetTransform()[0:3,0:3])
    link2=robot.GetLinks()[linkindex2]
    p_link2=link2.GetGlobalCOM()
    return FollowTrajectory.Jacobian(euler,linkindex,p_link2,params)


def stop(params):
    global thread_data
    global thread_lock
    thread_lock.acquire()
    thread_data['continue']=False
    thread_lock.release()
    time.sleep(0.1)
    params['robot'].GetEnv().Remove(thread_data['p_sphere'].GetParent())
    params['robot'].GetEnv().Remove(thread_data['v_sphere'].GetParent())
    del thread_lock
    


def update(params):
    global thread_data
    global thread_lock
    while thread_data['continue']:
        thread_lock.acquire()
        cur_pos=thread_data['p_sphere'].GetGlobalCOM()
        cur_vit=thread_data['v_sphere'].GetGlobalCOM()
        past_pos=thread_data['p_sphere_pos']
        past_vit=thread_data['v_sphere_pos']

        if(norm(cur_pos-past_pos)+norm(cur_vit-past_vit)<1e-5):
            if(thread_data['state']!='Rest'):
                exec_traj(cur_pos,cur_vit,params)
                thread_data['state']='Rest'
        else:
            if norm(cur_pos-thread_data['p_sphere_pos'])>1e-10:
                thread_data['state']='Move_p'
            else:
                thread_data['state']='Move_v'               

        thread_data['p_sphere_pos']=thread_data['p_sphere'].GetGlobalCOM()
        thread_data['v_sphere_pos']=thread_data['v_sphere'].GetGlobalCOM()
        thread_lock.release()
        time.sleep(0.1)
    print 'Terminated'
    return


def exec_traj(p_sphere_pos,v_sphere_pos,params):
    global thread_data
    global thread_lock
    robot=params['robot']

    #Reach the current position of p_sphere
    HRP4.SetConfig(robot,thread_data['initial_q'])
    config=Reach(params['linkindex'],params['linkindex2'],p_sphere_pos,params['n_steps'],params)
    HRP4.SetConfig(robot,config)
    #If Move_p, update the position of the v_sphere
    if thread_data['state']=='Move_p':
        Trans=eye(4)
        Trans[0:3,3]=p_sphere_pos+thread_data['cur_vit']*params['dist']
        thread_data['v_sphere'].SetTransform(Trans)
    #Else, i.e. Move_v, update the current v
    else:
        thread_data['cur_vit']=(v_sphere_pos-p_sphere_pos)/params['dist']

    #COmpute the deformed trajectory and move robot 2
    traj=params['traj']
    tau=params['tau']
    T=params['T']
    q_T=thread_data['initial_q']
    q_Td=config
    qd_T=thread_data['initial_qd']
    J=compute_Jacobian_speed(params)
    qd_Td=qd_T+dot(linalg.pinv(J),thread_data['cur_vit']-dot(J,qd_T))

    if  params['with_velocity']:
        traj_def=Affine.deform_traj(traj,tau,T,q_Td,params['active_dofs'],qd_T,qd_Td)
    else:
        traj_def=Affine.deform_traj(traj,tau,T,q_Td,params['active_dofs'])
       
    for k in range(traj_def.n_steps):
        traj_def.q_vect[[0,1,2],k]+=params['offset']

    robot2=params['robot2']
    Trajectory.Execute(robot2,traj_def,0.001)
    thread_data['traj_def']=traj_def


def CreateSphere(pos,color,params):
    robot=params['robot']
    sphere = RaveCreateKinBody(robot.GetEnv(),'')
    t=zeros(4)
    t[3]=params['sphere_radius']
    sphere.InitFromSpheres(array([t]),True)
    T=eye(4)
    T[0:3,3]=pos
    sphere.SetTransform(T)
    g=sphere.GetLinks()[0].GetGeometries()[0]
    g.SetTransparency(0.7)
    g.SetAmbientColor(color)
    g.SetDiffuseColor(color)
    sphere.SetName('sphere')
    robot.GetEnv().Add(sphere,True)
    if (not params['with_velocity']) and color==[1,0,0]:
        sphere.SetVisible(False)
    return sphere.GetLinks()[0]




def Reach(linkindex,linkindex2,p_end,n_steps,params):
    robot=params['robot']
    Q0=params['Q0']
    Q0inv=params['Q0inv']
    lower_lim=params['lower_lim']
    upper_lim=params['upper_lim']
    K_v=params['K_v']
    K_p=params['K_p']
    K_li=params['K_li']
    K_sr=params['K_sr']

    n_dof=robot.GetDOF()
    n_var=n_dof+6
    
    baselink=robot.GetLinks()[0]

    res=[]
    res.append(HRP4.GetConfig(robot))
    cur_config=res[0]

    link2=robot.GetLinks()[linkindex2]

    with robot:

        for step in range(n_steps):
            T=baselink.GetTransform()
            euler=HRP4.mat2euler(T[0:3,0:3])

            # Compute the Jacobians and desired velocities of the markers to follow
            p_cur=link2.GetGlobalCOM()
            J_marker=FollowTrajectory.Jacobian(euler,linkindex,p_cur,params)

            v_desired=(p_end-p_cur)/(n_steps-step)

            # Find the out-of-range DOFs
            lower_list=[]
            upper_list=[]
            DOFvalues=robot.GetDOFValues()       
            for j in range(n_dof):
                if DOFvalues[j]<lower_lim[j]:
                    lower_list.append(j)
                elif DOFvalues[j]>upper_lim[j]:
                    upper_list.append(j)

            # Compute the Jacobians and the desired velocities for the out-of-range DOFs
            J_lower=zeros((n_var,n_var))
            v_lower=zeros(n_var)
            J_upper=zeros((n_var,n_var))
            v_upper=zeros(n_var)
            for i in lower_list:            
                J_lower[6+i,6+i]=1
                v_lower[6+i]=K_li*(lower_lim[i]-DOFvalues[i])
            for i in upper_list:
                J_upper[6+i,6+i]=1
                v_upper[6+i]=K_li*(upper_lim[i]-DOFvalues[i])
            J_limits=FollowTrajectory.concat([J_lower,J_upper])
            v_limits=FollowTrajectory.concat([v_lower,v_upper])


            J_main=FollowTrajectory.concat([J_marker,J_limits])
            v_main=FollowTrajectory.concat([v_desired,v_limits])

            J_main_star=dot(J_main,Q0inv)
            J_main_star_dash=linalg.pinv(J_main_star)
            J_weighted_pinv=dot(Q0inv,J_main_star_dash)
            thetad_0=dot(J_weighted_pinv,v_main)

            thetad=thetad_0

            cur_config=cur_config+thetad
            res.append(cur_config)
            HRP4.SetConfig(robot,cur_config)

    return cur_config



def plot_limits(traj,color,params):
    robot=params['robot']
    active_dofs=params['active_dofs']
    limits=params['limits']
    lower_lim=limits[0]    
    upper_lim=limits[1]
    lower_lim_o=limits[2]    
    upper_lim_o=limits[3]
    
    
    for i in range(len(active_dofs)):
        subplot(3,4,i+1)
        k=active_dofs[i]
        title(robot.GetJoints()[k-6].GetName())
        xticks([])
        plot(traj.q_vect[k,:],color)
        plot([0,traj.n_steps-1],[lower_lim_o[k-6],lower_lim_o[k-6]],'k')
        plot([0,traj.n_steps-1],[upper_lim_o[k-6],upper_lim_o[k-6]],'k')
        plot([0,traj.n_steps-1],[lower_lim[k-6],lower_lim[k-6]],'k')
        plot([0,traj.n_steps-1],[upper_lim[k-6],upper_lim[k-6]],'k')






