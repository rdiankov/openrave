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

norm=linalg.norm

def start(params):
    global thread_params
    global thread_data
    global thread_lock
    thread_params=params
    thread_data={'continue':True,'dragsphere':None,'pinned_list':[],'drag':[-1,'simple']}
    thread_lock=thread.allocate_lock()
    robot=thread_params['robot']
    exclude_list=thread_params['exclude_list']
    CreateDummySphere([0,-0.2,1.75],'Pin',[1,0,0])
    CreateDummySphere([0,0.2,1.75],'Unpin',[0,1,0])
    for i in range(robot.GetDOF()):
        if not(i in exclude_list):
            CreateDragSphere(i)

    env=thread_params['robot'].GetEnv()
    thread_data['handle']=env.GetViewer().RegisterItemSelectionCallback(selection_callback)
    thread.start_new_thread(reach_thread,())

def stop():
    global thread_params
    global thread_data
    global thread_lock
    thread_lock.acquire()
    thread_data['continue']=False
    thread_lock.release()
    time.sleep(0.1)
    del thread_lock
    env=thread_params['robot'].GetEnv()
    for k in env.GetBodies():
        if k.GetName().find('drag')>=0 or k.GetName().find('in')>=0:
            env.Remove(k)
    del thread_data['handle']


def reach_thread():
    global thread_data
    global thread_lock
    while thread_data['continue']:
        #thread_lock.acquire()
        if thread_data['dragsphere']!=None:
            p_desired=thread_data['dragsphere'].GetLinks()[0].GetGlobalCOM()
            IKreach(thread_data['drag'],thread_data['pinned_list'],p_desired)
        #thread_lock.release()
        time.sleep(0.01)
    print 'Terminated'
    return



def selection_callback(clicked_link,v1,v2):
    global thread_params
    global thread_lock
    global thread_data
    link_name=clicked_link.GetParent().GetName()
    thread_lock.acquire()
    if link_name.find('dragsphere')>=0:
        selected_link=int(link_name[10:len(link_name)])
        if selected_link!=thread_data['drag'][0]:
            thread_data['dragsphere']=clicked_link.GetParent()
            thread_data['drag']=[selected_link,'simple']
    if link_name=='Pin':
        thread_data['dragsphere']=None
        pin_link=thread_data['drag'][0]
        if not(pin_link in thread_data['pinned_list']):
            thread_data['pinned_list'].append(pin_link)
            UpdateSphere(pin_link,[1,0,0])
    if link_name=='Unpin':
        thread_data['dragsphere']=None
        pin_link=thread_data['drag'][0]
        if pin_link in thread_data['pinned_list']:
            thread_data['pinned_list'].remove(pin_link)
            UpdateSphere(pin_link,[0,1,0])
    thread_lock.release()


def UpdateSphere(linkindex,color=None):
    global thread_params
    robot=thread_params['robot']
    env=robot.GetEnv()
    sphere_name='dragsphere'+str(linkindex)
    for k in env.GetBodies():
        if k.GetName()==sphere_name:
            linkCOM=robot.GetLinks()[linkindex].GetGlobalCOM()
            T=robot.GetLinks()[linkindex].GetTransform()
            T[0:3,3]=linkCOM
            k.SetTransform(T)
            k.SetDOFValues(zeros(k.GetDOF()))
            if color!=None:
                g=k.GetLinks()[0].GetGeometries()[0]
                g.SetAmbientColor(color)
                g.SetDiffuseColor(color)
                if color[0]==1:
                    g.SetTransparency(0.3)
                else:
                    g.SetTransparency(0.7)
                  
                    

def CreateDummySphere(pos,name,color):
    global thread_params
    robot=thread_params['robot']
    dummysphere = RaveCreateKinBody(robot.GetEnv(),'')
    t=zeros(4)
    t[3]=0.2
    dummysphere.InitFromSpheres(array([t]),True)
    g=dummysphere.GetLinks()[0].GetGeometries()[0]
    linkCOM=pos
    T=robot.GetLinks()[0].GetTransform()
    T[0:3,3]=linkCOM
    dummysphere.SetTransform(T)
    dummysphere.SetName(name)
    g=dummysphere.GetLinks()[0].GetGeometries()[0]
    g.SetTransparency(0.3)
    g.SetAmbientColor(color)
    g.SetDiffuseColor(color)
    robot.GetEnv().Add(dummysphere,True)



def CreateDragSphere(linkindex):
    global thread_params
    robot=thread_params['robot']
    if linkindex in [16]:
        l=0.12
    else:
        l=0.08
    dragsphere = RaveCreateKinBody(robot.GetEnv(),'')
    t=zeros(4)
    t[3]=l
    dragsphere.InitFromSpheres(array([t]),True)
    g=dragsphere.GetLinks()[0].GetGeometries()[0]
    linkCOM=robot.GetLinks()[linkindex].GetGlobalCOM()
    T=robot.GetLinks()[linkindex].GetTransform()
    T[0:3,3]=linkCOM
    dragsphere.SetTransform(T)
    dragsphere.SetName('dragsphere'+str(linkindex))
    g=dragsphere.GetLinks()[0].GetGeometries()[0]
    g.SetTransparency(0.7)
    g.SetAmbientColor([0,1,0])
    g.SetDiffuseColor([0,1,0])
    robot.GetEnv().Add(dragsphere,True)


def euler_jacobian(euler,u):
    gamma=euler[0] #yaw
    beta=euler[1] #pitch
    alpha=euler[2] #roll
    J=zeros((3,3))
    J[:,0]=dot(HRP4.euler2mat_dgamma(euler),u)
    J[:,1]=dot(HRP4.euler2mat_dbeta(euler),u)
    J[:,2]=dot(HRP4.euler2mat_dalpha(euler),u)
    return J



def Jacobian(euler,linkindex,position):
    global thread_params   
    robot=thread_params['robot']
    base_link=robot.GetLinks()[0]
    with robot:
        J0=robot.CalculateJacobian(linkindex,position)
        u=position-base_link.GetTransform()[0:3,3]
        
    (k,n)=shape(J0)
    J=zeros((3,n+6))
    if thread_params['activate_base_translations']:
        J[:,0:3]=eye(3) #Jacobian with respect to base link translations
    if thread_params['activate_base_rotations']:
        J[:,3:6]=euler_jacobian(euler,u) #Jacobian with respect to base link rotations
    J[:,6:n+6]=J0 #Jacobian with respect to joint angle changes

    return J


def concat(mat_list):
    if mat_list[0]==None:
        return concatenate(mat_list[1:len(mat_list)])
    return concatenate(mat_list)
       



def IKreach(drag,pinned_links,p_end):
    global thread_params
    robot=thread_params['robot']
    p_step=thread_params['p_step']
    Q0=thread_params['Q0']
    Q0inv=thread_params['Q0inv']
    lower_lim=thread_params['lower_lim']
    upper_lim=thread_params['upper_lim']
    K_li=thread_params['K_li']
    K_sr=thread_params['K_sr']

    ndof=robot.GetDOF()

    drag_link=drag[0]
    drag_type=drag[1]

    n_pins=len(pinned_links)
    baselink=robot.GetLinks()[0]

    link=robot.GetLinks()[drag_link]
    p_init=link.GetGlobalCOM()
    n_steps=norm(p_end-p_init)/p_step
    for steps in range(int(n_steps)+1):
        p_cur=link.GetGlobalCOM()
        T=baselink.GetTransform()
        euler=HRP4.mat2euler(T[0:3,0:3])

        # Compute the dragged link Jacobian
        if drag_type=='translation':
            J_drag_a=Jacobian(euler,drag_link,p_cur)
            J_drag_b=Jacobian(euler,drag_link,p_cur+array([0,0,1]))
            J_drag_c=Jacobian(euler,drag_link,p_cur+array([0,1,0]))
            J_drag=concat([J_drag_a,J_drag_b,J_drag_c])
            (k,nvar)=shape(J_drag_a)
        else:
            J_drag=Jacobian(euler,drag_link,p_cur)
            (k,nvar)=shape(J_drag)
           
        # Compute the desired_velocity
        dist=norm(p_end-p_cur)
        if dist<p_step:
            v_drag_0=p_end-p_cur
        else:
            v_drag_0=(p_end-p_cur)/dist*p_step

        if drag_type=='translation':
            v_drag=concat([v_drag_0,v_drag_0,v_drag_0])
        else:
            v_drag=v_drag_0

        # Compute the Jacobians and the desired velocities of the pins
        J_pins=None
        v_pins=None
        for i in range(n_pins):
            pinned_i=pinned_links[i]
            pinned_link=robot.GetLinks()[pinned_i]
            CoMi=pinned_link.GetGlobalCOM()
            J_pinned_ia=Jacobian(euler,pinned_i,CoMi)
            J_pinned_ib=Jacobian(euler,pinned_i,CoMi+array([0,0,1]))
            J_pinned_ic=Jacobian(euler,pinned_i,CoMi+array([0,1,0]))
            J_pins=concat([J_pins,J_pinned_ia,J_pinned_ib,J_pinned_ic])
            v_pins=concat([v_pins,zeros(3*k)])
        
        # Find the out-of-range DOFs
        lower_list=[]
        upper_list=[]
        DOFvalues=robot.GetDOFValues()       
        for j in range(ndof):
            if DOFvalues[j]<lower_lim[j]:
                lower_list.append(j)
            elif DOFvalues[j]>upper_lim[j]:
                upper_list.append(j)

        # Compute the Jacobians and the desired velocities for the out-of-range DOFs
        J_lower=zeros((nvar,nvar))
        v_lower=zeros(nvar)
        J_upper=zeros((nvar,nvar))
        v_upper=zeros(nvar)
        for i in lower_list:            
            J_lower[6+i,6+i]=1
            v_lower[6+i]=K_li*(lower_lim[i]-DOFvalues[i])
        for i in upper_list:
            J_upper[6+i,6+i]=1
            v_upper[6+i]=K_li*(upper_lim[i]-DOFvalues[i])
        J_limits=concat([J_lower,J_upper])
        v_limits=concat([v_lower,v_upper])

        # Computations
        if thread_params['priority']=='drag':
            J_main=J_drag
            v_main=v_drag
            J_aux=J_pins
            v_aux=v_pins
        else:
            J_main=J_pins
            v_main=v_pins
            J_aux=J_drag
            v_aux=v_drag

        J_aux=concat([J_aux,J_limits])
        v_aux=concat([v_aux,v_limits])

        if J_main!=None:
            J_main_star=dot(J_main,Q0inv)
            J_main_star_dash=linalg.pinv(J_main_star)
            J_weighted_pinv=dot(Q0inv,J_main_star_dash)
            thetad_0=dot(J_weighted_pinv,v_main)
            W=eye(nvar)-dot(J_weighted_pinv,J_main)
        else:
            thetad_0=zeros(nvar)
            W=eye(nvar)


        v_aux_0=dot(J_aux,thetad_0)
        S=dot(J_aux,W)
        [ms,ns]=shape(S)

        delta_v_aux=v_aux-v_aux_0
        Sstar=dot(transpose(S),linalg.inv(dot(S,transpose(S))+K_sr*eye(ms)))
        y=dot(Sstar,delta_v_aux)

        thetad=thetad_0+dot(W,y)

        HRP4.SetConfig(robot,HRP4.GetConfig(robot)+thetad)

        #Update the positions of the spheres
        for i in range(robot.GetDOF()):
            if not (i in thread_params['exclude_list']):
                UpdateSphere(i)

        #Draw the COM
        if thread_params['draw_com']:
            #try:
            #    params['com_handle']=None
            #except AttributeError:
            #    pass            
            CoM=ZMP.ComputeCOM([robot.GetLinks()[0].GetTransform()[0:3,3],robot.GetDOFValues()],{'robot':robot,'exclude_list':[]})
            CoM_proj=zeros(3)
            CoM_proj[0]=CoM[0]
            CoM_proj[1]=CoM[1]
            thread_params['com_handle']=robot.GetEnv().drawlinestrip(array([CoM,CoM_proj]),5)

        time.sleep(thread_params['timestep'])
