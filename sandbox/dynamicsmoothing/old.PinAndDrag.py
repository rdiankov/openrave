#http://openrave.org/latest_stable/en/main/_modules/openravepy/examples/testviewercallback.html#itemselectioncb
#g=dragsphere.GetLinks()[0].GetGeometries()[0]
#g.SetTransparency(0.5)
#g.SetAmbientColor([0,1,0])



from openravepy import *
from numpy import *
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
    CreateDummySphere([0,-0.4,1.75],'Pin',[1,0,0])
    CreateDummySphere([0,0,1.75],'Unpin',[0,1,0])
    CreateDummySphere([0,0.4,1.75],'DragTranslation',[0,0,1])
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
    print '----'
    print thread_data
    global thread_params
    global thread_lock
    global thread_data
    link_name=clicked_link.GetParent().GetName()
    thread_lock.acquire()
    if link_name.find('dragsphere')>=0:
        selected_link=int(link_name[10:len(link_name)])
        thread_data['dragsphere']=clicked_link.GetParent()
        [current_link,current_type]=thread_data['drag']
        if current_type=='translation' and current_link==selected_link:
            pass
        else:
            thread_data['drag']=[selected_link,'simple']
            UpdateSphere(selected_link,[0,1,0])
    if link_name=='DragTranslation':
        selected_link=thread_data['drag'][0]
        thread_data['drag']=[selected_link,'translation']
        UpdateSphere(selected_link,[0,0,1])
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
        selected_link=thread_data['drag'][0]
        if pin_link==selected_link:
            thread_data['drag']=[selected_link,'simple']
            UpdateSphere(selected_link,[0,1,0])
    print thread_data
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


def Jacobian(linkindex,position):
    global thread_params   
    robot=thread_params['robot']
    with robot:
        J0=robot.CalculateJacobian(linkindex,position)
    (k,n)=shape(J0)
    J=zeros((k,n+3))
    J[:,0:n]=J0
    J[:,n:n+3]=eye(3)

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

        # Compute the dragged link Jacobian
        if drag_type=='translation':
            J_drag_a=Jacobian(drag_link,p_cur)
            J_drag_b=Jacobian(drag_link,p_cur+array([0,0,1]))
            J_drag_c=Jacobian(drag_link,p_cur+array([0,1,0]))
            J_drag=concat([J_drag_a,J_drag_b,J_drag_c])
            (k,n)=shape(J_drag_a)
        else:
            J_drag=Jacobian(drag_link,p_cur)
            (k,n)=shape(J_drag)
           
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
            J_pinned_ia=Jacobian(pinned_i,CoMi)
            J_pinned_ib=Jacobian(pinned_i,CoMi+array([0,0,1]))
            J_pinned_ic=Jacobian(pinned_i,CoMi+array([0,1,0]))
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
        J_lower=zeros((n,n))
        v_lower=zeros(n)
        J_upper=zeros((n,n))
        v_upper=zeros(n)
        for i in lower_list:            
            J_lower[i,i]=1
            v_lower[i]=K_li*(lower_lim[i]-DOFvalues[i])
        for i in upper_list:
            J_upper[i,i]=1
            v_upper[i]=K_li*(upper_lim[i]-DOFvalues[i])
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
            W=eye(n)-dot(J_weighted_pinv,J_main)
        else:
            thetad_0=zeros(n)
            W=eye(n)

        v_aux_0=dot(J_aux,thetad_0)
        S=dot(J_aux,W)
        [ms,ns]=shape(S)

        delta_v_aux=v_aux-v_aux_0
        Sstar=dot(transpose(S),linalg.inv(dot(S,transpose(S))+K_sr*eye(ms)))
        y=dot(Sstar,delta_v_aux)

        thetad=thetad_0+dot(W,y)

        T0=baselink.GetTransform()
        T0[0:3,3]=T0[0:3,3]+thetad[len(thetad)-3:len(thetad)]
        baselink.SetTransform(T0)
        robot.SetDOFValues(robot.GetDOFValues()+thetad[0:len(thetad)-3])

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
