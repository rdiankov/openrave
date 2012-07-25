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


def euler_jacobian(euler,u):
    gamma=euler[0] #yaw
    beta=euler[1] #pitch
    alpha=euler[2] #roll
    J=zeros((3,3))
    J[:,0]=dot(HRP4.euler2mat_dgamma(euler),u)
    J[:,1]=dot(HRP4.euler2mat_dbeta(euler),u)
    J[:,2]=dot(HRP4.euler2mat_dalpha(euler),u)
    return J



def Jacobian(euler,linkindex,position,params):
    robot=params['robot']
    base_link=robot.GetLinks()[0]
    with robot:
        J0=robot.CalculateJacobian(linkindex,position)
        u=position-base_link.GetTransform()[0:3,3]
        
    (k,n)=shape(J0)
    J=zeros((3,n+6))
    J[:,0:3]=eye(3) #Jacobian with respect to base link translations
    J[:,3:6]=euler_jacobian(euler,u) #Jacobian with respect to base link rotations
    J[:,6:n+6]=J0 #Jacobian with respect to joint angle changes

    return J


def concat(mat_list):
    if mat_list[0]==None:
        return concatenate(mat_list[1:len(mat_list)])
    return concatenate(mat_list)
       



def Follow(markers,p_vect,params):
    robot=params['robot']
    p_step=params['p_step']
    Q0=params['Q0']
    Q0inv=params['Q0inv']
    lower_lim=params['lower_lim']
    upper_lim=params['upper_lim']
    K_p=params['K_p']
    K_li=params['K_li']
    K_sr=params['K_sr']

    

    ndof=robot.GetDOF()

    baselink=robot.GetLinks()[0]
    T=baselink.GetTransform()
    euler=HRP4.mat2euler(T[0:3,0:3])

    for steps in range(int(n_steps)+1):
        p_cur=link.GetGlobalCOM()

        # Compute the links Jacobian
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
        if params['priority']=='drag':
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
            if not (i in params['exclude_list']):
                UpdateSphere(i)

        #Draw the COM
        if params['draw_com']:
            #try:
            #    params['com_handle']=None
            #except AttributeError:
            #    pass            
            CoM=ZMP.ComputeCOM([robot.GetLinks()[0].GetTransform()[0:3,3],robot.GetDOFValues()],{'robot':robot,'exclude_list':[]})
            CoM_proj=zeros(3)
            CoM_proj[0]=CoM[0]
            CoM_proj[1]=CoM[1]
            params['com_handle']=robot.GetEnv().drawlinestrip(array([CoM,CoM_proj]),5)

        time.sleep(params['timestep'])
