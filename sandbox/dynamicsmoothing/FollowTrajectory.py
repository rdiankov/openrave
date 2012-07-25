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
import Trajectory

norm=linalg.norm


################### From file ####################

headf={'link_index':18,'index_in_file':2}
headr={'link_index':18,'index_in_file':5}
stern={'link_index':16,'index_in_file':92}

rsho={'link_index':20,'index_in_file':8}
relb={'link_index':21,'index_in_file':11}
rwrist={'link_index':22,'index_in_file':17}
rpel={'link_index':0,'index_in_file':41}
rhip={'link_index':3,'index_in_file':44}
rknee={'link_index':3,'index_in_file':47}
rank={'link_index':7,'index_in_file':50}
rheel={'link_index':7,'index_in_file':53}
rthumb={'link_index':7,'index_in_file':59}

lsho={'link_index':37,'index_in_file':26}
lelb={'link_index':38,'index_in_file':29}
lwrist={'link_index':39,'index_in_file':32}
lpel={'link_index':0,'index_in_file':62}
lhip={'link_index':10,'index_in_file':65}
lknee={'link_index':10,'index_in_file':68}
lank={'link_index':14,'index_in_file':71}
lheel={'link_index':14,'index_in_file':74}
lthumb={'link_index':14,'index_in_file':80}


#markers_list=[headf,headr,stern,rsho,relb,rwrist,rpel,rhip,rknee,rank,rheel,rthumb,lsho,lelb,lwrist,lpel,lhip,lknee,lank,lheel,lthumb]

markers_list=[headf,headr,stern,rsho,relb,rwrist,rpel,lsho,lelb,lwrist,lpel]

segments_list=[[headf,headr],[rsho,relb,rwrist],[lsho,lelb,lwrist],[rpel,rhip,rknee,rank],[lpel,lhip,lknee,lank],[rheel,rank,rthumb],[lheel,lank,lthumb]]


def plot_markers(env,q,frame,markers_list,segments_list):
    handle1=[]
    handle2=[]
    for marker in markers_list:
        i=marker['index_in_file']
        handle1.append(env.plot3(q[frame,[i,i+1,i+2]],10))
    for segment in segments_list:
        line=[]
        for marker in segment:
            i=marker['index_in_file']
            line.append(q[frame,[i,i+1,i+2]])
            handle2.append(env.drawlinestrip(array(line),5))
    return (handle1,handle2)



def compute_local_positions(robot,q,frame,incr,markers_list):
    (n_steps,dim)=q.shape
    for marker in markers_list:
        link=robot.GetLinks()[marker['link_index']]
        i=marker['index_in_file']
        global_pos=q[frame,[i,i+1,i+2]]
        local_pos=linalg.solve(link.GetTransform(),add1(global_pos))[0:3]
        marker['local_pos']=local_pos        
        marker['p_vect']=transpose(q[:,[i,i+1,i+2]][range(0,n_steps,incr),:])




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
    if params['activate_base_translations']:
        J[:,0:3]=eye(3) #Jacobian with respect to base link translations
    if params['activate_base_rotations']:
        J[:,3:6]=euler_jacobian(euler,u) #Jacobian with respect to base link rotations
    J[:,6:n+6]=J0 #Jacobian with respect to joint angle changes

    return J


def concat(mat_list):
    if mat_list[0]==None:
        return concatenate(mat_list[1:len(mat_list)])
    return concatenate(mat_list)
       
def add1(v):
    res=zeros(4)
    res[0:3]=v    
    res[3]=1
    return res


def Follow(markers,params):
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

    n_steps=shape(markers[0]['p_vect'])[1]
    res=zeros((n_var,n_steps))
    res[:,0]=HRP4.GetConfig(robot)
    cur_config=res[:,0]

    with robot:

        for step in range(n_steps-1):
            T=baselink.GetTransform()
            euler=HRP4.mat2euler(T[0:3,0:3])

            # Compute the Jacobians and desired velocities of the markers to follow
            J_markers=None
            v_desired=None
            for j in range(len(markers)):
                linkj=robot.GetLinks()[markers[j]['link_index']]
                p_cur=dot(linkj.GetTransform(),add1(markers[j]['local_pos']))[0:3]
                J_markers=concat([J_markers,Jacobian(euler,markers[j]['link_index'],p_cur,params)])
                p_vect=markers[j]['p_vect']
                v_ref=p_vect[:,step+1]-p_vect[:,step]
                v_desired=concat([v_desired,K_v*v_ref+dot(K_p,p_vect[:,step+1]-p_cur)])

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
            J_limits=concat([J_lower,J_upper])
            v_limits=concat([v_lower,v_upper])

            # Inverse kinematics computations
            # J_main=J_markers
            # v_main=v_desired
            # J_aux=J_limits
            # v_aux=v_limits

            # J_main_star=dot(J_main,Q0inv)
            # J_main_star_dash=linalg.pinv(J_main_star)
            # J_weighted_pinv=dot(Q0inv,J_main_star_dash)
            # thetad_0=dot(J_weighted_pinv,v_main)
            # W=eye(n_var)-dot(J_weighted_pinv,J_main)

            # v_aux_0=dot(J_aux,thetad_0)
            # S=dot(J_aux,W)
            # [ms,ns]=shape(S)

            # delta_v_aux=v_aux-v_aux_0
            # Sstar=dot(transpose(S),linalg.inv(dot(S,transpose(S))+K_sr*eye(ms)))
            # y=dot(Sstar,delta_v_aux)

            # thetad=thetad_0+dot(W,y)


            J_main=concat([J_markers,J_limits])
            v_main=concat([v_desired,v_limits])

            J_main_star=dot(J_main,Q0inv)
            J_main_star_dash=linalg.pinv(J_main_star)
            J_weighted_pinv=dot(Q0inv,J_main_star_dash)
            thetad_0=dot(J_weighted_pinv,v_main)

            thetad=thetad_0

            cur_config=cur_config+thetad
            res[:,step+1]=cur_config
            HRP4.SetConfig(robot,cur_config)

    return Trajectory.SampleTrajectory(res)


def Reach(markers,milestones,n_steps,params):
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

    with robot:

        for kk in range(len(milestones)):

            for step in range(n_steps):
                T=baselink.GetTransform()
                euler=HRP4.mat2euler(T[0:3,0:3])

                # Compute the Jacobians and desired velocities of the markers to follow
                J_markers=None
                v_desired=None
                for j in range(len(markers)):
                    # Jacobian
                    linkj=robot.GetLinks()[markers[j]['link_index']]
                    p_cur=dot(linkj.GetTransform(),add1(markers[j]['local_pos']))[0:3]
                    J_markers=concat([J_markers,Jacobian(euler,markers[j]['link_index'],p_cur,params)])

                    # velocity
                    p_end=markers[j]['p_vect'][:,milestones[kk]]
                    v_desired=concat([v_desired,(p_end-p_cur)/(n_steps-step)])

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
                J_limits=concat([J_lower,J_upper])
                v_limits=concat([v_lower,v_upper])

                # Inverse kinematics computations
                # J_main=J_markers
                # v_main=v_desired
                # J_aux=J_limits
                # v_aux=v_limits

                # J_main_star=dot(J_main,Q0inv)
                # J_main_star_dash=linalg.pinv(J_main_star)
                # J_weighted_pinv=dot(Q0inv,J_main_star_dash)
                # thetad_0=dot(J_weighted_pinv,v_main)
                # W=eye(n_var)-dot(J_weighted_pinv,J_main)

                # v_aux_0=dot(J_aux,thetad_0)
                # S=dot(J_aux,W)
                # [ms,ns]=shape(S)

                # delta_v_aux=v_aux-v_aux_0
                # Sstar=dot(transpose(S),linalg.inv(dot(S,transpose(S))+K_sr*eye(ms)))
                # y=dot(Sstar,delta_v_aux)

                # thetad=thetad_0+dot(W,y)


                J_main=concat([J_markers,J_limits])
                v_main=concat([v_desired,v_limits])

                J_main_star=dot(J_main,Q0inv)
                J_main_star_dash=linalg.pinv(J_main_star)
                J_weighted_pinv=dot(Q0inv,J_main_star_dash)
                thetad_0=dot(J_weighted_pinv,v_main)

                thetad=thetad_0

                cur_config=cur_config+thetad
                res.append(cur_config)
                HRP4.SetConfig(robot,cur_config)

    return Trajectory.SampleTrajectory(transpose(array(res)))


