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
Functions to compute the COM and the ZMP
For now, the base link moves in translation only, no rotation allowed
"""

from openravepy import *
from numpy import *





def v2t(v):
    T=eye(4)
    T[0:3,3]=v
    return T


##########################################################################

def ComputeJacobians(config,delta,i,params):

    base_T=config[0]
    base_vel=config[1] # Not used here
    base_acc=config[2] # Not used here

    q=config[3]
    qd=config[4]
    qdd=config[5]

    robot=params['robot']
    base_link=robot.GetLinks()[0]

    with robot:
        base_link.SetTransform(v2t(base_T))
        robot.SetDOFValues(q)
        rpq_0=robot.CalculateJacobian(i,robot.GetLinks()[i].GetGlobalCOM())
        ro_0=robot.CalculateAngularVelocityJacobian(i)

    n=len(q)
    norm_qd=numpy.linalg.norm(qd)
    if norm_qd<1e-10:
        rpqqfs=zeros((3,n))
        roqfs=zeros((3,n))
    else:
        qdunit=delta/norm_qd*qd
        with robot:
            robot.SetDOFValues(q+qdunit)
            rpqqfs=norm_qd/delta*(robot.CalculateJacobian(i,robot.GetLinks()[i].GetGlobalCOM())-rpq_0)
            roqfs=norm_qd/delta*(robot.CalculateAngularVelocityJacobian(i)-ro_0)
    
    return [rpq_0,ro_0,rpqqfs,roqfs]





##########################################################################

def ComputeCOM(config,params):
    base_T=config[0]
    q=config[1]
    robot=params['robot']
    exclude_list=params['exclude_list']
    base_link=robot.GetLinks()[0]
    n=len(q)

    with robot:
        base_link.SetTransform(v2t(base_T))
        robot.SetDOFValues(q)
        com_pos=array([k.GetGlobalCOM() for k in robot.GetLinks()])
        masses=[k.GetMass() for k in robot.GetLinks()]
     
    M=sum(masses)
    weighted_com=zeros(3)
    for i in range(n+1):
        if i in exclude_list:
            continue
        weighted_com+=masses[i]*array(com_pos[i])

    res=weighted_com/M
    return res

    
def ComputeZMP(config,params):

    base_T=config[0]
    base_vel=config[1]
    base_acc=config[2]

    q=config[3]
    qd=config[4]
    qdd=config[5]

    n=len(q)
    robot=params['robot']
    g=params['gravity']
    moment_coef=params['moment_coef'] # Usually =1, sometimes =0 for testing purpose
    exclude_list=params['exclude_list']

    base_link=robot.GetLinks()[0]

    with robot:
        base_link.SetTransform(v2t(base_T))
        robot.SetDOFValues(q)
        robot.SetDOFVelocities(qd)

        com_pos=array([k.GetGlobalCOM() for k in robot.GetLinks()])
        vel=robot.GetLinkVelocities()
        acc=robot.GetLinkAccelerations(qdd) # Includes gravity term
        for i in range(n):
            vel[i,0:3]=vel[i,0:3]+base_vel
            acc[i,0:3]=acc[i,0:3]+base_acc

        transforms=[k.GetTransform()[0:3,0:3] for k in robot.GetLinks()]
        masses=[k.GetMass() for k in robot.GetLinks()]
        inertiae=[k.GetLocalInertia() for k in robot.GetLinks()]
        localCOM=[k.GetLocalCOM() for k in robot.GetLinks()]

    xnum=0
    ynum=0
    denum=0

    for i in range(n+1):
        
        if i in exclude_list:
            continue

        # Compute the inertia matrix in the global frame
        R=transforms[i]
        Ii=dot(R,dot(inertiae[i],transpose(R)))
        ri=dot(R,localCOM[i])

        # Compute the inertia moment
        omegai=vel[i,3:6]
        omegadi=acc[i,3:6]
        Mi=moment_coef*(dot(Ii,omegadi)+cross(omegai,dot(Ii,omegai)))
 
        com_vel=vel[i,0:3]+cross(omegai,ri)
        com_acc=acc[i,0:3]+cross(omegai,cross(omegai,ri))+cross(omegadi,ri)

        # Extract the position and accelerations
        xi=com_pos[i,0]
        yi=com_pos[i,1]
        zi=com_pos[i,2]
        xddi=com_acc[0]
        yddi=com_acc[1]
        zddi=com_acc[2]

        # Testing purpose
        # print '-------'
        # print i
        # print xddi
        # print yddi
        # print zddi

        # Compute the numerators and denominator
        xnum+=masses[i]*(zddi*xi-xddi*zi)-Mi[1]
        ynum+=masses[i]*(zddi*yi-yddi*zi)-Mi[0]
        denum+=masses[i]*zddi
        
    return array([xnum/denum,ynum/denum])


def ComputeZMPTraj(traj,params_init):
    n_steps=traj.n_steps
    t_vect=traj.t_vect
    q_vect=traj.q_vect
    qd_vect=traj.qd_vect
    qdd_vect=traj.qdd_vect
    zmp_vect=zeros((2,n_steps))
    for i in range(n_steps):
        q=q_vect[:,i]
        qd=qd_vect[:,i]
        qdd=qdd_vect[:,i]
        # Here we assume there is no base link rotation
        zmp=ComputeZMP([q[0:3],qd[0:3],qdd[0:3],q[6:len(q)],qd[6:len(q)],qdd[6:len(q)]],params_init)
        zmp_vect[:,i]=zmp
    return zmp_vect


def ComputeCOMTraj(traj,params_init):
    n_steps=traj.n_steps
    t_vect=traj.t_vect
    q_vect=traj.q_vect
    com_vect=zeros((3,n_steps))
    for i in range(n_steps):
        q=q_vect[:,i]
        # Here we assume there is no base link rotation
        com_vect[:,i]=ComputeCOM([q[0:3],q[6:len(q)]],params_init)

    return com_vect

    


##########################################################################

def ComputeCoefsFractionZMP(config_pure,params):

    base_T=config_pure[0]
    base_s=config_pure[1]
    base_ss=config_pure[2]

    q=config_pure[3]
    qs=config_pure[4]
    qss=config_pure[5]

    n=len(q)
    robot=params['robot']
    g=params['gravity']
    exclude_list=params['exclude_list']
    moment_coef=params['moment_coef']

    base_link=robot.GetLinks()[0]

    #Initialize the coefficients
    ax,bx,cx=0,0,0
    ay,by,cy=0,0,0
    d,e,f=0,0,0
    
    for i in range(n+1):
        if i in exclude_list:
            continue

        params={'linkindex':i,'robot':robot}

        with robot:
            base_link.SetTransform(v2t(base_T))
            robot.SetDOFValues(q)
            LocalI=robot.GetLinks()[i].GetLocalInertia()
            R=robot.GetLinks()[i].GetTransform()[0:3,0:3]
            I=dot(R,dot(LocalI,transpose(R)))
            m=robot.GetLinks()[i].GetMass()
            rp=robot.GetLinks()[i].GetGlobalCOM()

        # Terms in x,y,z
        x=rp[0]
        y=rp[1]
        z=rp[2]

        # Compute the Jacobians
        delta=1e-10
        [rp_q,romega,rp_qqXqs,romega_qXqs]=ComputeJacobians(config_pure,delta,i,params)

        # Terms in xdd, ydd, zdd 
        apdd=dot(rp_q,qs)+base_s
        bpdd=dot(rp_q,qss)+dot(qs,transpose(rp_qqXqs))+base_ss

        axdd=apdd[0]
        bxdd=bpdd[0]
        aydd=apdd[1]
        bydd=bpdd[1]
        azdd=apdd[2]
        bzdd=bpdd[2]

        # Testing purpose
        # sd=1
        # sdd=1
        # print '-------'
        # print i
        # print axdd*sdd+bxdd*sd*sd
        # print aydd*sdd+bydd*sd*sd
        # print azdd*sdd+bzdd*sd*sd
       
        # Terms in omega
        if moment_coef>0:
            romegaXqs=dot(romega,qs)
            aM=dot(dot(I,romega),qs)        
            bM=dot(I,dot(romega,qss)+dot(qs,transpose(romega_qXqs)))+cross(romegaXqs,dot(I,romegaXqs))
            
            aMx=moment_coef*aM[0]
            bMx=moment_coef*bM[0]
            aMy=moment_coef*aM[1]
            bMy=moment_coef*bM[1]

        else:
            [aMx,bMx,aMy,bMy]=[0,0,0,0]
       
        # Computations of the coefficients of the numerators and denominator
        ax+=m*(azdd*x-axdd*z)-aMy
        bx+=m*(bzdd*x-bxdd*z)-bMy
        cx+=m*g*x

        ay+=m*(azdd*y-aydd*z)-aMx
        by+=m*(bzdd*y-bydd*z)-bMx
        cy+=m*g*y

        d+=m*azdd
        e+=m*bzdd
        f+=m*g

    return [ax,bx,cx,ay,by,cy,d,e,f]
