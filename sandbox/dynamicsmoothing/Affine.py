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



from openravepy import *
from numpy import *
from numpy.linalg import *
import Trajectory


# Functions to compute affine deformations


def compute_mat_c1(v_tau,q_tau,q_T,q_Td,v_T=None,v_Td=None):
    
    #Generate a basis with v_tau as the first vector
    n=len(v_tau)
    P=eye(n)
    P[:,0]=v_tau/norm(v_tau)
    [P_ortho,R]=qr(P)
    Q=transpose(P_ortho)

    #Compute the coordinates of the actual and desired vectors in this base
    q1=q_T-q_tau
    q2=q_Td-q_tau
    X1=dot(Q,q1)
    X2=dot(Q,q2)

    #Create the U matrix and its pseudo-inverse
    U=zeros((n,(n-1)*n))
    pad=transpose(X1[1:n])
    for i in range(n):
        U[i,range(i*(n-1),(i+1)*(n-1))]=pad


    #If there is a velocity constraint
    if v_Td!=None:
        V1=dot(Q,v_T)
        V2=dot(Q,v_Td)
        U_v=zeros((n,(n-1)*n))
        pad=transpose(V1[1:n])
        for i in range(n):
            U_v[i,range(i*(n-1),(i+1)*(n-1))]=pad
        U=concatenate((U,U_v))
        X1=concatenate((X1,V1))
        X2=concatenate((X2,V2))


    #Closeness optimization
    pinvU=pinv(U)
    Mstack=dot(pinvU,X2-X1)

    Mb=zeros((n,n))
    for i in range(n):
        Mb[i,range(1,n)]=transpose(Mstack[range(i*(n-1),(i+1)*(n-1))])

    Mb=Mb+eye(n)
    M=dot(P_ortho,dot(Mb,Q))

    return M



def fork(q_vect,tau,M):
    [dim,n_steps]=shape(q_vect)
    q_res=zeros(q_vect.shape)
    for i in range(tau+1):
        q_res[:,i]=q_vect[:,i]
    for i in range(tau+1,n_steps):
        q_res[:,i]=q_vect[:,tau]+dot(M,q_vect[:,i]-q_vect[:,tau])

    return q_res



def deform_traj(traj,tau,T,q_Td,active_dofs,v_T=None,v_Td=None):    
    q_red=traj.q_vect[active_dofs,:]
    v_tau_red=q_red[:,tau+1]-q_red[:,tau]
    q_Td_red=q_Td[active_dofs]
    
    if v_T!=None:
        M=compute_mat_c1(v_tau_red,q_red[:,tau],q_red[:,T],q_Td_red,v_T[active_dofs],v_Td[active_dofs])
    else:
        M=compute_mat_c1(v_tau_red,q_red[:,tau],q_red[:,T],q_Td_red)    

    q_fork_red=fork(q_red,tau,M)
   
    q_vect2=zeros(traj.q_vect.shape)
    for i in range(traj.dim):
        if not (i in active_dofs):
            q_vect2[i,:]=traj.q_vect[i,:]
    for i in range(len(active_dofs)):
        q_vect2[active_dofs[i],:]=q_fork_red[i,:]

    res_traj=Trajectory.SampleTrajectory(q_vect2)
    res_traj.t_vect=traj.t_vect
    return res_traj
        
    
    
