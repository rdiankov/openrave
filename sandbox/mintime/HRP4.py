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
Functions to deal with the HRP4
"""



from numpy import *


halfsitPose = pi/180*array([0,-0.76,-22.02, 41.29, -18.75 ,-0.45,
                                0, 1.15,-21.89, 41.21, -18.74, -1.10,
                                8,0, 0,0, -3,-10,0,-30,0,0,0,
                                0,0, -3, 10,0,-30,0,0,0, 0,0])

halfsitPose2 = pi/180*array([0,-0.76,-22.02, 41.29, -18.75 ,-0.45,
                                0, 1.15,-21.89, 41.21, -18.74, -1.10,
                                8,0, 0,0, -3,-40,0,-30,0,0,0,
                                0,0, -3, 10,0,-30,0,0,0, 0,0])


################## Euler Angles ################################

def euler2mat(euler):
    gamma=euler[0] #yaw
    beta=euler[1] #pitch
    alpha=euler[2] #roll
    ca=cos(alpha)
    sa=sin(alpha)
    cb=cos(beta)
    sb=sin(beta)
    cg=cos(gamma)
    sg=sin(gamma)
    mat=array([[ca*cb,ca*sb*sg-sa*cg,ca*sb*cg+sa*sg],
               [sa*cb,sa*sb*sg+ca*cg,sa*sb*cg-ca*sg],
               [-sb,cb*sg,cb*cg]])
    return mat

def Rgamma(x):
    c=cos(x)
    s=sin(x)
    return array([[1,0,0],
                  [0,c,-s],
                  [0,s,c]])

def Rbeta(x):
    c=cos(x)
    s=sin(x)
    return array([[c,0,s],
                  [0,1,0],
                  [-s,0,c]])

def Ralpha(x):
    c=cos(x)
    s=sin(x)
    return array([[c,-s,0],
                  [s,c,0],
                  [0,0,1]])


def Rgammad(x):
    c=cos(x+pi/2)
    s=sin(x+pi/2)
    return array([[0,0,0],
                  [0,c,-s],
                  [0,s,c]])

def Rbetad(x):
    c=cos(x+pi/2)
    s=sin(x+pi/2)
    return array([[c,0,s],
                  [0,0,0],
                  [-s,0,c]])

def Ralphad(x):
    c=cos(x+pi/2)
    s=sin(x+pi/2)
    return array([[c,-s,0],
                  [s,c,0],
                  [0,0,0]])


def euler2mat_dalpha(euler):
    gamma=euler[0] #yaw
    beta=euler[1] #pitch
    alpha=euler[2] #roll
    return dot(dot(Ralphad(alpha),Rbeta(beta)),Rgamma(gamma))

def euler2mat_dbeta(euler):
    gamma=euler[0] #yaw
    beta=euler[1] #pitch
    alpha=euler[2] #roll
    return dot(dot(Ralpha(alpha),Rbetad(beta)),Rgamma(gamma))

def euler2mat_dgamma(euler):
    gamma=euler[0] #yaw
    beta=euler[1] #pitch
    alpha=euler[2] #roll
    return dot(dot(Ralpha(alpha),Rbeta(beta)),Rgammad(gamma))



def mat2euler(mat):
    gamma=arctan2(mat[2,1],mat[2,2])
    beta=arctan2(-mat[2,0],sqrt(mat[2,1]*mat[2,1]+mat[2,2]*mat[2,2]))
    alpha=arctan2(mat[1,0],mat[0,0])
    return array([gamma,beta,alpha])


####################### Robot config ##############################

def v2t(v):
    T=eye(4)
    T[0:3,0:3]=euler2mat(v[3:6])
    T[0:3,3]=v[0:3]
    return T


def SetConfig(robot,config):
    robot.GetLinks()[0].SetTransform(v2t(config[0:6]))
    robot.SetDOFValues(config[6:len(config)])

def GetConfig(robot):
    config=zeros(56)
    T=robot.GetLinks()[0].GetTransform()
    config[0:3]=T[0:3,3]
    config[3:6]=mat2euler(T[0:3,0:3])
    config[6:56]=robot.GetDOFValues()
    return config



##################### From Files #################################


def make_config_vect(basePos,euler,q):
    (n1,dim)=shape(basePos)
    (n2,dim)=shape(euler)
    (n3,dim)=shape(q)
    n_steps=min([n1,n2,n3])
    dim=56
    config_vect=zeros((dim,n_steps))
    for i in range(n_steps):
        config_vect[0:3,i]=basePos[i,1:4]
        config_vect[3:6,i]=euler[i,1:4]
        config_vect[22:57,i]=q[i,1:35]
    return config_vect


class HRP4robot():

    def __init__(self,robot):
        self.T0=robot.GetLinks()[0].GetTransform()
        self.robot=robot
        self.dim=34

    def setdof(self,q):
        q2=zeros(50)
        q2[16:51]=q
        self.robot.SetDOFValues(q2)

    def getdof(self):
        return self.robot.GetDOFValues()[16:51]


    def setheight(self,h):
        T=self.robot.GetLinks()[0].GetTransform()
        T[2,3]=h
        self.robot.GetLinks()[0].SetTransform(T)
        self.setdof(self.getdof())

    def getheight(self):
        T=self.robot.GetLinks()[0].GetTransform()        
        return T[2,3]
        
    def halfsit(self):
        self.robot.GetLinks()[0].SetTransform(self.T0)
        self.setheight(0.75)
        self.setdof(halfsitPose)

    def init(self):
        self.robot.GetLinks()[0].SetTransform(self.T0)
        self.setdof(zeros(self.dim))

