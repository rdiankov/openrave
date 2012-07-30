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
Compute the inverse dynamics of a robot using the Recursive Newton-Euler algorithm
Copied and adapted from Corke
"""


from numpy import *
from DHProcess import *
import Trajectory


class RobotDynamicsTraj():

    ##########################################
    def __init__(self,env,robot,sample_traj):
        self.env=env
        self.robot=robot
        self.sample_traj=sample_traj
        self.grav=grav


    ##########################################
    # Inverse dynamics along the trajectory
    def compute_torques(self):


class RobotDynamics():

    def __init__(self,env,robot,q,grav):
        robot.SetDOFValues(q)
        self.env=env
        self.robot=robot
        self.n=len(q)
        self.grav=grav

    ######################
    # Compute the inertia term for a given acceleration vector
    def inertiaTerm(self,u_acc):
        self.robot.SetDOFVelocities(zeros(self.n)) 
        self.env.GetPhysicsEngine().SetGravity([0,0,0])
        return self.robot.ComputeInverseDynamics(u_acc)

    ######################
    # Compute the coriolis term for a given velocity vector
    def coriolisTerm(self,u_vel):
        self.robot.SetDOFVelocities(u_vel) 
        self.env.GetPhysicsEngine().SetGravity([0,0,0])
        return self.robot.ComputeInverseDynamics(zeros(self.n))

    ######################
    # Compute the gravity term
    def gravTerm(self):
        self.robot.SetDOFVelocities(zeros(self.n)) 
        self.env.GetPhysicsEngine().SetGravity(self.grav)
        return self.robot.ComputeInverseDynamics(zeros(self.n))

    ######################
    # Compute the gravity term
    def rne(self,qd,qdd):
        self.robot.SetDOFVelocities(qd) 
        self.env.GetPhysicsEngine().SetGravity(grav)
        return self.robot.ComputeInverseDynamics(qdd)

