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
# random code that helps with debugging/testing the python interfaces and examples
# this is not meant to be run by normal users
from __future__ import with_statement # for python 2.5
__copyright__ = 'Copyright (C) 2009-2010'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from numpy import *
from itertools import izip

def test_drawjoints():
    """draws the joint axes of the robot
    """
    env = robot.GetEnv()
    while True:
        h = None
        h = [env.drawlinelist(array([j.GetAnchor()-j.GetAxis(0),j.GetAnchor()+j.GetAxis(0)]),5,array([0,0,float(j.GetDOFIndex())/robot.GetDOF()]))  for j in robot.GetJoints()]
        time.sleep(0.1)

    while True:
        h = None
        joints = [robot.GetJoints()[i] for i in robot.GetManipulator('arm').GetArmJoints()]
        h = [env.drawlinelist(array([j.GetAnchor()-j.GetAxis(0),j.GetAnchor()+j.GetAxis(0)]),5,array([0,0,i/8.0]))  for i,j in enumerate(joints)]
        time.sleep(0.1)

def derive_normalizeAxisRotation():
    """Find the rotation theta around axis v such that rot(v,theta) * q is closest to the identity"""
    from sympy import *
    vx,vy,vz = Symbol('vx'),Symbol('vy'),Symbol('vz')
    v = Matrix(3,1,[vx,vy,vz])
    theta = Symbol('theta')
    q0 = Matrix(4,1,[cos(theta/2),sin(theta/2)*v[0],sin(theta/2)*v[1],sin(theta/2)*v[2]])
    q0dtheta = Matrix(4,1,[-sin(theta/2)/2,cos(theta/2)*v[0]/2,cos(theta/2)*v[1]/2,cos(theta/2)*v[2]/2])
    qx,qy,qz,qw = Symbol('qx'),Symbol('qy'),Symbol('qz'),Symbol('qw')
    q1 = Matrix(4,1,[qx,qy,qz,qw])
    qidentity = Matrix(4,1,[S.One,S.Zero,S.Zero,S.Zero])
    qfinal = Matrix(4,1,[q0[0]*q1[0] - q0[1]*q1[1] - q0[2]*q1[2] - q0[3]*q1[3],
                         q0[0]*q1[1] + q0[1]*q1[0] + q0[2]*q1[3] - q0[3]*q1[2],
                         q0[0]*q1[2] + q0[2]*q1[0] + q0[3]*q1[1] - q0[1]*q1[3],
                         q0[0]*q1[3] + q0[3]*q1[0] + q0[1]*q1[2] - q0[2]*q1[1]])
    qfinaldtheta = Matrix(4,1,[q0dtheta[0]*q1[0] - q0dtheta[1]*q1[1] - q0dtheta[2]*q1[2] - q0dtheta[3]*q1[3],
                         q0dtheta[0]*q1[1] + q0dtheta[1]*q1[0] + q0dtheta[2]*q1[3] - q0dtheta[3]*q1[2],
                         q0dtheta[0]*q1[2] + q0dtheta[2]*q1[0] + q0dtheta[3]*q1[1] - q0dtheta[1]*q1[3],
                         q0dtheta[0]*q1[3] + q0dtheta[3]*q1[0] + q0dtheta[1]*q1[2] - q0dtheta[2]*q1[1]])
    solveeq = qfinaldtheta.dot(qidentity-qfinal).expand()

    sthetad2 = Symbol('sthetad2') # sin(theta/2)
    cthetad2 = Symbol('cthetad2') # cos(theta/2)
    finaleq = Poly(solveeq.subs([(sin(theta/2),sthetad2),(cos(theta/2),cthetad2)]),sthetad2,cthetad2)
    # should be:
    # Poly((qw**2/2 + qx**2/2 + qy**2/2 + qz**2/2 - qw**2*vx**2/2 - qw**2*vy**2/2 - qw**2*vz**2/2 - qx**2*vx**2/2 - qx**2*vy**2/2 - qx**2*vz**2/2 - qy**2*vx**2/2 - qy**2*vy**2/2 - qy**2*vz**2/2 - qz**2*vx**2/2 - qz**2*vy**2/2 - qz**2*vz**2/2)*sthetad2*cthetad2 - qx/2*sthetad2 + (-qw*vz/2 - qy*vx/2 - qz*vy/2)*cthetad2, sthetad2, cthetad2)
    # sthetad2*cthetad2 coefficient reduces to 0
    # Poly(- qx/2*sthetad2 + (-qw*vz/2 - qy*vx/2 - qz*vy/2)*cthetad2, sthetad2, cthetad2)
    # theta = 2*atan2(-qw*vz-qz*vy-qy*vx,qx)
    
