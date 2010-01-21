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
def test_jacobian():
    """tests if jacobians work"""
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barretthand.robot.xml')
    env.AddRobot(robot)
    fingertip_global = array([-0.15,0,0.2])
    # get a link name
    link = robot.GetLink('Finger2-2')
    fingertip_local = dot(linalg.inv(link.GetTransform()),r_[fingertip_global,1])[0:3]
    J = robot.CalculateJacobian(link.GetIndex(),fingertip)
    
    # move each joint a little
    curvalues = robot.GetJointValues()
    for iter in range(100):
        robot.SetJointValues(curvalues + (random.randint(0,2,len(curvalues))-0.5)*0.03)
        fingertip_realdir = dot(link.GetTransform(),r_[fingertip_local,1])[0:3] - fingertip_global
        deltavalues = robot.GetJointValues()-curvalues
        fingertip_estimatedir = dot(J,deltavalues)
        dreal = fingertip_realdir/sqrt(sum(fingertip_realdir**2))
        destimate = fingertip_estimatedir/sqrt(sum(fingertip_estimatedir**2))
        if dot(dreal,destimate) < 0.99:
            print deltavalues
            raise ValueError('bad jacobian: %f'%dot(dreal,destimate))
