#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
"""Shows how to use translational inverse kinematics for an arm with few joints.

.. examplepre-block:: tutorial_iktranslation
  :image-width: 640

Description
-----------

There are two different types of translation 3D IKs:

* **Translation3D** - Align the origin of the manipulation coordinate system with the desired world position

* **TranslationLocalGlobal6D** - Align a dynamically chosen position with respect ot the manipulation coordinate system with the desired world position. To see this in action, execute the example with:

.. code-block:: bash

  openrave.py --example tutorial_iktranslation --withlocal

.. examplepost-block:: tutorial_iktranslation
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def main(env,options):
    "Main example code."
    env.Load(options.scene)
    robot = env.GetRobots()[0]
    robot.SetActiveManipulator(options.manipname)

    # generate the ik solver
    iktype = IkParameterization.Type.Translation3D if not options.withlocal else IkParameterization.Type.TranslationLocalGlobal6D
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=iktype,freeindices=robot.GetActiveManipulator().GetArmIndices()[3:])
    if not ikmodel.load():
        ikmodel.autogenerate()

    with env:
        robot2 = env.ReadRobotXMLFile(robot.GetXMLFilename())
        env.Add(robot2)
        T = robot.GetTransform()
        T[0,3] -= 1
        robot2.SetTransform(T)
        robot2.Enable(False)
        robot2.SetActiveManipulator(robot.GetActiveManipulator().GetName())
        for link in robot2.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(0.2)

    while True:
        with env:
            while True:
                target=ikmodel.manip.GetTransform()[0:3,3]+(random.rand(3)-0.5)
                if options.withlocal:
                    localtarget = 0.5*(random.rand(3)-0.5)
                    ikparam = IkParameterization([localtarget,target],IkParameterization.Type.TranslationLocalGlobal6D)
                else:
                    localtarget = zeros(3)
                    ikparam = IkParameterization(target,IkParameterization.Type.Translation3D)
                solutions = ikmodel.manip.FindIKSolutions(ikparam,IkFilterOptions.CheckEnvCollisions)
                if solutions is not None and len(solutions) > 0: # if found, then break
                    break
        h=env.plot3(array([target]),10.0)
        T2 = robot2.GetActiveManipulator().GetTransform()
        h2=env.plot3(dot(T2[0:3,0:3],localtarget)+T2[0:3,3],15.0,[0,1,0])
        for i in random.permutation(len(solutions))[0:min(80,len(solutions))]:
            with env:
                robot.SetDOFValues(solutions[i],ikmodel.manip.GetArmIndices())
                T = ikmodel.manip.GetTransform()
                env.UpdatePublishedBodies()
            time.sleep(0.05)
        h=None
        h2=None

from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how to use different IK solutions for arms with few joints.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/katanatable.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--manipname',action="store",type='string',dest='manipname',default='arm',
                      help='name of manipulator to use (default=%default)')
    parser.add_option('--withlocal',action="store_true",dest='withlocal',default=False,
                      help='If set, will use the TranslationLocalGlobal6D type to further specify the target point in the manipulator coordinate system')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
