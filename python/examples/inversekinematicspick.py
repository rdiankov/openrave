#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
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
"""Creates an object whose transform is used to solve IK solutions for each robot manipulator.

.. examplepre-block:: inversekinematicspick

Description
-----------

Will create a picker for each manipulator in each robot. Can change the scene with:

.. code-block:: bash

  openrave.py --example inversekinematicspick --scene=data/pr2test1.env.xml

Can also change the inverse kinematics type to use by:

.. code-block:: bash

  openrave.py --example inversekinematicspick --scene=data/katanatable.env.xml --iktype=translation3d

.. examplepost-block:: inversekinematicspick
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import threading
import time
import openravepy
from openravepy import databases
from openravepy import IkParameterizationType, IkParameterization, RaveCreateKinBody, raveLogInfo, raveLogWarn, IkFilterOptions
from numpy import array, zeros

def main(env,options):
    "Main example code."
    if options.iktype is not None:
        # cannot use .names due to python 2.5 (or is it boost version?)
        for value,type in IkParameterization.Type.values.iteritems():
            if type.name.lower() == options.iktype.lower():
                iktype = type
                break
    else:
        iktype = IkParameterizationType.Transform6D
        
    env.Load(options.scene)
    with env:
        # load the Transform6D IK models
        ikmodels = []
        for robot in env.GetRobots():
            for manip in robot.GetManipulators():
                print manip
                try:
                    robot.SetActiveManipulator(manip)
                    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=iktype)
                    if not ikmodel.load():
                        ikmodel.autogenerate()
                        if not ikmodel.has():
                            continue
                    ikmodels.append(ikmodel)
                except Exception, e:
                    print 'failed manip %s'%manip, e
                
        if len(ikmodels) == 0:
            raveLogWarn('no manipulators found that can be loaded with iktype %s'%str(iktype))
            
        # create the pickers
        pickers = []
        for ikmodel in ikmodels:
            raveLogInfo('creating picker for %s'%str(ikmodel.manip))
            picker = RaveCreateKinBody(env,'')
            picker.InitFromBoxes(array([ [0.05,0,0,0.05,0.01,0.01], [0,0.05,0,0.01,0.05,0.01], [0,0,0.05,0.01,0.01,0.05] ]),True)
            for igeom,geom in enumerate(picker.GetLinks()[0].GetGeometries()):
                color = zeros(3)
                color[igeom] = 1
                geom.SetDiffuseColor(color)
            picker.SetName(ikmodel.manip.GetName())
            env.Add(picker,True)
            # have to disable since not part of collision 
            picker.Enable(False)
            picker.SetTransform(ikmodel.manip.GetTransform())
            pickers.append([picker,ikmodel.manip])
              
    raveLogInfo('pickers loaded, try moving them')
    def PickerThread(env,pickers,iktype):
        """this function runs in a separate thread monitoring each of the pickers
        """
        Tpickers = [None]*len(pickers)
        while True:
            if env.GetViewer() is None:
                break
            with env:
                for ipicker,(picker,manip) in enumerate(pickers):
                    T=picker.GetTransform()
                    if Tpickers[ipicker] is not None and sum(abs(Tpickers[ipicker]-T).flatten()) <= 1e-10:
                        continue
                    data = None
                    if iktype == IkParameterizationType.Direction3D:
                        data = T[0:3,2]
                    elif iktype == IkParameterizationType.Lookat3D:
                        data = T[0:3,3]
                    elif iktype == IkParameterizationType.Ray4D:
                        data = Ray(T[0:3,3],T[0:3,2])
                    elif iktype == IkParameterizationType.Rotation3D:
                        data = T[0:3,0:3]
                    elif iktype == IkParameterizationType.Transform6D:
                        data = T
                    elif iktype == IkParameterizationType.Translation3D:
                        data = T[0:3,3]
                    elif iktype == IkParameterizationType.TranslationDirection5D:
                        ikparam = Ray(T[0:3,3],T[0:3,2])
                    elif iktype == IkParameterizationType.TranslationLocalGlobal6D:
                        ikparam = [T[0:3,3],zeros(3)]
                    else:
                        raveLogWarn('iktype %s unsupported'%str(iktype))
                        continue
                    sol = manip.FindIKSolution(IkParameterization(data,iktype),IkFilterOptions.CheckEnvCollisions)
                    if sol is not None:
                        robot.SetDOFValues(sol,manip.GetArmIndices())
                        # have to update all other pickers since two manipulators can be connected to the same joints (torso)
                        for ipicker2, (picker2,manip2) in enumerate(pickers):
                            Tpickers[ipicker2] = manip2.GetTransform()
                            picker2.SetTransform(manip2.GetTransform())
                    # changing color produces bugs with qtcoin
#                     for igeom,geom in enumerate(picker.GetLinks()[0].GetGeometries()):
#                         color = zeros(3)
#                         color[igeom] = 0.4 if sol is None else 1.0
#                         geom.SetDiffuseColor(color)
                    Tpickers[ipicker] = T

            # update rate of 0.05s
            time.sleep(0.05)

    # create the thread
    t = threading.Thread(target=PickerThread,args=[env,pickers,iktype])
    t.start()
    while True:
        t.join(1)
    
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments

@openravepy.with_destroy
def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Shows how to generate a 6D inverse kinematics solver and use it for getting all solutions.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--scene',action="store",type='string',dest='scene',default='data/lab1.env.xml',
                      help='Scene file to load (default=%default)')
    parser.add_option('--iktype', action='store',type='string',dest='iktype',default=None,
                      help='The ik type to build the solver current types are: %s'%(', '.join(iktype.name for iktype in IkParameterization.Type.values.values())))
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)

if __name__ == "__main__":
    run()
