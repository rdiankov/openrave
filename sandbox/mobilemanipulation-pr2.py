#!/usr/bin/env python
# Copyright (c) 2010 Rosen Diankov (rosen.diankov@gmail.com)
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
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from openravepy import *
from numpy import *
import numpy,time,scipy
from openravepy.examples import mobilemanipulation
from itertools import izip

def mm_create(robot,useclone=False):
    print 'creating mobile manipulation planners'
    env=robot.GetEnv()
    manips = [robot.GetManipulator('leftarm'), robot.GetManipulator('leftarm_torso'), robot.GetManipulator('rightarm'), robot.GetManipulator('rightarm_torso')]
    irmodels = []
    with robot:
        for manip in manips:
            robot.SetActiveManipulator(manip)
            dofindices = inversereachability.InverseReachabilityModel.getdofindices(manip)
            for id,value in [('0',[0]),('43',[0.43]),('n43',[-0.43])]:
                robot.SetJointValues(value,dofindices)
                irmodel = inversereachability.InverseReachabilityModel(robot=robot,id=id)
                if irmodel.load():
                    irmodels.append(irmodel)
                else:
                    print 'failed to load irmodel',manip.GetName(),id

    switchpatterns=[('frootloops(\d)*','data/box_frootloops_fat.kinbody.xml')]
    mm = mobilemanipulation.MobileManipulationPlanning(robot,grmodel=mobilemanipulation.GraspReachability(robot,irmodels=irmodels),maxvelmult=0.3,switchpatterns=switchpatterns)
    with env:
        if not useclone:
            return mm
        envclone = env.CloneSelf(CloningOptions.Bodies)
        mmclone = mm.clone(envclone)
        # create a redirect controller
        with envclone:
            mmclone.robot.SetController(envclone.CreateController('RedirectController'),'')
            mmclone.robot.GetController().Init(robot,'')
        mmclone.mm=mm
        return mmclone

if __name__ == "__main__":
    parser = OpenRAVEModel.CreateOptionParser()
    parser.description='Opens the openrave scene'
    parser.add_option('--scene',action="store",type='string',dest='scene',default='scenes/r602.env.xml',
                      help='offset from first link (default=%default)')
    parser.add_option('--nomobile',action="store_false",dest='createmobile',default=True,
                      help='if set, will not create any mobile manipulation objects (default=%default)')
    (options, args) = parser.parse_args()
    env = Environment()
    try:
        env.SetViewer('qtcoin')
        robot = hrp.start_real(env,scene=options.scene,mapframe=options.mapframe)
        with robot:
            origjointvalues = robot.GetJointValues()
            neutraljointvalues = array([ -4.22322080e-02,   1.06378300e-02,   0.8,-8.32389624e-09,   7.54900054e-09,  -2.1,-1.27,   1.08494973e+00,   0.00000000e+00,0.8,   0.00000000e+00,   0.00000000e+00,-2.1,   1.27,  -9.19997454e-01,-4.55254430e-08,   6.17270023e-02,   6.60099387e-01,-1.74536079e-01,   2.59349197e-01])
        if options.createmobile:
            mm = hrp.mm_create(robot,rosnavigation=False,useclone=False)
        else:
            mm = None
        while True:
            cmd = raw_input('Enter command (q-quit,n-neutral position,d-delete objects,g-grasp,c-reset controller,r-release,i-ipython,p-place,e-enable sensors,s-search objects,d-delete objects,j-jitter robot,m-mobile manipulation,h-head camera): ')
            try:
                if cmd == 'q':
                    break
                elif cmd == 'i':
                    from IPython.Shell import IPShellEmbed
                    ipshell = IPShellEmbed(argv='',banner = 'Dropping into IPython',exit_msg = 'Leaving Interpreter, back to program.')
                    ipshell(local_ns=locals())
                elif cmd == 'n':
                    print 'moving to neutral position'
                    with env:
                        robot.RegrabAll()
                    mm.moveToNeutral(neutraljointvalues=neutraljointvalues,bounds=array(((-0.15,-0.1,-0.05),(0.2,0.1,0.15))))
                    mm.moveWithFreeArms([0],[1])
                elif cmd == 'j':
                    with env:
                        mm.robot.SetActiveDOFs(range(mm.robot.GetDOF()))
                        try:
                            mm.basemanip.JitterActive(jitter=0.08)
                        except planning_error:
                            mm.basemanip.JitterActive(jitter=0.12)
                    mm.waitrobot()
                elif cmd == 'r':
                    mm.releasefingers()
                    mm.robot.ReleaseAllGrabbed()
                elif cmd == 'd':
                    with mm.env:
                        # get all graspable objects
                        for gmodel in mm.grmodel.getGraspables():
                            mm.env.RemoveKinBody(gmodel.target)
                elif cmd == 'g':
                    # stop any torque control
                    envclone = env.CloneSelf(CloningOptions.Bodies)
                    try:
                        with env: # create a redirect controller
                            mmclone = mm.clone(envclone)
                            with envclone:
                                mmclone.robot.SetController(envclone.CreateController('RedirectController'),'')
                                mmclone.robot.GetController().Init(robot,'')
                        usevisibilitycamera = {'sensorname':'l_forearm_cam_optical_frame','storeimage':True,'dosync':True,'ask':False,'maxdist':0.6,'syncdelay':5.0}
                        allgmodels = mmclone.grmodel.getGraspables()[::-1]
                        mmclone.robot.SetActiveManipulator('leftarm_torso')
                        gmodel=mmclone.graspObjectWithModels(allgmodels=allgmodels,usevisibilitycamera=usevisibilitycamera,neutraljointvalues=neutraljointvalues)
                        with env:
                            robot.RegrabAll()
                            robot.SetActiveManipulator(mmclone.robot.GetActiveManipulator())
                        print 'moving to neutral position'
                        mmclone.moveToNeutral(neutraljointvalues=neutraljointvalues,bounds=array(((-0.15,-0.1,-0.05),(0.2,0.1,0.15))))
                        if 'image' in usevisibilitycamera:
                            print 'saving camera image to cameraimage.png'
                            scipy.misc.pilutil.imsave('cameraimage.png',usevisibilitycamera['image'])
                    finally:
                        envclone.Destroy()
                elif cmd == 'm':
                    # stop any torque control
                    envclone = env.CloneSelf(CloningOptions.Bodies)
                    try:
                        with env: # create a redirect controller
                            mmclone = mm.clone(envclone)
                            with envclone:
                                mmclone.robot.SetController(envclone.CreateController('RedirectController'),'')
                                mmclone.robot.GetController().Init(robot,'')

                        usevisibilitycamera = {'sensorname':'l_forearm_cam_optical_frame','storeimage':True,'dosync':True,'ask':False,'maxdist':0.5,'syncdelay':5.0}
                        # pick up the object with visibility checking
                        targets = mmclone.grmodel.setGraspReachability(possiblemanips=[mmclone.robot.GetManipulator('leftarm_torso')])
                        gmodel = mmclone.graspObjectMobileSearch(usevisibilitycamera=usevisibilitycamera,neutraljointvalues=neutraljointvalues)
                        with env:
                            robot.RegrabAll()
                            robot.SetActiveManipulator(mmclone.robot.GetActiveManipulator())
                        print 'moving to neutral position'
                        mmclone.moveToNeutral(neutraljointvalues=neutraljointvalues,bounds=array(((-0.15,-0.1,-0.05),(0.2,0.1,0.15))))
                    finally:
                        envclone.Destroy()
                elif cmd == 'p':
                    with mm.env:
                        if len(mm.robot.GetGrabbed()) == 0:
                            raise planning_error('no grabbed objects')
                        robot.RegrabAll()

                    envclone = env.CloneSelf(CloningOptions.Bodies)
                    try:
                        with env: # create a redirect controller
                            mmclone = mm.clone(envclone)
                            with envclone:
                                mmclone.robot.SetController(envclone.CreateController('RedirectController'),'')
                                mmclone.robot.GetController().Init(robot,'')
                        target=mmclone.robot.GetGrabbed()[0]
                        try:
                            mmclone.placeObject(target=target,dests=mmclone.getDests(target,maxdist=0.9))
                            try:
                                mmclone.moveToNeutral(neutraljointvalues=neutraljointvalues,bounds=array(((-0.15,-0.1,-0.05),(0.2,0.1,0.15))))
                            except planning_error,e:
                                print e
                            break
                        except planning_error:
                            pass
                    finally:
                        envclone.Destroy()
            except planning_error,e:
                print 'script planning error',e
            except openrave_exception,e:
                print 'script planning error',e
            except KeyboardInterrupt,e:
                print 'cancelled by keyboard',e
    finally:
        env.Destroy()
