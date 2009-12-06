#!/usr/bin/env python
#
# Copyright (C) 2009 Rosen Diankov
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
import sys, os, time, signal, threading, pickle
import numpy,scipy # nice to be able to explicitly call some functions
from numpy import *
from optparse import OptionParser
from openravepy import *

from Tkinter import *
import tkFileDialog
import Image, ImageDraw, ImageTk

sys.path.append('..')
import metaclass
import visibilityprocessing

class CameraViewerGUI(threading.Thread):
    class Container:
        pass
    def __init__(self,sensor,title='Camera Viewer'):
        threading.Thread.__init__(self)
        self.sensor = sensor
        self.title = title
        self.lastid = -1
        self.imagelck = threading.Lock()

    def updateimage(self):
        data = self.sensor.GetSensorData()
        if data is not None and not self.lastid == data.id:
            width = data.imagedata.shape[1]
            height = data.imagedata.shape[0]
            self.imagelck.acquire()
            self.image = Image.frombuffer('RGB',[width,height], data.imagedata.tostring(), 'raw','RGB',0,1)
            self.imagelck.release()

            photo = ImageTk.PhotoImage(self.image)
            if self.container is None:
                self.container = self.Container()
                self.container.width = width
                self.container.height = height
                self.container.main = self.main
                self.container.canvas = Canvas(self.main, width=width, height=height)
                self.container.canvas.pack(expand=1, fill=BOTH)#side=TOP,fill=X)#
                self.container.obr = None

            self.container.canvas.create_image(self.container.width/2, self.container.height/2, image=photo)
            self.container.obr = photo
            self.lastid = data.id
        self.main.after(100,self.updateimage)

    def saveimage(self,filename):
        self.imagelck.acquire()
        self.image.save(filename)
        self.imagelck.release()

    def run(self):
        self.main = Tk()
        self.main.title(self.title)      # window title
        self.main.resizable(width=True, height=True)
        self.container = None
        self.main.after(0,self.updateimage)
        self.main.mainloop()

class HRP2GraspingScene(metaclass.AutoReloader):
    def __init__(self):
        self.orenv = Environment()
        self.orenv.SetViewer('qtcoin')

    def loadscene(self,scenefilename='scenes/r602cerealmanip.env.xml',robotname='HRP2JSK',graspingppfile=None):
        self.orenv.Reset()
        if not self.orenv.Load(scenefilename):
            raise ValueError('failed to open %s openrave file'%scenefilename)
        if len(self.orenv.GetRobots()) == 0:
            raise ValueError('no robots found in scene %s'%scenefilename)

        if robotname is None:
            self.robot = self.orenv.GetRobots()[0]
        else:
            self.robot = [r for r in self.orenv.GetRobots() if r.GetName()==robotname][0]
                    
        self.taskprob = self.orenv.CreateProblem('TaskManipulation')
        self.manipprob = self.orenv.CreateProblem('BaseManipulation')
        self.visualprob = self.orenv.CreateProblem('VisualFeedback')
        self.orenv.LoadProblem(self.taskprob,self.robot.GetName())
        self.orenv.LoadProblem(self.manipprob,self.robot.GetName())
        self.orenv.LoadProblem(self.visualprob,self.robot.GetName())

        self.homevalues = self.robot.GetJointValues()
        # create a camera viewer for every camera sensor
        self.viewers = []
        for attachedsensor in self.robot.GetSensors():
            if attachedsensor.GetSensor() is not None:
                sensordata = attachedsensor.GetSensor().GetSensorData()
                if sensordata is not None and sensordata.type == Sensor.SensorType.Camera:
                    attachedsensor.GetSensor().SendCommand('power 1')
                    title = attachedsensor.GetName()
                    if len(title) == 0:
                        title = attachedsensor.GetSensor().GetName()
                        if len(title) == 0:
                            title = 'Camera Sensor'
                    self.viewers.append(CameraViewerGUI(sensor=attachedsensor.GetSensor(),title=title))
        print 'found %d camera sensors on robot %s'%(len(self.viewers),self.robot.GetName())
        for viewer in self.viewers:
            viewer.start()
        self.prevaction = signal.signal(signal.SIGINT,lambda x,y: self.sighandle(x,y))

        self.sensor = [s for s in self.robot.GetSensors() if s.GetName()=='wristcam'][0]
        # find a manipulator whose end effector is the camera
        self.manip = [m for m in self.robot.GetManipulators() if m.GetEndEffector().GetIndex() == self.sensor.GetAttachingLink().GetIndex()][0]

        if graspingppfile is not None:
            convexdata,visibilitydata,graspdata = pickle.load(open(graspingppfile,'r'))

    def preprocessdata(self, maskfile = 'hrp2gripper_mask.mat',
                       convexfile = 'hrp2gripper_convex.mat',
                       robotfile = 'robots/hrp2jskreal.robot.xml',
                       graspsetfile = 'simple_grasp_hrp2_cereal.mat',
                       targetfile = 'data/box_frootloops.kinbody.xml',
                       graspingppfile = 'hrp2_frootloops_visibility.pp',
                       visibilityfile = 'cereal_visibility.mat',
                       robotjointinds = [21],
                       robotjoints = [-1.7]):
        jointstring = ' --robotjointinds="' + ' '.join(str(f) for f in robotjointinds) + '" --robotjoints="' + ' '.join(str(f) for f in robotjoints) + '"'
        ret = os.system('python visibilityprocessing.py --func=mask --rayoffset=0.025 --robotfile=' + robotfile + ' --graspsetfile=' + graspsetfile +  ' --savefile=' + maskfile + jointstring)
        if ret != 0:
            raise ValueError('failed to compute mask')
        ret = os.system("""octave --eval "GetLargestFreeConvexPolygon(load('%s'),'%s');" """%(maskfile,convexfile))
        if ret != 0:
            raise ValueError('failed to compute convex hull')
        ret = os.system('python visibilityprocessing.py --func=visibility --robotfile=' + robotfile + ' --kinbodyfile=' + targetfile + ' --graspsetfile=' + graspsetfile + ' --savepp=' + graspingppfile + ' --convexfile=' + convexfile + ' --visibilityfile=' + visibilityfile + jointstring)
        if ret != 0:
            raise ValueError('failed to process visibility information')

    def randomizescene(self):
#         %% randomize robot position
#         if( ~isempty(randomize) )
#             Trobot = reshape(orBodyGetTransform(robot.id),[3 4]);    
#             while(1)
#                 Tnew = Trobot;
#                 Tnew(1:2,4) = Tnew(1:2,4) + [-0.1;-0.5]+rand(2,1).*[0.3;1];
#                 orBodySetTransform(robot.id, Tnew);
#                 if( ~orEnvCheckCollision(robot.id) )
#                     break;
#                 end
#             end
#         end
# 
#         tablepattern = '^table$';
#         tableid = [];
#         bodies = orEnvGetBodies();
#         for i = 1:length(bodies)
#             if( regexp(bodies{i}.name, tablepattern) )
#                 tableid = bodies{i}.id;
#                 break;
#             end
#         end
# 
#         if( ~isempty(tableid) )
#             scenedata.dests = InsertRandomObjects(randomize,tableid,[scenedata.targetid robot.id],scenedata.targetid);
#         end
        pass

    def waitrobot(self):
        #self.robot.GetEnv().LockPhysics(False)
        while not self.robot.GetController().IsDone():
            self.robot.WaitForController(1.0)
        #self.robot.GetEnv().LockPhysics(True)

    def starttrajectory(self,traj):
        self.waitrobot()

    def robotgohome(self,homevalues=None):
        if homevalues is None:
            homevalues = self.homevalues
        armjoints = robot.manips{robot.activemanip}.armjoints;
        handjoints = robot.manips{robot.activemanip}.handjoints;
        print 'moving arm'
        trajdata = self.manipprob.SendCommand('MoveManipulator execute 0 outputtraj armvals ' + ' '.join(str(homevalues[i]) for i in self.manip.GetArmJoints()))
        if len(trajdata) == 0:
            raise ValueError('failed to start trajectory')
        self.starttrajectory(trajdata)
        
        print 'moving hand'
        values = self.robot.GetJointValues()
        values[self.manip.GetGripperJoints()] = homevalues[self.manip.GetGripperJoints()]
        self.robot.GetController().SetDesired(values)
        self.waitrobot()
        
    def testsim(self):
        self.target = self.orenv.ReadKinBodyXMLFile('data/box_frootloops.kinbody.xml')
        self.orenv.AddKinBody(k)
        self.target.SetTransform(array([[ 1.        ,  0.        ,  0.        ,  0.93609834],
                                        [ 0.        ,  1.        ,  0.        , -6.82693148],
                                        [ 0.        ,  0.        ,  1.        ,  0.87991142],
                                        [ 0.        ,  0.        ,  0.        ,  1.        ]]))

        self.robot.ReleaseAllGrabbed()
        self.robotgohome()
        self.robot.SetJointValues(graspdata[0][12:],self.manip.GetGripperJoints())

        self.visualprob.SendCommand('MoveToObserveTarget target ' + self.target.GetName() ' sampleprob 0.001 sensorindex 0 maxiter 4000 convexdata ' scenedata.convexdata ' visibilitydata ' scenedata.visibilitydata],probs.visual,1);
    if( isempty(res) )
        warning('failed to move to target');
        continue;
    end
    WaitForRobot(robot.id);

    if(dopause)
        disp('press any key');
        pause;
    end
    %% start visual servoing step
    res = orProblemSendCommand(['VisualFeedbackGrasping target ' scenedata.targetname ' sensorindex 0 convexfile ' scenedata.convexfile ' graspset ' scenedata.graspsetfile '; maxiter 100 visgraspthresh 0.1 gradientsamples 5 '],probs.visual,0);
    if( isempty(res) )
        warning('failed to find visual feedback grasp');
        continue;
    end
    WaitForRobot(robot.id);
        
    def sighandle(self,x,y):
        self.quitviewers()
        self.prevaction(x,y)

    def quitviewers(self):
        for viewer in self.viewers:
            viewer.main.quit()
    def __del__(self):
        self.quitviewers()
        self.orenv.Destroy()

if __name__=='__main__':
    parser = OptionParser(description='Visibility Planning Module.')
    parser.add_option('--scene',
                      action="store",type='string',dest='scene',default='scenes/r602cerealmanip.env.xml',
                      help='OpenRAVE scene to load')
    parser.add_option('--robotname',
                      action="store",type='string',dest='robotname',default=None,
                      help='Specific robot sensors to display (otherwise first robot found will be displayed)')
    (options, args) = parser.parse_args()
    scene = HRP2GraspingScene(options.scene,options.robotname)
    scene.quitviewers()
