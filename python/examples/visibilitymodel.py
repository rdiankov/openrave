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
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

import time
from numpy import *

from openravepy import *
from openravepy.examples import grasping, inversekinematics, kinematicreachability
from openravepy.interfaces import VisualFeedback

class VisibilityModel(OpenRAVEModel):
    def __init__(self,robot,target,sensorname=None):
        OpenRAVEModel.__init__(self,robot=robot)
        self.target = target
        self.visualprob = VisualFeedback(robot)
        self.convexhull = None
        self.sensorname = sensorname
        self.manip = robot.GetActiveManipulator()
        self.manipname = None if self.manip is None else self.manip.GetName()
        self.visibilitytransforms = None
        self.rmodel = self.gmodel = self.ikmodel = None
        self.preprocess()
    def clone(self,envother):
        clone = OpenRAVEModel.clone(self,envother)
        clone.rmodel = self.rmodel.clone(envother) if not self.rmodel is None else None
        clone.gmodel = self.gmodel.clone(envother) if not self.gmodel is None else None
        clone.ikmodel = self.ikmodel.clone(envother) if not self.ikmodel is None else None
        clone.visualprob = self.visualprob.clone(envother)
        clone.preprocess()
        return clone
    def has(self):
        return len(self.visibilitytransforms) > 0
    def getversion(self):
        return 1
    def getfilename(self):
        return os.path.join(OpenRAVEModel.getfilename(self),'visibility.' + self.manip.GetName() + '.' + self.attachedsensor.GetName() + '.' + self.target.GetKinematicsGeometryHash()+'.pp')
    def load(self):
        try:
            params = OpenRAVEModel.load(self)
            if params is None:
                return False
            self.visibilitytransforms,self.convexhull,self.KK,self.dims = params
            self.preprocess()
            return self.has()
        except e:
            return False
    def save(self):
        OpenRAVEModel.save(self,(self.visibilitytransforms,self.convexhull,self.KK,self.dims))

    def preprocess(self):
        with self.env:
            manipname = self.visualprob.SetCamera(sensorname=self.sensorname,manipname=self.manipname)
            assert(self.manipname is None or self.manipname==manipname)
            self.manip = self.robot.SetActiveManipulator(manipname)
            self.attachedsensor = [s for s in self.robot.GetSensors() if s.GetName() == self.sensorname][0]
            self.gmodel = grasping.GraspingModel(robot=self.robot,target=self.target)
            if not self.gmodel.load():
                self.gmodel.autogenerate()
            self.ikmodel = inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.Transform6D)
            if not self.ikmodel.load():
                self.ikmodel.autogenerate()
            self.visualprob.SetCameraTransforms(transforms=self.visibilitytransforms)
    
    def autogenerate(self,options=None):
        self.generate()
        self.save()
    def generate(self):
        self.preprocess()
        self.sensorname = self.attachedsensor.GetName()
        self.manipname = self.manip.GetName()
        bodies = [(b,b.IsEnabled()) for b in self.env.GetBodies() if b != self.robot and b != self.target]
        for b in bodies:
            b[0].Enable(False)
        try:
            with self.env:
                sensor = self.attachedsensor.GetSensor()
                if sensor is not None: # set power to 0?
                    sensordata = sensor.GetSensorData()
                    self.KK = sensordata.KK
                    self.dims = sensordata.imagedata.shape
                with RobotStateSaver(self.robot):
                    self.gmodel.setPreshape(self.gmodel.grasps[0]) # find better way of handling multiple grasps
                    extentsfile = os.path.join(self.env.GetHomeDirectory(),'kinbody.'+self.target.GetKinematicsGeometryHash(),'visibility.txt')
                    if os.path.isfile(extentsfile):
                        self.visibilitytransforms = self.visualprob.ProcessVisibilityExtents(target=self.target, extents=loadtxt(extentsfile,float))
                    else:
                        self.visibilitytransforms = self.visualprob.ProcessVisibilityExtents(target=self.target, sphere=[3,5,0.1,0.15,0.2,0.25,0.3])
                self.visualprob.SetCameraTransforms(transforms=self.visibilitytransforms)
        finally:
            for b,enable in bodies:
                b.Enable(enable)

    def SetCameraTransforms(self,transforms):
        self.visualprob.SetCameraTransforms(transforms=transforms)
    def showtransforms(self):
        pts = array([dot(self.target.GetTransform(),matrixFromPose(pose))[0:3,3] for pose in self.visibilitytransforms])
        h=self.env.plot3(pts,5,colors=array([0.5,0.5,1,0.2]))
        with RobotStateSaver(self.robot):
            with grasping.GraspingModel.GripperVisibility(self.manip):
                for pose in self.visibilitytransforms:
                    with self.env:
                        Trelative = dot(linalg.inv(self.attachedsensor.GetTransform()),self.manip.GetEndEffectorTransform())
                        Tcamera = dot(self.target.GetTransform(),matrixFromPose(pose))
                        Tgrasp = dot(Tcamera,Trelative)
                        Tdelta = dot(Tgrasp,linalg.inv(self.manip.GetEndEffectorTransform()))
                        for link in self.manip.GetChildLinks():
                            link.SetTransform(dot(Tdelta,link.GetTransform()))
                        self.env.UpdatePublishedBodies()
                    raw_input('press any key to continue: ')

    def computeValidTransform(self,returnall=False,checkcollision=True,computevisibility=True,randomize=False):
        with self.robot:
            validjoints = []
            if randomize:
                order = random.permutation(len(self.visibilitytransforms))
            else:
                order = xrange(len(self.visibilitytransforms))
            for i in order:
                pose = self.visibilitytransforms[i]
                Trelative = dot(linalg.inv(self.attachedsensor.GetTransform()),self.manip.GetEndEffectorTransform())
                Tcamera = dot(self.target.GetTransform(),matrixFromPose(pose))
                Tgrasp = dot(Tcamera,Trelative)
                s = self.manip.FindIKSolution(Tgrasp,checkcollision)
                if s is not None:
                    self.robot.SetJointValues(s,self.manip.GetArmJoints())
                    if computevisibility and not self.visualprob.ComputeVisibility(self.target):
                        continue
                    validjoints.append((s,i))
                    if not returnall:
                        return validjoints
                    print 'found',len(validjoints)

    def pruneTransformations(self,thresh=0.04,numminneighs=10,translationonly=True):
        if self.rmodel is None:
            self.rmodel = kinematicreachability.ReachabilityModel(robot=self.robot)
            if not self.rmodel.load():
                self.rmodel.autogenerate()
        kdtree=self.rmodel.ComputeNN(translationonly)
        newtrans = poseMultArrayT(poseFromMatrix(dot(linalg.inv(self.manip.GetBase().GetTransform()),self.target.GetTransform())),self.visibilitytransforms)
        if translationonly:
            transdensity = kdtree.kFRSearchArray(newtrans[:,4:7],thresh**2,0,thresh*0.01)[2]
            I=flatnonzero(transdensity>numminneighs)
            return self.visibilitytransforms[I[argsort(-transdensity[I])]]
#         Imask = GetCameraRobotMask(orenv,options.robotfile,sensorindex=options.sensorindex,gripperjoints=gripperjoints,robotjoints=robotjoints,robotjointinds=robotjointinds,rayoffset=options.rayoffset)
#         # save as a ascii matfile
#         numpy.savetxt(options.savefile,Imask,'%d')
#         print 'mask saved to ' + options.savefile
#         try:
#             scipy.misc.pilutil.imshow(array(Imask*255,'uint8'))
#         except:
#             pass

#     def GetCameraRobotMask(self,rayoffset=0):
#         with self.env:
#             inds = array(range(self.width*self.height))
#             imagepoints = array((mod(inds,self.width),floor(inds/self.width)))
#             camerapoints = dot(linalg.inv(self.KK), r_[imagepoints,ones((1,imagepoints.shape[1]))])
#             Tcamera = self.attached.GetSensor().GetTransform()
#             raydirs = dot(Tcamera[0:3,0:3], camerapoints / tile(sqrt(sum(camerapoints**2,0)),(3,1)))
#             rays = r_[tile(Tcamera[0:3,3:4],(1,raydirs.shape[1]))+rayoffset*raydirs,100.0*raydirs]
#             hitindices,hitpositions = self.prob.GetEnv().CheckCollisionRays(rays,self.robot,False)
#             # gather all the rays that hit and form an image
#             return reshape(array(hitindices,'float'),(height,width))

    def getCameraImage(self,delay=1.0):
        sensor=self.attachedsensor.GetSensor()
        sensor.SendCommand('power 1')
        try:
            time.sleep(delay)
            return sensor.GetSensorData().imagedata
        finally:
            sensor.SendCommand('power 0')

    @staticmethod
    def CreateOptionParser():
        parser = OpenRAVEModel.CreateOptionParser()
        parser.description='Computes and manages the visibility transforms for a manipulator/target.'
        parser.add_option('--target',action="store",type='string',dest='target',
                          help='OpenRAVE kinbody target filename')
        parser.add_option('--sensorname',action="store",type='string',dest='sensorname',default=None,
                          help='Name of the sensor to build visibilty model for (has to be camera). If none, takes first possible sensor.')
        parser.add_option('--rayoffset',action="store",type='float',dest='rayoffset',default=0.03,
                          help='The offset to move the ray origin (prevents meaningless collisions), default is 0.03')
        return parser
    @staticmethod
    def RunFromParser(Model=None,parser=None):
        if parser is None:
            parser = VisibilityModel.CreateOptionParser()
        (options, args) = parser.parse_args()
        env = Environment()
        try:
            if Model is None:
                Model = lambda robot: VisibilityModel(robot=robot)
            OpenRAVEModel.RunFromParser(env=env,Model=Model,parser=parser)
        finally:
            env.Destroy()

if __name__=='__main__':
    VisibilityModel.RunFromParser()
