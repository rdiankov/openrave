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


# random code that helps with debugging/testing the python interfaces and examples
# this is not meant to be run by normal users
from openravepy import *
import openravepy.examples
from openravepy.interfaces import *
from numpy import *
import numpy,time

def test_grasping():
    import grasping
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    T = eye(4)
    T[0,3] = 1
    robot.SetTransform(T)
    target = env.ReadKinBodyXMLFile('data/mug1.kinbody.xml')
    target.SetTransform(T)
    env.AddKinBody(target)
    env.SetViewer('qtcoin')
    self = grasping.GraspingModel(robot=robot,target=target)
    self.init(friction=0.4,avoidlinks=[])
    preshapes = array(((0.5,0.5,0.5,pi/3),(0.5,0.5,0.5,0),(0,0,0,pi/2)))
    rolls = arange(0,2*pi,pi/2)
    standoffs = array([0,0.025])
    approachrays = self.computeBoxApproachRays(stepsize=0.02)
    graspingnoise=None
    self.generate(preshapes=preshapes, rolls=rolls, standoffs=standoffs, approachrays=approachrays,graspingnoise=None,addSphereNorms=False)

def test_autograsping():
    import grasping
    env = Environment()
    env.SetDebugLevel(DebugLevel.Debug)
    env.SetViewer('qtcoin')
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    target = env.GetKinBody('mug1')
    self = grasping.GraspingModel(robot=robot,target=target)
    self.autogenerate()

def test_ikgeneration():
    import inversekinematics
    env = Environment()
    env.SetDebugLevel(DebugLevel.Debug)
    robot = env.ReadRobotXMLFile('/home/rdiankov/downloads/motoman.robot.xml')#robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    self = inversekinematics.InverseKinematicsModel(robot=robot)
    freejoints=None
    usedummyjoints=False
    accuracy = None
    precision = None
    self.generate(freejoints=freejoints,usedummyjoints=usedummyjoints)

    baselink=self.manip.GetBase().GetIndex()
    eelink = self.manip.GetEndEffector().GetIndex()
    solvejoints=solvejoints
    freeparams=freejoints
    usedummyjoints=usedummyjoints
    solvefn=solvefn

def test_reachability():
    import kinematicreachability
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    self = kinematicreachability.ReachabilityModel(robot=robot)
    self.autogenerate()

def test_inversereachabilitygen():
    import inversereachability
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    self = inversereachability.InverseReachabilityModel(robot=robot)
    heightthresh=0.05
    quatthresh=0.2
    self.generate(heightthresh=heightthresh2,quatthresh=quatthresh)

def test_inversereachabilityrun():
    import inversereachability, graspplanning
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    self = inversereachability.InverseReachabilityModel(robot=robot)
    self.load()
    gp = graspplanning.GraspPlanning(robot=robot,randomize=False)
    gm = gp.graspables[0][0]
    dests = gp.graspables[0][1]
    validgrasps,validindices = gm.computeValidGrasps()
    Tgrasp = gm.getGlobalGraspTransform(validgrasps[0])
    densityfn,samplerfn,bounds = self.computeBaseDistribution(Tgrasp,2000)
    h = self.showBaseDistribution(densityfn,bounds,zoffset=1.0,thresh=1.0)
    densityfn2,samplerfn2,bounds2 = self.computeAggregateBaseDistribution([Tgrasp],2000)
    h2 = self.showBaseDistribution(densityfn2,bounds2,zoffset=3.0,thresh=1.0)

def test_inversereachabilitytest():
    import inversereachability
    env = openravepy.Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    self = inversereachability.InverseReachabilityModel(robot=robot)
    self.load()
    self.testSampling()

def test_graspplanning():
    import graspplanning
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    self = graspplanning.GraspPlanning(robot)
    gm=self.graspables[0][0]
    dests=self.graspables[0][1]
    self.graspAndPlaceObject(gm=gm,dests=dests)

def test_graspreachability():
    import mobilemanipulation,graspplanning
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    planning = graspplanning.GraspPlanning(robot)
    gmodel=planning.graspables[0][0]
    dests=planning.graspables[0][1]
    self = mobilemanipulation.GraspReachability(robot=robot,gmodel=gmodel)
    starttime = time.time()
    densityfn,samplerfn,bounds,validgrasps = self.computeGraspDistribution(logllthresh=2000.0)
    print 'time to build distribution: %fs'%(time.time()-starttime)
    h = self.irmodel.showBaseDistribution(densityfn,bounds,self.target.GetTransform()[2,3],thresh=1.0)
    
    starttime = time.time()
    goals,numfailures = self.sampleGoals(lambda N: samplerfn(N,1.0),validgrasps,N=100)
    print 'numgrasps: %d, time: %f, failures: %d'%(len(goals),time.time()-starttime,numfailures)

    # compute performance with random sampler (given bounds)
    Trobot = self.robot.GetTransform()
    def randomsampler(N):
        angles = random.rand(N)*(bounds[1,0]-bounds[0,0])+bounds[0,0]
        X = random.rand(N)*(bounds[1,1]-bounds[0,1])+bounds[0,1]
        Y = random.rand(N)*(bounds[1,2]-bounds[0,2])+bounds[0,2]
        return c_[cos(angles),zeros((N,2)),sin(angles),X,Y,tile(Trobot[2,3],N)],array(random.randint(0,len(validgrasps),N))
    starttime = time.time()
    goals,numfailures = self.sampleGoals(randomsampler,validgrasps,N=100)
    print 'numgrasps: %d, time: %f, failures: %d'%(len(goals),time.time()-starttime,numfailures)

    self.graspAndPlaceObject(gm=gm,dests=dests)

def test_mobilemanipulation():
    import mobilemanipulation
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    self = mobilemanipulation.MobileManipulationPlanning(robot)
    gmodel=self.graspables[0][0]
    dests=self.graspables[0][1]

def test_visibilityplanning():
    import visibilityplanning, time
    self = visibilityplanning.HRP2GraspingScene()
    time.sleep(5)
    self.loadscene(scenefilename='scenes/r602real.env.xml')
    self.testsim()

def test_pyann():
    ktree = pyANN.KDTree(random.rand(10,7))
    neighs,dists = ktree.kSearch(random.rand(7),5,1e-3);

def test_ikfast():
    import openravepy
    env = openravepy.Environment()
    robot = env.ReadRobotXMLFile('robots/barrettwam.robot.xml')
    env.AddRobot(robot)
    manip = robot.GetManipulators()[0]
    solvejoints = list(manip.GetArmJoints())
    solvejoints.pop(2)
    solvefn=openravepy.ikfast.IKFastSolver.solveFullIK_6D
    self = openravepy.ikfast.IKFastSolver(kinbody=robot)
    baselink = manip.GetBase().GetIndex()
    eelink = manip.GetEndEffector().GetIndex()
    usedummyjoints = False
    code = self.generateIkSolver(baselink=baselink,eelink=eelink,solvejoints=solvejoints,freeparams=freejoints,usedummyjoints=usedummyjoints,solvefn=solvefn)

def test_gripper():
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    basemanip = openravepy.interfaces.BaseManipulation(robot)
    manip = robot.GetActiveManipulator()
    robot.SetActiveDOFs(manip.GetGripperJoints())
    basemanip.ReleaseFingers(execute=True)

def test_constraintplanning():
    import constraintplanning
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    env.UpdatePublishedBodies()
    time.sleep(0.1) # give time for environment to update
    self = constraintplanning.ConstraintPlanning(robot)
    
    self.robot.SetJointValues([-0.90993702,  1.4134903 ,  1.18074048,  1.6281302 , -1.42419982, 1.17677045, -2.48384023,  0.98699927,  0.59599888,  1.1350019 ,  0])
    target = self.gmodel.target
    robot.Grab(target)
    T = self.manip.GetEndEffectorTransform()
    T[2,3] += 0.4
    constraintfreedoms = [1,1,0,0,0,0]
    constraintmatrix = eye(4)
    constrainterrorthresh = 1e-2
    res = self.basemanip.MoveToHandPosition(matrices=[T],maxiter=10000,maxtries=1,seedik=8,constraintfreedoms=constraintfreedoms,constraintmatrix=constraintmatrix,constrainterrorthresh=constrainterrorthresh)

    #self.performGraspPlanning()

def test_convex():
    import convexdecomposition
    env = Environment()
    #env.SetViewer('qtcoin')
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    self = convexdecomposition.ConvexDecompositionModel(robot)
    self.load()
    
    hulls = self.linkgeometry[0][0][1]

def test_linkstatistics():
    import linkstatistics
    from itertools import izip
    from enthought.tvtk.api import tvtk
    env = openravepy.Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    self = linkstatistics.LinkStatisticsModel(robot)
    #self.load()

    self.robot.SetTransform(eye(4))
    links = self.robot.GetLinks()
    ilink = 1
    link = links[ilink]
    linkcd = self.cdmodel.linkgeometry[ilink]
    hulls = []
    for ig,geom in enumerate(link.GetGeometries()):
        cdhulls = [cdhull for i,cdhull in linkcd if i==ig]
        if len(cdhulls) > 0:
            hulls += [self.transformHull(geom.GetTransform(),hull) for hull in cdhulls[0]]
        elif geom.GetType() == KinBody.Link.GeomProperties.Type.Box:
            hulls.append(self.transformHull(geom.GetTransform(),ComputeBoxMesh(geom.GetBoxExtents())))
        elif geom.GetType() == KinBody.Link.GeomProperties.Type.Sphere:
            hulls.append(self.transformHull(geom.GetTransform(),ComputeGeodesicSphereMesh(geom.GetSphereRadius(),level=1)))
        elif geom.GetType() == KinBody.Link.GeomProperties.Type.Cylinder:
            hulls.append(self.transformHull(geom.GetTransform(),ComputeCylinderYMesh(radius=geom.GetCylinderRadius(),height=geom.GetCylinderHeight())))
    linkstat = self.computeGeometryStatistics(hulls)

    ijoint = 0
    joint = self.robot.GetJoints()[0]
    lower,upper = joint.GetLimits()
    volumepoints=linkstat['volumepoints']
    axis=joint.GetAxis(0)
    minangle=lower[0]
    maxangle=upper[0]
    sweptpoints,sweptindices = self.computeSweptVolume(volumepoints=volumepoints,axis=axis,minangle=minangle,maxangle=maxangle)

def test_contours():
    from enthought.tvtk.api import tvtk
    from numpy import *
    import pickle,numpy
    from openravepy import *
    env = Environment()
    env.SetViewer('qtcoin')

    N = 50
    Nj = N*(0+1j)
    x, y, z = numpy.mgrid[-10:10:Nj, -10:20:Nj, -10:40:Nj]
    scalars = x*x + 2.0*y*y + z*z/2.0
    spacing=array((0.005,0.005,0.005))
    id = tvtk.ImageData(origin=array((numpy.min(x),numpy.min(y),numpy.min(z))),spacing=spacing,dimensions=scalars.shape)
    id.point_data.scalars = scalars.ravel()

    x,y,z,t,sweptdata = pickle.load(open('tris.pp','r'))
    sweptdata = array(sweptdata,'float64')
    #sweptdata = sweptdata[0:61,0:60,0:60]
    id = tvtk.ImageData(origin=array((0,0,0)),spacing=array((0.005,0.005,0.005)),dimensions=sweptdata.shape[::-1])
    id.point_data.scalars = 100.0*sweptdata.ravel()

    m = tvtk.MarchingCubes()
    m.set_input(id)
    m.set_value(0,0.5)
    m.update()
    o = m.get_output()
    newpoints = array(o.points)

    h = env.plot3 (points=newpoints,pointsize=2.0,colors=array((1,0,0)))

    indices = array(o.polys.data)
    indices = array(reshape(indices,(len(indices)/4,4)),'int')
    h2 = env.drawtrimesh (points=newpoints,indices=indices[:,1:4],colors=array((0,0,1,0.5)))
