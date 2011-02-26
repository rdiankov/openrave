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
    quatthresh=0.15
    self.generate(heightthresh=heightthresh,quatthresh=quatthresh)

def test_inversereachabilitytest():
    import inversereachability
    from scipy.optimize import leastsq
    import bisect
    env = openravepy.Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    self = inversereachability.InverseReachabilityModel(robot=robot)
    self.load()
    self.env.StopSimulation()
    self.robot.SetTransform(eye(4))
    self.testSampling(heights=arange(-1,1,0.1),logllthresh=2.3)
    W = array([sum(e[2][:,-1]) for e in self.equivalenceclasses])
    W /= max(W)
    dirs = array([m[4]*linalg.inv(rotationMatrixFromQuat(m[0:4]))[:,2] for m in self.equivalencemeans])
    h = self.env.plot3 (points=dirs,pointsize=5.0,colors=c_[1-W,1-W,1-W,W])

def test_inversereachabilityrun():
    import inversereachability
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    self = inversereachability.InverseReachabilityModel(robot=robot)
    self.load()
    gp = examples.graspplanning.GraspPlanning(robot=robot,randomize=False)
    gm = gp.graspables[0][0]
    dests = gp.graspables[0][1]
    validgrasps,validindices = gm.computeValidGrasps()
    Tgrasp = gm.getGlobalGraspTransform(validgrasps[0])
    densityfn,samplerfn,bounds = self.computeBaseDistribution(Tgrasp,2000)
    h = self.showBaseDistribution(densityfn,bounds,zoffset=1.0,thresh=1.0)
    densityfn2,samplerfn2,bounds2 = self.computeAggregateBaseDistribution([Tgrasp],2000)
    h2 = self.showBaseDistribution(densityfn2,bounds2,zoffset=3.0,thresh=1.0)

def draw_inversereachability():
    import inversereachability
    env = openravepy.Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    self = inversereachability.InverseReachabilityModel(robot=robot)
    self.load()
    self.env.SetViewer('qtcoin')
    self.showEquivalenceClass(self.equivalenceclasses[0])

def test_graspplanning():
    import graspplanning
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('data/lab1.env.xml')#wamtest1.env.xml')
    robot = env.GetRobots()[0]
    self = graspplanning.GraspPlanning(robot)
    gmodel=self.graspables[0][0]
    dests=self.graspables[0][1]
    self.graspAndPlaceObject(gmodel=gmodel,dests=dests)

    # draw all valid grasps
    configs = []
    for gmodel,dests in self.graspables:
        validgrasps = gmodel.computeValidGrasps(checkcollision=True)[0]
        for g in validgrasps:
            T = dot(gmodel.getGlobalGraspTransform(g),linalg.inv(robot.GetActiveManipulator().GetGraspTransform()))
            configs.append((T,g[gmodel.graspindices['igrasppreshape']]))

    robotfilename='robots/barretthand.robot.xml'
    transparency=0.7
    newrobots=[]
    for T,preshape in configs:
        print len(newrobots)
        newrobot = env.ReadRobotXMLFile(robotfilename)
        for link in newrobot.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(transparency)
        env.AddRobot(newrobot,True)
        newrobot.SetTransform(T)
        newrobot.SetDOFValues(preshape)
        newrobots.append(newrobot)
    for newrobot in newrobots:
        env.RemoveKinBody(newrobot)
    newrobots=[]


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
    densityfn,samplerfn,bounds,validgrasps = self.computeGraspDistribution(logllthresh=2.4)
    print 'time to build distribution: %fs'%(time.time()-starttime)
    h = self.irmodel.showBaseDistribution(densityfn,bounds,self.target.GetTransform()[2,3],thresh=1.0)

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
    self = visibilityplanning.PA10GraspExample()
    self.loadscene(scenefilename='data/pa10grasp.env.xml',randomize=False,sensorname='wristcam',showsensors=False)
    self.start()

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
    robot.SetTransform(eye(4))
    self = linkstatistics.LinkStatisticsModel(robot)
    self.load()

    self.robot.SetTransform(eye(4))
    links = self.robot.GetLinks()
    ilink = 14
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

    ijoint = 3
    joint = self.robot.GetJoints()[0]
    lower,upper = joint.GetLimits()
    axis=joint.GetAxis(0)
    minangle=lower[0]
    maxangle=upper[0]
    Tlinkjoint = self.robot.GetLinkts()[ilink].GetTransform()
    Tlinkjoint[0:3,3] -= joint.GetAnchor() # joint anchor should be at center
    volumepoints = dot(linkstat['volumepoints'],transpose(Tlink[0:3,0:3]))
    volumepoints += tile(Tlink[0:3,3],(len(volumepoints),1))                    
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

def test_jointweights():
    jointdv = array([v['volumedelta'] for v in self.jointvolumes ])
    linkdv = array([v['volume'] for v in self.linkstats])

def test_simplenavigation():
    import simplenavigation
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    self = simplenavigation.SimpleNavigationPlanning(robot)

    self.robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
    self.robot.SetAffineRotationAxisMaxVels(ones(4))
    self.robot.SetActiveDOFs([],Robot.DOFAffine.X|Robot.DOFAffine.Y|Robot.DOFAffine.RotationAxis,[0,0,1])
    self.basemanip.MoveActiveJoints(goal=[0.737,0.304,0])

def test_sampling():
    import heapq
    self = SpaceSampler()
    stats = []
    for level in range(5):
        theta,pfi = self.sampleS2(level)
        dirs = c_[cos(theta),sin(theta)*cos(pfi),sin(theta)*sin(pfi)]
        dists = [heapq.nsmallest(2,arccos(dot(dirs,dir)))[1] for dir in dirs]
        stats.append([level,mean(dists)])
    
def test_hrp2():
    python convexdecomposition.py --volumeSplitThresholdPercent=5 --mergeThresholdPercent=10 --padding=0.005
    rosrun openrave_database kinematicreachability_ros.py --manipname=leftarm --xyzdelta=0.04 --launchservice='8*localhost' 
    python kinematicreachability.py --manipname=rightarm --xyzdelta=0.02
    python kinematicreachability.py --manipname=leftarm --xyzdelta=0.02
    python kinematicreachability.py --manipname=rightarm_chest --xyzdelta=0.02
    python inversereachability.py --manipname=rightarm --heightthresh=0.02 --quatthresh=0.2
    python inversereachability.py --manipname=leftarm --heightthresh=0.02 --quatthresh=0.2
    python inversereachability.py --manipname=rightarm_chest --heightthresh=0.02 --quatthresh=0.2
    python inversereachability.py --manipname=leftarm_chest --heightthresh=0.02 --quatthresh=0.2
    python inversereachability.py --manipname=leftarm_chest --heightthresh=0.02 --quatthresh=0.2 --id=0 --jointvalues='0'
    python inversereachability.py --manipname=leftarm_chest --heightthresh=0.02 --quatthresh=0.2 --id=43 --jointvalues='0.43'
    python grasping.py --robot=robots/hrp2jsk.robot.xml --manipname=rightarm --target=scenes/cereal_frootloops.kinbody.xml --standoff=0 --boxdelta=0.01 --normalanglerange=1 --avoidlink=RWristCam
    python grasping.py --robot=robots/hrp2jsk.robot.xml --manipname=leftarm --target=scenes/cereal_frootloops.kinbody.xml --standoff=0 --boxdelta=0.01 --normalanglerange=1 --graspingnoise=0.01 --noviewer
    rosrun openrave_database grasping_ros.py --robot=robots/hrp2jsk.robot.xml --manipname=leftarm_chest --target=scenes/cereal_frootloops.kinbody.xml --standoff=0 --boxdelta=0.01 --normalanglerange=1 --graspingnoise=0.01 --launchservice='8*localhost'
    rosrun openrave_database grasping_ros.py --robot=robots/hrp2jsk.robot.xml --manipname=leftarm_chest2 --target=scenes/jskcup0.kinbody.xml --standoff=0 --boxdelta=0.01 --normalanglerange=1 --graspingnoise=0.01 --launchservice='8*localhost'

    import inversereachability
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/hrp2jsk.robot.xml')
    env.AddRobot(robot)
    robot.SetActiveManipulator('leftarm')
    self = inversereachability.InverseReachabilityModel(robot=robot)
    heightthresh=0.02
    quatthresh=0.1
    self.generate(heightthresh=heightthresh,quatthresh=quatthresh)

    hand = env.ReadRobotXMLFile('robots/hrp2rhandjsk.robot.xml')
    env.AddRobot(hand)
    hand.SetTransform(Tgrasp)

    # test head movement
    import inversekinematics
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/hrp2jsk08.robot.xml')
    env.AddRobot(robot)
    robot.SetActiveManipulator('head')
    manip = robot.GetActiveManipulator()
    ikmodel = inversekinematics.InverseKinematicsModel(robot,IkParameterization.Type.Direction3D)
    if not ikmodel.load():
        ikmodel.generate()
    
    import inversereachability,mobilemanipulation,graspplanning,visibilitymodel
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('scenes/r602kitchen1.env.xml')
    robot = env.GetRobots()[0]
    origjointvalues = robot.GetJointValues()
    # define all the manipulators to use
    manips = [robot.GetManipulators('rightarm_chest')[0], robot.GetManipulators('rightarm_chest2')[0]]#,robot.GetManipulators('leftarm_chest')[0], robot.GetManipulators('leftarm_chest2')[0]]
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
    irgmodels = []
    targets = []
    for manip in manips:
        robot.SetActiveManipulator(manip)
        planning = graspplanning.GraspPlanning(robot,nodestinations=True)
        for gmodel,dests in planning.graspables:
            if True:#gmodel.target.GetName() == 'cereal0' or gmodel.target.GetName() == 'cereal1':
                for irmodel in irmodels:
                    if irmodel.manip == gmodel.manip:
                        irgmodels.append([irmodel,gmodel])
                        if not gmodel.target in targets:
                            targets.append(gmodel.target)
    grmodel = mobilemanipulation.GraspReachability(robot=robot,irgmodels=irgmodels)
    self = mobilemanipulation.MobileManipulationPlanning(robot,grmodel=grmodel)

    usevisibilitycamera = 'wristcam'
    gmodel = self.graspObjectMobileSearch(usevisibilitycamera=usevisibilitycamera)

    table = env.GetKinBody('table')
    if table is not None:
        graspables = None
        Trolls = [matrixFromAxisAngle(array((0,0,1)),roll) for roll in arange(0,2*pi,pi/4)]
        alldests = graspplanning.GraspPlanning.setRandomDestinations(targets,table,transdelta=0.05,Trolls=Trolls,randomize=False)
        targetdests=zip(targets,alldests)
        self.graspAndPlaceObjectMobileSearch(targetdests=targetdests)

    
    #h = gr.showBaseDistribution(thresh=1.0,logllthresh=logllthresh)
    #grmodel.testSampling(weight=1.5,logllthresh=0.5,randomgrasps=True,randomplacement=False,updateenv=False)
    
    validgrasps,validindices = gr.gmodel.computeValidGrasps(checkik=False,backupdist=0.01)
    grmodel.gmodel.showgrasp(validgrasps[0],collisionfree=True)

    densityfn,samplerfn,bounds,validgrasps = gr.computeGraspDistribution(logllthresh=logllthresh)
    goals,numfailures=gr.sampleGoals(lambda goals: samplerfn(goals,weight=1.0),updateenv=True)
    grasp,pose,q = goals[0]
    robot.SetTransform(pose)
    robot.SetJointValues(q)
    basemanip.CloseFingers()

    grasp = gr.gmodel.grasps[283]
    Tgrasp = gr.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
    equivalenceclass,logll = gr.irmodel.getEquivalenceClass(Tgrasp)
    densityfn,samplerfn,bounds = gr.irmodel.computeBaseDistribution(Tgrasp,logllthresh=logllthresh)
    h = gr.irmodel.showBaseDistribution(densityfn,bounds,zoffset=gr.target.GetTransform()[2,3],thresh=1.0)

    env = Environment()
    robot = env.ReadRobotXMLFile('robots/hrp2jsk08.robot.xml')
    env.AddRobot(robot)
    body = env.ReadKinBodyXMLFile('scenes/cereal_frootloops.kinbody.xml')
    env.AddKinBody(body)
    T = eye(4)
    T[0:3,3] = [0.466,-0.157,0.544]
    body.SetTransform(T)
    robot.SetActiveManipulator('rightarm_chest')
    robot.Grab(body)
    robot.SetJointValues([-1.4,1.35239005,1.036349],[5,7,8])
    robot.CheckSelfCollision()
    

def test_visibility():
    import visibilitymodel
    env = Environment()    
    env.Reset()
    robot = env.ReadRobotXMLFile('robots/hrp2jsk08real.robot.xml')
    env.AddRobot(robot)
    robot.SetActiveManipulator('rightarm_chest')
    target = env.ReadKinBodyXMLFile('scenes/cereal_frootloops.kinbody.xml')
    env.AddKinBody(target)

    robot.SetActiveManipulator('rightarm_chest')
    self = visibilitymodel.VisibilityModel(robot=robot,target=target,sensorname='wristcam')
    if not self.load():
        self.autogenerate()

    env.SetViewer('qtcoin')
    self.showtransforms()

    import kinematicreachability, grasping, visibilitymodel
    pose = array([-0.88932403,  0.        ,  0.        , -0.45727755,  8.26839721, 3.14201928,  0.66500002])
    grasp = array([  1.65648009e-06,  -2.83057716e-06,   1.00000000e+00,
         9.23879445e-01,   3.82683724e-01,  -4.47596904e-07,
        -3.82683724e-01,   9.23879445e-01,   3.24873599e-06,
        -4.32802886e-02,   8.53489910e-04,   7.89998472e-02,
         0.00000000e+00,   4.71238898e+00,   9.23879477e-01,
         3.82683432e-01,  -5.50675114e-08,   1.65648009e-06,
        -2.83057716e-06,   1.00000000e+00,   9.23879445e-01,
         3.82683724e-01,  -4.47596904e-07,  -3.82683724e-01,
         9.23879445e-01,   3.24873599e-06,  -4.23564091e-02,
         1.23617332e-03,   7.89998472e-02,   6.41213000e-03,
        -2.11999994e-02,   9.99999978e-03,   7.90000036e-02,
         2.35619450e+00])
    robot.SetTransform(pose)
    robot.SetActiveManipulator('rightarm_chest')
#     rmodel = kinematicreachability.ReachabilityModel(robot=robot)
#     rmodel.load()
    gmodel = grasping.GraspingModel(robot,target=env.GetKinBody('cereal0'))
    gmodel.moveToPreshape(grasp)
    gmodel.robot.GetController().Reset(0)
    vmodel = visibilitymodel.VisibilityModel(robot=robot,target=gmodel.target,sensorname='wristcam')
    vmodel.load()
    self = vmodel
    self.SetCameraTransforms(self.pruneTransformations(thresh=0.04))

    validjoints=self.computeValidTransform()
    self.robot.SetJointValues(validjoints[0][0],self.manip.GetArmJoints())
    s=vmodel.visualprob.SampleVisibilityGoal(target=gmodel.target)

    pts = array([dot(self.target.GetTransform(),matrixFromPose(pose))[0:3,3] for pose in self.visibilitytransforms])
    h=self.env.plot3(pts,5,colors=array([0.5,0.5,1,0.03]))

def test_navigation():
    import inversereachability,mobilemanipulation,graspplanning,visibilitymodel
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('scenes/r602kitchen1.env.xml')
    robot = env.GetRobots()[0]
    robot.GetController().Reset(0)
    Tstart = array([[ 1.        ,  0.        ,  0.        ,  6.26000023-7.37],
           [ 0.        ,  1.        ,  0.        ,  2.66899991-3.2],
           [ 0.        ,  0.        ,  1.        ,  0.66500002],
           [ 0.        ,  0.        ,  0.        ,  1.        ]])
    robot.SetTransform(Tstart)
    robot.SetJointValues(array([ 0.        ,  0.        ,  0.84761411,  0.        ,  0.        ,
           -2.20907021,  0.        ,  0.        ,  0.        ,  0.97831494,
            0.        ,  0.        , -2.23727012,  0.        ,  0.        ,
            0.        ,  0.        ,  0.        , -0.17453291,  0.17453295], dtype=float32))
    self = mobilemanipulation.MobileManipulationPlanning(robot)
    
    goal2d = array([0.29134295742674898, -0.26705494655604034, -3.1834347453894472])
    #goal2d = array([6.2547940332687197-7.37, 2.2240884123771689-3.2, -6.0887479146975627])
    envmin = []
    envmax = []
    for b in self.env.GetBodies():
        ab = b.ComputeAABB()
        envmin.append(ab.pos()-ab.extents())
        envmax.append(ab.pos()+ab.extents())
    abrobot = self.robot.ComputeAABB()
    envmin = numpy.min(array(envmin),0)+abrobot.extents()
    envmax = numpy.max(array(envmax),0)-abrobot.extents()
    bounds = array(((envmin[0],envmin[1],-pi),(envmax[0],envmax[1],pi)))
    self.robot.SetAffineTranslationLimits(envmin,envmax)
    self.robot.SetAffineTranslationMaxVels([0.5,0.5,0.5])
    self.robot.SetAffineRotationAxisMaxVels(ones(4))
    self.robot.SetActiveDOFs([],Robot.DOFAffine.X|Robot.DOFAffine.Y|Robot.DOFAffine.RotationAxis,[0,0,1])
    
    center = r_[goal2d[0:2],0.2]
    xaxis = 0.5*array((cos(goal2d[2]),sin(goal2d[2]),0))
    yaxis = 0.25*array((-sin(goal2d[2]),cos(goal2d[2]),0))
    #self.hgoal = self.env.drawlinelist(transpose(c_[center-xaxis,center+xaxis,center-yaxis,center+yaxis]),linewidth=5.0,colors=array((0,1,0)))
    env.SetDebugLevel(DebugLevel.Debug)
    starttime = time.time()
    self.basemanip.MoveActiveJoints(goal=goal2d,maxiter=3000,steplength=0.05)
    print time.time()-starttime

def test_fatmodels():
    env=Environment()
    env.SetViewer('qtcoin')
    env.Load('scenes/r602kitchen3.env.xml')
    robot=env.GetRobots()[0]
    taskmanip=TaskManipulation(robot)
    env.SetDebugLevel(DebugLevel.Verbose)
    taskmanip.SwitchModels(switchpatterns=[('frootloops(+d)$','scenes/cereal_frootloops_fat.kinbody.xml')])
    taskmanip.SwitchModels(switch=True)

def test_pr2():
    from openravepy.examples import inversekinematics, grasping
    env=Environment()
    robot=env.ReadRobotXMLFile('robots/pr2-beta-sim.robot.xml')
    env.AddRobot(robot)
    # kinematics
    # python inversekinematics.py --robot=robots/pr2-beta-sim.robot.xml --manipname=rightarm --freejoint=r_shoulder_pan_joint --numiktests=10
    # python inversekinematics.py --robot=robots/pr2-beta-sim.robot.xml --manipname=rightarm_torso --freejoint=r_shoulder_pan_joint --freejoint=torso_lift_joint --numiktests=10
    manipnames = ['leftarm','rightarm','leftarm_torso','rightarm_torso']
    for manipname in manipnames:
        manip=robot.SetActiveManipulator(manipname)
        ikmodel=inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        rmodel = kinematicreachability.ReachabilityModel(robot)
        if not rmodel.load():
            rmodel.autogenerate()

    # grasping
    # python grasping.py --robot=robots/pr2-beta-sim.robot.xml --target=data/box_frootloops.kinbody.xml --boxdelta=0.01 --standoff=0 --standoff=0.02 --standoff=0.05 --normalanglerange=1 --roll=0 --roll=1.5707963 --roll=3.141592 --roll=4.7123889 --graspingnoise=0.01
    with env:
        target=env.ReadKinBodyXMLFile('data/box_frootloops.kinbody.xml')
        env.AddKinBody(target)
    for manipname in ['leftarm','rightarm']:
        robot.SetActiveManipulator(manipname)
        gmodel = grasping.GraspingModel(robot,target)
        if not gmodel.load():
            with gmodel.target:
                gmodel.target.Enable(False)
                final,traj = gmodel.basemanip.ReleaseFingers(execute=False,outputfinal=True)
            gmodel.generate(preshapes = array([final]),rolls=arange(0,2*pi,pi/2),graspingnoise=0.01,standoffs=[0,0.02,0.05],approachrays=gmodel.computeBoxApproachRays(0.01,normalanglerange=1,directiondelta=0.1))
    # create symbolic links

def test_calibviews():
    import calibrationviews
    env=Environment()
    env.SetViewer('qtcoin')
    env.Load('scenes/pa10lab.env.xml')#data/pa10calib_envcamera.env.xml')
    robot=env.GetRobots()[0]
    self = calibrationviews.CalibrationViews(robot,sensorrobot=env.GetRobot('ceilingcamera'))
    dists=arange(0.05,2.0,0.15)
    orientationdensity=1
    self.computeAndMoveToObservations()
    
    for i,relativepose in enumerate(visibilitytransforms):
        pose = array(posebase)
        pose = poseMult(pose,relativepose)
        q = self.vmodel.manip.FindIKSolution(dot(matrixFromPose(pose),self.Tpatternrobot),True)
        if q is not None:
            print i
            #self.robot.SetJointValues(q,self.vmodel.manip.GetArmJoints())
            self.vmodel.visualprob.ComputeVisibleConfiguration(pose=pose)
            raw_input('asdf')

def test_freejoints():
    env=Environment()
    env.SetViewer('qtcoin')
    env.Load('data/lab1.env.xml')
    robot=env.GetRobots()[0]
    robot.GetController().Reset(0)
    robot.SetJointValues(array([ -3.64122450e-01,   1.27151251e+00,  -7.88666554e-09, 1.29461884e+00,  -2.69412994e-05,   4.65967804e-01, 9.38504954e-08,   2.44345713e+00,   2.44345832e+00, 2.44345665e+00,   0.00000000e+00]))
    task=interfaces.TaskManipulation(robot)
    basemanip=interfaces.BaseManipulation(robot)
    m=robot.GetActiveManipulator()
    robot.SetActiveDOFs(m.GetArmIndices())
    basemanip.MoveUnsyncJoints(jointvalues=[0,0,0,0],jointinds=m.GetGripperIndices())
    task.ReleaseFingers()

def test_pr2movehandstraight():
    env=Environment()
    env.Load('robots/pr2-beta-static.robot.xml')
    robot=env.GetRobots()[0]
    RaveSetDebugLevel(DebugLevel.Debug)
    basemanip = interfaces.BaseManipulation(robot)
    lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
    if not lmodel.load():
        lmodel.autogenerate()
    lmodel.setRobotWeights()
    lmodel.setRobotResolutions()
    with env:        
        robot.GetController().Reset(0)
        robot.SetDOFValues(array([  3.92742505e+00,  -1.11998514e+02,  -1.12338294e+02, -2.58996561e+01,  -4.50039340e+02,  -4.94179434e+02, -1.02276493e+01,  -7.15039024e+02,  -7.36311760e+02, 1.02395814e+01,   2.59771698e+02,   2.57443795e+02, 2.30174678e-01,   1.46851569e-02,  -4.67573707e-01, 5.14696340e-03,   8.37552006e-01,   1.02808376e+00, -2.17092539e+00,   7.99473354e+00,  -1.90625735e+00, 3.75677048e-02,   7.16413010e-03,   9.25338303e-04, 1.30856421e-02,  -1.44843374e-01,   1.06201002e+00, -1.84726393e+00,  -1.53293256e+00,   1.22156997e+00, 4.56456176e-03,  -5.94439315e+00,   7.32226686e-03, 9.54206204e-04]))
        Tgoal = array([[  1.77635684e-15,   0.00000000e+00,   1.00000000e+00, 0.6],
                       [  0.00000000e+00,   1.00000000e+00,   0.00000000e+00, 0],
                       [ -1.00000000e+00,   0.00000000e+00,   1.77635684e-15, 1.2],
                       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])
        robot.SetActiveManipulator('rightarm_torso')
        manip = robot.GetManipulator('rightarm_torso')
        sol = manip.FindIKSolution(Tgoal,IkFilterOptions.CheckEnvCollisions)
    for iter in range(10):
        basemanip.MoveToHandPosition(matrices=[Tgoal],execute=False)
    
    robot.SetDOFValues(sol,manip.GetArmIndices())
