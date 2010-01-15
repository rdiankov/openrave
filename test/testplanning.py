from openravepy import *
import openravepy.examples
from openravepy.interfaces import *
from numpy import *
import numpy

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
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    self = inversekinematics.InverseKinematicsModel(robot=robot)
    freejoints=None
    usedummyjoints=False
    self.generate(freejoints=freejoints,usedummyjoints=usedummyjoints)

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
    rotthresh=0.25
    self.generate(heightthresh=heightthresh2,rotthresh=rotthresh)

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
    validgrasps = gm.computeValidGrasps(gm)
    Tee = gm.getGlobalGraspTransform(validgrasps[0])
    densityfn,samplerfn,bounds = self.getBaseDistribution(Tee,2000)
    h = self.showBaseDistribution(densityfn,bounds,zoffset=1.0,thresh=1.0)

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

def test_mobilemanipulation():
    import mobilemanipulation
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    env.Load('data/lab1.env.xml')
    robot = env.GetRobots()[0]
    self = mobilemanipulation.MobileManipulationPlanning(robot)
    gm=self.graspables[0][0]
    dests=self.graspables[0][1]
    validgrasps = gm.computeValidGrasps(gm)
    Tee = gm.getGlobalGraspTransform(validgrasps[0])
    densityfn,samplerfn,bounds = self.irmodel.getBaseDistribution(Tee,2000)
    h = self.irmodel.showBaseDistribution(densityfn,bounds,zoffset=1.0,thresh=0.0)
    Trobot = robot.GetTransform()
    print basedistfn(Trobot)
    self.graspAndPlaceObject(gm=gm,dests=dests)

def test_visibilityplanning():
    import visibilityplanning, time
    self = visibilityplanning.HRP2GraspingScene()
    time.sleep(5)
    self.loadscene(scenefilename='scenes/r602real.env.xml')
    self.testsim()

def test_hash():
    env = Environment()
    robot = env.ReadRobotXMLFile('robots/barrettsegway.robot.xml')
    env.AddRobot(robot)
    s = robot.serialize(SerializationOptions.Kinematics)
    hash = robot.GetKinematicsGeometryHash()
    print hash

def test_pyann():
    ktree = pyANN.KDTree(random.rand(10,7))
    neighs,dists = ktree.kSearch(random.rand(7),5,1e-3);
