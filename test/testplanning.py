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
    validgrasps = gm.computeValidGrasps(gm)
    Tgrasp = gm.getGlobalGraspTransform(validgrasps[0])
    densityfn,samplerfn,bounds = self.computeBaseDistribution(Tgrasp,2000)
    h = self.showBaseDistribution(densityfn,bounds,zoffset=1.0,thresh=1.0)
    densityfn2,samplerfn2,bounds2 = self.computeAggregateBaseDistribution([Tgrasp],2000)
    h2 = self.showBaseDistribution(densityfn2,bounds2,zoffset=3.0,thresh=1.0)

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

def test_ikfast():
    import openravepy
    env = openravepy.Environment()
    robot = env.ReadRobotXMLFile('schunkleft.robot.xml')
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

