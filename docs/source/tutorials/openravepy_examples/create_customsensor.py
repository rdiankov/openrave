"""Creates a custom kinematics body with two links and one joint
"""
from openravepy import *
from numpy import eye, array, zeros

env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
with env:
    robot=RaveCreateRobot(env,'')
    robot.SetName('camera')

    linkinfo=KinBody.LinkInfo()
    linkinfo._name='camerabase'

    ginfo=KinBody.GeometryInfo()
    ginfo._type=GeometryType.Box
    ginfo._vGeomData=[0.1,0.1,0.1] # box extents
    ginfo._vDiffuseColor=[0,0,1]
    ginfo._t = eye(4)
    linkinfo._vgeometryinfos = [ginfo]

    camera1info=Robot.AttachedSensorInfo()
    camera1info._linkname='camerabase'
    camera1info._name = 'ensenson10'
    camera1info._sensorname = 'base_pinhole_camera'
    camera1info._trelative = eye(4)
    camera1info._trelative[0:3,3] = [0,0,0.1]
    camera1info._sensorgeometry = CameraGeomData()
    camera1info._sensorgeometry.width = 640
    camera1info._sensorgeometry.height = 480
    camera1info._sensorgeometry.intrinsics.K = array([[640.0,0,320],[0,640,240],[0,0,1]])
    camera1info._sensorgeometry.intrinsics.distortion_coeffs = zeros(5)
    camera1info._sensorgeometry.intrinsics.distortion_model = 'opencv'
    camera1info._sensorgeometry.intrinsics.focal_length = 0.05

    robot.Init([linkinfo],[],[],[])

    env.Add(robot)

    robot.AddAttachedSensor(camera1info,True)
