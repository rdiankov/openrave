"""Creates a custom kinematics body with two links and one joint
"""
from openravepy import *
import numpy, time
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
with env:
    # geometries
    infobox0 = KinBody.Link.GeometryInfo()
    infobox0._type = GeometryType.Box
    infobox0._t[0,3] = 0
    infobox0._vGeomData = [0.1,0.2,0.3]
    infobox0._vDiffuseColor = [1,0,0]
    infobox1 = KinBody.Link.GeometryInfo()
    infobox1._type = GeometryType.Box
    infobox1._t[0,3] = 0.1
    infobox1._vGeomData = [0.3,0.05,0.05]
    infobox1._vDiffuseColor = [0,1,0]
    infobox2 = KinBody.Link.GeometryInfo()
    infobox2._type = GeometryType.Box
    infobox2._t[0,3] = 0
    infobox2._vGeomData = [0.1,0.2,0.3]
    infobox2._vDiffuseColor = [0,0,1]
    # links
    link0 = KinBody.LinkInfo()
    link0._vgeometryinfos = [infobox0, infobox1]
    link0._name = 'link0'
    link0._mapFloatParameters = {'param0':[1,2.3]}
    link0._mapIntParameters = {'param0':[4,5.6]}
    link1 = KinBody.LinkInfo()
    link1._vgeometryinfos = [infobox2]
    link1._name = 'link1'
    link1._mapFloatParameters = {'param0':[1,2.3]}
    link1._mapIntParameters = {'param0':[4,5.6]}
    link1._t[0,3] = 0.5
    # joints
    joint0 = KinBody.JointInfo()
    joint0._name = 'j0'
    joint0._linkname0 = 'link0'
    joint0._linkname1 = 'link1'
    joint0._type = KinBody.JointType.Hinge
    joint0._vlowerlimit = [-0.5]
    joint0._vupperlimit = [1.0]
    joint0._vaxes = [[0,0,1]]
    # instantiate
    body = RaveCreateKinBody(env,'')
    success=body.Init([link0,link1],[joint0])
    body.SetName('temp')
    env.Add(body)
