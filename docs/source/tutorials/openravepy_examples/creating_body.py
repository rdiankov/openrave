"""Creates a box and then update the box's geometry dynamically.
"""
from openravepy import *
import numpy, time
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
with env:
    body = RaveCreateKinBody(env,'')
    body.SetName('testbody')
    body.InitFromBoxes(numpy.array([[0,0,0,0.1,0.2,0.3]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
    env.Add(body,True)

time.sleep(2) # sleep 2 seconds
with env:
    env.Remove(body)
    body.InitFromBoxes(numpy.array([[-0.4,0,0,0.1,0.2,0.3],[0.4,0,0,0.1,0.2,0.9]]),True) # set geometry as two boxes
    env.Add(body,True)

time.sleep(2) # sleep 2 seconds

# create from soup of cylinders
with env:
    infocylinder = KinBody.GeometryInfo()
    infocylinder._type = GeometryType.Cylinder
    infocylinder._t[0,3] = 0.1
    infocylinder._vGeomData = [0.1,0.4]
    infocylinder._bVisible = True
    infocylinder._fTransparency = 0.5
    infocylinder._vDiffuseColor = [1,0,0]
    infocylinder2 = KinBody.GeometryInfo()
    infocylinder2._type = GeometryType.Cylinder
    infocylinder2._t[0,3] = -0.1
    infocylinder2._vGeomData = [0.2,0.4]
    infocylinder2._bVisible = True
    infocylinder2._fTransparency = 0.5
    infocylinder2._vDiffuseColor = [0.5,0.5,0]
    k3 = RaveCreateKinBody(env,'')
    k3.InitFromGeometries([infocylinder,infocylinder2])
    k3.SetName('tempcylinder')
    env.Add(k3,True)
