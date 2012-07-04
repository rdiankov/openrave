"""Rotates all bodies along world z-direction by 45 degrees:
"""
from openravepy import *
import numpy
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('data/lab1.env.xml') # load a simple scene

Tz = matrixFromAxisAngle([0,0,numpy.pi/4])
with env:
    for body in env.GetBodies():
       body.SetTransform(numpy.dot(Tz,body.GetTransform()))
