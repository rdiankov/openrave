"""Save a 640x480 image from the viewer.
"""
from openravepy import *
import scipy
import time
env = Environment() # create openrave environment
env.SetViewer('qtcoin')
env.Load('data/lab1.env.xml') # load a simple scene
time.sleep(1) # wait for viewer to initialize

env.GetViewer().SendCommand('SetFiguresInCamera 1') # also shows the figures in the image
I = env.GetViewer().GetCameraImage(640,480,  env.GetViewer().GetCameraTransform(),[640,640,320,240])
scipy.misc.imsave('openrave.jpg',I)
