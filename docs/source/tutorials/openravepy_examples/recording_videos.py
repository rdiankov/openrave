"""Start and stop recording videos using the Python API.
"""
from openravepy import *
import scipy
import time
env = Environment() # create openrave environment
env.SetViewer('qtcoin')
env.Load('data/lab1.env.xml') # load a simple scene

recorder = RaveCreateModule(env,'viewerrecorder')
env.AddModule(recorder,'')
filename = 'openrave.mpg'
codec = 13 # mpeg2
recorder.SendCommand('Start 640 480 30 codec %d timing realtime filename %s\nviewer %s'%(codec,filename,env.GetViewer().GetName()))
