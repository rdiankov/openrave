"""Start and stop recording videos using the Python API.
"""
from openravepy import *
import time

env = Environment() # create openrave environment
env.SetViewer('qtcoin')
env.Load('data/lab1.env.xml') # load a simple scene

recorder = RaveCreateModule(env,'viewerrecorder')
env.AddModule(recorder,'')
codecs = recorder.SendCommand('GetCodecs') # linux only
filename = 'openrave.mpg'
codec = 13 # mpeg4
recorder.SendCommand('Start 640 480 30 codec %d timing realtime filename %s\nviewer %s'%(codec,filename,env.GetViewer().GetName()))
time.sleep(5)
recorder.SendCommand('Stop') # stop the video
env.Remove(recorder) # remove the recorder
