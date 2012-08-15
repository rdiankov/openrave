"""Save the current scene in COLLADA format
"""
from openravepy import *
env = Environment() # create openrave environment
env.Load('data/lab1.env.xml') # load a simple scene
# save in a zip archive
env.Save('newlab.zae',Environment.SelectionOptions.Everything)
