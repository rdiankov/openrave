"""Save the current scene using the :ref:`probleminstance-logging` plugin:
"""
from openravepy import *
env = Environment() # create openrave environment
env.Load('data/lab1.env.xml') # load a simple scene

logger = RaveCreateModule(env,'logging')
logger.SendCommand('savescene filename myscene.env.xml')
