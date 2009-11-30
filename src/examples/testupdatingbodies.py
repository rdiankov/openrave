#!/usr/bin/env python
# Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
import time
from openravepy import *
from numpy import *

if __name__ == "__main__":
    print 'Example shows how to manually update the environment published bodies for the viewer while the environment is locked'
    env = Environment()
    env.Load('data/lab1.env.xml')
    env.SetViewer('qtcoin')
    robot = env.GetRobots()[0]    
    manipprob = env.CreateProblem('basemanipulation')
    env.LoadProblem(manipprob,robot.GetName())

    Tcamera = array(((0.84028,  -0.14715,   0.52179,0.930986),
                     (0.52639,   0.45182,  -0.72026,-1.233453),
                     (-0.12976,   0.87989,   0.45711,2.412977)))
    env.SetCamera(Tcamera)
    destarmangles =  array((-0.75,1.24,-0.064,2.33,-1.16,-1.548,1.19))
    env.GetViewer().EnvironmentSync()
    
    print 'Stopping the environment loop from updating the simulation'
    env.StopSimulation()
    print 'Locking environment and starting to plan'
    env.LockPhysics(True)
    manipprob.SendCommand('MoveManipulator armvals ' + ' '.join(str(f) for f in destarmangles))
    
    print 'Calling the simulation loop internally to python'
    while not robot.GetController().IsDone():
        env.StepSimulation(0.01)
        env.UpdatePublishedBodies() # used to publish body information while environment is locked
        time.sleep(0.1)
    env.LockPhysics(False)
    
    raw_input('press any key to exit: ')
    env.Destroy() # done with the environment

