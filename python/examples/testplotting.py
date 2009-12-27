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
from openravepy import *
from numpy import *

def run():
    orenv = Environment()
    orenv.SetViewer('qtcoin')
    h1 = orenv.plot3(points=array((-1.5,-1,0)),
                pointsize=5.0,
                colors=array((1,0,0)))
    h2 = orenv.plot3(points=array(((-1.5,-0.5,0),(-1.5,0.5,0))),
                pointsize=15.0,
                colors=array(((0,1,0),(0,0,0))))
    h3 = orenv.drawlinestrip(points=array(((-1.25,-0.5,0),(-1.25,0.5,0),(-1.5,1,0))),
                        linewidth=3.0,
                        colors=array(((0,1,0),(0,0,1),(1,0,0))))
    h4 = orenv.plot3(points=array(((-0.5,-0.5,0),(-0.5,0.5,0))),
                pointsize=0.05,
                colors=array(((0,1,0),(1,1,0))),
                drawstyle=1)
    h5 = orenv.drawtrimesh(points=array(((0,0,0),(0.5,0,0),(0,0.5,0))),
                      indices=None,
                      colors=array(((0,1,0),(0,0,1),(1,0,0))))
    h6 = orenv.drawtrimesh(points=array(((0,0,0.5),(0.5,0,0.5),(0,0.5,0.5),(0.5,0.5,0.5))),
                      indices=array(((0,1,2),(2,1,3)),int64),
                      colors=array((1,0,0,0.5)))
    raw_input('Enter any key to quit. ')
    h1 = None
    h2 = None
    h3 = None
    h4 = None
    h5 = None
    h6 = None
    orenv.Destroy()
    
if __name__=='__main__':
    run()
