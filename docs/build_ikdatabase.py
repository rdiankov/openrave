# -*- coding: utf-8 -*-
# Copyright (C) 2011 Rosen Diankov (rosen.diankov@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import openravepy
from openravepy import *
from numpy import *
from optparse import OptionParser
import os, sys, operator
import scipy
import shutil
import pysvn
from openravepy import ikfast

imagedir = 'images/robots'
imagelinkdir = '../images/robots'

def floatRgb(mag, cmin=0.0, cmax=1.0):
    """Return a tuple of floats between 0 and 1 for the red, green and blue amplitudes.
    """
    try:
        # normalize to [0,1]
        x = float(mag-cmin)/float(cmax-cmin)
    except:
        # cmax = cmin
        x = 0.5
    blue = min((max((4*(0.75-x), 0.)), 1.))
    red = min((max((4*(x-0.25), 0.)), 1.))
    green = min((max((4*math.fabs(x-0.5)-1., 0.)), 1.))
    return [red, green, blue,0.5]

def writetable(rows):
    colwidths = [max([len(row[i]) for row in rows]) for i in range(len(rows[0]))]
    for row in rows:
        for i in range(len(row)):
            row[i] = row[i].ljust(colwidths[i])+'|'
        row[0] = '|'+row[0]
    seprow = ['-'*colwidth + '+' for colwidth in colwidths]
    sep = '+'+''.join(seprow)
    xml = sep+'\n'
    for row in rows:
        xml += ''.join(row)+'\n'+sep+'\n'
    xml += '\n\n'
    return xml

def buildrobot(outputdir, env, robotfilename, robotstats,buildoptions):
    width=480
    height=480
    focal = 10000.0 # bigger values make orthographic view
    robotname = os.path.split(os.path.splitext(robotfilename)[0])[1]
    sourceoutputdir = os.path.join(outputdir,robotname)
    try:
        os.makedirs(sourceoutputdir)
    except OSError:
        pass
    imagename = '%s.jpg'%robotname
    robot = None
    roboturl = ''
    if env is not None:
        env.Reset()
        robot=env.ReadRobotXMLFile(robotfilename)
        if robot is None:
            print 'failed ',robotfilename
        else:
            print 'processing ',robotname
            env.AddRobot(robot)
            try:
                entry = pysvn.Client().info(robot.GetXMLFilename())
                roboturl = str(entry.url)
            except pysvn.ClientError:
                try:
                    # try looking in the current robots directory
                    entry = pysvn.Client().info(os.path.join('..','src','robots',os.path.split(robot.GetXMLFilename())[1]))
                    roboturl = str(entry.url)
                except pysvn.ClientError:
                    pass
                
            # transform the robot so we don't render it at its axis aligned (white robots completely disappear)
            #robot.SetTransform(matrixFromQuat(quatRotateDirection([-1,0,0],[-1,1,0.5])))
            with env:
                robot.SetTransform(eye(4))
                ab = robot.ComputeAABB()
                T = eye(4)
                T[0:3,3] = -ab.pos()
                robot.SetTransform(T)
                # draw the axes
                N = [float(len(robot.GetJoints())),float(len(robot.GetPassiveJoints()))]
                scale = 0.3*linalg.norm(ab.extents())
                linewidth = 2
                h = [env.drawlinelist(array([j.GetAnchor()-scale*j.GetAxis(0),j.GetAnchor()+scale*j.GetAxis(0)]),linewidth,floatRgb(float(j.GetDOFIndex())/(N[0]+N[1])))  for j in robot.GetJoints() if not j.IsStatic()]
                hpassive = [env.drawlinelist(array([j.GetAnchor()-scale*j.GetAxis(0),j.GetAnchor()+scale*j.GetAxis(0)]),linewidth,floatRgb((jindex+N[0])/(N[0]+N[1])))  for jindex,j in enumerate(robot.GetPassiveJoints()) if not j.IsStatic()]
            K=[focal,focal,width/2,height/2]
            Lall = max(linalg.norm(ab.extents())*focal/(0.5*width),linalg.norm(ab.extents())*focal/(0.5*height))
            Lx = ab.extents()[0] + max(ab.extents()[1]*focal/(0.5*width),ab.extents()[2]*focal/(0.5*height))
            Ly = ab.extents()[1] + max(ab.extents()[0]*focal/(0.5*width),ab.extents()[2]*focal/(0.5*height))
            L = max([Lall,Lx,Ly]) # need to use the same length in order to preserve scale
            Tall=transformLookat(lookat=zeros(3),camerapos=ones(3)*L/sqrt(3),cameraup=[0,0,-1])
            Iall = viewer.GetCameraImage(width=width,height=height,transform=Tall,K=K)
            Tx=transformLookat(lookat=zeros(3),camerapos=array([1.0,0.0,0])*L,cameraup=[0,0,-1])
            Ix = viewer.GetCameraImage(width=width,height=height,transform=Tx,K=K)
            Ty=transformLookat(lookat=zeros(3),camerapos=array([0.0,-1.0,0])*L,cameraup=[0,0,-1])
            Iy = viewer.GetCameraImage(width=width,height=height,transform=Ty,K=K)
            scipy.misc.pilutil.imsave(os.path.join(imagedir,imagename),hstack([Iall,Ix,Iy]))

    print 'writing ',robotname
    robotlink = 'robot-'+robotname
    robotxml = """.. _%s:\n\n%s Robot\n%s======

.. image:: ../%s/%s
  :width: 640

**filename:** `%s <%s>`_

:ref:`ikfast_generatedcpp`

"""%(robotlink,robotname,'='*len(robotname),imagelinkdir,imagename,robotfilename,roboturl)

    # sort the stats based on manipulator name and ik type
    def statscmp(x,y):
        if x[0] == y[0]:
            if x[1] == y[1]:
                return cmp(x[2],y[2])
            
            else:
                return cmp(x[1],y[1])
            
        else:
            return cmp(x[0],y[0])
        
    robotstats.sort(statscmp)
    prevmanipname = None
    previktypestr = None
    rownames = ['free indices','success rate','wrong rate','mean time (s)','max time (s)','source','detailed results']
    rows = None
    successes = 0
    for manipname, iktypestr, freeindices, description, sourcefilename, meantime, maxtime, successrate, wrongrate, sourcecode in robotstats:
        if prevmanipname != manipname:
            if rows is not None:
                robotxml += writetable(rows)
                rows = None
            name = '**' + manipname + '** Manipulator'
            robotxml += name + '\n' + '-'*len(name) + '\n\n'
            if robot is not None:
                manip = robot.GetManipulator(manipname)
                if manip is not None:
                    robotxml += 'Manipulator Indices: %s\n\n'%manip.GetArmIndices()
            prevmanipname = manipname
        if previktypestr != iktypestr:
            if rows is not None:
                robotxml += writetable(rows)
                rows = None
            name = iktypestr + ' IK'
            robotxml += name + '\n' + '~'*len(name) + '\n\n'
            previktypestr = iktypestr

        if sourcefilename is not None:
            sourcefilename_tail = os.path.split(sourcefilename)[1]
            open(os.path.join(sourceoutputdir,sourcefilename_tail),'w').write(sourcecode)
            sourcename = ':download:`C++ Code <%s/%s>`'%(robotname,sourcefilename_tail)
        else:
            sourcename = ':red:`Failed`'
        descriptionurl = description[0].replace(':','_')
        descriptionurl = descriptionurl.replace(' ','_')
        descriptionurl = descriptionurl.replace('[','_')
        descriptionurl = descriptionurl.replace(']','_')
        descriptionurl = descriptionurl.replace('(','_')
        descriptionurl = descriptionurl.replace(')','_')
        descriptionurl = descriptionurl.replace(',','_')
        descriptionurl = descriptionurl.replace('/','_')
        description_parts = descriptionurl.split('.')
        testingname = '`View <%stestReport/(root)/%s/_%s_>`_'%(buildoptions.jenkinsbuild_url,description_parts[0],description_parts[1])
        if rows is None:
            rows = [list(rownames)]
        row = [','.join(str(index) for index in freeindices),]
        if successrate is not None:
            row.append('%.4f'%successrate)
        else:
            row.append('')
        if wrongrate is not None:
            if wrongrate == 0:
                row.append('%.4f'%wrongrate)
                successes += 1
            else:
                row.append(':red:`%.4f`'%wrongrate)
        else:
            row.append('')
        if meantime is not None:
            row.append('%.6f'%meantime)
        else:
            row.append('')
        if maxtime is not None:
            row.append('%.6f'%maxtime)
        else:
            row.append('')
        row.append(sourcename)
        row.append(testingname)
        rows.append(row)
    if rows is not None:
        robotxml += writetable(rows)
    open(os.path.join(outputdir,robotname+'.rst'),'w').write(robotxml)
    returnxml = """
:ref:`%s`
%s

**IK parameterizations:** %d, **Success:** %d%%

.. only:: htmltag

  .. image:: ../%s/%s
    :width: 640
    :target: %s.html

.. only:: jsontag

  .. image:: ../%s/%s
    :width: 640
    :target: ../%s/

----

"""%(robotlink,'~'*(len(robotlink)+7),len(robotstats),100*float(successes)/len(robotstats),imagelinkdir,imagename,robotname,imagelinkdir,imagename,robotname)
    return returnxml,robotname

def build(allstats,buildoptions,outputdir,env):
    try:
        os.makedirs(outputdir)
    except OSError:
        pass

    # write each robot
    robotdict = dict()
    for stat in allstats:
        if len(stat) == 10:
            stat.append(None)
        if len(stat) == 11:
            if not stat[0] in robotdict:
                robotdict[stat[0]] = []
            robotdict[stat[0]].append(stat[1:])
    robotlist = sorted(robotdict.iteritems(), key=operator.itemgetter(0))
    robotxml = ''
    robotnames = []
    for robotfilename, robotstats in robotlist:
        # only show robots that don't have fial in their name
        if os.path.split(robotfilename)[1].find('fail') < 0:
            xml,robotname = buildrobot(outputdir,env,robotfilename, robotstats,buildoptions)
            robotxml += xml
            robotnames.append(robotname)

    iktypes = ', '.join(iktype.name for iktype in IkParameterization.Type.values.values())
    text="""
.. _robots:

Robots Database
===============

A database of robots and statistics automatically generated by the `OpenRAVE Testing Server <http://www.openrave.org/testing>`_ for each OpenRAVE release. URL links to the robot files and results history are provided.

.. toctree::
  :maxdepth: 2
  
  robots_overview
  %s/index

Robots
------

`[testing run] <%s>`_

%s

.. toctree::
  :maxdepth: 1
  :hidden:

"""%(outputdir,buildoptions.jenkinsbuild_url, robotxml)
    for robotname in robotnames:
        text += '  %s\n'%(robotname)
    open(os.path.join(outputdir,'robots.rst'),'w').write(text)
    freeparameters = ', '.join('%s free - %s tests'%(i,num) for i,num in enumerate(buildoptions.numiktests))
    text="""
.. _ikfast-database:

IKFast Robot Database
=====================

  :Release: **%s**

Inverse kinematics statistics using :ref:`ikfast_compiler`. All possible permutations of the :ref:`supported inverse kinematics types <ikfast_types>` and the robot manipulators are tested.

.. _ikfast-testing:

IKFast Performance Testing
--------------------------

There are four different ways of calling the IK solver:

* GetSolution(*goal*) - call with just the end effector parameterization, return the first solution found within limits.
* GetSolution(*goal*,*free*) - call with end effector parameterization and specify all free joint values, return the first solution found within limits.
* GetSolutions(*goal*) - call with just the end effector parameterization, return all solutions searching through all free joint values.
* GetSolutions(*goal*,*free*) - call with end effector parameterization and specify all free joint values, return all possible solutions.

The following algorithm tests these four calls by:

1. Randomly sample a robot configuration and compute the correct parameterization of the end effector *goal* and free joint values *free*.
2. Move the robot to a random position.
3. Call GetSolution(*goal*), GetSolutions(*goal*), GetSolutions(*goal*,*free*)
4. Call GetSolution(*goal*,*free_random*) and GetSolutions(*goal*,*free_random*). Check that the returned solutions have same values as free parameters.

For every end effector parameterization and a returned solution, set the returned solution on the robot and compute the error between the original end effector and the new end effector. If the error is *greater than %f*, then the IK is wrong and the iteration is a *failure*. If no wrong solutions were returned and at least one correct IK solution is found within limits, then the iteration is a *success*. When the free values are not specified, the IK solver will discretize the range of the freevalues and test with all possible combinations [1]_. 

The number of tests is determined by the number of free parameters: %s

Four values are extracted to measure the performance of a generated IK solver:

* wrong rate - number of parameterizations where at least one wrong solution was returned.
* success rate - number of parameterizations where all returned solutions are correct
* no solution rate - number of parameterizations where no solutions were found within limits
* missing solution rate - number of parameterizations where the specific sampled solution was not returned, but at least one solution was found.

An IK is **successful** if the wrong rate is 0, success rate is > %f, and the no solution rate is < %f. 
The raw function call run-time is also recorded.

Degenerate configurations can frequently occur when the robot axes align, this produces a lot of divide by zero conditions inside the IK. In order to check all such code paths, the configuration sampler common tests angles 0, pi/2, pi, and -pi/2.

.. [1] The discretization of the free joint values depends on the robot manipulator and is given in each individual manipulator page.
"""%(ikfast.__version__, buildoptions.errorthreshold,freeparameters,buildoptions.minimumsuccess,buildoptions.maximumnosolutions)
    open(os.path.join(outputdir,'index.rst'),'w').write(text)

if __name__ == "__main__":
    parser = OptionParser(description='Builds the ik database')
    parser.add_option('--outdir','--outputdir',action="store",type='string',dest='outputdir',default='ikfast',
                      help='Output directory to write all interfaces reStructuredText files inside the folder (default=%default).')
    parser.add_option('--ikfaststats',action="store",type='string',dest='ikfaststats',default='ikfaststats.pp',
                      help='The python pickled file containing ikfast statistics.')
    (options,args) = parser.parse_args()

    if not os.path.exists(options.ikfaststats):
        sys.exit(1)

    try:
        # have to clean the directory since cached files can get in the way
        shutil.rmtree(options.outputdir)
    except OSError:
        pass

    try:
        os.makedirs(imagedir)
    except OSError:
        pass
    allstats,buildoptions = pickle.load(open(options.ikfaststats,'r'))
    env=Environment()
    try:
        env.SetViewer('qtcoin',False)
        viewer=env.GetViewer()
        viewer.SetBkgndColor([0.94,0.99,0.99])
        viewer.SendCommand('SetFiguresInCamera 1')
        build(allstats,buildoptions,options.outputdir,env)
    finally:
        RaveDestroy()
