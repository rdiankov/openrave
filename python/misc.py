# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
"""Misc openravepy functions. Need to explicitly import to use them.
"""
from __future__ import with_statement # for python 2.5
import openravepy_int
import openravepy_ext
import os.path
from sys import platform as sysplatformname
from sys import stdout
import numpy
try:
    from itertools import izip
except ImportError:
    pass

try:
    from threading import Thread
except ImportError:
    pass

import logging
log = logging.getLogger('openravepy.'+__name__.split('.',2)[-1])

def mkdir_recursive(newdir):
    log.warn('openravepy.misc.mkdir_recursive is deprecated, please use os.makedirs')
    from os import makedirs
    try:
        makedirs(newdir)
    except OSError:
        pass

try:
    from os.path import relpath    
except ImportError:
    # relpath is not present in python 2.5 and below, so hold an implementation of it.
    from posixpath import curdir, sep, pardir, join, abspath, commonprefix
    def relpath(path, start=curdir):
        """Return a relative version of a path"""
        if not path:
            raise ValueError("no path specified")
        
        start_list = abspath(start).split(sep)
        path_list = abspath(path).split(sep)
        # Work out how much of the filepath is shared by start and path.
        i = len(commonprefix([start_list, path_list]))
        rel_list = [pardir] * (len(start_list)-i) + path_list[i:]
        return curdir if not rel_list else join(*rel_list)

def LoadTrajectoryFromFile(env,trajfile,trajtype=''):
    return openravepy_int.RaveCreateTrajectory(env,trajtype).deserialize(open(trajfile,'r').read())

def InitOpenRAVELogging(stream=stdout):
    """Sets the python logging **openravepy** scope to the same debug level as OpenRAVE and initializes handles if they are not present
    """
    levelmap = {openravepy_int.DebugLevel.Verbose:logging.DEBUG, openravepy_int.DebugLevel.Debug:logging.DEBUG, openravepy_int.DebugLevel.Info:logging.INFO, openravepy_int.DebugLevel.Warn:logging.WARN, openravepy_int.DebugLevel.Error:logging.ERROR, openravepy_int.DebugLevel.Fatal:logging.FATAL }
    log=logging.getLogger('openravepy')
    log.setLevel(levelmap[openravepy_int.RaveGetDebugLevel()&0xffff])
    if len(log.handlers) == 0:
        try:
            import codecs
            colorize=__import__('logutils.colorize',fromlist=['colorize'])
            handler = colorize.ColorizingStreamHandler(codecs.getwriter('utf-8')(stream))
            handler.level_map[logging.DEBUG] =(None, 'green', False)
            handler.level_map[logging.INFO] = (None, None, False)
            handler.level_map[logging.WARNING] = (None, 'yellow', False)
            handler.level_map[logging.ERROR] = (None, 'red', False)
            handler.level_map[logging.CRITICAL] = ('white', 'magenta', True)

        except ImportError:
            handler = logging.StreamHandler(stream)
            openravepy_int.raveLogVerbose('python logutils not present so cannot colorize python output.')

        handler.setFormatter(logging.Formatter('%(name)s: %(funcName)s, %(message)s'))
        log.addHandler(handler)

def SetViewerUserThread(env,viewername,userfn):
    """Adds a viewer to the environment if one doesn't exist yet and starts it on this thread. Then creates a new thread to call the user-defined function to continue computation.
    This function will return when the viewer and uesrfn exits. If userfn exits first, then will quit the viewer
    """
    if env.GetViewer() is not None or viewername is None:
        userfn()
    viewer = None
    if sysplatformname.startswith('darwin'):
        viewer = openravepy_int.RaveCreateViewer(env,viewername)
    else:
        # create in a separate thread for windows and linux since the signals do not get messed up
        env.SetViewer(viewername)
    if viewer is None:
        userfn()
    # add the viewer before starting the user function
    env.Add(viewer)
    threading = __import__('threading')
    Thread = threading.Thread
    def localuserfn(userfn,viewer):
        try:
            userfn()
        finally:
            # user function quit, so have to destroy the viewer
            viewer.quitmainloop()
    userthread = Thread(target=localuserfn,args=(userfn,viewer))
    userthread.start()
    sig_thread_id = 0
    for tid, tobj in threading._active.items():
        if tobj is userthread:
            sig_thread_id = tid
            break
    try:
        viewer.main(True,sig_thread_id)
    finally:
        userthread.join()
            
class OpenRAVEGlobalArguments:
    """manages a global set of command-line options applicable to all openrave environments"""
    @staticmethod
    def addOptions(parser,testmode=True):
        from optparse import OptionGroup
        ogroup = OptionGroup(parser,"OpenRAVE Environment Options")
        ogroup.add_option('--loadplugin', action="append",type='string',dest='_loadplugins',default=[],
                          help='List all plugins and the interfaces they provide.')
        ogroup.add_option('--collision', action="store",type='string',dest='_collision',default=None,
                          help='Default collision checker to use')
        ogroup.add_option('--physics', action="store",type='string',dest='_physics',default=None,
                          help='physics engine to use (default=%default)')
        ogroup.add_option('--viewer', action="store",type='string',dest='_viewer',default=None,
                          help='viewer to use (default=qtcoin)' )
        ogroup.add_option('--server', action="store",type='string',dest='_server',default=None,
                          help='server to use (default=None).')
        ogroup.add_option('--serverport', action="store",type='int',dest='_serverport',default=4765,
                          help='port to load server on (default=%default).')
        ogroup.add_option('--module', action="append",type='string',dest='_modules',default=[],nargs=2,
                          help='module to load, can specify multiple modules. Two arguments are required: "name" "args".')
        ogroup.add_option('--level','-l','--log_level', action="store",type='string',dest='_level',default=None,
                          help='Debug level, one of (%s)'%(','.join(str(debugname).lower() for debuglevel,debugname in openravepy_int.DebugLevel.values.iteritems())))
        if testmode:
            ogroup.add_option('--testmode', action="store_true",dest='testmode',default=False,
                              help='if set, will run the program in a finite amount of time and spend computation time validating results. Used for testing')
        parser.add_option_group(ogroup)
    @staticmethod
    def parseGlobal(options,**kwargs):
        """Parses all global options independent of the environment"""
        if options._level is not None:
            for debuglevel,debugname in openravepy_int.DebugLevel.values.iteritems():
                if (not options._level.isdigit() and options._level.lower() == debugname.name.lower()) or (options._level.isdigit() and int(options._level) == int(debuglevel)):
                    openravepy_int.RaveSetDebugLevel(debugname)
                    break

        InitOpenRAVELogging()
        
    @staticmethod
    def parseEnvironment(options,env,defaultviewer=False,returnviewer=False,**kwargs):
        """Parses all options that affect the environment. If returnviewer is set, will return the viewer to set instead of setting it"""
        try:
            if options._collision:
                cc = openravepy_int.RaveCreateCollisionChecker(env,options._collision)
                if cc is not None:
                    env.SetCollisionChecker(cc)
        except openravepy_ext.openrave_exception, e:
            log.warn(e)
        try:
            if options._physics:
                ph = openravepy_int.RaveCreatePhysicsEngine(env,options._physics)
                if ph is not None:
                    env.SetPhysicsEngine(ph)
        except openravepy_ext.openrave_exception, e:
            log.warn(e)
        try:
            if options._server:
                sr = openravepy_int.RaveCreateModule(env,options._server)
                if sr is not None:
                    env.Add(sr,True,'%d'%options._serverport)
        except openravepy_ext.openrave_exception, e:
            log.warn(e)
        for name,args in options._modules:
            try:
                module = openravepy_int.RaveCreateModule(env,name)
                if module is not None:
                    env.Add(module,True,args)
            except openravepy_ext.openrave_exception, e:
                log.warn(e)
        try:
            viewername=None
            if options._viewer is not None:
                if len(options._viewer) > 0:
                    viewername=options._viewer
            elif defaultviewer:
                viewername='qtcoin'
            if returnviewer:
                return viewername
            elif viewername is not None:
                env.SetViewer(viewername)
        except openravepy_ext.openrave_exception, e:
            log.warn(e)
            
    @staticmethod
    def parseAndCreate(options,createenv=openravepy_int.Environment,returnviewer=False,**kwargs):
        """Parse all options and create the global Environment. The left over arguments are passed to the parse functions.
        If returnviewer is False, the viewer is created in a separate thread, so this method will not work for MacOSX if this is the main executing thread.
        """
        openravepy_int.RaveInitialize(True)
        for plugin in options._loadplugins:
            openravepy_int.RaveLoadPlugin(plugin)
        OpenRAVEGlobalArguments.parseGlobal(options,**kwargs)
        if createenv is None:
            return None
        env = createenv()
        viewername = OpenRAVEGlobalArguments.parseEnvironment(options,env,returnviewer=returnviewer,**kwargs)
        if returnviewer:
            return env,viewername
        else:
            return env

    @staticmethod
    def parseAndCreateThreadedUser(options,userfn,createenv=openravepy_int.Environment,returnviewer=True,**kwargs):
        """Parse all options and create the global Environment. The left over arguments are passed to the parse functions.
        If a viewer is requested, it is created in this thread, and another thread is executed with the user function. This is required for OSes that require viewer thread to be in main thread (Mac OSX)
        :param userfn: Call with userfn(env,options)
        :return: nothing
        """
        openravepy_int.RaveInitialize(True)
        for plugin in options._loadplugins:
            openravepy_int.RaveLoadPlugin(plugin)
        OpenRAVEGlobalArguments.parseGlobal(options,**kwargs)
        if createenv is None:
            raise openravepy_ext.openrave_exception('failed to create environment')
        env = createenv()
        viewername = OpenRAVEGlobalArguments.parseEnvironment(options,env,returnviewer=True,**kwargs)
        SetViewerUserThread(env,viewername,lambda: userfn(env,options))
        
def ComputeGeodesicSphereMesh(radius=1.0,level=2):
    """Computes a geodesic sphere to a specified level. Returns the vertices and triangle indices"""
    GTS_M_ICOSAHEDRON_X = numpy.sqrt(numpy.sqrt(5)+1)/numpy.sqrt(2*numpy.sqrt(5))
    GTS_M_ICOSAHEDRON_Y = numpy.sqrt(2)/numpy.sqrt(5+numpy.sqrt(5))
    GTS_M_ICOSAHEDRON_Z = 0.0
    vertices = [numpy.array((+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y)),
                numpy.array((+GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                numpy.array((+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X)),
                numpy.array((+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X)),
                numpy.array((+GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                numpy.array((+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y)),
                numpy.array((-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X)),
                numpy.array((+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y)),
                numpy.array((-GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                numpy.array((-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X)),
                numpy.array((-GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z)),
                numpy.array((+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y))]
    triindices = [[0, 1, 2],[1, 3, 4],[3, 5, 6],[2, 4, 7],[6, 5, 8],[2, 7, 9],[5, 0, 8],[9, 7, 10],[1, 0, 5],[10, 7, 11],[3, 1, 5],[6, 10, 11],[3, 6, 11],[9, 10, 8],[4, 3, 11],[6, 8, 10],[7, 4, 11],[2, 1, 4],[8, 0, 9],[0, 2, 9]]
    while level > 0:
        level -= 1
        newindices = []
        mapnewinds = dict()
        for tri in triindices:
            # for ever tri, create 3 new vertices and 4 new triangles.
            v = [vertices[i] for i in tri]
            inds = []
            for j in range(3):
                key = (tri[j],tri[numpy.mod(j+1,3)])
                if key in mapnewinds:
                    inds.append(mapnewinds[key])
                else:
                    mapnewinds[key] = mapnewinds[key[::-1]] = len(vertices)
                    inds.append(len(vertices))
                    vnew = v[j]+v[numpy.mod(j+1,3)]
                    vertices.append(vnew/numpy.sqrt(sum(vnew**2)))
            newindices += [[tri[0],inds[0],inds[2]],[inds[0],tri[1],inds[1]],[inds[2],inds[0],inds[1]],[inds[2],inds[1],tri[2]]]
        triindices = newindices
    return radius*numpy.array(vertices),triindices

def DrawAxes(env,target,dist=1.0,linewidth=1,coloradd=None):
    """draws xyz coordinate system around target.

    :param env: Environment
    :param target: can be a 7 element pose, 4x4 matrix, or the name of a kinbody in the environment
    :param dist: how far the lines extend from the origin
    :param linewidth: how thick the line is rendered in pixels
    :param coloradd: an optional 3-element vector for 
    """
    if isinstance(target,basestring):
        T = self.env.GetKinBody(target).GetTransform()
    elif len(target) == 7:
        T = openravepy_int.matrixFromPose(target)
    else:
        T = numpy.array(target)
    colors=numpy.array([[1,0,0],[1,0,0],[0,1,0],[0,1,0],[0,0,1],[0,0,1]])
    if coloradd is not None:
        colors = numpy.minimum(1.0, numpy.maximum(0.0, colors + numpy.tile(coloradd,(len(colors),1))))
    return env.drawlinelist(numpy.array([T[0:3,3],T[0:3,3]+T[0:3,0]*dist,T[0:3,3],T[0:3,3]+T[0:3,1]*dist,T[0:3,3],T[0:3,3]+T[0:3,2]*dist]),linewidth,colors=colors)

def DrawIkparam(env,ikparam,dist=1.0,linewidth=1,coloradd=None):
    """draws an IkParameterization

    """
    if ikparam.GetType() == openravepy_int.IkParameterizationType.Transform6D:
        return DrawAxes(env,ikparam.GetTransform6DPose(),dist,linewidth,coloradd)
    
    elif ikparam.GetType() == openravepy_int.IkParameterizationType.TranslationDirection5D:
        ray = ikparam.GetTranslationDirection5D()
        colors=numpy.array([[0,0,0],[1,0,0]])
        if coloradd is not None:
            colors = numpy.minimum(1.0, numpy.maximum(0.0, colors + numpy.tile(coloradd,(len(colors),1))))
        return env.drawlinelist(numpy.array([ray.pos(),ray.pos()+ray.dir()*dist]),linewidth,colors=colors)
    
    elif ikparam.GetType() == openravepy_int.IkParameterizationType.Translation3D:
        if coloradd is not None:
            colors = numpy.array([coloradd])
        else:
            colors=numpy.array([[0,0,0]])
        return env.plot3(ikparam.GetTranslation3D(),linewidth,colors=colors)
    
    elif ikparam.GetType() == openravepy_int.IkParameterizationType.TranslationXAxisAngleZNorm4D:
        pos,angle = ikparam.GetTranslationXAxisAngleZNorm4D()
        T = openravepy_int.matrixFromAxisAngle([0,0,angle])
        T[0:3,3] = pos
        return DrawAxes(env,T,dist,linewidth,coloradd)
    
    else:
        raise NotImplemented('iktype %s'%str(ikparam.GetType()))

def DrawIkparam2(env,ikparam,dist=1.0,linewidth=1,coloradd=None):
    """draws an IkParameterization

    """
    if ikparam.GetType() == openravepy_int.IkParameterizationType.Transform6D:
        return [DrawAxes(env,ikparam.GetTransform6DPose(),dist,linewidth,coloradd)]
    
    elif ikparam.GetType() == openravepy_int.IkParameterizationType.TranslationDirection5D:
        ray = ikparam.GetTranslationDirection5D()
        colors=numpy.array([[0,0,0],[1,0,0]])
        if coloradd is not None:
            colors = numpy.minimum(1.0, numpy.maximum(0.0, colors + numpy.tile(coloradd,(len(colors),1))))

        # have to draw the arrow
        dirangle = numpy.pi/6
        arrowc = numpy.cos(dirangle)*dist*0.1
        arrows = numpy.sin(dirangle)*dist*0.1
        arrowdirs = numpy.array([[0, 0, 0], [0, 0, dist],
                                 [0, 0, dist], [arrows, 0, dist-arrowc],
                                 [0, 0, dist], [-arrows, 0, dist-arrowc],
                                 [0, 0, dist], [0, arrows, dist-arrowc],
                                 [0, 0, dist], [0, -arrows, dist-arrowc]])
        q = openravepy_int.quatRotateDirection([0,0,1], ray.dir())
        realarrowlines = openravepy_ext.quatRotateArrayT(q, arrowdirs) + numpy.tile(ray.pos(), (len(arrowdirs),1))
        return [env.drawlinelist(realarrowlines[:2,:].flatten(),linewidth,colors=colors), env.drawlinelist(realarrowlines[2:].flatten(),linewidth,colors=colors[1])] + [env.plot3(ray.pos(), 4*linewidth, colors=[0.5,0,0])]
    
    elif ikparam.GetType() == openravepy_int.IkParameterizationType.Translation3D:
        if coloradd is not None:
            colors = numpy.array([coloradd])
        else:
            colors=numpy.array([[0,0,0]])
        return [env.plot3(ikparam.GetTranslation3D(),linewidth,colors=colors)]
    
    elif ikparam.GetType() == openravepy_int.IkParameterizationType.TranslationXAxisAngleZNorm4D:
        pos,angle = ikparam.GetTranslationXAxisAngleZNorm4D()
        T = openravepy_int.matrixFromAxisAngle([0,0,angle])
        T[0:3,3] = pos
        return [DrawAxes(env,T,dist,linewidth,coloradd)]
    
    else:
        raise NotImplemented('iktype %s'%str(ikparam.GetType()))

def DrawCircle(env, center, normal, radius, linewidth=1, colors=None):
    angles = numpy.arange(0, 2*numpy.pi+0.1, 0.1)
    R = openravepy_int.matrixFromQuat(openravepy_int.quatRotateDirection([0,0,1],normal))
    right = R[0:3,0]*radius
    up = R[0:3,1]*radius
    return env.drawlinestrip(c_[numpy.dot(numpy.transpose([numpy.cos(angles)]), [right]) + numpy.dot(numpy.transpose([numpy.sin(angles)]), [up]) + numpy.tile(center, (len(angles),1))], linewidth, colors=colors)

def ComputeBoxMesh(extents):
    """Computes a box mesh"""
    indices = numpy.reshape([0, 1, 2, 1, 2, 3, 4, 5, 6, 5, 6, 7, 0, 1, 4, 1, 4, 5, 2, 3, 6, 3, 6, 7, 0, 2, 4, 2, 4, 6, 1, 3, 5,3, 5, 7],(12,3))
    vertices = numpy.array(((extents[0],extents[1],extents[2]),
                            (extents[0],extents[1],-extents[2]),
                            (extents[0],-extents[1],extents[2]),
                            (extents[0],-extents[1],-extents[2]),
                            (-extents[0],extents[1],extents[2]),
                            (-extents[0],extents[1],-extents[2]),
                            (-extents[0],-extents[1],extents[2]),
                            (-extents[0],-extents[1],-extents[2])))
    return vertices,indices

def ComputeCylinderYMesh(radius,height,angledelta=0.1):
    """Computes a mesh of a cylinder oriented towards y-axis"""
    angles = numpy.arange(0,2*numpy.pi,angledelta)
    cangles = numpy.cos(angles)
    sangles = numpy.sin(angles)
    N = len(angles)
    vertices = numpy.c_[radius*numpy.tile(cangles,2),numpy.r_[numpy.tile(height*0.5,N),numpy.tile(-height*0.5,N)], radius*numpy.tile(sangles,2)]
    indices = []
    iprev = N-1
    for i in range(N):
        indices.append((iprev,i,iprev+N))
        indices.append((i,i+N,iprev+N))
        iprev = i
    return vertices,numpy.array(indices)


def TSP(solutions,distfn):
    """solution to travelling salesman problem. orders the set of solutions such that visiting them one after another is fast.
    """
    newsolutions = numpy.array(solutions)
    for i in range(newsolutions.shape[0]-2):
        n = newsolutions.shape[0]-i-1
        dists = [distfn(newsolutions[i,:],newsolutions[j,:]) for j in range(i+1,newsolutions.shape[0])]
        minind = numpy.argmin(dists)+i+1
        sol = numpy.array(newsolutions[i+1,:])
        newsolutions[i+1,:] = newsolutions[minind,:]
        newsolutions[minind,:] = sol
    return newsolutions

def sequence_cross_product(*sequences):
    """iterates through the cross product of all items in the sequences"""
    # visualize an odometer, with "wheels" displaying "digits"...:
    wheels = map(iter, sequences)
    digits = [it.next( ) for it in wheels]
    while True:
        yield tuple(digits)
        for i in range(len(digits)-1, -1, -1):
            try:
                digits[i] = wheels[i].next( )
                break
            except StopIteration:
                wheels[i] = iter(sequences[i])
                digits[i] = wheels[i].next( )
        else:
            break

class MultiManipIKSolver:
    """Finds the simultaneous IK solutions of all disjoint manipulators (no manipulators share a joint).

    The class is extremely useful in dual-manipulation IK solutions. It also handled grabbed bodies correctly.
    """
    def __init__(self,manips):
        self.robot = manips[0].GetRobot()
        self.manips = manips
        indeplinksets=[set([l for l in manip.GetIndependentLinks()]) for manip in self.manips]
        indeplinknames=indeplinksets[0].intersection(*indeplinksets[1:])
        alllinknames = set([l for l in self.robot.GetLinks()])
        self.enablelinknames = [alllinknames.difference(indeplinksets[i]).union(indeplinknames) for i in range(len(self.manips))]
    
    def findMultiIKSolution(self,Tgrasps,filteroptions=openravepy_int.IkFilterOptions.CheckEnvCollisions,dooptimize=False):
        """Return one set collision-free ik solutions for all manipulators.

        Method always checks self-collisions.
        
        :param Tgrasps: a list of all the end effector transforms of each of the manipualtors
        :param filteroptions: a bitmask of :class:`IkFilterOptions`
        """
        assert(len(Tgrasps)==len(self.manips))
        with self.robot:
            alljointvalues = []
            grabbed = self.robot.GetGrabbed()
            statesavers = [openravepy_int.KinBody.KinBodyStateSaver(body) for body in grabbed]
            try:
                with openravepy_ext.RobotStateSaver(self.robot): # for storing enabled state
                    for i,manip in enumerate(self.manips):
                        # invalidate all links that are controlled by the other manipulators
                        for link in self.robot.GetLinks():
                            link.Enable(link in self.enablelinknames[i])
                        # enable only the grabbed bodies of this manipulator
                        for body in grabbed:
                            body.Enable(manip.IsGrabbing(body))
                        values=manip.FindIKSolutions(Tgrasps[i],filteroptions)
                        if values is not None and len(values) > 0:
                            alljointvalues.append(values)
                        else:
                            return None
                        
            finally:
                for saver in statesavers:
                    saver.Restore()

            if dooptimize:
                curvalues = [self.robot.GetDOFValues(manip.GetArmIndices()) for main in self.manips]
                distancesolutions = []
                for sols in sequence_cross_product(*alljointvalues):
                    dist = numpy.sum([numpy.sum(numpy.abs(sol0-sol1)) for sol0,sol1 in izip(sols,curvalues)])
                    distancesolutions.append([dist, sols])
                distancesolutions.sort(lambda x,y: int(x[0]-y[0]))
                for dist,sols in distancesolutions:
                    for sol,manip in izip(sols,self.manips):
                        self.robot.SetDOFValues(sol,manip.GetArmIndices()) 
                    if not self.robot.CheckSelfCollision():
                        if not (filteroptions&openravepy_int.IkFilterOptions.CheckEnvCollisions) or not self.robot.GetEnv().CheckCollision(self.robot):
                            return sols
            else:
                for sols in sequence_cross_product(*alljointvalues):
                    for sol,manip in izip(sols,self.manips):
                        self.robot.SetDOFValues(sol,manip.GetArmIndices()) 
                    if not self.robot.CheckSelfCollision():
                        if not (filteroptions&openravepy_int.IkFilterOptions.CheckEnvCollisions) or not self.robot.GetEnv().CheckCollision(self.robot):
                            return sols
            
            return None

class SpaceSamplerExtra:
    def __init__(self):
         self.faceindices = self.facenumr = self.facenump = None
    @staticmethod
    def computeSepration(qarray):
        """used to test separation of a set of quaternions"""
        qdists = numpy.zeros((qarray.shape[0],qarray.shape[0]))
        for i,q in enumerate(qarray):
            dists = numpy.abs(numpy.dot(qarray[(i+1):],q))
            qdists[i,(i+1):] = qdists[(i+1):,i] = dists
        qmaxdists = numpy.max(qdists,axis=0)
        return numpy.arccos(numpy.min(1.0,numpy.max(qmaxdists))), numpy.arccos(numpy.min(1.0,numpy.min(qmaxdists)))
    def computeFaceIndices(self,N):
        if self.faceindices is None or len(self.faceindices[0]) < N:
            indices = numpy.arange(N**2)
            # separate the odd and even bits into odd,even
            maxiter = int(numpy.log2(len(indices)))
            oddbits = numpy.zeros(N**2,int)
            evenbits = numpy.zeros(N**2,int)
            mult = 1
            for i in range(maxiter):
                oddbits += (indices&1)*mult
                evenbits += mult*((indices&2)/2)
                indices >>= 2
                mult *= 2
            self.faceindices = [oddbits+evenbits,oddbits-evenbits]
        if self.facenumr is None or len(self.facenumr) != N*12:
            self.facenumr = numpy.reshape(numpy.transpose(numpy.tile([2,2,2,2,3,3,3,3,4,4,4,4],(N,1))),N*12)
            self.facenump = numpy.reshape(numpy.transpose(numpy.tile([1,3,5,7,0,2,4,6,1,3,5,7],(N,1))),N*12)
    def sampleS2(self,level=0,angledelta=None):
        """uses healpix algorithm with ordering from Yershova et. al. 2009 journal paper"""
        if angledelta is not None:
            # select the best sphere level matching angledelta;
            # level,delta: 
            # [0, 1.0156751592381095]
            # [1, 0.5198842203445676]
            # [2, 0.25874144949351713]
            # [3, 0.13104214473149575]
            # [4, 0.085649339187184162]
            level=max(0,int(0.5-numpy.log2(angledelta)))
        Nside = 2**level
        Nside2 = Nside**2
        N = 12*Nside**2
        self.computeFaceIndices(Nside**2)        
        # compute sphere z coordinate
        jr = self.facenumr*Nside-numpy.tile(self.faceindices[0][0:Nside2],12)-1
        nr = numpy.tile(Nside,N)
        z = 2*(2*Nside-jr)/(3.0*Nside)
        kshift = numpy.mod(jr-Nside,2)
        # north pole test
        northpoleinds = numpy.flatnonzero(jr<Nside)
        nr[northpoleinds] = jr[northpoleinds]
        z[northpoleinds] = 1.0 - nr[northpoleinds]**2*(1.0/(3.0*Nside2))
        kshift[northpoleinds] = 0
        # south pole test
        southpoleinds = numpy.flatnonzero(jr>3*Nside)
        nr[southpoleinds] = 4*Nside - jr[southpoleinds]
        z[southpoleinds] = -1.0 + nr[southpoleinds]**2*(1.0/(3.0*Nside2))
        kshift[southpoleinds] = 0
        # compute pfi
        facenump = numpy.reshape(numpy.transpose(numpy.tile([1,3,5,7,0,2,4,6,1,3,5,7],(Nside2,1))),N)
        jp = (self.facenump*nr+numpy.tile(self.faceindices[1][0:Nside2],12)+1+kshift)/2
        jp[jp>4*Nside] -= 4*Nside
        jp[jp<1] += 4*Nside
        return numpy.arccos(z),(jp-(kshift+1)*0.5)*((0.5*numpy.pi)/nr)
    @staticmethod
    def hopf2quat(hopfarray):
        """convert hopf rotation coordinates to quaternion"""
        half0 = hopfarray[:,0]*0.5
        half2 = hopfarray[:,2]*0.5
        c0 = numpy.cos(half0)
        c2 = numpy.cos(half2)
        s0 = numpy.sin(half0)
        s2 = numpy.sin(half2)
        return numpy.c_[c0*c2,c0*s2,s0*numpy.cos(hopfarray[:,1]+half2),s0*numpy.sin(hopfarray[:,1]+half2)]
    def sampleSO3(self,level=0,quatdelta=None):
        """Uniformly Sample 3D Rotations.
        If quatdelta is specified, will compute the best level aiming for that average quaternion distance.
        Algorithm From
        A. Yershova, S. Jain, S. LaValle, J. Mitchell "Generating Uniform Incremental Grids on SO(3) Using the Hopf Fibration", International Journal of Robotics Research, Nov 13, 2009.
        """
        if quatdelta is not None:
            # level=0, quatdist = 0.5160220
            # level=1: quatdist = 0.2523583
            # level=2: quatdist = 0.120735
            level=max(0,int(-0.5-numpy.log2(quatdelta)))
        s1samples,step = numpy.linspace(0.0,2*numpy.pi,6*(2**level),endpoint=False,retstep=True)
        s1samples += step*0.5
        theta,pfi = self.sampleS2(level)
        band = numpy.zeros((len(s1samples),3))
        band[:,2] = s1samples
        qarray = numpy.zeros((0,4))
        for i in range(len(theta)):
            band[:,0] = theta[i]
            band[:,1] = pfi[i]
            qarray = numpy.r_[qarray,self.hopf2quat(band)]
        return qarray
    @staticmethod
    def sampleR3lattice(averagedist,boxdims):
        """low-discrepancy lattice sampling in using the roots of x^3-3x+1.
        The samples are evenly distributed with an average distance of averagedist inside the box with extents boxextents.
        Algorithim from "Geometric Discrepancy: An Illustrated Guide" by Jiri Matousek"""
        roots = numpy.array([2.8793852415718155,0.65270364466613917,-0.53208888623795614])
        bases = numpy.c_[numpy.ones(3),roots,roots**2]
        tbases = numpy.transpose(bases)
        boxextents = 0.5*numpy.array(boxdims)
        # determine the input bounds, which can be very large and inefficient...
        bounds = numpy.array(((boxextents[0],boxextents[1],boxextents[2]),
                              (boxextents[0],boxextents[1],-boxextents[2]),
                              (boxextents[0],-boxextents[1],boxextents[2]),
                              (boxextents[0],-boxextents[1],-boxextents[2]),
                              (-boxextents[0],boxextents[1],boxextents[2]),
                              (-boxextents[0],boxextents[1],-boxextents[2]),
                              (-boxextents[0],-boxextents[1],boxextents[2]),
                              (-boxextents[0],-boxextents[1],-boxextents[2])))
        inputbounds = numpy.max(numpy.dot(bounds,numpy.linalg.inv(tbases)),0)
        scale = averagedist/numpy.sqrt(3.0)
        X,Y,Z = numpy.mgrid[-inputbounds[0]:inputbounds[0]:scale,-inputbounds[1]:inputbounds[1]:scale,-inputbounds[2]:inputbounds[2]:scale]
        p = numpy.c_[X.flat,Y.flat,Z.flat]
        pts = numpy.dot(p,tbases)
        ptsabs = numpy.abs(pts)
        newpts = pts[numpy.logical_and(ptsabs[:,0]<=boxextents[0],numpy.logical_and(ptsabs[:,1]<=boxextents[1] ,ptsabs[:,2]<=boxextents[2]))]
        newpts[:,0] += boxextents[0]
        newpts[:,1] += boxextents[1]
        newpts[:,2] += boxextents[2]
        return newpts
    @staticmethod
    def sampleR3(averagedist,boxdims):
        """low-discrepancy sampling using primes.
        The samples are evenly distributed with an average distance of averagedist inside the box with dimensions boxdims.
        Algorithim from "Geometric Discrepancy: An Illustrated Guide" by Jiri Matousek"""
        minaxis = numpy.argmin(boxdims)
        maxaxis = numpy.argmax(boxdims)
        meddimdist = numpy.sort(boxdims)[1]
        # convert average distance to number of samples.... do simple 3rd degree polynomial fitting...
        x = meddimdist/averagedist
        if x < 25.6:
            N = int(numpy.polyval([ -3.50181522e-01,   2.70202333e+01,  -3.10449514e+02, 1.07887093e+03],x))
        elif x < 36.8:
            N = int(numpy.polyval([  4.39770585e-03,   1.10961031e+01,  -1.40066591e+02, 1.24563464e+03],x))
        else:
            N = int(numpy.polyval([5.60147111e-01,  -8.77459988e+01,   7.34286834e+03, -1.67779452e+05],x))
        pts = numpy.zeros((N,3))
        pts[:,0] = numpy.linspace(0.0,meddimdist,N)
        pts[:,1] = meddimdist*numpy.mod(0.5+0.5*numpy.sqrt(numpy.arange(0,5.0*N,5.0)),1.0)
        pts[:,2] = meddimdist*numpy.mod(0.5+3*numpy.sqrt(numpy.arange(0,13.0*N,13.0)),1.0)
        if boxdims[minaxis] < meddimdist:
            pts = pts[pts[:,minaxis]<=boxdims[minaxis],:]
        if boxdims[maxaxis] > meddimdist:
            # have to copy across the max dimension
            numfullcopies = numpy.floor(boxdims[maxaxis]/meddimdist)
            if len(pts) > 0:
                oldpts = pts
                pts = numpy.array(oldpts)
                for i in range(int(numfullcopies)-1):
                    oldpts[:,maxaxis] += meddimdist
                    pts = numpy.r_[pts,oldpts]
                if boxdims[maxaxis]/meddimdist > numfullcopies:
                    oldpts[:,maxaxis] += meddimdist
                    pts = numpy.r_[pts,oldpts[oldpts[:,maxaxis]<=boxdims[maxaxis],:]]
            else:
                # sample the center
                pts = numpy.array([[0.0,0.0,0.0]])
        return pts

def CompareBodies(body0,body1,comparegeometries=True,comparesensors=True,comparemanipulators=True,comparegrabbed=True,comparephysics=True,computeadjacent=True,epsilon=1e-10):
    """Compares that two bodies are structurally and positionally equivalent without hashes, used for debug checking.
    """
    def transdist(list0,list1):
        assert(len(list0)==len(list1))
        return numpy.sum([numpy.sum(abs(item0-item1)) for item0, item1 in izip(list0,list1)])
    
    assert(body0.IsRobot() == body1.IsRobot())
    assert(len(body0.GetJoints())==len(body1.GetJoints()))
    assert(len(body0.GetPassiveJoints()) == len(body1.GetPassiveJoints()))
    assert(body0.GetDOF()==body1.GetDOF())
    assert(body1.GetDescription()==body0.GetDescription())
    assert(transdist(body0.GetTransform(), body1.GetTransform()) <= epsilon)
    with body1:
        body1.SetTransform(body0.GetTransform()) # in case
        body1.SetDOFValues(body0.GetDOFValues()) # in case
        joints0 = body0.GetJoints()+body0.GetPassiveJoints()
        joints1 = body1.GetJoints()+body1.GetPassiveJoints()
        for j0 in joints0:
            assert( len(j0.GetName()) > 0 )
            if j0.GetJointIndex() >= 0:
                # if not passive, indices should match
                j1 = joints1[j0.GetJointIndex()]
                assert(j1.GetJointIndex()==j0.GetJointIndex() and j1.GetDOFIndex() == j0.GetDOFIndex())
            else:
                j1s = [j1 for j1 in joints1 if j0.GetName() == j1.GetName()]
                assert( len(j1s) == 1 )
                j1 = j1s[0]
            assert( transdist(j0.GetAnchor(),j1.GetAnchor()) <= epsilon )
            assert( j0.GetDOF() == j1.GetDOF() and j0.GetType() == j1.GetType() )
            # todo, once physics is complete, uncomment
            #assert( j0.GetHierarchyParentLink().GetName() == j1.GetHierarchyParentLink().GetName() )
            #assert( j0.GetHierarchyChildLink().GetName() == j1.GetHierarchyChildLink().GetName() )
            # cannot compare individual j0.GetInternalHierarchyXTransform() since representation is ambiguous
            # compare product instead
            assert( transdist(numpy.dot(j0.GetInternalHierarchyLeftTransform(),j0.GetInternalHierarchyRightTransform()), numpy.dot(j1.GetInternalHierarchyLeftTransform(), j1.GetInternalHierarchyRightTransform()) <= epsilon ))
            assert( j0.IsStatic() == j1.IsStatic() )
            assert( transdist(j0.GetLimits(),j1.GetLimits()) <= epsilon )
            assert( transdist(j0.GetWeights(),j1.GetWeights()) <= epsilon )
            assert( transdist(j0.GetResolutions(),j1.GetResolutions()) <= epsilon )
            for idof in range(j0.GetDOF()):
                if not j0.IsStatic():
                    assert( abs(j0.GetMaxVel(idof)-j1.GetMaxVel(idof)) <= epsilon )
                    assert( abs(j0.GetMaxAccel(idof)-j1.GetMaxAccel(idof)) <= epsilon )
                    assert( abs(j0.GetWeight(idof)-j1.GetWeight(idof)) <= epsilon )
                    assert( abs(j0.GetResolution(idof)-j1.GetResolution(idof)) <= epsilon )
                assert( j0.IsCircular(idof) == j1.IsCircular(idof) )
                assert( j0.IsRevolute(idof) == j1.IsRevolute(idof) )
                assert( j0.IsPrismatic(idof) == j1.IsPrismatic(idof) )
                assert( transdist(j0.GetInternalHierarchyAxis(idof),j1.GetInternalHierarchyAxis(idof)) <= epsilon )
                assert( j0.IsMimic(idof) == j1.IsMimic(idof) )
                if j0.IsMimic(idof):
                    mimicjoints0 = [body0.GetJointFromDOFIndex(index).GetName() for index in j0.GetMimicDOFIndices(idof)]
                    mimicjoints1 = [body1.GetJointFromDOFIndex(index).GetName() for index in j1.GetMimicDOFIndices(idof)]
                    assert( mimicjoints0 == mimicjoints1 )
                    # is it possible to compare equations? perhaps just set random values and see if both robots behave the same?
                    # assert( j0.GetMimicEquation(idof) == j1.GetMimicEquation(idof) )
        assert(len(body0.GetLinks())==len(body1.GetLinks()))
        indexmap = []
        for link0 in body0.GetLinks():
            if len(link0.GetName()) == 0:
                # skip
                continue
            link1s = [link1 for link1 in body1.GetLinks() if link0.GetName() == link1.GetName()]
            assert( len(link1s) == 1 )
            link1 = link1s[0]
            indexmap.append(link1.GetIndex())

        for link0 in body0.GetLinks():
            if len(link0.GetName()) == 0:
                # skip
                continue
            link1s = [link1 for link1 in body1.GetLinks() if link0.GetName() == link1.GetName()]
            assert( len(link1s) == 1 )
            link1 = link1s[0]
            indexmap.append(link1.GetIndex())
            assert( transdist(link0.GetTransform(),link1.GetTransform()) <= epsilon )
            assert( link0.IsEnabled() == link1.IsEnabled() )
            #assert( link0.IsStatic() == link1.IsStatic() )
            assert( len(link0.GetParentLinks()) == len(link1.GetParentLinks()) )
            assert( all([lp0.GetName()==lp1.GetName() for lp0, lp1 in izip(link0.GetParentLinks(),link1.GetParentLinks())]) )
            if comparephysics:
                assert(abs(link0.GetMass()-link1.GetMass()) <= epsilon)
                assert(transdist(link0.GetLocalMassFrame(),link1.GetLocalMassFrame()) <= epsilon)
                assert(transdist(link0.GetGlobalCOM(),link1.GetGlobalCOM()) <= epsilon) # redundant
                assert(transdist(link0.GetPrincipalMomentsOfInertia(),link1.GetPrincipalMomentsOfInertia()) <= epsilon)
            if comparegeometries:
                assert( len(link0.GetGeometries()) == len(link1.GetGeometries()) )
                ab0=link0.ComputeAABB()
                ab1=link1.ComputeAABB()
                assert(transdist(ab0.pos(),ab1.pos()) <= epsilon*200) # tesselation
                assert(transdist(ab0.extents(),ab1.extents()) <= epsilon*200) # tesselation
                for ig,g0 in enumerate(link0.GetGeometries()):
                    g1=link1.GetGeometries()[ig]
                    assert(g0.GetType()==g1.GetType())
                    assert(transdist(g0.GetTransform(),g1.GetTransform()) <= epsilon)
                    assert(transdist(g0.GetBoxExtents(),g1.GetBoxExtents()) <= epsilon)
                    assert(transdist(g0.GetDiffuseColor(),g1.GetDiffuseColor()) <= epsilon)
                    assert(transdist(g0.GetAmbientColor(),g1.GetAmbientColor()) <= epsilon)
                    assert(g0.IsVisible()==g1.IsVisible())
                if computeadjacent:
                    # the geometry and initial configuration determine adjancent links
                    adjacentlinks = set([tuple(sorted((indexmap[index0],indexmap[index1]))) for index0,index1 in body0.GetAdjacentLinks()])
                    assert(adjacentlinks == set(body1.GetAdjacentLinks()))
        if body0.IsRobot():
            robot0 = body0.GetEnv().GetRobot(body0.GetName())
            robot1 = body1.GetEnv().GetRobot(body1.GetName())
            if comparemanipulators:
                assert(len(robot0.GetManipulators()) == len(robot1.GetManipulators()))
                for manip0 in robot0.GetManipulators():
                    manip1 = robot1.GetManipulator(manip0.GetName())
                    assert(transdist(manip0.GetLocalToolTransform(),manip1.GetLocalToolTransform()) <= epsilon)
                    assert(manip0.GetBase().GetName() == manip1.GetBase().GetName())
                    assert(manip0.GetEndEffector().GetName() == manip1.GetEndEffector().GetName())
                    assert(all(manip0.GetArmIndices() == manip1.GetArmIndices()))
                    assert(all(manip0.GetGripperIndices() == manip1.GetGripperIndices()))
            if comparegrabbed:
                grabbed0 = robot0.GetGrabbed()
                grabbed1 = robot1.GetGrabbed()
                assert( set([body.GetName() for body in grabbed0]) == set([body.GetName() for body in grabbed1]) )
                for g0 in grabbed0:
                    g1 = robot1.GetEnv().GetKinBody(g0.GetName())
                    grabbedlink0 = robot0.IsGrabbing(g0)
                    grabbedlink1 = robot1.IsGrabbing(g1)
                    assert(grabbedlink0.GetName()==grabbedlink1.GetName())
                # compare the positions

            if comparesensors:
                pass
                #assert(len(robot0.GetAttachedSensors()) == len(robot1.GetAttachedSensors()))

def CompareEnvironments(env,env2,options=openravepy_int.CloningOptions.Bodies,epsilon=1e-10):
    """compares two state of two environments and raises exceptions if anything is different, used for debugging.

    Structural information of bodies is compared with hashes.
    """
    if options & openravepy_int.CloningOptions.Bodies:
        def transdist(list0,list1):
            assert(len(list0)==len(list1))
            return numpy.sum([numpy.sum(abs(item0-item1)) for item0, item1 in izip(list0,list1)])
        
        bodies=env.GetBodies()
        bodies2=env2.GetBodies()
        assert(len(bodies)==len(bodies2))
        for body in bodies:
            body2 = env2.GetKinBody(body.GetName())
            assert(body.GetKinematicsGeometryHash()==body2.GetKinematicsGeometryHash())
            assert(transdist(body.GetLinkTransformations(),body2.GetLinkTransformations()) <= epsilon)
            assert(transdist(body.GetLinkVelocities(),body2.GetLinkVelocities()) <= epsilon)
            if body.GetDOF() > 0:
                assert(transdist(body.GetDOFValues(),body2.GetDOFValues()) <= epsilon)
                assert(transdist(body.GetDOFVelocities(),body2.GetDOFVelocities()) <= epsilon)
            if body.IsRobot():
                robot=env.GetRobot(body.GetName())
                robot2=env2.GetRobot(body2.GetName())
                grabbed = robot.GetGrabbed()
                grabbed2 = robot2.GetGrabbed()
                assert( set([body.GetName() for body in grabbed]) == set([body.GetName() for body in grabbed2]) )
                assert( transdist(robot.GetActiveDOFIndices(),robot2.GetActiveDOFIndices()) == 0)
                assert( robot.GetActiveManipulator().GetName() == robot2.GetActiveManipulator().GetName())
