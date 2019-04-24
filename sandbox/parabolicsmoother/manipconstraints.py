from numpy import arange ,array, cross, linalg
from openravepy import AABB
import os
import sys
sys.path.append(os.path.join(os.path.expanduser('~'), 'mujin/checkoutroot/openrave/sandbox/parabolicsmoother'))
import ramp, interpolation

class ManipConstraintInfo2(object):

    def __init__(self):
        self.fmaxdistfromcenter = 0
        self.pmanip = None
        self.plink = None
        self.checkpoints = None
        self.vuseddofindices = None
        self.vocnfigindices = None
        

class ManipConstraintChecker2(object):
    """Similar to the one in plugins/rplanners/manipconstraints2.h

    """
    def __init__(self, env):
        self.env = env
        self._maxmanipspeed = 0.0
        self._maxmanipaccel = 0.0


    def Init(self, manipname, spec, maxmanipspeed, maxmanipaccel):
        self._manipname = manipname
        self._maxmanipspeed = maxmanipspeed
        self._maxmanipaccel = maxmanipaccel
        self._listCheckManips = []

        listUsedBodies = spec.ExtractUsedBodies(self.env)
        for body in listUsedBodies:
            if not body.IsRobot():
                continue

            manip = robot.GetManipulator(manipname)
            if manip is None:
                continue

            endeffector = manip.GetEndEffector()

            # When the manipulator is grabbing items, globallinklist will contain all the grabbed
            # links. Otherwise, globallinklist will contain the manip's children links. These links
            # will be used for computing AABB to obtain checkpoints.
            globallinklist = []
            grabbedbodies = robot.GetGrabbed()
            if len(grabbedbodies) > 0:
                for gbody in grabbedbodies:
                    if robot.IsGrabbing(gbody) is not None:
                        globallinklist += gbody.GetLinks()
            else:
                globallinklist = manip.GetChildLinks()

            # Compute the AABB enclosing all links in globallinklist. Then we will use those eight
            # vertices of the AABB for manip constraint checking.
            enclosingaabb = self.ComputeEnclosingAABB(globallinklist, endeffector.GetTransform())
            manipinfo = ManipConstraintInfo2()
            manipinfo.vuseddofindices, manipinfo.vconfigindices = spec.ExtractUsedIndices(manip.GetRobot())
            manipinfo.pmanip = manip
            manipinfo.plink = endeffector
            manipinfo.checkpoints = self.ConvertAABBtoCheckPoints(enclosingaabb)
            for point in manipinfo.checkpoints:
                f = point.dot(point)
                if manipinfo.fmaxdistfromcenter < f:
                    manipinfo.fmaxdistfromcenter = f
            manipinfo.fmaxdistfromcenter = sqrt(manipinfo.fmaxdistfromcenter)
            self._listCheckManips.append(manipinfo)

            
    @staticmethod
    def ConvertAABBtoCheckPoints(aabb):
        signextents = array([ 1,  1,  1,
                              1,  1, -1,
                              1, -1,  1,
                              1, -1, -1,
                             -1,  1,  1,
                             -1,  1, -1,
                             -1, -1,  1,
                             -1, -1, -1])
        checkpoints = []
        extents = aabb.extents()
        pos = aabb.pos()
        for i in xrange(8):
            incr = hstack([extents[0] * signextents[3*i + 0],
                           extents[1] * signextents[3*i + 1],
                           extents[2] * signextents[3*i + 2],])
            checkpoints.append(pos + incr)
        return checkpoints


    def ComputeEnclosingAABB(self, linklist, Tparent):
        bInitialized = False
        Tparentinv = linalg.inv(Tparent)
        for link in linklist:
            ablink = link.ComputeLocalAABB()
            Tdelta = Tparentinv.dot(link.GetTransform())
            Rdelta = array(Tdelta[0:3, 0:3])
            vabsextents = Rdelta.dot(ablink.extents())
            vcenter = Tdelta[0:3, 3] + Rdelta.dot(ablink.pos())
            vnmin = vcenter - vabsextents
            vnmax = vcenter + vabsextents
            if not bInitialized:
                vmin = vnmin
                vmax = vnmax
                bInitialized = True
            else:
                vmin[0] = min(vmin[0], vnmin[0])
                vmin[1] = min(vmin[1], vnmin[1])
                vmin[2] = min(vmin[2], vnmin[2])

                vmax[0] = max(vmax[0], vnmax[0])
                vmax[1] = max(vmax[1], vnmax[1])
                vmax[2] = max(vmax[2], vnmax[2])

        pos = 0.5*(vmin + vmax)
        extents = vmax - pos
        return AABB(pos, extents)

    
    def _ComputeManipMaxSpeedAccel(self, q, qd, qdd):
        speedaccels = [] # list of lists of the form (speed, accel) for manipulators
        for manipinfo in self._listCheckManips:
            robot = manipinfo.pmanip.GetRobot()
            with robot:
                endeffindex = manipinfo.plink.GetIndex()

                robot.SetDOFValues(q, manipinfo.vuseddofindices)
                robot.SetDOFVelocities(qd)
                linkvels = robot.GetLinkVelocities()
                linkaccels = robot.GetLinkAccelerations(qdd)
                endeffvellin = linkvels[endeffindex][:3]
                endeffvelang = linkvels[endeffindex][3:]
                endeffacclin = linkaccels[endeffindex][:3]
                endeffaccang = linkaccels[endeffindex][3:]
                
                Teff = manipinfo.plink.GetTransform()

            speedaccel = [0, 0]
            for point in manipinfo.checkpoints:
                newpoint = Teff[0:3, 0:3].dot(point)

                vpoint = endeffvellin + cross(endeffvelang, newpoint)
                actualmanipspeed = linalg.norm(vpoint)
                if actualmanipspeed > speedaccel[0]:
                    speedaccel[0] = actualmanipspeed

                apoint = endeffacclin + cross(endeffvelang, cross(endeffvelang, newpoint)) + cross(endeffaccang, newpoint)
                actualmanipaccel = linalg.norm(apoint)
                if actualmanipaccel > speedaccel[1]:
                    speedaccel[1] = actualmanipaccel

            speedaccels.append(speedaccel)
        return speedaccels
                    

    def ComputeManipSpeedAccelProfileOfPoint(self, traj, ipoint, timestep=0.001, imanip=0):
        """

        """
        curvesnd = ramp.ConvertOpenRAVETrajectoryToParabolicCurvesND(traj)
        manipinfo = self._listCheckManips[imanip]
        robot = manipinfo.pmanip.GetRobot()
        
        point = manipinfo.checkpoints[ipoint]
        T = arange(0, traj.GetDuration(), timestep)
        speeds, accels = [], []

        with robot:
            endeffindex = manipinfo.plink.GetIndex()
            for t in T:
                q = curvesnd.EvalPos(t)
                qd = curvesnd.EvalVel(t)
                qdd = curvesnd.EvalAcc(t)

                robot.SetDOFValues(q, manipinfo.vuseddofindices)
                robot.SetDOFVelocities(qd)
                linkvels = robot.GetLinkVelocities()
                linkaccels = robot.GetLinkAccelerations(qdd)
                endeffvellin = linkvels[endeffindex][:3]
                endeffvelang = linkvels[endeffindex][3:]
                endeffacclin = linkaccels[endeffindex][:3]
                endeffaccang = linkaccels[endeffindex][3:]
                Teff = manipinfo.plink.GetTransform()

                newpoint = Teff[0:3, 0:3].dot(point)

                vpoint = endeffvellin + cross(endeffvelang, newpoint)
                actualmanipspeed = linalg.norm(vpoint)

                apoint = endeffacclin + cross(endeffvelang, cross(endeffvelang, newpoint)) + cross(endeffaccang, newpoint)
                actualmanipaccel = linalg.norm(apoint)

                speeds.append(actualmanipspeed)
                accels.append(actualmanipaccel)

        return T, speeds, accels

