import time
import numpy as np
import ramp
import matplotlib.pyplot as plt
from pylab import ion
ion()
from os.path import isfile
import openravepy as orpy

import logging
logging.basicConfig(format='[%(levelname)s] [%(name)s: %(funcName)s] %(message)s', level=logging.DEBUG)
log = logging.getLogger(__name__)


def LoadParabolicPathString(pathnumber, prefix="/private/cache/openrave/"):
    parabolicpathfilename = prefix + "parabolicpath{0:d}.xml".format(pathnumber)
    if not isfile(parabolicpathfilename):
        log.debug("{0} does not exist".format(parabolicpathfilename))
        parbolicpathstring = ""
    else:
        with open(parabolicpathfilename, 'r') as f:
            parabolicpathstring = f.read()
    return parabolicpathstring


def LoadDynamicPathString(pathnumber, prefix="/private/cache/openrave/"):
    dynamicpathfilename = prefix + "dynamicpath{0:d}.xml".format(pathnumber)
    if not isfile(dynamicpathfilename):
        log.debug("{0} does not exist".format(dynamicpathfilename))
        parbolicpathstring = ""
    else:
        with open(dynamicpathfilename, 'r') as f:
            dynamicpathstring = f.read()
    return dynamicpathstring


def LoadShortcutInformation(number, prefix="/private/cache/openrave/"):
    filename_beforeshortcut = prefix + "dynamicpath{0}.beforeshortcut.xml".format(number)
    filename_aftershortcut = prefix + "dynamicpath{0}.aftershortcut.xml".format(number)
    filename_shortcutprogress = prefix + "shortcutprogress{0}.xml".format(number)

    with open(filename_beforeshortcut, 'r') as f:
        d1 = f.read()
    with open(filename_aftershortcut, 'r') as f:
        d2 = f.read()
    with open(filename_shortcutprogress, 'r') as f:
        s = f.read()
    return [d1, d2, s]


def ReadShortcutProgress(shortcutprogressstring):
    rawdata = shortcutprogressstring.strip().split("\n")
    generalinfo = rawdata[0].strip().split()
    originaldur = float(generalinfo[0])
    maxiter = int(generalinfo[1])
    nlinespergroup = 9
    nshortcuts = (len(rawdata) - 1)/nlinespergroup

    rawdata = rawdata[1:] # pop out the first element (original duration)

    successfuliters = []
    T0 = []
    T1 = []
    prevdurs = []
    newdurs = []
    
    X0 = []
    X1 = []
    V0 = []
    V1 = []
    XMIN = []
    XMAX = []
    VM = []
    AM = []
    
    for i in xrange(nshortcuts):
        offset = i*nlinespergroup
        generalinfo = rawdata[offset].strip().split()
        it = int(generalinfo[0])
        [t0, t1, prevdur, newdur] = [float(x) for x in generalinfo[1:]]

        x0 = np.array([float(x) for x in rawdata[offset + 1].strip().split()])
        x1 = np.array([float(x) for x in rawdata[offset + 2].strip().split()])
        v0 = np.array([float(x) for x in rawdata[offset + 3].strip().split()])
        v1 = np.array([float(x) for x in rawdata[offset + 4].strip().split()])
        xmin = np.array([float(x) for x in rawdata[offset + 5].strip().split()])
        xmax = np.array([float(x) for x in rawdata[offset + 6].strip().split()])
        vm = np.array([float(x) for x in rawdata[offset + 7].strip().split()])
        am = np.array([float(x) for x in rawdata[offset + 8].strip().split()])

        successfuliters.append(it)
        T0.append(t0)
        T1.append(t1)
        prevdurs.append(prevdur)
        newdurs.append(newdur)
        X0.append(x0)
        X1.append(x1)
        V0.append(v0)
        V1.append(v1)
        XMIN.append(xmin)
        XMAX.append(xmax)
        VM.append(vm)
        AM.append(am)

    return [originaldur, maxiter, successfuliters, T0, T1, prevdurs, newdurs, X0, X1, V0, V1, XMIN, XMAX, VM, AM]


def PlotData(index, prefix="/private/cache/openrave/", plot=True, fignum=1, ncolumns=2, nrows=2):
    assert(ncolumns*nrows >= 4) # we have at least 4 subfigures
    info = dict()
    [d1, d2, s] = LoadShortcutInformation(index, prefix)
    [originaldur, maxiter, successfuliters, T0, T1, prevdurs, newdurs, X0, X1, V0, V1, XMIN, XMAX, VM, AM] = ReadShortcutProgress(s)
    info['d1'] = d1 # dynamicpath before shortcut
    info['d2'] = d2 # dynamicpath after shortcut
    info['s'] = s # shortcut progress
    c1 = ramp.DynamicPathStringToParabolicCurvesND(d1)
    c2 = ramp.DynamicPathStringToParabolicCurvesND(d2)
    info['c1'] = c1 # paraboliccurvesnd converted from d1 (before shortcut)
    info['c2'] = c2 # paraboliccurvesnd converted from d2 (after shortcut)
    
    info['originaldur'] = originaldur
    info['maxiter'] = maxiter
    info['successfuliters'] = successfuliters
    if len(info['successfuliters']) == 0:
        return info
    
    info['T0'] = T0
    info['T1'] = T1
    info['prevdurs'] = prevdurs
    info['newdurs'] = newdurs
    info['X0'] = X0
    info['X1'] = X1
    info['V0'] = V0
    info['V1'] = V1
    info['XMIN'] = XMIN
    info['XMAX'] = XMAX
    info['VM'] = VM
    info['AM'] = AM    

    info['successfuliters'].append(maxiter)
    info['prevdurs'].append(newdurs[-1])
    info['newdurs'].append(newdurs[-1])

    diff = [x - y for (x, y) in zip(prevdurs, newdurs)] # progress made in each successful iteration
    info['diff'] = diff
    
    timefromprevshortcut = []
    timefromprevshortcut.append(successfuliters[0])
    for i in xrange(len(successfuliters[1:])):
        timefromprevshortcut.append(successfuliters[i + 1] - successfuliters[i])
    info['timefromprevshortcut'] = timefromprevshortcut

    # score1 = normalized(progress made / waiting time)
    # The score ranges from 0 to 1
    scores1 = np.array([a/float(b) for (a, b) in zip(diff, timefromprevshortcut)])
    scores1 = scores1/max(scores1)
    info['scores1'] = scores1

    # scores2 = progress made / waiting time
    # The score of the successful iteration i will be relative to the best shortcut happened so far.
    scores2 = []
    scores2.append(1.0)
    for i in xrange(len(scores1) - 1):
        scores2.append(scores1[i + 1]/max(scores1[0:i + 1]))
    info['scores2'] = scores2

    # Plot data
    axes = []
    if plot:        
        fig = plt.figure(fignum)
        fig.clf()

        isubfig = 1

        axes.append(fig.add_subplot(ncolumns, nrows, isubfig))
        plt.plot(successfuliters, newdurs, 'go-', label='resulting duration')
        plt.plot(successfuliters, diff, 'r^-', label='duration difference')
        plt.title('trajectory duration')
        isubfig += 1

        setlog = True

        axes.append(fig.add_subplot(ncolumns, nrows, isubfig))
        plt.plot(successfuliters, scores1, 'bs-')
        plt.title('Score 1')
        if setlog:
            axes[-1].set_yscale('log')
        isubfig += 1

        axes.append(fig.add_subplot(ncolumns, nrows, isubfig))
        plt.plot(successfuliters, scores2, 'ko-')
        plt.title('Score 2')
        if setlog:
            axes[-1].set_yscale('log')
        isubfig += 1

        axes.append(fig.add_subplot(ncolumns, nrows, isubfig))
        plt.plot(successfuliters, timefromprevshortcut, 'm*-')
        plt.title('Time taken since the previous shortcut happened')
        isubfig += 1
        
    info['axes'] = axes
    return info


def ConvertOpenRAVETrajToParabolicCurvesND(traj):
    nwaypoints = traj.GetNumWaypoints()
    spec = traj.GetConfigurationSpecification()
    xgroup = spec.GetGroupFromName('joint_values')
    xoffset = xgroup.offset
    xdof = xgroup.dof
    vgroup = spec.GetGroupFromName('joint_velocities')
    voffset = vgroup.offset
    vdof = vgroup.dof
    deltatimegroup = spec.GetGroupFromName('deltatime')
    toffset = deltatimegroup.offset

    curvesnd = ramp.ParabolicCurvesND()
    for iwaypoint in xrange(nwaypoints - 1):
        x0 = traj.GetWaypoint(iwaypoint)[xoffset: xoffset + xdof]
        x1 = traj.GetWaypoint(iwaypoint + 1)[xoffset: xoffset + xdof]
        v0 = traj.GetWaypoint(iwaypoint)[voffset: voffset + vdof]
        v1 = traj.GetWaypoint(iwaypoint + 1)[voffset: voffset + vdof]
        t = traj.GetWaypoint(iwaypoint + 1)[toffset]
        temp = ramp.ParabolicCurvesND()
        temp.SetSegment(x0, x1, v0, v1, t)
        curvesnd.Append(temp)

    return curvesnd


def ExtractDOFValuesFromWaypoint(traj, iwaypoint):
    spec = traj.GetConfigurationSpecification()
    xgroup = spec.GetGroupFromName('joint_values')
    xoffset = xgroup.offset
    xdof = xgroup.dof
    waypoint = traj.GetWaypoint(iwaypoint)
    x = waypoint[xoffset: xoffset + xdof]
    return x


def ExtractDOFValues(traj, t):
    spec = traj.GetConfigurationSpecification()
    xgroup = spec.GetGroupFromName('joint_values')
    xoffset = xgroup.offset
    xdof = xgroup.dof
    values = traj.Sample(t)
    x = values[xoffset: xoffset + xdof]
    return x
