from openravepy import openravepy_piecewisepolynomials as piecewisepolynomials
import matplotlib.pyplot as plt
import numpy as np
from pylab import ion
ion()
import logging
log = logging.getLogger(__name__)


def PlotPolynomial(polynomial, tstart=None, tend=None, stepsize=0.01, derivative=0, fignum=None, timeoffset=0, reusefig=False, **kwargs):
    """
    
    """
    if tstart is None and tend is None:
        tstart = 0
        tend = polynomial.duration
    
    tVect = np.arange(tstart, tend, stepsize)
    if tVect[-1] < tend:
        tVect = np.append(tVect, tend)
    xVect = [polynomial.Evaldn(t, derivative) for t in tVect]

    fig = None
    if not reusefig:
        fig = plt.figure(fignum)
    
    plt.plot(tVect + timeoffset, xVect, **kwargs)
    plt.show(False)
    return fig

def PlotPiecewisePolynomial(pwpoly, stepsize=0.01, derivative=0, fignum=None, **kwargs):
    """
    """
    fig = plt.figure(fignum)
    
    tstart = 0
    for poly in pwpoly.GetPolynomials():
        PlotPolynomial(poly, stepsize=stepsize, derivative=derivative, timeoffset=tstart, reusefig=True, **kwargs)
        tstart += poly.duration
    plt.show(False)
    return fig

def PlotChunk(chunk, stepsize=0.01, derivative=0, fignum=None, timeoffset=0, reusefig=False, **kwargs):
    """
    
    """
    tVect = np.arange(0, chunk.duration, stepsize)
    if tVect[-1] < chunk.duration:
        tVect = np.append(tVect, chunk.duration)
    xVect = [chunk.Evaldn(t, derivative) for t in tVect]

    fig = None
    if not reusefig:
        fig = plt.figure(fignum)
    
    plt.plot(tVect + timeoffset, xVect, **kwargs)
    plt.show(False)
    return fig
    
def PlotChunks(chunks, stepsize=0.01, derivative=0, fignum=None, **kwargs):
    """
    
    """
    fig = plt.figure(fignum)

    tstart = 0
    for chunk in chunks:
        PlotChunk(chunk, stepsize=stepsize, derivative=derivative, timeoffset=tstart, reusefig=True, **kwargs)
        tstart += chunk.duration
    plt.show(False)
    return fig
    
def LoadPiecewisePolynomialTrajectoryFromFile(filename):
    with open(filename, 'r') as f:
        trajdata = f.read()
    pwtraj = piecewisepolynomials.PiecewisePolynomialTrajectory()
    pwtraj.Deserialize(trajdata)
    return pwtraj

def ConvertPiecewisePolynomialTrajectoryToOpenRAVETrajectory(env, robot, pwptraj):
    """Create and return an OpenRAVE trajectory representation of the given piecewise-polynomial trajectory.
    
    Assume that the robot active dofs are set correctly.
    
    """
    from openravepy import RaveCreateTrajectory, ConfigurationSpecification

    ortraj = RaveCreateTrajectory(env, '')
    valuesInterpolation = None
    if pwptraj.degree == 3:
        posspec = robot.GetActiveConfigurationSpecification('cubic')
    elif pwptraj.degree == 5:
        posspec = robot.GetActiveConfigurationSpecification('quintic')
    else:
        log.warn('Do not support converting a piecewise-polynomial trajectory of degree %d into OpenRAVE trajectory yet.',
                 pwptraj.degree)
        return None

    velspec = posspec.ConvertToDerivativeSpecification(1)
    accelspec = posspec.ConvertToDerivativeSpecification(2)
    newspec = posspec + velspec + accelspec
    deltatimeoffset = newspec.AddDeltaTimeGroup()
    ortraj.Init(newspec)

    gvalues = newspec.GetGroupFromName('joint_values')
    gvelocities = newspec.GetGroupFromName('joint_velocities')
    gaccelerations = newspec.GetGroupFromName('joint_accelerations')

    vtrajpoint = np.zeros(newspec.GetDOF())

    prevChunkDuration = None
    for ichunk, chunk in enumerate(pwptraj.GetChunks()):
        newspec.InsertJointValues(vtrajpoint, chunk.Eval(0), robot, robot.GetActiveDOFIndices(), 0)
        newspec.InsertJointValues(vtrajpoint, chunk.Evald1(0), robot, robot.GetActiveDOFIndices(), 1)
        newspec.InsertJointValues(vtrajpoint, chunk.Evald2(0), robot, robot.GetActiveDOFIndices(), 2)
        if prevChunkDuration is not None:
            newspec.InsertDeltaTime(vtrajpoint, prevChunkDuration)
        ortraj.Insert(ortraj.GetNumWaypoints(), vtrajpoint)
        
        prevChunkDuration = chunk.duration

    # Last waypoint
    lastChunk = pwptraj.GetChunk(pwptraj.GetNumChunks() - 1)
    newspec.InsertJointValues(vtrajpoint, lastChunk.Eval(lastChunk.duration), robot, robot.GetActiveDOFIndices(), 0)
    newspec.InsertJointValues(vtrajpoint, lastChunk.Evald1(lastChunk.duration), robot, robot.GetActiveDOFIndices(), 1)
    newspec.InsertJointValues(vtrajpoint, lastChunk.Evald2(lastChunk.duration), robot, robot.GetActiveDOFIndices(), 2)
    newspec.InsertDeltaTime(vtrajpoint, lastChunk.duration)
    ortraj.Insert(ortraj.GetNumWaypoints(), vtrajpoint)

    return ortraj
