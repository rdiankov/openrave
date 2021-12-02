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

def ComputeCubicCoefficients(x0, x1, dx0, dx1, ddx0, ddx1, t):
    """Compute the cubic coefficients a, b, c, d for
    
    p(x) = ax^3 + bx^2 + cx + d

    that satisfies the given boundary conditions
    
    Assume that the given boundary conditions are consistent.
    
    """
    d = x0
    c = dx0
    b = 0.5*ddx0
    a = (ddx1 - ddx0)/(6*t)
    return a, b, c, d

def ConvertOpenRAVETrajectoryToPiecewisePolynomialTrajectory(robot, traj, degree):
    """
    
    Assume that the given openrave trajectory is consistent.

    TODO: maybe can write a helper function to decide what degree the
    piecewise polynomial traj should be. for now, get it as an input.

    """
    trajspec = traj.GetConfigurationSpecification()
    assert(robot in trajspec.ExtractUsedBodies(robot.GetEnv()))

    robotIndices = robot.GetActiveDOFIndices()
    ndof = len(robotIndices)
    
    bExactMatch = False
    gpos = trajspec.FindCompatibleGroup('joint_values', bExactMatch)
    gvel = trajspec.FindCompatibleGroup('joint_velocities', bExactMatch)
    gacc = trajspec.FindCompatibleGroup('joint_accelerations', bExactMatch)
    

    if degree == 1:
        assert(gpos is not None)
        assert(gpos.interpolation == 'linear')
        raise NotImplementedError
    
    elif degree == 2:
        assert(gpos is not None)
        assert(gpos.interpolation == 'quadratic')
        assert(gvel is not None)
        assert(gvel.interpolation == 'linear')
        raise NotImplementedError

    elif degree == 3:
        assert(gpos is not None)
        assert(gpos.interpolation == 'cubic')
        assert(gvel is not None)
        assert(gvel.interpolation == 'quadratic')
        assert(gacc is not None)
        assert(gacc.interpolation == 'linear')

        chunks = []
        
        waypoint = traj.GetWaypoint(0)
        x0Vect = trajspec.ExtractJointValues(waypoint, robot, robotIndices, 0)
        v0Vect = trajspec.ExtractJointValues(waypoint, robot, robotIndices, 1)
        a0Vect = trajspec.ExtractJointValues(waypoint, robot, robotIndices, 2)
        # TODO: maybe need some codes to clean up the evaluated value (in case numerical errors result in values
        # violating their corresponding limits)

        for iwaypoint in range(1, traj.GetNumWaypoints()):
            nextWaypoint = traj.GetWaypoint(iwaypoint)
            x1Vect = trajspec.ExtractJointValues(nextWaypoint, robot, robotIndices, 0)
            v1Vect = trajspec.ExtractJointValues(nextWaypoint, robot, robotIndices, 1)
            a1Vect = trajspec.ExtractJointValues(nextWaypoint, robot, robotIndices, 2)
            # TODO: maybe need some codes to clean up the evaluated value (in case numerical errors result in values
            # violating their corresponding limits)

            deltaTime = trajspec.ExtractDeltaTime(nextWaypoint)

            polynomials = []
            for idof in range(ndof):
                a, b, c, d = ComputeCubicCoefficients(x0Vect[idof], x1Vect[idof],
                                                      v0Vect[idof], v1Vect[idof],
                                                      a0Vect[idof], a1Vect[idof],
                                                      deltaTime)
                poly = piecewisepolynomials.Polynomial(deltaTime, [d, c, b, a])
                polynomials.append(poly)
            chunk = piecewisepolynomials.Chunk(deltaTime, polynomials)
            chunks.append(chunk)

            x0Vect = np.array(x1Vect)
            v0Vect = np.array(v1Vect)
            a0Vect = np.array(a1Vect)

        return piecewisepolynomials.PiecewisePolynomialTrajectory(chunks)
        
    else:
        raise NotImplementedError
        

