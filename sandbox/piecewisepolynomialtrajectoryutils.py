from openravepy import openravepy_piecewisepolynomials as piecewisepolynomials
import matplotlib.pyplot as plt
import numpy as np
from pylab import ion
ion()

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

