from openravepy import openravepy_piecewisepolynomials as piecewisepolynomials
import matplotlib.pyplot as plt
import numpy as np
from pylab import ion
ion()

def PlotPolynomial(polynomial, tstart=None, tend=None, stepsize=0.01, fignum=None, **kwargs):
    """
    
    """
    if tstart is None and tend is None:
        tstart = 0
        tend = polynomial.duration
    tVect = np.arange(tstart, tend, stepsize)
    if tVect[-1] < tend:
        tVect = np.append(tVect, tend)

    xVect = [polynomial.Eval(t) for t in tVect]

    fig = plt.figure(fignum)
    plt.plot(tVect, xVect, **kwargs)
    plt.show(False)
    return fig

def PlotChunk(chunk, stepsize=0.01, fignum=None, **kwargs):
    """
    
    """
    tVect = np.arange(0, chunk.duration, stepsize)
    if tVect[-1] < chunk.duration:
        tVect = np.append(tVect, chunk.duration)
    
    xVect = [chunk.Eval(t) for t in tVect]

    fig = plt.figure(fignum)
    plt.plot(tVect, xVect, **kwargs)
    plt.show(False)
    return fig
    
