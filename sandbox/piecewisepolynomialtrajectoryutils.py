from openravepy import openravepy_piecewisepolynomials as piecewisepolynomials
import matplotlib.pyplot as plt
import numpy as np
from pylab import ion
ion()

def PlotPolynomial(polynomial, t0, t1, stepsize=0.01, fignum=None, **kwargs):
    """
    
    """
    tVect = np.arange(t0, t1, stepsize)
    if tVect[-1] < t1:
        tVect = np.append(tVect, t1)

    xVect = [polynomial.Eval(t) for t in tVect]

    fig = plt.figure(fignum)
    plt.plot(tVect, xVect, **kwargs)
    plt.show(False)
    return fig

def PlotChunk(chunk, stepsize=0.01, fignum=None, **kwargs):
    """
    
    """
    tVect = np.arange(0, chunk.duration)
    if tVect[-1] < chunk.duration:
        tVect = np.append(tVect, chunk.duration)
    
    xVect = [chunk.Eval(t) for t in tVect]

    fig = plt.figure(fignum)
    plt.plot(tVect, xVect, **kwargs)
    plt.show(False)
    return fig
    
