import mpmath
from mpmath import mp
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('../')

import ramp

ramp1 = ramp.Ramp(0, 2, 1, 0)
ramp2 = ramp.Ramp(ramp1.v1, -ramp1.a, 0.8, ramp1.d)
curve = ramp.ParabolicCurve([ramp1, ramp2])

assert(ramp.Abs(ramp.Sub(curve.duration, mp.mpf('1.8'))) < ramp.epsilon)


from interpolation import *
vm = 5
am = 2
x0 = 0
x1 = 3
v0 = 0.5
v1 = 0.9

curve1 = Interpolate1D(x0, x1, v0, v1, vm, am)
curve2 = Interpolate1D(x0, x1, v0, v1, 1.5, am)

curvesnd = ParabolicCurvesND([curve1])
curvesnd.PlotVel(includingSW=True)
curvesnd.PlotAcc()


################################################################################
import random
rng = random.SystemRandom()

def RandVect1(n, l, u):
    return np.asarray([rng.uniform(l, u) for _ in xrange(n)])

def RandVect2(l, u):
    return np.asarray([rng.uniform(l[i], u[i]) for i in xrange(len(l))])

x0Vect = RandVect1(6, -2, 2)
x1Vect = RandVect1(6, -2, 2)

vmVect = RandVect1(6, 0, 5)
amVect = RandVect1(6, 0, 3)

v0 = RandVect2(-vmVect, vmVect)
v1 = RandVect2(-vmVect, vmVect)

curvesnd = InterpolateZeroVelND(x0Vect, x1Vect, vmVect, amVect)


