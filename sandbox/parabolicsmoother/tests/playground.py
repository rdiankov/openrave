import mpmath
from mpmath import mp
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('../')
import IPython

import ramp
from ramp import ParabolicCheckReturn as PCR

import random
rng = random.SystemRandom()

def RandVect1(n, l, u):
    return np.asarray([rng.uniform(l, u) for _ in xrange(n)])

def RandVect2(l, u):
    return np.asarray([rng.uniform(l[i], u[i]) for i in xrange(len(l))])

################################################################################
# test precision for easy number assignments
ramp1 = ramp.Ramp(0, 2, 1, 0)
ramp2 = ramp.Ramp(ramp1.v1, -ramp1.a, 0.8, ramp1.d)
curve = ramp.ParabolicCurve([ramp1, ramp2])

assert(ramp.Abs(ramp.Sub(curve.duration, mp.mpf('1.8'))) < ramp.epsilon)


from interpolation import *
import interpolation
vm = 5
am = 2
x0 = 0
x1 = 3
v0 = 0.5
v1 = 0.9

curve1 = Interpolate1D(x0, x1, v0, v1, vm, am)
curve2 = Interpolate1D(x0, x1, v0, v1, 1.5, am)

# curvesnd = ParabolicCurvesND([curve1])
# curvesnd.PlotVel(includingSW=True)
# curvesnd.PlotAcc()

################################################################################
# Check displacement after interpolation (d == x1 - x0)
# Check velocity continuity between consecutive ramps
x0 = 0
x1 = 3
maxit = 100
curves = []
for i in xrange(maxit):
    v0 = rng.uniform(-vm, vm)
    v1 = rng.uniform(-vm, vm)
    try:
        temp = Interpolate1D(x0, x1, v0, v1, vm, am)
        assert(ramp.IsEqual(temp.d, 3))
        assert(ramp.IsEqual(temp[0].v1, temp[1].v0))
        if len(temp) == 3:
            assert(ramp.IsEqual(ramp.Abs(temp[0].v1), vm))
            assert(ramp.IsEqual(ramp.Abs(temp[1].v0), vm))
            assert(ramp.IsEqual(ramp.Abs(temp[2].v0), vm))
    except:
        print 'x0 = {0}'.format(x0)
        print 'x1 = {0}'.format(x1)
        print 'v0 = {0}'.format(v0)
        print 'v1 = {0}'.format(v1)
        break

################################################################################
# Check rest-to-rest interpolation
x0Vect = RandVect1(6, -2, 2)
x1Vect = RandVect1(6, -2, 2)

vmVect = RandVect1(6, 0, 5)
amVect = RandVect1(6, 0, 3)

v0 = RandVect2(-vmVect, vmVect)
v1 = RandVect2(-vmVect, vmVect)

curvesnd = InterpolateZeroVelND(x0Vect, x1Vect, vmVect, amVect)
# Check the duration of each parabolic curve
# Check the displacement after interpolation
dur = curvesnd[0].duration
for (i, curve) in enumerate(curvesnd[1:]):
    assert(curve.duration == dur)
    assert(ramp.IsEqual(curve.d, x1Vect[i + 1] - x0Vect[i + 1]))

################################################################################
# Check interpolation with fixed duration
x0 = 0
x1 = 3
v0 = 0
v1 = 2
duration = mp.mpf('5')
vm = 2.5
am = 1.8

prevCurve = Interpolate1D(x0, x1, v0, v1, vm, am)
fixedCurve = interpolation._Stretch1D(prevCurve, duration, vm, am)
# Check if the interpolation preserves the displacement
assert(ramp.IsEqual(prevCurve.d, 3))
assert(ramp.IsEqual(fixedCurve.d, 3))
# Check if the interpolation preserves the duration
assert(ramp.IsEqual(fixedCurve.duration, duration))

x0 = 0
x1 = mp.rand()*3
v0 = 0
v1 = mp.rand()*2
duration = mp.mpf('5')
vm = 2.5
am = 1.8

prevCurve = Interpolate1D(x0, x1, v0, v1, vm, am)
fixedCurve = interpolation._Stretch1D(prevCurve, duration, vm, am)
# Check if the interpolation preserves the displacement
assert(ramp.IsEqual(prevCurve.d, mp.mpf(str(x1))))
assert(ramp.IsEqual(fixedCurve.d, mp.mpf(str(x1))))
# Check if the interpolation preserves the duration
assert(ramp.IsEqual(fixedCurve.duration, duration))

nTrials = 0
nSuccess = 0
for _ in xrange(nTrials):
    x0 = mp.rand()
    x1 = mp.rand()*3
    v0 = mp.rand()*(-2)
    v1 = mp.rand()*2
    duration = mp.mpf('5')
    vm = 3 + (2*mp.rand() - 1)*0.5
    am = 2 + (2*mp.rand() - 1)*0.2

    prevCurve = Interpolate1D(x0, x1, v0, v1, vm, am)
    fixedCurve = interpolation._Stretch1D(prevCurve, duration, vm, am)
    if not fixedCurve.isEmpty:
        assert(ramp.IsEqual(prevCurve.d, ramp.Sub(mp.mpf(str(x1)), mp.mpf(str(x0)))))
        assert(ramp.IsEqual(fixedCurve.d, ramp.Sub(mp.mpf(str(x1)), mp.mpf(str(x0)))))
        assert(ramp.IsEqual(fixedCurve.duration, duration))
        nSuccess += 1
print "Easy bounds : interpolation with fixed duration successful instance = {0}/{1}".format(nSuccess, nTrials)

################################################################################
# Check interpolation with fixed duration (bounds from a real robot)
xMin = np.array([-2.9670597283903604,  0.                ,  0.                , -3.1415926535897931, -2.0943951023931953, -3.1415926535897931])
xMax = np.array([ 2.9670597283903604,  2.3561944901923448,  2.6703537555513241,  3.1415926535897931,  2.0943951023931953,  3.1415926535897931])

realVel = np.array([ 4.3943027242087229,  3.8975383858598374,  4.6770460630318027,  5.846307578789756 ,  5.846307578789756 ,  9.3540921260636072])
realAcc = np.array([ 24.0331837999619182,  18.3783170235002906,  25.2489801569011689,  50.893800988154652 ,  46.3133588992207308,  62.4862778799010101])

vScale = 0.9
vMax = vScale*realVel
vMin = -1.0*np.array(vMax)

aScale = 0.81
aMax = aScale*realAcc

nTrials = 1000
nSuccess = 0
vboundfailed = 0
xboundfailed = 0
interpfailed = 0
for it in xrange(nTrials):
    print "iteration {0}/{1}".format(it + 1, nTrials)
    x0Vect = RandVect2(xMin, xMax)
    x1Vect = RandVect2(xMin, xMax)
    v0Vect = RandVect2(vMin, vMax)
    v1Vect = RandVect2(vMin, vMax)

    curvesnd = InterpolateArbitraryVelND(x0Vect, x1Vect, v0Vect, v1Vect, xMin, xMax, vMax, aMax, tryHarder=True)
    if not curvesnd.isEmpty:
        ret = ramp.CheckParabolicCurvesND(curvesnd, xMin, xMax, vMax, aMax, x0Vect, x1Vect, v0Vect, v1Vect)
        # print ret
        # raw_input()
        if ret == PCR.Normal:
            nSuccess += 1
        elif ret == PCR.VBoundViolated:
            vboundfailed += 1
            break
        elif ret == PCR.XBoundViolated:
            xboundfailed += 1
            # IPython.embed()
        else:
            print ret
            raw_input()
        # if ramp.CheckParabolicCurvesND(curvesnd, vMax, aMax, x0Vect=x0Vect, x1Vect=x1Vect):
        #     nSuccess += 1
        #     break
    else:
        interpfailed += 1
        # IPython.embed()
print "Real bounds : interpolation with fixed duration successful instances = {0}/{1}".format(nSuccess, nTrials)
print "Real bounds : interpolation failed = {0}/{1}".format(interpfailed, nTrials)
print "Real bounds : x-bound failed = {0}/{1}".format(xboundfailed, nTrials)
print "Real bounds : other failures = {0}/{1}".format(nTrials - nSuccess - interpfailed - xboundfailed, nTrials)
