from mpmath import mp, iv, arange
import numpy as np

from ramp import Ramp, ParabolicCurve, ParabolicCurvesND
from ramp import ConvertFloatToMPF, ConvertFloatArrayToMPF
from ramp import zero, pointfive, epsilon
from ramp import Add, Abs, IsEqual, Mul, Neg, Prod, Sqr, Sub, Sum, FuzzyEquals, FuzzyZero

inf = mp.inf
one = mp.mpf('1')
number = mp.mpf
_prec = 15

import logging
logging.basicConfig(format='[%(levelname)s] [%(name)s: %(funcName)s] %(message)s', level=logging.DEBUG)
log = logging.getLogger(__name__)

####################################################################################################
# Multi DOF


def InterpolateZeroVelND(x0Vect, x1Vect, vmVect, amVect, delta=zero):
    """Interpolate a trajectory connecting two waypoints, x0Vect and x1Vect. Velocities at both
    waypoints are zeros.

    """
    ndof = len(x0Vect)
    assert(ndof == len(x1Vect))
    assert(ndof == len(vmVect))
    assert(ndof == len(amVect))

    # Convert all vector elements into mp.mpf (if necessary)
    x0Vect_ = ConvertFloatArrayToMPF(x0Vect)
    x1Vect_ = ConvertFloatArrayToMPF(x1Vect)
    vmVect_ = ConvertFloatArrayToMPF(vmVect)
    amVect_ = ConvertFloatArrayToMPF(amVect)
    
    dVect = x1Vect - x0Vect

    if type(delta) is not mp.mpf:
        delta = mp.mpf("{:.15e}".format(delta))

    vMin = inf # the tightest velocity bound
    aMin = inf # the tightest acceleration bound
    for i in xrange(ndof):
        if not IsEqual(x1Vect[i], x0Vect[i]):
            vMin = min(vMin, vmVect[i]/Abs(dVect[i]))
            aMin = min(aMin, amVect[i]/Abs(dVect[i]))

    if (not (vMin < inf and aMin < inf)):
        # dVect is zero.
        curvesnd = ParabolicCurvesND()
        curvesnd.SetConstant(x0Vect_, 0)
        return curvesnd

    if delta == zero:
        sdProfile = Interpolate1D(zero, one, zero, zero, vMin, aMin) # parabolic ramp (velocity profile sd(t))
    else:
        # Not handle interpolation with switch-time constraints yet
        raise NotImplementedError

    # Scale each DOF according to the obtained sd-profile
    curves = [ParabolicCurve() for _ in xrange(ndof)] # a list of (empty) parabolic curves
    for sdRamp in sdProfile:
        aVect = sdRamp.a * dVect
        v0Vect = sdRamp.v0 * dVect
        dur = sdRamp.duration

        for j in xrange(ndof):
            ramp = Ramp(v0Vect[j], aVect[j], dur, x0Vect[j])
            curve = ParabolicCurve([ramp])
            curves[j].Append(curve)

    for (i, curve) in enumerate(curves):
        curve.SetInitialValue(x0Vect[i])        
    curvesnd = ParabolicCurvesND(curves)

    return curvesnd


def InterpolateArbitraryVelND(x0Vect_, x1Vect_, v0Vect_, v1Vect_, xminVect_, xmaxVect_, vmVect_, amVect_, delta=zero, tryHarder=False):
    """Interpolate a trajectory connecting two waypoints, (x0Vect, v0Vect) and (x1Vect, v1Vect).

    """
    ndof = len(x0Vect_)
    assert(ndof == len(x1Vect_))
    assert(ndof == len(v0Vect_))
    assert(ndof == len(v1Vect_))
    assert(ndof == len(xminVect_))
    assert(ndof == len(xmaxVect_))
    assert(ndof == len(vmVect_))
    assert(ndof == len(amVect_))

    # Convert all vector elements into mp.mpf (if necessary)
    x0Vect = ConvertFloatArrayToMPF(x0Vect_)
    x1Vect = ConvertFloatArrayToMPF(x1Vect_)
    v0Vect = ConvertFloatArrayToMPF(v0Vect_)
    v1Vect = ConvertFloatArrayToMPF(v1Vect_)
    xminVect = ConvertFloatArrayToMPF(xminVect_)
    xmaxVect = ConvertFloatArrayToMPF(xmaxVect_)
    vmVect = ConvertFloatArrayToMPF(vmVect_)
    amVect = ConvertFloatArrayToMPF(amVect_)
    
    dVect = x1Vect - x0Vect

    if type(delta) is not mp.mpf:
        delta = mp.mpf("{:.15e}".format(delta))

    # First independently interpolate each DOF to find out the slowest one.
    curves = []
    maxDuration = zero
    maxIndex = 0
    for i in xrange(ndof):
        if delta == zero:
            curve = Interpolate1D(x0Vect[i], x1Vect[i], v0Vect[i], v1Vect[i], vmVect[i], amVect[i])
        else:
            raise NotImplementedError
        curves.append(curve)
        if curve.duration > maxDuration:
            maxDuration = curve.duration
            maxIndex = i        

    ## TEMPORARY
    # print "maxIndex = {0}".format(maxIndex)
    curvesnd = ReinterpolateNDFixedDuration(curves, vmVect, amVect, maxIndex, delta, tryHarder)

    newCurves = []
    for (i, curve) in enumerate(curvesnd.curves):
        newCurve = _ImposeJointLimitFixedDuration(curve, xminVect[i], xmaxVect[i], vmVect[i], amVect[i])
        if newCurve.isEmpty:
            return ParabolicCurvesND()
        newCurves.append(newCurve)
    
    return ParabolicCurvesND(newCurves)


def ReinterpolateNDFixedDuration(curves, vmVect, amVect, maxIndex, delta=zero, tryHarder=False):
    ndof = len(curves)
    assert(ndof == len(vmVect))
    assert(ndof == len(amVect))
    assert(maxIndex < ndof)

    # Convert all vector elements into mp.mpf (if necessary)
    vmVect_ = ConvertFloatArrayToMPF(vmVect)
    amVect_ = ConvertFloatArrayToMPF(amVect)

    newCurves = []

    if not tryHarder:
        newDuration = curves[maxIndex].duration
        for (idof, curve) in enumerate(curves):
            if idof == maxIndex:
                newCurves.append(curve)
                continue

            stretchedCurve = _Stretch1D(curve, newDuration, vmVect[idof], amVect[idof])
            if stretchedCurve.isEmpty:
                log.debug('ReinterpolateNDFixedDuration: dof {0} failed'.format(idof))
                return ParabolicCurvesND()
            else:
                newCurves.append(stretchedCurve)

        assert(len(newCurves) == ndof)
        return ParabolicCurvesND(newCurves)

    else:
        # Try harder to re-interpolate this trajectory. This is guaranteed to have 100% success rate
        isPrevDurationSafe = False
        newDuration = zero
        for (idof, curve) in enumerate(curves):
            t = _CalculateLeastUpperBoundInoperativeInterval(curve.x0, curve.EvalPos(curve.duration), curve.v0, curve.EvalVel(curve.duration), vmVect[idof], amVect[idof])
            assert(t > zero)
            if t > newDuration:
                newDuration = t
        assert(not (newDuration == zero))
        if curves[maxIndex].duration > newDuration:
            # The duration of the slowest DOF is already safe
            newDuration = curves[maxIndex].duration
            isPrevDurationSafe = True

        for (idof, curve) in enumerate(curves):
            if (isPrevDurationSafe) and (idof == maxIndex):
                newCurves.append(curve)
                continue

            stretchedCurve = _Stretch1D(curve, newDuration, vmVect[idof], amVect[idof])
            if stretchedCurve.isEmpty:
                log.debug('ReinterpolateNDFixedDuration: dof {0} failed even when trying harder'.format(idof))
                log.debug('x0 = {0}; x1 = {1}; v0 = {2}; v1 = {3}; vm = {4}; am = {5}; newDuration = {6}'.\
                          format(mp.nstr(curve.x0, n=_prec), mp.nstr(curve.EvalPos(curve.duration), n=_prec),
                                 mp.nstr(curve.v0, n=_prec), mp.nstr(curve.EvalVel(curve.duration), n=_prec),
                                 mp.nstr(vmVect[idof], n=_prec), mp.nstr(amVect[idof], n=_prec),
                                 mp.nstr(newDuration, n=_prec)))
                raise Exception('Something is wrong when calculating the least upper bound of inoperative intervals')

            newCurves.append(stretchedCurve)

        assert(len(newCurves) == ndof)
        return ParabolicCurvesND(newCurves)


def InterpolateNDFixedDuration(x0Vect_, x1Vect_, v0Vect_, v1Vect_, duration, xminVect_, xmaxVect_, vmVect_, amVect_):
    assert(duration > 0)

    ndof = len(x0Vect_)
    assert(ndof == len(x1Vect_))
    assert(ndof == len(v0Vect_))
    assert(ndof == len(v1Vect_))
    assert(ndof == len(xminVect_))
    assert(ndof == len(xmaxVect_))
    assert(ndof == len(vmVect_))
    assert(ndof == len(amVect_))

    # Convert all vector elements into mp.mpf (if necessary)
    x0Vect = ConvertFloatArrayToMPF(x0Vect_)
    x1Vect = ConvertFloatArrayToMPF(x1Vect_)
    v0Vect = ConvertFloatArrayToMPF(v0Vect_)
    v1Vect = ConvertFloatArrayToMPF(v1Vect_)
    xminVect = ConvertFloatArrayToMPF(xminVect_)
    xmaxVect = ConvertFloatArrayToMPF(xmaxVect_)
    vmVect = ConvertFloatArrayToMPF(vmVect_)
    amVect = ConvertFloatArrayToMPF(amVect_)
    
    dVect = x1Vect - x0Vect

    duration = ConvertFloatToMPF(duration)

    curves = []
    for idof in xrange(ndof):
        curve = Interpolate1DFixedDuration(x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], duration, vmVect[idof], amVect[idof])
        if curve.isEmpty:
            return ParabolicCurvesND()

        newCurve = _ImposeJointLimitFixedDuration(curve, xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof])
        if newCurve.isEmpty:
            return ParabolicCurvesND()
        
        curves.append(newCurve)
    return ParabolicCurvesND(curves)


####################################################################################################
# Single DOF


def Interpolate1D(x0, x1, v0, v1, vm, am, delta=zero):
    # Check types
    if type(x0) is not mp.mpf:
        x0 = mp.mpf("{:.15e}".format(x0))
    if type(x1) is not mp.mpf:
        x1 = mp.mpf("{:.15e}".format(x1))
    if type(v0) is not mp.mpf:
        v0 = mp.mpf("{:.15e}".format(v0))
    if type(v1) is not mp.mpf:
        v1 = mp.mpf("{:.15e}".format(v1))
    if type(vm) is not mp.mpf:
        vm = mp.mpf("{:.15e}".format(vm))
    if type(am) is not mp.mpf:
        am = mp.mpf("{:.15e}".format(am))

    # Check inputs
    assert(vm > zero)
    assert(am > zero)
    assert(Abs(v0) <= vm)
    assert(Abs(v1) <= vm)
    
    curve = _Interpolate1DNoVelocityLimit(x0, x1, v0, v1, am)
    if len(curve) == 1:
        return curve

    return _ImposeVelocityLimit(curve, vm)


def _Interpolate1DNoVelocityLimit(x0, x1, v0, v1, am):
    # Check types
    if type(x0) is not mp.mpf:
        x0 = mp.mpf("{:.15e}".format(x0))
    if type(x1) is not mp.mpf:
        x1 = mp.mpf("{:.15e}".format(x1))
    if type(v0) is not mp.mpf:
        v0 = mp.mpf("{:.15e}".format(v0))
    if type(v1) is not mp.mpf:
        v1 = mp.mpf("{:.15e}".format(v1))
    if type(am) is not mp.mpf:
        am = mp.mpf("{:.15e}".format(am))

    # Check inputs
    assert(am > zero)

    # Check for an appropriate acceleration direction of the first ramp
    d = Sub(x1, x0)
    dv = Sub(v1, v0)
    difVSqr = Sub(v1**2, v0**2)
    
    if Abs(dv) < epsilon:
        if Abs(d) < epsilon:
            # Stationary ramp
            ramp0 = Ramp(zero, zero, zero, x0)
            return ParabolicCurve([ramp0])

        else:
            dStraight = zero
    else:    
        dStraight = mp.fdiv(difVSqr, Prod([2, mp.sign(dv), am]))
    
    if IsEqual(d, dStraight):
        # With the given distance, v0 and v1 can be directly connected using max/min
        # acceleration. Here the resulting profile has only one ramp.
        a0 = mp.sign(dv) * am
        ramp0 = Ramp(v0, a0, mp.fdiv(dv, a0), x0)
        return ParabolicCurve([ramp0])

    sumVSqr = Add(v0**2, v1**2)
    sigma = mp.sign(Sub(d, dStraight))
    a0 = sigma * am # acceleration of the first ramp
    vp = sigma * mp.sqrt(Add(Mul(pointfive, sumVSqr), Mul(a0, d)))
    t0 = mp.fdiv(Sub(vp, v0), a0)
    t1 = mp.fdiv(Sub(vp, v1), a0)
    ramp0 = Ramp(v0, a0, t0, x0)    
    assert(IsEqual(ramp0.v1, vp)) # check soundness
    ramp1 = Ramp(vp, Neg(a0), t1)

    curve = ParabolicCurve([ramp0, ramp1])
    assert(IsEqual(curve.d, d)) # check soundness
    return curve


def _ImposeVelocityLimit(curve, vm):
    """_ImposeVelocityLimit imposes the given velocity limit to the ParabolicCurve. In case the velocity
    limit cannot be satisfied, this function will return an empty ParabolicCurve.

    """
    # Check types
    if type(vm) is not mp.mpf:
        vm = mp.mpf("{:.15e}".format(vm))

    # Check inputs
    assert(vm > zero)
    assert(len(curve) == 2)
    assert(Add(curve[0].a, curve[1].a) == zero)

    if Sub(Abs(curve[0].v0), vm) > epsilon:
        # Initial velocity violates the constraint
        return ParabolicCurve()

    if Sub(Abs(curve[1].v1), vm) > epsilon:
        # Final velocity violates the constraint
        return ParabolicCurve()
    
    vp = curve[1].v0
    if Abs(vp) <= vm:
        # Velocity limit is not violated
        return curve

    # h = Sub(Abs(vp), vm)
    # t = mp.fdiv(h, Abs(curve[0].a))
    
    ramp0, ramp1 = curve
    h = Sub(Abs(vp), vm)
    t = mp.fdiv(h, Abs(ramp0.a))

    # import IPython; IPython.embed()

    ramps = []
    if IsEqual(Abs(ramp0.v0), vm) and (mp.sign(ramp0.v0) == mp.sign(vp)):
        assert(IsEqual(ramp0.duration, t)) # check soundness
    else:
        newRamp0 = Ramp(ramp0.v0, ramp0.a, Sub(ramp0.duration, t), ramp0.x0)
        ramps.append(newRamp0)
        
    nom = h**2
    denom = Mul(Abs(curve[0].a), vm)
    newRamp1 = Ramp(Mul(mp.sign(vp), vm), zero, Sum([t, t, mp.fdiv(nom, denom)]), curve.x0)
    ramps.append(newRamp1)

    if IsEqual(Abs(ramp1.v1), vm) and (mp.sign(ramp1.v1) == mp.sign(vp)):
        assert(IsEqual(ramp1.duration, t)) # check soundness
    else:
        newRamp2 = Ramp(Mul(mp.sign(vp), vm), ramp1.a, Sub(ramp1.duration, t))
        ramps.append(newRamp2)

    return ParabolicCurve(ramps)


####################################################################################################
"""The following procedure for fixing x-bound violation is copied from OpenRAVE's
ParabolicPathSmooth library.
"""

def _SolveAXMB(a, b, eps, xmin, xmax):
    """
    This function 'safely' solves for a value x \in [xmin, xmax] such that  |a*x - b| <= eps*max(|a|, |b|).
    Assume xmin <= 0 <= xmax.

    This function returns [result, x].
    """
    if (a < 0):
        return _SolveAXMB(-a, -b, eps, xmin, xmax)

    epsScaled = eps*max(a, Abs(b)) # we already know that a > 0

    # Infinite range
    if ((xmin == -inf) and (xmax == inf)):
        if (a == zero):
            x = zero
            result = Abs(b) <= epsScaled
            return [result, x]

        x = mp.fdiv(b, a)
        return [True, x]

    axmin = Mul(a, xmin)
    axmax = Mul(a, xmax)

    if not ((Add(b, epsScaled) >= axmin) and (Sub(b, epsScaled) <= axmax)):
        # Ranges do not intersect
        return [False, zero]

    if not (a == zero):
        x = mp.fdiv(b, a)
        if (xmin <= x) and (x <= xmax):
            return [True, x]

    if (Abs(Sub(Mul(pointfive, Add(axmin, axmax)), b)) <= epsScaled):
        x = Mul(pointfive, Add(xmin, xmax))
        return [True, x]

    if (Abs(Sub(axmax, b)) <= epsScaled):
        x = xmax
        return [True, x]

    assert(Abs(Sub(axmin, b)) <= epsScaled)
    x = xmin
    return [True, x]
            

def _BrakeTime(x, v, xbound):
    [result, t] = _SolveAXMB(v, Mul(number('2'), Sub(xbound, x)), epsilon, 0, inf)
    if not result:
        log.debug("Cannot solve for braking time from the equation {0}*t - {1} = 0".\
                  format(mp.nstr(v, n=_prec), mp.nstr(Mul(number('2'), Sub(xbound, x)), n=_prec)))
        return 0
    return t


def _BrakeAccel(x, v, xbound):
    coeff0 = Mul(number('2'), Sub(xbound, x))
    coeff1 = Sqr(v)
    [result, a] = _SolveAXMB(coeff0, Neg(coeff1), epsilon, -inf, inf)
    if not result:
        log.debug("Cannot solve for braking acceleration from the equation {0}*a + {1} = 0".\
                  format(mp.nstr(coeff0, n=_prec), mp.nstr(coeff1, n=_prec)))
        return 0
    return a
    

def _ImposeJointLimitFixedDuration(curve, xmin, xmax, vm, am):
    bmin, bmax = curve.GetPeaks()
    if (bmin >= Sub(xmin, epsilon)) and (bmax <= Add(xmax, epsilon)):
        # Joint limits are not violated
        return curve
    
    duration = curve.duration
    x0 = curve.x0
    x1 = curve.EvalPos(duration)
    v0 = curve.v0
    v1 = curve.v1

    bt0 = inf
    bt1 = inf
    ba0 = inf
    ba1 = inf
    bx0 = inf
    bx1 = inf
    if (v0 > zero):
        bt0 = _BrakeTime(x0, v0, xmax)
        bx0 = xmax
        ba0 = _BrakeAccel(x0, v0, xmax)
    elif (v0 < zero):
        bt0 = _BrakeTime(x0, v0, xmin)
        bx0 = xmin
        ba0 = _BrakeAccel(x0, v0, xmin)

    if (v1 < zero):
        bt1 = _BrakeTime(x1, -v1, xmax)
        bx1 = xmax
        ba1 = _BrakeAccel(x1, -v1, xmax)
    elif (v1 > zero):
        bt1 = _BrakeTime(x1, -v1, xmin)
        bx1 = xmin
        ba1 = _BrakeAccel(x1, -v1, xmin)

    # import IPython; IPython.embed()
        
    newCurve = ParabolicCurve()
    if ((bt0 < duration) and (Abs(ba0) < Add(am, epsilon))):
        # Case IIa
        log.debug("Case IIa")
        firstRamp = Ramp(v0, ba0, bt0, x0)
        if (Abs(Sub(x1, bx0)) < Mul(Sub(duration, bt0), vm)):
            tempCurve1 = Interpolate1D(bx0, x1, zero, v1, vm, am)
            if not tempCurve1.isEmpty:
                if (Sub(duration, bt0) >= tempCurve1.duration):
                    tempCurve2 = _Stretch1D(tempCurve1, Sub(duration, bt0), vm, am)
                    if not tempCurve2.isEmpty:
                        tempbmin, tempbmax = tempCurve2.GetPeaks()
                        if not ((tempbmin < Sub(xmin, epsilon)) or (tempbmax > Add(xmax, epsilon))):
                            log.debug("Case IIa successful")
                            newCurve = ParabolicCurve([firstRamp] + tempCurve2.ramps)
                                        

    if ((bt1 < duration) and (Abs(ba1) < Add(am, epsilon))):
        # Case IIb
        log.debug("Case IIb")
        lastRamp = Ramp(0, ba1, bt1, bx1)
        if (Abs(Sub(x0, bx1)) < Mul(Sub(duration, bt1), vm)):
            tempCurve1 = Interpolate1D(x0, bx1, v0, zero, vm, am)
            if not tempCurve1.isEmpty:
                if (Sub(duration, bt1) >= tempCurve1.duration):
                    tempCurve2 = _Stretch1D(tempCurve1, Sub(duration, bt1), vm, am)
                    if not tempCurve2.isEmpty:
                        tempbmin, tempbmax = tempCurve2.GetPeaks()
                        if not ((tempbmin < Sub(xmin, epsilon)) or (tempbmax > Add(xmax, epsilon))):
                            log.debug("Case IIb successful")
                            newCurve = ParabolicCurve(tempCurve2.ramps + [lastRamp])          
        

    if (bx0 == bx1):
        # Case III
        if (Add(bt0, bt1) < duration) and (max(Abs(ba0), Abs(ba1)) < Add(am, epsilon)):
            log.debug("Case III")
            ramp0 = Ramp(v0, ba0, bt0, x0)
            ramp1 = Ramp(zero, zero, Sub(duration, Add(bt0, bt1)))
            ramp2 = Ramp(zero, ba1, bt1)
            newCurve = ParabolicCurve([ramp0, ramp1, ramp2])
    else:
        # Case IV
        if (Add(bt0, bt1) < duration) and (max(Abs(ba0), Abs(ba1)) < Add(am, epsilon)):
            log.debug("Case IV")
            firstRamp = Ramp(v0, ba0, bt0, x0)
            lastRamp = Ramp(zero, ba1, bt1)
            if (Abs(Sub(bx0, bx1)) < Mul(Sub(duration, Add(bt0, bt1)), vm)):
                tempCurve1 = Interpolate1D(bx0, bx1, zero, zero, vm, am)
                if not tempCurve1.isEmpty:
                    if (Sub(duration, Add(bt0, bt1)) >= tempCurve1.duration):
                        tempCurve2 = _Stretch1D(tempCurve1, Sub(duration, Add(bt0, bt1)), vm, am)
                        if not tempCurve2.isEmpty:
                            tempbmin, tempbmax = tempCurve2.GetPeaks()
                            if not ((tempbmin < Sub(xmin, epsilon)) or (tempbmax > Add(xmax, epsilon))):
                                log.debug("Case IV successful")
                                newCurve = ParabolicCurve([firstRamp] + tempCurve2.ramps + [lastRamp])
        

    if (newCurve.isEmpty):
        log.warn("Cannot solve for a bounded trajectory")
        log.warn("x0 = {0}; x1 = {1}; v0 = {2}; v1 = {3}; xmin = {4}; xmax = {5}; vm = {6}; am = {7}; duration = {8}".\
                 format(mp.nstr(curve.x0, n=_prec), mp.nstr(curve.EvalPos(curve.duration), n=_prec),
                        mp.nstr(curve.v0, n=_prec), mp.nstr(curve.EvalVel(curve.duration), n=_prec),
                        mp.nstr(xmin, n=_prec), mp.nstr(xmax, n=_prec),
                        mp.nstr(vm, n=_prec), mp.nstr(am, n=_prec), mp.nstr(duration, n=_prec)))
        return newCurve

    newbmin, newbmax = newCurve.GetPeaks()
    if (newbmin < Sub(xmin, epsilon)) or (newbmax > Add(xmax, epsilon)):
        log.warn("Solving finished but the trajectory still violates the bounds")
        # import IPython; IPython.embed()
        log.warn("x0 = {0}; x1 = {1}; v0 = {2}; v1 = {3}; xmin = {4}; xmax = {5}; vm = {6}; am = {7}; duration = {8}".\
                 format(mp.nstr(curve.x0, n=_prec), mp.nstr(curve.EvalPos(curve.duration), n=_prec),
                        mp.nstr(curve.v0, n=_prec), mp.nstr(curve.EvalVel(curve.duration), n=_prec),
                        mp.nstr(xmin, n=_prec), mp.nstr(xmax, n=_prec),
                        mp.nstr(vm, n=_prec), mp.nstr(am, n=_prec), mp.nstr(duration, n=_prec)))
        return ParabolicCurve()

    log.debug("Successfully fixed x-bound violation")
    return newCurve
    
    
####################################################################################################
    

def _Stretch1D(curve, newDuration, vm, am):
    return Interpolate1DFixedDuration(curve.x0, curve.x1, curve.v0, curve.v1, newDuration, vm, am)


def Interpolate1DFixedDuration(x0, x1, v0, v1, newDuration, vm, am):
    x0 = ConvertFloatToMPF(x0)
    x1 = ConvertFloatToMPF(x1)
    v0 = ConvertFloatToMPF(v0)
    v1 = ConvertFloatToMPF(v1)
    vm = ConvertFloatToMPF(vm)
    am = ConvertFloatToMPF(am)
    newDuration = ConvertFloatToMPF(newDuration)
    log.debug("\nx0 = {0}; x1 = {1}; v0 = {2}; v1 = {3}; vm = {4}; am = {5}; newDuration = {6}".\
              format(mp.nstr(x0, n=_prec), mp.nstr(x1, n=_prec), mp.nstr(v0, n=_prec), mp.nstr(v1, n=_prec),
                     mp.nstr(vm, n=_prec), mp.nstr(am, n=_prec), mp.nstr(newDuration, n=_prec)))
    
    # Check inputs
    assert(vm > zero)
    assert(am > zero)

    if (newDuration < -epsilon):
        log.info("duration = {0} is negative".format(newDuration))
        return ParabolicCurve()
    if (newDuration <= epsilon):
        # Check if this is a stationary trajectory
        if (FuzzyEquals(x0, x1, epsilon) and FuzzyEquals(v0, v1, epsilon)):
            log.info("stationary trajectory")
            ramp0 = Ramp(v0, 0, 0, x0)
            newCurve = ParabolicCurve(ramp0)
            return newCurve
        else:
            log.info("newDuration is too short for any movement to be made")
            return ParabolicCurve()

    # Correct small discrepancies if any
    if (v0 > vm):
        if FuzzyEquals(v0, vm, epsilon):
            v0 = vm
        else:
            log.info("v0 > vm: {0} > {1}".format(v0, vm))
            return ParabolicCurve()
    elif (v0 < -vm):
        if FuzzyEquals(v0, -vm, epsilon):
            v0 = -vm
        else:
            log.info("v0 < -vm: {0} < {1}".format(v0, -vm))
            return ParabolicCurve()
    if (v1 > vm):
        if FuzzyEquals(v1, vm, epsilon):
            v1 = vm
        else:
            log.info("v1 > vm: {0} > {1}".format(v1, vm))
            return ParabolicCurve()
    elif (v1 < -vm):
        if FuzzyEquals(v1, -vm, epsilon):
            v1 = -vm
        else:
            log.info("v1 < -vm: {0} < {1}".format(v1, -vm))
            return ParabolicCurve()

    d = Sub(x1, x0)

    # First assume no velocity bound -> re-interpolated trajectory will have only two ramps.
    # Solve for a0 and a1 (the acceleration of the first and the last ramps).
    #         a0 = A + B/t0
    #         a1 = A + B/(t - t0)
    # where t is the (new) total duration, t0 is the (new) duration of the first ramp, and
    #         A = (v1 - v0)/t
    #         B = (2d/t) - (v0 + v1).
    newDurInverse = mp.fdiv(one, newDuration)
    A = Mul(Sub(v1, v0), newDurInverse)
    B = Sub(Prod([mp.mpf('2'), d, newDurInverse]), Add(v0, v1))

    interval0 = iv.mpf([zero, newDuration]) # initial interval for t0

    # Now consider the interval(s) computed from a0's constraints
    sum1 = Neg(Add(am, A))
    sum2 = Sub(am, A)
    C = mp.fdiv(B, sum1)
    D = mp.fdiv(B, sum2)

    log.debug("\nA = {0}; \nB = {1}; \nC = {2}; \nD = {3}; \nsum1 = {4}; \nsum2 = {5};".\
              format(mp.nstr(A, n=_prec), mp.nstr(B, n=_prec), mp.nstr(C, n=_prec), mp.nstr(D, n=_prec),
                     mp.nstr(sum1, n=_prec), mp.nstr(sum2, n=_prec)))

    if (sum1 > zero):
        # This implied that the duration is too short
        log.debug("the given duration ({0}) is too short.".format(newDuration))
        return ParabolicCurve()
    if (sum2 < zero):
        # This implied that the duration is too short
        log.debug("the given duration ({0}) is too short.".format(newDuration))
        return ParabolicCurve()
    
    if IsEqual(sum1, zero):
        raise NotImplementedError # not yet considered
    elif sum1 > epsilon:
        log.debug("sum1 > 0. This implies that newDuration is too short.")
        return ParabolicCurve()
    else:
        interval1 = iv.mpf([C, inf])
        
    if IsEqual(sum2, zero):
        raise NotImplementedError # not yet considered
    elif sum2 > epsilon:
        interval2 = iv.mpf([D, inf])
    else:
        log.debug("sum2 < 0. This implies that newDuration is too short.")
        return ParabolicCurve()
        
    if Sub(interval2.a, interval1.b) > epsilon or Sub(interval1.a, interval2.b) > epsilon:
        # interval1 and interval2 do not intersect each other
        return ParabolicCurve()    
    # interval3 = interval1 \cap interval2 : valid interval for t0 computed from a0's constraints
    interval3 = iv.mpf([max(interval1.a, interval2.a), min(interval1.b, interval2.b)])
    
    # Now consider the interval(s) computed from a1's constraints
    if IsEqual(sum1, zero):
        raise NotImplementedError # not yet considered
    elif sum1 > epsilon:
        log.debug("sum1 > 0. This implies that newDuration is too short.")
        return ParabolicCurve()
    else:
        interval4 = iv.mpf([Neg(inf), Add(C, newDuration)])
        
    if IsEqual(sum2, zero):
        raise NotImplementedError # not yet considered
    elif sum2 > epsilon:
        interval5 = iv.mpf([Neg(inf), Add(D, newDuration)])
    else:
        log.debug("sum2 < 0. This implies that newDuration is too short.")
        return ParabolicCurve()

    if Sub(interval5.a, interval4.b) > epsilon or Sub(interval4.a, interval5.b) > epsilon:
        log.debug("interval4 and interval5 do not intersect each other")
        return ParabolicCurve()
    # interval6 = interval4 \cap interval5 : valid interval for t0 computed from a1's constraints
    interval6 = iv.mpf([max(interval4.a, interval5.a), min(interval4.b, interval5.b)])

    # import IPython; IPython.embed()

    if Sub(interval3.a, interval6.b) > epsilon or Sub(interval6.a, interval3.b) > epsilon:
        log.debug("interval3 and interval6 do not intersect each other")
        return ParabolicCurve()
    # interval7 = interval3 \cap interval6
    interval7 = iv.mpf([max(interval3.a, interval6.a), min(interval3.b, interval6.b)])

    if Sub(interval0.a, interval7.b) > epsilon or Sub(interval7.a, interval0.b) > epsilon:
        log.debug("interval0 and interval7 do not intersect each other")
        return ParabolicCurve()
    # interval8 = interval0 \cap interval7 : valid interval of t0 when considering all constraints (from a0 and a1)
    interval8 = iv.mpf([max(interval0.a, interval7.a), min(interval0.b, interval7.b)])

    # import IPython; IPython.embed()
    
    # We choose the value t0 (the duration of the first ramp) by selecting the mid point of the
    # valid interval of t0.
    
    t0 = _SolveForT0(A, B, newDuration, interval8)
    if t0 is None:
        # The fancy procedure fails. Now consider no optimization whatsoever.
        # TODO: Figure out why solving fails.
        t0 = mp.convert(interval8.mid) # select the midpoint
        # return ParabolicCurve()
    t1 = Sub(newDuration, t0)

    a0 = Add(A, Mul(mp.fdiv(one, t0), B))
    if (Abs(t1) < epsilon):
        a1 = zero
    else:
        a1 = Add(A, Mul(mp.fdiv(one, Neg(t1)), B))
    assert(Sub(Abs(a0), am) < epsilon) # check if a0 is really below the bound
    assert(Sub(Abs(a1), am) < epsilon) # check if a1 is really below the bound

    # import IPython; IPython.embed()
    
    # Check if the velocity bound is violated    
    vp = Add(v0, Mul(a0, t0))
    if Abs(vp) > vm:
        vmnew = Mul(mp.sign(vp), vm)
        D2 = Prod([pointfive, Sqr(Sub(vp, vmnew)), Sub(mp.fdiv(one, a0), mp.fdiv(one, a1))])
        # print "D2",
        # mp.nprint(D2, n=_prec)
        # print "vmnew",
        # mp.nprint(vmnew, n=_prec)
        A2 = Sqr(Sub(vmnew, v0))
        B2 = Neg(Sqr(Sub(vmnew, v1)))
        t0trimmed = mp.fdiv(Sub(vmnew, v0), a0)
        t1trimmed = mp.fdiv(Sub(v1, vmnew), a1)
        C2 = Sum([Mul(t0trimmed, Sub(vmnew, v0)), Mul(t1trimmed, Sub(vmnew, v1)), Mul(mp.mpf('-2'), D2)])

        log.debug("\nA2 = {0}; \nB2 = {1}; \nC2 = {2}; \nD2 = {3};".format(mp.nstr(A2, n=_prec), mp.nstr(B2, n=_prec), mp.nstr(C2, n=_prec), mp.nstr(D2, n=_prec)))
        
        temp = Prod([A2, B2, B2])
        initguess = mp.sign(temp)*(Abs(temp)**(1./3.))
        root = mp.findroot(lambda x: Sub(Prod([x, x, x]), temp), x0=initguess)

        # import IPython; IPython.embed()
        log.debug("root = {0}".format(mp.nstr(root, n=_prec)))
        a0new = mp.fdiv(Add(A2, root), C2)
        if (Abs(a0new) > Add(am, epsilon)):
            if FuzzyZero(Sub(Mul(C2, a0new), A2), epsilon):
                # The computed a0new is exceeding the bound and its corresponding a1new is
                # zero. Therefore, there is no other way to fix this. This is probably because the
                # given newDuration is less than the minimum duration (x0, x1, v0, v1, vm, am) can
                # get.
                log.debug("abs(a0new) > am and a1new = 0; Cannot fix this case. This happens probably because the given newDuration is too short.")
                return ParabolicCurve()
            
            a0new = Mul(mp.sign(a0new), am)

        if (Abs(a0new) < epsilon):
            a1new = mp.fdiv(B2, C2)
            if (Abs(a1new) > Add(am, epsilon)):
                # Similar to the case above
                log.debug("a0new = 0 and abs(a1new) > am; Cannot fix this case. This happens probably because the given newDuration is too short.")
                return ParabolicCurve()

        else:
            if FuzzyZero(Sub(Mul(C2, a0new), A2), epsilon):
                # import IPython; IPython.embed()
                a1new = 0
            else:
                a1new = Mul(mp.fdiv(B2, C2), Add(one, mp.fdiv(A2, Sub(Mul(C2, a0new), A2))))
                if (Abs(a1new) > Add(am, epsilon)):
                    a1new = Mul(mp.sign(a1new), am)
                    a0new = Mul(mp.fdiv(A2, C2), Add(one, mp.fdiv(B2, Sub(Mul(C2, a1new), B2))))

        if (Abs(a0new) > Add(am, epsilon)) or (Abs(a1new) > Add(am, epsilon)):
            log.warn("Cannot fix acceleration bounds violation")
            return ParabolicCurve()        

        log.debug("\na0 = {0}; \na0new = {1}; \na1 = {2}; \na1new = {3};".format(mp.nstr(a0, n=_prec), mp.nstr(a0new, n=_prec), mp.nstr(a1, n=_prec), mp.nstr(a1new, n=_prec)))
        
        if (Abs(a0new) < epsilon) and (Abs(a1new) < epsilon):
            log.warn("Both accelerations are zero. Should we allow this case?")
            return ParabolicCurve()

        if (Abs(a0new) < epsilon):
            # This is likely because v0 is at the velocity bound
            t1new = mp.fdiv(Sub(v1, vmnew), a1new)
            assert(t1new > 0)
            ramp2 = Ramp(v0, a1new, t1new)

            t0new = Sub(newDuration, t1new)
            assert(t0new > 0)
            ramp1 = Ramp(v0, zero, t0new, x0)
            newCurve = ParabolicCurve([ramp1, ramp2])
            return newCurve

        elif (Abs(a1new) < epsilon):
            t0new = mp.fdiv(Sub(vmnew, v0), a0new)
            assert(t0new > 0)
            ramp1 = Ramp(v0, a0new, t0new, x0)
            
            t1new = Sub(newDuration, t0new)
            assert(t1new > 0)
            ramp2 = Ramp(ramp1.v1, zero, t1new)
            newCurve = ParabolicCurve([ramp1, ramp2])
            return newCurve

        else:
            # No problem with those new accelerations
            # import IPython; IPython.embed()
            t0new = mp.fdiv(Sub(vmnew, v0), a0new)
            if (t0new < 0):
                log.debug("t0new < 0. The given newDuration not achievable with the given bounds")
                return ParabolicCurve()
            
            t1new = mp.fdiv(Sub(v1, vmnew), a1new)
            if (t1new < 0):
                log.debug("t1new < 0. The given newDuration not achievable with the given bounds")
                return ParabolicCurve()
            
            if (Add(t0new, t1new) > newDuration):
                # Final fix. Since we give more weight to acceleration bounds, we make the velocity
                # bound saturated. Therefore, we set vp to vmnew.

                # import IPython; IPython.embed()
                if FuzzyZero(A, epsilon):
                    log.warn("(final fix) A is zero. Don't know how to fix this case")
                    return ParabolicCurve()

                t0new = mp.fdiv(Sub(Sub(vmnew, v0), B), A)
                if (t0new < 0):
                    log.debug("(final fix) t0new is negative")
                    return ParabolicCurve()

                t1new = Sub(newDuration, t0new)
                
                a0new = Add(A, Mul(mp.fdiv(one, t0new), B))
                a1new = Add(A, Mul(mp.fdiv(one, Neg(t1new)), B))
                ramp1 = Ramp(v0, a0new, t0new, x0)
                ramp2 = Ramp(ramp1.v1, a1new, t1new)
                newCurve = ParabolicCurve([ramp1, ramp2])

            else:
                ramp1 = Ramp(v0, a0new, t0new, x0)
                ramp3 = Ramp(ramp1.v1, a1new, t1new)
                ramp2 = Ramp(ramp1.v1, zero, Sub(newDuration, Add(t0new , t1new)))
                newCurve = ParabolicCurve([ramp1, ramp2, ramp3])
                
                # import IPython; IPython.embed()
            
            return newCurve
    else:    
        ramp1 = Ramp(v0, a0, t0, x0)
        ramp2 = Ramp(ramp1.v1, a1, t1)
        newCurve = ParabolicCurve([ramp1, ramp2])
        return newCurve


####################################################################################################
# Utilities

def _CalculateLeastUpperBoundInoperativeInterval(x0, x1, v0, v1, vm, am):
    # All input must already be of mp.mpf type
    d = x1 - x0
    temp1 = Prod([number('2'), Neg(Sqr(am)), Sub(Prod([number('2'), am, d]), Add(Sqr(v0), Sqr(v1)))])
    if temp1 < zero:
        T0 = number('-1')
        T1 = number('-1')
    else:
        term1 = mp.fdiv(Add(v0, v1), am)
        term2 = mp.fdiv(mp.sqrt(temp1), Sqr(am))
        T0 = Add(term1, term2)
        T1 = Sub(term1, term2)

    temp2 = Prod([number('2'), Sqr(am), Add(Prod([number('2'), am, d]), Add(Sqr(v0), Sqr(v1)))])
    if temp2 < zero:
        T2 = number('-1')
        T3 = number('-1')
    else:
        term1 = Neg(mp.fdiv(Add(v0, v1), am))
        term2 = mp.fdiv(mp.sqrt(temp2), Sqr(am))
        T2 = Add(term1, term2)
        T3 = Sub(term1, term2)

    newDuration = max(max(T0, T1), max(T2, T3))
    if newDuration > zero:
        dStraight = Prod([pointfive, Add(v0, v1), newDuration])
        if Sub(d, dStraight) > 0:
            amNew = am
            vmNew = vm
        else:
            amNew = -am
            vmNew = -vm

        # import IPython; IPython.embed()

        vp = Mul(pointfive, Sum([Mul(newDuration, amNew), v0, v1])) # the peak velocity
        if (Abs(vp) > vm):
            dExcess = mp.fdiv(Sqr(Sub(vp, vmNew)), am)
            assert(dExcess > 0)
            deltaTime = mp.fdiv(dExcess, vm)
            newDuration = Add(newDuration, deltaTime)

        log.debug('Calculation successful: T0 = {0}; T1 = {1}; T2 = {2}; T3 = {3}'.format(mp.nstr(T0, n=_prec), mp.nstr(T1, n=_prec), mp.nstr(T2, n=_prec), mp.nstr(T3, n=_prec)))
        
        newDuration = Mul(newDuration, number('1.01')) # add 1% safety bound
        return newDuration
    else:
        if (FuzzyEquals(x0, x1, epsilon) and FuzzyZero(v0, epsilon) and FuzzyZero(v1, epsilon)):
            # t = 0 is actually a correct solution
            newDuration = 0
            return newDuration
        log.debug('Unable to calculate the least upper bound: T0 = {0}; T1 = {1}; T2 = {2}; T3 = {3}'.\
                  format(mp.nstr(T0, n=_prec), mp.nstr(T1, n=_prec), mp.nstr(T2, n=_prec), mp.nstr(T3, n=_prec)))
        return number('-1')
        

def _SolveForT0(A, B, t, tInterval):
    """There are four solutions to the equation (including complex ones). Here we use x instead of t0
    for convenience.
    """

    """
    SolveQuartic(2*A, -4*A*T + 2*B, 3*A*T**2 - 3*B*T, -A*T**3 + 3*B*T**2, -B*T**3)
    """
    if (Abs(A) < epsilon):
        def f(x):
            return Mul(number('2'), B)*x*x*x - Prod([number('3'), B, t])*x*x + Prod([number('3'), B, t, t])*x - Mul(B, mp.power(t, 3))
        sols = [mp.findroot(f, x0=0.5*t)]
    else:
        sols = SolveQuartic(Add(A, A),
                            Add(Prod([number('-4'), A, t]), Mul(number('2'), B)),
                            Sub(Prod([number('3'), A, t, t]), Prod([number('3'), B, t])),
                            Sub(Prod([number('3'), B, t, t]), Mul(A, mp.power(t, 3))),
                            Neg(Mul(B, mp.power(t, 3))))

    realSols = [sol for sol in sols if type(sol) is mp.mpf and sol in tInterval]
    if len(realSols) > 1:
        # I think this should not happen. We should either have one or no solution.
        raise NotImplementedError
    elif len(realSols) == 0:
        return None
    else:
        return realSols[0]


def SolveQuartic(a, b, c, d, e):
    """
    SolveQuartic solves a quartic (fouth order) equation of the form
            ax^4 + bx^3 + cx^2 + dx + e = 0.
    For the detail of formulae presented here, see https://en.wikipedia.org/wiki/Quartic_function
    """
    # Check types
    if type(a) is not mp.mpf:
        a = mp.mpf("{:.15e}".format(a))
    if type(b) is not mp.mpf:
        b = mp.mpf("{:.15e}".format(b))
    if type(c) is not mp.mpf:
        c = mp.mpf("{:.15e}".format(c))
    if type(d) is not mp.mpf:
        d = mp.mpf("{:.15e}".format(d))
    if type(e) is not mp.mpf:
        e = mp.mpf("{:.15e}".format(e))

    """
    # Working code (more readable but probably less precise)
    p = (8*a*c - 3*b*b)/(8*a*a)
    q = (b**3 - 4*a*b*c + 8*a*a*d)/(8*a*a*a)
    delta0 = c*c - 3*b*d + 12*a*e
    delta1 = 2*(c**3) - 9*b*c*d + 27*b*b*e + 27*a*d*d - 72*a*c*e
    Q = mp.nthroot(pointfive*(delta1 + mp.sqrt(delta1*delta1 - 4*mp.power(delta0, 3))), 3)
    S = pointfive*mp.sqrt(-mp.fdiv(mp.mpf('2'), mp.mpf('3'))*p + (one/(3*a))*(Q + delta0/Q))

    x1 = -b/(4*a) - S + pointfive*mp.sqrt(-4*S*S - 2*p + q/S)
    x2 = -b/(4*a) - S - pointfive*mp.sqrt(-4*S*S - 2*p + q/S)
    x3 = -b/(4*a) + S + pointfive*mp.sqrt(-4*S*S - 2*p - q/S)
    x4 = -b/(4*a) + S - pointfive*mp.sqrt(-4*S*S - 2*p - q/S)
    """
    p = mp.fdiv(Sub(Prod([number('8'), a, c]), Mul(number('3'), mp.power(b, 2))), Mul(number('8'), mp.power(a, 2)))
    q = mp.fdiv(Sum([mp.power(b, 3), Prod([number('-4'), a, b, c]), Prod([number('8'), mp.power(a, 2), d])]), Mul(8, mp.power(a, 3)))
    delta0 = Sum([mp.power(c, 2), Prod([number('-3'), b, d]), Prod([number('12'), a, e])])
    delta1 = Sum([Mul(2, mp.power(c, 3)), Prod([number('-9'), b, c, d]), Prod([number('27'), mp.power(b, 2), e]), Prod([number('27'), a, mp.power(d, 2)]), Prod([number('-72'), a, c, e])])
    Q = mp.nthroot(Mul(pointfive, Add(delta1, mp.sqrt(Add(mp.power(delta1, 2), Mul(number('-4'), mp.power(delta0, 3)))))), 3)
    S = Mul(pointfive, mp.sqrt(Mul(mp.fdiv(mp.mpf('-2'), mp.mpf('3')), p) + Mul(mp.fdiv(one, Mul(number('3'), a)), Add(Q, mp.fdiv(delta0, Q)))))

    # log.debug("p = {0}".format(mp.nstr(p, n=_prec)))
    # log.debug("q = {0}".format(mp.nstr(q, n=_prec)))
    # log.debug("delta0 = {0}".format(mp.nstr(delta0, n=_prec)))
    # log.debug("delta1 = {0}".format(mp.nstr(delta1, n=_prec)))
    # log.debug("Q = {0}".format(mp.nstr(Q, n=_prec)))
    # log.debug("S = {0}".format(mp.nstr(S, n=_prec)))

    x1 = Sum([mp.fdiv(b, Mul(number('-4'), a)), Neg(S), Mul(pointfive, mp.sqrt(Sum([Mul(number('-4'), mp.power(S, 2)), Mul(number('-2'), p), mp.fdiv(q, S)])))])
    x2 = Sum([mp.fdiv(b, Mul(number('-4'), a)), Neg(S), Neg(Mul(pointfive, mp.sqrt(Sum([Mul(number('-4'), mp.power(S, 2)), Mul(number('-2'), p), mp.fdiv(q, S)]))))])
    x3 = Sum([mp.fdiv(b, Mul(number('-4'), a)), S, Mul(pointfive, mp.sqrt(Sum([Mul(number('-4'), mp.power(S, 2)), Mul(number('-2'), p), Neg(mp.fdiv(q, S))])))])
    x4 = Sum([mp.fdiv(b, Mul(number('-4'), a)), S, Neg(Mul(pointfive, mp.sqrt(Sum([Mul(number('-4'), mp.power(S, 2)), Mul(number('-2'), p), Neg(mp.fdiv(q, S))]))))])
    
    return [x1, x2, x3, x4]
