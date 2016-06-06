from mpmath import mp, arange
import numpy as np

from ramp import Ramp, ParabolicCurve, ParabolicCurvesND
from ramp import zero, pointfive, epsilon
from ramp import Add, Abs, Mul, Neg, Prod, Sub, Sum

inf = mp.inf
one = mp.mpf('1')

def IsEqual(a, b):
    # It is better to check equality using this function than using == when a or b or both are a
    # product of some operations involving division.    
    return Abs(Sub(a, b)) < epsilon

def ConvertFloatArrayToMPF(a):
    a_ = np.asarray([mp.mpf(str(x)) for x in a])
    return a_


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
        delta = mp.mpf(str(delta))

    vMin = inf # the tightest velocity bound
    aMin = inf # the tightest acceleration bound
    for i in xrange(ndof):
        if not IsEqual(x1Vect[i], x0Vect[i]):
            vMin = min(vMin, vmVect[i]/Abs(dVect[i]))
            aMin = min(aMin, amVect[i]/Abs(dVect[i]))

    assert(vMin < inf)
    assert(aMin < inf)

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

    curvesnd = ParabolicCurvesND(curves)

    return curvesnd


def InterpolateArbitraryVelND(x0Vect, x1Vect, v0Vect, v1Vect, vmVect, amVect, delta=zero):
    """Interpolate a trajectory connecting two waypoints, (x0Vect, v0Vect) and (x1Vect, v1Vect).

    """
    ndof = len(x0Vect)
    assert(ndof == len(x1Vect))
    assert(ndof == len(vmVect))
    assert(ndof == len(amVect))

    # Convert all vector elements into mp.mpf (if necessary)
    x0Vect_ = ConvertFloatArrayToMPF(x0Vect)
    x1Vect_ = ConvertFloatArrayToMPF(x1Vect)
    v0Vect_ = ConvertFloatArrayToMPF(v0Vect)
    v1Vect_ = ConvertFloatArrayToMPF(v1Vect)
    vmVect_ = ConvertFloatArrayToMPF(vmVect)
    amVect_ = ConvertFloatArrayToMPF(amVect)
    
    dVect = x1Vect - x0Vect

    if type(delta) is not mp.mpf:
        delta = mp.mpf(str(delta))

    # First independently interpolate each DOF to find out the slowest one.
    curves = []
    maxDuration = zero
    maxIndex = 0
    for i in xrange(ndof):
        if delta == zero:
            curve = Interpolate1D(x0Vect[i], x1Vect[i], v0Vect[i], v1Vect[i], vmVect[i], amVect[i])
        else:
            raise NotImplementedError
        if curve.duration > maxDuration:
            maxDuration = curve.duration
            maxIndex = i        

    curvesnd = ReinterpolateNDFixedDuration(curves, vmVect, amVect, maxIndex, delta)
    return curvesnd


def ReinterpolateNDFixedDuration(curves, vmVect, amVect, maxIndex, delta=zero):
    pass


####################################################################################################
# Single DOF


def Interpolate1D(x0, x1, v0, v1, vm, am, delta=zero):
    # Check types
    if type(x0) is not mp.mpf:
        x0 = mp.mpf(str(x0))
    if type(x1) is not mp.mpf:
        x1 = mp.mpf(str(x1))
    if type(v0) is not mp.mpf:
        v0 = mp.mpf(str(v0))
    if type(v1) is not mp.mpf:
        v1 = mp.mpf(str(v1))
    if type(vm) is not mp.mpf:
        vm = mp.mpf(str(vm))
    if type(am) is not mp.mpf:
        am = mp.mpf(str(am))

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
        x0 = mp.mpf(str(x0))
    if type(x1) is not mp.mpf:
        x1 = mp.mpf(str(x1))
    if type(v0) is not mp.mpf:
        v0 = mp.mpf(str(v0))
    if type(v1) is not mp.mpf:
        v1 = mp.mpf(str(v1))
    if type(am) is not mp.mpf:
        am = mp.mpf(str(am))

    # Check inputs
    assert(am > zero)

    # Check for an appropriate acceleration direction of the first ramp
    d = Sub(x1, x0)
    dv = Sub(v1, v0)
    sumVSqr = Add(v0**2, v1**2)
    
    if Abs(dv) < epsilon:
        if Abs(d) < epsilon:
            # Stationary ramp
            ramp0 = Ramp(zero, zero, zero, x0)
            return ParabolicCurve([ramp0])

        else:
            dStraight = zero
    else:    
        dStraight = mp.fdiv(sumVSqr, Prod([2, mp.sign(dv), am]))
    
    if IsEqual(d, dStraight):
        # With the given distance, v0 and v1 can be directly connected using max/min
        # acceleration. Here the resulting profile has only one ramp.
        a0 = mp.sign(dv) * am
        ramp0 = Ramp(v0, a0, mp.fdiv(dv, a0), x0)
        return ParabolicCurve([ramp0])

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
        vm = mp.mpf(str(vm))

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

    h = Sub(Abs(vp), vm)
    t = mp.fdiv(h, Abs(curve[0].a))
    
    ramp0, ramp1 = curve
    h = Sub(Abs(vp), vm)
    t = mp.fdiv(h, Abs(ramp0.a))

    ramps = []
    if IsEqual(Abs(ramp0.v0), vm):
        assert(IsEqual(ramp0.duration, t)) # check soundness
    else:
        newRamp0 = Ramp(ramp0.v0, ramp0.a, Sub(ramp0.duration, t), ramp0.x0)
        ramps.append(newRamp0)
        
    nom = h**2
    denom = Mul(Abs(curve[0].a), vm)
    newRamp1 = Ramp(vm, zero, Sum([t, t, mp.fdiv(nom, denom)]), newRamp0.x0)
    ramps.append(newRamp1)

    if IsEqual(Abs(ramp1.v1), vm):
        assert(IsEqual(ramp1.duration, t)) # check soundness
    else:
        newRamp2 = Ramp(vm, ramp1.a, Sub(ramp1.duration, t))
        ramps.append(newRamp2)

    return ParabolicCurve(ramps)


