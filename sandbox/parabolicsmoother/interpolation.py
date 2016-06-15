from mpmath import mp, iv, arange
import numpy as np

from ramp import Ramp, ParabolicCurve, ParabolicCurvesND
from ramp import zero, pointfive, epsilon
from ramp import Add, Abs, IsEqual, Mul, Neg, Prod, Sub, Sum

inf = mp.inf
one = mp.mpf('1')
number = mp.mpf

def ConvertFloatArrayToMPF(a):
    a_ = np.asarray([mp.mpf("{:.16e}".format(x)) for x in a if type(x) is not mp.mpf])
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
        delta = mp.mpf("{:.16e}".format(delta))

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

    for (i, curve) in enumerate(curves):
        curve.SetInitialValue(x0Vect[i])        
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
        delta = mp.mpf("{:.16e}".format(delta))

    # First independently interpolate each DOF to find out the slowest one.
    curves = []
    maxDuration = zero
    maxIndex = 0
    for i in xrange(ndof):
        if delta == zero:
            curve = Interpolate1D(x0Vect_[i], x1Vect_[i], v0Vect_[i], v1Vect_[i], vmVect_[i], amVect_)[i]
        else:
            raise NotImplementedError
        if curve.duration > maxDuration:
            maxDuration = curve.duration
            maxIndex = i        

    curvesnd = ReinterpolateNDFixedDuration(curves, vmVect_, amVect_, maxIndex, delta)
    return curvesnd


def ReinterpolateNDFixedDuration(curves, vmVect, amVect, maxIndex, delta=zero):
    ndof = len(curves)
    assert(ndof == len(vmVect))
    assert(ndof == len(amVect))
    assert(maxIndex < ndof)

    # Convert all vector elements into mp.mpf (if necessary)
    vmVect_ = ConvertFloatArrayToMPF(vmVect)
    amVect_ = ConvertFloatArrayToMPF(amVect)

    newDuration = curves[maxIndex].duration
    newCurves = []
    for (idof, curve) in enumerate(curves):
        if idof == maxIndex:
            newCurves.append(curve)
            continue

        stretchedCurve = _Stretch1D(curve, newDuration, vmVect[idof], amVect[idof])
        if stretchedCurve.isEmpty:
            print 'ReinterpolateNDFixedDuration: dof {0} failed'.format(idof)
            return ParabolicCurvesND()
        else:
            newCurves.append(stretchedCurve)
    assert(len(newCurves) == ndof)
    return ParabolicCurvesND(newCurves)


def _Stretch1D(curve, newDuration, vm, am):
    # Check types
    if type(newDuration) is not mp.mpf:
        newDuration = mp.mpf("{:.16e}".format(newDuration))
    if type(vm) is not mp.mpf:
        vm = mp.mpf("{:.16e}".format(vm))
    if type(am) is not mp.mpf:
        am = mp.mpf("{:.16e}".format(am))

    # Check inputs
    assert(newDuration > curve.duration)
    assert(vm > zero)
    assert(am > zero)

    v0 = curve[0].v0
    v1 = curve[-1].v1
    d = curve.d

    # First assume no velocity bound -> re-interpolated trajectory will have only two ramps.
    # Solve for a1 and a2 (the acceleration of the first and the last ramps).
    #         a1 = A + B/t1
    #         a2 = A + B/(t - t1)
    # where t is the (new) total duration, t1 is the (new) duration of the first ramp, and
    #         A = (v1 - v0)/t
    #         B = (2d/t) - (v0 + v1).
    newDurInverse = mp.fdiv(one, newDuration)
    A = Mul(Sub(v1, v0), newDurInverse)
    B = Sub(Prod([mp.mpf('2'), d, newDurInverse]), Add(v0, v1))

    interval0 = iv.mpf([zero, newDuration]) # initial interval for t1

    # Now consider the interval(s) computed from a1's constraints
    sum1 = Neg(Add(am, A))
    sum2 = Sub(am, A)
    C = mp.fdiv(B, sum1)
    D = mp.fdiv(B, sum2)
    if IsEqual(sum1, zero):
        raise NotImplementedError # not yet considered
    elif sum1 > epsilon:
        interval1 = iv.mpf([Neg(inf), C])
    else:
        interval1 = iv.mpf([C, inf])
        
    if IsEqual(sum2, zero):
        raise NotImplementedError # not yet considered
    elif sum2 > epsilon:
        interval2 = iv.mpf([D, inf])
    else:
        interval2 = iv.mpf([Neg(inf), D])

    if Sub(interval2.a, interval1.b) > epsilon or Sub(interval1.a, interval2.b) > epsilon:
        # interval1 and interval2 do not intersect each other
        return ParabolicCurve()    
    # interval3 = interval1 \cap interval2 : valid interval for t1 computed from a1's constraints
    interval3 = iv.mpf([max(interval1.a, interval2.a), min(interval1.b, interval2.b)])
    
    # Now consider the interval(s) computed from a2's constraints
    if IsEqual(sum1, zero):
        raise NotImplementedError # not yet considered
    elif sum1 > epsilon:
        interval4 = iv.mpf([Add(C, newDuration), inf])
    else:
        interval4 = iv.mpf([Neg(inf), Add(C, newDuration)])
        
    if IsEqual(sum2, zero):
        raise NotImplementedError # not yet considered
    elif sum2 > epsilon:
        interval5 = iv.mpf([Neg(inf), Add(D, newDuration)])
    else:
        interval5 = iv.mpf([Add(D, newDuration), inf])

    if Sub(interval5.a, interval4.b) > epsilon or Sub(interval4.a, interval5.b) > epsilon:
        # interval4 and interval5 do not intersect each other
        return ParabolicCurve()
    # interval6 = interval4 \cap interval5 : valid interval for t1 computed from a2's constraints
    interval6 = iv.mpf([max(interval4.a, interval5.a), min(interval4.b, interval5.b)])

    if Sub(interval3.a, interval6.b) > epsilon or Sub(interval6.a, interval3.b) > epsilon:
        # interval3 and interval6 do not intersect each other
        return ParabolicCurve()
    # interval7 = interval3 \cap interval6
    interval7 = iv.mpf([max(interval3.a, interval6.a), min(interval3.b, interval6.b)])

    if Sub(interval0.a, interval7.b) > epsilon or Sub(interval7.a, interval0.b) > epsilon:
        # interval0 and interval7 do not intersect each other
        return ParabolicCurve()
    # interval8 = interval0 \cap interval7 : valid interval of t1 when considering all constraints (from a1 and a2)
    interval8 = iv.mpf([max(interval0.a, interval7.a), min(interval0.b, interval7.b)])

    # from IPython.terminal import embed
    # ipshell = embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
    
    # We choose the value t1 (the duration of the first ramp) by selecting the mid point of the
    # valid interval of t1.
    
    # t1 = mp.convert(interval8.mid)
    t1 = _SolveForT1(A, B, newDuration, interval8)
    if t1 is None:
        return ParabolicCurve()
    t2 = Sub(newDuration, t1)

    a1 = Add(A, Mul(mp.fdiv(one, t1), B))
    a2 = Add(A, Mul(mp.fdiv(one, Neg(t2)), B))
    assert(Sub(Abs(a1), am) < epsilon) # check if a1 is really below the bound
    assert(Sub(Abs(a2), am) < epsilon) # check if a2 is really below the bound

    ramp1 = Ramp(v0, a1, t1, curve.x0)
    ramp2 = Ramp(ramp1.v1, a2, t2)
    newCurve = ParabolicCurve([ramp1, ramp2])
    return newCurve


def _SolveForT1(A, B, t, tInterval):
    """There are four solutions to the equation (including complex ones). Here we use x instead of t1
    for convenience.
    """

    """
    SolveQuartic(2*A, -4*A*T + 2*B, 3*A*T**2 - 3*B*T, -A*T**3 + 3*B*T**2, -B*T**3)
    """
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
    

####################################################################################################
# Single DOF


def Interpolate1D(x0, x1, v0, v1, vm, am, delta=zero):
    # Check types
    if type(x0) is not mp.mpf:
        x0 = mp.mpf("{:.16e}".format(x0))
    if type(x1) is not mp.mpf:
        x1 = mp.mpf("{:.16e}".format(x1))
    if type(v0) is not mp.mpf:
        v0 = mp.mpf("{:.16e}".format(v0))
    if type(v1) is not mp.mpf:
        v1 = mp.mpf("{:.16e}".format(v1))
    if type(vm) is not mp.mpf:
        vm = mp.mpf("{:.16e}".format(vm))
    if type(am) is not mp.mpf:
        am = mp.mpf("{:.16e}".format(am))

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
        x0 = mp.mpf("{:.16e}".format(x0))
    if type(x1) is not mp.mpf:
        x1 = mp.mpf("{:.16e}".format(x1))
    if type(v0) is not mp.mpf:
        v0 = mp.mpf("{:.16e}".format(v0))
    if type(v1) is not mp.mpf:
        v1 = mp.mpf("{:.16e}".format(v1))
    if type(am) is not mp.mpf:
        am = mp.mpf("{:.16e}".format(am))

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
        vm = mp.mpf("{:.16e}".format(vm))

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


####################################################################################################
# Utilities

def SolveQuartic(a, b, c, d, e):
    """
    SolveQuartic solves a quartic (fouth order) equation of the form
            ax^4 + bx^3 + cx^2 + dx + e = 0.
    For the detail of formulae presented here, see https://en.wikipedia.org/wiki/Quartic_function
    """
    # Check types
    if type(a) is not mp.mpf:
        a = mp.mpf("{:.16e}".format(a))
    if type(b) is not mp.mpf:
        b = mp.mpf("{:.16e}".format(b))
    if type(c) is not mp.mpf:
        c = mp.mpf("{:.16e}".format(c))
    if type(d) is not mp.mpf:
        d = mp.mpf("{:.16e}".format(d))
    if type(e) is not mp.mpf:
        e = mp.mpf("{:.16e}".format(e))

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

    print "p = {0}".format(p)
    print "q = {0}".format(q)
    print "delta0 = {0}".format(delta0)
    print "delta1 = {0}".format(delta1)
    print "Q = {0}".format(Q)
    print "S = {0}".format(S)

    x1 = Sum([mp.fdiv(b, Mul(number('-4'), a)), Neg(S), Mul(pointfive, mp.sqrt(Sum([Mul(number('-4'), mp.power(S, 2)), Mul(number('-2'), p), mp.fdiv(q, S)])))])
    x2 = Sum([mp.fdiv(b, Mul(number('-4'), a)), Neg(S), Neg(Mul(pointfive, mp.sqrt(Sum([Mul(number('-4'), mp.power(S, 2)), Mul(number('-2'), p), mp.fdiv(q, S)]))))])
    x3 = Sum([mp.fdiv(b, Mul(number('-4'), a)), S, Mul(pointfive, mp.sqrt(Sum([Mul(number('-4'), mp.power(S, 2)), Mul(number('-2'), p), Neg(mp.fdiv(q, S))])))])
    x4 = Sum([mp.fdiv(b, Mul(number('-4'), a)), S, Neg(Mul(pointfive, mp.sqrt(Sum([Mul(number('-4'), mp.power(S, 2)), Mul(number('-2'), p), Neg(mp.fdiv(q, S))]))))])
    
    return [x1, x2, x3, x4]
