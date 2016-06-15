from mpmath import mp, iv, arange
import numpy as np
import matplotlib.pyplot as plt
import bisect

_prec = 500
epsilon = mp.mpf('1e-100')

mp.dps = _prec
iv.dps = _prec
pointfive = mp.mpf('0.5')
zero = mp.mpf('0')

"""
ramp.py

For testing and verifying results of precise interpolation.
"""

# Aliases (alphabetically ordered)
def Abs(a):
    return mp.fabs(a)

def Add(a, b):
    return mp.fadd(a, b, exact=True)

def IsEqual(a, b):
    # It is better to check equality using this function than using == when a or b or both are a
    # product of some operations involving division.    
    return Abs(Sub(a, b)) < epsilon

def Mul(a, b):
    return mp.fmul(a, b, exact=True)

def Neg(a):
    return mp.fneg(a, exact=True)

def Prod(A):
    assert(len(A) > 0)
    return mp.fprod(A)

def Sub(a, b):
    return mp.fsub(a, b, exact=True)

def Sum(A):
    assert(len(A) > 0)
    return mp.fsum(A)

def ConvertFloatToMPF(a):
    if type(a) is not mp.mpf:
        return mp.mpf("{:.15e}".format(a))
    else:
        return a

def ConvertFloatArrayToMPF(a):
    a_ = np.asarray([mp.mpf("{:.15e}".format(x)) for x in a if type(x) is not mp.mpf])
    return a_


class Ramp(object):
    """
    v0 : the initial velocity
    a  : the acceleration
    duration  : the duration
    v1 : the final velocity
    d  : the total displacement 'done' by this ramp (i.e. x1 = x0 + d)
    """
    def __init__(self, v0, a, dur, x0=zero):
        if type(dur) is not mp.mpf:
            dur = mp.mpf("{:.15e}".format(dur))
        assert(dur > -epsilon)

        # Check types
        if type(x0) is not mp.mpf:
            x0 = mp.mpf("{:.15e}".format(x0))
        if type(v0) is not mp.mpf:
            v0 = mp.mpf("{:.15e}".format(v0))
        if type(a) is not mp.mpf:
            a = mp.mpf("{:.15e}".format(a))
        
        self.x0 = x0
        self.v0 = v0
        self.a = a
        self.duration = dur
        
        self.v1 = Add(self.v0, Mul(self.a, self.duration))
        self.d = Prod([pointfive, Add(self.v0, self.v1), self.duration])
   

    def UpdateDuration(self, newDur):
        if type(newDur) is not mp.mpf:
            newDur = mp.mpf("{:.15e}".format(newDur))
        assert(newDur > -epsilon)

        self.duration = newDur
        self.v1 = Add(self.v0, Mul(self.a, self.duration))
        self.d = Prod([pointfive, Add(self.v0, self.v1), self.duration])


    def EvalPos(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)        

        d_incr = Mul(t, Add(self.v0, Prod([pointfive, t, self.a])))
        return Add(self.x0, d_incr)

    
    def EvalVel(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)
        
        return Add(self.v0, Mul(self.a, t))
        

    def EvalAcc(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)

        return self.a


    def PlotVel(self, t0=0, fignum=None, **kwargs):
        if fignum is not None:
            plt.figure(fignum)

        line = plt.plot([t0, t0 + self.duration], [self.v0, self.v1], **kwargs)[0]
        plt.show(False)
        return line


    def PlotAcc(self, t0=0, fignum=None, **kwargs):
        if fignum is not None:
            plt.figure(fignum)

        line = plt.plot([t0, t0 + self.duration], [self.a, self.a], **kwargs)[0]
        plt.show(False)
        return line
# end class Ramp


class ParabolicCurve(object):
    """
    ramps    : a list of all ramps
    v0       : the initial velocity of this curve (v0 = ramps[0].v0)
    x0       : the initial displacement of this curve (x0 = ramps[0].x0)
    d        : the total displacement 'done' by this curve (i.e. x1 = x0 + d)
    duration : the total duration of this curve
    switchpointsList : a list of all switch points (nSwitchpoints = nRamps + 1, i.e. we have 2 switch points for each ramp)
    """
    def __init__(self, ramps=[]):
        self.switchpointsList = [] # a list of all switch points, including ones at t = 0 and t = duration
        dur = zero
        d = zero
        self.ramps = []
        
        if len(ramps) == 0:
            self.isEmpty = True
            self.x0 = zero
            self.v0 = zero
            self.switchpointsList = []
            self.duration = dur
            self.d = d
        else:
            self.ramps = ramps[:] # copy all the ramps
            self.isEmpty = False
            self.SetInitialValue(self.ramps[0].x0) # set self.x0
            self.v0 = self.ramps[0].v0
            
            self.switchpointsList.append(dur)
            for ramp in self.ramps:
                dur = Add(dur, ramp.duration)
                self.switchpointsList.append(dur)
                d = Add(d, ramp.d)
            self.duration = dur
            self.d = d


    def __getitem__(self, index):
        return self.ramps[index]


    def __len__(self):
        return len(self.ramps)


    def Append(self, curve):
        if self.isEmpty:
            if not curve.isEmpty:
                self.ramps = curve.ramps[:]
                self.x0 = curve.x0
                self.v0 = curve.v0
                self.switchpointsList = curve.switchpointsList[:]
                self.isEmpty = False
                self.duration = curve.duration
                self.d = curve.d
            else:
                # do nothing
                pass
        else:
            dur = self.duration
            d = self.d
            ramps_ = curve[:]
            for ramp in ramps_:
                self.ramps.append(ramp)
                # Update displacement
                self.ramps[-1].x0 = d
                d = Add(d, self.ramps[-1].d)                
                dur = Add(dur, self.ramps[-1].duration)
                self.switchpointsList.append(dur)

            self.duration = dur
            self.d = d


    def Merge(self):
        """
        Merge merges consecutive ramp(s) if they have the same acceleration
        """
        if not self.isEmpty:
            aCur = self.ramps[0].a
            nmerged = 0 # the number of merged ramps
            for i in xrange(1, len(self.ramps)):
                j = i - nmerged
                if Abs(Sub(self.ramps[j].a, aCur)) < epsilon:
                    # merge ramps
                    redundantRamp = self.ramps.pop(j)
                    newDur = Add(self.ramps[j - 1].duration, redundantRamp.duration)
                    self.ramps[j - 1].UpdateDuration(newDur)

                    # merge switchpointsList
                    self.switchpointsList.pop(j)

                    nmerged += 1
                else:
                    aCur = self.ramps[j].a


    def _FindRampIndex(self, t):
        # t = mp.mpf("{:.15e}".format(t))
        # assert(t > -epsilon)
        # assert(t < self.duration + epsilon)

        if t < epsilon:
            i = 0
            remainder = zero
        else:
            i = bisect.bisect_left(self.switchpointsList, t) - 1
            remainder = Sub(t, self.switchpointsList[i])
        return i, remainder


    def EvalPos(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)

        i, remainder = self._FindRampIndex(t)
        return self.ramps[i].EvalPos(remainder)


    def EvalVel(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)

        i, remainder = self._FindRampIndex(t)
        return self.ramps[i].EvalVel(remainder)


    def EvalAcc(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)

        i, remainder = self._FindRampIndex(t)
        return self.ramps[i].EvalAcc(remainder)


    def SetInitialValue(self, x0):
        if type(x0) is not mp.mpf:
            x0 = mp.mpf("{:.15e}".format(x0))
        self.x0 = x0
        newx0 = x0
        for ramp in self.ramps:
            ramp.x0 = newx0
            newx0 = Add(newx0, ramp.d)
    

    def Trim(self, deltaT):
        """
        Trim trims the curve such that it has the duration of self.duration - deltaT.

        Trim also takes care of where to trim out the deltaT. However, normally we should not have any problem
        since deltaT is expected to be very small. This function is aimed to be used when combining Curves to
        get ParabolicCurvesND.

        Return True if the operation is successful, False otherwise.
        """
        if type(deltaT) is not mp.mpf:
            dt = mp.mpf("{:.15e}".format(deltaT))
        else:
            dt = deltaT
        if dt > self.duration:
            return False # cannot trim
        if Abs(dt) < epsilon:
            return True # no trimming needed
        
        if dt < self.ramps[-1].duration:
            # trim the last ramp
            newDur = Sub(self.ramps[-1].duration, dt)
            self.ramps[-1].UpdateDuration(newDur)
            return True
        else:
            # have not decided what to do here yet. This is not likely to happen, though, since
            # deltaT is expected to be very small.
            return False


    # Visualization
    def PlotPos(self, fignum=None, color='g', dt=0.01, lw=2):
        tVect = arange(0, self.duration, dt)
        if tVect[-1] < self.duration:
            tVect = np.append(tVect, self.duration)
            
        xVect = [self.EvalPos(t) for t in tVect]
        if fignum is not None:
            plt.figure(fignum)
        plt.plot(tVect, xVect, color=color, linewidth=lw)
        plt.show(False)


    def PlotVel(self, fignum=None, color=None, lw=2, **kwargs):
        if fignum is not None:
            plt.figure(fignum)

        t0 = zero
        for ramp in self.ramps:
            if color is None:
                line = ramp.PlotVel(t0=t0, fignum=fignum, linewidth=lw, **kwargs)
                color = line.get_color()
            else:
                line = ramp.PlotVel(t0=t0, fignum=fignum, color=color, linewidth=lw, **kwargs)
            t0 += ramp.duration
        plt.show(False)
        return line


    def PlotAcc(self, fignum=None, color=None, lw=2, **kwargs):
        if fignum is not None:
            plt.figure(fignum)
            
        t0 = zero
        for ramp in self.ramps:
            if color is None:
                line = ramp.PlotAcc(t0=t0, fignum=fignum, linewidth=lw, **kwargs)
                color = line.get_color()
            else:
                line = ramp.PlotAcc(t0=t0, fignum=fignum, color=color, linewidth=lw, **kwargs)
            t0 += ramp.duration
        plt.show(False)
        return line
# end class ParabolicCurve


class ParabolicCurvesND(object):
    """
    """
    def __init__(self, curves=[]):
        if (len(curves) == 0):
            self.curves = []
            self.isEmpty = True
            self.x0Vect = None
            self.v0Vect = None
            self.dVect = None
            self.ndof = 0
            self.switchpointsList = []
            self.duration = zero
        else:
            # Check first if every curve in curves has the same duration.
            # (if necessary) Trim all curve to have the same duration.
            curves_ = curves[:]
            minDur = curves_[0].duration
            for curve in curves_[1:]:
                assert(Abs(Sub(curve.duration, minDur)) < epsilon)
                minDur = min(minDur, curve.duration)
            for curve in curves_:
                deltaT = Sub(curve.duration, minDur)
                if curve.Trim(deltaT):
                    continue
                else:
                    # Cannot trim the curve
                    assert(False)

            # Now all curves have the same duration
            self.isEmpty = False
            self.duration = minDur
            self.curves = curves_
            self.ndof = len(self.curves)
            self.x0Vect = np.asarray([curve.x0 for curve in self.curves])
            self.v0Vect = np.asarray([curve.v0 for curve in self.curves])
            self.dVect = np.asarray([curve.d for curve in self.curves])

            # Create a list of switch points
            switchpointsList = curves[0].switchpointsList[:]
            for curve in self.curves[1:]:
                for s in curve.switchpointsList:
                    switchpointsList.insert(bisect.bisect_left(switchpointsList, s), s)

            self.switchpointsList = []
            if len(switchpointsList) > 0:
                self.switchpointsList.append(switchpointsList[0])
                for s in switchpointsList[1:]:
                    if Sub(s, self.switchpointsList[-1]) > epsilon:
                        # Add only non-redundant switch points
                        self.switchpointsList.append(s)


    def __getitem__(self, index):
        return self.curves[index]


    def __len__(self):
        return len(self.curves)


    def Append(self, curvesnd):
        if self.isEmpty:
            if len(curvesnd) > 0:
                self.duration = curvesnd.duration
                self.curves = curvesnd[:]
                self.ndof = len(self.curves)
                self.x0Vect = np.asarray([curve.x0 for curve in self.curves])
                self.v0Vect = np.asarray([curve.v0 for curve in self.curves])
                self.dVect = np.asarray([curve.d for curve in self.curves])
                self.switchpointsList = curvesnd.switchpointsList[:]
                self.isEmpty = False
        else:
            assert(self.ndof == curvesnd.ndof)
            originalDur = self.duration
            self.duration = Add(self.duration, curvesnd.duration)
            for (i, curve) in enumerate(curvesnd):
                self.curves[i].Append(curve)
                self.dVect[i] = Add(self.dVect[i], curve.d)

            newSwitchpoints = [Add(s, originalDur) for s in curvesnd.switchpointsList]
            self.switchpointsList.extend(newSwitchpoints)


    def SetInitialValues(self, x0Vect):
        x0Vect_ = ConvertFloatArrayToMPF(x0Vect)
        self.x0Vect = np.array(x0Vect_)
        for (i, curve) in enumerate(self.curves):
            curve.SetInitialValue(self.x0Vect[i])
            

    def EvalPos(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)
        
        xVect = [curve.EvalPos(t) for curve in self.curves]
        return np.asarray(xVect)


    def EvalVel(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)
        
        vVect = [curve.EvalVel(t) for curve in self.curves]
        return np.asarray(vVect)


    def EvalAcc(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)
        
        aVect = [curve.EvalAcc(t) for curve in self.curves]
        return np.asarray(aVect)


    # Visualization
    def PlotPos(self, fignum='Displacement Profiles', includingSW=False, dt=0.005):
        plt.figure(fignum)

        tVect = arange(0, self.duration, dt)
        if tVect[-1] < self.duration:
            tVect = np.append(tVect, self.duration)

        xVect = [self.EvalPos(t) for t in tVect]
        plt.plot(tVect, xVect, linewidth=2)
        handle = ['joint {0}'.format(i + 1) for i in xrange(self.ndof)]
        plt.legend(handle)

        if includingSW:
            ax = plt.gca().axis()
            for s in self.switchpointsList:
                plt.plot([s, s], [ax[2], ax[3]], 'r', linewidth=1)
        plt.show(False)
        

    def PlotVel(self, fignum='Velocity Profile', includingSW=False, **kwargs):
        plt.figure(fignum)
        plt.hold(True)

        lines = []
        for curve in self.curves:
            lines.append(curve.PlotVel(fignum=fignum, **kwargs))

        handles = ['joint {0}'.format(i + 1) for i in xrange(self.ndof)]
        plt.legend(lines, handles)

        if includingSW:
            ax = plt.gca().axis()
            for s in self.switchpointsList:
                plt.plot([s, s], [ax[2], ax[3]], 'r', linewidth=1)
        plt.show(False)
        

    def PlotAcc(self, fignum='Acceleration Profiles', **kwargs):
        plt.figure(fignum)
        plt.hold(True)

        lines = []
        for curve in self.curves:
            lines.append(curve.PlotAcc(fignum=fignum, **kwargs))

        handles = ['joint {0}'.format(i + 1) for i in xrange(self.ndof)]
        plt.legend(lines, handles)
        plt.show(False)
# end class ParabolicCurvesND

    
################################################################################
# Utilities (for checking)
_printInfo = True
def FuzzyEquals(a, b, eps):
    return Abs(Sub(a, b)) < eps

def CheckRamp(rampIn, vm, am, **kwargs):
    x0passed = True
    x1passed = True
    v0passed = True
    v1passed = True
    vboundpassed = True
    aboundpassed = True
    dpassed = True
    if kwargs.has_key('x0'):
        x0 = ConvertFloatToMPF(kwargs['x0'])
        x0passed = FuzzyEquals(rampIn.x0, x0, epsilon)
        if _printInfo and (not x0passed):
            print "CheckRamp: check x0 failed"
    if kwargs.has_key('x1'):
        x1 = ConvertFloatToMPF(kwargs['x1'])
        x1passed = FuzzyEquals(rampIn.Eval(rampIn.duration), kwargs['x1'], epsilon)
        if _printInfo and (not x1passed):
            print "CheckRamp: check x1 failed"
    if kwargs.has_key('v0'):
        v0 = ConvertFloatToMPF(kwargs['v0'])
        v0passed = FuzzyEquals(rampIn.v0, kwargs['v0'], epsilon)
        if _printInfo and (not v0passed):
            print "CheckRamp: check v0 failed"
    if kwargs.has_key('v1'):
        v0 = ConvertFloatToMPF(kwargs['v1'])
        v1passed = FuzzyEquals(rampIn.v0, kwargs['v1'], epsilon)
        if _printInfo and (not v1passed):
            print "CheckRamp: check v1 failed"
    if kwargs.has_key('d'):
        v0 = ConvertFloatToMPF(kwargs['d'])
        dpassed = FuzzyEquals(rampIn.v0, kwargs['d'], epsilon)
        if _printInfo and (not dpassed):
            print "CheckRamp: check d failed"
    vm = ConvertFloatToMPF(vm)
    vboundpassed = ((Abs(rampIn.v0) < vm + epsilon) and (Abs(rampIn.v1) < vm + epsilon))
    if _printInfo and (not vboundpassed):
        print "CheckRamp: check v-bound failed"
    am = ConvertFloatToMPF(am)
    aboundpassed = (Abs(rampIn.a) < am + epsilon)
    if _printInfo and (not aboundpassed):
        print "CheckRamp: check a-bound failed"
    result = x0passed and x1passed and v0passed and v1passed and vboundpassed and aboundpassed and dpassed
    return result


def CheckParabolicCurve(curve, vm, am, **kwargs):
    allRampsPassed = True
    x0passed = True
    x1passed = True
    v0passed = True
    durationpassed = True
    dpassed = True
    nRamps = len(curve)
    print "{0:.15e}, {1:.15e}".format(vm, am)
    for i in xrange(nRamps):
        # Check velocity continuity as well as velocity and acceleration bounds
        if i == 0:
            if kwargs.has_key('v0'):
                v0 = ConvertFloatToMPF(kwargs['v0'])
                allRampsPassed = allRampsPassed and CheckRamp(curve[i], vm, am, v0=v0)
            else:
                allRampsPassed = allRampsPassed and CheckRamp(curve[i], vm, am)
        else:
            allRampsPassed = allRampsPassed and CheckRamp(curve[i], vm, am, v0=curve[i - 1].v1)
        if _printInfo and (not allRampsPassed):
            print "CheckParabolicCurve: Check Ramp {0} failed".format(i)

    if kwargs.has_key('x0'):
        # Check initial condition
        x0 = ConvertFloatToMPF(kwargs['x0'])
        x0passed = FuzzyEquals(curve[0].x0, x0, epsilon) and FuzzyEquals(curve.x0, x0, epsilon)
        if _printInfo and (not x0passed):
            print "CheckParabolicCurve: check x0 failed"
    if kwargs.has_key('x1'):
        # Check initial condition
        x1 = ConvertFloatToMPF(kwargs['x1'])
        x1passed = (FuzzyEquals(curve[0].EvalPos(curve[0].duration), x0, epsilon) and
                    FuzzyEquals(curve.EvalPos(curve.duration), x0, epsilon))
        if _printInfo and (not x1passed):
            print "CheckParabolicCurve: check x1 failed"
    if kwargs.has_key('v0'):
        # Check initial velocity (of this Curve)
        v0passed = FuzzyEquals(curve.v0, v0, epsilon)
        if _printInfo and (not v0passed):
            print "CheckParabolicCurve: check v0 failed"
    if kwargs.has_key('duration'):
        # Check duration
        duration = ConvertFloatToMPF(kwargs['duration'])
        durationpassed = FuzzyEquals(curve.duration, duration, epsilon)
        if _printInfo and (not durationpassed):
            print "CheckParabolicCurve: check duration failed"
    if kwargs.has_key('d'):
        # Check displacement
        d = ConvertFloatToMPF(kwargs['d'])
        dpassed = FuzzyEquals(curve.d, d, epsilon)
        if _printInfo and (not dpassed):
            print "CheckParabolicCurve: check displacement failed"

    result = allRampsPassed and x0passed and v0passed and durationpassed and dpassed
    return result


def CheckParabolicCurvesND(curvesnd, vmVect, amVect, **kwargs):
    allCurvesPassed = True
    durationpassed = True

    newkwargs = dict() # for passing to CheckParabolicCurve
    hasx0Vect = kwargs.has_key('x0Vect')
    if hasx0Vect:
        x0Vect = ConvertFloatArrayToMPF(kwargs['x0Vect'])
    hasx1Vect = kwargs.has_key('x1Vect')
    if hasx1Vect:
        x1Vect = ConvertFloatArrayToMPF(kwargs['x1Vect'])
    hasdVect = kwargs.has_key('dVect')
    if hasdVect:
        dVect = ConvertFloatArrayToMPF(kwargs['dVect'])

    ndof = curvesnd.ndof
    for i in xrange(ndof):
        if hasx0Vect:
            newkwargs['x0'] = x0Vect[i]
        if hasx1Vect:
            newkwargs['x1'] = x1Vect[i]
        if hasdVect:
            newkwargs['d'] = dVect[i]
        allCurvesPassed = allCurvesPassed and CheckParabolicCurve(curvesnd[i], vmVect[i], amVect[i], kwargs=newkwargs)
        if _printInfo and (not allCurvesPassed):
            print "CheckParabolicCurvesND: DOF {0} failed".format(i)
        if not allCurvesPassed:
            return False
        durationpassed = durationpassed and FuzzyEquals(curvesnd.duration, curvesnd[i].duration, epsilon)
        if _printInfo and (not durationpassed):
            print "CheckParabolicCurvesND: DOF {0} duration failed".format(i)

    result = allCurvesPassed and durationpassed
    return result
