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
inf = mp.inf

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
    return Abs(Sub(a, b)) <= epsilon

def Mul(a, b):
    return mp.fmul(a, b, exact=True)

def Neg(a):
    return mp.fneg(a, exact=True)

def Prod(A):
    assert(len(A) > 0)
    return mp.fprod(A)

def Sqr(a):
    return mp.fmul(a, a, exact=True)

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

def ConvertFloatArrayToMPF(A):
    A_ = np.asarray([ConvertFloatToMPF(x) for x in A])
    return A_


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
        assert(dur >= -epsilon)

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
        self.x1 = Add(self.x0, self.d)
        

    def UpdateDuration(self, newDur):
        if type(newDur) is not mp.mpf:
            newDur = mp.mpf("{:.15e}".format(newDur))
        assert(newDur >= -epsilon)

        self.duration = newDur
        self.v1 = Add(self.v0, Mul(self.a, self.duration))
        self.d = Prod([pointfive, Add(self.v0, self.v1), self.duration])
        self.x1 = Add(self.x0, self.d)
        

    def EvalPos(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t >= -epsilon)
        assert(t <= self.duration + epsilon)        

        d_incr = Mul(t, Add(self.v0, Prod([pointfive, t, self.a])))
        return Add(self.x0, d_incr)

    
    def EvalVel(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t >= -epsilon)
        assert(t <= self.duration + epsilon)
        
        return Add(self.v0, Mul(self.a, t))
        

    def EvalAcc(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t >= -epsilon)
        assert(t <= self.duration + epsilon)

        return self.a

    def GetPeaks(self):
        if (FuzzyZero(self.a, epsilon)):
            if self.v0 > 0:
                xmin = self.x0
                xmax = self.EvalPos(self.duration)
            else:
                xmin = self.EvalPos(self.duration)
                xmax = self.x0
            return [xmin, xmax]
        elif (self.a > 0):
            xmin = self.x0
            xmax = self.EvalPos(self.duration)
        else:
            xmin = self.EvalPos(self.duration)
            xmax = self.x0            
            
        tDeflection = Neg(mp.fdiv(self.v0, self.a))
        if (tDeflection <= 0) or (tDeflection >= self.duration):
            return [xmin, xmax]
        
        xDeflection = self.EvalPos(tDeflection)
        xmax = max(xmax, xDeflection)
        xmin = min(xmin, xDeflection)
        return [xmin, xmax]


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


    def __repr__(self):
        bmin, bmax = self.GetPeaks()
        return "x0 = {0}; x1 = {1}; v0 = {2}; v1 = {3}; a = {4}; duration = {5}; bmin = {6}; bmax = {7}".\
            format(mp.nstr(self.x0, n=_prec), mp.nstr(self.EvalPos(self.duration), n=_prec),
                   mp.nstr(self.v0, n=_prec), mp.nstr(self.v1, n=_prec), mp.nstr(self.a, n=_prec),
                   mp.nstr(self.duration, n=_prec), mp.nstr(bmin, n=_prec), mp.nstr(bmax, n=_prec))
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
            self.x1 = zero
            self.v0 = zero
            self.v1 = zero
            self.switchpointsList = []
            self.duration = dur
            self.d = d
        else:
            self.ramps = ramps[:] # copy all the ramps
            self.isEmpty = False
            self.v0 = self.ramps[0].v0
            self.v1 = self.ramps[-1].v1
            
            self.switchpointsList.append(dur)
            for ramp in self.ramps:
                dur = Add(dur, ramp.duration)
                self.switchpointsList.append(dur)
                d = Add(d, ramp.d)
            self.duration = dur
            self.d = d

            self.SetInitialValue(self.ramps[0].x0) # set self.x0


    def __getitem__(self, index):
        return self.ramps[index]


    def __len__(self):
        return len(self.ramps)


    def Append(self, curve):
        if self.isEmpty:
            if not curve.isEmpty:
                self.ramps = curve.ramps[:]
                self.x0 = curve.x0
                self.x1 = curve.x1
                self.v0 = curve.v0
                self.v1 = curve.v1
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
                self.ramps[-1].x0 = self.ramps[-2].x1
                d = Add(d, self.ramps[-1].d)                
                dur = Add(dur, self.ramps[-1].duration)
                self.switchpointsList.append(dur)

            self.v1 = self.ramps[-1].v1
            self.duration = dur
            self.d = d
            self.x1 = Add(self.x0, self.d)


    def Merge(self, prec=epsilon):
        """
        Merge merges consecutive ramp(s) if they have the same acceleration
        """
        if not self.isEmpty:
            if Abs((Abs(mp.log10(prec)) - (Abs(mp.floor(mp.log10(prec)))))) < Abs((Abs(mp.log10(prec)) - (Abs(mp.ceil(mp.log10(prec)))))):
                precexp = mp.floor(mp.log10(prec))
            else:
                precexp = mp.ceil(mp.log10(prec))
                
            aCur = self.ramps[0].a
            nmerged = 0 # the number of merged ramps
            for i in xrange(1, len(self.ramps)):
                j = i - nmerged
                if (Abs(self.ramps[j].a) > 1):
                    if Abs((Abs(mp.log10(Abs(self.ramps[j].a))) - (Abs(mp.floor(mp.log10(Abs(self.ramps[j].a))))))) < Abs((Abs(mp.log10(Abs(self.ramps[j].a))) - (Abs(mp.ceil(mp.log10(Abs(self.ramps[j].a))))))):
                        threshold = 10**(precexp + mp.floor(mp.log10(Abs(self.ramps[j].a))) + 1)
                    else:
                        threshold = 10**(precexp + mp.ceil(mp.log10(Abs(self.ramps[j].a))) + 1)
                else:
                    threshold = 10**(precexp)
                if Abs(Sub(self.ramps[j].a, aCur)) < threshold:
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
        assert(t >= -epsilon)
        assert(t <= self.duration + epsilon)

        i, remainder = self._FindRampIndex(t)
        return self.ramps[i].EvalPos(remainder)


    def EvalVel(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t >= -epsilon)
        assert(t <= self.duration + epsilon)

        i, remainder = self._FindRampIndex(t)
        return self.ramps[i].EvalVel(remainder)


    def EvalAcc(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t >= -epsilon)
        assert(t <= self.duration + epsilon)

        i, remainder = self._FindRampIndex(t)
        return self.ramps[i].EvalAcc(remainder)


    def GetPeaks(self):
        xmin = inf
        xmax = -inf

        for ramp in self.ramps:
            [bmin, bmax] = ramp.GetPeaks()
            if bmin < xmin:
                xmin = bmin
            if bmax > xmax:
                xmax = bmax

        assert(xmin < inf)
        assert(xmax > -inf)
        return [xmin, xmax]


    def SetInitialValue(self, x0):
        if type(x0) is not mp.mpf:
            x0 = mp.mpf("{:.15e}".format(x0))
        self.x0 = x0
        newx0 = x0
        for ramp in self.ramps:
            ramp.x0 = newx0
            newx0 = Add(newx0, ramp.d)
        self.x1 = Add(self.x0, self.d)
    

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
            self.v1 = self.ramps[-1].v1
            return True
        else:
            # have not decided what to do here yet. This is not likely to happen, though, since
            # deltaT is expected to be very small.
            return False


    # Visualization
    def PlotPos(self, fignum=None, color='g', dt=0.01, lw=2, includingSW=False):
        tVect = arange(0, self.duration, dt)
        if tVect[-1] < self.duration:
            tVect = np.append(tVect, self.duration)
            
        xVect = [self.EvalPos(t) for t in tVect]
        if fignum is not None:
            plt.figure(fignum)
        plt.plot(tVect, xVect, color=color, linewidth=lw)

        if includingSW:
            ax = plt.gca().axis()
            for s in self.switchpointsList:
                plt.plot([s, s], [ax[2], ax[3]], 'r', linewidth=1)
                
        plt.show(False)


    def PlotVel(self, fignum=None, color=None, lw=2, includingSW=False, **kwargs):
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

        if includingSW:
            ax = plt.gca().axis()
            for s in self.switchpointsList:
                plt.plot([s, s], [ax[2], ax[3]], 'r', linewidth=1)
            
        plt.show(False)
        return line


    def PlotAcc(self, fignum=None, color=None, lw=2, **kwargs):
        if fignum is not None:
            plt.figure(fignum)
            
        t0 = zero
        prevacc = self.ramps[0].a
        for ramp in self.ramps:
            if color is None:
                line = ramp.PlotAcc(t0=t0, fignum=fignum, linewidth=lw, **kwargs)
                color = line.get_color()
            else:
                line = ramp.PlotAcc(t0=t0, fignum=fignum, color=color, linewidth=lw, **kwargs)
            plt.plot([t0, t0], [prevacc, ramp.a], color=color, linewidth=lw, **kwargs)
            t0 += ramp.duration
            prevacc = ramp.a
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
            self.x1Vect = None
            self.v0Vect = None
            self.v1Vect = None
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
            self.x1Vect = np.asarray([curve.x1 for curve in self.curves])
            self.v0Vect = np.asarray([curve.v0 for curve in self.curves])
            self.v0Vect = np.asarray([curve.v1 for curve in self.curves])
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
                self.x1Vect = np.asarray([curve.x1 for curve in self.curves])
                self.v0Vect = np.asarray([curve.v0 for curve in self.curves])
                self.v1Vect = np.asarray([curve.v1 for curve in self.curves])
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

            newSwitchpoints = [Add(s, originalDur) for s in curvesnd.switchpointsList[1:]]
            self.switchpointsList.extend(newSwitchpoints)
            self.x1Vect = np.asarray([curve.x1 for curve in self.curves])


    def SetInitialValues(self, x0Vect):
        x0Vect_ = ConvertFloatArrayToMPF(x0Vect)
        self.x0Vect = np.array(x0Vect_)
        for (i, curve) in enumerate(self.curves):
            curve.SetInitialValue(self.x0Vect[i])
        self.x1Vect = np.asarray([Add(x0, d) for (x0, d) in zip(self.x0Vect, self.dVect)])
            

    def EvalPos(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t >= -epsilon)
        assert(t <= self.duration + epsilon)
        
        xVect = [curve.EvalPos(t) for curve in self.curves]
        return np.asarray(xVect)


    def EvalVel(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t >= -epsilon)
        assert(t <= self.duration + epsilon)
        
        vVect = [curve.EvalVel(t) for curve in self.curves]
        return np.asarray(vVect)


    def EvalAcc(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf("{:.15e}".format(t))
        assert(t >= -epsilon)
        assert(t <= self.duration + epsilon)
        
        aVect = [curve.EvalAcc(t) for curve in self.curves]
        return np.asarray(aVect)


    def GetPeaks(self):
        xmin = np.zeros(self.ndof)
        xmax = np.zeros(self.ndof)
        for i in xrange(self.ndof):
            xmin[i], xmax[i] = self.curves[i].GetPeaks()
        return [xmin, xmax]
        

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
def VectToString(A):
    A_ = ConvertFloatArrayToMPF(A)
    separator = ""
    s = "["
    for a in A_:
        s += separator
        s += mp.nstr(a, n=_prec)
        separator = ", "
    return s

_printInfo = False#True
def FuzzyEquals(a, b, eps):
    return Abs(Sub(a, b)) < eps

def FuzzyZero(a, eps):
    return Abs(a) < eps

class ParabolicCheckReturn:
    Normal = 0
    NegativeDuration = 1
    XBoundViolated = 2
    VBoundViolated = 3
    ABoundViolated = 4
    VDiscontinuous = 5
    XDiscrepancy = 6
    VDiscrepancy = 7
    DurationDiscrepancy = 8


def CheckRamp(ramp, xmin, xmax, vm, am):
    xmin = ConvertFloatToMPF(xmin)
    xmax = ConvertFloatToMPF(xmax)
    vm = ConvertFloatToMPF(vm)
    am = ConvertFloatToMPF(am)

    bmin, bmax = ramp.GetPeaks()
    if (bmin < Sub(xmin, epsilon)) or (bmax > Add(xmax, epsilon)):
        return ParabolicCheckReturn.XBoundViolated

    if (Abs(ramp.v0) > Add(vm, epsilon)) or (Abs(ramp.v1) > Add(vm, epsilon)):
        return ParabolicCheckReturn.VBoundViolated
    
    if (Abs(ramp.a) > Add(am, epsilon)):
        return ParabolicCheckReturn.ABoundViolated
    
    return ParabolicCheckReturn.Normal


def CheckRamps(rampsVect, xmin, xmax, vm, am):
    xmin = ConvertFloatToMPF(xmin)
    xmax = ConvertFloatToMPF(xmax)
    vm = ConvertFloatToMPF(vm)
    am = ConvertFloatToMPF(am)
    
    ret = CheckRamp(rampsVect[0], xmin, xmax, vm, am)
    if not (ret == ParabolicCheckReturn.Normal):
        return ret
    
    for i in xrange(1, len(rampsVect)):
        if not FuzzyEquals(rampsVect[i - 1].v1, rampsVect[i].v0, epsilon):
            return ParabolicCheckReturn.VDiscrepancy
        ret = CheckRamp(rampsVect[i], xmin, xmax, vm, am)
        if not (ret == ParabolicCheckReturn.Normal):
            return ret
    return ParabolicCheckReturn.Normal


def CheckParabolicCurve(curve, xmin, xmax, vm, am, x0, x1, v0, v1):
    vm = ConvertFloatToMPF(vm)
    am = ConvertFloatToMPF(am)
    v0 = ConvertFloatToMPF(v0)
    v1 = ConvertFloatToMPF(v1)
    x0 = ConvertFloatToMPF(x0)
    x1 = ConvertFloatToMPF(x1)
    
    ret = CheckRamps(curve.ramps, xmin, xmax, vm, am)
    if not (ret == ParabolicCheckReturn.Normal):
        return ret
    
    # Check boundary conditions
    if not FuzzyEquals(curve.v0, curve.ramps[0].v0, epsilon):
        return ParabolicCheckReturn.VDiscrepancy
    if not FuzzyEquals(curve.v0, v0, epsilon):
        return ParabolicCheckReturn.VDiscrepancy
    if not FuzzyEquals(curve.v1, curve.ramps[-1].v1, epsilon):
        return ParabolicCheckReturn.VDiscrepancy
    if not FuzzyEquals(curve.v1, v1, epsilon):
        return ParabolicCheckReturn.VDiscrepancy
    if not FuzzyEquals(curve.x0, curve.ramps[0].x0, epsilon):
        return ParabolicCheckReturn.XDiscrepancy
    if not FuzzyEquals(curve.x0, x0, epsilon):
        return ParabolicCheckReturn.XDiscrepancy
    if not FuzzyEquals(curve.EvalPos(curve.duration), x1, epsilon):
        return ParabolicCheckReturn.XDiscrepancy
    if not FuzzyEquals(curve.d, x1 - x0, epsilon):
        return ParabolicCheckReturn.XDiscrepancy
    return ParabolicCheckReturn.Normal


def CheckParabolicCurvesND(curvesnd, xminVect, xmaxVect, vmVect, amVect, x0Vect, x1Vect, v0Vect, v1Vect):
    xminVect_ = ConvertFloatArrayToMPF(xminVect)
    xmaxVect_ = ConvertFloatArrayToMPF(xmaxVect)
    vmVect_ = ConvertFloatArrayToMPF(vmVect)
    amVect_ = ConvertFloatArrayToMPF(amVect)
    v0Vect_ = ConvertFloatArrayToMPF(v0Vect)
    v1Vect_ = ConvertFloatArrayToMPF(v1Vect)
    x0Vect_ = ConvertFloatArrayToMPF(x0Vect)
    x1Vect_ = ConvertFloatArrayToMPF(x1Vect)
    for i in xrange(curvesnd.ndof):
        ret = CheckParabolicCurve(curvesnd.curves[i], xminVect_[i], xmaxVect_[i], vmVect_[i], amVect_[i], x0Vect_[i], x1Vect_[i], v0Vect_[i], v1Vect_[i])
        if not (ret == ParabolicCheckReturn.Normal):
            return ret
        if not FuzzyEquals(curvesnd.duration, curvesnd.curves[i].duration, epsilon):
            return ParabolicCheckReturn.DurationDiscrepancy
    return ParabolicCheckReturn.Normal
    

def DynamicPathStringToParabolicCurvesND(dynamicpathstring):
    dynamicpathstring = dynamicpathstring.strip()
    data = dynamicpathstring.split("\n")
    ndof = int(data[0])
    nlines = ndof + 2 # the number of lines containing the data for 1 ParabolicRampND

    curves = [ParabolicCurve() for _ in xrange(ndof)]
    nParabolicRampND = len(data)/(nlines)

    for iramp in xrange(nParabolicRampND):
        curoffset = iramp*nlines
        for idof in xrange(ndof):
            ramp1ddata = data[curoffset + 2 + idof]
            x0, v0, x1, v1, a1, v, a2, tswitch1, tswitch2, ttotal = [mp.mpf(x) for x in ramp1ddata.split(" ")]
            ramps = []
            ramp0 = Ramp(v0, a1, tswitch1, x0)
            if ramp0.duration > epsilon:
                ramps.append(ramp0)
            ramp1 = Ramp(v, 0, tswitch2 - tswitch1, ramp0.x1)
            if ramp1.duration > epsilon:
                ramps.append(ramp1)
            ramp2 = Ramp(v, a2, ttotal - tswitch2, ramp1.x1)
            if ramp2.duration > epsilon:
                ramps.append(ramp2)
            assert(len(ramps) > 0)
            curve = ParabolicCurve(ramps)
            
            curves[idof].Append(curve)

    return ParabolicCurvesND(curves)


def ParabolicPathStringToParabolicCurvesND(parabolicpathstring):
    """
    Data format:

             /   ndof
             |   duration
             |   curve1
    chunk 1 <   curve2
             |   curve3
             |   :
             |   :
             \   curvendof
             /   ndof
             |   duration
             |   curve1
    chunk 2 <   curve2
             |   curve3
             |   :
             |   :
             \   curvendof
     :
     :
    
    chunk N

    For each ParabolicCurve:
      ramp 0      ramp 1            ramp M
    v0 a t x0 | v0 a t x0 | ... | v0 a t x0
    
    """
    parabolicpathstring = parabolicpathstring.strip()
    rawdata = parabolicpathstring.split("\n")
    ndof = int(rawdata[0])
    nlines_chunk = 2 + ndof

    nchunks = len(rawdata)/nlines_chunk

    curvesnd = ParabolicCurvesND()
    for ichunk in xrange(nchunks):
        curves = []
        for idof in xrange(ndof):
            curvedata = rawdata[(ichunk*nlines_chunk) + 2 + idof]
            curvedata = curvedata.strip().split(" ")
            curve = ParabolicCurve()
            nramps = len(curvedata)/4
            for iramp in xrange(nramps):
                v, a, t, x0 = [float(dummy) for dummy in curvedata[(iramp*4):((iramp + 1)*4)]]
                ramp = Ramp(v, a, t, x0)
                nextCurve = ParabolicCurve([ramp])
                curve.Append(nextCurve)
            curves.append(curve)
        nextCurvesND = ParabolicCurvesND(curves)
        curvesnd.Append(nextCurvesND)

    return curvesnd
