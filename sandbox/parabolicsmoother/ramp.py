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


class Ramp(object):
    """
    """
    def __init__(self, v0, a, dur, x0=zero):
        if type(dur) is not mp.mpf:
            dur = mp.mpf(str(dur))
        assert(dur > -epsilon)

        # Check types
        if type(x0) is not mp.mpf:
            x0 = mp.mpf(str(x0))
        if type(v0) is not mp.mpf:
            v0 = mp.mpf(str(v0))
        if type(a) is not mp.mpf:
            a = mp.mpf(str(a))
        
        self.x0 = x0
        self.v0 = v0
        self.a = a
        self.duration = dur
        
        self.v1 = Add(self.v0, Mul(self.a, self.duration))
        self.d = Prod([pointfive, Add(self.v0, self.v1), self.duration])
   

    def UpdateDuration(self, newDur):
        if type(newDur) is not mp.mpf:
            newDur = mp.mpf(str(newDur))
        assert(newDur > -epsilon)

        self.duration = newDur
        self.v1 = Add(self.v0, Mul(self.a, self.duration))
        self.d = Prod([pointfive, Add(self.v0, self.v1), self.duration])


    def EvalPos(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf(str(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)        

        d_incr = Mul(t, Add(self.v0, Prod([pointfive, t, self.a])))
        return Add(self.x0, d_incr)

    
    def EvalVel(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf(str(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)
        
        return Add(self.v0, Mul(self.a, t))
        

    def EvalAcc(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf(str(t))
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
    """
    def __init__(self, ramps=[]):
        self.switchpointsList = [] # a list of all switch points, including ones at t = 0 and t = duration
        dur = zero
        d = zero
        self.ramps = []
        
        if len(ramps) == 0:
            self.isEmpty = True
            self.x0 = zero
            self.switchpointsList = []
            self.duration = dur
            self.d = d
        else:
            ramps_ = ramps[:]
            self.isEmpty = False
            self.x0 = ramps_[0].x0
            self.switchpointsList.append(dur)
            for ramp in ramps_:
                self.ramps.append(ramp)
                # Update displacement
                self.ramps[-1].x0 = d
                d = Add(d, self.ramps[-1].d)
                
                dur = Add(dur, self.ramps[-1].duration)
                self.switchpointsList.append(dur)

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
        # t = mp.mpf(str(t))
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
            t = mp.mpf(str(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)

        i, remainder = self._FindRampIndex(t)
        return self.ramps[i].EvalPos(remainder)


    def EvalVel(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf(str(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)

        i, remainder = self._FindRampIndex(t)
        return self.ramps[i].EvalVel(remainder)


    def EvalAcc(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf(str(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)

        i, remainder = self._FindRampIndex(t)
        return self.ramps[i].EvalAcc(remainder)


    def Trim(self, deltaT):
        """
        Trim trims the curve such that it has the duration of self.duration - deltaT.

        Trim also takes care of where to trim out the deltaT. However, normally we should not have any problem
        since deltaT is expected to be very small. This function is aimed to be used when combining Curves to
        get ParabolicCurvesND.

        Return True if the operation is successful, False otherwise.
        """
        if type(deltaT) is not mp.mpf:
            dt = mp.mpf(str(deltaT))
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
                self.switchpointsList = curvesnd.switchpointsList[:]
                self.isEmpty = False
        else:
            assert(self.ndof == curvesnd.ndof)
            originalDur = self.duration
            self.duration = Add(self.duration, curvesnd.duration)
            for (i, curve) in enumerate(curvesnd):
                self.curves[i].Append(curve)

            newSwitchpoints = [Add(s, originalDur) for s in curvesnd.switchpointsList]
            self.switchpointsList.extend(newSwitchpoints)


    def EvalPos(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf(str(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)
        
        xVect = [curve.EvalPos(t) for curve in self.curves]
        return np.asarray(xVect)


    def EvalVel(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf(str(t))
        assert(t > -epsilon)
        assert(t < self.duration + epsilon)
        
        vVect = [curve.EvalVel(t) for curve in self.curves]
        return np.asarray(vVect)


    def EvalAcc(self, t):
        if type(t) is not mp.mpf:
            t = mp.mpf(str(t))
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

    
        
