from ramp import *
import mpmath as mp
import numpy as np
import bisect
from copy import deepcopy


class ParabolicPath(object):

    def __init__(self, curvesndVect=[]):
        self.curvesndVect = []
        self.duratoin = zero
        
        for curvesnd in curvesndVect:
            self.AppendParabolicCurvesND(curvesnd)


    def IsEmpty(self):
        return len(self.curvesndVect) == 0
    
        
    def AppendParabolicCurvesND(self, curvesnd):
        if (self.IsEmpty()):
            self.curvesndVect.append(deepcopy(curvesnd))
            self.duration = curvesnd.duration

            self.mainSwitchpoints = [0, self.duration]

            self.x0Vect = np.array(curvesnd.x0Vect)
            self.x1Vect = np.array(curvesnd.x1Vect)
            self.v0Vect = np.array(curvesnd.v0Vect)
            self.v1Vect = np.array(curvesnd.v1Vect)
        else:
            self.curvesndVect.append(deepcopy(curvesnd))
            self.curvesndVect[-1].SetInitialValues(self.x1Vect)
            self.x1Vect = self.curvesndVect[-1].x1Vect
            self.v1Vect = self.curvesndVect[-1].v1Vect

            self.duration = Add(self.duration, curvesnd.duration)
            self.mainSwitchpoints.append(self.duration)

        return
    

    def AppendParabolicPath(self, parabolicpath):
        if (self.IsEmpty()):
            self.ReConstruct(parabolicpath.curvesndVect)
            return
        else:
            for curvesnd in parabolicpath.curvesndVect:
                self.AppendParabolicCurvesND(curvesnd)
        return

    
    def EvalPos(self, t):
        index, remainder = self.FindParabolicCurvesNDIndex(t)
        return self.curvesndVect[index].EvalPos(remainder)

    
    def EvalVel(self, t):
        index, remainder = self.FindParabolicCurvesNDIndex(t)
        return self.curvesndVect[index].EvalVel(remainder)
    
    
    def EvalAcc(self, t):
        index, remainder = self.FindParabolicCurvesNDIndex(t)
        return self.curvesndVect[index].EvalAcc(remainder)

    
    def FindParabolicCurvesNDIndex(self, t):
        t = ConvertFloatToMPF(t)
        assert(t >= -epsilon)
        assert(t <= Add(self.duration, epsilon))

        if (t <= 0):
            index = 0
            remainder = 0
        elif (t >= self.duration):
            index = len(self.curvesndVect) - 1
            remainder = self.curvesndVect[-1].duration
        else:
            index = bisect.bisect_left(self.mainSwitchpoints, t) - 1
            remainder = Sub(t, self.mainSwitchpoints[index])
        return index, remainder            

    
    def Reconstruct(self, curvesndVect):
        assert(len(curvesndVect) > 0)

        self.curvesndVect = []
        for curvesnd in curvesndVect:
            self.AppendParabolicCurvesND(curvesnd)
        return


    def ReplaceSegment(self, t0, t1, curvesndVectIn):
        i0, rem0 = self.FindParabolicCurvesNDIndex(t0)
        i1, rem1 = self.FindParabolicCurvesNDIndex(t1)

        newCurvesNDVect = []
        for i in xrange(i0):
            newCurvesNDVect.append(self.curvesndVect[i])

        tempCurvesND = ParabolicCurvesND()
        tempCurvesND.Initialize(self.curvesndVect[i0].curves)
        tempCurvesND.TrimBack(rem0)
        if (tempCurvesND.duration > 0):
            newCurvesNDVect.append(deepcopy(tempCurvesND))

        for curvesnd in curvesndVectIn:
            newCurvesNDVect.append(deepcopy(curvesnd))

        tempCurvesND.Initialize(self.curvesndVect[i1].curves)
        tempCurvesND.TrimFront(rem1)
        if (tempCurvesND.duration > 0):
            newCurvesNDVect.append(deepcopy(tempCurvesND))

        for i in xrange(i1 + 1, len(self.curvesndVect)):
            newCurvesNDVect.append(self.curvesndVect[i])

        self.Reconstruct(newCurvesNDVect)
        return


def ConvertDynamicPathStringIntoParabolicPath(dynamicpathstring):
    dynamicpathstring = dynamicpathstring.strip()
    data = dynamicpathstring.split("\n")
    ndof = int(data[0])
    nlines = ndof + 2 # the number of lines containing the data for 1 ParabolicRampND

    nParabolicRampND = len(data)/(nlines)

    parabolicpath = ParabolicPath()
    
    for iramp in xrange(nParabolicRampND):
        curoffset = iramp*nlines
        curves = [ParabolicCurve() for _ in xrange(ndof)]
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
        curvesnd = ParabolicCurvesND(curves)
        parabolicpath.AppendParabolicCurvesND(curvesnd)

    return parabolicpath
