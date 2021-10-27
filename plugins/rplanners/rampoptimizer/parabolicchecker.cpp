// -*- coding: utf-8 -*-
// Copyright (C) 2016 Puttichai Lertkultanon & Rosen Diankov
//
// This program is free software: you can redistribute it and/or modify it under the terms of the
// GNU Lesser General Public License as published by the Free Software Foundation, either version 3
// of the License, or at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
// even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
#include "parabolicchecker.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

void _GetPeaks(dReal x0, dReal x1, dReal v0, dReal v1, dReal a, dReal t, dReal& bmin, dReal& bmax)
{
    if( FuzzyZero(a, g_fRampEpsilon) ) {
        if( v0 > 0 ) {
            bmin = x0;
            bmax = x1;
        }
        else {
            bmin = x1;
            bmax = x0;
        }
        return;
    }

    dReal curMin, curMax;
    if( x0 > x1 ) {
        curMin = x1;
        curMax = x0;
    }
    else {
        curMin = x0;
        curMax = x1;
    }

    dReal tDeflection = -v0/a; // the time when velocity crosses zero
    if( (tDeflection <= 0) || (tDeflection >= t) ) {
        bmin = curMin;
        bmax = curMax;
        return;
    }

    dReal xDeflection = x0 + 0.5*v0*tDeflection;
    bmin = Min(curMin, xDeflection);
    bmax = Max(curMax, xDeflection);
    return;
}

ParabolicCheckReturn CheckSegment(dReal x0, dReal x1, dReal v0, dReal v1, dReal a, dReal t, dReal xmin, dReal xmax, dReal vm, dReal am)
{
    if( t < -g_fRampEpsilon ) {
        RAVELOG_WARN_FORMAT("PCR_NegativeDuration: duration = %.15e", t);
        return PCR_NegativeDuration;
    }
    if( !FuzzyEquals(v1, v0 + a*t, g_fRampEpsilon) ) {
        dReal v1_ = v0 + a*t;
        RAVELOG_WARN_FORMAT("PCR_VDiscrepancy: v1 = %.15e; computed v1 = %.15e; diff = %.15e", v1%v1_%(v1 - v1_));
        RAVELOG_WARN_FORMAT("Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; a = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", x0%x1%v0%v1%a%t%xmin%xmax%vm%am);
        return PCR_VDiscrepancy;
    }
    if( !FuzzyEquals(x1, x0 + t*(v0 + 0.5*a*t), g_fRampEpsilon) ) {
        dReal x1_ = x0 + t*(v0 + 0.5*a*t);
        RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: x1 = %.15e; computed x1 = %.15e; diff = %.15e", x1%x1_%(x1 - x1_));
        RAVELOG_WARN_FORMAT("Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; a = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", x0%x1%v0%v1%a%t%xmin%xmax%vm%am);
        return PCR_XDiscrepancy;
    }
    if( xmin == g_fRampInf && xmax == g_fRampInf ) {
        return PCR_Normal;
    }
    dReal bmin, bmax;
    _GetPeaks(x0, x1, v0, v1, a, t, bmin, bmax);
    if( (bmin < xmin - g_fRampEpsilon) || (bmax > xmax + g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_XBoundViolated: xmin = %.15e; bmin = %.15e; diff@min = %.15e; xmax = %.15e; bmax = %.15e; diff@max = %.15e", xmin%bmin%(xmin - bmin)%xmax%bmax%(bmax - xmax));
        RAVELOG_WARN_FORMAT("Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; a = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", x0%x1%v0%v1%a%t%xmin%xmax%vm%am);
        return PCR_XBoundViolated;
    }
    if( (Abs(v0) > vm + g_fRampEpsilon) || (Abs(v1) > vm + g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_VBoundViolated: vm = %.15e; v0 = %.15e; v1 = %.15e; diff@v0 = %.15e; diff@v1 = %.15e", vm%v0%v1%(Abs(v0) - vm)%(Abs(v1) - vm));
        RAVELOG_WARN_FORMAT("Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; a = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", x0%x1%v0%v1%a%t%xmin%xmax%vm%am);
        return PCR_VBoundViolated;
    }
    if( Abs(a) > am + g_fRampEpsilon ) {
        RAVELOG_WARN_FORMAT("PCR_ABoundViolated: am = %.15e; a = %.15e; diff = %.15e", am%a%(Abs(a) - am));
        RAVELOG_WARN_FORMAT("Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; a = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", x0%x1%v0%v1%a%t%xmin%xmax%vm%am);
        return PCR_ABoundViolated;
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckRamp(const Ramp& ramp, dReal xmin, dReal xmax, dReal vm, dReal am)
{
    return CheckSegment(ramp.x0, ramp.x1, ramp.v0, ramp.v1, ramp.a, ramp.duration, xmin, xmax, vm, am);
}

ParabolicCheckReturn CheckRamps(const std::vector<Ramp>& ramps, dReal xmin, dReal xmax, dReal vm, dReal am, dReal x0, dReal x1, dReal v0, dReal v1)
{
    // Check the first ramp
    if( !FuzzyEquals(ramps[0].x0, x0, g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: ramps[0].x0 = %.15e; x0 = %.15e; diff = %.15e", ramps[0].x0%x0%(ramps[0].x0 - x0));
        return PCR_XDiscrepancy;
    }
    if( !FuzzyEquals(ramps[0].v0, v0, g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_VDiscrepancy: ramps[0].v0 = %.15e; v0 = %.15e; diff = %.15e", ramps[0].v0%v0%(ramps[0].v0 - v0));
        return PCR_VDiscrepancy;
    }
    ParabolicCheckReturn ret = CheckRamp(ramps[0], xmin, xmax, vm, am);
    if( ret != PCR_Normal ) {
        RAVELOG_WARN("ramps[0] does not pass CheckRamp");
        return ret;
    }
    
    for (size_t iramp = 1; iramp < ramps.size(); ++iramp) {
        if( !FuzzyEquals(ramps[iramp - 1].x1, ramps[iramp].x0, g_fRampEpsilon) ) {
            RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: ramps[%d].x1 != ramps[%d].x0; %.15e != %.15e; diff = %.15e", (iramp - 1)%iramp%ramps[iramp - 1].x1%ramps[iramp].x0%(ramps[iramp - 1].x1 - ramps[iramp].x0));
            return PCR_XDiscrepancy;
        }
        if( !FuzzyEquals(ramps[iramp - 1].v1, ramps[iramp].v0, g_fRampEpsilon) ) {
            RAVELOG_WARN_FORMAT("PCR_VDiscrepancy: ramps[%d].v1 != ramps[%d].v0; %.15e != %.15e; diff = %.15e", (iramp - 1)%iramp%ramps[iramp - 1].v1%ramps[iramp].v0%(ramps[iramp - 1].v1 - ramps[iramp].v0));
            return PCR_VDiscrepancy;
        }
        ret = CheckRamp(ramps[iramp], xmin, xmax, vm, am);
        if( ret != PCR_Normal ) {
            RAVELOG_WARN_FORMAT("ramps[%d] does not pass CheckRamp", iramp);
            return ret;
        }
    }

    // Check the last ramp
    if( !FuzzyEquals(ramps[ramps.size() - 1].x1, x1, g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: ramps[%d].x1 = %.15e; x1 = %.15e; diff = %.15e", (ramps.size() - 1)%ramps[ramps.size() - 1].x0%x0%(ramps[ramps.size() - 1].x1 - x1));
        return PCR_XDiscrepancy;
    }
    if( !FuzzyEquals(ramps[ramps.size() - 1].v1, v1, g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_VDiscrepancy: ramps[%d].v1 = %.15e; v1 = %.15e; diff = %.15e", (ramps.size() - 1)%ramps[ramps.size() - 1].v0%v0%(ramps[ramps.size() - 1].v1 - v1));
        return PCR_VDiscrepancy;
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckRampND(const RampND& rampnd, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect)
{
    ParabolicCheckReturn ret;
    // Sometimes we want to check without joint limits
    if( xminVect.size() == 0 && xmaxVect.size() == 0 ) {
        for (size_t idof = 0; idof < rampnd.GetDOF(); ++idof) {
            ret = CheckSegment(rampnd.GetX0At(idof), rampnd.GetX1At(idof), rampnd.GetV0At(idof), rampnd.GetV1At(idof), rampnd.GetAAt(idof), rampnd.GetDuration(), -g_fRampInf, g_fRampInf, vmVect[idof], amVect[idof]);
            if( ret != PCR_Normal ) {
                RAVELOG_WARN_FORMAT("rampnd: idof = %d does not pass CheckSegment", idof);
                return ret;
            }
        }
    }
    else if( xminVect.size() == 0 ) {
        for (size_t idof = 0; idof < rampnd.GetDOF(); ++idof) {
            ret = CheckSegment(rampnd.GetX0At(idof), rampnd.GetX1At(idof), rampnd.GetV0At(idof), rampnd.GetV1At(idof), rampnd.GetAAt(idof), rampnd.GetDuration(), -g_fRampInf, xmaxVect[idof], vmVect[idof], amVect[idof]);
            if( ret != PCR_Normal ) {
                RAVELOG_WARN_FORMAT("rampnd: idof = %d does not pass CheckSegment", idof);
                return ret;
            }
        }
    }
    else if( xmaxVect.size() == 0 ) {
        for (size_t idof = 0; idof < rampnd.GetDOF(); ++idof) {
            ret = CheckSegment(rampnd.GetX0At(idof), rampnd.GetX1At(idof), rampnd.GetV0At(idof), rampnd.GetV1At(idof), rampnd.GetAAt(idof), rampnd.GetDuration(), xminVect[idof], g_fRampInf, vmVect[idof], amVect[idof]);
            if( ret != PCR_Normal ) {
                RAVELOG_WARN_FORMAT("rampnd: idof = %d does not pass CheckSegment", idof);
                return ret;
            }
        }
    }
    else {
        for (size_t idof = 0; idof < rampnd.GetDOF(); ++idof) {
            ret = CheckSegment(rampnd.GetX0At(idof), rampnd.GetX1At(idof), rampnd.GetV0At(idof), rampnd.GetV1At(idof), rampnd.GetAAt(idof), rampnd.GetDuration(), xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof]);
            if( ret != PCR_Normal ) {
                RAVELOG_WARN_FORMAT("rampnd: idof = %d does not pass CheckSegment", idof);
                return ret;
            }
        }
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckRampNDs(const std::vector<RampND>& rampnds, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect)
{
    size_t ndof = rampnds[0].GetDOF();
    size_t nrampnds = rampnds.size();

    // Check the first RampND
    for (size_t idof = 0; idof < ndof; ++idof) {
        if( !FuzzyEquals(rampnds[0].GetX0At(idof), x0Vect[idof], g_fRampEpsilon) ) {
            RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: rampnds[0].GetX0At(%d) = %.15e; x0Vect[%d] = %.15e; diff = %.15e", idof%rampnds[0].GetX0At(idof)%idof%x0Vect[idof]%(rampnds[0].GetX0At(idof) - x0Vect[idof]));
            return PCR_XDiscrepancy;
        }
        if( !FuzzyEquals(rampnds[0].GetV0At(idof), v0Vect[idof], g_fRampEpsilon) ) {
            RAVELOG_WARN_FORMAT("PCR_VDiscrepancy: rampnds[0].GetV0At(%d) = %.15e; v0Vect[%d] = %.15e; diff = %.15e", idof%rampnds[0].GetV0At(idof)%idof%v0Vect[idof]%(rampnds[0].GetV0At(idof) - v0Vect[idof]));
            return PCR_VDiscrepancy;
        }
    }
    ParabolicCheckReturn ret = CheckRampND(rampnds[0], xminVect, xmaxVect, vmVect, amVect);
    if( ret != PCR_Normal ) {
        RAVELOG_WARN_FORMAT("rampnds[0] does not pass CheckRampND; retcode = %d", ret);
        return ret;
    }

    // Check from the second RampND to the second from last RampND
    for (size_t irampnd = 1; irampnd < nrampnds; ++irampnd) {
        for (size_t idof = 0; idof < ndof; ++idof) {
            // Check initial values
            if( !FuzzyEquals(rampnds[irampnd - 1].GetX1At(idof), rampnds[irampnd].GetX0At(idof), g_fRampEpsilon) ) {
                RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: rampnds[%d].GetX1At(%d) = %.15e; rampnds[%d].GetX0At(%d) = %.15e; diff = %.15e", (irampnd - 1)%idof%rampnds[irampnd - 1].GetX1At(idof)%irampnd%idof%rampnds[irampnd].GetX0At(idof)%(rampnds[irampnd - 1].GetX1At(idof) - rampnds[irampnd].GetX0At(idof)));
                return PCR_XDiscrepancy;
            }
            if( !FuzzyEquals(rampnds[irampnd - 1].GetV1At(idof), rampnds[irampnd].GetV0At(idof), g_fRampEpsilon) ) {
                RAVELOG_WARN_FORMAT("PCR_VDiscrepancy: rampnds[%d].GetV1At(%d) = %.15e; rampnds[%d].GetV0At(%d) = %.15e; diff = %.15e", (irampnd - 1)%idof%rampnds[irampnd - 1].GetV1At(idof)%irampnd%idof%rampnds[irampnd].GetV0At(idof)%(rampnds[irampnd - 1].GetV1At(idof) - rampnds[irampnd].GetV0At(idof)));
                return PCR_VDiscrepancy;
            }
        }
        ret = CheckRampND(rampnds[irampnd], xminVect, xmaxVect, vmVect, amVect);
        if( ret != PCR_Normal ) {
            RAVELOG_WARN_FORMAT("rampnds[%d] does not pass CheckRampND; retcode = %d", irampnd%ret);
            return ret;
        }
    }

    // Check the last RampND
    for (size_t idof = 0; idof < ndof; ++idof) {
        // Check final values
        if( !FuzzyEquals(rampnds[nrampnds - 1].GetX1At(idof), x1Vect[idof], g_fRampEpsilon) ) {
            RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: rampnds[%d].GetX1At(%d) = %.15e; x1Vect[%d] = %.15e; diff = %.15e", (nrampnds - 1)%idof%rampnds[nrampnds - 1].GetX1At(idof)%idof%x1Vect[idof]%(rampnds[nrampnds - 1].GetX1At(idof) - x1Vect[idof]));
            return PCR_XDiscrepancy;
        }
        if( !FuzzyEquals(rampnds[nrampnds - 1].GetV1At(idof), v1Vect[idof], g_fRampEpsilon) ) {
            RAVELOG_WARN_FORMAT("PCR_VDiscrepancy: rampnds[%d].GetV1At(%d) = %.15e; v1Vect[%d] = %.15e; diff = %.15e", (nrampnds - 1)%idof%rampnds[nrampnds - 1].GetV1At(idof)%idof%v1Vect[idof]%(rampnds[nrampnds - 1].GetV1At(idof) - v1Vect[idof]));
            return PCR_VDiscrepancy;
        }
    }
    ret = CheckRampND(rampnds[nrampnds - 1], xminVect, xmaxVect, vmVect, amVect);
    if( ret != PCR_Normal ) {
        RAVELOG_WARN_FORMAT("rampnds[%d] does not pass CheckRampND; retcode = %d", (nrampnds - 1)%ret);
        return ret;
    }

    return PCR_Normal;
}


} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
