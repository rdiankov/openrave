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

    dReal xDeflection = x0 + tDeflection*(v0 + 0.5*a*tDeflection);
    bmin = Min(curMin, xDeflection);
    bmax = Max(curMax, xDeflection);
    return;    
}

ParabolicCheckReturn CheckSegment(dReal x0, dReal x1, dReal v0, dReal v1, dReal a, dReal d, dReal t, dReal xmin, dReal xmax, dReal vm, dReal am)
{
    if( t < -g_fRampEpsilon ) {
        RAVELOG_WARN_FORMAT("PCR_NegativeDuration: duration = %.15e", t);
        return PCR_NegativeDuration;
    }
    dReal v1_ = v0 + a*t;
    if( !FuzzyEquals(v1, v1_, g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_VDiscrepancy: v1 = %.15e; computed v1 = %.15e; diff = %.15e", v1%v1_%(v1 - v1_));
        return PCR_VDiscrepancy;
    }
    dReal d_ = t*(v0 + 0.5*a*t);
    if( !FuzzyEquals(d, d_, g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: d = %.15e; computed d = %.15e; diff = %.15e", d%d_%(d - d_));
        return PCR_XDiscrepancy;
    }
    dReal x1_ = x0 + d;
    if( !FuzzyEquals(x1, x1_, g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: x1 = %.15e; computed x1 = %.15e; diff = %.15e", x1%x1_%(x1 - x1_));
        return PCR_XDiscrepancy;
    }
    dReal bmin, bmax;
    _GetPeaks(x0, x1, v0, v1, a, t, bmin, bmax);
    if( (bmin < xmin - g_fRampEpsilon) || (bmax > xmax + g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_XBoundViolated: xmin = %.15e; bmin = %.15e; diff@min = %.15e; xmax = %.15e; bmax = %.15e; diff@max = %.15e", xmin%bmin%(xmin - bmin)%xmax%bmax%(bmax - xmax));
        return PCR_XBoundViolated;
    }
    if( (Abs(v0) > vm + g_fRampEpsilon) || (Abs(v1) > vm + g_fRampEpsilon) ) {
        RAVELOG_WARN_FORMAT("PCR_VBoundViolated: vm = %.15e; v0 = %.15e; v1 = %.15e; diff@v0 = %.15e; diff@v1 = %.15e", vm%v0%v1%(Abs(v0) - vm)%(Abs(v1) - vm));
        return PCR_VBoundViolated;
    }
    if( Abs(a) > am + g_fRampEpsilon ) {
        RAVELOG_WARN_FORMAT("PCR_ABoundViolated: am = %.15e; a = %.15e; diff = %.15e", am%a%(Abs(a) - am));
        return PCR_ABoundViolated;
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckRamp(const Ramp& ramp, dReal xmin, dReal xmax, dReal vm, dReal am)
{
    return CheckSegment(ramp.x0, ramp.x1, ramp.v0, ramp.v1, ramp.a, ramp.d, ramp.duration, xmin, xmax, vm, am);
}

ParabolicCheckReturn CheckRamps(const std::vector<Ramp>& ramps, dReal xmin, dReal xmax, dReal vm, dReal am)
{
    ParabolicCheckReturn ret = CheckRamp(ramps[0], xmin, xmax, vm, am);
    if( ret != PCR_Normal ) {
        RAVELOG_WARN("ramps[0] does not pass CheckRamp");
        return ret;
    }
    for (size_t iramp = 1; iramp < ramps.size(); ++iramp) {
        if( !FuzzyEquals(ramps[iramp - 1].v1, ramps[iramp].v0, g_fRampEpsilon) ) {
            RAVELOG_WARN_FORMAT("PCR_VDiscontinuous: ramps[%d].v1 != ramps[%d].v0; %.15e != %.15e; diff = %.15e", (iramp - 1)%iramp%ramps[iramp - 1].v1%ramps[iramp].v0%(ramps[iramp - 1].v1 - ramps[iramp].v0));
            return PCR_VDiscontinuous;
        }
        ret = CheckRamp(ramps[iramp], xmin, xmax, vm, am);
        if( ret != PCR_Normal ) {
            RAVELOG_WARN_FORMAT("ramps[%d] does not pass CheckRamp", iramp);
            return ret;
        }
    }
    return PCR_Normal;
}

ParabolicCheckReturn CheckRampND(const RampND& rampnd, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect)
{
    for (size_t idof = 0; idof < rampnd.GetDOF(); ++idof) {
        ParabolicCheckReturn ret = CheckSegment(rampnd.GetX0At(idof), rampnd.GetX1At(idof), rampnd.GetV0At(idof), rampnd.GetV1At(idof), rampnd.GetAAt(idof), rampnd.GetDAt(idof), rampnd.GetDuration(), xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof]);
        if( ret != PCR_Normal ) {
            RAVELOG_WARN_FORMAT("rampnd: idof = %d does not pass CheckSegment", idof);
            return ret;
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
        return ret;
    }

    // Check from the second RampND to the second from last RampND
    for (size_t irampnd = 1; irampnd < nrampnds - 1; ++irampnd) {
        for (size_t idof = 0; idof < ndof; ++idof) {
            // Check initial values
            if( !FuzzyEquals(rampnds[irampnd - 1].GetX1At(idof), rampnds[irampnd].GetX0At(idof), g_fRampEpsilon) ) {
                RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: rampnds[%d].GetX1At(%d) = %.15e; rampnds[%d].GetX0At(%d) = %.15e; diff = %.15e", (irampnd - 1)%idof%rampnds[irampnd - 1].GetX1At(idof)%irampnd%idof%rampnds[irampnd].GetX0At(idof)%(rampnds[irampnd - 1].GetX1At(idof) - rampnds[irampnd].GetX0At(idof)));
                return PCR_XDiscrepancy;
            }
            if( !FuzzyEquals(rampnds[irampnd - 1].GetV1At(idof), rampnds[irampnd].GetV0At(idof), g_fRampEpsilon) ) {
                RAVELOG_WARN_FORMAT("PCR_VDiscrepancy: rampnds[%d].GetV1At(%d) = %.15e; rampnds[%d].GetV0At(idof) = %.15e; diff = %.15e", (irampnd - 1)%idof%rampnds[irampnd - 1].GetV1At(idof)%irampnd%idof%rampnds[irampnd].GetV0At(idof)%(rampnds[irampnd - 1].GetV1At(idof) - rampnds[irampnd].GetV0At(idof)));
                return PCR_VDiscrepancy;
            }
        }
        ret = CheckRampND(rampnds[irampnd], xminVect, xmaxVect, vmVect, amVect);
        if( ret != PCR_Normal ) {
            return ret;
        }
    }

    // Check the last RampND
    for (size_t idof = 0; idof < ndof; ++idof) {
        // Check initial values
        if( !FuzzyEquals(rampnds[nrampnds - 2].GetX1At(idof), rampnds[nrampnds - 1].GetX0At(idof), g_fRampEpsilon) ) {
            RAVELOG_WARN_FORMAT("PCR_XDiscrepancy: rampnds[%d].GetX1At(%d) = %.15e; rampnds[%d].GetX0At(%d) = %.15e; diff = %.15e", (nrampnds - 2)%idof%rampnds[nrampnds - 2].GetX1At(idof)%(nrampnds - 1)%idof%rampnds[nrampnds - 1].GetX0At(idof)%(rampnds[nrampnds - 2].GetX1At(idof) - rampnds[nrampnds - 1].GetX0At(idof)));
            return PCR_XDiscrepancy;
        }
        if( !FuzzyEquals(rampnds[nrampnds - 2].GetV1At(idof), rampnds[nrampnds - 1].GetV0At(idof), g_fRampEpsilon) ) {
            RAVELOG_WARN_FORMAT("PCR_VDiscrepancy: rampnds[%d].GetV1At(%d) = %.15e; rampnds[%d].GetV0At(idof) = %.15e; diff = %.15e", (nrampnds - 2)%idof%rampnds[nrampnds - 2].GetV1At(idof)%(nrampnds - 1)%idof%rampnds[nrampnds - 1].GetV0At(idof)%(rampnds[nrampnds - 2].GetV1At(idof) - rampnds[nrampnds - 1].GetV0At(idof)));
            return PCR_VDiscrepancy;
        }

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
        return ret;
    }

    return PCR_Normal;
}


} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
