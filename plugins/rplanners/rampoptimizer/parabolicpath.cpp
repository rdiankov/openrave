// -*- coding: utf-8 -*-
// Copyright (C) 2016 Puttichai Lertkultanon <L.Puttichai@gmail.com>
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
#include "parabolicpath.h"
#include "Timer.h"
#include <algorithm>

namespace OpenRAVE {

namespace RampOptimizerInternal {

inline dReal LInfDistance(const std::vector<dReal>& a, const std::vector<dReal>& b) {
    RAMP_OPTIM_ASSERT(a.size() == b.size());
    dReal d = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        d = Max(d, Abs(a[i] - b[i]));
    }
    return d;
}

inline dReal MaxBBLInfDistance(const std::vector<dReal>& x, const std::vector<dReal>& bmin, const std::vector<dReal>& bmax) {
    RAMP_OPTIM_ASSERT(x.size() == bmin.size());
    RAMP_OPTIM_ASSERT(x.size() == bmax.size());
    dReal d = 0;
    for (size_t i = 0; i < x.size(); ++i) {
        d = Max(d, Max(Abs(x[i] - bmin[i]), Abs(x[i] - bmax[i])));
    }
    return d;
}

struct CurvesNDSection {
    dReal ta, tb;
    std::vector<dReal> xa, xb;
    dReal da, db; // obstacle distances at xa and xb
};

/// \brief Check feasibility of the given ParabolicCurvesND by recursively bisecting the curve and
/// checking the midpoints. Each consecutive pair of checking points will be equally spaced (in
/// time).
int CheckParabolicCurvesNDFeasibility(const ParabolicCurvesND& curvesnd, FeasibilityCheckerBase* feas, DistanceCheckerBase* dist, int maxiter, __attribute__((unused)) int options) {
    for (std::vector<bool>::iterator itCheck = curvesnd.constraintCheckedVect.begin(); itCheck != curvesnd.constraintCheckedVect.end(); ++itCheck) {
        *itCheck = true;
    }

    // Check boundary configurations
    int ret0 = feas->ConfigFeasible(curvesnd.x0Vect, curvesnd.v0Vect);
    if (ret0 != 0) {
        return ret0;
    }
    int ret1 = feas->ConfigFeasible(curvesnd.x1Vect, curvesnd.v1Vect);
    if (ret1 != 0) {
        return ret1;
    }

    RAMP_OPTIM_ASSERT(dist->ObstacleDistanceNorm() == Inf);
    CurvesNDSection section;
    section.ta = 0;
    section.tb = curvesnd.duration;
    section.xa = curvesnd.x0Vect;
    section.xb = curvesnd.x1Vect;
    section.da = dist->ObstacleDistance(curvesnd.x0Vect);
    section.db = dist->ObstacleDistance(curvesnd.x1Vect);
    if (section.da <= 0) {
        return 0xffff;
    }
    if (section.db <= 0) {
        return 0xffff;
    }

    std::list<CurvesNDSection> queue;
    queue.push_back(section);
    int iters = 0;
    while (!queue.empty()) {
        section = queue.front();
        queue.erase(queue.begin());

        if (LInfDistance(section.xa, section.xb) <= section.da + section.db) {
            std::vector<dReal> bminVect, bmaxVect;
            curvesnd.GetPeaks(section.ta, section.tb, bminVect, bmaxVect);
            if (MaxBBLInfDistance(section.xa, bminVect, bmaxVect) < section.da && MaxBBLInfDistance(section.xb, bminVect, bmaxVect) < section.db) {
                continue;
            }
        }

        // Subdivide the section into two halves.
        dReal tc = 0.5*(section.ta + section.tb);
        std::vector<dReal> xc, dxc;
        curvesnd.EvalPos(tc, xc);
        if (feas->NeedDerivativeForFeasibility()) {
            curvesnd.EvalVel(tc, dxc);
        }

        int retseg = feas->ConfigFeasible(xc, dxc);
        if (retseg != 0) {
            return retseg;
        }

        dReal dc = dist->ObstacleDistance(xc);
        CurvesNDSection sa, sb;
        sa.ta = section.ta;
        sa.xa = section.xa;
        sa.da = section.da;
        sa.tb = sb.ta = tc;
        sa.xb = sb.xa = xc;
        sa.db = sb.da = dc;
        sb.tb = section.tb;
        sb.xb = section.xb;
        sb.db = section.db;

        queue.push_back(sa);
        queue.push_back(sb);

        if (iters++ >= maxiter) {
            return 0xffff;
        }
    }
    return 0;
}

/// \brief Check feasibility of the given ParabolicCurvesND by dividing the curve into segments
/// where in each segment, every DOF has its deviation from the straight line connecting end points
/// no more than the corresponding tol.
int CheckParabolicCurvesNDFeasibility(const ParabolicCurvesND& curvesnd, FeasibilityCheckerBase* feas, const std::vector<dReal>& tol, int options) {
    RAMP_OPTIM_ASSERT(tol.size() == curvesnd.ndof);

    for (size_t i = 0; i < tol.size(); ++i) {
        RAMP_OPTIM_ASSERT(tol[i] > 0);
    }
    int ret0 = feas->ConfigFeasible(curvesnd.x0Vect, curvesnd.v0Vect, options);
    if (ret0 != 0) {
        return ret0;
    }

    int ret1 = feas->ConfigFeasible(curvesnd.x1Vect, curvesnd.v1Vect, options);
    if (ret1 != 0) {
        return ret1;
    }

    /*
       The procedure is taken from the original ParabolicPathSmooth library.

       Consider a parabola f(x) = a*x^2 + b*x (assume f(0) = 0, i.e. f passes through the origin) and
       a straight line passing through the origin g(x, u) = f(u)*x (g intersects f at x = 1).
     */
    std::vector<dReal> divs;
    dReal t = 0;
    divs.push_back(t);
    while (t < curvesnd.duration) {
        dReal tnext = t;
        dReal am = 0;
        dReal switchNext = curvesnd.duration;
        dReal dtmin = 1e30;
        for (size_t i = 0; i < curvesnd.ndof; ++i) {
            
        }
    }


    return 0;
}

ParabolicCurvesNDFeasibilityChecker::ParabolicCurvesNDFeasibilityChecker(FeasibilityCheckerBase* _feas) : feas(_feas), tol(0), distance(NULL), maxiter(0), constraintmask(0) {
}

ParabolicCurvesNDFeasibilityChecker::ParabolicCurvesNDFeasibilityChecker(FeasibilityCheckerBase* _feas, DistanceCheckerBase* _dist, int _maxiter) : feas(_feas), tol(0), distance(_dist), maxiter(_maxiter), constraintmask(0) {
}

int ParabolicCurvesNDFeasibilityChecker::Check(const ParabolicCurvesND &curvesnd, int options) {
    if ((options & constraintmask) == constraintmask) {
        for (std::vector<bool>::iterator itCheck = curvesnd.constraintCheckedVect.begin(); itCheck != curvesnd.constraintCheckedVect.end(); ++itCheck) {
            *itCheck = true;
        }
    }
    if (distance) {
        return CheckParabolicCurvesNDFeasibility(curvesnd, feas, distance, maxiter, options);
    }
    else {
        return CheckParabolicCurvesNDFeasibility(curvesnd, feas, tol, options);
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// ParabolicPath
ParabolicPath::ParabolicPath() {

}

void ParabolicPath::Init(const std::vector<dReal>& velMax, const std::vector<dReal>& accMax) {
    vmVect = velMax;
    amVect = accMax;
    RAMP_OPTIM_ASSERT(vmVect.size() == amVect.size());
}

void ParabolicPath::SetJointLimits(const std::vector<dReal>& qMin, const std::vector<dReal>& qMax) {
    xminVect = qMin;
    xmaxVect = qMax;
    RAMP_OPTIM_ASSERT(xminVect.size() == xmaxVect.size());
}

bool ParabolicPath::IsValid() {
    for (size_t icurvesnd = 0; icurvesnd < curvesndVect.size(); ++icurvesnd) {
        ParabolicCheckReturn ret = CheckParabolicCurvesND(curvesndVect[icurvesnd], xminVect, xmaxVect, vmVect, amVect, curvesndVect[icurvesnd].x0Vect, curvesndVect[icurvesnd].x1Vect, curvesndVect[icurvesnd].v0Vect, curvesndVect[icurvesnd].v1Vect);
        if (!(ret == PCR_Normal)) {
            RAMP_OPTIM_WARN("CheckParabolicCurvesND on ParaboolicCurve %d/%d returns %d", icurvesnd, curvesndVect.size(), ret);
            return false;
        }
    }
    return true;
}

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE

