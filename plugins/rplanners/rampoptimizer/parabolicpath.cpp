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
        dReal tNext = t;
        dReal am = 0;
        dReal switchNext = curvesnd.duration;
        dReal dtmin = 1e30;
        for (size_t idof = 0; idof < curvesnd.ndof; ++idof) {
            int index;
            dReal remainder;
            curvesnd.curves[idof].FindRampIndex(t, index, remainder);
            switchNext = Min(switchNext, curvesnd.curves[idof].switchpointsList[index + 1]);
            am = Max(am, Abs(curvesnd.curves[idof].ramps[index].a));
            dtmin = Min(dtmin, 2.0*Sqrt(tol[idof]/am));
        }

        if (t + dtmin > switchNext) {
            tNext = switchNext;
        }
        else {
            tNext = t + dtmin;
        }
        t = tNext;
        divs.push_back(tNext);
    }
    divs.push_back(curvesnd.duration);

    std::list<std::pair<int, int> > segs;
    segs.push_back(std::pair<int, int>(0, divs.size() - 1));
    std::vector<dReal> q0, q1, dq0, dq1;

    while (!segs.empty()) {
        int i = segs.front().first;
        int j = segs.front().second;
        segs.erase(segs.begin());
        if (j == i + 1) {
            curvesnd.EvalPos(divs[i], q0);
            if (feas->NeedDerivativeForFeasibility()) {
                curvesnd.EvalVel(divs[i], dq0);
            }

            curvesnd.EvalPos(divs[j], q0);
            if (feas->NeedDerivativeForFeasibility()) {
                curvesnd.EvalVel(divs[j], dq0);
            }

            int retseg = feas->SegmentFeasible(q0, q1, dq0, dq1, divs[j] - divs[i], options);
            if (retseg != 0) {
                return retseg;
            }
        }
        else {
            int k = (i + j)/2;
            curvesnd.EvalPos(divs[k], q0);
            if (feas->NeedDerivativeForFeasibility()) {
                curvesnd.EvalVel(divs[k], dq0);
            }

            int retconf = feas->ConfigFeasible(q1, dq1, options);
            if (retconf != 0) {
                return retconf;
            }

            segs.push_back(std::pair<int, int>(i, k));
            segs.push_back(std::pair<int, int>(k, j));
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
ParabolicPath::ParabolicPath() : isInitialized(false) {
}

void ParabolicPath::Initialize(const std::vector<dReal>& qMin, const std::vector<dReal>& qMax, const std::vector<dReal>& velMax, const std::vector<dReal>& accMax) {
    ndof = qMin.size();
    xminVect = qMin;
    xmaxVect = qMax;
    RAMP_OPTIM_ASSERT(xmaxVect.size() == ndof);

    vmVect = velMax;
    amVect = accMax;
    RAMP_OPTIM_ASSERT(vmVect.size() == amVect.size());

    RAMP_OPTIM_ASSERT(vmVect.size() == ndof);
    isInitialized = true;
    return;
}

bool ParabolicPath::IsValid() {
    for (size_t icurvesnd = 0; icurvesnd < curvesndVect.size(); ++icurvesnd) {
        ParabolicCheckReturn ret = CheckParabolicCurvesND(curvesndVect[icurvesnd], xminVect, xmaxVect, vmVect, amVect, curvesndVect[icurvesnd].x0Vect, curvesndVect[icurvesnd].x1Vect, curvesndVect[icurvesnd].v0Vect, curvesndVect[icurvesnd].v1Vect);
        if (!(ret == PCR_Normal)) {
            RAMP_OPTIM_WARN("CheckParabolicCurvesND on ParabolicCurve %d/%d returns %d", icurvesnd, curvesndVect.size(), ret);
            return false;
        }
    }
    return true;
}

void ParabolicPath::AppendParabolicCurvesND(const ParabolicCurvesND& curvesndIn) {
    RAMP_OPTIM_ASSERT(isInitialized);

    if (IsEmpty()) {
        curvesndVect.reserve(1);
        curvesndVect.push_back(curvesndIn);

        duration = curvesndIn.duration;

        mainSwitchpoints.reserve(2);
        mainSwitchpoints.push_back(0);
        mainSwitchpoints.push_back(duration);

        x0Vect = curvesndIn.x0Vect;
        x1Vect = curvesndIn.x1Vect;
        v0Vect = curvesndIn.v0Vect;
        v1Vect = curvesndIn.v1Vect;

        isInitialized = true;
    }
    else {
        RAMP_OPTIM_ASSERT(curvesndIn.ndof == ndof);

        curvesndVect.reserve(curvesndVect.size() + 1);
        curvesndVect.push_back(curvesndIn);

        curvesndVect.back().SetInitialValues(x1Vect);
        x1Vect = curvesndVect.back().x1Vect;
        v1Vect = curvesndVect.back().v1Vect;

        duration = duration + curvesndIn.duration;
        mainSwitchpoints.reserve(mainSwitchpoints.size() + 1);
        mainSwitchpoints.push_back(duration);
    }
    return;
}

void ParabolicPath::AppendParabolicPath(const ParabolicPath &pathIn) {
    RAMP_OPTIM_ASSERT(isInitialized);

    if (IsEmpty()) {
        Reconstruct(pathIn.curvesndVect);
        return;
    }
    else {
        for (size_t icurvesnd = 0; icurvesnd < pathIn.curvesndVect.size(); ++icurvesnd) {
            AppendParabolicCurvesND(pathIn.curvesndVect[icurvesnd]);
        }
        return;
    }
}

void ParabolicPath::FindParabolicCurvesNDIndex(dReal t, int &index, dReal &remainder) const {
    RAMP_OPTIM_ASSERT(t >= -epsilon);
    RAMP_OPTIM_ASSERT(t <= duration + epsilon);
    if (t < epsilon) {
        index = 0;
        remainder = 0;
    }
    else if (t > duration - epsilon) {
        index = ((int) curvesndVect.size()) - 1;
        remainder = curvesndVect.back().duration;
    }
    else {
        index = 0;
        // Iterate through mainSwitchpoints
        std::vector<dReal>::const_iterator it = mainSwitchpoints.begin();
        while (it != mainSwitchpoints.end() && t > *it) {
            index++;
            it++;
        }
        RAMP_OPTIM_ASSERT(index < (int)mainSwitchpoints.size());
        index = index - 1;
        remainder = t - *(it - 1);
    }
    return;
}

void ParabolicPath::PopBack() {
    if (IsEmpty()) {
        return;
    }

    curvesndVect.pop_back();
    mainSwitchpoints.pop_back();
    duration = mainSwitchpoints.back();

    x1Vect = curvesndVect.back().x1Vect;
    v1Vect = curvesndVect.back().v1Vect;
}

void ParabolicPath::Reconstruct(const std::vector<ParabolicCurvesND> &curvesndVectIn) {
    RAMP_OPTIM_ASSERT(curvesndVectIn.size() > 0);
    RAMP_OPTIM_ASSERT(isInitialized);
    RAMP_OPTIM_ASSERT(curvesndVectIn.front().ndof == ndof);

    Clear();
    for (size_t icurvesnd = 0; icurvesnd < curvesndVectIn.size(); ++icurvesnd) {
        AppendParabolicCurvesND(curvesndVectIn[icurvesnd]);
    }
    return;
}

void ParabolicPath::ReplaceSegment(dReal t0, dReal t1, const ParabolicPath &pathIn) {
    ReplaceSegment(t0, t1, pathIn.curvesndVect);
    return;
}

void ParabolicPath::ReplaceSegment(dReal t0, dReal t1, const std::vector<ParabolicCurvesND> &curvesndVectIn) {
    int index0, index1;
    dReal rem0, rem1;

    FindParabolicCurvesNDIndex(t0, index0, rem0);
    FindParabolicCurvesNDIndex(t1, index1, rem1);

    ParabolicCurvesND tempCurvesND;
    std::vector<ParabolicCurvesND> newCurvesNDVect;
    newCurvesNDVect.reserve((index0 + 1) + (index1 + 1) + (curvesndVectIn.size()));

    // Insert the left part from the original ParabolicPath
    for (int i = 0; i < index0; ++i) {
        newCurvesNDVect.push_back(curvesndVect[i]);
    }
    tempCurvesND = curvesndVect[index0];
    tempCurvesND.TrimBack(rem0);
    if (tempCurvesND.duration > 0) {
        newCurvesNDVect.push_back(tempCurvesND);
    }

    // Insert the middle part from pathIn
    for (std::vector<ParabolicCurvesND>::const_iterator it = curvesndVectIn.begin(); it != curvesndVectIn.end(); ++it) {
        newCurvesNDVect.push_back(*it);
    }

    // Insert the right part from the original ParabolicPath
    tempCurvesND = curvesndVect[index1];
    tempCurvesND.TrimFront(rem1);
    if (tempCurvesND.duration > 0) {
        newCurvesNDVect.push_back(tempCurvesND);
    }
    for (int i = index1 + 1; i < ((int) curvesndVect.size()); ++i) {
        newCurvesNDVect.push_back(curvesndVect[i]);
    }

    // Initialize the ParabolicPath with the new set of ParabolicCurvesND
    Reconstruct(newCurvesNDVect);
    return;
}

void ParabolicPath::Save(std::string filename) const {
    // Do simple verification
    for (size_t i = 0; i < curvesndVect.size(); ++i) {
        RAMP_OPTIM_ASSERT(curvesndVect[i].ndof == ndof);
    }

    std::string s = "";
    std::string dummy;

    for (size_t icurvesnd = 0; icurvesnd < curvesndVect.size(); ++icurvesnd) {
        curvesndVect[icurvesnd].ToString(dummy);
        s = s + dummy;
    }

    std::ofstream f(filename.c_str());
    f << s;
    return;
}

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE

