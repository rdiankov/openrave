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
#ifndef RAMP_OPTIM_PARABOLIC_PATH_H
#define RAMP_OPTIM_PARABOLIC_PATH_H

#include "parabolicchecker.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

struct CheckReturn {
    CheckReturn(int retcode=0, dReal fmult=1.0) : retcode(retcode), fTimeBasedSurpassMult(fmult), bDifferentVelocity(false) {
    }

    int retcode;
    dReal fTimeBasedSurpassMult;
    bool bDifferentVelocity;
};


class FeasibilityCheckerBase {
public:
    virtual ~FeasibilityCheckerBase() {
    }

    virtual int ConfigFeasible(const std::vector<dReal>& q0, const std::vector<dReal>& dq0, int options=0xffff)=0;
    virtual int SegmentFeasible(const std::vector<dReal>& q0, const std::vector<dReal>& dq0, const std::vector<dReal>& q1, const std::vector<dReal>& dq1, dReal timeElapsed, int options=0xffff) {
        BOOST_ASSERT(0);
        return 0;
    }
    virtual CheckReturn ConfigFeasible2(const std::vector<dReal>& q0, const std::vector<dReal>& dq0, int options=0xffff) {
        return CheckReturn(ConfigFeasible(q0, dq0, options));
    }
    virtual CheckReturn SegmentFeasible2(const std::vector<dReal>& q0, const std::vector<dReal>& dq0, const std::vector<dReal>& q1, const std::vector<dReal>& dq1, dReal timeElapsed, int options, ParabolicCurvesND& curvendOut) {
        BOOST_ASSERT(0);
        return 0;
    }
    virtual bool NeedDerivativeForFeasibility() {
        return false;
    }
};


class DistanceCheckerBase {
public:
    virtual ~DistanceCheckerBase() {
    }
    virtual dReal ObstacleDistanceNorm() const {
        return inf;
    }
    virtual dReal ObstacleDistance(const std::vector<dReal>& x)=0;
};


int CheckParabolicCurvesNDFeasibility(const ParabolicCurvesND& curvesnd, FeasibilityCheckerBase* feas, DistanceCheckerBase* dist, int maxiter, int options=0xffff);
int CheckParabolicCurvesNDFeasibility(const ParabolicCurvesND& curvesnd, FeasibilityCheckerBase* feas, const std::vector<dReal>& tol, int options=0xffff);

class ParabolicCurvesNDFeasibilityChecker {
public:
    ParabolicCurvesNDFeasibilityChecker(FeasibilityCheckerBase* feas);
    ParabolicCurvesNDFeasibilityChecker(FeasibilityCheckerBase* feas, DistanceCheckerBase* dist, int maxiter);

    virtual int Check(const ParabolicCurvesND& curvesnd, int options=0xffff);
    virtual CheckReturn Check2(const ParabolicCurvesND& curvesnd, int options, ParabolicCurvesND& curvesndOut) {
        BOOST_ASSERT(0);
        return CheckReturn(0);
    }

    FeasibilityCheckerBase* feas;
    std::vector<dReal> tol;
    DistanceCheckerBase* distance;
    int maxiter;
    int constraintmask;
};

class RandomNumberGeneratorBase {
public:
    virtual dReal Rand() {
        return ::OpenRAVE::RampOptimizerInternal::Rand();
    }
};


////////////////////////////////////////////////////////////////////////////////////////////////////
class ParabolicPath {
public:
    ParabolicPath();
    void Init(const std::vector<dReal>& velMax, const std::vector<dReal>& accMax);
    void SetJointLimits(const std::vector<dReal>& qMin, const std::vector<dReal>& qMax);
    inline void Clear() {
        curvesndVect.clear();
    }
    inline bool IsEmpty() const {
        return (curvesndVect.size() == 0);
    }
    bool IsValid();

    // Members
    std::vector<dReal> xminVect, xmaxVect, vmVect, amVect;
    std::vector<dReal> x0Vect, x1Vect, v0Vect, v1Vect;
    // ParabolicCurvesND curvesnd;
    std::vector<ParabolicCurvesND> curvesndVect;
    std::vector<dReal> mainSwitchpoints; // switchpoints at which all DOFs switch. It includes both end points.
    dReal duration;
};

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
#endif
