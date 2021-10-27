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
#ifndef RAMP_OPTIM_FEAS_CHECKER_H
#define RAMP_OPTIM_FEAS_CHECKER_H
#include "ramp.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

struct CheckReturn {
    CheckReturn(int retcode=0, dReal fmult=1.0, dReal fvel=0, dReal faccel=0) : retcode(retcode), fTimeBasedSurpassMult(fmult), bDifferentVelocity(false), fMaxManipSpeed(fvel), fMaxManipAccel(faccel) {
        vReductionFactors.resize(0);
        // vVelReductionFactors.resize(0);
        // vAccelReductionFactors.resize(0);
    }

    CheckReturn(int retcode, std::vector<dReal> vfactors) : retcode(retcode), fTimeBasedSurpassMult(1.0), bDifferentVelocity(false), fMaxManipSpeed(0), fMaxManipAccel(0), vReductionFactors(vfactors) {
    }

    int retcode; // one of CFO_X defined in planner.h
    dReal fTimeBasedSurpassMult; // if retcode == CFO_CheckTimeBasedConstraints, then the multiplier is set to (some factor)*|max|/|actual max|
    bool bDifferentVelocity; // the segment ends with some velocity other than the desired value (resulting from modifications CheckPathAllConstraints)
    dReal fMaxManipSpeed; // when > 0, the value is the computed max manip speed
    dReal fMaxManipAccel; // when > 0, the value is the computed max manip accel
    std::vector<dReal> vReductionFactors; // experimental: scaling factors for each DOF
    // std::vector<dReal> vVelReductionFactors; // experimental: scaling factors for each DOF
    // std::vector<dReal> vAccelReductionFactors; // experimental: scaling factors for each DOF
};

class FeasibilityCheckerBase {
public:
    virtual ~FeasibilityCheckerBase() {
    }

    virtual int ConfigFeasible(const std::vector<dReal>& q0, const std::vector<dReal>& dq0, int options=0xffff)=0;
    virtual int SegmentFeasible(const std::vector<dReal>& q0, const std::vector<dReal>& q1, const std::vector<dReal>& dq0, const std::vector<dReal>& dq1, dReal timeElapsed, int options=0xffff)
    {
        BOOST_ASSERT(0);
        return 0;
    }
    virtual CheckReturn ConfigFeasible2(const std::vector<dReal>& q0, const std::vector<dReal>& dq0, int options=0xffff)
    {
        return CheckReturn(ConfigFeasible(q0, dq0, options));
    }
    virtual CheckReturn SegmentFeasible2(const std::vector<dReal>& q0, const std::vector<dReal>& dq0, const std::vector<dReal>& q1, const std::vector<dReal>& dq1, dReal timeElapsed, int options, std::vector<RampND> &rampndVectOut, std::vector<dReal> &vIntermediateConfigurations)
    {
        BOOST_ASSERT(0);
        return 0;
    }
    virtual bool NeedDerivativeForFeasibility()
    {
        return false;
    }
};

class DistanceCheckerBase {
public:
    virtual ~DistanceCheckerBase() {
    }
    virtual dReal ObstacleDistanceNorm() const
    {
        return g_fRampInf;
    }
    virtual dReal ObstacleDistance(const std::vector<dReal>& x)=0;
};

class RampNDFeasibilityChecker {
public:
    RampNDFeasibilityChecker(FeasibilityCheckerBase* feas);
    RampNDFeasibilityChecker(FeasibilityCheckerBase* feas, DistanceCheckerBase* dist, int maxiter);

    virtual int Check(const std::vector<RampND>& rampsndVect, int options=0xffff);
    virtual CheckReturn Check2(const RampND& rampnd, int options, std::vector<RampND>& rampsndVectOut)
    {
        BOOST_ASSERT(0);
        return CheckReturn(0);
    }

    virtual CheckReturn Check2(const std::vector<RampND>& rampsndVect, int options, std::vector<RampND>& rampsndVectOut)
    {
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
    virtual dReal Rand()
    {
        return ::OpenRAVE::RampOptimizerInternal::Rand();
    }
};

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
#endif
