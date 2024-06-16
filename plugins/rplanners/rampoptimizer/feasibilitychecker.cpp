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
#include "openraveplugindefs.h"
#include "feasibilitychecker.h"
#include <algorithm>

namespace OpenRAVE {

namespace RampOptimizerInternal {

inline dReal LInfDistance(const std::vector<dReal>& a, const std::vector<dReal>& b)
{
    OPENRAVE_ASSERT_OP(a.size(), ==, b.size());
    dReal d = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        d = Max(d, Abs(a[i] - b[i]));
    }
    return d;
}

inline dReal MaxBBLInfDistance(const std::vector<dReal>& x, const std::vector<dReal>& bmin, const std::vector<dReal>& bmax)
{
    OPENRAVE_ASSERT_OP(x.size(), ==, bmin.size());
    OPENRAVE_ASSERT_OP(x.size(), ==, bmax.size());
    dReal d = 0;
    for (size_t i = 0; i < x.size(); ++i) {
        d = Max(d, Max(Abs(x[i] - bmin[i]), Abs(x[i] - bmax[i])));
    }
    return d;
}

struct RampNDSection {
    dReal ta, tb;
    std::vector<dReal> xa, xb;
    dReal da, db; // obstacle distances at xa and xb
};

int CheckRampNDFeasibility(const std::vector<RampND>& rampndVect, FeasibilityCheckerBase* feas, DistanceCheckerBase* dist, int maxiter, __attribute__((unused)) int options)
{
    BOOST_ASSERT(0);
    return 0xffff;
}

int CheckRampNDFeasibility(const std::vector<RampND>& rampndVect, FeasibilityCheckerBase* feas, const std::vector<dReal>& tol, int options)
{
    BOOST_ASSERT(0);
    return 0xffff;
}

RampNDFeasibilityChecker::RampNDFeasibilityChecker(FeasibilityCheckerBase* _feas) : feas(_feas), tol(0), distance(NULL), maxiter(0), constraintmask(0) {
}

RampNDFeasibilityChecker::RampNDFeasibilityChecker(FeasibilityCheckerBase* _feas, DistanceCheckerBase* _dist, int _maxiter) : feas(_feas), tol(0), distance(_dist), maxiter(_maxiter), constraintmask(0) {
}

int RampNDFeasibilityChecker::Check(const std::vector<RampND>& rampndVect, int options)
{
    if ((options & constraintmask) == constraintmask) {
        FOREACH(itrampnd, rampndVect) {
            itrampnd->constraintChecked = true;
        }
    }
    if( distance ) {
        return CheckRampNDFeasibility(rampndVect, feas, distance, maxiter, options);
    }
    else {
        return CheckRampNDFeasibility(rampndVect, feas, tol, options);
    }
}

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
