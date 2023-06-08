// -*- coding: utf-8 -*-
// Copyright (C) 2019 Puttichai Lertkultanon
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
#ifndef PIECEWISE_POLY_FEAS_CHECKER_H
#define PIECEWISE_POLY_FEAS_CHECKER_H

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

struct CheckReturn {
    CheckReturn(int retcode_=0, dReal fmult=1.0, dReal fvel=0, dReal faccel=0) : retcode(retcode_), fTimeBasedSurpassMult(fmult), bDifferentVelocity(false), fMaxManipSpeed(fvel), fMaxManipAccel(faccel) {
    }

    int retcode; ///< one of CFO_X defined in planner.h
    dReal fTimeBasedSurpassMult; ///< if retcode == CFO_CheckTimeBasedConstraints, then the multiplier is set to (some factor)*|max|/|actual max|
    bool bDifferentVelocity; ///< the segment ends with some velocity other than the desired value (resulting from modifications CheckPathAllConstraints)
    dReal fMaxManipSpeed; ///< when > 0, the value is the computed max manip speed
    dReal fMaxManipAccel; ///< when > 0, the value is the computed max manip accel
};

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
#endif
