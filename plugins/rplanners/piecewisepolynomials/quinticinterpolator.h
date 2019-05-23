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
#ifndef PIECEWISE_POLY_QUINTIC_INTERPOLATOR_H
#define PIECEWISE_POLY_QUINTIC_INTERPOLATOR_H

#include "polynomialtrajectory.h"

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

class QuinticInterpolator {
public:
    /*
     */
    QuinticInterpolator()
    {
    }
    QuinticInterpolator(size_t ndof, int envid=0);
    ~QuinticInterpolator()
    {
    }

    //
    // Functions
    //
    /// \brief
    void Initialize(size_t ndof, int envid=0);

    /*
       \brief Compute a 1d quintic polynomial that interpolates x0 and x1. The duration is assigned
              such that the trajectory is as fast as possible while respecting all the given
              velocity, acceleration, and jerk bounds. Since the trajectory is a straight line in
              joint space, there is no need to check for joint limits, assuming that the given
              initial and final positions are within the limits.

       \params x0 initial position
       \params x1 final position
       \params vm velocity limit
       \params am acceleration limit
       \params jm jerk limit
     */
    void Compute1DTrajectoryZeroTimeDerivativesOptimizeDuration(dReal x0, dReal x1, dReal vm, dReal am, dReal jm, Polynomial& p, dReal& T);

    /*
       \brief Compute a 1d quintic polynomial that interpolates (x0, v0, a0) and (x1, v1, a1) and
             that has the given duration. Since there is a unique polynomial that interpolates the
             given boundary conditions, we do not perform checking of limits here.

       \params x0 initial position
       \params x1 final position
       \params v0 initial velocity
       \params v1 final velocity
       \params a0 initial acceleration
       \params a1 final acceleration
       \params T the duration
     */
    void Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal a0, dReal a1, dReal T, Polynomial& p);

    /*
       \brief Compute an nd quintic polynomial that interpolates x0Vect and x1Vect. The problem is
              essentially reduced to a 1d interpolation problem where the velocity, acceleration,
              and jerk limits are determined by the limiting dof.

          Consider the following *path* in joint space

                  xVect(s) = x0Vect + s(x1Vect - x0Vect),

          where the path parameters s is in the range [0, 1]. The *time* derivatives of the
          function are then

                  vVect(s) = sd(x1Vect - x0Vect),
          aVect(s) = sdd(x1Vect - x0Vect),
          jVect(s) = sddd(x1Vect - x0Vect).

          One can see that, for example, the velocity bound sd_max is determined by

                  sd_max = min_{i=1,...,dof} abs(vmVect[i]/dVect[i]),

          where dVect = x1Vect - x0Vect.
     */
    void ComputeNDTrajectoryZeroTimeDerivativesOptimizeDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect, Chunk& chunk);

    /*
       \brief Compute an nd quintic polynomial that interpolates (x0Vect, v0Vect, a0Vect) and
              (x1Vect, v1Vect, a1Vect) and that has the given duration. This essentially amounts to
              solving n independent 1d problems.

       \params x0Vect initial position
       \params x1Vect final position
       \params v0Vect initial velocity
       \params v1Vect final velocity
       \params a0Vect initial acceleration
       \params a1Vect final acceleration
       \params T
     */
    void ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect, dReal T, Chunk& chunk);

    /*
       \brief First compute an nd quintic polynomial that satisfies the given boundary conditions
             with the least duration while satisfying all constraints.

         pseudocode:
         curDuration = T
         traj = ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(..., curDuration)
         if traj is feasible:
             stepSize = curDuration/2
             while stepSize > epsilon:
             testDuration = curDuration - stepSize
             traj = ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(..., testDuration)
             if traj is feasible:
                 curDuration = testDuration
             stepsize /= 2
         else:
             traj = NULL
         return traj

       \params x0Vect initial position
       \params x1Vect final position
       \params v0Vect initial velocity
       \params v1Vect final velocity
       \params a0Vect initial acceleration
       \params a1Vect final acceleration
       \params xminVect lower position limits
       \params xmaxVect upper position limits
       \params vmVect velocity limits
       \params amVect acceleration limits
       \params jmVect jerk limits
       \params T the initial value of the duration
     */
    bool ComputeNDTrajectoryArbitraryTimeDerivativesOptimizeDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect, const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect, dReal T, Chunk& chunk);

    //
    // Members
    //
    size_t ndof;
    int envid;

    static const dReal _fifteenOverEight;
    static const dReal _tenOverSqrtThree;

    std::vector<dReal> _cache1DCoeffs;
    std::vector<dReal> _cacheDVect;
    std::vector<Polynomial> _cachePolynomialsVect;
    Polynomial _cachePolynomial;
};

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE

#endif
