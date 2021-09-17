// -*- coding: utf-8 -*-
// Copyright (C) 2021 Puttichai Lertkultanon
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
#ifndef PIECEWISE_POLY_INTERPOLATOR_BASE_H
#define PIECEWISE_POLY_INTERPOLATOR_BASE_H

#include "polynomialtrajectory.h"
#include "polynomialchecker.h"

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

class InterpolatorBase;
typedef boost::shared_ptr<InterpolatorBase> InterpolatorBasePtr;

class InterpolatorBase {
public:
    /*
     */
    InterpolatorBase()
    {
    }
    InterpolatorBase(size_t ndof, int envid=0);
    ~InterpolatorBase()
    {
    }

    //
    // Functions
    //
    /// \brief Initialize this interpolator
    virtual void Initialize(size_t ndof, int envid=0)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("Initialize not implemented", ORE_NotImplemented);
    }

    //
    // 1D problems
    //

    /*
       \brief Compute a 1d polynomial that interpolates x0 and x1. The duration is assigned
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
    virtual PolynomialCheckReturn Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(dReal x0, dReal x1,
                                                                                          dReal vm, dReal am, dReal jm,
                                                                                          std::vector<Polynomial>& polynomials, dReal& T)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("Compute1DTrajectoryZeroTimeDerivativesOptimizeDuration not implemented", ORE_NotImplemented);
    }

    /*
       \brief Compute a 1d polynomial that interpolates (x0, v0, a0) and (x1, v1, a1). The duration
              should computed to be as fast as possible while respecting all the given velocity,
              acceleration, and jerk bounds.

       \params x0 initial position
       \params x1 final position
       \params v0 initial velocity
       \params v1 final velocity
       \params a0 initial acceleration
       \params a1 final acceleration
       \params T the duration
     */
    virtual PolynomialCheckReturn Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal a0, dReal a1,
                                                                                               dReal xmin, dReal xmax, dReal vm, dReal am, dReal jm,
                                                                                               std::vector<Polynomial>& polynomials, dReal& T)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration not implemented", ORE_NotImplemented);
    }

    /*
       \brief Compute a 1d polynomial that interpolates (x0, v0, a0) and (x1, v1, a1) and that has
              the given duration while respecting all the given velocity, acceleration, and jerk
              bounds.

       \params x0 initial position
       \params x1 final position
       \params v0 initial velocity
       \params v1 final velocity
       \params a0 initial acceleration
       \params a1 final acceleration
       \params T the duration
     */
    virtual PolynomialCheckReturn Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal a0, dReal a1, dReal T,
                                                                                           dReal xmin, dReal xmax, dReal vm, dReal am, dReal jm,
                                                                                           std::vector<Polynomial>& polynomials)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration not implemented", ORE_NotImplemented);
    }

    //
    // ND problems
    //

    /*
       \brief Compute an nd chunk that interpolates x0Vect and x1Vect. The problem is
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
    virtual PolynomialCheckReturn ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                          const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                          std::vector<Chunk>& chunks)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("ComputeNDTrajectoryZeroTimeDerivativesOptimizeDuration not implemented", ORE_NotImplemented);
    }

    /*
       \brief Compute an nd chunk that interpolates (x0Vect, v0Vect, a0Vect) and
              (x1Vect, v1Vect, a1Vect) and that has the given duration. This essentially amounts to
              solving n independent 1d problems.

       \params x0Vect initial position
       \params x1Vect final position
       \params v0Vect initial velocity
       \params v1Vect final velocity
       \params a0Vect initial acceleration
       \params a1Vect final acceleration
       \params T the required fixed duration
     */
    virtual PolynomialCheckReturn ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                           const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                                           const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect, const dReal T,
                                                                                           const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                                           const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                           std::vector<Chunk>& chunks)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration not implemented", ORE_NotImplemented);
    }

    /*
       \brief First compute an nd chunk that satisfies the given boundary conditions
             with the least duration while satisfying all constraints.

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
       \params T the initial guess of the duration (may be used or not used depending on the underlying method)
     */
    virtual PolynomialCheckReturn ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                               const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                                               const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect,
                                                                                               const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                                               const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                               const dReal T, std::vector<Chunk>& chunks)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("ComputeNDTrajectoryArbitraryTimeDerivativesOptimizeDuration not implemented", ORE_NotImplemented);
    }

    //
    // Members
    //
    size_t ndof;
    int envid;
};

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE

#endif
