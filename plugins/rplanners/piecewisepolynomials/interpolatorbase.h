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
    virtual ~InterpolatorBase()
    {
    }

    virtual const std::string GetXMLId() const
    {
        return "interpolatorbase";
    }

    virtual const std::string& GetDescription() const {
        return __description;
    }

    //
    // Functions
    //
    /// \brief Initialize this interpolator
    virtual void Initialize(size_t ndof_, int envid_=0)
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
                                                                                          PiecewisePolynomial& pwpoly)
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
                                                                                               PiecewisePolynomial& pwpoly)
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
                                                                                           PiecewisePolynomial& pwpoly)
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

    void ConvertPiecewisePolynomialsToChunks(const std::vector<PiecewisePolynomial>& pwpolynomialsIn, std::vector<Chunk>& chunksOut)
    {
        std::vector<dReal>& switchPointsList = _switchPointsList; // store the time instants where any dofs switches polynomials
        switchPointsList.resize(0);

        switchPointsList.push_back(0);
        dReal maxDuration = 0;
        for( std::vector<PiecewisePolynomial>::const_iterator itpwpoly = pwpolynomialsIn.begin(); itpwpoly != pwpolynomialsIn.end(); ++itpwpoly ) {
            if( itpwpoly->GetDuration() > maxDuration ) {
                maxDuration = itpwpoly->GetDuration();
            }
        }
        switchPointsList.push_back(maxDuration);

        // Collect all remaining switch points
        for( std::vector<PiecewisePolynomial>::const_iterator itpwpoly = pwpolynomialsIn.begin(); itpwpoly != pwpolynomialsIn.end(); ++itpwpoly ) {
            dReal sw = 0;
            for( std::vector<Polynomial>::const_iterator itpoly = itpwpoly->GetPolynomials().begin(); itpoly != itpwpoly->GetPolynomials().end(); ++itpoly ) {
                sw += itpoly->duration;
                std::vector<dReal>::iterator itsw = std::lower_bound(switchPointsList.begin(), switchPointsList.end(), sw);

                // Since we already have t = 0 and t = duration in switchPointsList, itsw must point to some value
                // between *(switchPointsList.begin()) and *(switchPointsList.end() - 1) (exclusive).

                // Note also that skipping some switchpoints which are closer to their neighbors than
                // g_fPolynomialEpsilon may introduce discrepancies greater than g_fPolynomialEpsilon.
                if( !FuzzyEquals(sw, *itsw, g_fEpsilonForTimeInstant) && !FuzzyEquals(sw, *(itsw - 1), g_fEpsilonForTimeInstant) ) {
                    switchPointsList.insert(itsw, sw);
                }
            }
        }

        std::vector<Polynomial>& vpolynomials = _vpolynomials; // used for Chunk initialization

        chunksOut.resize(switchPointsList.size() - 1);
        for( size_t iswitch = 1; iswitch < switchPointsList.size(); ++iswitch ) {
            dReal t1 = switchPointsList[iswitch], t0 = switchPointsList[iswitch - 1];
            dReal duration = t1 - t0;

            vpolynomials.resize(0);
            vpolynomials.reserve(ndof);
            for( std::vector<PiecewisePolynomial>::const_iterator itpwpoly = pwpolynomialsIn.begin(); itpwpoly != pwpolynomialsIn.end(); ++itpwpoly ) {
                vpolynomials.push_back( itpwpoly->ExtractPolynomial(t0, t1) );
            }
            chunksOut[iswitch - 1].Initialize(duration, vpolynomials);
        }
    }

    //
    // Members
    //
    size_t ndof;
    int envid;

    std::string __description;

private:
    // For use in ConvertPiecewisePolynomialsToChunks
    std::vector<dReal> _switchPointsList; // store the time instants where any dofs switches polynomials
    std::vector<Polynomial> _vpolynomials;
};

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE

#endif
