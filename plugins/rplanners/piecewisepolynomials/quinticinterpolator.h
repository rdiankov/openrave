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

#include "interpolatorbase.h"
#include "generalrecursiveinterpolator.h"

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

class QuinticInterpolator : public InterpolatorBase {
public:
    /*
     */
    QuinticInterpolator(size_t ndof, int envid=0);
    ~QuinticInterpolator()
    {
    }

    virtual const std::string GetXMLId() const
    {
        return "quinticinterpolator";
    }

    //
    // Functions
    //
    virtual void Initialize(size_t ndof, int envid=0) override;

    virtual PolynomialCheckReturn Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(dReal x0, dReal x1,
                                                                                          dReal vm, dReal am, dReal jm,
                                                                                          PiecewisePolynomial& pwpoly) override;

    virtual PolynomialCheckReturn Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal a0, dReal a1,
                                                                                               dReal xmin, dReal xmax, dReal vm, dReal am, dReal jm,
                                                                                               PiecewisePolynomial& pwpoly) override;

    virtual PolynomialCheckReturn Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal a0, dReal a1, dReal T,
                                                                                           dReal xmin, dReal xmax, dReal vm, dReal am, dReal jm,
                                                                                           PiecewisePolynomial& pwpoly) override;

    virtual PolynomialCheckReturn ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                          const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                          std::vector<Chunk>& chunks) override;

    virtual PolynomialCheckReturn ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                           const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                                           const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect, const dReal T,
                                                                                           const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                                           const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                           std::vector<Chunk>& chunks) override;

    /*
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
     */
    virtual PolynomialCheckReturn ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                               const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                                               const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect,
                                                                                               const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                                               const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                               const dReal T, std::vector<Chunk>& chunks) override;

    //
    // Members
    //
    PolynomialChecker checker;
    GeneralRecursiveInterpolatorPtr _pGeneralInterpolator;

    const dReal _fifteenOverEight = 1.875;
    const dReal _tenOverSqrtThree = 10/Sqrt(3);

    std::vector<dReal> _cache1DCoeffs;
    std::vector<dReal> _cacheDVect;
    std::vector<Polynomial> _cachePolynomials;
    Polynomial _cachePolynomial;
    PiecewisePolynomial _cachePWPolynomial;
    std::vector<Chunk> _cacheChunks;
};

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE

#endif
