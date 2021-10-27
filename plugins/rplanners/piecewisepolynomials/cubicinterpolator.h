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
#ifndef PIECEWISE_POLY_CUBIC_INTERPOLATOR_H
#define PIECEWISE_POLY_CUBIC_INTERPOLATOR_H

#include "interpolatorbase.h"
#include "generalrecursiveinterpolator.h"

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

class CubicInterpolator : public InterpolatorBase {
public:
    /*
     */
    CubicInterpolator(size_t ndof, int envid=0);
    ~CubicInterpolator()
    {
    }

    virtual const std::string GetXMLId() const
    {
        return "cubicinterpolator";
    }

    //
    // Functions
    //
    /// \brief Initialize this interpolator
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

    virtual PolynomialCheckReturn ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                                               const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                                               const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect,
                                                                                               const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                                               const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                                               const dReal T, std::vector<Chunk>& chunks) override;

    // Utility functions for 1D zero time derivatives case
    int DetermineZeroTimeDerivativesCaseFromBoundaryConditions(const dReal d, const dReal vm, const dReal am, const dReal jm);

    //
    // Members
    //
    PolynomialChecker checker;
    GeneralRecursiveInterpolatorPtr _pGeneralInterpolator;

    std::vector<dReal> _cache1DCoeffs;
    std::vector<dReal> _cacheDVect, _cacheXVect;
    std::vector<Polynomial> _cachePolynomials;
    PiecewisePolynomial _cachePWPolynomial;
    std::vector<PiecewisePolynomial> _cachePWPolynomials;
};

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE

#endif
