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
#ifndef PIECEWISE_POLY_GENERAL_INTERPOLATOR_H
#define PIECEWISE_POLY_GENERAL_INTERPOLATOR_H

#include "polynomialtrajectory.h"
#include "polynomialchecker.h"
#include "../rampoptimizer/interpolator.h"
#include "../rampoptimizer/parabolicchecker.h"

namespace OpenRAVE {

namespace RampOptimizer = RampOptimizerInternal;

namespace PiecewisePolynomialsInternal {

class GeneralRecursiveInterpolator {
public:
    GeneralRecursiveInterpolator(int envid=0);
    ~GeneralRecursiveInterpolator()
    {
    }

    void Initialize(int envid=0);

    /// \brief Compute a parabolic trajectory that interpolates (x0, v0) and (x1, v1), respects up
    ///        to acceleration limits, and has the shortest possible duration.
    PolynomialCheckReturn ComputeParabolic1DTrajectoryOptimizedDuration(const dReal x0, const dReal x1,
                                                                        const dReal v0, const dReal v1,
                                                                        const dReal xmin, const dReal xmax,
                                                                        const dReal vm, const dReal am,
                                                                        PiecewisePolynomial& pwpoly);

    /// \brief Compute a parabolic trajectory that interpolates (x0, v0) and (x1, v1), respects up
    ///        to acceleration limits, and has the duration fixedDuration.
    PolynomialCheckReturn ComputeParabolic1DTrajectoryFixedDuration(const dReal x0, const dReal x1,
                                                                    const dReal v0, const dReal v1,
                                                                    const dReal xmin, const dReal xmax,
                                                                    const dReal vm, const dReal am,
                                                                    const dReal fixedDuration,
                                                                    PiecewisePolynomial& pwpoly);

    /// \brief Ensure that the input parabolic curve satisfies the given position limits and other
    ///        constraints. Then convert the validated parabolic curve into a piecewise polynomial
    ///        instance.
    ///
    /// \param[in] curve The input parabolic curve
    /// \param[out] pwpoly The output piecewise polynomial
    PolynomialCheckReturn PostProcessParabolic1DTrajectory(RampOptimizer::ParabolicCurve& curve,
                                                           const dReal x0, const dReal x1,
                                                           const dReal v0, const dReal v1,
                                                           const dReal xmin, const dReal xmax,
                                                           const dReal vm, const dReal am,
                                                           PiecewisePolynomial& pwpoly);

    void ConvertParabolicCurveToPiecewisePolynomial(const RampOptimizer::ParabolicCurve& curve, PiecewisePolynomial& pwpoly);

    /// \brief Compute a piecewise polynomial that satisfies all the given conditions. The
    ///        implementation follows Ezair, B., Tassa, T. and Shiller, Z., 2014. Planning high
    ///        order trajectories with general initial and final conditions and asymmetric
    ///        bounds. The International Journal of Robotics Research, 33(6), pp.898-916.
    ///
    /// \param[in] degree The degree of the output piecewise polynomial trajectory.
    /// \param[in] initialState The initial state the output polynomial must start with. This state contains up to the (degree - 1)-th derivative.
    /// \param[in] finalState The final state the output polynomial must end with. This state contains up to the (degree - 1)-th derivative.
    /// \param[in] lowerBounds The lower bounds that the output polynomial must satisfy. This bounds contain up to the bound for (degree)-th derivative.
    /// \param[in] upperBounds The upper bounds that the output polynomial must satisfy. This bounds contain up to the bound for (degree)-th derivative.
    /// \param[in] fixedDuration If > 0, the output piecewise polynomial must have this as its total duration.
    /// \param[out] pwpoly The output piecewise polynomial
    /// \return one of PolynomialCheckReturn
    PolynomialCheckReturn Compute1DTrajectory(const size_t degree,
                                              const std::vector<dReal>& initialState, const std::vector<dReal>& finalState,
                                              const std::vector<dReal>& lowerBounds, const std::vector<dReal>& upperBounds,
                                              const dReal fixedDuration,
                                              PiecewisePolynomial& pwpoly);

    //
    // Members
    //
    std::string __description;

    int envid;
    PolynomialChecker checker;
    RampOptimizer::ParabolicInterpolator parabolicInterpolator;

    const size_t positionIndex = 0;
    const size_t velocityIndex = 1;
    const size_t accelerationIndex = 2;

    // Use tighter epsilon for checking convergence while still using g_fPolynomialEpsilon for checking zero duration.
    const dReal epsilon = 1e-5*g_fPolynomialEpsilon;
    const dReal epsilonFinalValidation = 100*epsilon;

private:
    std::vector<Polynomial> _cachePolynomials; // cached vector of polynomials for use in ConvertParabolicCurveToPiecewisePolynomial
    std::vector<dReal> _cacheParabolicCoeffs; // for use in ConvertParabolicCurveToPiecewisePolynomial only. This is to reduce the number of calls to destructor of std::vector.
    RampOptimizer::ParabolicCurve _cacheParabolicCurve; // for use in ComputeParabolic1DTrajectoryX only. This is to reduce the number of calls to destructor of ParabolicCurve.

    std::vector<Polynomial> _cacheFinalPolynomials;
    PiecewisePolynomial _cacheFinalPWPoly;

}; // end class GeneralRecursiveInterpolator

typedef boost::shared_ptr<GeneralRecursiveInterpolator> GeneralRecursiveInterpolatorPtr;

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE

#endif
