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
#ifndef PIECEWISE_POLY_POLY_CHECKER_H
#define PIECEWISE_POLY_POLY_CHECKER_H

#include "polynomialtrajectory.h"

#define JERK_LIMITED_POLY_CHECKER_DEBUG

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

enum PolynomialCheckReturn : uint8_t {
    PCR_Normal = 0,
    PCR_PositionLimitsViolation = 1,     ///< local extrema of polynomial in the given range violate position limits
    PCR_VelocityLimitsViolation = 2,     ///< local extrema of polynomial in the given range violate velocity limits
    PCR_AccelerationLimitsViolation = 3, ///< local extrema of polynomial in the given range violate acceleration limits
    PCR_JerkLimitsViolation = 4,         ///< local extrema of polynomial in the given range violate jerk limits
    PCR_NegativeDuration = 5,            ///< the maximum of the range in which polynomial is defined is negative
    PCR_PositionDiscrepancy = 6,         ///< a terminal position (initial/final) is different from the given value
    PCR_VelocityDiscrepancy = 7,         ///< a terminal velocity (initial/final) is different from the given value
    PCR_AccelerationDiscrepancy = 8,     ///< a terminal acceleration (initial/final) is different from the given value
    PCR_DurationDiscrepancy = 9,         ///< polynomial is defined in a range different from the given range
    PCR_DurationTooLong = 10,            ///< the duration is too long so the polynomial will not be useful as a shortcut
    PCR_GenericError = 0x10,
};

const char* GetPolynomialCheckReturnString(PolynomialCheckReturn ret);

class PolynomialChecker {
public:
    PolynomialChecker()
    {
    }
    PolynomialChecker(size_t ndof, int envid);
    ~PolynomialChecker()
    {
    }

    void Initialize(size_t ndof, int envid);

    inline dReal GetEpsilonForPositionDiscrepancyChecking() const {
        return epsilonForPositionDiscrepancyChecking;
    }
    inline dReal GetEpsilonForVelocityDiscrepancyChecking() const {
        return epsilonForVelocityDiscrepancyChecking;
    }
    inline dReal GetEpsilonForAccelerationDiscrepancyChecking() const {
        return epsilonForAccelerationDiscrepancyChecking;
    }
    inline dReal GetEpsilonForJerkLimitsChecking() const {
        return epsilonForJerkLimitsChecking;
    }
    inline void SetEpsilonForAccelerationDiscrepancyChecking(const dReal epsilon) {
        epsilonForAccelerationDiscrepancyChecking = epsilon;
    }
    inline void SetEpsilonForJerkLimitsChecking(const dReal epsilon) {
        epsilonForJerkLimitsChecking = epsilon;
    }

    /// \brief Check if the input polynomial is consistent and respects all limits
    ///
    /// \param[in] p the input polynomial
    /// \param[in] xmin the lower position limit
    /// \param[in] xmax the upper position limit
    /// \param[in] vm the velocity limit. If zero, then skip checking. Otherwise, vm must be strictly positive.
    /// \param[in] am the acceleration limit. If zero, then skip checking. Otherwise, am must be strictly positive.
    /// \param[in] jm the jerk limit. If zero, then skip checking. Otherwise, jm must be strictly positive.
    /// \param[in] x0 the expected position at the first point
    /// \param[in] x1 the expected position at the last point
    /// \param[in] v0 the expected velocity at the first point
    /// \param[in] v1 the expected velocity at the last point
    /// \param[in] a0 the expected acceleration at the first point
    /// \param[in] a1 the expected acceleration at the last point
    PolynomialCheckReturn CheckPolynomial(const Polynomial& p, const dReal xmin, const dReal xmax, const dReal vm, const dReal am, const dReal jm,
                                          const dReal x0, const dReal x1, const dReal v0, const dReal v1, const dReal a0, const dReal a1)
    {
        PolynomialCheckReturn ret = PCR_Normal;

        ret = CheckPolynomialValues(p, 0, x0, v0, a0);
        if( ret != PCR_Normal ) {
            return ret;
        }
        ret = CheckPolynomialValues(p, p.duration, x1, v1, a1);
        if( ret != PCR_Normal ) {
            return ret;
        }
        ret = CheckPolynomialLimits(p, xmin, xmax, vm, am, jm);
        return ret;
    }

    /// \brief Check if the input polynomial values evaluated at time t is consistent with the given conditions
    PolynomialCheckReturn CheckPolynomialValues(const Polynomial& p, const dReal t, const dReal x, const dReal v, const dReal a);

    /// \brief Check if the input polynomial respects all the limits
    PolynomialCheckReturn CheckPolynomialLimits(const Polynomial& p, const dReal xmin, const dReal xmax, const dReal vm, const dReal am, const dReal jm);

    /// \brief Check if the input piecewise polynomial is consistent and respects all limits
    PolynomialCheckReturn CheckPiecewisePolynomial(const PiecewisePolynomial& p, const dReal xmin, const dReal xmax, const dReal vm, const dReal am, const dReal jm,
                                                   const dReal x0, const dReal x1, const dReal v0, const dReal v1, const dReal a0, const dReal a1);

    /// \brief Check if the input chunk is consistent and respects all limits
    PolynomialCheckReturn CheckChunk(const Chunk& c, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                     const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                     const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                     const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                     const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect);

    /// \brief Check if the input chunk evaluates to the given values at time t.
    PolynomialCheckReturn CheckChunkValues(const Chunk& c, dReal t, const std::vector<dReal>& xVect, const std::vector<dReal>& vVect, const std::vector<dReal>& aVect);

    /// \brief Check if the input chunk respects all limits
    PolynomialCheckReturn CheckChunkLimits(const Chunk& c, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                           const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect);

    /// \brief Check if the input sequence of chunks is consistent and respects all limits
    PolynomialCheckReturn CheckChunks(const std::vector<Chunk>& chunks, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                      const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                      const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                      const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                      const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect);

    /// \brief Check if the input trajectory is consistent and respects all limits
    PolynomialCheckReturn CheckPiecewisePolynomialTrajectory(const PiecewisePolynomialTrajectory& traj, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                             const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                             const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                             const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                             const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect);

    //
    // Members
    //
    size_t ndof;
    int envid;

    std::vector<Coordinate> _cacheCoordsVect;
    std::vector<dReal> _cacheXVect, _cacheVVect, _cacheAVect;

#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
    dReal _failedPoint;
    dReal _failedValue;
    dReal _expectedValue;
    size_t _failedIndex;
    size_t _failedDOF;
#endif

private:
    // Specific tolerance for checking discrepancies.
    dReal epsilonForPositionDiscrepancyChecking = g_fPolynomialEpsilon;
    dReal epsilonForVelocityDiscrepancyChecking = g_fPolynomialEpsilon;
    dReal epsilonForAccelerationDiscrepancyChecking = g_fPolynomialEpsilon;

    dReal epsilonForJerkLimitsChecking = g_fPolynomialEpsilon;

}; // end class PolynomialChecker

typedef boost::shared_ptr<PolynomialChecker> PolynomialCheckerPtr;

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
#endif
