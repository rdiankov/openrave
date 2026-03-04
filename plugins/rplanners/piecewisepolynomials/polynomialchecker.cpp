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
#include "polynomialchecker.h"

namespace OpenRAVE {

namespace PiecewisePolynomialsInternal {

const char* GetPolynomialCheckReturnString(PolynomialCheckReturn ret)
{
    switch(ret) {
    case PCR_Normal: return "Normal";
    case PCR_PositionLimitsViolation: return "PositionLimitsViolation";
    case PCR_VelocityLimitsViolation: return "VelocityLimitsViolation";
    case PCR_AccelerationLimitsViolation: return "AccelerationLimitsViolation";
    case PCR_JerkLimitsViolation: return "JerkLimitsViolation";
    case PCR_NegativeDuration: return "NegativeDuration";
    case PCR_PositionDiscrepancy: return "PositionDiscrepancy";
    case PCR_VelocityDiscrepancy: return "VelocityDiscrepancy";
    case PCR_AccelerationDiscrepancy: return "AccelerationDiscrepancy";
    case PCR_DurationDiscrepancy: return "DurationDiscrepancy";
    case PCR_DurationTooLong: return "DurationTooLong";
    case PCR_GenericError: return "GenericError";
    }
    return "PCR(Unknown)";
}

PolynomialChecker::PolynomialChecker(size_t ndof_, int envid_) : ndof(ndof_), envid(envid_)
{
    _cacheCoordsVect.reserve(4); // quintic polynomial has at most 4 extrema
    _cacheXVect.resize(ndof);
    _cacheVVect.resize(ndof);
    _cacheAVect.resize(ndof);
}

void PolynomialChecker::Initialize(size_t ndof_, int envid_)
{
    this->ndof = ndof_;
    this->envid = envid_;
    if( _cacheCoordsVect.capacity() < 4 ) {
        _cacheCoordsVect.reserve(4); // quintic polynomial has at most 4 extrema
    }
    _cacheXVect.resize(ndof_);
    _cacheVVect.resize(ndof_);
    _cacheAVect.resize(ndof_);
}

PolynomialCheckReturn PolynomialChecker::CheckPolynomialValues(const Polynomial& p, const dReal t, const dReal x, const dReal v, const dReal a)
{
    if( t > p.duration + g_fPolynomialEpsilon ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
        _failedPoint = t;
        _failedValue = t;
        _expectedValue = p.duration;
#endif
        return PCR_DurationDiscrepancy;
    }
    dReal pos = p.Eval(t);
    if( !FuzzyEquals(pos, x, epsilonForPositionDiscrepancyChecking) ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
        _failedPoint = t;
        _failedValue = pos;
        _expectedValue = x;
#endif
        return PCR_PositionDiscrepancy;
    }

    dReal vel = p.Evald1(t);
    if( !FuzzyEquals(vel, v, epsilonForVelocityDiscrepancyChecking) ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
        _failedPoint = t;
        _failedValue = vel;
        _expectedValue = v;
#endif
        return PCR_VelocityDiscrepancy;
    }

    dReal accel = p.Evald2(t);
    if( !FuzzyEquals(accel, a, epsilonForAccelerationDiscrepancyChecking) ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
        _failedPoint = t;
        _failedValue = accel;
        _expectedValue = a;
#endif
        return PCR_AccelerationDiscrepancy;
    }

    return PCR_Normal;
}

PolynomialCheckReturn PolynomialChecker::CheckPolynomialLimits(const Polynomial& p, const dReal xmin, const dReal xmax, const dReal vm, const dReal am, const dReal jm)
{
    std::vector<Coordinate>& vcoords = _cacheCoordsVect;
    const dReal T = p.duration;
    const bool bCheckVelocity = p.degree > 0 && vm > g_fPolynomialEpsilon;
    const bool bCheckAcceleration = p.degree > 1 && am > g_fPolynomialEpsilon;
    const bool bCheckJerk = p.degree > 2 && jm > g_fPolynomialEpsilon;

    dReal val; // for holding a temporary value

    // Check position limits at boundaries
    val = p.Eval(0);
    if( val > xmax + g_fPolynomialEpsilon || val < xmin - g_fPolynomialEpsilon ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
        _failedPoint = 0;
        _failedValue = val;
        _expectedValue = val > xmax ? xmax : xmin;
#endif
        return PCR_PositionLimitsViolation;
    }
    val = p.Eval(T);
    if( val > xmax + g_fPolynomialEpsilon || val < xmin - g_fPolynomialEpsilon ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
        _failedPoint = T;
        _failedValue = val;
        _expectedValue = val > xmax ? xmax : xmin;
#endif
        return PCR_PositionLimitsViolation;
    }

    // Check velocity limits at boundaries
    if( bCheckVelocity ) {
        val = p.Evald1(0);
        if( val > vm + g_fPolynomialEpsilon || val < -vm - g_fPolynomialEpsilon ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedPoint = 0;
            _failedValue = val;
            _expectedValue = val > vm ? vm : -vm;
#endif
            return PCR_VelocityLimitsViolation;
        }
        val = p.Evald1(T);
        if( val > vm + g_fPolynomialEpsilon || val < -vm - g_fPolynomialEpsilon ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedPoint = T;
            _failedValue = val;
            _expectedValue = val > vm ? vm : -vm;
#endif
            return PCR_VelocityLimitsViolation;
        }
    }

    // Check acceleration limits at boundaries
    if( bCheckAcceleration ) {
        val = p.Evald2(0);
        if( val > am + g_fPolynomialEpsilon || val < -am - g_fPolynomialEpsilon ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedPoint = 0;
            _failedValue = val;
            _expectedValue = val > am ? am : -am;
#endif
            return PCR_AccelerationLimitsViolation;
        }
        val = p.Evald2(T);
        if( val > am + g_fPolynomialEpsilon || val < -am - g_fPolynomialEpsilon ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedPoint = T;
            _failedValue = val;
            _expectedValue = val > am ? am : -am;
#endif
            return PCR_AccelerationLimitsViolation;
        }
    }

    // Check jerk limits at boundaries
    if( bCheckJerk ) {
        val = p.Evald3(0);
        if( val > jm + epsilonForJerkLimitsChecking || val < -jm - epsilonForJerkLimitsChecking ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedPoint = 0;
            _failedValue = val;
            _expectedValue = val > jm ? jm : -jm;
#endif
            return PCR_JerkLimitsViolation;
        }
        val = p.Evald3(T);
        if( val > jm + epsilonForJerkLimitsChecking || val < -jm - epsilonForJerkLimitsChecking ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedPoint = T;
            _failedValue = val;
            _expectedValue = val > jm ? jm : -jm;
#endif
            return PCR_JerkLimitsViolation;
        }
    }

    // Now bounadries are ok. Check in-between values.
    // Check position limits
    for( std::vector<Coordinate>::const_iterator it = p.GetExtrema().begin(); it != p.GetExtrema().end(); ++it ) {
        if( it->point >= -g_fPolynomialEpsilon && it->point <= T + g_fPolynomialEpsilon ) {
            // This extremum occurs in the range
            if( it->value > xmax + g_fPolynomialEpsilon || it->value < xmin - g_fPolynomialEpsilon ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                _failedPoint = it->point;
                _failedValue = it->value;
                _expectedValue = it->value > xmax ? xmax : xmin;
#endif
                return PCR_PositionLimitsViolation;
            }
        }
    }

    // Check velocity limits
    if( bCheckVelocity ) {
        p.FindAllLocalExtrema(1, vcoords);
        for( std::vector<Coordinate>::const_iterator it = vcoords.begin(); it != vcoords.end(); ++it ) {
            if( it->point >= -g_fPolynomialEpsilon && it->point <= T + g_fPolynomialEpsilon ) {
                // This extremum occurs in the range
                if( it->value > vm + g_fPolynomialEpsilon || it->value < -vm - g_fPolynomialEpsilon ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                    _failedPoint = it->point;
                    _failedValue = it->value;
                    _expectedValue = it->value > vm ? vm : -vm;
#endif
                    return PCR_VelocityLimitsViolation;
                }
            }
        }
    }

    // Check acceleration limits
    if( bCheckAcceleration ) {
        p.FindAllLocalExtrema(2, vcoords);
        for( std::vector<Coordinate>::const_iterator it = vcoords.begin(); it != vcoords.end(); ++it ) {
            if( it->point >= -g_fPolynomialEpsilon && it->point <= T + g_fPolynomialEpsilon ) {
                // This extremum occurs in the range
                if( it->value > am + g_fPolynomialEpsilon || it->value < -am - g_fPolynomialEpsilon ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                    _failedPoint = it->point;
                    _failedValue = it->value;
                    _expectedValue = it->value > am ? am : -am;
#endif
                    return PCR_AccelerationLimitsViolation;
                }
            }
        }
    }

    // Check jerk limits
    if( bCheckJerk ) {
        p.FindAllLocalExtrema(3, vcoords);
        for( std::vector<Coordinate>::const_iterator it = vcoords.begin(); it != vcoords.end(); ++it ) {
            if( it->point >= -g_fPolynomialEpsilon && it->point <= T + g_fPolynomialEpsilon ) {
                // This extremum occurs in the range
                if( it->value > jm + epsilonForJerkLimitsChecking || it->value < -jm - epsilonForJerkLimitsChecking ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                    _failedPoint = it->point;
                    _failedValue = it->value;
                    _expectedValue = it->value > jm ? jm : -jm;
#endif
                    return PCR_JerkLimitsViolation;
                }
            }
        }
    }

    return PCR_Normal;
}

PolynomialCheckReturn PolynomialChecker::CheckPiecewisePolynomial(const PiecewisePolynomial& p, const dReal xmin, const dReal xmax, const dReal vm, const dReal am, const dReal jm,
                                                                  const dReal x0, const dReal x1, const dReal v0, const dReal v1, const dReal a0, const dReal a1)
{
    const std::vector<Polynomial>& vpolynomials = p.GetPolynomials();
    const size_t numPolynomials = vpolynomials.size();
    if( numPolynomials == 0 ) {
        return PCR_Normal;
    }

    PolynomialCheckReturn ret = PCR_Normal;

    // Check its initial and final values
    ret = CheckPolynomialValues(vpolynomials.at(0), 0, x0, v0, a0);
    if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
        _failedIndex = 0;
        RAVELOG_VERBOSE_FORMAT("polynomial index=%d/%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedIndex%numPolynomials%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
        return ret;
    }
    ret = CheckPolynomialValues(vpolynomials.at(numPolynomials - 1), vpolynomials.at(numPolynomials - 1).duration, x1, v1, a1);
    if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
        _failedIndex = numPolynomials - 1;
        RAVELOG_VERBOSE_FORMAT("polynomial index=%d/%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedIndex%numPolynomials%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
        return ret;
    }

    dReal prevPosition, prevVelocity, prevAcceleration;
    for( std::vector<Polynomial>::const_iterator itpoly = vpolynomials.begin(); itpoly != vpolynomials.end(); ++itpoly ) {
        // Check boundary values
        if( itpoly == vpolynomials.begin() ) {
            // For the first polynomial, just evaluate its final values. Its initial values have already been checked.
        }
        else {
            // For subsequent polynomials, check if their initial values match the final values of their preceeding ones.
            ret = CheckPolynomialValues(*itpoly, 0, prevPosition, prevVelocity, prevAcceleration);
            if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                _failedIndex = (itpoly - vpolynomials.begin());
                RAVELOG_VERBOSE_FORMAT("polynomials index=%d/%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedIndex%numPolynomials%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
                return ret;
            }
        }

        // Check limits
        ret = CheckPolynomialLimits(*itpoly, xmin, xmax, vm, am, jm);
        if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedIndex = (itpoly - vpolynomials.begin());
            RAVELOG_VERBOSE_FORMAT("polynomials index=%d/%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedIndex%numPolynomials%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
            return ret;
        }

        if( (itpoly + 1) != vpolynomials.end() ) {
            prevPosition = itpoly->Eval(itpoly->duration);
            prevVelocity = itpoly->Evald1(itpoly->duration);
            prevAcceleration = itpoly->Evald2(itpoly->duration);
        }
    }
    return PCR_Normal;
}

PolynomialCheckReturn PolynomialChecker::CheckChunk(const Chunk& c, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                    const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                    const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                    const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                    const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect)
{
    dReal vm = 0;
    dReal am = 0;
    dReal jm = 0;
    bool bHasVelocityLimits = vmVect.size() == ndof;
    bool bHasAccelerationLimits = amVect.size() == ndof;
    bool bHasJerkLimits = jmVect.size() == ndof;
    PolynomialCheckReturn ret = PCR_Normal;
    for( size_t idof = 0; idof < ndof; ++idof ) {
        if( bHasVelocityLimits ) {
            vm = vmVect[idof];
        }
        if( bHasAccelerationLimits ) {
            am = amVect[idof];
        }
        if( bHasJerkLimits ) {
            jm = jmVect[idof];
        }
        if( !FuzzyEquals(c.duration, c.vpolynomials[idof].duration, g_fPolynomialEpsilon) ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedValue = c.vpolynomials[idof].duration;
            _failedPoint = c.vpolynomials[idof].duration;
            _failedDOF = idof;
            _expectedValue = c.duration;
            RAVELOG_VERBOSE_FORMAT("idof=%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedDOF%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
            return PCR_DurationDiscrepancy;
        }
        ret = CheckPolynomial(c.vpolynomials[idof], xminVect[idof], xmaxVect[idof], vm, am, jm,
                              x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], a0Vect[idof], a1Vect[idof]);
        if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedDOF = idof;
            RAVELOG_VERBOSE_FORMAT("idof=%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedDOF%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
            break;
        }
    }
    return ret;
}

PolynomialCheckReturn PolynomialChecker::CheckChunkValues(const Chunk& c, dReal t,
                                                          const std::vector<dReal>& xVect, const std::vector<dReal>& vVect, const std::vector<dReal>& aVect)
{
    PolynomialCheckReturn ret = PCR_Normal;
    for( size_t idof = 0; idof < ndof; ++idof ) {
        ret = CheckPolynomialValues(c.vpolynomials[idof], t, xVect[idof], vVect[idof], aVect[idof]);
        if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedDOF = idof;
#endif
            return ret;
        }
    }
    return PCR_Normal;
}

PolynomialCheckReturn PolynomialChecker::CheckChunkLimits(const Chunk& c, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                          const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect)
{
    PolynomialCheckReturn ret = PCR_Normal;
    for( size_t idof = 0; idof < ndof; ++idof ) {
        ret = CheckPolynomialLimits(c.vpolynomials[idof], xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], jmVect[idof]);
        if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
            _failedDOF = idof;
#endif
            return ret;
        }
    }
    return PCR_Normal;
}

PolynomialCheckReturn PolynomialChecker::CheckPiecewisePolynomialTrajectory(const PiecewisePolynomialTrajectory& traj, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                                            const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                                            const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                                            const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                                            const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect)
{
    return CheckChunks(traj.vchunks, xminVect, xmaxVect, vmVect, amVect, jmVect,
                       x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect);
}

PolynomialCheckReturn PolynomialChecker::CheckChunks(const std::vector<Chunk>& vchunks, const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                                     const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect,
                                                     const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                                     const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                                     const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect)
{
    dReal vm = 0;
    dReal am = 0;
    dReal jm = 0;
    bool bHasVelocityLimits = vmVect.size() == ndof;
    bool bHasAccelerationLimits = amVect.size() == ndof;
    bool bHasJerkLimits = jmVect.size() == ndof;
    PolynomialCheckReturn ret = PCR_Normal;

    // Check boundary conditions
    ret = CheckChunkValues(vchunks.front(), 0, x0Vect, v0Vect, a0Vect);
    if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
        _failedIndex = 0;
        RAVELOG_VERBOSE_FORMAT("chunkIndex=%d/%d; idof=%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedIndex%vchunks.size()%_failedDOF%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
        return ret;
    }

    ret = CheckChunkValues(vchunks.back(), vchunks.back().duration, x1Vect, v1Vect, a1Vect);
    if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
        _failedIndex = vchunks.size() - 1;
        RAVELOG_VERBOSE_FORMAT("chunkIndex=%d/%d; idof=%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedIndex%vchunks.size()%_failedDOF%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
        return ret;
    }

    std::vector<dReal>& prevXVect = _cacheXVect;
    std::vector<dReal>& prevVVect = _cacheVVect;
    std::vector<dReal>& prevAVect = _cacheAVect;
    for( std::vector<Chunk>::const_iterator itchunk = vchunks.begin(); itchunk != vchunks.end(); ++itchunk ) {
        bool bCheckValues = false;
        if( itchunk != vchunks.begin() ) {
            bCheckValues = true;
        }

        for( size_t idof = 0; idof < ndof; ++idof ) {
            if( bHasVelocityLimits ) {
                vm = vmVect[idof];
            }
            if( bHasAccelerationLimits ) {
                am = amVect[idof];
            }
            if( bHasJerkLimits ) {
                jm = jmVect[idof];
            }

            // Check polynomial duration consistency
            if( !FuzzyEquals(itchunk->duration, itchunk->vpolynomials[idof].duration, g_fPolynomialEpsilon) ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                _failedPoint = itchunk->vpolynomials[idof].duration;
                _failedValue = itchunk->vpolynomials[idof].duration;
                _failedIndex = (itchunk - vchunks.begin());
                _failedDOF = idof;
                RAVELOG_VERBOSE_FORMAT("chunkIndex=%d/%d; idof=%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedIndex%vchunks.size()%_failedDOF%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
                return PCR_DurationDiscrepancy;
            }

            // Check polynomial boundary values
            if( bCheckValues ) {
                ret = CheckPolynomialValues(itchunk->vpolynomials[idof], 0, prevXVect[idof], prevVVect[idof], prevAVect[idof]);
                if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                    _failedDOF = idof;
                    _failedIndex = (itchunk - vchunks.begin());
                    RAVELOG_VERBOSE_FORMAT("chunkIndex=%d/%d; idof=%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedIndex%vchunks.size()%_failedDOF%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
                    return ret;
                }
            }

            // Check polynomial limits
            ret = CheckPolynomialLimits(itchunk->vpolynomials[idof], xminVect[idof], xmaxVect[idof], vm, am, jm);
            if( ret != PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                _failedDOF = idof;
                _failedIndex = (itchunk - vchunks.begin());
                RAVELOG_VERBOSE_FORMAT("chunkIndex=%d/%d; idof=%d; t=%.15f; value=%.15f; expected=%.15f; result=%s", _failedIndex%vchunks.size()%_failedDOF%_failedPoint%_failedValue%_expectedValue%GetPolynomialCheckReturnString(ret));
#endif
                return ret;
            }
        }

        if( (itchunk + 1) != vchunks.end() ) {
            // Evaluate final values, preparing for validation of the next chunk
            itchunk->Eval(itchunk->duration, prevXVect);
            itchunk->Evald1(itchunk->duration, prevVVect);
            itchunk->Evald2(itchunk->duration, prevAVect);
        }
    }
    return PCR_Normal;
}

} // end namespace PiecewisePolynomialsInternal

} // end namespace OpenRAVE
