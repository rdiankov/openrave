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
#include <math.h>
#include <openrave/mathextra.h>

#include "interpolator.h"
#include "parabolicchecker.h"

namespace OpenRAVE {

namespace RampOptimizerInternal {

#define BCHECK_1D_TRAJ false // set this to true to enable checking at the end of functions Compute1DTrajectory and _ImposeJointLimitFixedDuration (for verification and debugging purposes)

ParabolicInterpolator::ParabolicInterpolator(size_t ndof, int envid)
{
    OPENRAVE_ASSERT_OP(ndof, >, 0);
    _ndof = ndof;
    _cacheVect.resize(_ndof);
    _cacheX0Vect.resize(_ndof);
    _cacheX1Vect.resize(_ndof);
    _cacheV0Vect.resize(_ndof);
    _cacheV1Vect.resize(_ndof);
    _cacheAVect.resize(_ndof);
    _cacheCurvesVect.resize(_ndof);
    _envid = envid;

    _cacheRampsVect.reserve(5);
    _cacheRampsVect2.reserve(3);
}

void ParabolicInterpolator::Initialize(size_t ndof, int envid)
{
    OPENRAVE_ASSERT_OP(ndof, >, 0);
    _ndof = ndof;
    _cacheVect.resize(_ndof);
    _cacheX0Vect.resize(_ndof);
    _cacheX1Vect.resize(_ndof);
    _cacheV0Vect.resize(_ndof);
    _cacheV1Vect.resize(_ndof);
    _cacheAVect.resize(_ndof);
    _cacheCurvesVect.resize(_ndof);
    _envid = envid;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// ND Trajectory
bool ParabolicInterpolator::ComputeZeroVelNDTrajectory(const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, std::vector<RampND>& rampndVectOut)
{
    OPENRAVE_ASSERT_OP(x0Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(x1Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(vmVect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(amVect.size(), ==, _ndof);

    // Cache
    ParabolicCurve& curve = _cacheCurve;
    std::vector<dReal>& v0Vect = _cacheV0Vect, &v1Vect = _cacheV0Vect, &aVect = _cacheAVect, &dVect = _cacheX0Vect;

    SubtractVector(x1Vect, x0Vect, dVect); // total displacement

    // Find the limiting velocity and acceleration (sdMax and sddMax when parameterizing the path
    // with the parameter s).
    dReal vMin = g_fRampInf;
    dReal aMin = g_fRampInf;
    dReal dinv;
    for (size_t idof = 0; idof < _ndof; ++idof) {
        if( !FuzzyZero(dVect[idof], g_fRampEpsilon) ) {
            dinv = 1/dVect[idof];
            vMin = Min(vMin, vmVect[idof]*Abs(dinv));
            aMin = Min(aMin, amVect[idof]*Abs(dinv));
        }
    }
    if( !(vMin < g_fRampInf && aMin < g_fRampInf) ) {
        // Displacements are zero.
        rampndVectOut.resize(1);
        rampndVectOut[0].Initialize(_ndof);
        rampndVectOut[0].SetConstant(x0Vect, 0);
        return true;
    }

    // Compute the sd profile.
    if( !Compute1DTrajectory(0, 1, 0, 0, vMin, aMin, curve, BCHECK_1D_TRAJ) ) {
        return false;
    }

    // Map the computed sd profile to joint velocity profiles
    if( _cacheCurve.GetRamps().size() == 2 ) {
        if( rampndVectOut.size() != 2 ) {
            rampndVectOut.resize(2);
        }
        // Initialize and fill rampnd._data with zeros
        rampndVectOut[0].Initialize(_ndof);
        rampndVectOut[1].Initialize(_ndof);

        rampndVectOut[0].SetDuration(curve.GetRamp(0).duration);
        rampndVectOut[1].SetDuration(curve.GetRamp(1).duration);

        rampndVectOut[0].SetX0Vect(x0Vect);
        rampndVectOut[1].SetX1Vect(x1Vect);

        ScaleVector(dVect, 0.5); // displacement for each RampND is half of the total displacement
        AddVector2(dVect, x0Vect); // distance after the first RampND
        rampndVectOut[0].SetX1Vect(dVect);
        rampndVectOut[1].SetX0Vect(dVect);

        // The final velocity of the first RampND can be computed by scaling the total displacement
        // by the velocity of the sd-profile.
        SubtractVector(x1Vect, x0Vect, v0Vect);
        ScaleVector(v0Vect, curve.GetRamp(1).v0); // scaling
        rampndVectOut[0].SetV1Vect(v0Vect);
        rampndVectOut[1].SetV0Vect(v0Vect);

        // The same goes for acceleration.
        SubtractVector(x1Vect, x0Vect, aVect);
        ScaleVector(aVect, curve.GetRamp(0).a); // scaling
        rampndVectOut[0].SetAVect(aVect);
        ScaleVector(aVect, -1); // acceleration of the second RampND
        rampndVectOut[1].SetAVect(aVect);
    }
    else {
        if( rampndVectOut.size() != 3 ) {
            rampndVectOut.resize(3);
        }
        // Initialize and fill rampnd._data with zeros
        rampndVectOut[0].Initialize(_ndof);
        rampndVectOut[1].Initialize(_ndof);
        rampndVectOut[2].Initialize(_ndof);

        rampndVectOut[0].SetDuration(curve.GetRamp(0).duration);
        rampndVectOut[1].SetDuration(curve.GetRamp(1).duration);
        rampndVectOut[2].SetDuration(curve.GetRamp(2).duration);

        rampndVectOut[0].SetX0Vect(x0Vect);
        rampndVectOut[2].SetX1Vect(x1Vect);

        ScaleVector(dVect, curve.GetRamp(0).x1 - curve.GetRamp(0).x0); // displacement done by the first RampND
        AddVector2(dVect, x0Vect); // now dVect contains positions at the first switch point
        rampndVectOut[0].SetX1Vect(dVect);
        rampndVectOut[1].SetX0Vect(dVect);

        SubtractVector(x1Vect, x0Vect, dVect); // displacement
        ScaleVector(dVect, curve.GetRamp(0).x1 - curve.GetRamp(0).x0); // displacement done by the last RampND
        SubtractVector2(dVect, x1Vect);
        ScaleVector(dVect, -1); // now dVect contains positions at the second switch point
        rampndVectOut[1].SetX1Vect(dVect);
        rampndVectOut[2].SetX0Vect(dVect);

        // Compute the velocity at the end of the first RampND
        SubtractVector(x1Vect, x0Vect, v0Vect); // displacement
        ScaleVector(v0Vect, curve.GetRamp(1).v0);
        rampndVectOut[0].SetV1Vect(v0Vect);
        rampndVectOut[1].SetV0Vect(v0Vect);
        rampndVectOut[1].SetV1Vect(v0Vect);
        rampndVectOut[2].SetV0Vect(v0Vect);

        // Compute the acceleration of the first RampND
        SubtractVector(x1Vect, x0Vect, aVect); // displacement
        ScaleVector(aVect, curve.GetRamp(0).a); // acceleration of the first RampND
        rampndVectOut[0].SetAVect(aVect);
        ScaleVector(aVect, -1); // acceleration of the last RampND
        rampndVectOut[2].SetAVect(aVect);
    }

    {// Check RampNDs before returning
        std::fill(v0Vect.begin(), v0Vect.end(), 0);
        std::fill(v1Vect.begin(), v1Vect.end(), 0);
        ParabolicCheckReturn ret = CheckRampNDs(rampndVectOut, std::vector<dReal>(), std::vector<dReal>(), vmVect, amVect, x0Vect, x1Vect, v0Vect, v1Vect);
        if( ret != PCR_Normal ) {
            return false;
        }
    }
    return true;
}

bool ParabolicInterpolator::ComputeArbitraryVelNDTrajectory(const std::vector<dReal>&x0Vect, const std::vector<dReal>&x1Vect, const std::vector<dReal>&v0Vect, const std::vector<dReal>&v1Vect, const std::vector<dReal>&xminVect, const std::vector<dReal>&xmaxVect, const std::vector<dReal>&vmVect, const std::vector<dReal>&amVect, std::vector<RampND>&rampndVectOut, bool tryHarder)
{
    OPENRAVE_ASSERT_OP(x0Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(x1Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(v0Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(v1Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(xminVect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(xmaxVect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(vmVect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(amVect.size(), ==, _ndof);

    // Check inputs
    for (size_t idof = 0; idof < _ndof; ++idof) {
        if( x0Vect[idof] > xmaxVect[idof] + g_fRampEpsilon || x0Vect[idof] < xminVect[idof] - g_fRampEpsilon ) {
            RAVELOG_WARN_FORMAT("env=%d, x0Vect[%d] = %.15e exceeds the bounds; xmin = %.15e; xmax = %.15e", _envid%idof%x0Vect[idof]%xminVect[idof]%xmaxVect[idof]);
            return false;
        }

        if( x1Vect[idof] > xmaxVect[idof] + g_fRampEpsilon || x1Vect[idof] < xminVect[idof] - g_fRampEpsilon ) {
            RAVELOG_WARN_FORMAT("env=%d, x1Vect[%d] = %.15e exceeds the bounds; xmin = %.15e; xmax = %.15e", _envid%idof%x1Vect[idof]%xminVect[idof]%xmaxVect[idof]);
            return false;
        }

        if( vmVect[idof] <= 0 ) {
            RAVELOG_WARN_FORMAT("env=%d, vmVect[%d] = %.15e is not positive", _envid%idof%vmVect[idof]);
            return false;
        }
        if( amVect[idof] <= 0 ) {
            RAVELOG_WARN_FORMAT("env=%d, amVect[%d] = %.15e is not positive", _envid%idof%amVect[idof]);
            return false;
        }

        if( v0Vect[idof] > vmVect[idof] + g_fRampEpsilon || v0Vect[idof] < -vmVect[idof] - g_fRampEpsilon ) {
            RAVELOG_WARN_FORMAT("env=%d, v0Vect[%d] = %.15e exceeds the bounds; vm = %.15e", _envid%idof%v0Vect[idof]%vmVect[idof]);
            return false;
        }

        if( v1Vect[idof] > vmVect[idof] + g_fRampEpsilon || v1Vect[idof] < -vmVect[idof] - g_fRampEpsilon ) {
            RAVELOG_WARN_FORMAT("env=%d, v1Vect[%d] = %.15e exceeds the bounds; vm = %.15e", _envid%idof%v1Vect[idof]%vmVect[idof]);
            return false;
        }
    }

    // First compute the minimum trajectory duration for each joint.
    dReal maxDuration = 0;
    size_t maxIndex = 0;
    for (size_t idof = 0; idof < _ndof; ++idof) {
        if( !Compute1DTrajectory(x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], vmVect[idof], amVect[idof], _cacheCurvesVect[idof], BCHECK_1D_TRAJ) ) {
            return false;
        }
        if( _cacheCurvesVect[idof].GetDuration() > maxDuration ) {
            maxDuration = _cacheCurvesVect[idof].GetDuration();
            maxIndex = idof;
        }
    }

    //RAVELOG_VERBOSE_FORMAT("Joint %d has the longest duration of %.15e s.", maxIndex%maxDuration);

    // Now stretch all the trajectories to some duration t. If not tryHarder, t will be
    // maxDuration. Otherwise, t will be the maximum of maxDuration and tbound (computed by taking
    // into account inoperative time intervals.
    if( !_RecomputeNDTrajectoryFixedDuration(_cacheCurvesVect, vmVect, amVect, maxIndex, tryHarder) ) {
        // Note, however, that even with tryHarder = true, the above interpolation may fail due to
        // inability to fix joint limits violation.
        return false;
    }

    //RAVELOG_VERBOSE_FORMAT("ND Trajectory generated with duration %.15e s.", _cacheCurvesVect[0].GetDuration());

    for (size_t idof = 0; idof < _ndof; ++idof) {
        if( !_ImposeJointLimitFixedDuration(_cacheCurvesVect[idof], xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], BCHECK_1D_TRAJ) ) {
            return false;
        }
    }

    //RAVELOG_VERBOSE("Imposing joint limits successful");

    if( IS_DEBUGLEVEL(Level_Verbose) ) {// Debugging: check ParabolicCurves before conversion
        for (size_t idof = 0; idof < _ndof; ++idof) {
            ParabolicCheckReturn ret = CheckRamps(_cacheCurvesVect[idof].GetRamps(), xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof]);
            if( ret != PCR_Normal ) {
                RAVELOG_WARN_FORMAT("env=%d, Failed before conversion to RampNDs. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", _envid%x0Vect[idof]%x1Vect[idof]%v0Vect[idof]%v1Vect[idof]%_cacheCurvesVect[idof].GetDuration()%xminVect[idof]%xmaxVect[idof]%vmVect[idof]%amVect[idof]);
                OPENRAVE_ASSERT_OP(ret, ==, PCR_Normal);
                return false;
            }
        }
        //RAVELOG_VERBOSE("CheckParabolicCurves successful");
    }

    _ConvertParabolicCurvesToRampNDs(_cacheCurvesVect, rampndVectOut, amVect);

    {// Check RampNDs before returning
        ParabolicCheckReturn ret = CheckRampNDs(rampndVectOut, xminVect, xmaxVect, vmVect, amVect, x0Vect, x1Vect, v0Vect, v1Vect);
        OPENRAVE_ASSERT_OP(ret, ==, PCR_Normal);
        if( ret != PCR_Normal ) {
            return false;
        }
    }
    return true;
}

bool ParabolicInterpolator::_RecomputeNDTrajectoryFixedDuration(std::vector<ParabolicCurve>& curvesVect, const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, size_t maxIndex, bool tryHarder)
{
    dReal newDuration = curvesVect[maxIndex].GetDuration();
    bool bSuccess = true;
    size_t iFailingDOF;
    for (size_t idof = 0; idof < _ndof; ++idof) {
        if( idof == maxIndex ) {
            //RAVELOG_VERBOSE_FORMAT("joint %d is already the slowest DOF, continue to the next DOF (if any)", idof);
            continue;
        }
        if( !Compute1DTrajectoryFixedDuration(curvesVect[idof].GetX0(), curvesVect[idof].GetX1(), curvesVect[idof].GetV0(), curvesVect[idof].GetV1(), vmVect[idof], amVect[idof], newDuration, _cacheCurve) ) {
            bSuccess = false;
            iFailingDOF = idof;
            break;
        }
        // Store the result back in the input curvesVect
        curvesVect[idof] = _cacheCurve;
    }

    if( !bSuccess ) {
        if( !tryHarder ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, Failed for joint %d. Info: x0=%.15e; x1=%.15e; v0=%.15e; v1=%.15e; duration=%.15e; vm=%.15e; am=%.15e", _envid%iFailingDOF%curvesVect[iFailingDOF].GetX0()%curvesVect[iFailingDOF].GetX1()%curvesVect[iFailingDOF].GetV0()%curvesVect[iFailingDOF].GetV1()%newDuration%vmVect[iFailingDOF]%amVect[iFailingDOF]);
            return bSuccess;
        }

        for (size_t idof = 0; idof < _ndof; ++idof) {
            dReal tBound;
            if( !_CalculateLeastUpperBoundInoperativeTimeInterval(curvesVect[idof].GetX0(), curvesVect[idof].GetX1(), curvesVect[idof].GetV0(), curvesVect[idof].GetV1(), vmVect[idof], amVect[idof], tBound) ) {
                return false;
            }
            if( tBound > newDuration ) {
                newDuration = tBound;
            }
        }
        RAVELOG_VERBOSE_FORMAT("env=%d, Desired trajectory duration changed: %.15e --> %.15e; diff = %.15e", _envid%curvesVect[maxIndex].GetDuration()%newDuration%(newDuration - curvesVect[maxIndex].GetDuration()));
        bSuccess = true;
        for (size_t idof = 0; idof < _ndof; ++idof) {
            if( !Compute1DTrajectoryFixedDuration(curvesVect[idof].GetX0(), curvesVect[idof].GetX1(), curvesVect[idof].GetV0(), curvesVect[idof].GetV1(), vmVect[idof], amVect[idof], newDuration, _cacheCurve) ) {
                bSuccess = false;
                iFailingDOF = idof;
                break;
            }
            // Store the result back in the input curvesVect
            curvesVect[idof] = _cacheCurve;
        }
        if( !bSuccess ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, Failed for joint %d. Info: x0=%.15e; x1=%.15e; v0=%.15e; v1=%.15e; duration=%.15e; vm=%.15e; am=%.15e", _envid%iFailingDOF%curvesVect[iFailingDOF].GetX0()%curvesVect[iFailingDOF].GetX1()%curvesVect[iFailingDOF].GetV0()%curvesVect[iFailingDOF].GetV1()%newDuration%vmVect[iFailingDOF]%amVect[iFailingDOF]);
        }
    }
    return bSuccess;
}

bool ParabolicInterpolator::ComputeNDTrajectoryFixedDuration(const std::vector<dReal>&x0Vect, const std::vector<dReal>&x1Vect, const std::vector<dReal>&v0Vect, const std::vector<dReal>&v1Vect, dReal duration, const std::vector<dReal>&xminVect, const std::vector<dReal>&xmaxVect, const std::vector<dReal>&vmVect, const std::vector<dReal>&amVect, std::vector<RampND>&rampndVectOut)
{
    OPENRAVE_ASSERT_OP(x0Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(x1Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(v0Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(v1Vect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(xminVect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(xmaxVect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(vmVect.size(), ==, _ndof);
    OPENRAVE_ASSERT_OP(amVect.size(), ==, _ndof);

    // Check inputs
    for (size_t idof = 0; idof < _ndof; ++idof) {
        if( x0Vect[idof] > xmaxVect[idof] + g_fRampEpsilon || x0Vect[idof] < xminVect[idof] - g_fRampEpsilon ) {
            RAVELOG_WARN_FORMAT("env=%d, x0Vect[%d] = %.15e exceeds the bounds; xmin = %.15e; xmax = %.15e", _envid%idof%x0Vect[idof]%xminVect[idof]%xmaxVect[idof]);
            return false;
        }

        if( x1Vect[idof] > xmaxVect[idof] + g_fRampEpsilon || x1Vect[idof] < xminVect[idof] - g_fRampEpsilon ) {
            RAVELOG_WARN_FORMAT("env=%d, x1Vect[%d] = %.15e exceeds the bounds; xmin = %.15e; xmax = %.15e", _envid%idof%x1Vect[idof]%xminVect[idof]%xmaxVect[idof]);
            return false;
        }

        if( vmVect[idof] <= 0 ) {
            RAVELOG_WARN_FORMAT("env=%d, vmVect[%d] = %.15e is not positive", _envid%idof%vmVect[idof]);
            return false;
        }
        if( amVect[idof] <= 0 ) {
            RAVELOG_WARN_FORMAT("env=%d, amVect[%d] = %.15e is not positive", _envid%idof%amVect[idof]);
            return false;
        }

        if( v0Vect[idof] > vmVect[idof] + g_fRampEpsilon || v0Vect[idof] < -vmVect[idof] - g_fRampEpsilon ) {
            RAVELOG_WARN_FORMAT("env=%d, v0Vect[%d] = %.15e exceeds the bounds; vm = %.15e", _envid%idof%v0Vect[idof]%vmVect[idof]);
            return false;
        }

        if( v1Vect[idof] > vmVect[idof] + g_fRampEpsilon || v1Vect[idof] < -vmVect[idof] - g_fRampEpsilon ) {
            RAVELOG_WARN_FORMAT("env=%d, v1Vect[%d] = %.15e exceeds the bounds; vm = %.15e", _envid%idof%v1Vect[idof]%vmVect[idof]);
            return false;
        }
    }

    for (size_t idof = 0; idof < _ndof; ++idof) {
        if( !Compute1DTrajectoryFixedDuration(x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], vmVect[idof], amVect[idof], duration, _cacheCurve) ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, Computing 1D Trajectory with fixed duration for idof = %d failed", _envid%idof);
            return false;
        }

        if( !_ImposeJointLimitFixedDuration(_cacheCurve, xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], BCHECK_1D_TRAJ) ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, Cannot impose joint limit on idof = %d", _envid%idof);
            return false;
        }

        _cacheCurvesVect[idof] = _cacheCurve;
    }

    //RAVELOG_VERBOSE("Successfully computed ND trajectory with joint limits and fixed duration");
    if( 0 ) {
        std::stringstream sss;
        sss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        sss << "x0 = [";
        SerializeValues(sss, x0Vect);
        sss << "]; x1 = [";
        SerializeValues(sss, x1Vect);
        sss << "]; v0 = [";
        SerializeValues(sss, v0Vect);
        sss << "]; v1 = [";
        SerializeValues(sss, v1Vect);
        sss << "]; duration = " << duration;
        sss << "; xminVect = [";
        SerializeValues(sss, xminVect);
        sss << "]; xmaxVect = [";
        SerializeValues(sss, xmaxVect);
        sss << "]; vmVect = [";
        SerializeValues(sss, vmVect);
        sss << "]; amVect = [";
        SerializeValues(sss, amVect);
        sss << "];";
        RAVELOG_VERBOSE(sss.str());
    }

    if( IS_DEBUGLEVEL(Level_Verbose) ) {// Debugging: check ParabolicCurves before conversion
        for (size_t idof = 0; idof < _ndof; ++idof) {
            ParabolicCheckReturn ret = CheckRamps(_cacheCurvesVect[idof].GetRamps(), xminVect[idof], xmaxVect[idof], vmVect[idof], amVect[idof], x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof]);
            if( ret != PCR_Normal ) {
                RAVELOG_WARN_FORMAT("env=%d, Failed before conversion to RampNDs. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", _envid%x0Vect[idof]%x1Vect[idof]%v0Vect[idof]%v1Vect[idof]%duration%xminVect[idof]%xmaxVect[idof]%vmVect[idof]%amVect[idof]);
                OPENRAVE_ASSERT_OP(ret, ==, PCR_Normal);
                return false;
            }
        }
        //RAVELOG_VERBOSE("CheckParabolicCurves successful");
    }

    _ConvertParabolicCurvesToRampNDs(_cacheCurvesVect, rampndVectOut, amVect);

    {// Check RampNDs before returning
        ParabolicCheckReturn ret = CheckRampNDs(rampndVectOut, xminVect, xmaxVect, vmVect, amVect, x0Vect, x1Vect, v0Vect, v1Vect);
        OPENRAVE_ASSERT_OP(ret, ==, PCR_Normal);
        if( ret != PCR_Normal ) {
            return false;
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// 1D Trajectory
bool ParabolicInterpolator::Compute1DTrajectory(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, ParabolicCurve& curveOut, bool bCheck)
{
    OPENRAVE_ASSERT_OP(vm, >, 0);
    OPENRAVE_ASSERT_OP(am, >, 0);
    OPENRAVE_ASSERT_OP(Abs(v0), <=, vm + g_fRampEpsilon);
    OPENRAVE_ASSERT_OP(Abs(v1), <=, vm + g_fRampEpsilon);

    dReal d = x1 - x0;
    dReal dv = v1 - v0;
    dReal v0Sqr = v0*v0;
    dReal v1Sqr = v1*v1;
    dReal dVSqr = v1Sqr - v0Sqr;

    dReal dStraight; // displacement caused when maximally acceleration/decelerate from v0 to v1
    if( dv == 0 ) {
        if( d == 0 ) {
            _cacheRamp.Initialize(0, 0, 0, x0);
            curveOut.Initialize(_cacheRamp);
            return true;
        }
        else {
            dStraight = 0;
        }
    }
    else if( dv > 0 ) {
        dStraight = 0.5*dVSqr/am;
    }
    else {
        dStraight = -0.5*dVSqr/am;
    }

    if( FuzzyEquals(d, dStraight, g_fRampEpsilon) ) {
        /*
           v1 can be reached from v0 by the acceleration am or -am.

           Will discrepancy between d and dStraight cause error elsewhere?

           Consider for example the case when deltad = d - dStraight > 0 and deltad < epsilon. Since
           we are estimating d to be dStraight instead, the computed x1 will be off by deltad (which
           is in turn bounded by epsilon).

           Let's consider the case when v1 > v0 > 0. If we are using the real value d instead, it is
           equivalent to adding two ramps at the end. Each extra ramp has duration deltat / 2, the
           first one going up from v1 to v' = v1 + am*deltat/2, the second one going down from v' to
           v1. From deltad that we have, we can calculate the resulting deltat:

                  deltad = v1*deltat + 0.25*am*deltat**2.

           Therefore, deltat will be in the magnitude of around 2*|log10(epsilon)| which is really
           too small.

           So it's ok to check d and dStraight using FuzzyEquals.
         */
        dReal a = dv > 0 ? am : -am;
        _cacheRamp.Initialize(v0, a, dv/a, x0);
        curveOut.Initialize(_cacheRamp);
        if( bCheck ) {// Check curveOut before returning
            ParabolicCheckReturn ret = CheckRamps(curveOut.GetRamps(), -g_fRampInf, g_fRampInf, g_fRampInf, am, x0, x1, v0, v1);
            if( ret != PCR_Normal ) {
                return false;
            }
        }
        return true;
    }

    dReal sumVSqr = v0Sqr + v1Sqr;
    dReal a0, vp; // acceleration of the first ramp and peak velocity
    bool noViolation = true; // indicates if the peak velocity, vp, exceeds the bound
    if( d > dStraight ) {
        a0 = am;
        vp = Sqrt((0.5*sumVSqr) + (a0*d));
        if( vp > vm + g_fRampEpsilon ) {
            noViolation = false;
        }
    }
    else {
        a0 = -am;
        vp = -Sqrt((0.5*sumVSqr) + (a0*d));
        if( -vp > vm + g_fRampEpsilon ) {
            noViolation = false;
        }
    }

    dReal a0inv = 1/a0;
    if( noViolation ) {
        if( _cacheRampsVect.size() != 2 ) {
            _cacheRampsVect.resize(2);
        }
        _cacheRampsVect[0].Initialize(v0, a0, (vp - v0)*a0inv, x0);
        _cacheRampsVect[1].Initialize(_cacheRampsVect[0].v1, -a0, (vp - v1)*a0inv);
        curveOut.Initialize(_cacheRampsVect);
    }
    else {
        // Idea: try to compensate the exceeding displacement (when plotting the velocity profile,
        // this exceeding displacement is the triangle whose base lies at v = vm or -vm and one of
        // its apex is at vp) by adding a middle ramp
        dReal h = Abs(vp) - vm;
        dReal t = h*Abs(a0inv);

        _cacheRampsVect.resize(3);
        _cacheRampsVect[0].Initialize(v0, a0, (vp - v0)*a0inv - t, x0);
        dReal nom = h*h;
        dReal denom = Abs(a0)*vm;
        dReal newVp = vp > 0 ? vm : -vm;
        _cacheRampsVect[1].Initialize(newVp, 0, 2*t + (nom/denom));
        _cacheRampsVect[2].Initialize(newVp, -a0, (vp - v1)*a0inv - t);
        curveOut.Initialize(_cacheRampsVect);
    }

    if( bCheck ) {// Check ParabolicCurve before returning
        ParabolicCheckReturn ret = CheckRamps(curveOut.GetRamps(), -g_fRampInf, g_fRampInf, vm, am, x0, x1, v0, v1);
        if( ret != PCR_Normal ) {
            return false;
        }
    }
    return true;
}

bool ParabolicInterpolator::_ImposeJointLimitFixedDuration(ParabolicCurve& curve, dReal xmin, dReal xmax, dReal vm, dReal am, bool bCheck)
{
    dReal bmin, bmax;
    curve.GetPeaks(bmin, bmax);
    if( (bmin >= xmin - g_fRampEpsilon) && (bmax <= xmax + g_fRampEpsilon) ) {
        //RAVELOG_VERBOSE("The input curve does not violate joint limits.");
        return true;
    }

    dReal duration = curve.GetDuration();
    dReal x0 = curve.GetX0();
    dReal x1 = curve.GetX1();
    dReal v0 = curve.GetV0();
    dReal v1 = curve.GetV1();

    dReal bt0 = g_fRampInf, ba0 = g_fRampInf, bx0 = g_fRampInf;
    dReal bt1 = g_fRampInf, ba1 = g_fRampInf, bx1 = g_fRampInf;
    if( v0 > 0 ) {
        bt0 = SolveBrakeTime(x0, v0, xmax);
        bx0 = xmax;
        ba0 = SolveBrakeAccel(x0, v0, xmax);
    }
    else if( v0 < 0 ) {
        bt0 = SolveBrakeTime(x0, v0, xmin);
        bx0 = xmin;
        ba0 = SolveBrakeAccel(x0, v0, xmin);
    }

    if( v1 < 0 ) {
        bt1 = SolveBrakeTime(x1, -v1, xmax);
        bx1 = xmax;
        ba1 = SolveBrakeAccel(x1, -v1, xmax);
    }
    else if( v1 > 0 ) {
        bt1 = SolveBrakeTime(x1, -v1, xmin);
        bx1 = xmin;
        ba1 = SolveBrakeAccel(x1, -v1, xmin);
    }

    bool bSuccess = false;

    if( (bt0 < duration) && (Abs(ba0) <= am + g_fRampEpsilon) ) {
        RAVELOG_VERBOSE("Case IIA: checking...");
        if( Abs(x1 - bx0) < (duration - bt0)*vm ) {
            if( Compute1DTrajectoryFixedDuration(bx0, x1, 0, v1, vm, am, duration - bt0, _cacheCurve) ) {
                _cacheCurve.GetPeaks(bmin, bmax);
                if( (bmin >= xmin - g_fRampEpsilon) && (bmax <= xmax + g_fRampEpsilon) ) {
                    RAVELOG_VERBOSE("Case IIA: passed");
                    bSuccess = true;
                    _cacheRampsVect.resize(1 + _cacheCurve.GetRamps().size());
                    _cacheRampsVect[0].Initialize(v0, ba0, bt0, x0);
                    for (size_t iramp = 0; iramp < _cacheCurve.GetRamps().size(); ++iramp) {
                        _cacheRampsVect[iramp + 1] = _cacheCurve.GetRamp(iramp);
                    }
                }
            }
        }
    }// end case IIA

    if( (bt1 < duration) && (Abs(ba1) <= am + g_fRampEpsilon) ) {
        RAVELOG_VERBOSE("Case IIB: checking...");
        if( Abs(x0 - bx1) < (duration - bt1)*vm ) {
            if( Compute1DTrajectoryFixedDuration(x0, bx1, v0, 0, vm, am, duration - bt1, _cacheCurve) ) {
                _cacheCurve.GetPeaks(bmin, bmax);
                if( (bmin >= xmin - g_fRampEpsilon) && (bmax <= xmax + g_fRampEpsilon) ) {
                    RAVELOG_VERBOSE("Case IIB: passed");
                    bSuccess = true;
                    _cacheRampsVect.resize(1 + _cacheCurve.GetRamps().size());
                    for (size_t iramp = 0; iramp < _cacheCurve.GetRamps().size(); ++iramp) {
                        _cacheRampsVect[iramp] = _cacheCurve.GetRamp(iramp);
                    }
                    _cacheRampsVect.back().Initialize(0, ba1, bt1, bx1);
                }
            }
        }
    }// end case IIB

    if( bx0 == bx1 ) {
        if( (bt0 + bt1 < duration) && (Max(Abs(ba0), Abs(ba1)) <= am + g_fRampEpsilon) ) {
            RAVELOG_VERBOSE("Case III");
            bSuccess = true;
            _cacheRampsVect.resize(3);
            _cacheRampsVect[0].Initialize(v0, ba0, bt0, x0);
            _cacheRampsVect[1].Initialize(0, 0, duration - (bt0 + bt1));
            _cacheRampsVect[2].Initialize(0, ba1, bt1);
        }
    }
    else {
        if( (bt0 + bt1 < duration) && (Max(Abs(ba0), Abs(ba1)) <= am + g_fRampEpsilon) ) {
            RAVELOG_VERBOSE("Case IV: checking...");
            if( Abs(bx0 - bx1) < (duration - (bt0 + bt1))*vm ) {
                if( Compute1DTrajectoryFixedDuration(bx0, bx1, 0, 0, vm, am, duration - (bt0 + bt1), _cacheCurve) ) {
                    _cacheCurve.GetPeaks(bmin, bmax);
                    if( (bmin >= xmin - g_fRampEpsilon) && (bmax <= xmax + g_fRampEpsilon) ) {
                        RAVELOG_VERBOSE("Case IV: passed");
                        bSuccess = true;
                        _cacheRampsVect.resize(2 + _cacheCurve.GetRamps().size());
                        _cacheRampsVect[0].Initialize(v0, ba0, bt0, x0);
                        for (size_t iramp = 0; iramp < _cacheCurve.GetRamps().size(); ++iramp) {
                            _cacheRampsVect[iramp + 1] = _cacheCurve.GetRamp(iramp);
                        }
                        _cacheRampsVect.back().Initialize(0, ba1, bt1);
                    }
                }
            }
        }
    }

    if( !bSuccess ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, Cannot solve for a bounded trajectory. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", _envid%x0%x1%v0%v1%duration%xmin%xmax%vm%am);
        return false;
    }

    curve.Initialize(_cacheRampsVect);
    if( bCheck ) {// Check curve before returning
        ParabolicCheckReturn ret = CheckRamps(curve.GetRamps(), xmin, xmax, vm, am, x0, x1, v0, v1);
        if( ret != PCR_Normal ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; duration = %.15e; xmin = %.15e; xmax = %.15e; vm = %.15e; am = %.15e", _envid%x0%x1%v0%v1%duration%xmin%xmax%vm%am);
            return false;
        }
    }
    return true;
}

bool ParabolicInterpolator::Stretch1DTrajectory(ParabolicCurve& curve, dReal vm, dReal am, dReal newDuration)
{
    return Compute1DTrajectoryFixedDuration(curve.GetX0(), curve.GetX1(), curve.GetV0(), curve.GetV1(), vm, am, newDuration, curve);
}

bool ParabolicInterpolator::Compute1DTrajectoryFixedDuration(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, dReal duration, ParabolicCurve& curveOut)
{
    /*
       We want to 'stretch' this velocity profile to have a new duration of endTime. First, try
       re-interpolating this profile to have two ramps. If that doesn't work, try modifying the
       profile accordingly.

       Two-ramp case: let t = endTime (the new duration that we want), a0 and a1 the new
       accelerations of the profile, t0 the duration of the new first ramp.

       Starting from

          d = (v0*t0 + 0.5*a0*(t0*t0)) + ((v0 + a0*t0)*t1 + 0.5*a1*(t1*t1)),

       where d is the displacement done by this trajectory, t1 = duration - t0, i.e., the
       duration of the second ramp.  Then we can write a0 and a1 in terms of t0 as

          a0 = A + B/t0
          a1 = A - B/t1,

       where A = (v1 - v0)/t and B = (2d/t) - (v0 + v1). We want to get the velocity profile
       which has minimal acceleration: set the minimization objective to

          J(t0) = a0*a0 + a1*a1.

       We start by calculating feasible ranges of t0 due to various constraints.

       1) Acceleration constraints for the first ramp:

          -amax <= a0 <= amax.

       From this, we have

          -amax - A <= B/t0            ---   I)
          B/t0 >= amax - A.            ---  II)

       Let sum1 = -amax - A and sum2 = amax - A. We can obtain the feasible ranges of t0
       accordingly.

       2) Acceleration constraints for the second ramp:

          -amax <= a1 <= amax.

       From this, we have

          -amax - A <= -B/(t - t0)      --- III)
          -B/(t - t0) <= amax - A.      ---  IV)

       As before, the feasible ranges of t0 can be computed accordingly.

       We will obtain an interval iX for each constraint X. Since t0 needs to satisfy all the
       four constraints plus the initial feasible range [0, endTime], we will obtain only one single
       feasible range for t0. (Proof sketch: intersection operation is associative and
       intersection of two intervals gives either an interval or an empty set.)

     */
    if( duration < -g_fRampEpsilon ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, duration = %.15e is negative", _envid%duration);
        return false;
    }

    // Cache vector
    std::vector<Ramp>& ramps = _cacheRampsVect2;

    if( duration <= g_fRampEpsilon ) {
        // Check if this is a stationary trajectory
        if( FuzzyEquals(x0, x1, g_fRampEpsilon) && FuzzyEquals(v0, v1, g_fRampEpsilon) ) {
            // This is actually a stationary trajectory
            ramps.resize(1);
            ramps[0].Initialize(v0, 0, 0, x0);
            curveOut.Initialize(ramps);
            {// Check curveOut before returing
                ParabolicCheckReturn ret = CheckRamps(curveOut.GetRamps(), -g_fRampInf, g_fRampInf, vm, am, x0, x1, v0, v1);
                if( ret != PCR_Normal ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                    return false;
                }
            }
            return true;
        }
        else {
            RAVELOG_VERBOSE_FORMAT("env=%d, The given duration is too short for any movement to be made. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
            return false;
        }
    }

    // Correct small and acceptable discrepancies in velocities (if any)
    if( v0 > vm ) {
        if( v0 <= vm + g_fRampEpsilon ) {
            // Acceptable
            v0 = vm;
        }
        else {
            RAVELOG_WARN_FORMAT("env=%d, v0 > vm: %.15e > %.15e", _envid%v0%vm);
            return false;
        }
    }
    else if( v0 < -vm ) {
        if( v0 >= -vm - g_fRampEpsilon ) {
            // Acceptable
            v0 = -vm;
        }
        else {
            RAVELOG_WARN_FORMAT("env=%d, v0 < -vm: %.15e < %.15e", _envid%v0%(-vm));
            return false;
        }
    }

    if( v1 > vm ) {
        if( v1 <= vm + g_fRampEpsilon ) {
            // Acceptable
            v1 = vm;
        }
        else {
            RAVELOG_WARN_FORMAT("env=%d, v1 > vm: %.15e > %.15e", _envid%v1%vm);
            return false;
        }
    }
    else if( v1 < -vm ) {
        if( v1 >= -vm - g_fRampEpsilon ) {
            // Acceptable
            v1 = -vm;
        }
        else {
            RAVELOG_WARN_FORMAT("env=%d, v1 < -vm: %.15e < %.15e", _envid%v1%(-vm));
            return false;
        }
    }

    dReal d = x1 - x0;
    dReal t0, t1, vp, a0, a1; // values (to be computed) for the new velocity profile
    dReal A, B, C, D; // temporary variables for solving equations (c.f. comments in the code)

    dReal durInverse = 1/duration;
    A = (v1 - v0)*durInverse;
    B = (2*d)*durInverse - (v0 + v1);

    /*
       A velocity profile having t = duration connecting (x0, v0) and (x1, v1) will have one ramp iff

              x1 - x0 = dStraight
                    d = 0.5*(v0 + v1)*duration

       The above equation is actually equivalent to

                    B = 0.

       Therefore, if B = 0 we can just interpolate the trajectory right away and return early.
     */
    if (FuzzyZero(B, g_fRampEpsilon)) {
        // In this case the boundary conditions match the given duration, i.e., (x1, v1) can be
        // reached from (x0, v0) using one ramp.
        // RAVELOG_VERBOSE("case B == 0: one-ramp trajectory");
        dReal a = 2*(x1 - x0 - v0*duration)*durInverse*durInverse; // giving priority to displacement and consistency between
                                                                   // acceleration and displacement
        Ramp ramp0(v0, a, duration, x0);
        ramps.resize(1);
        ramps[0] = ramp0;
        curveOut.Initialize(ramps);
        {// Check curveOut before returning
            ParabolicCheckReturn ret = CheckRamps(curveOut.GetRamps(), -g_fRampInf, g_fRampInf, vm, am, x0, x1, v0, v1);
            if( ret != PCR_Normal ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                return false;
            }
        }
        return true;
    }

    dReal sum1 = -am - A;
    dReal sum2 = am - A;
    C = B/sum1;
    D = B/sum2;

    if( IS_DEBUGLEVEL(Level_Verbose) ) {
        if( Abs(A) <= g_fRampEpsilon && Abs(B) <= g_fRampEpsilon ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, A and B are zero. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
        }
    }

    // Now we need to check a number of feasible intervals of tswitch1 induced by constraints on the
    // acceleration. Instead of having a class representing an interval, we use the interval bounds
    // directly. Naming convention: iXl = lower bound of interval X, iXu = upper bound of interval X.
    dReal i0l = 0, i0u = duration;
    dReal i1l = -g_fRampInf, i1u = g_fRampInf;
    dReal i2l = -g_fRampInf, i2u = g_fRampInf;
    dReal i3l = -g_fRampInf, i3u = g_fRampInf;
    dReal i4l = -g_fRampInf, i4u = g_fRampInf;

    // Intervals 1 and 2 are derived from constraints on a0 (the acceleration of the first ramp)
    // I) sum1 <= B/t0
    if( FuzzyZero(sum1, g_fRampEpsilon) ) {
        if( FuzzyZero(B, g_fRampEpsilon) ) {
            // t0 can be anything
        }
        else {
            i1l = g_fRampInf;
        }
    }
    else if( sum1 > 0 ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, sum1 > 0. This implies that duration is too short. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
        return false;
    }
    else {
        // i1 = [C, Inf)
        i1l = C;
    }

    // II) B/t0 <= sum2
    if( FuzzyZero(sum2, g_fRampEpsilon) ) {
        if( FuzzyZero(B, g_fRampEpsilon) ) {
            // t0 can be anything
        }
        else {
            i2l = g_fRampInf;
        }
    }
    else if( sum2 > 0 ) {
        // i2 = [D, Inf)
        i2l = D;
    }
    else {
        RAVELOG_VERBOSE_FORMAT("env=%d, sum2 < 0. This implies that duration is too short. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
        return false;
    }

    // Find the intersection between interval 1 and interval 2, store it in interval 2.
    if( (i1l > i2u) || (i1u < i2l) ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, interval 1 and interval 2 do not have any intersection. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
        return false;
    }
    else {
        i2l = Max(i1l, i2l);
        i2u = Min(i1u, i2u);
    }

    // Intervals 3 and 4 are derived from constraints on a1 (the acceleration of the second (last) ramp
    // III) sum1 <= B/(t0 - t)
    if( FuzzyZero(sum1, g_fRampEpsilon) ) {
        if( FuzzyZero(B, g_fRampEpsilon) ) {
            // t0 can be anything
        }
        else {
            i3l = g_fRampInf;
        }
    }
    else if( sum1 > 0 ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, sum1 > 0. This implies that duration is too short. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
        return false;
    }
    else {
        // i3 = (-Inf, t + C]
        i3u = duration + C;
    }

    // IV)
    if( FuzzyZero(sum2, g_fRampEpsilon) ) {
        if( FuzzyZero(B, g_fRampEpsilon) ) {
            // t0 can be anything
        }
        else {
            i4l = g_fRampInf;
        }
    }
    else if( sum2 > 0 ) {
        // i4 = (-Inf, t + D]
        i4u = duration + D;
    }
    else {
        RAVELOG_VERBOSE_FORMAT("env=%d, sum2 < 0. This implies that duration is too short. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
        return false;
    }

    // Find the intersection between interval 3 and interval 4, store it in interval 4.
    if( (i3l > i4u) || (i3u < i4l) ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, interval 3 and interval 4 do not have any intersection. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
        return false;
    }
    else {
        i4l = Max(i3l, i4l);
        i4u = Min(i3u, i4u);
    }

    // Find the intersection between interval 2 and interval 4, store it in interval 4. This is a
    // bit tricky because if the given duration is actually the minimum time that this trajectory
    // can get, the two intervals will theoretically intersect at only one point.
    if( FuzzyEquals(i2l, i4u, g_fRampEpsilon) || FuzzyEquals(i2u, i4l, g_fRampEpsilon) ) {
        RAVELOG_VERBOSE("interval 2 and interval 4 intersect at a point, most likely because the given endTime is actually its minimum time.");
        // Make sure that the above statement is true.
        if( !Compute1DTrajectory(x0, x1, v0, v1, vm, am, curveOut) ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, Compute1DTrajectory failed. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
            return false; // what ?
        }
        else {
            if( FuzzyEquals(curveOut.GetDuration(), duration, g_fRampEpsilon) ) {
                RAVELOG_VERBOSE("The hypothesis is correct.");
                // The curve has already been validated in Compute1DTrajectory
                return true;
            }
            else {
                RAVELOG_VERBOSE_FORMAT("env=%d, The hypothesis is wrong. Something else just happened. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                return false; // what ?
            }
        }
    }
    else if( (i2l > i4u) || (i2u < i4l) ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, interval 2 and interval 4 do not have any intersection. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
        return false;
    }
    else {
        i4l = Max(i2l, i4l);
        i4u = Min(i2u, i4u);
    }

    // Find the intersection between interval 0 and interval 4, store it in interval 4.
    if( (i0l > i4u) || (i0u < i4l) ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, interval 0 and interval 4 do not have any intersection. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
        return false;
    }
    else {
        i4l = Max(i0l, i4l);
        i4u = Min(i0u, i4u);
    }

    // Check soundness
    if( i4l > i4u ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, interval 4 is empty but the algorithm cannot detect this; interval4 = [%.15e, %.15e]", _envid%i4l%i4u);
        return false;
    }

    /*
       Now we have already obtained a range of feasible values for t0 (the duration of the first
       ramp). We choose a value of t0 by selecting the one which minimize J(t0) := (a0^2 + a1^2).

       Let x = t0 for convenience. We can write J(x) as

              J(x) = (A + B/x)^2 + (A - B/(t - x))^2.

       Then we find x which minimizes J(x) by examining the roots of dJ/dx.
     */
    if( !_SolveForT0(A, B, duration, i4l, i4u, t0) ) {
        // Solving dJ/dx = 0 somehow failed. We just choose the midpoint of the feasible interval.
        t0 = 0.5*(i4l + i4u);
    }

    /*
       We skip rounding t0 and t1 to zero since
       1. it may result in too large displacement discrepancy
       2. the original reason for this rounding is just to prevent zero division when calculating
       accelerations, a0 and a1. When t0 or t1 is very small (but non-zero), although it will result
       in huge accelerations, those accelerations will be checked and limited to the bound anyway.
     */
    if( t0 == 0 ) {
        ramps.resize(1);
        ramps[0].Initialize(v0, A, duration, x0);
        curveOut.Initialize(ramps);
        {// Check curveOut before returning
            ParabolicCheckReturn ret = CheckRamps(curveOut.GetRamps(), -g_fRampInf, g_fRampInf, vm, am, x0, x1, v0, v1);
            if( ret != PCR_Normal ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                return false;
            }
        }
        return true;
    }

    t1 = duration - t0;

    if( t1 == 0 ) {
        ramps.resize(1);
        ramps[0].Initialize(v0, A, duration, x0);
        curveOut.Initialize(ramps);
        {// Check curveOut before returning
            ParabolicCheckReturn ret = CheckRamps(curveOut.GetRamps(), -g_fRampInf, g_fRampInf, vm, am, x0, x1, v0, v1);
            if( ret != PCR_Normal ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                return false;
            }
        }
        return true;
    }

    a0 = A + B/t0;
    a1 = A - B/t1;
    vp = v0 + (a0*t0);

    // Consistency checking
    if( !FuzzyEquals(vp, v1 - (a1*t1), g_fRampEpsilon) ) {
        RAVELOG_VERBOSE_FORMAT("env=%d, Verification failed (vp != v1 - a1*d1): %.15e != %.15e", _envid%vp%(v1 - (a1*t1)));
        RAVELOG_VERBOSE_FORMAT("Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", x0%x1%v0%v1%vm%am%duration);
        RAVELOG_VERBOSE_FORMAT("Calculated values: A = %.15e; B = %.15e; t0 = %.15e; t1 = %.15e; vp = %.15e; a0 = %.15e; a1 = %.15e", A%B%t0%t1%vp%a0%a1);
        return false;
    }

    // Velocity bound checking
    if( Abs(vp) <= vm + g_fRampEpsilon ) {
        // The two-ramp profile works. Go for it.
        ramps.resize(2);
        ramps[0].Initialize(v0, a0, t0, x0);
        ramps[1].Initialize(vp, a1, t1);
        curveOut.Initialize(ramps);
        {// Check curveOut before returning
            ParabolicCheckReturn ret = CheckRamps(curveOut.GetRamps(), -g_fRampInf, g_fRampInf, vm, am, x0, x1, v0, v1);
            if( ret != PCR_Normal ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                return false;
            }
        }
        return true;
    }
    else {
        // The two-ramp velocity profile violates the velocity limit. Modify it accordingly.
        dReal vmNew = vp > 0 ? vm : -vm;

        // a0 and a1 should not be zero if the velocity limit is violated. The first check is done
        // at the line: FuzzyEquals(vp, v1 - (a1*t1)) above.
        if( (FuzzyZero(a0, g_fRampEpsilon)) || (FuzzyZero(a1, g_fRampEpsilon)) ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, Velocity limit is violated but at least one acceleration is zero: a0 = %.15e; a1 = %.15e; Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%a0%a1%x0%x1%v0%v1%vm%am%duration);
            return false;
        }

        dReal a0inv = 1/a0;
        dReal a1inv = 1/a1;

        dReal dv1 = vp - vmNew;
        dReal dv2 = vmNew - v0;
        dReal dv3 = vmNew - v1;
        dReal t0Trimmed = dv2*a0inv;  // from vmaxNew = dx0 + a0*t0Trimmed
        dReal t1Trimmed = -dv3*a1inv; // from dx1 = vmaxNew + a1*t1Trimmed

        /*
           Idea: we cut the excessive area above the velocity limit and paste that on both sides of
           the velocity profile. We do not divide the area and paste it equally on both
           sides. Instead, we try to minimize the sum of the new accelerations squared:

                  minimize    a0New^2 + a1New^2.

           Let D2 be the area of the velocity profile above the velocity limit. We have

                  D2 = 0.5*dt1*dv2 + 0.5*dt2*dv3.

           Using the relations

                  a0New = dv2/(t0Trimmed - dt1)    and
                  a1New = -dv3/(t1Trimmed - dt2)

           we finally arrive at the equation

                  A2/a0New + B2/a1New = C2,

           where A2 = dv2^2, B2 = -dv3^2, and C2 = t0Trimmed*dv2 + t1Trimmed*dv3 - 2*D2.

           Let x = a0New and y = a1New for convenience, we can formulate the problem as

                  minimize(x, y)    x^2 + y^2
                  subject to        A2/x + B2/y = C2.

           From the above problem, we can see that the objective function is actually a circle while
           the constraint function is a hyperbola. (The hyperbola is centered at (A2/C2,
           B2/C2)). Therefore, the minimizer is the point where both curves touch.

           Let p = (x0, y0) be the point that the two curves touch. Then

                  (slope of the hyperbola at p)*(y0/x0) = -1,

           i.e., the tangent line of the hyperbola at p and the line connecting the origin and p are
           perpendicular. Solving the above equation gives us

                  x0 = (A2 + (A2*B2*B2)^(1/3))/C2.
         */

        dReal A2 = dv2*dv2;
        dReal B2 = -dv3*dv3;
        dReal D2 = 0.5*dv1*(duration - t0Trimmed - t1Trimmed); // area of the velocity profile above the velocity limit.
        dReal C2 = t0Trimmed*dv2 + t1Trimmed*dv3 - 2*D2;

        dReal root = cbrt(A2*B2*B2); // from math.h

        if( FuzzyZero(C2, g_fRampEpsilon) ) {
            // This means the excessive area is too large such that after we paste it on both sides
            // of the original velocity profile, the whole profile becomes one-ramp with a = 0 and v
            // = vmNew.
            RAVELOG_VERBOSE_FORMAT("env=%d, C2 == 0. Unable to fix this case. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
            return false;
        }

        dReal C2inv = 1/C2;
        a0 = (A2 + root)*C2inv;
        if( Abs(a0) > am ) {
            if( FuzzyZero(root, g_fRampEpsilon*g_fRampEpsilon) ) {// This condition gives a1 = 0.
                // The computed a0 is exceeding the bound and its corresponding a1 is
                // zero. Therefore, we cannot fix this case. This is probably because the given
                // duration is actually less than the minimum duration that it can get.
                RAVELOG_VERBOSE_FORMAT("env=%d, |a0| > am and a1 == 0. Unable to fix this case since the given duration is too short. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                return false;
            }

            // a0 exceeds the bound, try making it stays at the bound.
            a0 = a0 > 0 ? am : -am;
            // Recalculate the related variable
            root = C2*a0 - A2;
        }

        // Now compute a1
        // Special case: a0 == 0. Then this implies vm == dx0. Reevaluate those above equations
        // leads to a1 = B2/C2
        if( Abs(a0) <= g_fRampEpsilon ) {
            a0 = 0;
            a1 = B2/C2;
            if( Abs(a1) > am ) {
                // The computed a1 is exceeding the bound while a0 being zero. This is similar to
                // the case above when |a0| > am and a1 == 0.
                RAVELOG_VERBOSE_FORMAT("env=%d, a0 == 0 and |a1| > am. Unable to fix this case since the given duration is too short. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                return false;
            }

            root = C2*a0 - A2;
            // RAVELOG_DEBUG_FORMAT("case1: a0 = %.15e; a1 = %.15e", a0%a1);
        }
        else {
            // From the hyperbola equation, we have y = B2*x/(C2*x - A2) = B2*x/root
            if( Abs(root) < g_fRampEpsilon*g_fRampEpsilon ) {
                // Special case: a1 == 0. This implies vm == dx1. If we calculate back the value of a0,
                // we will get a0 = A2/C2 which is actually root = 0.
                a1 = 0;
                a0 = A2/C2;
                // RAVELOG_DEBUG_FORMAT("case2: a0 = %.15e; a1 = %.15e", a0%a1);
            }
            else {
                a1 = B2*a0/root;
                if( Abs(a1) > am ) {
                    // a1 exceeds the bound, try making it stays at the bound.
                    a1 = a1 > 0 ? am : -am;
                    // Recalculate the related variable
                    if( C2*a1 - B2 == 0 ) {
                        // this case means a0 == 0 which shuold have been catched from above
                        RAVELOG_VERBOSE_FORMAT("env=%d, C2*a1 - B2 == 0. a0 should have been zero but a0 = %.15e. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%a0%x0%x1%v0%v1%vm%am%duration);
                        return false;
                    }
                    a0 = A2*a1/(C2*a1 - B2);
                    // RAVELOG_DEBUG_FORMAT("case3: a0 = %.15e; a1 = %.15e", a0%a1);
                }
            }
        }

        // Final check on the accelerations
        if( (Abs(a0) > am + g_fRampEpsilon) || (Abs(a1) > am + g_fRampEpsilon) ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, Cannot fix accelration bounds violation. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
            return false;
        }

        if( (Abs(a0) <= g_fRampEpsilon) && (Abs(a1) <= g_fRampEpsilon) ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, Both accelerations are zero. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e; A2 = %.15e; B2 = %.15e; C2 = %.15e; D2 = %.15e", _envid%x0%x1%v0%v1%vm%am%duration%A2%B2%C2%D2);
            return false;
        }

        if( Abs(a0) <= g_fRampEpsilon ) {
            RAVELOG_VERBOSE("|a0| < epsilon");
            t0 = duration + dv3/a1;
            t1 = duration - t0;
            vp = vmNew;

            ramps.resize(2);
            ramps[0].Initialize(v0, a0, t0, x0);
            ramps[1].Initialize(vp, a1, t1);
            curveOut.Initialize(ramps);
        }
        else if( Abs(a1) <= g_fRampEpsilon ) {
            RAVELOG_VERBOSE("|a1| < epsilon");
            t0 = dv2/a0;
            t1 = duration - t0;
            vp = vmNew;

            ramps.resize(2);
            ramps[0].Initialize(v0, a0, t0, x0);
            ramps[1].Initialize(vp, a1, t1);
            curveOut.Initialize(ramps);
        }
        else {
            t0 = dv2/a0;
            if( t0 < 0 ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, t0 < 0. The given duration is not achievable with the given bounds. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                return false;
            }

            vp = vmNew;
            dReal tLastRamp = -dv3/a1;
            if( tLastRamp < 0 ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, tLastRamp < 0. The given duration is not achievable with the given bounds. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                return false;
            }

            if( t0 + tLastRamp > duration ) {
                // Final fix
                if( A == 0 ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, (final fix) A == 0. No fixing method available. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                    return false;
                }
                t0 = (dv2 - B)/A; // note that we use A and B, not A2 and B2.
                if( t0 < 0 ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, (final fix) t0 < 0. No fixing method available. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                    return false;
                }
                t1 = duration - t0;
                a0 = A + (B/t0);
                a1 = A - (B/t1);

                ramps.resize(2);
                ramps[0].Initialize(v0, a0, t0, x0);
                ramps[1].Initialize(vp, a1, t1);
                curveOut.Initialize(ramps);
            }
            else {
                dReal tMiddle = duration - (t0 + tLastRamp);
                if( FuzzyZero(tMiddle, g_fRampEpsilon) ) {
                    RAVELOG_VERBOSE("Three-ramp profile works but having too short middle ramp.");
                    // If we leave it like this, it may cause errors later on.
                    t0 = (2*d - (v1 + vmNew)*duration)/(v0 - v1);
                    t1 = duration - t0;
                    vp = vmNew;
                    a0 = dv2/t0;
                    a1 = -dv3/t1;
                    if( (Abs(a0) > am + g_fRampEpsilon) || (Abs(a1) > am + g_fRampEpsilon) ) {
                        RAVELOG_VERBOSE_FORMAT("env=%d, Cannot merge into two-ramp because of acceleration limits. Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e; Calculated values: t0 = %.15e; t1 = %.15e; vp = %.15e; a0 = %.15e; a1 = %.15e", _envid%x0%x1%v0%v1%vm%am%duration%t0%t1%vp%a0%a1);
                        return false;
                    }

                    ramps.resize(2);
                    ramps[0].Initialize(v0, a0, t0, x0);
                    ramps[1].Initialize(vp, a1, t1);
                    curveOut.Initialize(ramps);
                }
                else {
                    // Three-ramp profile really works now
                    ramps.resize(3);
                    ramps[0].Initialize(v0, a0, t0, x0);
                    ramps[1].Initialize(vp, 0, tMiddle);
                    ramps[2].Initialize(vp, a1, tLastRamp);
                    curveOut.Initialize(ramps);
                }
            }
        }

        {// Check curveOut before returning
            ParabolicCheckReturn ret = CheckRamps(curveOut.GetRamps(), -g_fRampInf, g_fRampInf, vm, am, x0, x1, v0, v1);
            if( ret != PCR_Normal ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e; duration = %.15e", _envid%x0%x1%v0%v1%vm%am%duration);
                return false;
            }
        }
        return true;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities
bool ParabolicInterpolator::_CalculateLeastUpperBoundInoperativeTimeInterval(dReal x0, dReal x1, dReal v0, dReal v1, dReal vm, dReal am, dReal& t)
{
    /*
       Let t be the total duration of the velocity profile, a0 and a1 be the accelerations of both
       ramps. We write, in the way that has already been described in SolveMinAccel, a0 and a1 in
       terms of t0 as

              a0 = A + B/t0        and
              a1 = A - B/(t - t0).

       Imposing the acceleration bounds, we have the following inequalities:

          from -am <= a0 <= am, we have

                        t0*sum1 <= B
                              B <= sum2*t0

          from -am <= a1 <= am, we have

                  (t - t0)*sum1 <= -B
                             -B <= sum2*(t - t0),

       where sum1 = -am - A, sum2 = am - A.

       From those inequalities, we can deduce that a feasible value of t0 must fall in the
       intersection of

              [B/sum1, t + B/sum1]    and
              [B/sum2, t + B/sum2].

       Therefore, the total duration t must satisfy

              t >= B*(1/sum2 - 1/sum1)    and
              t >= B*(1/sum1 - 1/sum2).

       By substituting A = (v1 - v0)/t and B = 2*d/t - (v0 + v1) into the above inequalities, we
       have

              t >= (2*am*((2*d)/t - (v0 + v1)))/(am*am - ((v1 - v0)/t)**2)    and
              t >= -(2*am*((2*d)/t - (v0 + v1)))/(am*am - ((v1 - v0)/t)**2),

       (the inequalities are derived using Sympy). Finally, we have two solutions (for the total
       time) from each inequality. Then we select the maximum one.

       Important note: position limits are not taken into account here. The calculated upper bound
       may be invalidated because of position constraints.
     */
    dReal d = x1 - x0;
    dReal T0, T1, T2, T3;

    dReal firstTerm = (v0 + v1)/am;

    dReal temp1 = 2*(-Sqr(am))*(2*am*d - Sqr(v0) - Sqr(v1));
    dReal secondTerm1 = Sqrt(temp1)/Sqr(am);
    if( temp1 < 0 ) {
        T0 = -1;
        T1 = -1;
    }
    else {
        T0 = firstTerm + secondTerm1;
        T1 = firstTerm - secondTerm1;
    }
    T1 = Max(T0, T1);

    dReal temp2 = 2*(Sqr(am))*(2*am*d + Sqr(v0) + Sqr(v1));
    dReal secondTerm2 = Sqrt(temp2)/Sqr(am);
    if( temp2 < 0 ) {
        T2 = -1;
        T3 = -1;
    }
    else {
        T2 = -firstTerm + secondTerm2;
        T3 = -firstTerm - secondTerm2;
    }
    T3 = Max(T2, T3);

    t = Max(T1, T3);
    if( t > g_fRampEpsilon ) {
        // Sanity check

        // dStraight is the displacement produced if we were to travel with only one acceleration
        // from v0 to v1 in t. It is used to determine which direction we should aceelerate
        // first (posititve or negative acceleration).
        dReal dStraight = 0.5*(v0 + v1)*t;
        dReal amNew = d - dStraight > 0 ? am : -am;
        dReal vmNew = d - dStraight > 0 ? vm : -vm;

        dReal vp = 0.5*(amNew*t + v0 + v1); // the peak velocity
        if( Abs(vp) > vm ) {
            dReal dExcess = (vp - vmNew)*(vp - vmNew)/am;
            dReal deltaTime = dExcess/vm;
            t += deltaTime; // the time increased from correcting the velocity bound violation
        }
        // Should be no problem now.
        t = t * 1.01; // for safety reasons, we don't make t too close to the bound
        return true;
    }
    else {
        if( FuzzyEquals(x1, x0, g_fRampEpsilon) && FuzzyZero(v0, g_fRampEpsilon) && FuzzyZero(v1, g_fRampEpsilon) ) {
            t = 0;
            return true;
        }
        else {
            RAVELOG_VERBOSE_FORMAT("env=%d, Unable to calculate the least upper bound: T0 = %.15e,;T1 = %.15e; T2 = %.15e; T3 = %.15e; Info: x0 = %.15e; x1 = %.15e; v0 = %.15e; v1 = %.15e; vm = %.15e; am = %.15e", _envid%T0%T1%T2%T3%x0%x1%v0%v1%vm%am);
            return false;
        }
    }
}

bool ParabolicInterpolator::_SolveForT0(dReal A, dReal B, dReal t, dReal l, dReal u, dReal& t0)
{
    /*
       Let x = t0 for convenience. The two accelerations can be written in terms of x as

          a0 = A + B/x    and
          a1 = A - B/(t - x),

       where t is the total duration. We want to solve the following optimization problem:

          minimize(x)    J(x) = a0^2 + a1^2.

       We find the minimizer by solving dJ/dx = 0. From

          J(x) = (A + B/x)^2 + (A - B/(t - x))^2,

       we have

          dJ/dx = (2*A)*x^4 + (2*B - 4*A*t)*x^3 + (3*A*t^2 - 3*B*t)*x^2 + (A*t^3 + 3*t^2)*x + (B*t^3).
     */
    if( l < 0 ) {
        if( u < 0 ) {
            RAVELOG_VERBOSE_FORMAT("env=%d, The given interval is invalid: l = %.15e; u = %.15e", _envid%l%u);
            return false;
        }
        RAVELOG_VERBOSE_FORMAT("env=%d, Invalid lower bound is given, so reset it to zero.", _envid);
        l = 0;
    }

    if( (Abs(A) < g_fRampEpsilon) && (Abs(B) < g_fRampEpsilon) ) {
        if( l > 0 ) {
            return false;
        }
        else {
            t0 = 0;
            return true;
        }
    }

    dReal rawRoots[4];
    int numRoots;
    double tSqr = t*t;
    double tCube = tSqr*t;

    if( Abs(A) < g_fRampEpsilon ) {
        dReal coeffs[4] = {2*B, -3*B*t, 3*B*tSqr, -B*tCube};
        OpenRAVE::mathextra::polyroots<dReal, 3>(&coeffs[0], &rawRoots[0], numRoots);
    }
    else {
        dReal coeffs[5] = {2*A, -4*A*t + 2*B, 3*A*tSqr - 3*B*t, -A*tCube + 3*B*tSqr, -B*tCube};
        OpenRAVE::mathextra::polyroots<dReal, 4>(&coeffs[0], &rawRoots[0], numRoots);
    }

    if( numRoots == 0 ) {
        return false;
    }

    // Among all the solutions, find the one that minimizes the objective function.
    dReal J = g_fRampInf;
    dReal bestT = -1;
    for (int i = 0; i < numRoots; ++i) {
        if( (rawRoots[i] <= u) && (rawRoots[i] >= l) ) {
            dReal root = rawRoots[i];
            dReal firstTerm, secondTerm;
            if( Abs(root) < g_fRampEpsilon ) {
                firstTerm = 0;
            }
            else {
                firstTerm = A + (B/root);
            }

            if( Abs(t - root) < g_fRampEpsilon ) {
                secondTerm = 0;
            }
            else {
                secondTerm = A - (B/(t - root));
            }
            dReal curObj = firstTerm*firstTerm + secondTerm*secondTerm;
            if( curObj < J ) {
                J = curObj;
                bestT = root;
            }
        }
    }
    if( bestT < 0 ) {
        return false;
    }
    else {
        t0 = bestT;
        return true;
    }
}

void ParabolicInterpolator::_ConvertParabolicCurvesToRampNDs(const std::vector<ParabolicCurve>& curvesVectIn, std::vector<RampND>& rampndVectOut, const std::vector<dReal>& amVect)
{
    std::vector<dReal>& switchpointsList = _cacheSwitchpointsList;
    switchpointsList.resize(0);
    if( switchpointsList.capacity() < 4*_ndof ) {
        switchpointsList.reserve(4*_ndof); // just an estimate of the maximum possible number of switch points
    }

    switchpointsList.push_back(0);
    switchpointsList.push_back(curvesVectIn[0].GetDuration());
    for (size_t idof = 0; idof < _ndof; ++idof) {
        dReal sw = 0;
        for (std::vector<Ramp>::const_iterator itramp = curvesVectIn[idof].GetRamps().begin(); itramp != curvesVectIn[idof].GetRamps().end(); ++itramp) {
            sw += itramp->duration;
            std::vector<dReal>::iterator it = std::lower_bound(switchpointsList.begin(), switchpointsList.end(), sw);
            // Since we already have t = 0 and t = duration in switchpointsList, it must point to
            // some value between *(switchpointsList.begin()) and *(switchpointsList.end() - 1)
            // (exclusive).

            // Note also that skipping some switchpoints which are closer to their neighbors than
            // g_fRampEpsilon may introduce discrepancies greater than g_fRampEpsilon.
            // if( !(sw == *it) && !(sw == *(it - 1)) ) {
            if( !FuzzyEquals(sw, *it, 0.01*g_fRampEpsilon) && !FuzzyEquals(sw, *(it - 1), 0.01*g_fRampEpsilon) ) {
                switchpointsList.insert(it, sw);
            }
        }
    }

    rampndVectOut.resize(switchpointsList.size() - 1);
    std::vector<dReal>& x0Vect = _cacheX0Vect, &x1Vect = _cacheX1Vect, &v0Vect = _cacheV0Vect, &v1Vect = _cacheV1Vect, &aVect = _cacheAVect;
    for (size_t jdof = 0; jdof < _ndof; ++jdof) {
        x0Vect[jdof] = curvesVectIn[jdof].GetX0();
        v0Vect[jdof] = curvesVectIn[jdof].GetV0();
    }

    bool bRecomputeAccel = (amVect.size() == _ndof);

    for (size_t iswitch = 1; iswitch < switchpointsList.size(); ++iswitch) {
        dReal dur = switchpointsList[iswitch] - switchpointsList[iswitch - 1];
        dReal durSqr = dur*dur, a, temp1, temp2;
        dReal divMult = 1/(dur*(0.5*durSqr + 2));
        for (size_t jdof = 0; jdof < _ndof; ++jdof) {
            x1Vect[jdof] = curvesVectIn[jdof].EvalPos(switchpointsList[iswitch]);
            v1Vect[jdof] = curvesVectIn[jdof].EvalVel(switchpointsList[iswitch]);
            aVect[jdof] = curvesVectIn[jdof].EvalAcc(0.5*(switchpointsList[iswitch] + switchpointsList[iswitch - 1]));
            if( bRecomputeAccel ) {
                temp1 = x0Vect[jdof] - x1Vect[jdof] + v0Vect[jdof]*dur;
                temp2 = v0Vect[jdof] - v1Vect[jdof];
                a = -(dur*temp1 + 2*temp2)*divMult;
                // if( Sqr(temp1 + 0.5*durSqr*a) + Sqr(temp2 + a*dur) < Sqr(temp1 + 0.5*durSqr*aVect[jdof]) + Sqr(temp2 + aVect[jdof]*dur) ) {
                if( Abs(temp1 + 0.5*durSqr*a) <= g_fRampEpsilon && Abs(temp2 + a*dur) <= g_fRampEpsilon ) {
                    // The recomputed acceleration gives smaller discrepancy
                    if( Abs(a) <= amVect[jdof] + g_fRampEpsilon ) {
                        aVect[jdof] = a;
                    }
                }
            }
        }
        rampndVectOut[iswitch - 1].Initialize(x0Vect, x1Vect, v0Vect, v1Vect, aVect, dur);

        x0Vect.swap(x1Vect);
        v0Vect.swap(v1Vect);
    }
}

} // end namespace RampOptimizerInternal

} // end namespace OpenRAVE
