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
#include "openraveplugindefs.h"
#include <fstream>
#include <openrave/planningutils.h>

#include "piecewisepolynomials/interpolatorbase.h"
#include "piecewisepolynomials/feasibilitychecker.h"
#include "manipconstraints3.h"

// #define JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG

namespace rplanners {

namespace PiecewisePolynomials = PiecewisePolynomialsInternal;

using PiecewisePolynomials::PolynomialCheckReturn;

class JerkLimitedSmootherBase : public PlannerBase {
public:
    JerkLimitedSmootherBase(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = "";
        _bManipConstraints = false;
        _constraintReturn.reset(new ConstraintFilterReturn());
        _loggingUniformSampler = RaveCreateSpaceSampler(GetEnv(), "mt19937");
        if (!!_loggingUniformSampler) {
            _loggingUniformSampler->SetSeed(utils::GetMicroTime());
        }
        _envId = GetEnv()->GetId();
        // Pre-allocate in order to keep memory growth predictable.
        _nMaxDiscretizationSize = 0x1000;
        _cacheVVisitedDiscretization.resize(_nMaxDiscretizationSize*_nMaxDiscretizationSize, 0);
    }

    virtual const char* GetPlannerName() const
    {
        return "jerklimitedsmootherbase";
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        _parameters->copy(params);
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& sparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        sparams >> *_parameters;
        return _InitPlan();
    }

    virtual bool _InitPlan()
    {
        // Allow zero iterations.
        // if( _parameters->_nMaxIterations <= 0 ) {
        //     _parameters->_nMaxIterations = 100;
        // }
        _ndof = _parameters->GetDOF();
        _bUsePerturbation = true;
        _bExpectedModifiedConfigurations = (_parameters->fCosManipAngleThresh > -1 + g_fEpsilonLinear);

        // Initialize a uniform sampler
        if( !_uniformSampler ) {
            _uniformSampler = RaveCreateSpaceSampler(GetEnv(), "mt19937");
        }
        _uniformSampler->SetSeed(_parameters->_nRandomGeneratorSeed);
        _fileIndexMod = 1000;
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
        _dumpLevel = Level_Debug;
#else
        _dumpLevel = Level_Verbose;
#endif
        _errorDumpLevel = Level_Info;

        // Initialize manip constraints checker
        RAVELOG_DEBUG_FORMAT("env=%d, manipname=%s; maxmanipspeed=%f; maxmanipaccel=%f", _envId%_parameters->manipname%_parameters->maxmanipspeed%_parameters->maxmanipaccel);
        _bManipConstraints = (_parameters->manipname.size() > 0) && (_parameters->maxmanipspeed > 0 || _parameters->maxmanipaccel > 0);
        if( _bManipConstraints ) {
            if( !_manipConstraintChecker ) {
                _manipConstraintChecker.reset(new ManipConstraintChecker3(GetEnv()));
            }
            _manipConstraintChecker->Init(_parameters->manipname, _parameters->_configurationspecification, _parameters->maxmanipspeed, _parameters->maxmanipaccel);
        }

        _limitsChecker.Initialize(_ndof, _envId);

        // Caching stuff
        _cacheX0Vect.reserve(_ndof);
        _cacheX1Vect.reserve(_ndof);
        _cacheV0Vect.reserve(_ndof);
        _cacheV1Vect.reserve(_ndof);
        _cacheA0Vect.reserve(_ndof);
        _cacheA1Vect.reserve(_ndof);
        _cacheX0Vect2.reserve(_ndof);
        _cacheX1Vect2.reserve(_ndof);
        _cacheV0Vect2.reserve(_ndof);
        _cacheV1Vect2.reserve(_ndof);
        _cacheA0Vect2.reserve(_ndof);
        _cacheA1Vect2.reserve(_ndof);

        _cacheLowerLimits.resize(_ndof);
        _cacheUpperLimits.resize(_ndof);
        _cacheResolutions.resize(_ndof);

        _InitializeInterpolator();
        if( _maskinterpolation == IT_Default ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("interpolation type is not set by the smoother yet.", ORE_InvalidArguments);
        }

        return !!_uniformSampler;
    }

    virtual PlannerParametersConstPtr GetParameters() const
    {
        return _parameters;
    }

    /// \brief Check if the given sequence of chunks violates any constraints (excluding joint velocity, acceleration,
    ///        and jerk limits, which are assumed to already be satisfied).
    virtual PiecewisePolynomials::CheckReturn CheckAllChunksAllConstraints(const std::vector<PiecewisePolynomials::Chunk>& vChunksIn, int options, std::vector<PiecewisePolynomials::Chunk>& vChunksOut)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("CheckAllChunksAllConstraints not implemented", ORE_NotImplemented);
    }

    virtual PiecewisePolynomials::CheckReturn CheckConfigAllConstraints(const std::vector<dReal>& xVect, const std::vector<dReal>& vVect, const std::vector<dReal>& aVect, int options)
    {
        if( _bUsePerturbation ) {
            options |= CFO_CheckWithPerturbation;
        }
        try {
            int ret = _parameters->CheckPathAllConstraints(xVect, xVect, vVect, vVect, aVect, aVect, 0, static_cast<IntervalType>(IT_OpenStart|_maskinterpolation), options);
            PiecewisePolynomials::CheckReturn checkret(ret);
            if( ret == CFO_CheckTimeBasedConstraints ) {
                checkret.fTimeBasedSurpassMult = 0.98;
            }
            return checkret;
        }
        catch( const std::exception& ex ) {
            RAVELOG_WARN_FORMAT("env=%d, CheckPathAllConstraints threw an exception: %s", _envId%ex.what());
            return 0xffff|CFO_FromTrajectorySmoother;
        }
    }

    /// \brief Check if the given chunk violates any constraints (excluding joint velocity, acceleration, and jerk
    ///        limits, which are assumed to already be satisfied).
    ///        Important note: this function assumes that the first point of the input chunk has already been checked.
    virtual PiecewisePolynomials::CheckReturn CheckChunkAllConstraints(const PiecewisePolynomials::Chunk& chunkIn, int options, std::vector<PiecewisePolynomials::Chunk>& vChunksOut)
    {
        std::vector<dReal> &x0Vect = _cacheX0Vect2, &x1Vect = _cacheX1Vect2, &v0Vect = _cacheV0Vect2, &v1Vect = _cacheV1Vect2, &a0Vect = _cacheA0Vect2, &a1Vect = _cacheA1Vect2;
        chunkIn.Eval(0, x0Vect);

        if( chunkIn.duration <= g_fEpsilon ) {
            vChunksOut.resize(1);
            vChunksOut[0].SetConstant(x0Vect, 0, 5);
            // TODO: correcrtly handle this case. Should we actually store boundary conditions (v0,
            // v1, a0, a1) directly in Chunk?
            return PiecewisePolynomials::CheckReturn(0);
        }

        vChunksOut.resize(0);

        if( _bUsePerturbation ) {
            options |= CFO_CheckWithPerturbation;
        }
        if( _bExpectedModifiedConfigurations || _bManipConstraints ) {
            options |= CFO_FillCheckedConfiguration;
            _constraintReturn->Clear();
        }

        chunkIn.Eval(chunkIn.duration, x1Vect);
        chunkIn.Evald1(0, v0Vect);
        chunkIn.Evald1(chunkIn.duration, v1Vect);
        chunkIn.Evald2(0, a0Vect);
        chunkIn.Evald2(chunkIn.duration, a1Vect);

        // Check all constraints by the check functtion
        try {
            int ret = _parameters->CheckPathAllConstraints(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, chunkIn.duration, static_cast<IntervalType>(IT_OpenStart|_maskinterpolation), options, _constraintReturn);
            if( ret != 0 ) {
                PiecewisePolynomials::CheckReturn checkret(ret);
                if( ret == CFO_CheckTimeBasedConstraints ) {
                    checkret.fTimeBasedSurpassMult = 0.98;
                }
                return checkret;
            }
        }
        catch( const std::exception& ex ) {
            RAVELOG_WARN_FORMAT("env=%d, CheckPathAllConstraints threw an exception: %s", _envId%ex.what());
            return PiecewisePolynomials::CheckReturn(0xffff);
        }

        // When _bExpectedModifiedConfigurations is true, the configurations between q0 and q1 do
        // not lie exactly on the polynomial path any more. So need to reconstruct the trajectory.

        // TODO: it seems that even though every neighstatefn call in CheckPathAllConstraints
        // returns NSS_Reached, there are still some discrepancies between checked configurations
        // (in _constraintReturn->_configurations) and the original configurations (evaluated from
        // chunkIn). These discrepancies eventually result in failure to reconstruct chunks from the
        // checked configurations. For now, check _bHasRampDeviatedFromInterpolation. If the flag is
        // false (i.e. all neighstatefn calls return NSS_Reached), then trust that the original
        // chunkIn is safe.
        if( _bExpectedModifiedConfigurations && _constraintReturn->_configurationtimes.size() > 0 && _constraintReturn->_bHasRampDeviatedFromInterpolation ) {
            OPENRAVE_ASSERT_OP(_constraintReturn->_configurations.size(), ==, _constraintReturn->_configurationtimes.size()*_ndof);
            PiecewisePolynomials::CheckReturn processret = _ProcessConstraintReturnIntoChunks(_constraintReturn, chunkIn,
                                                                                              x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect,
                                                                                              vChunksOut);
            if( processret.retcode != 0 ) {
                return processret;
            }
        }
        else {
            vChunksOut.resize(1);
            vChunksOut[0] = chunkIn;
            vChunksOut[0].constraintChecked = true;
        }

        // RAVELOG_DEBUG_FORMAT("env=%d, _bExpectedModifiedConfigurations=%d, _bManipConstraints=%d; options=0x%x; num chunks=%d", _envId%_bExpectedModifiedConfigurations%_bManipConstraints%options%vChunksOut.size());
        // Check manip speed/accel constraints
        if( _bManipConstraints && (options & CFO_CheckTimeBasedConstraints) ) {
            try {
                PiecewisePolynomials::CheckReturn manipret;
                FOREACHC(itChunk, vChunksOut) {
                    manipret = _manipConstraintChecker->CheckChunkManipConstraints(*itChunk);
                    if( manipret.retcode != 0 ) {
                        return manipret;
                    }
                }
            }
            catch( const std::exception& ex ) {
                RAVELOG_WARN_FORMAT("env=%d, CheckChunkManipConstraints threw an exception: %s", _envId%ex.what());
                return PiecewisePolynomials::CheckReturn(0xffff);
            }
        }

        return PiecewisePolynomials::CheckReturn(0);
    }

    virtual dReal Rand()
    {
        return _uniformSampler->SampleSequenceOneReal(IT_OpenEnd);
    }

protected:

    enum ShortcutStatus
    {
        SS_Successful = 1,
    };

    /// \brief Initialize _pinterpolator. Also set _maskinterpolation.
    virtual void _InitializeInterpolator()=0;

    /// \brief Transfer information (configurations and timestamps) from constraintReturn into vChunksOut. Since the
    ///        configurations in constraintReturn could deviate from the original input chunk (chunkIn), we need to take
    ///        special care to construct new chunks that still respect all robot limits.
    ///
    /// \param[in] constraintReturn information returned from the Check function
    /// \param[in] chunkIn the original chunk being checked.
    /// \param[in] x0Vect for holding intermediate values
    /// \param[in] x1Vect for holding intermediate values
    /// \param[in] v0Vect for holding intermediate values
    /// \param[in] v1Vect for holding intermediate values
    /// \param[in] a0Vect for holding intermediate values
    /// \param[in] a1Vect for holding intermediate values
    /// \param[out] vChunksOut the resulting chunks reconstructed from constraintReturn
    /// \return CheckReturn struct containing check result.
    virtual PiecewisePolynomials::CheckReturn _ProcessConstraintReturnIntoChunks(ConstraintFilterReturnPtr contraintReturn, const PiecewisePolynomials::Chunk chunkIn,
                                                                                 std::vector<dReal>& x0Vect, std::vector<dReal>& x1Vect,
                                                                                 std::vector<dReal>& v0Vect, std::vector<dReal>& v1Vect,
                                                                                 std::vector<dReal>& a0Vect, std::vector<dReal>& a1Vect,
                                                                                 std::vector<PiecewisePolynomials::Chunk>& vChunksOut)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("ProcessConstraintReturnIntoChunks not implemented", ORE_NotImplemented);
    }

    /// \brief Given an OpenRAVE trajectory with the same type of joint values-interpolation as the initialized
    ///        interpolator, convert it into PiecewisePolynomialTrajectory representation by using the function
    ///        _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration to compute chunks.
    ///
    /// \param[in] ptraj
    /// \param[in] posSpec ConfigurationSpecification for joint values
    /// \param[in] velSpec ConfigurationSpecification for joint velocities
    /// \param[in] accelSpec ConfigurationSpecification for joint accelerations
    /// \param[in] timeSpec ConfigurationSpecification for deltatime
    /// \param[out] pwptraj
    /// \return true if successulf. false otherwise.
    PlannerStatus ConvertOpenRAVETrajectoryToPiecewisePolynomialTrajectorySameInterpolation(TrajectoryBasePtr ptraj,
                                                                                            ConfigurationSpecification& posSpec, ConfigurationSpecification& velSpec,
                                                                                            ConfigurationSpecification& accelSpec, ConfigurationSpecification& timeSpec,
                                                                                            PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj)
    {
        // Cache stuff
        std::vector<dReal> &x0Vect = _cacheX0Vect, &x1Vect = _cacheX1Vect, &v0Vect = _cacheV0Vect, &v1Vect = _cacheV1Vect, &a0Vect = _cacheA0Vect, &a1Vect = _cacheA1Vect, &tVect = _cacheTVect;
        std::vector<PiecewisePolynomials::Chunk>& tempChunks = _cacheInterpolatedChunks;
        std::vector<PiecewisePolynomials::Chunk>& vChunks = _cacheCheckedChunks;

        // Convert the OpenRAVE trajectory to a PiecewisePolynomialTrajectory
        vChunks.resize(0);
        if( vChunks.capacity() < ptraj->GetNumWaypoints() - 1 ) {
            vChunks.reserve(ptraj->GetNumWaypoints() - 1);
        }
        ptraj->GetWaypoint(0, x0Vect, posSpec);
        ptraj->GetWaypoint(0, v0Vect, velSpec);
        ptraj->GetWaypoint(0, a0Vect, accelSpec);
        for( size_t iWaypoint = 1; iWaypoint < ptraj->GetNumWaypoints(); ++iWaypoint ) {
            ptraj->GetWaypoint(iWaypoint, tVect, timeSpec);
            if( tVect.at(0) > g_fEpsilonLinear ) {
                ptraj->GetWaypoint(iWaypoint, x1Vect, posSpec);
                ptraj->GetWaypoint(iWaypoint, v1Vect, velSpec);
                ptraj->GetWaypoint(iWaypoint, a1Vect, accelSpec);
                PolynomialCheckReturn ret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, tVect[0], _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit, _parameters->_vConfigJerkLimit, tempChunks);
                if( ret != PolynomialCheckReturn::PCR_Normal ) {
                    return PS_Failed;
                }
                vChunks.insert(vChunks.end(), tempChunks.begin(), tempChunks.end());
                x0Vect.swap(x1Vect);
                v0Vect.swap(v1Vect);
                a0Vect.swap(a1Vect);
            }
        }
        pwptraj.Initialize(vChunks);

        if( !_parameters->verifyinitialpath ) {
            // Since the path is perfectly modeled, we can skip checking at the end.
            FOREACH(itchunk, pwptraj.vchunks) {
                itchunk->constraintChecked = true;
            }
        }
        return PS_HasSolution;
    }

    /// \brief Given an OpenRAVE path (trajectory without any timing information) and with every pair of waypoints
    ///        connected via a straight line (i.e. using linear interpolation), compute a piecewise polynomial
    ///        trajectory based on the path.
    ///
    /// \param[in] ptraj
    /// \param[in] posSpec ConfigurationSpecification for joint values
    /// \param[out] pwptraj
    /// \return PS_HasSolution only if successful.
    PlannerStatus ConvertOpenRAVEPathToPiecewisePolynomialTrajectory(TrajectoryBasePtr ptraj, ConfigurationSpecification& posSpec, PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj)
    {
        // Cache stuff
        std::vector<dReal> &x0Vect = _cacheX0Vect;
        std::vector<dReal> vAllWaypoints = _cacheAllWaypoints;
        std::vector<std::vector<dReal> >& vWaypoints = _cacheWaypoints;

        // If there is timing information, simply ignore it.
        vWaypoints.resize(0);
        if( vWaypoints.capacity() < ptraj->GetNumWaypoints() ) {
            vWaypoints.reserve(ptraj->GetNumWaypoints());
        }

        // Remove collinear waypoints. In the next stage, we are computing time-parameterization
        // such that the trajectory stops at every waypoint. Having too many waypoints will
        // simply slow down the overall trajectory.
        dReal collinearThresh = 1e-14;

        ptraj->GetWaypoints(0, ptraj->GetNumWaypoints(), vAllWaypoints, posSpec);
        for( size_t iWaypoint = 0; iWaypoint < ptraj->GetNumWaypoints(); ++iWaypoint ) {
            // Copy vAllWaypoints[iWaypoint] into x0Vect and check if x0Vect and the previous
            // two waypoints that just have been added to vWaypoints are collinear
            x0Vect.resize(_ndof);
            std::copy(vAllWaypoints.begin() + iWaypoint*_ndof, vAllWaypoints.begin() + (iWaypoint + 1)*_ndof, x0Vect.begin());

            // Check collinearity
            if( vWaypoints.size() > 1 ) {
                const std::vector<dReal>& xPrev1 = vWaypoints[vWaypoints.size() - 1];
                const std::vector<dReal>& xPrev2 = vWaypoints[vWaypoints.size() - 2];
                dReal dotProduct = 0;
                dReal xPrev1Length2 = 0;
                dReal xPrev2Length2 = 0;
                for( size_t idof = 0; idof < _ndof; ++idof ) {
                    dReal dx0 = xPrev1[idof] - x0Vect[idof];
                    dReal dx1 = xPrev2[idof] - x0Vect[idof];
                    dotProduct += dx0*dx1;
                    xPrev1Length2 += dx0*dx0;
                    xPrev2Length2 += dx1*dx1;
                }
                if( RaveFabs(dotProduct*dotProduct - xPrev1Length2*xPrev2Length2) <= collinearThresh ) {
                    // Waypoints are collinear. Update the last waypoint in vNewWaypoints and continue.
                    vWaypoints.back() = x0Vect;
                    continue;
                }
            }

            // Check if the new waypoint is the same as the previous one.
            if( vWaypoints.size() > 0 ) {
                dReal dist = 0;
                for( size_t idof = 0; idof < _ndof; ++idof ) {
                    dist += RaveFabs(x0Vect[idof] - vWaypoints.back()[idof]);
                }
                if( dist <= x0Vect.size()*std::numeric_limits<dReal>::epsilon() ) {
                    // This waypoint is redundant so continue.
                    continue;
                }
            }

            // The new waypoint is good. Add it to vNewWaypoints.
            vWaypoints.push_back(x0Vect);
        }

        // Time-parameterize the initial path
        if( !_ComputeInitialTiming(vWaypoints, pwptraj) ) {
            RAVELOG_WARN_FORMAT("env=%d, Failed to time-parameterize the initial piecewise linear path", _envId);
            _DumpOpenRAVETrajectory(ptraj, "failedinitial", _errorDumpLevel);
            return PS_Failed;
        }
        RAVELOG_DEBUG_FORMAT("env=%d, Finished time-parameterizating the initial piecewise linear path. numWaypoints: %d -> %d", _envId%ptraj->GetNumWaypoints()%vWaypoints.size());
        return PS_HasSolution;
    }

    /// \brief Compute an initial timing for an ordered set of waypoints. To maintain the geometry
    ///        of the initial path, the returned trajectory (PiecewisePolynomialTrajectory) will
    ///        stop at every waypoint. This function will also remove collinear waypoints.
    ///
    /// \param[in]  vWaypoints a vector of waypoints
    /// \param[out] pwptraj a PiecewisePolynomialTrajectory
    ///
    /// \return true if successful.
    bool _ComputeInitialTiming(const std::vector<std::vector<dReal> >&vWaypoints, PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj)
    {
        if( vWaypoints.size() == 1) {
            pwptraj.vchunks.resize(1);
            pwptraj.vchunks[0].SetConstant(vWaypoints[0], 0, 5);
            return true;
        }

        int options = CFO_CheckTimeBasedConstraints;
        if( !_parameters->verifyinitialpath ) {
            options = options & (~CFO_CheckEnvCollisions) & (~CFO_CheckSelfCollisions);
            RAVELOG_VERBOSE_FORMAT("env=%d, Initial path verification is disabled using options=0x%x", _envId%options);
        }

        // Prepare waypoints. If _neighstatefn exists, make sure that every pair of consecutive waypoints is good.
        std::vector<std::vector<dReal> >& vNewWaypoints = _cacheNewWaypoints;
        std::vector<uint8_t> vForceInitialChecking(vWaypoints.size(), 0); // TODO: write a description for this variable
        if( !!_parameters->_neighstatefn ) {
            vNewWaypoints = vWaypoints;
            std::vector<dReal> xMid(_ndof), xMidDelta(_ndof);

            size_t iWaypoint = 0;
            // We add more waypoints between the original pair of waypoints x0 and x1 if the
            // constraint-satisfying middle point (computed using _neighstatefn) is far from the
            // expected middle point 0.5*(x0 + x1).
            // If we need to add too many waypoints in between the original pair, something might be wrong.
            int nConsecutiveExpansionsAllowed = 10;
            int nConsecutiveExpansions = 0;
            while( iWaypoint + 1 < vNewWaypoints.size() ) {
                for( size_t idof = 0; idof < _ndof; ++idof ) {
                    xMidDelta[idof] = 0.5*(vNewWaypoints[iWaypoint + 1][idof] - vNewWaypoints[iWaypoint][idof]);
                }
                xMid = vNewWaypoints[iWaypoint];
                // Need to set the state before calling _neighstatefn
                if( _parameters->SetStateValues(xMid) != 0 ) {
                    std::stringstream ss;
                    ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                    ss << "xVect=[";
                    SerializeValues(ss, xMid);
                    ss << "]";
                    RAVELOG_WARN_FORMAT("env=%d, Setting state at iWaypoint=%d failed; %s", _envId%iWaypoint%ss.str());
                    return false;
                }

                int neighstateret = _parameters->_neighstatefn(xMid, xMidDelta, NSO_OnlyHardConstraints);
                if( neighstateret == NSS_Failed ) {
                    std::stringstream ss;
                    ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                    ss << "xMid=[";
                    SerializeValues(ss, xMid);
                    ss << "]; xMidDelta=[";
                    SerializeValues(ss, xMidDelta);
                    ss << "];";
                    RAVELOG_WARN_FORMAT("env=%d, _neighstatefn failed at iWaypoint=%d failed; %s", _envId%iWaypoint%ss.str());
                    return false;
                }
                else if( neighstateret == NSS_Reached ) {
                    // The returned configuration from _neighstatefn is what we expected
                    if( nConsecutiveExpansions > 0 ) {
                        nConsecutiveExpansions--;
                    }
                }
                else {
                    RAVELOG_VERBOSE_FORMAT("env=%d, Adding extra waypoint into the segment (%d, %d)", _envId%iWaypoint%(iWaypoint + 1));
                    // The returned configuration from _neighstatefn deviates from the expected
                    // one. So insert it to the list of waypoints.
                    vNewWaypoints.insert(vNewWaypoints.begin() + iWaypoint + 1, xMid);
                    // Need to force checking constraints with the new pair of waypoints.
                    vForceInitialChecking[iWaypoint + 1] = 1;
                    vForceInitialChecking.insert(vForceInitialChecking.begin() + iWaypoint + 1, 1);
                    nConsecutiveExpansions += 2; // not sure why incrementing by 2 here
                    if( nConsecutiveExpansions > nConsecutiveExpansionsAllowed ) {
                        std::stringstream ss;
                        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                        ss << "waypoint0=[";
                        SerializeValues(ss, vNewWaypoints[iWaypoint]);
                        ss << "]; waypoint1=[";
                        SerializeValues(ss, vNewWaypoints[iWaypoint + 1]);
                        ss << "];";
                        RAVELOG_WARN_FORMAT("env=%d, Too many consecutive expansions. iWaypoint=%d is bad; %s", _envId%iWaypoint%ss.str());
                        return false;
                    }
                    continue; // not incrementing iWaypoint yet
                }
                iWaypoint += 1;
            }
        }
        else {
            vNewWaypoints = vWaypoints;
        }

        // Time-parameterize each pair of waypoints.
        pwptraj.vchunks.resize(0);
        pwptraj.vchunks.reserve(1000); // not sure what a good value is
        OPENRAVE_ASSERT_OP(vNewWaypoints[0].size(), ==, _ndof);
        std::vector<PiecewisePolynomials::Chunk>& vChunksOut = _cacheCheckedChunks;
        size_t numWaypoints = vNewWaypoints.size();
        for( size_t iWaypoint = 1; iWaypoint < numWaypoints; ++iWaypoint ) {
            OPENRAVE_ASSERT_OP(vNewWaypoints[iWaypoint].size(), ==, _ndof);
            if( !_ComputeSegmentWithZeroVelAccelEndpoints(vNewWaypoints[iWaypoint - 1], vNewWaypoints[iWaypoint], options, vChunksOut, iWaypoint, numWaypoints) ) {
                RAVELOG_WARN_FORMAT("env=%d, Failed to time-parameterize the path connecting iWaypoint %d and %d", _envId%(iWaypoint - 1)%iWaypoint);
                return false;
            }

            if( !_parameters->verifyinitialpath && !vForceInitialChecking[iWaypoint] ) {
                FOREACHC(itchunk, vChunksOut) {
                    itchunk->constraintChecked = true;
                }
            }

            FOREACHC(itChunk, vChunksOut) {
                pwptraj.vchunks.push_back(*itChunk);
            }

            // TODO: Maybe keeping track of zero velocity points as well.
        }
        pwptraj.Initialize();
        return true;
    }

    /// \brief Given two set of joint values, compute a set of quintic polynomials that connect them
    ///        while respecting all constraints. If time-based constraints are violated, we
    ///        iteratively scale velocity and accelration limits down until the constraints are
    ///        satisfied, the maximum number of iterations is exceeded, or some other types of
    ///        constraints are violated.
    ///
    /// \param[in]  x0VectIn
    /// \param[in]  x1VectIn
    /// \param[in]  options
    /// \param[out] chunk a chunk that contains all the polynomials
    /// \return true if successful. false otherwise.
    bool _ComputeSegmentWithZeroVelAccelEndpoints(const std::vector<dReal> &x0VectIn, const std::vector<dReal>& x1VectIn, int options, std::vector<PiecewisePolynomials::Chunk>& vChunksOut, size_t iWaypoint=0, size_t numWaypoints=0)
    {
        std::vector<dReal> &velLimits = _cacheVellimits, &accelLimits = _cacheAccelLimits, &jerkLimits = _cacheJerkLimits;
        velLimits = _parameters->_vConfigVelocityLimit;
        accelLimits = _parameters->_vConfigAccelerationLimit;
        jerkLimits = _parameters->_vConfigJerkLimit;

        dReal fCurVelMult = 1.0; ///< multiplier for the velocity limits.
        int numTries = 1000; // number of times to try scaling down velLimits and accelLimits before giving up
        int itry = 0;
        PiecewisePolynomials::CheckReturn checkret(0xffff);
        for(; itry < numTries; ++itry ) {
            PolynomialCheckReturn interpolatorret = _pinterpolator->ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration(x0VectIn, x1VectIn, velLimits, accelLimits, jerkLimits, _cacheInterpolatedChunks);
            if( interpolatorret != PolynomialCheckReturn::PCR_Normal ) {
                std::stringstream ss;
                ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                ss << "x0Vect=[";
                SerializeValues(ss, x0VectIn);
                ss << "]; x1Vect=[";
                SerializeValues(ss, x1VectIn);
                ss << "]; velLimits=[";
                SerializeValues(ss, velLimits);
                ss << "]; accelLimits=[";
                SerializeValues(ss, accelLimits);
                ss << "]; jerkLimits=[";
                SerializeValues(ss, jerkLimits);
                ss << "]";
                RAVELOG_WARN_FORMAT("env=%d, Segment (%d, %d)/%d, itry=%d; Initial interpolation failed with interpolatorret=0x%x; %s; giving up.", _envId%(iWaypoint - 1)%iWaypoint%numWaypoints%itry%checkret.retcode%ss.str());
                return false;
            }
            checkret = CheckAllChunksAllConstraints(_cacheInterpolatedChunks, options, vChunksOut);
            if( checkret.retcode == 0 ) {
                break;
            }
            else if( checkret.retcode == CFO_CheckTimeBasedConstraints ) {
                // Time-based constraints are violated so scale the velocity and acceleration limits down and try again.
                RAVELOG_DEBUG_FORMAT("env=%d, Segment (%d, %d)/%d, itry=%d violated time-based constraints; fActualManipSpeed=%f; fActualManipAccel=%f; fTimeBasedSurpassMult=%f", _envId%(iWaypoint - 1)%iWaypoint%numWaypoints%itry%checkret.fMaxManipSpeed%checkret.fMaxManipAccel%checkret.fTimeBasedSurpassMult);
                fCurVelMult *= checkret.fTimeBasedSurpassMult;
                dReal fMult2 = checkret.fTimeBasedSurpassMult*checkret.fTimeBasedSurpassMult;
                for( size_t idof = 0; idof < _ndof; ++idof ) {
                    velLimits[idof] *= checkret.fTimeBasedSurpassMult;
                    accelLimits[idof] *= fMult2;
                }
                continue;
            }
            else {
                std::stringstream ss;
                ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                ss << "x0Vect=[";
                SerializeValues(ss, x0VectIn);
                ss << "]; x1Vect=[";
                SerializeValues(ss, x1VectIn);
                ss << "]; velLimits=[";
                SerializeValues(ss, velLimits);
                ss << "]; accelLimits=[";
                SerializeValues(ss, accelLimits);
                ss << "]; jerkLimits=[";
                SerializeValues(ss, jerkLimits);
                ss << "]";
                RAVELOG_WARN_FORMAT("env=%d, Segment (%d, %d)/%d, itry=%d; CheckChunkAllConstraints failed with ret=0x%x; %s; giving up.", _envId%(iWaypoint - 1)%iWaypoint%numWaypoints%itry%checkret.retcode%ss.str());
                return false;
            }
        }
        if( checkret.retcode != 0 ) {
            std::stringstream ss;
            ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
            ss << "x0Vect=[";
            SerializeValues(ss, x0VectIn);
            ss << "]; x1Vect=[";
            SerializeValues(ss, x1VectIn);
            ss << "]; velLimits=[";
            SerializeValues(ss, velLimits);
            ss << "]; accelLimits=[";
            SerializeValues(ss, accelLimits);
            ss << "]; jerkLimits=[";
            SerializeValues(ss, jerkLimits);
            ss << "]";
            RAVELOG_WARN_FORMAT("env=%d, Segment (%d, %d)/%d; Initialization failed after %d trials. ret=0x%x; %s", _envId%(iWaypoint - 1)%iWaypoint%numWaypoints%itry%checkret.retcode%ss.str());
            return false;
        }
        else {
            if( checkret.bDifferentVelocity ) {
                RAVELOG_WARN_FORMAT("env=%d, Segment(%d, %d)/%d; Initialization failed after %d trials. bDifferentVelocity=true", _envId%(iWaypoint - 1)%(iWaypoint)%numWaypoints%itry);
                return false;
            }
        }
        return true;
    }

    /// \brief Perform shortcutting procedure on the given piecewise polynomial trajectory
    virtual int _Shortcut(PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj, int numIters, dReal minTimeStep)
    {
        // Should be implemented in each derived class.
        return 0;
    }

    /// \brief Dump OpenRAVE Trajectory to file.
    ///        The filename will have the form "quinticsmoother_<fileIndex>_<suffix>.ortraj".
    ///
    /// \param[in] ptraj trajectory to dump to file
    /// \param[in] suffix The suffix to append to the filename
    /// \param[in] level Only dump the trajectory if the current debug level is not less than this.
    /// \return true if dumping is successful. false otherwise.
    bool _DumpOpenRAVETrajectory(TrajectoryBasePtr ptraj, const char* suffix, DebugLevel level)
    {
        if( IS_DEBUGLEVEL(level) ) {
            std::string filename = boost::str(boost::format("%s/%s_%d_%s.ortraj")%RaveGetHomeDirectory()%GetPlannerName()%_fileIndex%suffix);
            try {
                std::ofstream f(filename.c_str());
                f << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                ptraj->serialize(f);
                RAVELOG_INFO_FORMAT("env=%s, dumped openrave trajectory to %s", GetEnv()->GetNameId()%filename);
            }
            catch (const std::exception& ex) {
                RAVELOG_WARN_FORMAT("env=%s, failed to dump openrave trajectory to %s: %s", GetEnv()->GetNameId()%filename%ex.what());
                return false;
            }
            return true;
        }
        else {
            return false;
        }
    }

    /// \brief Dump PiecewisePolynomialTrajectory to file.
    ///        The filename will have the form "quinticsmoother_<fileindex>_<suffix>.pwptraj".
    ///
    /// \param[in] ptraj trajectory to dump to file
    /// \param[in] suffix The suffix to append to the filename
    /// \param[in] level Only dump the trajectory if the current debug level is not less than this.
    /// \return true if dumping is successful. false otherwise.
    bool _DumpPiecewisePolynomialTrajectory(PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj, const char* suffix, DebugLevel level)
    {
        if( IS_DEBUGLEVEL(level) ) {
            std::string filename = boost::str(boost::format("%s/%s_%d_%s.pwptraj")%RaveGetHomeDirectory()%GetPlannerName()%_fileIndex%suffix);
            try {
                std::ofstream f(filename.c_str());
                f << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                pwptraj.Serialize(f);
                RAVELOG_INFO_FORMAT("env=%s, dumped pwp-trajectory to %s", GetEnv()->GetNameId()%filename);
            }
            catch (const std::exception& ex) {
                RAVELOG_WARN_FORMAT("env=%s, failed to dump pwp-trajectory to %s: %s", GetEnv()->GetNameId()%filename%ex.what());
                return false;
            }
            return true;
        }
        else {
            return false;
        }
    }

    void _FormatInterpolationConditions(std::stringstream& ss,
                                        const std::vector<dReal>& x0Vect, const std::vector<dReal>& x1Vect,
                                        const std::vector<dReal>& v0Vect, const std::vector<dReal>& v1Vect,
                                        const std::vector<dReal>& a0Vect, const std::vector<dReal>& a1Vect,
                                        const dReal deltaTime,
                                        const std::vector<dReal>& xminVect, const std::vector<dReal>& xmaxVect,
                                        const std::vector<dReal>& vmVect, const std::vector<dReal>& amVect, const std::vector<dReal>& jmVect)
    {
        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        ss << "x0Vect=[";
        SerializeValues(ss, x0Vect);
        ss << "]; x1Vect=[";
        SerializeValues(ss, x1Vect);
        ss << "]; v0Vect=[";
        SerializeValues(ss, v0Vect);
        ss << "]; v1Vect=[";
        SerializeValues(ss, v1Vect);
        ss << "]; a0Vect=[";
        SerializeValues(ss, a0Vect);
        ss << "]; a1Vect=[";
        SerializeValues(ss, a1Vect);
        ss << "]";
        if( deltaTime > 0 ) {
            ss << "; deltaTime=" << deltaTime;
        }
        ss << "; xminVect=[";
        SerializeValues(ss, xminVect);
        ss << "]; xmaxVect=[";
        SerializeValues(ss, xmaxVect);
        ss << "]; vmVect=[";
        SerializeValues(ss, vmVect);
        ss << "]; amVect=[";
        SerializeValues(ss, amVect);
        ss << "]; jmVect=[";
        SerializeValues(ss, jmVect);
        ss << "];";
    }

    //
    // Members
    //

    // for planning
    PiecewisePolynomials::InterpolatorBasePtr _pinterpolator;
    PiecewisePolynomials::PolynomialChecker _limitsChecker;
    size_t _ndof;
    ConstraintTrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _uniformSampler; ///< seed is controlled so that the behavior is reproducible
    bool _bUsePerturbation;
    bool _bExpectedModifiedConfigurations; ///<
    bool _bManipConstraints; ///< if true, then there are manip vel/accel constraints
    boost::shared_ptr<ManipConstraintChecker3> _manipConstraintChecker;
    PlannerProgress _progress;
    IntervalType _maskinterpolation = IT_Default; // a smoother derived from this class must set this according to their interpolation type

    // for logging
    int _envId;
    SpaceSamplerBasePtr _loggingUniformSampler; ///< used for logging. seed is randomly set.
    uint32_t _fileIndex;    ///< index of all the files saved within the current planning session
    uint32_t _fileIndexMod; ///< maximum number of trajectory index allowed when saving.
    DebugLevel _dumpLevel;  ///< minimum debug level that triggers trajectory saving.
    DebugLevel _errorDumpLevel; ///< minimum debug level that triggers trajectory saving when an unexpected error occurs.

    // cache
    size_t _nMaxDiscretizationSize;
    ConstraintFilterReturnPtr _constraintReturn;
    TrajectoryBasePtr _pDummyTraj; ///< for storing intermediate result during the process of converting pwptraj to openrave traj.
    PiecewisePolynomials::PiecewisePolynomialTrajectory _cacheTraj;

    std::vector<dReal> _cacheX0Vect, _cacheX1Vect, _cacheV0Vect, _cacheV1Vect, _cacheA0Vect, _cacheA1Vect, _cacheTVect;
    std::vector<dReal> _cacheAllWaypoints; ///< stores the concatenation of all waypoints from the initial trajectory
    std::vector<std::vector<dReal> > _cacheWaypoints, _cacheNewWaypoints;
    std::vector<dReal> _cacheVellimits, _cacheAccelLimits, _cacheJerkLimits;

    std::vector<PiecewisePolynomials::Chunk> _cacheInterpolatedChunks; ///< for storing interpolation results
    std::vector<PiecewisePolynomials::Chunk> _cacheCheckedChunks; ///< for storing results from CheckChunkAllConstraints

    // for use in CheckChunkAllConstraints.
    std::vector<dReal> _cacheX0Vect2, _cacheX1Vect2, _cacheV0Vect2, _cacheV1Vect2, _cacheA0Vect2, _cacheA1Vect2;
    std::vector<dReal> _cacheLowerLimits, _cacheUpperLimits, _cacheResolutions; // for use in ProcessConstraintReturnIntoChunks

    std::vector<uint8_t> _cacheVVisitedDiscretization;
}; // end class QuinticSmoother

} // end namespace rplanners

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(PiecewisePolynomials::Polynomial)
BOOST_TYPEOF_REGISTER_TYPE(PiecewisePolynomials::Chunk)
BOOST_TYPEOF_REGISTER_TYPE(PiecewisePolynomials::PiecewisePolynomialTrajectory)
#endif
