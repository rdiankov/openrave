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
#include "openraveplugindefs.h"
#include <fstream>
#include <openrave/planningutils.h>

#include "piecewisepolynomials/polynomialtrajectory.h"
#include "piecewisepolynomials/quinticinterpolator.h"
#include "piecewisepolynomials/feasibilitychecker.h"
#include "manipconstraints2.h"

// #define QUINTIC_SMOOTHER_PROGRESS_DEBUG

namespace rplanners {

namespace PiecewisePolynomials = PiecewisePolynomialsInternal;

class QuinticSmoother : public PlannerBase {
public:
    QuinticSmoother(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = "";
        _bManipConstraints = false;
        _constraintReturn.reset(new ConstraintFilterReturn());
        _loggingUniformSampler = RaveCreateSpaceSampler(GetEnv(), "mt19937");
        if (!!_loggingUniformSampler) {
            _loggingUniformSampler->SetSeed(utils::GetMicroTime());
        }
        _envId = GetEnv()->GetId();
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

    bool _InitPlan()
    {
        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 100;
        }
        _ndof = _parameters->GetDOF();
        _bUsePerturbation = true;
        _bExpectedModifiedConfigurations = (_parameters->fCosManipAngleThresh > -1 + g_fEpsilonLinear);

        // Initialize a uniform sampler
        if( !_uniformSampler ) {
            _uniformSampler = RaveCreateSpaceSampler(GetEnv(), "mt19937");
        }
        _uniformSampler->SetSeed(_parameters->_nRandomGeneratorSeed);
        _fileIndexMod = 1000;
#ifdef QUINTIC_SMOOTHER_PROGRESS_DEBUG
        _dumpLevel = Level_Debug;
#else
        _dumpLevel = Level_Verbose;
#endif

        // Initialize manip constraints checker
        _bManipConstraints = (_parameters->manipname.size() > 0) && (_parameters->maxmanipspeed > 0 || _parameters->maxmanipaccel > 0);
        if( _bManipConstraints ) {
            if( !_manipConstraintChecker ) {
                _manipConstraintChecker.reset(new ManipConstraintChecker2(GetEnv()));
            }
            _manipConstraintChecker->Init(_parameters->manipname, _parameters->_configurationspecification, _parameters->maxmanipspeed, _parameters->maxmanipaccel);
        }

        _quinticInterpolator.Initialize(_ndof, _envId);
        _limitsChecker.Initialize(_ndof, _envId);

        return !!_uniformSampler;
    }

    virtual PlannerParametersConstPtr GetParameters() const
    {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        uint32_t startTime = utils::GetMilliTime();

        BOOST_ASSERT(!!_parameters && !!ptraj);
        if( ptraj->GetNumWaypoints() < 2 ) {
            RAVELOG_WARN_FORMAT("env=%d, Input traj has %d waypoints.", _envId%ptraj->GetNumWaypoints());
            return PS_Failed;
        }

        // Sample _fileIndex
        if( !!_loggingUniformSampler ) {
            _fileIndex = _loggingUniformSampler->SampleSequenceOneUInt32()%_fileIndexMod;
        }
        else {
            _fileIndex = RaveRandomInt()%_fileIndexMod;
        }

        // Save planning parameters
        if( IS_DEBUGLEVEL(_dumpLevel) ) {
        }

        // Save original trajectory
        _DumpOpenRAVETrajectory(ptraj, _dumpLevel);

        // Save states
        std::vector<KinBody::KinBodyStateSaverPtr> vstatesavers;
        std::vector<KinBodyPtr> vusedbodies;
        _parameters->_configurationspecification.ExtractUsedBodies(GetEnv(), vusedbodies);
        if( vusedbodies.size() == 0 ) {
            RAVELOG_WARN_FORMAT("env=%d, There are no used bodies in this spec", _envId);
        }
        FOREACH(itbody, vusedbodies) {
            KinBody::KinBodyStateSaverPtr statesaver;
            if( (*itbody)->IsRobot() ) {
                statesaver.reset(new RobotBase::RobotStateSaver(RaveInterfaceCast<RobotBase>(*itbody), KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_ActiveDOF|KinBody::Save_ActiveManipulator|KinBody::Save_LinkVelocities));
            }
            else {
                statesaver.reset(new KinBody::KinBodyStateSaver(*itbody, KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_ActiveDOF|KinBody::Save_ActiveManipulator|KinBody::Save_LinkVelocities));
            }
            vstatesavers.push_back(statesaver);
        }

        //
        // TODO
        //
        // Prepare configuration specifications for the final trajectory
        ConfigurationSpecification posSpec = _parameters->_configurationspecification;
        ConfigurationSpecification velSpec = posSpec.ConvertToVelocitySpecification();
        ConfigurationSpecification accelSpec = posSpec.ConvertToDerivativeSpecification(2);
        ConfigurationSpecification timeSpec;
        timeSpec.AddDeltaTimeGroup();

        // Get joint values from the passed-in OpenRAVE trajectory
        bool bExactMatch = false;
        std::vector<ConfigurationSpecification::Group>::const_iterator itcompatposgroup = ptraj->GetConfigurationSpecification().FindCompatibleGroup(posSpec._vgroups.at(0), bExactMatch);
        OPENRAVE_ASSERT_FORMAT(itcompatposgroup != ptraj->GetConfigurationSpecification()._vgroups.end(), "env=%d, Failed to find group %s in the passed-in trajectory", _envId%posSpec._vgroups.at(0).name, ORE_InvalidArguments);

        PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj = _cacheTraj;
	pwptraj.Reset();
        bool bPathIsPerfectlyModeled = false; // will be true if the initial interpolation is linear or quintic
        RAVELOG_VERBOSE_FORMAT("env=%d, Initial trajectory joint values interpolation is %s", _envId%itcompatposgroup->interpolation);
        if( _parameters->_hastimestamps && itcompatposgroup->interpolation == "quintic" ) {
            bPathIsPerfectlyModeled = true;

	    // Convert the OpenRAVE trajectory to a PiecewisePolynomialTrajectory
	    
        }
        else {
            if( itcompatposgroup->interpolation.size() == 0 || itcompatposgroup->interpolation == "linear" ) {
                bPathIsPerfectlyModeled = true;
            }
            // If there is timing information, simply ignore it.
        }


        return _ProcessPostPlanners(RobotBasePtr(), ptraj);
    }

    /// \brief Check if the given chunk violates any constraints (excluding joint velocity,
    /// acceleration, and jerk limits, which are assumed to already be satisfied).
    virtual PiecewisePolynomials::CheckReturn CheckChunkAllConstraints(const PiecewisePolynomials::Chunk& chunkIn, int options, std::vector<PiecewisePolynomials::Chunk>& vChunksOut)
    {
        std::vector<dReal> &x0Vect = _cacheX0Vect, &x1Vect = _cacheX1Vect, &v0Vect = _cacheV0Vect, &v1Vect = _cacheV1Vect, &a0Vect = _cacheA0Vect, &a1Vect = _cacheA1Vect;
        chunkIn.Eval(0, x0Vect);

        if( chunkIn.duration <= g_fEpsilon ) {
            // TODO: correcrtly handle this case
            vChunksOut.resize(1);
            vChunksOut[0].SetConstant(x0Vect, 0, 5);
            BOOST_ASSERT(false);
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
            int ret = _parameters->CheckPathAllConstraints(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, chunkIn.duration, IT_OpenStart, options, _constraintReturn);
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
        if( _bExpectedModifiedConfigurations && _constraintReturn->_configurationtimes.size() > 0 ) {
            OPENRAVE_ASSERT_OP(_constraintReturn->_configurations.size(), ==, _constraintReturn->_configurationtimes.size()*_ndof);

            std::vector<dReal> &curPos = _cacheCurPos, &newPos = _cacheNewPos, &curVel = _cacheCurVel, &newVel = _cacheNewVel, &curAccel = _cacheCurAccel, &newAccel = _cacheNewAccel;
            curPos = x0Vect;
            curVel = v0Vect;
            curAccel = a0Vect;
            dReal curTime = 0;
            std::vector<dReal>::const_iterator it = _constraintReturn->_configurations.begin();
            if( vChunksOut.capacity() < _constraintReturn->_configurationtimes.size() ) {
                vChunksOut.reserve(_constraintReturn->_configurationtimes.size());
            }
            for( size_t itime = 0; itime < _constraintReturn->_configurationtimes.size(); ++itime, it += _ndof ) {
                // Retrieve the next config from _constraintReturn. Velocity and acceleration are
                // evaluated from the original chunk.
                std::copy(it, it + _ndof, newPos.begin());
                chunkIn.Evald1(_constraintReturn->_configurations[itime], newVel);
                chunkIn.Evald2(_constraintReturn->_configurations[itime], newAccel);

                dReal deltaTime = _constraintReturn->_configurationtimes[itime] - curTime;
                _quinticInterpolator.ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(curPos, newPos, curVel, newVel, curAccel, newAccel, deltaTime, _cacheChunk);

                int limitsret = _limitsChecker.CheckChunk(_cacheChunk, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit, _parameters->_vConfigJerkLimit);
                if( limitsret != 0 ) {
                    RAVELOG_WARN_FORMAT("env=%d, the output chunk is invalid: t=%f/%f; limitsret=%d", _envId%curTime%chunkIn.duration%limitsret);
                    return PiecewisePolynomials::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9);
                }

                vChunksOut.push_back(_cacheChunk);
                vChunksOut.back().constraintChecked = true;
                curTime = _constraintReturn->_configurationtimes[itime];
                curPos.swap(newPos);
                curVel.swap(newVel);
                curAccel.swap(newAccel);
            }

            // Make sure that the last configuration is the desired value
            for( size_t idof = 0; idof <= _ndof; ++idof ) {
                if( RaveFabs(curPos[idof] - x1Vect[idof]) > PiecewisePolynomials::g_fPolynomialEpsilon ) {
                    RAVELOG_WARN_FORMAT("env=%d, Detected discrepancy at the last configuration: idof=%d; (%f != %f)", _envId%idof%curPos[idof]%x1Vect[idof]);
                    return PiecewisePolynomials::CheckReturn(CFO_FinalValuesNotReached);
                }
            }
        }
        else {
            vChunksOut.resize(1);
            vChunksOut[0] = chunkIn;
            vChunksOut[0].constraintChecked = true;
        }

        // Check manip speed/accel constraints
        if( _bManipConstraints && (options & CFO_CheckTimeBasedConstraints) ) {
            // try {
            //     PiecewisePolynomials::CheckReturn manipret = _manipConstraintChecker->CheckChunkManipConstraints();
            //     if( manipret.retcode != 0 ) {
            //         return manipret;
            //     }
            // }
            // catch( const std::exception& ex ) {
            //     RAVELOG_WARN_FORMAT("env=%d, CheckChunkManipConstraints threw an exception: %s", _envId%ex.what());
            //     return PiecewisePolynomials::CheckReturn(0xffff);
            // }
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

    /// \brief Compute an initial timing for an ordered set of waypoints. To maintain the geometry
    ///        of the initial path, the returned trajectory (PiecewisePolynomialTrajectory) will
    ///        stop at every waypoint. This function will also remove collinear waypoints.
    ///
    /// \param[in]  vWaypoints a vector of waypoints
    /// \param[out] pwptraj a PiecewisePolynomialTrajectory
    ///
    /// \return true if successful.
    bool _ComputeInitialTiming(const std::vector<std::vector<dReal> >& vWaypoints, PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj)
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
            dReal distThresh = 1e-5;
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
        OPENRAVE_ASSERT_OP(vNewWaypoints[0].size(), ==, _ndof);
        std::vector<PiecewisePolynomials::Chunk>& vChunksOut = _cacheVChunksOut;
        size_t numWaypoints = vNewWaypoints.size();
        for( size_t iWaypoint = 1; iWaypoint < numWaypoints; ++iWaypoint ) {
            OPENRAVE_ASSERT_OP(vNewWaypoints[iWaypoint].size(), ==, _ndof);
            if( _ComputeSegmentWithZeroVelAccelEndpoints(vNewWaypoints[iWaypoint - 1], vNewWaypoints[iWaypoint], options, vChunksOut, iWaypoint, numWaypoints) ) {
                RAVELOG_WARN_FORMAT("env=%d, Failed to time-parameterize the path connecting iWaypoint %d and %d", _envId%(iWaypoint - 1)%iWaypoint);
                return false;
            }

            if( !_parameters->verifyinitialpath && !vForceInitialChecking[iWaypoint] ) {
                FOREACHC(itchunk, vChunksOut) {
                    itchunk->constraintChecked = true;
                }
            }

            // TODO: Maybe keeping track of zero velocity points as well.
        }
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
    bool _ComputeSegmentWithZeroVelAccelEndpoints(const std::vector<dReal>& x0VectIn, const std::vector<dReal>& x1VectIn, int options, std::vector<PiecewisePolynomials::Chunk>& vChunksOut, size_t iWaypoint=0, size_t numWaypoints=0)
    {
        std::vector<dReal> &velLimits = _cacheVellimits, &accelLimits = _cacheAccelLimits, &jerkLimits = _cacheJerkLimits;
        dReal fCurVelMult = 1.0; ///< multiplier for the velocity limits.
        int numTries = 1000; // number of times to try scaling down velLimits and accelLimits before giving up
        int itry = 0;
        PiecewisePolynomials::CheckReturn checkret(0);
        for(; itry < numTries; ++itry ) {
            _quinticInterpolator.ComputeNDTrajectoryZeroTimeDerivativesOptimizeDuration(x0VectIn, x1VectIn, velLimits, accelLimits, jerkLimits, _cacheChunk);
            checkret = CheckChunkAllConstraints(_cacheChunk, options, vChunksOut);
            if( checkret.retcode == 0 ) {
                break;
            }
            else if( checkret.retcode == CFO_CheckTimeBasedConstraints ) {
                // Time-based constraints are violated so scale the velocity and acceleration limits down and try again.
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
                RAVELOG_WARN_FORMAT("env=%d, Segment (%d, %d); numWaypoints=%d; CheckChunkAllConstraints failed with ret=0x%x; %s; giving up.", _envId%(iWaypoint - 1)%iWaypoint%numWaypoints%checkret.retcode%ss.str());
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
            RAVELOG_WARN_FORMAT("env=%d, Segment (%d, %d); numWaypoints=%d; Initialization failed after %d trials. ret=0x%x; %s", _envId%(iWaypoint - 1)%iWaypoint%numWaypoints%checkret.retcode%ss.str());
            return false;
        }
        return true;
    }

    /// \brief
    int _Shortcut(PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj, int numIters, dReal minTimeStep)
    {
        int numShortcuts = 0;
        return numShortcuts;
    }

    /// \brief
    bool _DumpOpenRAVETrajectory(TrajectoryBasePtr ptraj, DebugLevel level)
    {
        if( IS_DEBUGLEVEL(level) ) {
            return true;
        }
        else {
            return false;
        }
    }

    /// \brief
    bool _DumpPiecewisePolynomialTrajectory(PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj, DebugLevel level)
    {
        if( IS_DEBUGLEVEL(level) ) {
            return true;
        }
        else {
            return false;
        }
    }

    //
    // Members
    //

    // for planning
    PiecewisePolynomials::QuinticInterpolator _quinticInterpolator;
    PiecewisePolynomials::PolynomialChecker _limitsChecker;
    size_t _ndof;
    ConstraintTrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _uniformSampler; ///< seed is controlled so that the behavior is reproducible
    bool _bUsePerturbation;
    bool _bExpectedModifiedConfigurations; ///<
    bool _bManipConstraints; ///< if true, then there are manip vel/accel constraints
    boost::shared_ptr<ManipConstraintChecker2> _manipConstraintChecker;

    // for logging
    int _envId;
    SpaceSamplerBasePtr _loggingUniformSampler; ///< used for logging. seed is randomly set.
    uint32_t _fileIndex;    ///< index of all the files saved within the current planning session
    uint32_t _fileIndexMod; ///< maximum number of trajectory index allowed when saving.
    DebugLevel _dumpLevel;  ///< minimum debug level that triggers trajectory saving.

    // cache
    ConstraintFilterReturnPtr _constraintReturn;
    TrajectoryBasePtr _pDummyTraj; ///< TODO: write a description for this
    PiecewisePolynomials::PiecewisePolynomialTrajectory _cacheTraj;

    std::vector<std::vector<dReal> > _cacheNewWaypoints;
    std::vector<dReal> _cacheVellimits, _cacheAccelLimits, _cacheJerkLimits;
    std::vector<PiecewisePolynomials::Chunk> _cacheVChunksOut;

    // for use in CheckChunkAllConstraints. TODO: write descriptions for these variables
    std::vector<dReal> _cacheX0Vect, _cacheX1Vect, _cacheV0Vect, _cacheV1Vect, _cacheA0Vect, _cacheA1Vect;
    std::vector<dReal> _cacheCurPos, _cacheNewPos, _cacheCurVel, _cacheNewVel, _cacheCurAccel, _cacheNewAccel;
    PiecewisePolynomials::Chunk _cacheChunk;
}; // end class QuinticSmoother

PlannerBasePtr CreateQuinticSmoother(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new QuinticSmoother(penv, sinput));
}

} // end namespace rplanners

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(PiecewisePolynomials::Chunk)
BOOST_TYPEOF_REGISTER_TYPE(PiecewisePolynomials::PiecewisePolynomialTrajectory)
#endif
