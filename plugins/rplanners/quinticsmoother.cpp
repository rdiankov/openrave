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

#include "piecewisepolynomials/quinticinterpolator.h"
#include "piecewisepolynomials/polynomialchecker.h"
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
        _errorDumpLevel = Level_Info;

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

        ConfigurationSpecification posSpec = _parameters->_configurationspecification;
        ConfigurationSpecification velSpec = posSpec.ConvertToVelocitySpecification();
        ConfigurationSpecification accelSpec = posSpec.ConvertToDerivativeSpecification(2);
        ConfigurationSpecification timeSpec;
        timeSpec.AddDeltaTimeGroup();

        // Get joint values from the passed-in OpenRAVE trajectory
        bool bExactMatch = false;
        std::vector<ConfigurationSpecification::Group>::const_iterator itcompatposgroup = ptraj->GetConfigurationSpecification().FindCompatibleGroup(posSpec._vgroups.at(0), bExactMatch);
        OPENRAVE_ASSERT_FORMAT(itcompatposgroup != ptraj->GetConfigurationSpecification()._vgroups.end(), "env=%d, Failed to find group %s in the passed-in trajectory", _envId%posSpec._vgroups.at(0).name, ORE_InvalidArguments);

        // Caching stuff
        PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj = _cacheTraj;
        pwptraj.Reset();
        std::vector<dReal> &x0Vect = _cacheX0Vect, &x1Vect = _cacheX1Vect, &v0Vect = _cacheV0Vect, &v1Vect = _cacheV1Vect, &a0Vect = _cacheA0Vect, &a1Vect = _cacheA1Vect, &tVect = _cacheTVect;
        std::vector<dReal> vAllWaypoints = _cacheAllWaypoints;

        bool bPathIsPerfectlyModeled = false; // will be true if the initial interpolation is linear or quintic
        RAVELOG_VERBOSE_FORMAT("env=%d, Initial trajectory joint values interpolation is %s", _envId%itcompatposgroup->interpolation);
        if( _parameters->_hastimestamps && itcompatposgroup->interpolation == "quintic" ) {
            bPathIsPerfectlyModeled = true;

            // Convert the OpenRAVE trajectory to a PiecewisePolynomialTrajectory
            PiecewisePolynomials::Chunk& tempChunk = _cacheChunk;
            std::vector<PiecewisePolynomials::Chunk>& vChunks = _cacheVChunksOut;
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
                    _quinticInterpolator.ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, tVect[0], tempChunk);
                    vChunks.push_back(tempChunk);
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
        }
        // TODO: Maybe we need to handle other cases of interpolation as well
        else {
            if( itcompatposgroup->interpolation.size() == 0 || itcompatposgroup->interpolation == "linear" ) {
                bPathIsPerfectlyModeled = true;
            }
            // If there is timing information, simply ignore it.
            std::vector<std::vector<dReal> >& vWaypoints = _cacheWaypoints;
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
                _DumpOpenRAVETrajectory(ptraj, _errorDumpLevel);
                return PS_Failed;
            }
            RAVELOG_DEBUG_FORMAT("env=%d, Finished time-parameterizating the initial piecewise linear path. numWaypoints: %d -> %d", _envId%ptraj->GetNumWaypoints()%vWaypoints.size());
        }
        // Finish initializing PiecewisePolynomialTrajectory

        //
        // Main planning loop
        //
        try {
            _progress._iteration = 0;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return PS_Interrupted;
            }

            int numShortcuts = 0;
            dReal originalDuration = pwptraj.duration;
            if( !!_parameters->_setstatevaluesfn || !!_parameters->_setstatefn ) {
                // TODO: _parameters->_fStepLength*0.99 is chosen arbitrarily here. Maybe we can do better.
                numShortcuts = _Shortcut(pwptraj, _parameters->_nMaxIterations, _parameters->_fStepLength*0.99);
                if( numShortcuts < 0 ) {
                    return PS_Interrupted;
                }
            }
            RAVELOG_DEBUG_FORMAT("env=%d, After shortcutting: duration %.15e -> %.15e, diff=%.15e", _envId%originalDuration%pwptraj.duration%(pwptraj.duration - originalDuration));

            ++_progress._iteration;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return PS_Interrupted;
            }

            // Finish shortcutting. Now converting PiecewisePolynomialTrajectory to OpenRAVE trajectory
            // Prepare configuration specification
            ConfigurationSpecification newSpec = posSpec;
            bool bAddDeltaTime = true;
            newSpec.AddDerivativeGroups(1, bAddDeltaTime);
            newSpec.AddDerivativeGroups(2, bAddDeltaTime);
            int iIsWaypointOffset = newSpec.AddGroup("iswaypoint", 1, "next");
            int iTimeOffset = -1;
            FOREACH(itgroup, newSpec._vgroups) {
                if( itgroup->name == "deltatime" ) {
                    iTimeOffset = itgroup->offset;
                }
                else if( posSpec.FindCompatibleGroup(*itgroup) != posSpec._vgroups.end() ) {
                    itgroup->interpolation = "quintic";
                }
                else if( velSpec.FindCompatibleGroup(*itgroup) != velSpec._vgroups.end() ) {
                    itgroup->interpolation = "quartic";
                }
                else if( accelSpec.FindCompatibleGroup(*itgroup) != accelSpec._vgroups.end() ) {
                    itgroup->interpolation = "cubic";
                }
            }
            if( !_pDummyTraj || _pDummyTraj->GetXMLId() != ptraj->GetXMLId() ) {
                _pDummyTraj = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
            }
            _pDummyTraj->Init(newSpec);

            // Check consistency
            FOREACHC(itchunk, pwptraj.vchunks) {
                OPENRAVE_ASSERT_OP(itchunk->dof, ==, _ndof);
            }

            std::vector<dReal>& waypoint = _cacheAllWaypoints; // reuse _cacheAllWaypoints
            waypoint.resize(newSpec.GetDOF());
            pwptraj.vchunks.at(0).Eval(0, x0Vect);
            pwptraj.vchunks.at(0).Evald1(0, v0Vect);
            pwptraj.vchunks.at(0).Evald2(0, a0Vect);
            ConfigurationSpecification::ConvertData(waypoint.begin(), newSpec, x0Vect.begin(), posSpec, 1, GetEnv(), true);
            ConfigurationSpecification::ConvertData(waypoint.begin(), newSpec, v0Vect.begin(), velSpec, 1, GetEnv(), false);
            ConfigurationSpecification::ConvertData(waypoint.begin(), newSpec, a0Vect.begin(), accelSpec, 1, GetEnv(), false);
            waypoint[iIsWaypointOffset] = 1;
            waypoint.at(iTimeOffset) = 0;
            _pDummyTraj->Insert(_pDummyTraj->GetNumWaypoints(), waypoint);

            // For consistency checking
            dReal fExpectedDuration = 0;
            dReal fDurationDiscrepancyThresh = 1e-2;
            dReal fTrimEdgesTime = _parameters->_fStepLength*2; // we ignore constraints during [0, fTrimEdgesTime] and [duration - fTrimEdgesTime, duration]
            PiecewisePolynomials::Chunk& tempChunk = _cacheChunk; // for storing interpolation results
            std::vector<PiecewisePolynomials::Chunk>& vChunksOut = _cacheVChunksOut; // for storing chunks before putting them into the final trajcetory and for storing CheckChunkAllConstraints results
            PiecewisePolynomials::Chunk &trimmedChunk = _cacheTrimmedChunk, &remChunk = _cacheRemChunk;

            for( std::vector<PiecewisePolynomials::Chunk>::const_iterator itChunk = pwptraj.vchunks.begin(); itChunk != pwptraj.vchunks.end(); ++itChunk ) {
                ++_progress._iteration;
                if( _CallCallbacks(_progress) == PA_Interrupt ) {
                    return PS_Interrupted;
                }

                trimmedChunk = *itChunk;
                vChunksOut.resize(1);
                vChunksOut[0] = trimmedChunk;
                ++_progress._iteration;

                // Check constraints if not yet checked
                if( !itChunk->constraintChecked ) {
                    // Check joint limits + velocity/acceleration/jerk limits
                    int limitsret = _limitsChecker.CheckChunk(trimmedChunk, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit, _parameters->_vConfigJerkLimit);
                    if( limitsret != PiecewisePolynomials::PCR_Normal ) {
                        RAVELOG_WARN_FORMAT("env=%d, Detected limits violation after shortcutting; iChunk=%d; limitsret=0x%x", _envId%(itChunk - pwptraj.vchunks.begin())%limitsret);
                        return PS_Failed;
                    }

                    // Check other remaining constraints. Ignore the first and the last bits of the trajectory.
                    bool bTrimmedFront = false;
                    bool bTrimmedBack = false;
                    bool bCheck = true;
                    if( itChunk - pwptraj.vchunks.begin() == 0 ) {
                        if( itChunk->duration <= fTrimEdgesTime + g_fEpsilonLinear ) {
                            // This chunk is too short. Skip checking
                            bCheck = false;
                        }
                        else {
                            remChunk = trimmedChunk;
                            remChunk.Cut(fTrimEdgesTime, trimmedChunk);
                            bTrimmedFront = true;
                        }
                    }
                    else if( itChunk + 1 == pwptraj.vchunks.end() ) {
                        if( itChunk->duration <= fTrimEdgesTime + g_fEpsilonLinear ) {
                            // This chunk is too short. Skip checking
                            bCheck = false;
                        }
                        else {
                            trimmedChunk.Cut(trimmedChunk.duration - fTrimEdgesTime, remChunk);
                            bTrimmedBack = true;
                        }
                    }

                    _bUsePerturbation = false; // turn checking with perturbation off here.
                    if( bCheck ) {
                        PiecewisePolynomials::CheckReturn checkret = CheckChunkAllConstraints(trimmedChunk, 0xffff, vChunksOut);
                        if( checkret.retcode != 0 ) {
                            // Try to stretch the duration of the segment in hopes of fixing constraints violation.
                            bool bDilationSuccessful = false;
                            dReal newChunkDuration = trimmedChunk.duration;
                            dReal timeIncrement = 0.05*newChunkDuration;
                            size_t maxTries = 4;

                            // In the first try, just increasing the duration a tiny bit.
                            newChunkDuration += 5*PiecewisePolynomials::g_fPolynomialEpsilon;

                            trimmedChunk.Eval(0, x0Vect);
                            trimmedChunk.Eval(trimmedChunk.duration, x1Vect);
                            trimmedChunk.Evald1(0, v0Vect);
                            trimmedChunk.Evald1(trimmedChunk.duration, v1Vect);
                            trimmedChunk.Evald2(0, a0Vect);
                            trimmedChunk.Evald2(trimmedChunk.duration, a1Vect);
                            for( size_t iDilation = 0; iDilation < maxTries; ++iDilation ) {
                                _quinticInterpolator.ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, newChunkDuration, tempChunk);

                                // TODO
                                PiecewisePolynomials::CheckReturn newcheckret = CheckChunkAllConstraints(tempChunk, 0xffff, vChunksOut);
                                if( newcheckret.retcode == 0 ) {
                                    // The new chunk passes constraints checking. Need to
                                    // re-populate vChunksOut with the result.
                                    vChunksOut.resize(0);
                                    if( vChunksOut.capacity() < 2 ) {
                                        vChunksOut.reserve(2);
                                    }
                                    if( bTrimmedFront ) {
                                        vChunksOut.push_back(remChunk);
                                    }
                                    vChunksOut.push_back(tempChunk);
                                    if( bTrimmedBack ) {
                                        vChunksOut.push_back(remChunk);
                                    }
                                    bDilationSuccessful = true;
                                    break;
                                }

                                if( iDilation > 1 ) {
                                    iDilation += timeIncrement;
                                }
                                else {
                                    // Increase the duration a tiny bit first
                                    iDilation += 5*PiecewisePolynomials::g_fPolynomialEpsilon;
                                }
                            }
                            if( !bDilationSuccessful ) {
                                _DumpOpenRAVETrajectory(ptraj, _errorDumpLevel);
                                return PS_Failed;
                            }
                        }
                    }

                    ++_progress._iteration;
                    if( _CallCallbacks(_progress) == PA_Interrupt ) {
                        return PS_Interrupted;
                    }
                } // end if( !itChunk->constraintChecked )

                FOREACH(itNewChunk, vChunksOut) {
                    itNewChunk->Eval(0, x1Vect);
                    itNewChunk->Evald1(0, v1Vect);
                    itNewChunk->Evald2(0, a1Vect);
                    ConfigurationSpecification::ConvertData(waypoint.begin(), newSpec, x1Vect.begin(), posSpec, 1, GetEnv(), true);
                    ConfigurationSpecification::ConvertData(waypoint.begin(), newSpec, v1Vect.begin(), velSpec, 1, GetEnv(), false);
                    ConfigurationSpecification::ConvertData(waypoint.begin(), newSpec, a1Vect.begin(), accelSpec, 1, GetEnv(), false);
                    waypoint[iIsWaypointOffset] = 1;
                    waypoint.at(iTimeOffset) = itNewChunk->duration;
                    _pDummyTraj->Insert(_pDummyTraj->GetNumWaypoints(), waypoint);
                    fExpectedDuration += itNewChunk->duration;
                    if( IS_DEBUGLEVEL(Level_Verbose) ) {
                        OPENRAVE_ASSERT_OP(RaveFabs(fExpectedDuration - _pDummyTraj->GetDuration()), <=, 0.1*fDurationDiscrepancyThresh);
                    }
                }
            } // end iterating throuh pwptraj.vchunks

            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                OPENRAVE_ASSERT_OP(RaveFabs(fExpectedDuration - _pDummyTraj->GetDuration()), <=, fDurationDiscrepancyThresh);
            }

            ptraj->Swap(_pDummyTraj);
        }
        catch( const std::exception& ex ) {
            _DumpOpenRAVETrajectory(ptraj, _errorDumpLevel);
            RAVELOG_WARN_FORMAT("env=%d, Main planning loop threw an expection: %s", _envId%ex.what());
            return PS_Failed;
        }
        RAVELOG_DEBUG_FORMAT("env=%d, path optimizing - computation time=%f", _envId%(0.001f*(dReal)(utils::GetMilliTime() - startTime)));

        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            try {
                ptraj->Sample(x0Vect, 0.5*ptraj->GetDuration()); // reuse x0Vect
                RAVELOG_VERBOSE_FORMAT("env=%d, Sampling for verification successful.", _envId);
            }
            catch( const std::exception& ex ) {
                RAVELOG_WARN_FORMAT("env=%d, Traj sampling for verification failed: %s", _envId%ex.what());
                _DumpOpenRAVETrajectory(ptraj, _errorDumpLevel);
                return PS_Failed;
            }
        }

        // Save the final trajectory
        _DumpOpenRAVETrajectory(ptraj, _dumpLevel);

        return _ProcessPostPlanners(RobotBasePtr(), ptraj);
    }

    /// \brief Check if the given chunk violates any constraints (excluding joint velocity,
    /// acceleration, and jerk limits, which are assumed to already be satisfied).
    virtual PiecewisePolynomials::CheckReturn CheckChunkAllConstraints(const PiecewisePolynomials::Chunk& chunkIn, int options, std::vector<PiecewisePolynomials::Chunk>& vChunksOut)
    {
        std::vector<dReal> &x0Vect = _cacheX0Vect2, &x1Vect = _cacheX1Vect2, &v0Vect = _cacheV0Vect2, &v1Vect = _cacheV1Vect2, &a0Vect = _cacheA0Vect2, &a1Vect = _cacheA1Vect2;
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

            PiecewisePolynomials::Chunk& tempChunk = _cacheChunk2;
            dReal curTime = 0;
            std::vector<dReal>::const_iterator it = _constraintReturn->_configurations.begin();
            if( vChunksOut.capacity() < _constraintReturn->_configurationtimes.size() ) {
                vChunksOut.reserve(_constraintReturn->_configurationtimes.size());
            }
            for( size_t itime = 0; itime < _constraintReturn->_configurationtimes.size(); ++itime, it += _ndof ) {
                // Retrieve the next config from _constraintReturn. Velocity and acceleration are
                // evaluated from the original chunk.
                std::copy(it, it + _ndof, x1Vect.begin());
                chunkIn.Evald1(_constraintReturn->_configurations[itime], v1Vect);
                chunkIn.Evald2(_constraintReturn->_configurations[itime], a1Vect);

                dReal deltaTime = _constraintReturn->_configurationtimes[itime] - curTime;
                _quinticInterpolator.ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, deltaTime, tempChunk);

                int limitsret = _limitsChecker.CheckChunk(tempChunk, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit, _parameters->_vConfigJerkLimit);
                if( limitsret != PiecewisePolynomials::PCR_Normal ) {
                    RAVELOG_WARN_FORMAT("env=%d, the output chunk is invalid: t=%f/%f; limitsret=%d", _envId%curTime%chunkIn.duration%limitsret);
                    return PiecewisePolynomials::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9);
                }

                vChunksOut.push_back(tempChunk);
                vChunksOut.back().constraintChecked = true;
                curTime = _constraintReturn->_configurationtimes[itime];
                x0Vect.swap(x1Vect);
                v0Vect.swap(v1Vect);
                a0Vect.swap(a1Vect);
            }

            // Make sure that the last configuration is the desired value
            chunkIn.Eval(chunkIn.duration, x1Vect);
            for( size_t idof = 0; idof <= _ndof; ++idof ) {
                if( RaveFabs(x0Vect[idof] - x1Vect[idof]) > PiecewisePolynomials::g_fPolynomialEpsilon ) {
                    RAVELOG_WARN_FORMAT("env=%d, Detected discrepancy at the last configuration: idof=%d; (%f != %f)", _envId%idof%x0Vect[idof]%x1Vect[idof]);
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
        pwptraj.vchunks.reserve(vNewWaypoints.size() - 1);
        OPENRAVE_ASSERT_OP(vNewWaypoints[0].size(), ==, _ndof);
        std::vector<PiecewisePolynomials::Chunk>& vChunksOut = _cacheVChunksOut;
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
    bool _ComputeSegmentWithZeroVelAccelEndpoints(const std::vector<dReal>&x0VectIn, const std::vector<dReal>&x1VectIn, int options, std::vector<PiecewisePolynomials::Chunk>&vChunksOut, size_t iWaypoint=0, size_t numWaypoints=0)
    {
        std::vector<dReal> &velLimits = _cacheVellimits, &accelLimits = _cacheAccelLimits, &jerkLimits = _cacheJerkLimits;
        velLimits = _parameters->_vConfigVelocityLimit;
        accelLimits = _parameters->_vConfigAccelerationLimit;
        jerkLimits = _parameters->_vConfigJerkLimit;

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
    PlannerProgress _progress;

    // for logging
    int _envId;
    SpaceSamplerBasePtr _loggingUniformSampler; ///< used for logging. seed is randomly set.
    uint32_t _fileIndex;    ///< index of all the files saved within the current planning session
    uint32_t _fileIndexMod; ///< maximum number of trajectory index allowed when saving.
    DebugLevel _dumpLevel;  ///< minimum debug level that triggers trajectory saving.
    DebugLevel _errorDumpLevel; ///< minimum debug level that triggers trajectory saving when an unexpected error occurs.

    // cache
    ConstraintFilterReturnPtr _constraintReturn;
    TrajectoryBasePtr _pDummyTraj; ///< TODO: write a description for this
    PiecewisePolynomials::PiecewisePolynomialTrajectory _cacheTraj;
    std::vector<dReal> _cacheTrajPoint; ///< stores a waypoint when converting PiecewisePolynomialTrajectory to OpenRAVE trajectory

    std::vector<dReal> _cacheX0Vect, _cacheX1Vect, _cacheV0Vect, _cacheV1Vect, _cacheA0Vect, _cacheA1Vect, _cacheTVect;
    std::vector<dReal> _cacheAllWaypoints; ///< stores the concatenation of all waypoints from the initial trajectory
    std::vector<std::vector<dReal> > _cacheWaypoints, _cacheNewWaypoints;
    std::vector<dReal> _cacheVellimits, _cacheAccelLimits, _cacheJerkLimits;
    PiecewisePolynomials::Chunk _cacheChunk;
    PiecewisePolynomials::Chunk _cacheTrimmedChunk, _cacheRemChunk; ///< for constraints checking at the very end
    std::vector<PiecewisePolynomials::Chunk> _cacheVChunksOut;

    // for use in CheckChunkAllConstraints. TODO: write descriptions for these variables
    std::vector<dReal> _cacheX0Vect2, _cacheX1Vect2, _cacheV0Vect2, _cacheV1Vect2, _cacheA0Vect2, _cacheA1Vect2;
    PiecewisePolynomials::Chunk _cacheChunk2;
}; // end class QuinticSmoother

PlannerBasePtr CreateQuinticSmoother(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new QuinticSmoother(penv, sinput));
}

} // end namespace rplanners

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(PiecewisePolynomials::Polynomial)
BOOST_TYPEOF_REGISTER_TYPE(PiecewisePolynomials::Chunk)
BOOST_TYPEOF_REGISTER_TYPE(PiecewisePolynomials::PiecewisePolynomialTrajectory)
#endif
