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
#include "manipconstraints3.h"

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
        // Pre-allocate in order to keep memory growth predictable.
        _nMaxDiscretizationSize = 0x1000;
        _cacheVVisitedDiscretization.resize(_nMaxDiscretizationSize*_nMaxDiscretizationSize, 0);
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
        RAVELOG_DEBUG_FORMAT("env=%d, manipname=%s; maxmanipspeed=%f; maxmanipaccel=%f", _envId%_parameters->manipname%_parameters->maxmanipspeed%_parameters->maxmanipaccel);
        _bManipConstraints = (_parameters->manipname.size() > 0) && (_parameters->maxmanipspeed > 0 || _parameters->maxmanipaccel > 0);
        if( _bManipConstraints ) {
            if( !_manipConstraintChecker ) {
                _manipConstraintChecker.reset(new ManipConstraintChecker3(GetEnv()));
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
            RAVELOG_DEBUG_FORMAT("env=%d, After shortcutting: duration %.15e -> %.15e, diff=%.15e", _envId%originalDuration%pwptraj.duration%(originalDuration - pwptraj.duration));

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

            RAVELOG_VERBOSE_FORMAT("env=%d, start inserting waypoints into ptraj; expecting total=%d waypoints", _envId%(pwptraj.vchunks.size() + 1));
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
                            RAVELOG_DEBUG_FORMAT("env=%d, Final CheckChunkAllConstraints failed iChunk=%d/%d with ret=0x%x. bTrimmedFront=%d; bTrimmedBack=%d", _envId%(itChunk - pwptraj.vchunks.begin())%pwptraj.vchunks.size()%checkret.retcode%bTrimmedBack%bTrimmedBack);
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
                                    trimmedChunk = tempChunk; // copy the result to trimmedChunk
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
                                RAVELOG_WARN_FORMAT("env=%d, Failed checking constraints of iChunk=%d/%d", _envId%(itChunk - pwptraj.vchunks.begin())%pwptraj.vchunks.size());
                                _DumpOpenRAVETrajectory(ptraj, _errorDumpLevel);
                                return PS_Failed;
                            }
                        }
                    }

                    vChunksOut.resize(0);
                    if( vChunksOut.capacity() < 2 ) {
                        vChunksOut.reserve(2);
                    }
                    if( bTrimmedFront ) {
                        vChunksOut.push_back(remChunk);
                    }
                    vChunksOut.push_back(trimmedChunk);
                    if( bTrimmedBack ) {
                        vChunksOut.push_back(remChunk);
                    }

                    ++_progress._iteration;
                    if( _CallCallbacks(_progress) == PA_Interrupt ) {
                        return PS_Interrupted;
                    }
                } // end if( !itChunk->constraintChecked )

                FOREACH(itNewChunk, vChunksOut) {
                    itNewChunk->Eval(itNewChunk->duration, x1Vect);
                    itNewChunk->Evald1(itNewChunk->duration, v1Vect);
                    itNewChunk->Evald2(itNewChunk->duration, a1Vect);
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
                chunkIn.Evald1(_constraintReturn->_configurationtimes[itime], v1Vect);
                chunkIn.Evald2(_constraintReturn->_configurationtimes[itime], a1Vect);

                dReal deltaTime = _constraintReturn->_configurationtimes[itime] - curTime;
                if( deltaTime > PiecewisePolynomials::g_fPolynomialEpsilon ) {
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
            }

            // Make sure that the last configuration is the desired value
            chunkIn.Eval(chunkIn.duration, x1Vect);
            for( size_t idof = 0; idof < _ndof; ++idof ) {
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
        PiecewisePolynomials::CheckReturn checkret(0xffff);
        for(; itry < numTries; ++itry ) {
            _quinticInterpolator.ComputeNDTrajectoryZeroTimeDerivativesOptimizeDuration(x0VectIn, x1VectIn, velLimits, accelLimits, jerkLimits, _cacheChunk);
            checkret = CheckChunkAllConstraints(_cacheChunk, options, vChunksOut);
            if( checkret.retcode == 0 ) {
                break;
            }
            else if( checkret.retcode == CFO_CheckTimeBasedConstraints ) {
                // Time-based constraints are violated so scale the velocity and acceleration limits down and try again.
                RAVELOG_DEBUG_FORMAT("env=%d, Segment (%d, %d)/%d violated time-based constraints; fActualManipSpeed=%f; fActualManipAccel=%f; fTimeBasedSurpassMult=%f", _envId%(iWaypoint - 1)%iWaypoint%numWaypoints%checkret.fMaxManipSpeed%checkret.fMaxManipAccel%checkret.fTimeBasedSurpassMult);
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
                RAVELOG_WARN_FORMAT("env=%d, Segment (%d, %d)/%d; CheckChunkAllConstraints failed with ret=0x%x; %s; giving up.", _envId%(iWaypoint - 1)%iWaypoint%numWaypoints%checkret.retcode%ss.str());
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

        //
        // Parameters
        //
        int numSlowDowns = 0; // counts how many times we scale down the velocity/acceleration limits.
        size_t maxSlowDownTries = 100; // the limit of the number of slowdowns in one shortcut iteration.
        dReal fiSearchVelAccelMult = 1.0/_parameters->fSearchVelAccelMult; // magic constant

        // fStartTimeVelMult (resp. fStartTimeAccelMult) tracks the multiplier used in th most recent successful
        // shortcut iteration. In the beginning of each shortcut iteration, velocity/acceleration limits will be set to
        // the original limits multiplied by this value. The reason is that if the most recent successful multiplier is
        // low, say 0.1, it is unlikely that using the original velocity/acceleration limits will lead to a successful
        // shortcut.
        dReal fStartTimeVelMult = 1.0;
        dReal fStartTimeAccelMult = 1.0;
        dReal fVelMultCutoff = 0.01; // stop slowing down when the current velocity multiplier goes below this value
        dReal fAccelMultCutoff = fVelMultCutoff*fVelMultCutoff;

        // We stop shortcutting if no progress has been made in the past nCutoffIters iterations.
        size_t nCutoffIters = std::max(_parameters->nshortcutcycles, min(100, numIters/2));
        size_t nItersFromPrevSuccessful = 0; // counts how many iterations have passed since the most recent successful iteration
        size_t nTimeBasedConstraintsFailed = 0; // counts how many times time-based constraints failed since the most
        // recent successful iteration.

        // We stop shortcutting if the latest successful shortcut made too little progress (i.e. if
        // fScore/fCurrentBestScore < fCutoffRatio)
        dReal fCutoffRatio = _parameters->durationImprovementCutoffRatio;
        dReal fScore = 1.0;
        dReal fCurrentBestScore = 1.0; // keeps track of the best score so far

        // TODO: write descriptions
        dReal fiMinDiscretization = 4.0/minTimeStep;
        std::vector<uint8_t>& vVisitedDiscretization = _cacheVVisitedDiscretization;
        vVisitedDiscretization.clear();
        int nEndTimeDiscretization = 0; // counts how many bins we discretize the trajectory given the current duration.

        //
        // Caching stuff
        //
        std::vector<dReal> &x0Vect = _cacheX0Vect, &x1Vect = _cacheX1Vect, &v0Vect = _cacheV0Vect, &v1Vect = _cacheV1Vect, &a0Vect = _cacheA0Vect, &a1Vect = _cacheA1Vect;
        std::vector<dReal> &velLimits = _cacheVellimits, &accelLimits = _cacheAccelLimits, &jerkLimits = _cacheJerkLimits;
        PiecewisePolynomials::Chunk& tempChunk = _cacheChunk;
        std::vector<PiecewisePolynomials::Chunk>& vChunksOut = _cacheVChunksOut; // for storing chunks from CheckChunkAllConstraints results

        //
        // Main shortcut loop
        //
        const dReal tOriginal = pwptraj.duration;
        dReal tTotal = tOriginal;
        int iter = 0;
        for(; iter < numIters; ++iter ) {
            if( tTotal < minTimeStep ) {
                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, tTotal=%.15e is too shortcut to continue (minTimeStep=%.15e)", _envId%iter%numIters%minTimeStep);
                break;
            }

            if( nItersFromPrevSuccessful + nTimeBasedConstraintsFailed > nCutoffIters ) {
                break;
            }
            ++nItersFromPrevSuccessful;

            // Sample two time instants.
            dReal t0, t1;
            if( iter == 0 ) {
                t0 = 0;
                t1 = tTotal;
            }
            // TODO: handle zero velocity points
            else {
                t0 = Rand()*tTotal;
                t1 = Rand()*tTotal;
                if( t0 > t1 ) {
                    PiecewisePolynomials::Swap(t0, t1);
                }
            }

            if( t1 - t0 < minTimeStep ) {
                // Time instants are too close.
                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e and t1=%.15e are too close (minTimeStep=%.15e)", _envId%iter%numIters%t0%t1%minTimeStep);
                continue;
            }

            // vVisitedDiscretization logic goes here
            {
                // Resize vVisitedDiscretization appropriately.
                if( vVisitedDiscretization.size() == 0 ) {
                    nEndTimeDiscretization = (int)(tTotal*fiMinDiscretization) + 1;
                    if( nEndTimeDiscretization <= (int)_nMaxDiscretizationSize ) {
                        vVisitedDiscretization.resize(nEndTimeDiscretization*nEndTimeDiscretization);
                    }
                }

                // Check redundancy of the sampled time instants
                int t0Index = t0*fiMinDiscretization;
                int t1Index = t1*fiMinDiscretization;
                size_t testPairIndex = t0Index*nEndTimeDiscretization + t1Index;
                if( testPairIndex < vVisitedDiscretization.size() ) {
                    if( vVisitedDiscretization[testPairIndex] ) {
                        // This bin has already been visited.
                        RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; testPairIndex=%d has been visited", _envId%iter%numIters%t0%t1%testPairIndex);
                        continue;
                    }
                    vVisitedDiscretization[testPairIndex] = 1;
                }
            }

            uint32_t iIterProgress = 0; // for debugging purposes
            try {
                // RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, shortcutting with t0=%.15e; t1=%.15e; fStartTimeVelMult=%.15f; fStartTimeAccelMult=%.15e", _envId%iter%numIters%t0%t1%fStartTimeVelMult%fStartTimeAccelMult);

                pwptraj.Eval(t0, x0Vect);
                if( _parameters->SetStateValues(x0Vect) != 0 ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; state setting at x0 failed", _envId%iter%numIters%t0%t1);
                    continue;
                }
                iIterProgress += 0x10000000;
                _parameters->_getstatefn(x0Vect);
                iIterProgress += 0x10000000;

                pwptraj.Eval(t1, x1Vect);
                if( _parameters->SetStateValues(x1Vect) != 0 ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; state setting at x1 failed", _envId%iter%numIters%t0%t1);
                    continue;
                }
                iIterProgress += 0x10000000;
                _parameters->_getstatefn(x1Vect);
                iIterProgress += 0x10000000;

                pwptraj.Evald1(t0, v0Vect);
                pwptraj.Evald2(t0, a0Vect);
                pwptraj.Evald1(t1, v1Vect);
                pwptraj.Evald2(t1, a1Vect);
                ++_progress._iteration;

                // The following variables are for debugging purposes
                size_t ichunk0, ichunk1;
                dReal rem0, rem1;
                pwptraj.FindChunkIndex(t0, ichunk0, rem0);
                pwptraj.FindChunkIndex(t1, ichunk1, rem1);

                // Set the limits for this iterations
                velLimits = _parameters->_vConfigVelocityLimit;
                accelLimits = _parameters->_vConfigAccelerationLimit;
                jerkLimits = _parameters->_vConfigJerkLimit;
                // {
                //     dReal fVelLowerBound, fAccelLowerBound;
                //     dReal fVel, fAccel;
                //     for( size_t idof = 0; idof < _ndof; ++idof ) {
                //         // The scaled velocity/acceleration must not be less than these values
                //         fVelLowerBound = std::max(RaveFabs(v0Vect[idof]), RaveFabs(v1Vect[idof]));
                //         fVel = std::max(fVelLowerBound, fStartTimeVelMult*_parameters->_vConfigVelocityLimit[idof]);
                //         if( velLimits[idof] > fVel ) {
                //             velLimits[idof] = fVel;
                //         }

                //         fAccelLowerBound = std::max(RaveFabs(a0Vect[idof]), RaveFabs(a1Vect[idof]));
                //         fAccel = std::max(fAccelLowerBound, fStartTimeAccelMult*_parameters->_vConfigAccelerationLimit[idof]);
                //         if( accelLimits[idof] > fAccel ) {
                //             accelLimits[idof] = fAccel;
                //         }
                //     }
                // }
                // These parameters keep track of multiplier for the current shortcut iteration.
                dReal fCurVelMult = fStartTimeVelMult;
                dReal fCurAccelMult = fStartTimeAccelMult;
                dReal fCurJerkMult = 1.0; // experimental
                // RAVELOG_DEBUG_FORMAT("env=%d, fCurVelMult=%f; fCurAccelMult=%f;", _envId%fCurVelMult%fCurAccelMult);

                bool bSuccess = false;
                dReal fTryDuration = t1 - t0;
                dReal fDurationMult = 1.1; // how much to increase the duration each time time-based constraints failed.
                dReal fCurDurationMult = 1.0; // greater than or equal to 1.0
                for( size_t iSlowDown = 0; iSlowDown < maxSlowDownTries; ++iSlowDown ) {
                    if( iSlowDown == 0 ) {
                        PiecewisePolynomials::PolynomialCheckReturn polycheckret = _quinticInterpolator.ComputeNDTrajectoryArbitraryTimeDerivativesOptimizeDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, velLimits, accelLimits, jerkLimits, fTryDuration, tempChunk);
                        if( polycheckret != PiecewisePolynomials::PCR_Normal ) {
                            RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; initial interpolation failed. polycheckret=0x%x", _envId%iter%numIters%t0%t1%polycheckret);
                            break; // must not slow down any further.
                        }
                        // If this slow down iteration fails due to time-based constraints, we will try to generate a slightly longer trajectory segment in the next iteration.
                        fTryDuration = tempChunk.duration;
                    }
                    else {
                        _quinticInterpolator.ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, fTryDuration*fCurDurationMult, tempChunk);
                        PiecewisePolynomials::PolynomialCheckReturn polycheckret = _limitsChecker.CheckChunk(tempChunk, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, velLimits, accelLimits, jerkLimits);
                        if( polycheckret != PiecewisePolynomials::PCR_Normal ) {
                            RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; iSlowDown=%d; current duration=%.15e; interpolation failed with polycheckret=0x%x", _envId%iter%numIters%t0%t1%iSlowDown%(fTryDuration*fCurDurationMult)%polycheckret);
                            fCurDurationMult *= fDurationMult;
                            continue; // maybe incrasing the duration might affect the peaks of vel/accel positively
                        }
                    }

                    if( tempChunk.duration + minTimeStep > t1 - t0 ) {
                        // Segment does not make significant improvement.
                        RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; shortcut does not make significant improvement. prevduration=%.15e; newduration=%.15e; diff=%.15e; minTimeStep=%.15e", _envId%iter%numIters%t0%t1%(t1 - t0)%tempChunk.duration%(t1 - t0 - tempChunk.duration)%minTimeStep);
                        break; // must not slow down any further.
                    }

                    if( _CallCallbacks(_progress) == PA_Interrupt ) {
                        return -1;
                    }
                    iIterProgress += 0x1000;

                    // Start checking constraints
                    PiecewisePolynomials::CheckReturn checkret = CheckChunkAllConstraints(tempChunk, 0xffff, vChunksOut);
                    iIterProgress += 0x1000;

                    if( checkret.retcode == 0 ) {
                        bSuccess = true;
                        break; // break out of slowdown loop.
                    }
                    else if( checkret.retcode == CFO_CheckTimeBasedConstraints ) {
                        ++nTimeBasedConstraintsFailed;
                        if( 0 ) {//( _bManipConstraints && !!_manipConstraintChecker ) {
                            // Scale down accelLimits and/or velLimits based on what constraints are violated.
                        }
                        else if( 1 ) {
                            // Experimental: try scaling the duration instead of vel/accel limits
                            fCurDurationMult *= fDurationMult;
                            continue;
                        }
                        else {
                            // No manip constraints. Scale down both velLimits and accelLimits
                            fCurVelMult *= checkret.fTimeBasedSurpassMult;
                            fCurAccelMult *= checkret.fTimeBasedSurpassMult*checkret.fTimeBasedSurpassMult;
                            fCurJerkMult *= checkret.fTimeBasedSurpassMult*checkret.fTimeBasedSurpassMult*checkret.fTimeBasedSurpassMult;
                            if( fCurVelMult < fVelMultCutoff ) {
                                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; fCurVelMult goes below threshold (%.15e < %.15e).", _envId%iter%numIters%t0%t1%fCurVelMult%fVelMultCutoff);
                                break;
                            }
                            if( fCurAccelMult < fAccelMultCutoff ) {
                                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; fCurAccelMult goes below threshold (%.15e < %.15e).", _envId%iter%numIters%t0%t1%fCurAccelMult%fAccelMultCutoff);
                                break;
                            }

                            ++numSlowDowns;
                            for( size_t idof = 0; idof < _ndof; ++idof ) {
                                dReal fVelLowerBound = std::max(RaveFabs(v0Vect[idof]), RaveFabs(v1Vect[idof]));
                                dReal fAccelLowerBound = std::max(RaveFabs(a0Vect[idof]), RaveFabs(a1Vect[idof]));
                                velLimits[idof] = std::max(fVelLowerBound, checkret.fTimeBasedSurpassMult*velLimits[idof]);
                                accelLimits[idof] = std::max(fAccelLowerBound, checkret.fTimeBasedSurpassMult*checkret.fTimeBasedSurpassMult*velLimits[idof]);
                                // Scaling down jerk limits likely leads to significantly slower final traj.
                                // jerkLimits[idof] *= (checkret.fTimeBasedSurpassMult*checkret.fTimeBasedSurpassMult*checkret.fTimeBasedSurpassMult);
                            }
                        }
                    }
                    else {
                        RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; rejecting shortcut due to ret=0x%x", _envId%iter%numIters%t0%t1%checkret.retcode);
                        break; // do not slow down any further.
                    }
                    iIterProgress += 0x1000;
                } // end slowdown iterations


                if( !bSuccess ) {
                    // Shortcut failed. Continue to the next iteration.
                    continue;
                }

                if( vChunksOut.size() == 0 ) {
                    RAVELOG_WARN_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; shortcut chunks vector is empty", _envId%iter%numIters%t0%t1);
                    continue;
                }

                // Now this shortcut iteration is really successful.
                ++numShortcuts;

                // Keep track of multipliers
                fStartTimeVelMult = min(1.0, fCurVelMult*fiSearchVelAccelMult);
                fStartTimeAccelMult = min(1.0, fCurAccelMult*fiSearchVelAccelMult);

                // Update parameters
                nTimeBasedConstraintsFailed = 0;
                vVisitedDiscretization.clear();
                dReal fSegmentTime = 0;
                FOREACHC(itChunk, vChunksOut) {
                    fSegmentTime += itChunk->duration;
                }
                dReal fDiff = (t1 - t0) - fSegmentTime;

                // Replace the original portion with the shortcut segment.
                pwptraj.ReplaceSegment(t0, t1, vChunksOut);
                // RAVELOG_DEBUG_FORMAT("env=%d, fSegmentTime=%f; fDiff=%f; prevduration=%f; newduration=%f", _envId%fSegmentTime%fDiff%tTotal%pwptraj.duration);
                tTotal = pwptraj.duration;
                fScore = fDiff/nItersFromPrevSuccessful;
                if( fScore > fCurrentBestScore ) {
                    fCurrentBestScore = fScore;
                }
                nItersFromPrevSuccessful = 0;

                if( (fScore/fCurrentBestScore < fCutoffRatio) && (numShortcuts > 5) ) {
                    // We have already shortcut for a bit (numShortcuts > 5). The progress made in this iteration is
                    // below the curoff ratio. If we continue, it is unlikely that we will make much more progress. So
                    // stop here.
                    break;
                }

                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; successful. numSlowDowns=%d, tTotal=%.15e", _envId%iter%numIters%t0%t1%numSlowDowns%pwptraj.duration);
            }
            catch( const std::exception& ex ) {
                RAVELOG_WARN_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; an exception occured. iIterProgress=0x%x: %s", _envId%iter%numIters%t0%t1%iIterProgress%ex.what());
                break;
            }
        } // end shortcut iterations

        // Report statistics
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        if( fScore/fCurrentBestScore < fCutoffRatio ) {
            ss << "current score falls below threshold (" << fScore/fCurrentBestScore << " < " << fCutoffRatio << ")";
        }
        else if( nItersFromPrevSuccessful + nTimeBasedConstraintsFailed > nCutoffIters ) {
            ss << "did not make progress in the past " << nItersFromPrevSuccessful << " iterations and";
            ss << " time-based constraints failed " << nTimeBasedConstraintsFailed << " times";
        }
        else {
            ss << "normal exit";
        }
        RAVELOG_DEBUG_FORMAT("env=%d, Finished at shortcut iter=%d/%d (%s), successful=%d; numSlowDowns=%d; duration: %.15e -> %.15e; diff=%.15e", _envId%iter%numIters%ss.str()%numShortcuts%numSlowDowns%tOriginal%tTotal%(tOriginal - tTotal));
        _DumpPiecewisePolynomialTrajectory(pwptraj, _dumpLevel);

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
    boost::shared_ptr<ManipConstraintChecker3> _manipConstraintChecker;
    PlannerProgress _progress;

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

    std::vector<uint8_t> _cacheVVisitedDiscretization;
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
