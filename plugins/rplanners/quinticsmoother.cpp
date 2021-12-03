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
#include "jerklimitedsmootherbase.h"
#define QUINTIC_SMOOTHER_PROGRESS_DEBUG

namespace rplanners {

namespace PiecewisePolynomials = PiecewisePolynomialsInternal;

class QuinticSmoother : public JerkLimitedSmootherBase {
public:
    QuinticSmoother(EnvironmentBasePtr penv, std::istream& sinput) : JerkLimitedSmootherBase(penv, sinput)
    {
    }

    virtual const char* GetPlannerName() const override
    {
        return "quinticsmoother";
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
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
        _DumpOpenRAVETrajectory(ptraj, "beforeshortcut", _dumpLevel);

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

        // bool bPathIsPerfectlyModeled = false; // will be true if the initial interpolation is linear or quintic
        RAVELOG_VERBOSE_FORMAT("env=%d, Initial trajectory joint values interpolation is %s", _envId%itcompatposgroup->interpolation);
        PlannerStatus conversionStatus = PS_Failed;
        if( _parameters->_hastimestamps && itcompatposgroup->interpolation == "quintic" ) {
            // bPathIsPerfectlyModeled = true;

            conversionStatus = ConvertOpenRAVETrajectoryToPiecewisePolynomialTrajectorySameInterpolation(ptraj, posSpec, velSpec, accelSpec, timeSpec, pwptraj);
        }
        // TODO: Maybe we need to handle other cases of interpolation as well
        else {
            if( itcompatposgroup->interpolation.size() == 0 || itcompatposgroup->interpolation == "linear" ) {
                // bPathIsPerfectlyModeled = true;
            }
            conversionStatus = ConvertOpenRAVEPathToPiecewisePolynomialTrajectory(ptraj, posSpec, pwptraj);
        }
        if( conversionStatus.statusCode != PS_HasSolution ) {
            return conversionStatus;
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
            if( !!_parameters->_setstatevaluesfn ) {
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
            std::vector<dReal> &x0Vect = _cacheX0Vect, &x1Vect = _cacheX1Vect, &v0Vect = _cacheV0Vect, &v1Vect = _cacheV1Vect, &a0Vect = _cacheA0Vect, &a1Vect = _cacheA1Vect;
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
            std::vector<PiecewisePolynomials::Chunk>& tempInterpolatedChunks = _cacheInterpolatedChunks; // for storing interpolation results
            std::vector<PiecewisePolynomials::Chunk>& tempCheckedChunks = _cacheCheckedChunks; // for storing results from CheckChunkAllConstraints
            std::vector<PiecewisePolynomials::Chunk>& vFinalChunks = _cacheFinalChunks; // for storing chunks before putting them into the final trajcetory
            PiecewisePolynomials::Chunk &trimmedChunk = _cacheTrimmedChunk, &remChunk = _cacheRemChunk;

            for( std::vector<PiecewisePolynomials::Chunk>::const_iterator itChunk = pwptraj.vchunks.begin(); itChunk != pwptraj.vchunks.end(); ++itChunk ) {
                ++_progress._iteration;
                if( _CallCallbacks(_progress) == PA_Interrupt ) {
                    return PS_Interrupted;
                }

                trimmedChunk = *itChunk;
                ++_progress._iteration;

                // Check constraints if not yet checked
                if( itChunk->constraintChecked ) {
                    vFinalChunks.resize(1);
                    vFinalChunks[0] = trimmedChunk;
                }
                else {
                    // Check joint limits + velocity/acceleration/jerk limits
                    int limitsret = _limitsChecker.CheckChunkLimits(trimmedChunk, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit, _parameters->_vConfigJerkLimit);
                    if( limitsret != PiecewisePolynomials::PCR_Normal ) {
                        RAVELOG_WARN_FORMAT("env=%d, Detected limits violation after shortcutting; iChunk=%d; limitsret=0x%x", _envId%(itChunk - pwptraj.vchunks.begin())%limitsret);
                        return PS_Failed;
                    }

                    // Check other remaining constraints. Ignore the first and the last bits of the trajectory.
                    bool bTrimmedFront = false;
                    bool bTrimmedBack = false;
                    bool bCheck = true;
                    if( itChunk - pwptraj.vchunks.begin() == 0 ) {
                        if( itChunk->duration <= fTrimEdgesTime + PiecewisePolynomials::g_fPolynomialEpsilon ) {
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
                        if( itChunk->duration <= fTrimEdgesTime + PiecewisePolynomials::g_fPolynomialEpsilon ) {
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
                        PiecewisePolynomials::CheckReturn checkret = CheckChunkAllConstraints(trimmedChunk, defaultCheckOptions, tempCheckedChunks);
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
                                PolynomialCheckReturn interpolatorret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration
                                                                            (x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, newChunkDuration,
                                                                            _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit,
                                                                            _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit, _parameters->_vConfigJerkLimit,
                                                                            tempInterpolatedChunks);

                                if( interpolatorret == PolynomialCheckReturn::PCR_Normal ) {
                                    // TODO
                                    // For quintic interpolator, tempInterpolatedChunks always has one element.
                                    PiecewisePolynomials::CheckReturn newcheckret = CheckAllChunksAllConstraints(tempInterpolatedChunks, defaultCheckOptions, tempCheckedChunks);
                                    if( newcheckret.retcode == 0 ) {
                                        // The new chunk passes constraints checking.
                                        bDilationSuccessful = true;
                                        break;
                                    }
                                }

                                if( iDilation > 1 ) {
                                    newChunkDuration += timeIncrement;
                                }
                                else {
                                    // Increase the duration a tiny bit first
                                    newChunkDuration += 5*PiecewisePolynomials::g_fPolynomialEpsilon;
                                }
                            }
                            if( !bDilationSuccessful ) {
                                RAVELOG_WARN_FORMAT("env=%d, Failed checking constraints of iChunk=%d/%d", _envId%(itChunk - pwptraj.vchunks.begin())%pwptraj.vchunks.size());
                                _DumpOpenRAVETrajectory(ptraj, "faileddilation", _errorDumpLevel);
                                return PS_Failed;
                            }
                        }
                    } // end bCheck

                    vFinalChunks.resize(0);
                    if( bTrimmedFront ) {
                        vFinalChunks.push_back(remChunk);
                    }
                    vFinalChunks.insert(vFinalChunks.end(), tempCheckedChunks.begin(), tempCheckedChunks.end());
                    if( bTrimmedBack ) {
                        vFinalChunks.push_back(remChunk);
                    }

                    ++_progress._iteration;
                    if( _CallCallbacks(_progress) == PA_Interrupt ) {
                        return PS_Interrupted;
                    }
                } // end if( !itChunk->constraintChecked )

                FOREACH(itNewChunk, vFinalChunks) {
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
            _DumpOpenRAVETrajectory(ptraj, "failedexception", _errorDumpLevel);
            RAVELOG_WARN_FORMAT("env=%d, Main planning loop threw an expection: %s", _envId%ex.what());
            return PS_Failed;
        }
        RAVELOG_DEBUG_FORMAT("env=%d, path optimizing - computation time=%f", _envId%(0.001f*(dReal)(utils::GetMilliTime() - startTime)));

        // Save the final trajectory
        _DumpOpenRAVETrajectory(ptraj, "final", _dumpLevel);

        return _ProcessPostPlanners(RobotBasePtr(), ptraj);
    } // end PlanPath

    virtual int _Shortcut(PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj, int numIters, dReal minTimeStep) override
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
        std::vector<PiecewisePolynomials::Chunk>& tempChunks = _cacheInterpolatedChunks; // for storing interpolation result
        std::vector<PiecewisePolynomials::Chunk>& vChunksOut = _cacheCheckedChunks; // for storing chunks from CheckAllChunksAllConstraints results

        //
        // Main shortcut loop
        //
        const dReal tOriginal = pwptraj.duration;
        dReal tTotal = tOriginal;
        int iter = 0;
        for(; iter < numIters; ++iter ) {
            if( tTotal < minTimeStep ) {
                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, tTotal=%.15e is too shortcut to continue (minTimeStep=%.15e)", _envId%iter%numIters%tTotal%minTimeStep);
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
                    dReal fChunksDuration = 0;
                    if( iSlowDown == 0 ) {
                        PiecewisePolynomials::PolynomialCheckReturn polycheckret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration
                                                                                       (x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect,
                                                                                       _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit,
                                                                                       velLimits, accelLimits, jerkLimits, fTryDuration, tempChunks);
                        if( polycheckret != PolynomialCheckReturn::PCR_Normal ) {
                            RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; initial interpolation failed. polycheckret=0x%x", _envId%iter%numIters%t0%t1%polycheckret);
                            break; // must not slow down any further.
                        }
                        // If this slow down iteration fails due to time-based constraints, we will try to generate a slightly longer trajectory segment in the next iteration.
                        FOREACHC(itchunk, tempChunks) {
                            fChunksDuration += itchunk->duration;
                        }
                        fTryDuration = fChunksDuration;
                    }
                    else {
                        PiecewisePolynomials::PolynomialCheckReturn polycheckret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration
                                                                                       (x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, fTryDuration*fCurDurationMult,
                                                                                       _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit,
                                                                                       velLimits, accelLimits, jerkLimits, tempChunks);
                        if( polycheckret != PolynomialCheckReturn::PCR_Normal ) {
                            dReal prevTryDuration = fTryDuration*fCurDurationMult;
                            fCurDurationMult *= fDurationMult;
                            if( fTryDuration*fCurDurationMult + minTimeStep > t1 - t0 ) {
                                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; iSlowDown=%d; current duration=%.15e; interpolation failed with polycheckret=0x%x. shortcut will not make significant improvement.", _envId%iter%numIters%t0%t1%iSlowDown%(prevTryDuration)%polycheckret);
                                break; // must not slow down any further
                            }
                            RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; iSlowDown=%d; current duration=%.15e; interpolation failed with polycheckret=0x%x.", _envId%iter%numIters%t0%t1%iSlowDown%(prevTryDuration)%polycheckret);
                            continue; // maybe incrasing the duration might affect the peaks of vel/accel positively
                        }
                    }

                    if( fChunksDuration + minTimeStep > t1 - t0 ) {
                        // Segment does not make significant improvement.
                        RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; shortcut does not make significant improvement. prevduration=%.15e; newduration=%.15e; diff=%.15e; minTimeStep=%.15e", _envId%iter%numIters%t0%t1%(t1 - t0)%fChunksDuration%(t1 - t0 - fChunksDuration)%minTimeStep);
                        break; // must not slow down any further.
                    }

                    if( _CallCallbacks(_progress) == PA_Interrupt ) {
                        return -1;
                    }
                    iIterProgress += 0x1000;

                    // Start checking constraints
                    PiecewisePolynomials::CheckReturn checkret = CheckAllChunksAllConstraints(tempChunks, defaultCheckOptions, vChunksOut);
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
                if( iter == 0 ) {
                    // Since replacing the entire initial trajectory, just initial pwptraj with vChunksOut directly.
                    pwptraj.Initialize(vChunksOut);
                }
                else {
                    pwptraj.ReplaceSegment(t0, t1, vChunksOut);
                }
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
                RAVELOG_WARN_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; an exception occurred. iIterProgress=0x%x: %s", _envId%iter%numIters%t0%t1%iIterProgress%ex.what());
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
        _DumpPiecewisePolynomialTrajectory(pwptraj, "aftershortcut", _dumpLevel);

        return numShortcuts;
    }

    /// \brief Verify that the input sequence of chunks satisfy all constraints (including collisions, manip speed/accel, and possibly dynamics).
    virtual PiecewisePolynomials::CheckReturn CheckAllChunksAllConstraints(const std::vector<PiecewisePolynomials::Chunk>& vChunksIn, int options, std::vector<PiecewisePolynomials::Chunk>& vChunksOut) override
    {
        // For now, just suppose that vChunksIn has only one chunk (which is of course true when
        // using the current quinticinterpolator)
        const PiecewisePolynomials::Chunk& chunk = vChunksIn.front();
        // Make sure the first configuration is safe
        std::vector<dReal> &x0Vect = _cacheX0Vect2, &v0Vect = _cacheV0Vect2, &a0Vect = _cacheA0Vect2;
        chunk.Eval(0, x0Vect);
        chunk.Evald1(0, v0Vect);
        chunk.Evald2(0, a0Vect);
        PiecewisePolynomials::CheckReturn checkret = CheckConfigAllConstraints(x0Vect, v0Vect, a0Vect, options);
        if( checkret.retcode != 0 ) {
            return checkret;
        }
        // Verify the rest.
        return CheckChunkAllConstraints(vChunksIn.front(), options, vChunksOut);
    }

protected:

    virtual void _InitializeInterpolator() override
    {
        _pinterpolator.reset(new PiecewisePolynomials::QuinticInterpolator(_ndof, _envId));
        _maskinterpolation = IT_Quintic;
    }

    virtual PiecewisePolynomials::CheckReturn _ProcessConstraintReturnIntoChunks(ConstraintFilterReturnPtr contraintReturn, const PiecewisePolynomials::Chunk chunkIn,
                                                                                 const bool bMarkConstraintChecked,
                                                                                 std::vector<dReal>& x0Vect, std::vector<dReal>& x1Vect,
                                                                                 std::vector<dReal>& v0Vect, std::vector<dReal>& v1Vect,
                                                                                 std::vector<dReal>& a0Vect, std::vector<dReal>& a1Vect,
                                                                                 std::vector<PiecewisePolynomials::Chunk>& vChunksOut) override
    {
        std::vector<PiecewisePolynomials::Chunk>& tempChunks = _cacheInterpolatedChunksDuringCheck;
        dReal curTime = 0;
        std::vector<dReal>::const_iterator it = _constraintReturn->_configurations.begin();
        if( vChunksOut.capacity() < _constraintReturn->_configurationtimes.size() ) {
            vChunksOut.reserve(_constraintReturn->_configurationtimes.size());
        }

        std::vector<dReal>& lowerPositionLimit = _cacheLowerLimits;
        std::vector<dReal>& upperPositionLimit = _cacheUpperLimits;
        std::vector<dReal>& positionAllownace = _cacheResolutions;
        for( size_t idof = 0; idof < _ndof; ++idof ) {
            // Allow each dof to go beyond their temporary limit by this much.
            positionAllownace[idof] = 0.01*_parameters->_vConfigResolution[idof];
        }

        for( size_t itime = 0; itime < _constraintReturn->_configurationtimes.size(); ++itime, it += _ndof ) {
            // Retrieve the next config from _constraintReturn. Velocity and acceleration are
            // evaluated from the original chunk.
            std::copy(it, it + _ndof, x1Vect.begin());
            chunkIn.Evald1(_constraintReturn->_configurationtimes[itime], v1Vect);
            chunkIn.Evald2(_constraintReturn->_configurationtimes[itime], a1Vect);

            dReal deltaTime = _constraintReturn->_configurationtimes[itime] - curTime;
            if( deltaTime > PiecewisePolynomials::g_fPolynomialEpsilon ) {
                // Since CheckPathAllConstraints might have modified the configurations on chunkIn due to constraints,
                // when reconstructing chunks from constraintReturn, should make sure again that the reconstructed
                // chunks do not have overshooting positions as we may not be checking constraints on these chunks
                // again.

                // TODO: can actually deduce whether or not there are some extra constraints used in the check
                // function. If not, then can skip these chunk reconstruction entirely.

                // Set up temporary position limits for chunk reconstruction.
                for( size_t idof = 0; idof < _ndof; ++idof ) {
                    if( x0Vect[idof] > x1Vect[idof] ) {
                        lowerPositionLimit[idof] = x1Vect[idof];
                        upperPositionLimit[idof] = x0Vect[idof];
                    }
                    else {
                        lowerPositionLimit[idof] = x0Vect[idof];
                        upperPositionLimit[idof] = x1Vect[idof];
                    }
                    if( upperPositionLimit[idof] - lowerPositionLimit[idof] < _parameters->_vConfigResolution[idof] ) {
                        const dReal midPoint = 0.5*(upperPositionLimit[idof] + lowerPositionLimit[idof]);
                        const dReal allowance = 0.5*_parameters->_vConfigResolution[idof] + positionAllownace[idof];
                        lowerPositionLimit[idof] = midPoint - allowance;
                        upperPositionLimit[idof] = midPoint + allowance;
                    }
                    else {
                        lowerPositionLimit[idof] -= positionAllownace[idof];
                        upperPositionLimit[idof] += positionAllownace[idof];
                    }
                }
                PiecewisePolynomials::PolynomialCheckReturn interpolatorret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration
                                                                                  (x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, deltaTime,
                                                                                  lowerPositionLimit, upperPositionLimit,
                                                                                  _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit, _parameters->_vConfigJerkLimit,
                                                                                  tempChunks);

                if( interpolatorret != PiecewisePolynomials::PCR_Normal ) {
                    RAVELOG_VERBOSE_FORMAT("env=%d, the output chunk is invalid: t=%f/%f; ret=%d", _envId%curTime%chunkIn.duration%interpolatorret);
                    return PiecewisePolynomials::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9);
                }

                FOREACH(itchunk, tempChunks) {
                    vChunksOut.push_back(*itchunk);
                    vChunksOut.back().constraintChecked = bMarkConstraintChecked;
                }
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

        return PiecewisePolynomials::CheckReturn(); // successful
    } // end ProcessConstraintReturnIntoChunks

private:

    std::vector<PiecewisePolynomials::Chunk> _cacheFinalChunks; // for storing chunks before putting them into the final trajcetory
    PiecewisePolynomials::Chunk _cacheTrimmedChunk, _cacheRemChunk; ///< for constraints checking at the very end

    // For use during CheckX process
    std::vector<PiecewisePolynomials::Chunk> _cacheInterpolatedChunksDuringCheck;

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
