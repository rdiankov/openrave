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

#include "piecewisepolynomials/cubicinterpolator.h"
#include "jerklimitedsmootherbase.h"

namespace rplanners {

namespace PiecewisePolynomials = PiecewisePolynomialsInternal;

class CubicSmoother : public JerkLimitedSmootherBase {
public:
    CubicSmoother(EnvironmentBasePtr penv, std::istream& sinput) : JerkLimitedSmootherBase(penv, sinput)
    {
        __description = ":Interface Author: Puttichai Lertkultanon\n\nTime-parameterizes the given path using cubic polynomials and performs trajectory smoothing.";
    }

    virtual const char* GetPlannerName() const override
    {
        return "cubicsmoother";
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        uint32_t startTime = utils::GetMilliTime();

        _limitsChecker.SetEpsilonForAccelerationDiscrepancyChecking(100*PiecewisePolynomials::g_fPolynomialEpsilon); // this follows cubic interpolator (see comments in CubicInterpolator::Initialize).

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
        std::stringstream ssenablestate;
        std::vector<uint8_t> vEnableStates;
        std::vector<KinBodyPtr> vGrabbedBodies;
        FOREACH(itbody, vusedbodies) {
            KinBody::KinBodyStateSaverPtr statesaver;
            if( (*itbody)->IsRobot() ) {
                {
                    ssenablestate << "\"" << (*itbody)->GetName() << "\": [";
                    (*itbody)->GetLinkEnableStates(vEnableStates);
                    FOREACHC(it, vEnableStates) {
                        ssenablestate << (int)*it << ", ";
                    }
                    ssenablestate << "], ";

                    (*itbody)->GetGrabbed(vGrabbedBodies);
                    FOREACHC(itGrabbed, vGrabbedBodies) {
                        ssenablestate << "\"" << (*itGrabbed)->GetName() << "\": [";
                        (*itGrabbed)->GetLinkEnableStates(vEnableStates);
                        FOREACHC(it, vEnableStates) {
                            ssenablestate << (int)*it << ", ";
                        }
                        ssenablestate << "], ";
                    }
                }
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
        ConfigurationSpecification newSpec; // for the final trajectory

        // Get joint values from the passed-in OpenRAVE trajectory
        bool bExactMatch = false;
        std::vector<ConfigurationSpecification::Group>::const_iterator itcompatposgroup = ptraj->GetConfigurationSpecification().FindCompatibleGroup(posSpec._vgroups.at(0), bExactMatch);
        OPENRAVE_ASSERT_FORMAT(itcompatposgroup != ptraj->GetConfigurationSpecification()._vgroups.end(), "env=%d, Failed to find group %s in the passed-in trajectory", _envId%posSpec._vgroups.at(0).name, ORE_InvalidArguments);

        // Caching stuff
        PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj = _cacheTraj;
        pwptraj.Reset();

        // bool bPathIsPerfectlyModeled = false; // will be true if the initial interpolation is linear or cubic
        RAVELOG_VERBOSE_FORMAT("env=%d, Initial trajectory joint values interpolation is %s", _envId%itcompatposgroup->interpolation);
        PlannerStatus conversionStatus = PS_Failed;
        if( _parameters->_hastimestamps && itcompatposgroup->interpolation == "cubic" ) {
            // bPathIsPerfectlyModeled = true;
            fEstimatedVelMult = 1.0;
            conversionStatus = ConvertOpenRAVETrajectoryToPiecewisePolynomialTrajectorySameInterpolation(ptraj, posSpec, velSpec, accelSpec, timeSpec, pwptraj);
        }
        // TODO: Maybe we need to handle other cases of interpolation as well
        else {
            if( itcompatposgroup->interpolation.size() == 0 || itcompatposgroup->interpolation == "linear" ) {
                // bPathIsPerfectlyModeled = true;
            }
            fEstimatedVelMult = 0.0; // will be computed as the average of vel mults computed in the following function
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
            RAVELOG_DEBUG_FORMAT("env=%d, initial segments=%d; duration=%.15e; verifyinitialpath=%d; enablestates={%s}", _envId%pwptraj.vchunks.size()%pwptraj.duration%_parameters->verifyinitialpath%ssenablestate.str());
            _progress._iteration = 0;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return PS_Interrupted;
            }

            _DumpPiecewisePolynomialTrajectory(pwptraj, "beforeshortcut", _dumpLevel);

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
            newSpec = posSpec;
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
                    itgroup->interpolation = "cubic";
                }
                else if( velSpec.FindCompatibleGroup(*itgroup) != velSpec._vgroups.end() ) {
                    itgroup->interpolation = "quadratic";
                }
                else if( accelSpec.FindCompatibleGroup(*itgroup) != accelSpec._vgroups.end() ) {
                    itgroup->interpolation = "linear";
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
            // dReal fTrimEdgesTime = _parameters->_fStepLength*2; // we ignore constraints during [0, fTrimEdgesTime] and [duration - fTrimEdgesTime, duration]
            std::vector<PiecewisePolynomials::Chunk>& tempInterpolatedChunks = _cacheInterpolatedChunks; // for storing interpolation results
            std::vector<PiecewisePolynomials::Chunk>& tempCheckedChunks = _cacheCheckedChunks; // for storing results from CheckChunkAllConstraints
            std::vector<PiecewisePolynomials::Chunk>& vFinalChunks = _cacheFinalChunks; // for storing chunks before putting them into the final trajcetory
            PiecewisePolynomials::Chunk &trimmedChunk = _cacheTrimmedChunk, &remChunk = _cacheRemChunk;

            _bUsePerturbation = false; // turn checking with perturbation off here.
#ifdef JERK_LIMITED_SMOOTHER_VALIDATE
            size_t iOriginalChunk = 0;
            int prevChunkShortcutIter = -1;
            for( std::vector<PiecewisePolynomials::Chunk>::const_iterator itChunk = pwptraj.vchunks.begin(); itChunk != pwptraj.vchunks.end(); ++itChunk, ++iOriginalChunk ) {
#else
            for( std::vector<PiecewisePolynomials::Chunk>::const_iterator itChunk = pwptraj.vchunks.begin(); itChunk != pwptraj.vchunks.end(); ++itChunk ) {
#endif
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
                    // I don't think we need this custom trimming logic anymore.
                    // if( itChunk - pwptraj.vchunks.begin() == 0 ) {
                    //     if( itChunk->duration <= fTrimEdgesTime + PiecewisePolynomials::g_fPolynomialEpsilon ) {
                    //         // This chunk is too short. Skip checking
                    //         bCheck = false;
                    //     }
                    //     else {
                    //         remChunk = trimmedChunk;
                    //         remChunk.Cut(fTrimEdgesTime, trimmedChunk);
                    //         bTrimmedFront = true;
                    //     }
                    // }
                    // else if( itChunk + 1 == pwptraj.vchunks.end() ) {
                    //     if( itChunk->duration <= fTrimEdgesTime + PiecewisePolynomials::g_fPolynomialEpsilon ) {
                    //         // This chunk is too short. Skip checking
                    //         bCheck = false;
                    //     }
                    //     else {
                    //         trimmedChunk.Cut(trimmedChunk.duration - fTrimEdgesTime, remChunk);
                    //         bTrimmedBack = true;
                    //     }
                    // }

                    if( bCheck ) {
                        trimmedChunk.Eval(0, x0Vect);
                        trimmedChunk.Evald1(0, v0Vect);
                        trimmedChunk.Evald2(0, a0Vect);
                        PiecewisePolynomials::CheckReturn initialcheckret = CheckConfigAllConstraints(x0Vect, v0Vect, a0Vect, defaultCheckOptions);
                        if( initialcheckret.retcode != 0 ) {
                            RAVELOG_WARN_FORMAT("env=%d, Failed checking constraints at initial config of iChunk=%d/%d with retcode=0x%x", _envId%(itChunk - pwptraj.vchunks.begin())%pwptraj.vchunks.size()%initialcheckret.retcode);
                            _DumpOpenRAVETrajectory(ptraj, "failedfinalcheck", _errorDumpLevel);
                            return PS_Failed;
                        }

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

#ifdef JERK_LIMITED_SMOOTHER_VALIDATE
                // Verify that chunks are continuously connected.
                BOOST_ASSERT(vFinalChunks.size() > 0);

                if( iOriginalChunk > 0 ) {
                    PiecewisePolynomials::Chunk& prevChunk = pwptraj.vchunks[iOriginalChunk - 1];
                    prevChunk.Eval(prevChunk.duration, x0Vect);
                    prevChunk.Evald1(prevChunk.duration, v0Vect);
                    prevChunk.Evald2(prevChunk.duration, a0Vect);
                }
                else {
                    itChunk->Eval(0, x0Vect);
                    itChunk->Evald1(0, v0Vect);
                    itChunk->Evald2(0, a0Vect);
                }
                size_t isubchunk = 0;
                FOREACHC(itTestChunk, vFinalChunks) {
                    PiecewisePolynomials::PolynomialCheckReturn chunkcheckret = _limitsChecker.CheckChunkValues(*itTestChunk, 0, x0Vect, v0Vect, a0Vect);
                    if( chunkcheckret != PolynomialCheckReturn::PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                        RAVELOG_WARN_FORMAT("env=%d, idof=%d; value=%.15e; expected=%.15e; result=%s", _envId%_limitsChecker._failedDOF%_limitsChecker._failedValue%_limitsChecker._expectedValue%PiecewisePolynomials::GetPolynomialCheckReturnString(chunkcheckret));
#endif
                        _DumpPiecewisePolynomialTrajectory(pwptraj, "beforconversion", static_cast<DebugLevel>(RaveGetDebugLevel()));
                        PiecewisePolynomials::PiecewisePolynomialTrajectory subtraj(vFinalChunks);
                        _DumpPiecewisePolynomialTrajectory(subtraj, boost::str(boost::format("chunk%d")%(itChunk - pwptraj.vchunks.begin())).c_str(), static_cast<DebugLevel>(RaveGetDebugLevel()));
                        OPENRAVE_ASSERT_FORMAT(chunkcheckret == PolynomialCheckReturn::PCR_Normal, "got checkret=%s when checking the start of isubchunk=%d/%d; ichunk=%d/%d; introduced in shortcut iter=%d; constraintChecked=%d; prev chunk shortcut iter=%d", PiecewisePolynomials::GetPolynomialCheckReturnString(chunkcheckret)%isubchunk%vFinalChunks.size()%iOriginalChunk%pwptraj.vchunks.size()%(itChunk->_iteration)%(itChunk->constraintChecked)%prevChunkShortcutIter, ORE_InconsistentConstraints);
                    }

                    ++isubchunk;
                    itTestChunk->Eval(itTestChunk->duration, x0Vect);
                    itTestChunk->Evald1(itTestChunk->duration, v0Vect);
                    itTestChunk->Evald2(itTestChunk->duration, a0Vect);
                }

                // Now x0Vect, v0Vect, and a0Vect hold the final values of the last chunk of vFinalChunks
                PiecewisePolynomials::PolynomialCheckReturn lastchunkcheckret = _limitsChecker.CheckChunkValues(*itChunk, itChunk->duration, x0Vect, v0Vect, a0Vect);
                if( lastchunkcheckret != PolynomialCheckReturn::PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                    RAVELOG_WARN_FORMAT("env=%d, idof=%d; value=%.15e; expected=%.15e; result=%s", _envId%_limitsChecker._failedDOF%_limitsChecker._failedValue%_limitsChecker._expectedValue%PiecewisePolynomials::GetPolynomialCheckReturnString(lastchunkcheckret));
#endif
                    _DumpPiecewisePolynomialTrajectory(pwptraj, "beforconversion", static_cast<DebugLevel>(RaveGetDebugLevel()));
                    PiecewisePolynomials::PiecewisePolynomialTrajectory subtraj(vFinalChunks);
                    _DumpPiecewisePolynomialTrajectory(subtraj, boost::str(boost::format("chunk%d")%(itChunk - pwptraj.vchunks.begin())).c_str(), static_cast<DebugLevel>(RaveGetDebugLevel()));
                    OPENRAVE_ASSERT_FORMAT(lastchunkcheckret == PolynomialCheckReturn::PCR_Normal, "got checkret=%s when checking the end of isubchunk=%d/%d; ichunk=%d/%d; introduced in shortcut iter=%d; constraintChecked=%d", PiecewisePolynomials::GetPolynomialCheckReturnString(lastchunkcheckret)%isubchunk%vFinalChunks.size()%iOriginalChunk%pwptraj.vchunks.size()%(itChunk->_iteration)%(itChunk->constraintChecked), ORE_InconsistentConstraints);
                }

                prevChunkShortcutIter = itChunk->_iteration;
#endif

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
#ifdef JERK_LIMITED_SMOOTHER_VALIDATE
            throw;
#else
            return PS_Failed;
#endif
        }
        RAVELOG_DEBUG_FORMAT("env=%d, path optimizing - computation time=%f", _envId%(0.001f*(dReal)(utils::GetMilliTime() - startTime)));

        // Save the final trajectory
        _DumpOpenRAVETrajectory(ptraj, "final", _dumpLevel);

#ifdef JERK_LIMITED_SMOOTHER_VALIDATE
        if( _parameters->verifyinitialpath ) {
            // When verifyinitialpath is true, at this point every single waypoint has to pass constraints checking. So can do the following validation.
            const int oldDebugLevel = RaveGetDebugLevel();
            RaveSetDebugLevel(Level_Verbose);

            // Check to really make sure that all waypoints are certainly at least collision-free.
            std::vector<dReal>& waypoint = _cacheAllWaypoints; // reuse _cacheAllWaypoints
            std::vector<dReal> &x0Vect = _cacheX0Vect, &v0Vect = _cacheV0Vect, &a0Vect = _cacheA0Vect;
            for( int iwaypoint = 0; iwaypoint < (int)ptraj->GetNumWaypoints(); ++iwaypoint ) {
                // Get waypoint data from the trajectory
                ptraj->GetWaypoint(iwaypoint, waypoint, newSpec);

                // Extract joint values, velocities, and accelerations from the data
                ConfigurationSpecification::ConvertData(x0Vect.begin(), posSpec, // target data
                                                        waypoint.begin(), newSpec, // source data
                                                        /*numWaypoints*/ 1, GetEnv(), /*filluninitialized*/ true);
                ConfigurationSpecification::ConvertData(v0Vect.begin(), velSpec, // target data
                                                        waypoint.begin(), newSpec, // source data
                                                        /*numWaypoints*/ 1, GetEnv(), /*filluninitialized*/ true);
                ConfigurationSpecification::ConvertData(a0Vect.begin(), accelSpec, // target data
                                                        waypoint.begin(), newSpec, // source data
                                                        /*numWaypoints*/ 1, GetEnv(), /*filluninitialized*/ true);
                PiecewisePolynomials::CheckReturn checkret = CheckConfigAllConstraints(x0Vect, v0Vect, a0Vect, defaultCheckOptions);
                if( checkret.retcode != 0 ) {
                    std::stringstream ss;
                    ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                    ss << "xVect=[";
                    SerializeValues(ss, x0Vect);
                    ss << "]; vVect=[";
                    SerializeValues(ss, v0Vect);
                    ss << "]; aVect=[";
                    SerializeValues(ss, a0Vect);
                    ss << "];";
                    OPENRAVE_ASSERT_OP_FORMAT(checkret.retcode, ==, 0, "env=%d, got retcode=0x%x at iwaypoint=%d/%d: %s", _envId%checkret.retcode%iwaypoint%ptraj->GetNumWaypoints()%ss.str(), ORE_InconsistentConstraints);
                }
            }
            RaveSetDebugLevel(oldDebugLevel);
            RAVELOG_INFO_FORMAT("env=%d, final validation successful with traj of %d waypoints", _envId%ptraj->GetNumWaypoints());
        }
        else {
            RAVELOG_INFO_FORMAT("env=%d, skipped final validation with traj of %d waypoints", _envId%ptraj->GetNumWaypoints());
        }
#endif
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
        std::vector<dReal> &vVelLowerBound = _cacheVelLowerBound, &vAccelLowerBound = _cacheAccelLowerBound; // lower bounds of how much we can scale down velocity/acceleration limits in a certain shortcutting iteration

        //
        // Main shortcut loop
        //
        const dReal tOriginal = pwptraj.duration;
        dReal tTotal = tOriginal;
        int iter = 0;
        int lastSuccessfulShortcutIter = -1;
        for(; iter < numIters; ++iter ) {
            if( tTotal < minTimeStep ) {
                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, tTotal=%.15e is too shortcut to continue (minTimeStep=%.15e)", _envId%iter%numIters%tTotal%minTimeStep);
                break;
            }

            if( !CORRECT_VELACCELMULT ) {
                // When using correct vel/accel mult, for now expect to get a lot more of slowing down iterations
                if( nItersFromPrevSuccessful + nTimeBasedConstraintsFailed > nCutoffIters ) {
                    break;
                }
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
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                ++_vShortcutStats[SS_TimeInstantsTooClose];
#endif
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
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                        ++_vShortcutStats[SS_RedundantShortcut];
#endif
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
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                    ++_vShortcutStats[SS_StateSettingFailed];
#endif
                    continue;
                }
                iIterProgress += 0x10000000;
                _parameters->_getstatefn(x0Vect);
                iIterProgress += 0x10000000;

                pwptraj.Eval(t1, x1Vect);
                if( _parameters->SetStateValues(x1Vect) != 0 ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; state setting at x1 failed", _envId%iter%numIters%t0%t1);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                    ++_vShortcutStats[SS_StateSettingFailed];
#endif
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

                if( ichunk0 == ichunk1 ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e are on the same chunk", _envId%iter%numIters%t0%t1);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                    ++_vShortcutStats[SS_TimeInstantsTooClose];
#endif
                    continue;
                }

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
                // // These parameters keep track of multiplier for the current shortcut iteration.
                // dReal fCurVelMult = fStartTimeVelMult;
                // dReal fCurAccelMult = fStartTimeAccelMult;
                // dReal fCurJerkMult = 1.0; // experimental
                // RAVELOG_DEBUG_FORMAT("env=%d, fCurVelMult=%f; fCurAccelMult=%f;", _envId%fCurVelMult%fCurAccelMult);

                {
                    // Precompute the velocity/acceleration lower bounds from the boundary conditions.
                    for( size_t idof = 0; idof < _ndof; ++idof ) {
                        vVelLowerBound[idof] = std::max(RaveFabs(v0Vect[idof]), RaveFabs(v1Vect[idof]));
                        vAccelLowerBound[idof] = std::max(RaveFabs(a0Vect[idof]), RaveFabs(a1Vect[idof]));
                    }
                }

                // These parameters keep track of multiplier for the current shortcut iteration.
                dReal fCurVelMult;
                dReal fCurAccelMult;
                dReal fCurJerkMult;
                if( USE_ESTIMATED_VELMULT && fEstimatedVelMult > 0 ) {
                    fCurVelMult = fEstimatedVelMult;
                    fCurAccelMult = fCurVelMult * fEstimatedVelMult;
                    fCurJerkMult = fCurAccelMult * fEstimatedVelMult;
                    if( fCurVelMult < 1.0 - PiecewisePolynomials::g_fPolynomialEpsilon ) {
                        dReal fVelLowerBound, fAccelLowerBound;
                        for( size_t idof = 0; idof < _ndof; ++idof ) {
                            fVelLowerBound = vVelLowerBound[idof];
                            if( !PiecewisePolynomials::FuzzyEquals(fVelLowerBound, velLimits[idof], PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                                velLimits[idof] = std::max(fVelLowerBound, fCurVelMult*velLimits[idof]);
                            }
                            fAccelLowerBound = vAccelLowerBound[idof];
                            if( !PiecewisePolynomials::FuzzyEquals(fAccelLowerBound, accelLimits[idof], PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                                accelLimits[idof] = std::max(fAccelLowerBound, fCurAccelMult*accelLimits[idof]);
                            }
                            jerkLimits[idof] *= fCurJerkMult;
                        }
                    }
                }
                else {
                    fCurVelMult = fStartTimeVelMult;
                    fCurAccelMult = fStartTimeAccelMult;
                    fCurJerkMult = 1.0; // experimental
                }

                if( 0 ) {
                    std::stringstream ssdebug;
                    _FormatInterpolationConditions(ssdebug, x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect,
                                                   t1 - t0,
                                                   _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit,
                                                   velLimits, accelLimits, jerkLimits);
                    RAVELOG_INFO_FORMAT("env=%d, PUTTICHAI: shortcut iter=%d/%d:\n%s", _envId%iter%numIters%ssdebug.str());
                }

                bool bSuccess = false;
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                ShortcutStatus currentStatus = SS_Successful;
#endif
                for( size_t iSlowDown = 0; iSlowDown < maxSlowDownTries; ++iSlowDown ) {
#ifdef JERK_LIMITED_SMOOTHER_TIMING_DEBUG
                    _StartCaptureInterpolator();
#endif
                    PiecewisePolynomials::PolynomialCheckReturn polycheckret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration
                                                                                   (x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect,
                                                                                   _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit,
                                                                                   velLimits, accelLimits, jerkLimits, t1 - t0 - minTimeStep, tempChunks);
#ifdef JERK_LIMITED_SMOOTHER_TIMING_DEBUG
                    _EndCaptureInterpolator();
#endif
                    dReal fChunksDuration = 0;
                    FOREACHC(itchunk, tempChunks) {
                        fChunksDuration += itchunk->duration;
                    }

                    if( polycheckret != PolynomialCheckReturn::PCR_Normal ) {
                        RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; iSlowDown=%d; initial interpolation failed. polycheckret=%s", _envId%iter%numIters%t0%t1%iSlowDown%PiecewisePolynomials::GetPolynomialCheckReturnString(polycheckret));
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                        if( polycheckret == PolynomialCheckReturn::PCR_DurationTooLong ) {
                            currentStatus = iSlowDown == 0 ? SS_InterpolatedSegmentTooLong : SS_InterpolatedSegmentTooLongFromSlowDown;
                        }
                        else {
                            currentStatus = SS_InitialInterpolationFailed;
                        }
#endif
                        break; // must not slow down any further.
                    }

                    if( fChunksDuration + minTimeStep > t1 - t0 ) {
                        // Segment does not make significant improvement.
                        RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; shortcut does not make significant improvement. prevduration=%.15e; newduration=%.15e; diff=%.15e; minTimeStep=%.15e", _envId%iter%numIters%t0%t1%(t1 - t0)%fChunksDuration%(t1 - t0 - fChunksDuration)%minTimeStep);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                        currentStatus = iSlowDown == 0 ? SS_InterpolatedSegmentTooLong : SS_InterpolatedSegmentTooLongFromSlowDown;
#endif
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
                        {
                            // TODO:
                            if( checkret.bDifferentVelocity ) {
                                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; rejecting shortcut since the checked segment ends with different velocities", _envId%iter%numIters%t0%t1);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                currentStatus = SS_CheckFailedWithDifferentVelocity;
#endif
                                break; // do not slow down any further.
                            }
                        }
                        bSuccess = true;
                        break; // break out of slowdown loop.
                    }
                    else if( checkret.retcode == CFO_CheckTimeBasedConstraints ) {
                        ++nTimeBasedConstraintsFailed;
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                        currentStatus = SS_CheckFailedTimeBasedConstraints;
#endif

                        bool bScaledDown = false;
                        if( _bManipConstraints && !!_manipConstraintChecker ) {
                            // Scale down accelLimits and/or velLimits based on what constraints are violated.
                            dReal fVelMult, fAccelMult, fJerkMult;
                            if( checkret.fMaxManipAccel > _parameters->maxmanipaccel ) {
                                if( SCALE_ALL_WHEN_TOOLACCEL_VIOLATED ) {
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                    RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; max manip accel violated (%.15e > %.15e). fTimeBasedSurpassMult=%.15e", _envId%iter%numIters%t0%t1%checkret.fMaxManipAccel%_parameters->maxmanipaccel%checkret.fTimeBasedSurpassMult);
#endif
                                }
                                else {
                                    bScaledDown = true;

                                    fAccelMult = checkret.fTimeBasedSurpassMult*checkret.fTimeBasedSurpassMult;
                                    fJerkMult = fAccelMult*checkret.fTimeBasedSurpassMult;
                                    fCurAccelMult *= fAccelMult;
                                    if( fCurAccelMult < fAccelMultCutoff ) {
                                        RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; fCurAccelMult goes below threshold (%.15e < %.15e).", _envId%iter%numIters%t0%t1%fCurAccelMult%fAccelMultCutoff);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                        currentStatus = SS_SlowDownFailed;
#endif
                                        break; // break out of slowdown loop
                                    }
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                    RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; max manip accel violated (%.15e > %.15e). fTimeBasedSurpassMult=%.15e; new fCurVelMult=%.15e; fCurAccelMult=%.15e", _envId%iter%numIters%t0%t1%checkret.fMaxManipAccel%_parameters->maxmanipaccel%checkret.fTimeBasedSurpassMult%fCurVelMult%fCurAccelMult);
#endif
                                    bool bAccelLimitsChanged = false;
                                    for( size_t idof = 0; idof < _ndof; ++idof ) {
                                        dReal fAccelLowerBound = vAccelLowerBound[idof];
                                        if( !PiecewisePolynomials::FuzzyEquals(accelLimits[idof], fAccelLowerBound, PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                                            if( CORRECT_VELACCELMULT ) {
                                                accelLimits[idof] = std::max(fAccelLowerBound, fAccelMult*accelLimits[idof]);
                                                jerkLimits[idof] = fJerkMult*jerkLimits[idof];
                                            }
                                            else {
                                                accelLimits[idof] = std::max(fAccelLowerBound, fCurAccelMult*accelLimits[idof]);
                                            }
                                            bAccelLimitsChanged = true;
                                        }
                                    }
                                    if( !bAccelLimitsChanged ) {
                                        RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; cannot scale down accel limits further", _envId%iter%numIters%t0%t1);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                        currentStatus = SS_SlowDownFailed;
#endif
                                        break; // break out of slowdown loop
                                    }
                                } // end if !SCALE_ALL_WHEN_TOOLACCEL_VIOLATED
                            }
                            else if( checkret.fMaxManipSpeed > _parameters->maxmanipspeed ) {
                                bScaledDown = true;

                                fVelMult = checkret.fTimeBasedSurpassMult;
                                fCurVelMult *= fVelMult;
                                if( fCurVelMult < fVelMultCutoff ) {
                                    RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; fCurVelMult goes below threshold (%.15e < %.15e).", _envId%iter%numIters%t0%t1%fCurVelMult%fVelMultCutoff);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                    currentStatus = SS_SlowDownFailed;
#endif
                                    break; // break out of slowdown loop
                                }
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; max manip speed violated (%.15e > %.15e). fTimeBasedSurpassMult=%.15e; new fCurVelMult=%.15e; fCurAccelMult=%.15e", _envId%iter%numIters%t0%t1%checkret.fMaxManipSpeed%_parameters->maxmanipspeed%checkret.fTimeBasedSurpassMult%fCurVelMult%fCurAccelMult);
#endif
                                bool bVelLimitsChanged = false;
                                for( size_t idof = 0; idof < _ndof; ++idof ) {
                                    dReal fVelLowerBound = vVelLowerBound[idof];
                                    if( !PiecewisePolynomials::FuzzyEquals(velLimits[idof], fVelLowerBound, PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                                        if( CORRECT_VELACCELMULT ) {
                                            velLimits[idof] = std::max(fVelLowerBound, fVelMult*velLimits[idof]);
                                        }
                                        else {
                                            velLimits[idof] = std::max(fVelLowerBound, fCurVelMult*velLimits[idof]);
                                        }
                                        bVelLimitsChanged = true;
                                    }
                                }
                                if( !bVelLimitsChanged ) {
                                    RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; cannot scale down vel limits further", _envId%iter%numIters%t0%t1);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                    currentStatus = SS_SlowDownFailed;
#endif
                                    break; // break out of slowdown loop
                                }
                            }
                        }
                        if( !bScaledDown ) {
                            // The segment does not fail due to manip speed/accel constraints. Scale down both velLimits
                            // and accelLimits
                            dReal fMult1 = checkret.fTimeBasedSurpassMult;
                            dReal fMult2 = fMult1 * checkret.fTimeBasedSurpassMult;
                            dReal fMult3 = fMult2 * checkret.fTimeBasedSurpassMult;
                            fCurVelMult *= fMult1;
                            fCurAccelMult *= fMult2;
                            fCurJerkMult *= fMult3;
                            if( fCurVelMult < fVelMultCutoff ) {
                                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; fCurVelMult goes below threshold (%.15e < %.15e).", _envId%iter%numIters%t0%t1%fCurVelMult%fVelMultCutoff);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                currentStatus = SS_SlowDownFailed;
#endif
                                break; // break out of slowdown loop
                            }
                            if( fCurAccelMult < fAccelMultCutoff ) {
                                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; fCurAccelMult goes below threshold (%.15e < %.15e).", _envId%iter%numIters%t0%t1%fCurAccelMult%fAccelMultCutoff);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                currentStatus = SS_SlowDownFailed;
#endif
                                break; // break out of slowdown loop
                            }

                            RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; new fCurVelMult=%.15e; fCurAccelMult=%.15e", _envId%iter%numIters%t0%t1%fCurVelMult%fCurAccelMult);

                            bool bLimitsChanged = false;
                            for( size_t idof = 0; idof < _ndof; ++idof ) {
                                dReal fVelLowerBound = vVelLowerBound[idof];
                                if( !PiecewisePolynomials::FuzzyEquals(velLimits[idof], fVelLowerBound, PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                                    if( CORRECT_VELACCELMULT ) {
                                        velLimits[idof] = std::max(fVelLowerBound, fMult1*velLimits[idof]);
                                    }
                                    else {
                                        velLimits[idof] = std::max(fVelLowerBound, fCurVelMult*velLimits[idof]);
                                    }
                                    bLimitsChanged = true;
                                }

                                dReal fAccelLowerBound = vAccelLowerBound[idof];
                                if( !PiecewisePolynomials::FuzzyEquals(accelLimits[idof], fAccelLowerBound, PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                                    if( CORRECT_VELACCELMULT ) {
                                        accelLimits[idof] = std::max(fAccelLowerBound, fMult2*accelLimits[idof]);
                                    }
                                    else {
                                        accelLimits[idof] = std::max(fAccelLowerBound, fCurAccelMult*accelLimits[idof]);
                                    }
                                    bLimitsChanged = true;
                                }
                                // Scaling down jerk limits likely leads to significantly slower final traj.
                                jerkLimits[idof] *= fMult3;
                            }
                            if( !bLimitsChanged ) {
                                RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; cannot scale down vel/accel limits further", _envId%iter%numIters%t0%t1);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                                currentStatus = SS_SlowDownFailed;
#endif
                                break; // break out of slowdown loop
                            }
                        }
                        ++numSlowDowns;
                    }
                    else {
                        RAVELOG_DEBUG_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; rejecting shortcut due to ret=0x%x", _envId%iter%numIters%t0%t1%checkret.retcode);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                        if( (checkret.retcode & CFO_CheckEnvCollisions) == CFO_CheckEnvCollisions  ) {
                            currentStatus = SS_CheckFailedCollisions;
                        }
                        else if( (checkret.retcode & CFO_CheckUserConstraints) == CFO_CheckUserConstraints ) {
                            currentStatus = SS_CheckFailedUserConstraints;
                        }
                        else {
                            currentStatus = SS_Failed;
                        }
#endif
                        break; // do not slow down any further.
                    }
                    iIterProgress += 0x1000;
                } // end slowdown iterations


                if( !bSuccess ) {
                    // Shortcut failed. Continue to the next iteration.
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                    ++_vShortcutStats[currentStatus];
#endif
                    continue;
                }

                if( vChunksOut.size() == 0 ) {
                    RAVELOG_WARN_FORMAT("env=%d, shortcut iter=%d/%d, t0=%.15e; t1=%.15e; shortcut chunks vector is empty", _envId%iter%numIters%t0%t1);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                    ++_vShortcutStats[SS_Failed];
#endif
                    continue;
                }

                dReal fSegmentTime = 0;
                FOREACHC(itChunk, vChunksOut) {
                    fSegmentTime += itChunk->duration;
                }
                dReal fDiff = (t1 - t0) - fSegmentTime;
                // Make sure that the new segment duration is really less than the original segment duration. (The new segment duration may have changed during the checking process due to tool constraints projection, etc.)
                if( fDiff < 0 ) {
                    RAVELOG_WARN_FORMAT("env=%d, the new segment time=%.15f becomes larger than the original segment time=%.15f, so discarding it.", _envId%fSegmentTime%(t1 - t0));
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                    ++_vShortcutStats[SS_CheckedSegmentTooLong];
#endif
                    continue;
                }

                // Now this shortcut iteration is really successful.
                ++numShortcuts;
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
                ++_vShortcutStats[SS_Successful];
#endif
                lastSuccessfulShortcutIter = iter;

                if( !REMOVE_STARTTIMEMULT ) {
                    // Keep track of multipliers
                    fStartTimeVelMult = min(1.0, fCurVelMult*fiSearchVelAccelMult);
                    fStartTimeAccelMult = min(1.0, fCurAccelMult*fiSearchVelAccelMult);
                }

                // Update parameters
                nTimeBasedConstraintsFailed = 0;
                vVisitedDiscretization.clear();

                // Replace the original portion with the shortcut segment.
                if( iter == 0 ) {
                    // Since replacing the entire initial trajectory, just initialize pwptraj with vChunksOut directly.
                    pwptraj.Initialize(vChunksOut);
#ifdef JERK_LIMITED_SMOOTHER_VALIDATE
                    FOREACH(itchunk, pwptraj.vchunks) {
                        itchunk->_iteration = iter;
                    }
#endif
                }
                else {
#ifdef JERK_LIMITED_SMOOTHER_VALIDATE
                    FOREACH(itchunk, vChunksOut) {
                        itchunk->_iteration = iter;
                    }
#endif
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
#ifdef JERK_LIMITED_SMOOTHER_VALIDATE
                throw;
#else
                break;
#endif
            }
        } // end shortcut iterations

        // Report statistics
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        if( fScore/fCurrentBestScore < fCutoffRatio ) {
            ss << "current score falls below threshold (" << fScore/fCurrentBestScore << " < " << fCutoffRatio << ")";
        }
        else if( nItersFromPrevSuccessful + nTimeBasedConstraintsFailed > nCutoffIters ) {
            ss << "did not make progress in the past " << nItersFromPrevSuccessful << " iterations and time-based constraints failed " << nTimeBasedConstraintsFailed << " times";
        }
        else {
            ss << "normal exit";
        }
        RAVELOG_DEBUG_FORMAT("env=%d, Finished at shortcut iter=%d/%d (%s), successful=%d; numSlowDowns=%d; duration: %.15e -> %.15e; diff=%.15e", _envId%iter%numIters%ss.str()%numShortcuts%numSlowDowns%tOriginal%tTotal%(tOriginal - tTotal));
        _DumpPiecewisePolynomialTrajectory(pwptraj, "aftershortcut", _dumpLevel);
#ifdef JERK_LIMITED_SMOOTHER_PROGRESS_DEBUG
        ss.str(""); ss.clear();
        GetShortcutStatusString(ss);
        RAVELOG_INFO_FORMAT("env=%d, Shortcut statistics: total iterations=%d, last successful iteration=%d\n%s", _envId%iter%lastSuccessfulShortcutIter%ss.str());
#endif
#ifdef JERK_LIMITED_SMOOTHER_TIMING_DEBUG
        ss.str(""); ss.clear();
        _GetShortcutSubprocessesTiming(ss);
        RAVELOG_INFO_FORMAT("env=%d, Shortcut subprocesses timing:\n%s", _envId%ss.str());
#endif
        return numShortcuts;
    }

    /// \brief Verify that the input sequence of chunks satisfy all constraints (including collisions, manip speed/accel, and possibly dynamics).
    virtual PiecewisePolynomials::CheckReturn CheckAllChunksAllConstraints(const std::vector<PiecewisePolynomials::Chunk>& vChunksIn, int options, std::vector<PiecewisePolynomials::Chunk>& vChunksOut) override
    {
        // Make sure the first configuration is safe
        std::vector<dReal> &x0Vect = _cacheX0Vect2, &v0Vect = _cacheV0Vect2, &a0Vect = _cacheA0Vect2;
        const PiecewisePolynomials::Chunk& chunk = vChunksIn.front();
        chunk.Eval(0, x0Vect);
        chunk.Evald1(0, v0Vect);
        chunk.Evald2(0, a0Vect);
        PiecewisePolynomials::CheckReturn checkret = CheckConfigAllConstraints(x0Vect, v0Vect, a0Vect, options);
        if( checkret.retcode != 0 ) {
            return checkret;
        }

        vChunksOut.resize(0);

        std::vector<PiecewisePolynomials::Chunk>& vIntermediateChunks = _vIntermediateChunks; // will be resized in CheckChunkAllConstraints
        for( std::vector<PiecewisePolynomials::Chunk>::const_iterator itchunk = vChunksIn.begin(); itchunk != vChunksIn.end(); ++itchunk ) {
            checkret = CheckChunkAllConstraints(*itchunk, options, vIntermediateChunks);
            if( checkret.retcode != 0 ) {
                return checkret;
            }

            vChunksOut.insert(vChunksOut.end(), vIntermediateChunks.begin(), vIntermediateChunks.end());
        }

#ifdef JERK_LIMITED_SMOOTHER_VALIDATE
        // Check that the boundary conditions of vChunksOut are aligned with those of the input
        PiecewisePolynomials::PiecewisePolynomialTrajectory temptraj;
        // Check the beginning
        {
            const PiecewisePolynomials::Chunk& inputInitChunk = vChunksIn.front();
            inputInitChunk.Eval(0, x0Vect);
            inputInitChunk.Evald1(0, v0Vect);
            inputInitChunk.Evald2(0, a0Vect);
            const PiecewisePolynomials::Chunk& outputInitChunk = vChunksOut.front();
            PiecewisePolynomials::PolynomialCheckReturn chunkcheckret = _limitsChecker.CheckChunkValues(outputInitChunk, 0, x0Vect, v0Vect, a0Vect);
            if( chunkcheckret != PolynomialCheckReturn::PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                RAVELOG_WARN_FORMAT("env=%d, idof=%d; value=%.15e; expected=%.15e; result=%s", _envId%_limitsChecker._failedDOF%_limitsChecker._failedValue%_limitsChecker._expectedValue%PiecewisePolynomials::GetPolynomialCheckReturnString(chunkcheckret));
#endif
                temptraj.Initialize(vChunksIn);
                _DumpPiecewisePolynomialTrajectory(temptraj, "inputchunks", static_cast<DebugLevel>(RaveGetDebugLevel()));
                temptraj.Initialize(vChunksOut);
                _DumpPiecewisePolynomialTrajectory(temptraj, "outputchunks", static_cast<DebugLevel>(RaveGetDebugLevel()));
                OPENRAVE_ASSERT_FORMAT(false, "got checkret=%s while checking the start of chunks", PiecewisePolynomials::GetPolynomialCheckReturnString(chunkcheckret), ORE_InconsistentConstraints);
            }
        } // end checking for the beginning

        // Check the end
        {
            std::vector<dReal> &x1Vect = _cacheX1Vect2, &v1Vect = _cacheV1Vect2, &a1Vect = _cacheA1Vect2;
            const PiecewisePolynomials::Chunk& inputFinalChunk = vChunksIn.back();
            inputFinalChunk.Eval(inputFinalChunk.duration, x1Vect);
            inputFinalChunk.Evald1(inputFinalChunk.duration, v1Vect);
            inputFinalChunk.Evald2(inputFinalChunk.duration, a1Vect);
            const PiecewisePolynomials::Chunk& outputFinalChunk = vChunksOut.back();
            PiecewisePolynomials::PolynomialCheckReturn chunkcheckret = _limitsChecker.CheckChunkValues(outputFinalChunk, outputFinalChunk.duration, x1Vect, v1Vect, a1Vect);
            if( chunkcheckret != PolynomialCheckReturn::PCR_Normal ) {
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                RAVELOG_WARN_FORMAT("env=%d, idof=%d; value=%.15e; expected=%.15e; result=%s", _envId%_limitsChecker._failedDOF%_limitsChecker._failedValue%_limitsChecker._expectedValue%PiecewisePolynomials::GetPolynomialCheckReturnString(chunkcheckret));
#endif
                temptraj.Initialize(vChunksIn);
                _DumpPiecewisePolynomialTrajectory(temptraj, "inputchunks", static_cast<DebugLevel>(RaveGetDebugLevel()));
                temptraj.Initialize(vChunksOut);
                _DumpPiecewisePolynomialTrajectory(temptraj, "outputchunks", static_cast<DebugLevel>(RaveGetDebugLevel()));
                OPENRAVE_ASSERT_FORMAT(false, "got checkret=%s while checking the end of chunks", PiecewisePolynomials::GetPolynomialCheckReturnString(chunkcheckret), ORE_InconsistentConstraints);
            }
        } // end checking for the end
#endif

        return PiecewisePolynomials::CheckReturn(0);
    }

protected:

    virtual void _InitializeInterpolator() override
    {
        _pinterpolator.reset(new PiecewisePolynomials::CubicInterpolator(_ndof, _envId));
        _maskinterpolation = IT_Cubic;
    }

    virtual PiecewisePolynomials::CheckReturn _ProcessConstraintReturnIntoChunks(ConstraintFilterReturnPtr contraintReturn, const PiecewisePolynomials::Chunk chunkIn,
                                                                                 const bool bMarkConstraintChecked,
                                                                                 std::vector<dReal>& x0Vect, std::vector<dReal>& x1Vect,
                                                                                 std::vector<dReal>& v0Vect, std::vector<dReal>& v1Vect,
                                                                                 std::vector<dReal>& a0Vect, std::vector<dReal>& a1Vect,
                                                                                 std::vector<PiecewisePolynomials::Chunk>& vChunksOut) override
    {
        dReal curTime = 0;
        if( vChunksOut.capacity() < _constraintReturn->_configurationtimes.size() ) {
            vChunksOut.reserve(_constraintReturn->_configurationtimes.size());
        }

        PiecewisePolynomials::Polynomial tempPolynomial; // for use in the following loop

        std::vector<dReal>::const_iterator itConfigLowerLimit = _parameters->_vConfigLowerLimit.begin();
        std::vector<dReal>::const_iterator itConfigUpperLimit = _parameters->_vConfigUpperLimit.begin();
        std::vector<dReal>::const_iterator itVelocityLimit = _parameters->_vConfigVelocityLimit.begin();
        std::vector<dReal>::const_iterator itAccelerationLimit = _parameters->_vConfigAccelerationLimit.begin();
        std::vector<dReal>::const_iterator itJerkLimit = _parameters->_vConfigJerkLimit.begin();

        // x0Vect, v0Vect, a0Vect already carry the initial values of chunkIn
        std::vector<dReal>::const_iterator it = _constraintReturn->_configurations.begin();
        for( size_t itime = 0; itime < _constraintReturn->_configurationtimes.size(); ++itime, it += _ndof ) {
            std::copy(it, it + _ndof, x1Vect.begin());
            const dReal deltaTime = _constraintReturn->_configurationtimes[itime] - curTime;
            const dReal ideltaTime = 1.0/deltaTime;
            if( deltaTime > PiecewisePolynomials::g_fPolynomialEpsilon ) {

                std::vector<PiecewisePolynomials::Polynomial> vpolynomials(_ndof); // TODO: cache this

                for( size_t idof = 0; idof < _ndof; ++idof ) {
                    /*
                       p(t) = a*t^3 + b*t^2 + c*t + d
                       with a given duration (deltaTime), and boundary conditions:
                       p(0) = x0,
                       p'(0) = v0,
                       p''(0) = a0, and
                       p(deltaTime) = x1.

                       Coefficients can be computed as
                       d = x0,
                       c = v0,
                       b = a0/2, and
                       a = (x1 - x0)/deltaTime*^3 - c/deltaTime^2 - b/deltaTime
                         = (((x1 - x0)/deltaTime - c)/deltaTime - b)/deltaTime
                     */
                    const dReal d = x0Vect[idof];
                    const dReal c = v0Vect[idof];
                    const dReal b = 0.5*a0Vect[idof];
                    const dReal a = (((x1Vect[idof] - x0Vect[idof])*ideltaTime - c)*ideltaTime - b)*ideltaTime;
                    tempPolynomial.Initialize(deltaTime, std::vector<dReal>({d, c, b, a}));
                    PolynomialCheckReturn limitsret = _limitsChecker.CheckPolynomialLimits(tempPolynomial, *(itConfigLowerLimit + idof), *(itConfigUpperLimit + idof), *(itVelocityLimit + idof), *(itAccelerationLimit + idof), *(itJerkLimit + idof));
                    if( limitsret != PiecewisePolynomials::PCR_Normal ) {
                        RAVELOG_VERBOSE_FORMAT("env=%d, the output chunk is invalid: idof=%d; itime=%d/%d; t=%f/%f; ret=%s", _envId%idof%itime%_constraintReturn->_configurationtimes.size()%curTime%chunkIn.duration%PiecewisePolynomials::GetPolynomialCheckReturnString(limitsret));
#ifdef JERK_LIMITED_POLY_CHECKER_DEBUG
                        RAVELOG_VERBOSE_FORMAT("env=%d, failedPoint=%.15e; failedValue=%.15e; expectedValue=%.15e", _envId%_limitsChecker._failedPoint%_limitsChecker._failedValue%_limitsChecker._expectedValue);
#endif
                        return PiecewisePolynomials::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9);
                    }

                    vpolynomials[idof] = tempPolynomial;
                } // end for idof

                PiecewisePolynomials::Chunk tempChunk(deltaTime, vpolynomials);
                vChunksOut.push_back(tempChunk);
                vChunksOut.back().constraintChecked = bMarkConstraintChecked;

                curTime = _constraintReturn->_configurationtimes[itime];
                vChunksOut.back().Eval(deltaTime, x0Vect);
                vChunksOut.back().Evald1(deltaTime, v0Vect);
                vChunksOut.back().Evald2(deltaTime, a0Vect);
            } // end if deltaTime > epsilon
        } // end for itime

        // Make sure that the last configuration is the desired value
        chunkIn.Eval(chunkIn.duration, x1Vect);
        for( size_t idof = 0; idof < _ndof; ++idof ) {
            if( RaveFabs(x0Vect[idof] - x1Vect[idof]) > PiecewisePolynomials::g_fPolynomialEpsilon ) {
                RAVELOG_WARN_FORMAT("env=%d, Detected discrepancy at the last configuration: idof=%d; (%.15e != %.15e)", _envId%idof%x0Vect[idof]%x1Vect[idof]);
                return PiecewisePolynomials::CheckReturn(CFO_FinalValuesNotReached);
            }
        }

        // Check the final velocities
        bool bDifferentVelocity = false;
        chunkIn.Evald1(chunkIn.duration, v1Vect);
        for( size_t idof = 0; idof < _ndof; ++idof ) {
            if( RaveFabs(v0Vect[idof] - v1Vect[idof]) > PiecewisePolynomials::g_fPolynomialEpsilon ) {
                RAVELOG_DEBUG_FORMAT("env=%d, idof=%d does not finish at the desired velocity. diff=%.15e", _envId%idof%(v0Vect[idof] - v1Vect[idof]));
                bDifferentVelocity = true;
                break;
            }
        }

        PiecewisePolynomials::CheckReturn finalret(0);
        finalret.bDifferentVelocity = bDifferentVelocity;
        return finalret;
    } // end ProcessConstraintReturnIntoChunks

    PlannerStatus ConvertOpenRAVETrajectoryToPiecewisePolynomialTrajectorySameInterpolation(TrajectoryBasePtr ptraj,
                                                                                            ConfigurationSpecification& posSpec, ConfigurationSpecification& velSpec,
                                                                                            ConfigurationSpecification& accelSpec, ConfigurationSpecification& timeSpec,
                                                                                            PiecewisePolynomials::PiecewisePolynomialTrajectory& pwptraj) override
    {
        // Cache stuff
        std::vector<dReal> &x0Vect = _cacheX0Vect, &x1Vect = _cacheX1Vect, &v0Vect = _cacheV0Vect, &v1Vect = _cacheV1Vect, &a0Vect = _cacheA0Vect, &a1Vect = _cacheA1Vect, &tVect = _cacheTVect;
        std::vector<PiecewisePolynomials::Chunk>& vChunks = _cacheCheckedChunks;

        std::vector<PiecewisePolynomials::Polynomial> vTempPolynomials(_ndof);
        std::vector<dReal> vCubicCoeffs(4, 0);

        std::vector<dReal>::const_iterator itVelLimit = _parameters->_vConfigVelocityLimit.begin();
        std::vector<dReal>::const_iterator itAccelLimit = _parameters->_vConfigAccelerationLimit.begin();

        // Convert the OpenRAVE trajectory to a PiecewisePolynomialTrajectory
        vChunks.resize(0);
        if( vChunks.capacity() < ptraj->GetNumWaypoints() - 1 ) {
            vChunks.reserve(ptraj->GetNumWaypoints() - 1);
        }
        ptraj->GetWaypoint(0, x0Vect, posSpec);
        ptraj->GetWaypoint(0, v0Vect, velSpec);
        ptraj->GetWaypoint(0, a0Vect, accelSpec);
        // Clamp velocities and accelerations to the actual limits
        for( size_t idof = 0; idof < _ndof; ++idof ) {
            if( !PiecewisePolynomials::ClampValueToLimit(v0Vect[idof], *(itVelLimit + idof), PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                throw OPENRAVE_EXCEPTION_FORMAT("env=%d, iWaypoint=%d/%d; v=%.15e violates vm=%.15e", _envId%0%ptraj->GetNumWaypoints()%v0Vect[idof]%(*(itVelLimit + idof)), ORE_InconsistentConstraints);
            }
            if( !PiecewisePolynomials::ClampValueToLimit(a0Vect[idof], *(itAccelLimit + idof), PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                throw OPENRAVE_EXCEPTION_FORMAT("env=%d, iWaypoint=%d/%d; a=%.15e violates am=%.15e", _envId%0%ptraj->GetNumWaypoints()%a0Vect[idof]%(*(itAccelLimit + idof)), ORE_InconsistentConstraints);
            }
        }
        // For each segment connecting two consecutive waypoints, compute polynomial coefficients directly from the
        // given boundary conditions instead of using an interpolation function. Slightly different boundary conditions
        // might lead to a vastly different interpolated trajectory.
        for( size_t iWaypoint = 1; iWaypoint < ptraj->GetNumWaypoints(); ++iWaypoint ) {
            ptraj->GetWaypoint(iWaypoint, tVect, timeSpec);
            const dReal duration = tVect.at(0);
            if( duration > g_fEpsilonLinear ) {
                ptraj->GetWaypoint(iWaypoint, x1Vect, posSpec);
                ptraj->GetWaypoint(iWaypoint, v1Vect, velSpec);
                ptraj->GetWaypoint(iWaypoint, a1Vect, accelSpec);
                for( size_t idof = 0; idof < _ndof; ++idof ) {
                    if( !PiecewisePolynomials::ClampValueToLimit(v1Vect[idof], *(itVelLimit + idof), PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                        throw OPENRAVE_EXCEPTION_FORMAT("env=%d, iWaypoint=%d/%d; v=%.15e violates vm=%.15e", _envId%iWaypoint%ptraj->GetNumWaypoints()%v1Vect[idof]%(*(itVelLimit + idof)), ORE_InconsistentConstraints);
                    }
                    if( !PiecewisePolynomials::ClampValueToLimit(a1Vect[idof], *(itAccelLimit + idof), PiecewisePolynomials::g_fPolynomialEpsilon) ) {
                        throw OPENRAVE_EXCEPTION_FORMAT("env=%d, iWaypoint=%d/%d; a=%.15e violates am=%.15e", _envId%iWaypoint%ptraj->GetNumWaypoints()%a1Vect[idof]%(*(itAccelLimit + idof)), ORE_InconsistentConstraints);
                    }
                    mathextra::computecubiccoeffs(x0Vect[idof], x1Vect[idof], v0Vect[idof], v1Vect[idof], a0Vect[idof], a1Vect[idof], duration, &vCubicCoeffs[0]);
                    std::reverse(vCubicCoeffs.begin(), vCubicCoeffs.end()); // PiecewisePolynomials' polynomial coefficient vector has the weakest term first so need to reverse the vector.
                    vTempPolynomials[idof].Initialize(duration, vCubicCoeffs);
                }
                vChunks.emplace_back(duration, vTempPolynomials);
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
    } // end ConvertOpenRAVETrajectoryToPiecewisePolynomialTrajectorySameInterpolation

private:

    std::vector<PiecewisePolynomials::Chunk> _cacheFinalChunks; // for storing chunks before putting them into the final trajcetory
    PiecewisePolynomials::Chunk _cacheTrimmedChunk, _cacheRemChunk; ///< for constraints checking at the very end

    // For use during CheckX process
    std::vector<PiecewisePolynomials::Chunk> _vIntermediateChunks;

    const bool REMOVE_STARTTIMEMULT=true; // do not keep track of fStartTimeVelMult and fStartTimeAccelMult of successful shortcut iterations
    const bool CORRECT_VELACCELMULT=true; // correct the formula for computing new scaled-down vel/accel limits
    const bool SCALE_ALL_WHEN_TOOLACCEL_VIOLATED=true; // scale vel/accel/jerk limits down when max tool accel is violated
    const bool USE_ESTIMATED_VELMULT=true; // use the average fTimeBasedSurpassMult from initial timing computation as the starting multiplier.

}; // end class CubicSmoother

PlannerBasePtr CreateCubicSmoother(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new CubicSmoother(penv, sinput));
}

} // end namespace rplanners
