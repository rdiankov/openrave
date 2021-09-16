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

    virtual void _InitializeInterpolator() override
    {
        _pinterpolator.reset(new PiecewisePolynomials::QuinticInterpolator(_ndof, _envId));
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

        bool bPathIsPerfectlyModeled = false; // will be true if the initial interpolation is linear or quintic
        RAVELOG_VERBOSE_FORMAT("env=%d, Initial trajectory joint values interpolation is %s", _envId%itcompatposgroup->interpolation);
        PlannerStatus conversionStatus = PS_Failed;
        if( _parameters->_hastimestamps && itcompatposgroup->interpolation == "quintic" ) {
            bPathIsPerfectlyModeled = true;

            conversionStatus = ConvertOpenRAVETrajectoryToPiecewisePolynomialTrajectorySameInterpolation(ptraj, posSpec, velSpec, accelSpec, timeSpec, pwptraj);
        }
        // TODO: Maybe we need to handle other cases of interpolation as well
        else {
            if( itcompatposgroup->interpolation.size() == 0 || itcompatposgroup->interpolation == "linear" ) {
                bPathIsPerfectlyModeled = true;
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
            std::vector<dReal> &x0Vect = _cacheX0Vect, &x1Vect = _cacheX1Vect, &v0Vect = _cacheV0Vect, &v1Vect = _cacheV1Vect, &a0Vect = _cacheA0Vect, &a1Vect = _cacheA1Vect, &tVect = _cacheTVect;
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
            std::vector<PiecewisePolynomials::Chunk>& tempChunks = _cacheTempChunks;
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
                    vChunksOut.resize(1);
                    vChunksOut[0] = trimmedChunk;
                }
                else {
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
                        PiecewisePolynomials::CheckReturn checkret = CheckChunkAllConstraints(trimmedChunk, 0xffff, tempChunks);
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
                                _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, newChunkDuration, tempChunk);

                                // TODO
                                PiecewisePolynomials::CheckReturn newcheckret = CheckChunkAllConstraints(tempChunk, 0xffff, tempChunks);
                                if( newcheckret.retcode == 0 ) {
                                    // The new chunk passes constraints checking.
                                    bDilationSuccessful = true;
                                    break;
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

                    vChunksOut.resize(0);
                    if( bTrimmedFront ) {
                        vChunksOut.push_back(remChunk);
                    }
                    vChunksOut.insert(vChunksOut.end(), tempChunks.begin(), tempChunks.end());
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
            _DumpOpenRAVETrajectory(ptraj, "failedexception", _errorDumpLevel);
            RAVELOG_WARN_FORMAT("env=%d, Main planning loop threw an expection: %s", _envId%ex.what());
            return PS_Failed;
        }
        RAVELOG_DEBUG_FORMAT("env=%d, path optimizing - computation time=%f", _envId%(0.001f*(dReal)(utils::GetMilliTime() - startTime)));

        // Save the final trajectory
        _DumpOpenRAVETrajectory(ptraj, "final", _dumpLevel);

        return _ProcessPostPlanners(RobotBasePtr(), ptraj);
    }

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
