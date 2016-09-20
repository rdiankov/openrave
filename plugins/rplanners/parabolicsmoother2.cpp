// -*- coding: utf-8 -*-
// Copyright (C) 2016 Puttichai Lertkultanon & Rosen DianKov
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

#include "rampoptimizer/interpolator.h"
#include "rampoptimizer/parabolicchecker.h"
#include "rampoptimizer/feasibilitychecker.h"
#include "manipconstraints2.h"

namespace rplanners {

namespace RampOptimizer = RampOptimizerInternal;

class ParabolicSmoother2 : public PlannerBase, public RampOptimizer::FeasibilityCheckerBase, public RampOptimizer::RandomNumberGeneratorBase {

    class MyRampNDFeasibilityChecker : public RampOptimizer::RampNDFeasibilityChecker {
public:
        MyRampNDFeasibilityChecker(RampOptimizer::FeasibilityCheckerBase* feas) : RampOptimizer::RampNDFeasibilityChecker(feas) {
            _cacheRampNDVectIn.resize(1);
        }

        /// \brief A wrapper function for Check2.
        RampOptimizer::CheckReturn Check2(const RampOptimizer::RampND& rampndIn, int options, std::vector<RampOptimizer::RampND>& rampndVectOut)
        {
            _cacheRampNDVectIn[0] = rampndIn;
            return Check2(_cacheRampNDVectIn, options, rampndVectOut);
        }

        /// \brief Check all constraints on all rampnds in the given vector of rampnds. options is passed to the OpenRAVE check function.
        RampOptimizer::CheckReturn Check2(const std::vector<RampOptimizer::RampND>& rampndVect, int options, std::vector<RampOptimizer::RampND>& rampndVectOut)
        {
            // If all necessary constraints are checked (specified by options), then we set constraintChecked to true.
            if( (options & constraintmask) == constraintmask ) {
                FOREACH(itrampnd, rampndVect) {
                    itrampnd->constraintChecked = true;
                }
            }
            OPENRAVE_ASSERT_OP(tol.size(), ==, rampndVect[0].GetDOF());
            for (size_t idof = 0; idof < tol.size(); ++idof) {
                OPENRAVE_ASSERT_OP(tol[idof], >, 0);
            }

            // Extract all switch points (including t = 0 and t = duration).
            _vswitchtimes.resize(0);
            _vswitchtimes.reserve(rampndVect.size() + 1);
            dReal switchtime = 0;
            _vswitchtimes.push_back(switchtime);
            FOREACH(itrampnd, rampndVect) {
                switchtime += itrampnd->GetDuration();
                _vswitchtimes.push_back(switchtime);
            }

            // Check boundary configurations
            rampndVect[0].GetX0Vect(_q0);
            rampndVect[0].GetV0Vect(_dq0);
            RampOptimizer::CheckReturn ret0 = feas->ConfigFeasible2(_q0, _dq0, options);
            if( ret0.retcode != 0 ) {
                return ret0;
            }

            rampndVect.back().GetX1Vect(_q1);
            rampndVect.back().GetV1Vect(_dq1);
            RampOptimizer::CheckReturn ret1 = feas->ConfigFeasible2(_q1, _dq1, options);
            if( ret1.retcode != 0 ) {
                return ret1;
            }

            // Check configurations at all switch time
            _vsearchsegments.resize(_vswitchtimes.size(), 0);
            for (size_t i = 0; i < _vsearchsegments.size(); ++i) {
                _vsearchsegments[i] = i;
            }
            size_t midIndex = _vsearchsegments.size()/2;
            std::swap(_vsearchsegments[0], _vsearchsegments[midIndex]); // put the mid point as the first point to be considered

            if( _vswitchtimes.size() > 2 ) {
                // Check at every switch time except at midIndex
                for (size_t iswitch = 0; (iswitch != midIndex) && (iswitch < _vswitchtimes.size() - 2); ++iswitch) {
                    switchtime = _vswitchtimes[_vsearchsegments[iswitch]];
                    rampndVect[_vsearchsegments[iswitch]].GetX0Vect(_q0);
                    if( feas->NeedDerivativeForFeasibility() ) {
                        rampndVect[_vsearchsegments[iswitch]].GetV0Vect(_dq0);
                    }
                    RampOptimizer::CheckReturn retconf = feas->ConfigFeasible2(_q0, _dq0, options);
                    if( retconf.retcode != 0 ) {
                        return retconf;
                    }
                }
            }
            else {
                // The initial and final configurations have already been checked. No need to do it again.
            }

            rampndVectOut.resize(0);

            // Now check each RampND
            rampndVect[0].GetX0Vect(_q0);
            rampndVect[0].GetV0Vect(_dq0);
            _q1.resize(_q0.size());
            _dq1.resize(_dq0.size());
            dReal elapsedTime, expectedElapsedTime, newElapsedTime, iElapsedTime, totalWeight;
            for (size_t iswitch = 1; iswitch < _vswitchtimes.size(); ++iswitch) {
                rampndVect[iswitch - 1].GetX1Vect(_q1); // configuration at _vswitchtimes[iswitch]
                elapsedTime = _vswitchtimes[iswitch] - _vswitchtimes[iswitch - 1]; // current elapsed time of this ramp

                if( feas->NeedDerivativeForFeasibility() ) {
                    rampndVect[iswitch - 1].GetV1Vect(_dq1);
                    // Due to constraints, configurations along the segment may have been modified
                    // (via CheckPathAllConstraints called from SegmentFeasible2). This may cause
                    // _dq1 not being consistent with _q0, _q1, _dq0, and elapsedTime. So we check
                    // consistency here as well as modify _dq1 and elapsedTime if necessary.

                    expectedElapsedTime = 0;
                    totalWeight = 0;
                    for (size_t idof = 0; idof < _q0.size(); ++idof) {
                        dReal avgVel = 0.5*(_dq0[idof] + _dq1[idof]);
                        if( RaveFabs(avgVel) > g_fEpsilon ) {
                            dReal fWeight = RaveFabs(_q1[idof] - _q0[idof]);
                            expectedElapsedTime += fWeight*(_q1[idof] - _q0[idof])/avgVel;
                            totalWeight += fWeight;
                        }
                    }

                    if( totalWeight > g_fEpsilon ) {
                        // Recompute elapsed time
                        newElapsedTime = expectedElapsedTime/totalWeight;

                        // Check elapsed time consistency
                        if( RaveFabs(newElapsedTime) > RampOptimizer::g_fRampEpsilon ) {
                            elapsedTime = newElapsedTime;
                            if( elapsedTime > g_fEpsilon ) {
                                iElapsedTime = 1/elapsedTime;
                                for (size_t idof = 0; idof < _q0.size(); ++idof) {
                                    _dq1[idof] = 2*iElapsedTime*(_q1[idof] - _q0[idof]) - _dq0[idof];
                                }
                            }
                            else {
                                _dq1 = _dq0;
                            }
                        }
                    }
                }

                RampOptimizer::CheckReturn retseg = feas->SegmentFeasible2(_q0, _q1, _dq0, _dq1, elapsedTime, options, _cacheRampNDVectOut);
                if( retseg.retcode != 0 ) {
                    return retseg;
                }

                if( _cacheRampNDVectOut.size() > 0 ) {
                    if( IS_DEBUGLEVEL(Level_Verbose) ) {
                        for (size_t idof = 0; idof < _q0.size(); ++idof) {
                            if( RaveFabs(_q1[idof] - _cacheRampNDVectOut.back().GetX1At(idof)) > RampOptimizer::g_fRampEpsilon ) {
                                RAVELOG_VERBOSE_FORMAT("rampndVect[%d] idof = %d: end point does not finish at the desired position, diff = %.15e", (iswitch - 1)%idof%RaveFabs(_q1[idof] - _cacheRampNDVectOut.back().GetX1At(idof)));
                            }
                            if( RaveFabs(_dq1[idof] - _cacheRampNDVectOut.back().GetV1At(idof)) > RampOptimizer::g_fRampEpsilon ) {
                                RAVELOG_VERBOSE_FORMAT("rampndVect[%d] idof = %d: end point does not finish at the desired velocity, diff = %.15e", (iswitch - 1)%idof%RaveFabs(_dq1[idof] - _cacheRampNDVectOut.back().GetV1At(idof)));
                            }
                        }
                    }
                    rampndVectOut.insert(rampndVectOut.end(), _cacheRampNDVectOut.begin(), _cacheRampNDVectOut.end());
                    rampndVectOut.back().GetX1Vect(_q0);
                    rampndVectOut.back().GetV1Vect(_dq0);
                }
            }

            // Note that now _q0 and _dq0 are actually the final joint position and velocity
            bool bDifferentVelocity = false;
            for (size_t idof = 0; idof < _q0.size(); ++idof) {
                if( RaveFabs(rampndVect.back().GetX1At(idof) - _q0[idof]) > RampOptimizer::g_fRampEpsilon ) {
                    RAVELOG_VERBOSE_FORMAT("rampndVectOut idof = %d: end point does not finish at the desired position, diff = %.15e. Rejecting...", idof%RaveFabs(rampndVect.back().GetX1At(idof) - _q0[idof]));
                    return RampOptimizer::CheckReturn(CFO_FinalValuesNotReached);
                }
                if( RaveFabs(rampndVect.back().GetV1At(idof) - _dq0[idof]) > RampOptimizer::g_fRampEpsilon ) {
                    RAVELOG_VERBOSE_FORMAT("rampndVectOut idof = %d: end point does not finish at the desired velocity, diff = %.15e", idof%RaveFabs(rampndVect.back().GetV1At(idof) - _dq0[idof]));
                    bDifferentVelocity = true;
                }
            }
            RampOptimizer::CheckReturn finalret(0);
            finalret.bDifferentVelocity = bDifferentVelocity;
            return finalret;
        }

private:
        // Cache
        std::vector<dReal> _vswitchtimes;
        std::vector<dReal> _q0, _q1, _dq0, _dq1;
        std::vector<uint8_t> _vsearchsegments;
        std::vector<RampOptimizer::RampND> _cacheRampNDVectIn, _cacheRampNDVectOut;        

    }; // end class MyRampNDFeasibilityChecker

public:
    ParabolicSmoother2(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv), _feasibilitychecker(this)
    {
        __description = "";
        _bmanipconstraints = false;
        _constraintreturn.reset(new ConstraintFilterReturn());
        _logginguniformsampler = RaveCreateSpaceSampler(GetEnv(), "mt19937");
        if (!!_logginguniformsampler) {
            _logginguniformsampler->SetSeed(utils::GetMicroTime());
        }
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        _parameters->copy(params);
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        isParameters >> *_parameters;
        return _InitPlan();
    }

    bool _InitPlan()
    {
        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 100;
        }

        _bUsePerturbation = true;
        _bmanipconstraints = (_parameters->manipname.size() > 0) && (_parameters->maxmanipspeed > 0 || _parameters->maxmanipaccel > 0);

        _interpolator.Initialize(_parameters->GetDOF());

        // Initialize workspace constraints on manipulators
        if( _bmanipconstraints ) {
            if( !_manipconstraintchecker ) {
                _manipconstraintchecker.reset(new ManipConstraintChecker2(GetEnv()));
            }
            _manipconstraintchecker->Init(_parameters->manipname, _parameters->_configurationspecification, _parameters->maxmanipspeed, _parameters->maxmanipaccel);
        }

        // Initialize a uniform sampler
        if( !_uniformsampler ) {
            _uniformsampler = RaveCreateSpaceSampler(GetEnv(), "mt19937");
        }
        _uniformsampler->SetSeed(_parameters->_nRandomGeneratorSeed);

        _fileIndexMod = 10000; // for trajectory saving
        _dumplevel = Level_Debug;
#ifdef SMOOTHER_TIMING_DEBUG
        // Statistics
        _nCallsCheckManip = 0;
        _totalTimeCheckManip = 0;
        _nCallsInterpolator = 0;
        _totalTimeInterpolator = 0;
#endif
        return !!_uniformsampler;
    }

    virtual PlannerParametersConstPtr GetParameters() const
    {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        BOOST_ASSERT(!!_parameters && !!ptraj);

        if( ptraj->GetNumWaypoints() < 2 ) {
            return PS_Failed;
        }

        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            // Save parameters for planning
            uint32_t randNum;
            if( !!_logginguniformsampler ) {
                randNum = _logginguniformsampler->SampleSequenceOneUInt32();
            }
            else {
                randNum = RaveRandomInt();
            }
            std::string filename = str(boost::format("%s/parabolicsmoother2_%d.parameters.xml")%RaveGetHomeDirectory()%(randNum%1000));
            ofstream f(filename.c_str());
            f << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
            f << *_parameters;
            RAVELOG_VERBOSE_FORMAT("planner parameters saved to %s", filename);
        }
        _DumpTrajectory(ptraj, Level_Verbose);

        // Save velocities
        std::vector<KinBody::KinBodyStateSaverPtr> vstatesavers;
        std::vector<KinBodyPtr> vusedbodies;
        _parameters->_configurationspecification.ExtractUsedBodies(GetEnv(), vusedbodies);
        if( vusedbodies.size() == 0 ) {
            RAVELOG_WARN("There is no used bodies in this configuration\n");
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

        uint32_t baseTime = utils::GetMilliTime();
        ConfigurationSpecification posSpec = _parameters->_configurationspecification;
        ConfigurationSpecification velSpec = posSpec.ConvertToVelocitySpecification();
        ConfigurationSpecification timeSpec;
        timeSpec.AddDeltaTimeGroup();

        std::vector<ConfigurationSpecification::Group>::const_iterator itcompatposgroup = ptraj->GetConfigurationSpecification().FindCompatibleGroup(posSpec._vgroups.at(0), false);
        OPENRAVE_ASSERT_FORMAT(itcompatposgroup != ptraj->GetConfigurationSpecification()._vgroups.end(), "Failed to find group %s in the passed-in trajectory", posSpec._vgroups.at(0).name, ORE_InvalidArguments);

        ConstraintTrajectoryTimingParametersConstPtr parameters = boost::dynamic_pointer_cast<ConstraintTrajectoryTimingParameters const>(GetParameters());

        // Initialize a parabolicpath
        RampOptimizer::ParabolicPath& parabolicpath = _cacheparabolicpath;
        parabolicpath.Reset();
        OPENRAVE_ASSERT_OP(parameters->_vConfigVelocityLimit.size(), ==, parameters->_vConfigAccelerationLimit.size());
        OPENRAVE_ASSERT_OP((int) parameters->_vConfigVelocityLimit.size(), ==, parameters->GetDOF());

        // Retrieve waypoints
        bool bPathIsPerfectlyModeled = false; // will be true if the initial interpolation is linear or quadratic
        std::vector<dReal> q(_parameters->GetDOF());
        std::vector<dReal>& waypoints = _cacheWaypoints; // to store concatenated waypoints obtained from ptraj
        std::vector<dReal>& x0Vect = _cacheX0Vect, &x1Vect = _cacheX1Vect, &v0Vect = _cacheV0Vect, &v1Vect = _cacheV1Vect, &tVect = _cacheTVect;
        RampOptimizer::RampND& tempRampND = _cacheRampND;

        if( _parameters->_hastimestamps && itcompatposgroup->interpolation == "quadratic" ) {
            RAVELOG_VERBOSE("The initial trajectory is piecewise quadratic");

            // Convert the original OpenRAVE trajectory to a parabolicpath
            ptraj->GetWaypoint(0, x0Vect, posSpec);
            ptraj->GetWaypoint(0, v0Vect, velSpec);

            for (size_t iwaypoint = 1; iwaypoint < ptraj->GetNumWaypoints(); ++iwaypoint) {
                ptraj->GetWaypoint(iwaypoint, tVect, timeSpec);
                if( tVect.at(0) > g_fEpsilonLinear ) {
                    ptraj->GetWaypoint(iwaypoint, x1Vect, posSpec);
                    ptraj->GetWaypoint(iwaypoint, v1Vect, velSpec);
                    tempRampND.Initialize(x0Vect, x1Vect, v0Vect, v1Vect, std::vector<dReal>(), std::vector<dReal>(), tVect[0]);
                    parabolicpath.AppendRampND(tempRampND);
                    x0Vect.swap(x1Vect);
                    v0Vect.swap(v1Vect);
                }
            }
            bPathIsPerfectlyModeled = true;            
        }
        else if( _parameters->_hastimestamps && itcompatposgroup->interpolation == "cubic" ) {
            RAVELOG_VERBOSE("The initial trajectory is piecewise cubic");

            // Convert the original OpenRAVE trajectory to a parabolicpath
            ptraj->GetWaypoint(0, x0Vect, posSpec);
            ptraj->GetWaypoint(0, v0Vect, velSpec);

            std::vector<RampOptimizer::RampND>& tempRampNDVect = _cacheRampNDVect;
            for (size_t iwaypoint = 1; iwaypoint < ptraj->GetNumWaypoints(); ++iwaypoint) {
                ptraj->GetWaypoint(iwaypoint, tVect, timeSpec);
                if( tVect.at(0) > g_fEpsilonLinear ) {
                    ptraj->GetWaypoint(iwaypoint, x1Vect, posSpec);
                    ptraj->GetWaypoint(iwaypoint, v1Vect, velSpec);

                    dReal iDeltaTime = 1/tVect[0];
                    dReal iDeltaTime2 = iDeltaTime*iDeltaTime;
                    bool isParabolic = true;
                    for (size_t jdof = 0; jdof < x0Vect.size(); ++jdof) {
                        dReal coeff = (2.0*iDeltaTime*(x0Vect[jdof] - x1Vect[jdof]) + v0Vect[jdof] + v1Vect[jdof])*iDeltaTime2;
                        if( RaveFabs(coeff) > 1e-5 ) {
                            isParabolic = false;
                        }
                    }

                    if( isParabolic ) {
                        tempRampND.Initialize(x0Vect, x1Vect, v0Vect, v1Vect, std::vector<dReal>(), std::vector<dReal>(), tVect[0]);
                        if( !_parameters->verifyinitialpath ) {
                            tempRampND.constraintChecked = true;
                        }
                    }
                    else {
                        // We only check time-based constraints since the path is anyway likely to be modified during shortcutting.
                        if( !_TimeParameterizeZeroVel(x0Vect, x1Vect, CFO_CheckTimeBasedConstraints, tempRampNDVect) ) {
                            RAVELOG_WARN_FORMAT("env = %d: Failed to initialize from cubic waypoints", GetEnv()->GetId());
                            _DumpTrajectory(ptraj, Level_Debug);
                            return PS_Failed;
                        }
                    }

                    FOREACHC(itrampnd, tempRampNDVect) {
                        parabolicpath.AppendRampND(*itrampnd);
                    }
                    x0Vect.swap(x1Vect);
                    v0Vect.swap(v1Vect);
                }
            }
        }
        else {
            if( itcompatposgroup->interpolation.size() == 0 || itcompatposgroup->interpolation == "linear" ) {
                RAVELOG_VERBOSE("The initial trajectory is piecewise linear");
                bPathIsPerfectlyModeled = true;
            }
            else {
                RAVELOG_VERBOSE("The initial trajectory is with unspecified interpolation");
            }

            std::vector<std::vector<dReal> >& vWaypoints = _cacheWaypointVect;
            vWaypoints.resize(0);
            if( vWaypoints.capacity() < ptraj->GetNumWaypoints() ) {
                vWaypoints.reserve(ptraj->GetNumWaypoints());
            }

            ptraj->GetWaypoints(0, ptraj->GetNumWaypoints(), waypoints, posSpec);

            // Iterate through all waypoints to remove collinear ones.
            dReal collinearThresh = 1e-14;
            for (size_t iwaypoint = 0; iwaypoint < ptraj->GetNumWaypoints(); ++iwaypoint) {
                // Copy waypoints[iwaypoint] into q
                std::copy(waypoints.begin() + iwaypoint*_parameters->GetDOF(), waypoints.begin() + (iwaypoint + 1)*_parameters->GetDOF(), q.begin());

                if( vWaypoints.size() > 1 ) {
                    // Check if the new waypoint (q) is collinear with the previous ones.
                    const std::vector<dReal>& x0 = vWaypoints[vWaypoints.size() - 2];
                    const std::vector<dReal>& x1 = vWaypoints[vWaypoints.size() - 1];
                    dReal dotProduct = 0, x0Length2 = 0, x1Length2 = 0;

                    for (size_t idof = 0; idof < q.size(); ++idof) {
                        dReal dx0 = x0[idof] - q[idof];
                        dReal dx1 = x1[idof] - q[idof];
                        dotProduct += dx0*dx1;
                        x0Length2 += dx0*dx0;
                        x1Length2 += dx1*dx1;
                    }
                    if( RaveFabs(dotProduct*dotProduct - x0Length2*x1Length2) < collinearThresh ) {
                        // Points are collinear
                        vWaypoints.back() = q;
                        continue;
                    }
                }

                // Check if the new point is not the same as the previous one
                if( vWaypoints.size() > 0 ) {
                    dReal d = 0;
                    for (size_t idof = 0; idof < q.size(); ++idof) {
                        d += RaveFabs(q[idof] - vWaypoints.back()[idof]);
                    }
                    if( d <= q.size()*std::numeric_limits<dReal>::epsilon() ) {
                        continue;
                    }
                }

                // The new point is not redundant. Add it to vWaypoints.
                vWaypoints.push_back(q);
            }

            // Time-parameterize the initial path
            if( !_SetMileStones(vWaypoints, parabolicpath) ) {
                RAVELOG_WARN_FORMAT("env = %d: Failed to initialize from piecewise linear waypoints", GetEnv()->GetId());
                _DumpTrajectory(ptraj, Level_Debug);
                return PS_Failed;
            }
            RAVELOG_DEBUG_FORMAT("env = %d: Finished initializing linear waypoints via _SetMileStones", GetEnv()->GetId());
            RAVELOG_DEBUG_FORMAT("#waypoint: %d -> %d", ptraj->GetNumWaypoints()%vWaypoints.size());
        }

        // Tell parabolicsmoother not to check constraints again if we already did (e.g. in linearsmoother, etc.)
        if( !_parameters->verifyinitialpath && bPathIsPerfectlyModeled ) {
            FOREACH(itrampnd, parabolicpath.GetRampNDVect()) {
                itrampnd->constraintChecked = true;
            }
        }

        // Main planning loop
        try {
            _bUsePerturbation = true;
            _feasibilitychecker.tol = parameters->_vConfigResolution;
            FOREACH(it, _feasibilitychecker.tol) {
                *it *= parameters->_pointtolerance;
            }

            _progress._iteration = 0;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return PS_Interrupted;
            }

            int numShortcuts = 0;
            if( !!parameters->_setstatevaluesfn || !!parameters->_setstatefn ) {
                numShortcuts = _Shortcut(parabolicpath, parameters->_nMaxIterations, this, parameters->_fStepLength*0.99);
                if( numShortcuts < 0 ) {
                    return PS_Interrupted;
                }
            }

            ++_progress._iteration;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return PS_Interrupted;
            }

            // Now start converting parabolicpath to OpenRAVE trajectory
            ConfigurationSpecification newSpec = posSpec;
            newSpec.AddDerivativeGroups(1, true);
            int waypointOffset = newSpec.AddGroup("iswaypoint", 1, "next");
            int timeOffset = -1;
            FOREACH(itgroup, newSpec._vgroups) {
                if( itgroup->name == "deltatime" ) {
                    timeOffset = itgroup->offset;
                }
                else if( velSpec.FindCompatibleGroup(*itgroup) != velSpec._vgroups.end() ) {
                    itgroup->interpolation = "linear";
                }
                else if( posSpec.FindCompatibleGroup(*itgroup) != posSpec._vgroups.end() ) {
                    itgroup->interpolation = "quadratic";
                }
            }

            // Write shortcut trajectory to dummytraj first
            if( !_pdummytraj || (_pdummytraj->GetXMLId() != ptraj->GetXMLId()) ) {
                _pdummytraj = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
            }
            _pdummytraj->Init(newSpec);

            // Consistency checking
            FOREACH(itrampnd, parabolicpath.GetRampNDVect()) {
                OPENRAVE_ASSERT_OP((int) itrampnd->GetDOF(), ==, _parameters->GetDOF());
            }

            RAVELOG_DEBUG("Start inserting the first waypoint to dummytraj");
            waypoints.resize(newSpec.GetDOF()); // reuse _cacheWaypoints

            ConfigurationSpecification::ConvertData(waypoints.begin(), newSpec, parabolicpath.GetRampNDVect().front().GetX0Vect(), posSpec, 1, GetEnv(), true);
            ConfigurationSpecification::ConvertData(waypoints.begin(), newSpec, parabolicpath.GetRampNDVect().front().GetV0Vect(), velSpec, 1, GetEnv(), false);
            waypoints.at(waypointOffset) = 1;
            waypoints.at(timeOffset) = 0;
            _pdummytraj->Insert(_pdummytraj->GetNumWaypoints(), waypoints);

            RampOptimizer::RampND& rampndTrimmed = _cacheRampND; // another reference to _cacheRampND
            RampOptimizer::RampND& remRampND = _cacheRemRampND;
            std::vector<RampOptimizer::RampND>& tempRampNDVect = _cacheRampNDVect;
            std::vector<dReal> vConfig;
            dReal fTrimEdgesTime = parameters->_fStepLength*2; // we ignore collisions duration [0, fTrimEdgesTime] and [fTrimEdgesTime, duration]
            dReal fExpextedDuration = 0; // for consistency checking
            dReal durationDiscrepancyThresh = 0.01; // for consistency checking

            for (size_t irampnd = 0; irampnd < parabolicpath.GetRampNDVect().size(); ++irampnd) {
                rampndTrimmed = parabolicpath.GetRampNDVect()[irampnd];

                // tempRampNDVect will contain the finalized result of each RampND
                tempRampNDVect.resize(1);
                tempRampNDVect[0] = rampndTrimmed;
                ++_progress._iteration;

                // Check constraints if not yet checked.
                if( !rampndTrimmed.constraintChecked ) {
                    bool bTrimmedFront = false;
                    bool bTrimmedBack = false;
                    bool bCheck = true;
                    if( irampnd == 0 ) {
                        if( rampndTrimmed.GetDuration() <= fTrimEdgesTime + g_fEpsilonLinear ) {
                            // The initial RampND is too short so ignore checking
                            bCheck = false;
                        }
                        else {
                            remRampND = rampndTrimmed;
                            remRampND.Cut(fTrimEdgesTime, rampndTrimmed);
                            bTrimmedFront = true;
                        }
                    }
                    else if( irampnd + 1 == parabolicpath.GetRampNDVect().size() ) {
                        if( rampndTrimmed.GetDuration() <= fTrimEdgesTime + g_fEpsilonLinear ) {
                            // The final RampND is too short so ignore checking
                            bCheck = false;
                        }
                        else {
                            rampndTrimmed.Cut(fTrimEdgesTime, remRampND);
                            bTrimmedBack = true;
                        }
                    }

                    _bUsePerturbation = false;

                    std::vector<RampOptimizer::RampND>& rampndVectOut = _cacheRampNDVectOut;
                    if( bCheck ) {
                        RampOptimizer::CheckReturn checkret = _feasibilitychecker.Check2(rampndTrimmed, 0xffff, rampndVectOut);
                        if( checkret.retcode != 0 ) {
                            RAVELOG_DEBUG_FORMAT("env = %d: Check2 for RampND %d/%d return retcode = 0x%x", GetEnv()->GetId()%irampnd%parabolicpath.GetRampNDVect().size()%checkret.retcode);

                            bool bSuccess = false;
                            // Try to stretch the duration of the RampND in hopes of fixing constraints violation.
                            dReal newDuration = rampndTrimmed.GetDuration();
                            newDuration += RampOptimizer::g_fRampEpsilon;
                            dReal timeIncrement = 0.05*newDuration;
                            size_t maxTries = 30;

                            rampndTrimmed.GetX0Vect(x0Vect);
                            rampndTrimmed.GetX1Vect(x1Vect);
                            rampndTrimmed.GetV0Vect(v0Vect);
                            rampndTrimmed.GetV1Vect(v1Vect);                            
                            for (size_t iDilate = 0; iDilate < maxTries; ++iDilate) {
#ifdef SMOOTHER_TIMING_DEBUG
                                _nCallsInterpolator += 1;
                                _tStartInterpolator = utils::GetMicroTime();
#endif
                                bool result = _interpolator.ComputeNDTrajectoryFixedDuration(x0Vect, x1Vect, v0Vect, v1Vect, newDuration, parameters->_vConfigLowerLimit, parameters->_vConfigUpperLimit, parameters->_vConfigVelocityLimit, parameters->_vConfigAccelerationLimit, rampndVectOut);
#ifdef SMOOTHER_TIMING_DEBUG
                                _tEndInterpolator = utils.GetMicroTime();
                                _totalTimeInterpolator += 0.000001f*(float)(_tEndInterpolator - _tStartInterpolator);
#endif
                                if( result ) {
                                    // Stretching is successful
                                    RAVELOG_VERBOSE_FORMAT("env = %d: duration %.15e -> %.15e", GetEnv()->GetId()%rampndTrimmed.GetDuration()%newDuration);
                                    RampOptimizer::CheckReturn newrampndret = _feasibilitychecker.Check2(rampndVectOut, 0xffff, tempRampNDVect);
                                    if( newrampndret.retcode == 0 ) {
                                        // The new RampND passes the check
                                        if( bTrimmedFront ) {
                                            tempRampNDVect.insert(tempRampNDVect.begin(), remRampND);
                                        }
                                        else if( bTrimmedBack ) {
                                            tempRampNDVect.reserve(tempRampNDVect.size() + 1);
                                            tempRampNDVect.push_back(remRampND);
                                        }
                                        bSuccess = true;
                                        break;
                                    }
                                }

                                // ComputeNDTrajectoryFixedDuration failed or Check2 failed.
                                if( iDilate > 2 ) {
                                    newDuration += timeIncrement;
                                }
                                else {
                                    // Start slowly
                                    newDuration += RampOptimizer::g_fRampEpsilon;
                                }
                            }
                            // Finished stretching.

                            if( !bSuccess ) {
                                if (IS_DEBUGLEVEL(Level_Verbose)) {
                                    std::stringstream ss;
                                    std::string separator = "";
                                    ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                                    ss << "x0 = [";
                                    FOREACHC(itvalue, x0Vect) {
                                        ss << separator << *itvalue;
                                        separator = ", ";
                                    }
                                    ss << "]; x1 = [";
                                    separator = "";
                                    FOREACHC(itvalue, x1Vect) {
                                        ss << separator << *itvalue;
                                        separator = ", ";
                                    }
                                    ss << "]; v0 = [";
                                    separator = "";
                                    FOREACHC(itvalue, v0Vect) {
                                        ss << separator << *itvalue;
                                        separator = ", ";
                                    }
                                    ss << "]; v1 = [";
                                    separator = "";
                                    FOREACHC(itvalue, v1Vect) {
                                        ss << separator << *itvalue;
                                        separator = ", ";
                                    }
                                    ss << "]; deltatime = " << rampndTrimmed.GetDuration();
                                    RAVELOG_WARN_FORMAT("env = %d: original RampND %d/%d does not satisfy constraints. retcode = 0x%x. %s", GetEnv()->GetId()%irampnd%parabolicpath.GetRampNDVect().size()%checkret.retcode%ss.str());
                                }
                                else {
                                    RAVELOG_WARN_FORMAT("env = %d: original RampND %d/%d does not satisfy constraints. retcode = 0x%x", GetEnv()->GetId()%irampnd%parabolicpath.GetRampNDVect().size()%checkret.retcode);
                                }
                                _DumpTrajectory(ptraj, Level_Debug);
                                return PS_Failed;
                            }
                        }
                    }
                    _bUsePerturbation = true;
                    ++_progress._iteration;

                    if( _CallCallbacks(_progress) == PA_Interrupt ) {
                        return PS_Interrupted;
                    }
                }// Finished checking constraints

                FOREACH(itrampnd, tempRampNDVect) {
                    fExpextedDuration += itrampnd->GetDuration();
                    waypoints.resize(newSpec.GetDOF());
                    std::vector<dReal>::iterator ittargetdata = waypoints.begin();

                    itrampnd->GetX1Vect(vConfig);
                    ConfigurationSpecification::ConvertData(ittargetdata, newSpec, vConfig.begin(), posSpec, 1, GetEnv(), true);
                    itrampnd->GetV1Vect(vConfig);
                    ConfigurationSpecification::ConvertData(ittargetdata, newSpec, vConfig.begin(), velSpec, 1, GetEnv(), false);

                    *(ittargetdata + timeOffset) = itrampnd->GetDuration();
                    *(ittargetdata + waypointOffset) = 1; /////////////// Check this
                    ittargetdata += newSpec.GetDOF();
                    _pdummytraj->Insert(_pdummytraj->GetNumWaypoints(), waypoints);
                }                

                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    // If verbose, do tighter bound checking
                    OPENRAVE_ASSERT_OP(RaveFabs(fExpextedDuration - _pdummytraj->GetDuration()), <=, 0.1*durationDiscrepancyThresh);
                }
            }
            OPENRAVE_ASSERT_OP(RaveFabs(fExpextedDuration - _pdummytraj->GetDuration()), <=, durationDiscrepancyThresh);
            ptraj->Swap(_pdummytraj);
        }
        catch (const std::exception& ex) {
            _DumpTrajectory(ptraj, Level_Debug);
            RAVELOG_WARN_FORMAT("env = %d: Main planning loop threw exception %s", GetEnv()->GetId()%ex.what());
            return PS_Failed;
        }
        RAVELOG_DEBUG_FORMAT("env = %d: path optimizing - computation time = %f s.", GetEnv()->GetId()%(0.001f*(float)(utils::GetMilliTime() - baseTime)));

        if( IS_DEBUGLEVEL(Level_Debug) ) {
            RAVELOG_DEBUG_FORMAT("env = %d: Start sampling trajectory after shortcutting (for verification)", GetEnv()->GetId());
            try {
                ptraj->Sample(x0Vect, 0); // reuse x0Vect
                RAVELOG_DEBUG_FORMAT("env = %d: Sampling for verification successful", GetEnv()->GetId());
            }
            catch (const std::exception& ex) {
                RAVELOG_WARN_FORMAT("env = %d: Sampling for verification failed: %s", GetEnv()->GetId()%ex.what());
                _DumpTrajectory(ptraj, Level_Debug);
                return PS_Failed;
            }
        }
        _DumpTrajectory(ptraj, Level_Debug);

#ifdef SMOOTHER_TIMING_DEBUG
        RAVELOG_DEBUG_FORMAT("measured %d interpolations; total exectime = %.15e; time/iter = %.15e", _nCallsInterpolator%_totalTimeInterpolator%(_totalTimeInterpolator/_nCallsInterpolator));
        RAVELOG_DEBUG_FORMAT("measured %d checkmanips; total exectime = %.15e; time/iter = %.15e", _nCallsCheckManip%_totalTimeCheckManip%(_totalTimeCheckManip/_nCallsCheckManip));
#endif
        return _ProcessPostPlanners(RobotBasePtr(), ptraj);
    }

    virtual int ConfigFeasible(const std::vector<dReal>& q0, const std::vector<dReal>& dq0, int options)
    {
        if( _bUsePerturbation ) {
            options |= CFO_CheckWithPerturbation;
        }
        try {
            return _parameters->CheckPathAllConstraints(q0, q0, dq0, dq0, 0, IT_OpenStart, options);
        }
        catch (const std::exception& ex) {
            RAVELOG_WARN_FORMAT("env = %d: CheckPathAllConstraints threw an exception: %s", GetEnv()->GetId()%ex.what());
            return 0xffff;
        }
    }

    /// \brief ConfigFeasible2 does the same thing as ConfigFeasible. The difference is that it
    /// returns RampOptimizer::CheckReturn instead of an int. fTimeBasedSurpassMult is also set to
    /// some value if the configuration violates some time-based constraints.
    virtual RampOptimizer::CheckReturn ConfigFeasible2(const std::vector<dReal>& q0, const std::vector<dReal>& dq0, int options)
    {
        if( _bUsePerturbation ) {
            options |= CFO_CheckWithPerturbation;
        }
        try {
            int ret = _parameters->CheckPathAllConstraints(q0, q0, dq0, dq0, 0, IT_OpenStart, options);
            RampOptimizer::CheckReturn checkret(ret);
            if( ret == CFO_CheckTimeBasedConstraints ) {
                checkret.fTimeBasedSurpassMult = 0.8;
            }
            return checkret;
        }
        catch (const std::exception& ex) {
            RAVELOG_WARN_FORMAT("env = %d: CheckPathAllConstraints threw an exception: %s", GetEnv()->GetId()%ex.what());
            return 0xffff;
        }
    }

    /// \brief Check if the segment interpolating (q0, dq0) and (q1, dq1) is feasible. The function
    /// first calls CheckPathAllConstraints to check all constraints. Since the input path may be
    /// modified from inside CheckPathAllConstraints, after the checking this function also try to
    /// correct any discrepancy occured.
    virtual RampOptimizer::CheckReturn SegmentFeasible2(const std::vector<dReal>& q0, const std::vector<dReal>& q1, const std::vector<dReal>& dq0, const std::vector<dReal>& dq1, dReal timeElapsed, int options, std::vector<RampOptimizer::RampND>& rampndVectOut)
    {
        size_t ndof = q0.size();

        if( timeElapsed <= g_fEpsilon ) {
            rampndVectOut.resize(1);
            rampndVectOut[0].Initialize(_parameters->GetDOF());
            rampndVectOut[0].SetConstant(q0, 0);
            rampndVectOut[0].SetV0Vect(dq0);
            rampndVectOut[0].SetV1Vect(dq1);
            return ConfigFeasible2(q0, dq0, options);
        }

        rampndVectOut.resize(0);
        if( _bUsePerturbation ) {
            options |= CFO_CheckWithPerturbation;
        }

        bool bExpectedModifiedConfigurations = (_parameters->fCosManipAngleThresh > -1 + g_fEpsilonLinear);
        if( bExpectedModifiedConfigurations || _bmanipconstraints ) {
            options |= CFO_FillCheckedConfiguration;
            _constraintreturn->Clear();
        }

        try {
            int ret = _parameters->CheckPathAllConstraints(q0, q1, dq0, dq1, timeElapsed, IT_OpenStart, options, _constraintreturn);
            if( ret != 0 ) {
                RampOptimizer::CheckReturn checkret(ret);
                if( ret == CFO_CheckTimeBasedConstraints ) {
                    checkret.fTimeBasedSurpassMult = 0.8;
                }
                return checkret;
            }
        }
        catch (const std::exception& ex) {
            RAVELOG_WARN_FORMAT("env = %d: CheckPathAllConstraints threw an exception: %s", GetEnv()->GetId()%ex.what());
            return RampOptimizer::CheckReturn(0xffff);
        }

        // Configurations between (q0, dq0) and (q1, dq1) may have been modified.
        if( bExpectedModifiedConfigurations && _constraintreturn->_configurationtimes.size() > 0 ) {
            OPENRAVE_ASSERT_OP(_constraintreturn->_configurations.size(), ==, _constraintreturn->_configurationtimes.size()*ndof);

            std::vector<dReal> curPos = q0, newPos(ndof);
            std::vector<dReal> curVel = dq0, newVel(ndof);

            std::vector<dReal>::const_iterator it = _constraintreturn->_configurations.begin();
            dReal curTime = 0;

            for (size_t itime = 0; itime < _constraintreturn->_configurationtimes.size(); ++itime, it += ndof) {
                std::copy(it, it + ndof, newPos.begin());
                dReal deltaTime = _constraintreturn->_configurationtimes[itime] - curTime;
                if( deltaTime > g_fEpsilon ) {
                    dReal iDeltaTime = 1/deltaTime;

                    // Compute the next velocity for each DOF as well as check consistency
                    for (size_t idof = 0; idof < ndof; ++idof) {
                        newVel[idof] = 2*iDeltaTime*(newPos[idof] - curPos[idof]) - curVel[idof];

                        // Check velocity limit
                        if( RaveFabs(newVel[idof]) > _parameters->_vConfigVelocityLimit[idof] + RampOptimizer::g_fRampEpsilon  ) {
                            if( 0.9*_parameters->_vConfigVelocityLimit[idof] < 0.1*RaveFabs(newVel[idof]) ) {
                                // Warn if the velocity is really too high
                                RAVELOG_WARN_FORMAT("env = %d: the new velocity for idof = %d is too high. |%.15e| > %.15e", GetEnv()->GetId()%idof%newVel[idof]%_parameters->_vConfigVelocityLimit[idof]);
                            }
                            RAVELOG_VERBOSE_FORMAT("retcode = 0x4; idof = %d; newVel = %.15e; vellimit = %.15e; diff = %.15e", idof%newVel[idof]%_parameters->_vConfigVelocityLimit[idof]%(RaveFabs(newVel[idof]) - _parameters->_vConfigVelocityLimit[idof]));
                            return RampOptimizer::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9*_parameters->_vConfigVelocityLimit[idof]/RaveFabs(newVel[idof]));
                        }
                    }

                    // The computed next velocity is fine.
                    _cacheRampNDSeg.Initialize(curPos, newPos, curVel, newVel, std::vector<dReal>(), std::vector<dReal>(), deltaTime);

                    // Now check the acceleration
                    bool bAccelChanged = false;
                    for (size_t idof = 0; idof < ndof; ++idof) {
                        if( _cacheRampNDSeg.GetAAt(idof) < -_parameters->_vConfigAccelerationLimit[idof] ) {
                            _cacheRampNDSeg.SetAAt(idof) = -_parameters->_vConfigAccelerationLimit[idof];
                        }
                        else if( _cacheRampNDSeg.GetAAt(idof) > _parameters->_vConfigAccelerationLimit[idof] ) {
                            _cacheRampNDSeg.SetAAt(idof) = _parameters->_vConfigAccelerationLimit[idof];
                        }
                    }
                    if( bAccelChanged ) {
                        RampOptimizer::ParabolicCheckReturn parabolicret = RampOptimizer::CheckRampND(_cacheRampNDSeg, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit);
                        if( parabolicret != RampOptimizer::PCR_Normal ) {
                            std::stringstream ss;
                            std::string separator = "";
                            ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                            ss << "x0 = [";
                            FOREACHC(itvalue, curPos) {
                                ss << separator << *itvalue;
                                separator = ", ";
                            }
                            ss << "]; x1 = [";
                            separator = "";
                            FOREACHC(itvalue, newPos) {
                                ss << separator << *itvalue;
                                separator = ", ";
                            }
                            ss << "]; v0 = [";
                            separator = "";
                            FOREACHC(itvalue, curVel) {
                                ss << separator << *itvalue;
                                separator = ", ";
                            }
                            ss << "]; v1 = [";
                            separator = "";
                            FOREACHC(itvalue, newVel) {
                                ss << separator << *itvalue;
                                separator = ", ";
                            }
                            ss << "]; deltatime = " << deltaTime;

                            RAVELOG_WARN_FORMAT("env = %d: the output RampND becomes invalid (ret = %x) after fixing accelerations. %s", GetEnv()->GetId()%parabolicret%ss.str());
                            return RampOptimizer::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9);
                        }
                    }
                    _cacheRampNDSeg.constraintChecked = true;

                    rampndVectOut.push_back(_cacheRampNDSeg);
                    curTime = _constraintreturn->_configurationtimes[itime];
                    curPos.swap(newPos);
                    curVel.swap(newVel);
                }
            }

            // Make sure the last configuration ends at the desired value.
            for (size_t idof = 0; idof < ndof; ++idof) {
                if( RaveFabs(curPos[idof] - q1[idof]) + g_fEpsilon > RampOptimizer::g_fRampEpsilon ) {
                    RAVELOG_WARN_FORMAT("env = %d: discrepancy at the last configuration: curPos[%d] (%.15e) != q1[%d] (%.15e)", GetEnv()->GetId()%idof%curPos[idof]%idof%q1[idof]);
                    return RampOptimizer::CheckReturn(CFO_FinalValuesNotReached);
                }
            }
        }

        if( rampndVectOut.size() == 0 ) {
            _cacheRampNDSeg.Initialize(q0, q1, dq0, dq1, std::vector<dReal>(), std::vector<dReal>(), timeElapsed);
            _cacheRampNDSeg.constraintChecked = true;
            rampndVectOut.push_back(_cacheRampNDSeg);
        }

        if( _bmanipconstraints && (options & CFO_CheckTimeBasedConstraints) ) {
            try {
#ifdef SMOOTHER_TIMING_DEBUG
                _nCallsCheckManip += 1;
                _tStartCheckManip = utils::GetMicroTime();
#endif
                RampOptimizer::CheckReturn retmanip = _manipconstraintchecker->CheckManipConstraints2(rampndVectOut);
#ifdef SMOOTHER_TIMING_DEBUG
                _tEndCheckManip = utils::GetMicroTime();
                _totalTimeCheckManip += 0.000001f*(float)(_tEndCheckManip - _tStartCheckManip);
#endif
                if( retmanip.retcode != 0 ) {
                    RAVELOG_VERBOSE_FORMAT("env = %d: CheckManipConstraints2 returns retcode = 0x%x", GetEnv()->GetId()%retmanip.retcode);
                    return retmanip;
                }
            }
            catch (const std::exception& ex) {
                RAVELOG_VERBOSE_FORMAT("env = %d: CheckManipConstraints2 (modified = %d) threw an exception: %s", GetEnv()->GetId()%((int) bExpectedModifiedConfigurations)%ex.what());
                return RampOptimizer::CheckReturn(0xffff);
            }
        }

        return RampOptimizer::CheckReturn(0);
    }

    virtual dReal Rand()
    {
        return _uniformsampler->SampleSequenceOneReal(IT_OpenEnd);
    }

    virtual bool NeedDerivativeForFeasibility()
    {
        return true;
    }
    
protected:

    /// \brief Time-parameterize the ordered set of waypoints to a trajectory that stops at every
    /// waypoint. _SetMilestones also adds some extra waypoints to the original set if any two
    /// consecutive waypoints are too far apart.
    bool _SetMileStones(const std::vector<std::vector<dReal> >& vWaypoints, RampOptimizer::ParabolicPath& parabolicpath)
    {
        _zeroVelPoints.resize(0);
        if( _zeroVelPoints.capacity() < vWaypoints.size() ) {
            _zeroVelPoints.reserve(vWaypoints.size());
        }

        size_t ndof = _parameters->GetDOF();
        parabolicpath.Reset();
        RAVELOG_VERBOSE_FORMAT("env = %d: Initial numwaypoints = %d", GetEnv()->GetId()%vWaypoints.size());

        std::vector<RampOptimizer::RampND>& rampndVect = _cacheRampNDVect1;
        if( vWaypoints.size() == 1 ) {
            rampndVect.resize(1);
            rampndVect[0].Initialize(_parameters->GetDOF());
            rampndVect[0].SetConstant(vWaypoints[0], 0);
            parabolicpath.Initialize(rampndVect[0]);
        }
        else if( vWaypoints.size() > 1 ) {
            int options = CFO_CheckTimeBasedConstraints;
            if( !_parameters->verifyinitialpath ) {
                options = options & (~CFO_CheckEnvCollisions) & (~CFO_CheckSelfCollisions);
                RAVELOG_VERBOSE_FORMAT("env = %d: Initial path verification disabled using options = 0x%x", GetEnv()->GetId()%options);
            }

            // In some cases (e.g. when there are manipulator constraints), the midpoint 0.5*(x0 +
            // x1) may not satisfy the constraints. Instead of returning failure, we try to compute
            // a better midpoint.
            std::vector<std::vector<dReal> >& vNewWaypoints = _cacheNewWaypointsVect;
            std::vector<uint8_t> vForceInitialChecking(vWaypoints.size(), 0);

            if( !!_parameters->_neighstatefn ) {
                std::vector<dReal> xmid(ndof), xmidDelta(ndof);
                vNewWaypoints = vWaypoints;

                // We add more waypoints in between the original consecutive waypoints x0 and x1 if
                // the constraint-satisfying middle point (computed using _neighstatefn) is far from
                // the expected middle point 0.5*(x0 + x1).
                dReal distThresh = 0.00001;
                int nConsecutiveExpansionsAllowed = 10; // adding too many intermediate waypoints is considered bad
                int nConsecutiveExpansions = 0;
                size_t iwaypoint = 0; // keeps track of the total number of (fianlized) waypoints
                while (iwaypoint + 1 < vNewWaypoints.size()) {
                    // xmidDelta is the difference between the expected middle point and the initial point
                    for (size_t idof = 0; idof < ndof; ++idof) {
                        xmidDelta[idof] = 0.5*(vNewWaypoints[iwaypoint + 1][idof] - vNewWaypoints[iwaypoint][idof]);
                    }

                    xmid = vNewWaypoints[iwaypoint];
                    if( _parameters->SetStateValues(xmid) != 0 ) {
                        RAVELOG_WARN_FORMAT("env = %d: Could not set values at waypoint %d", GetEnv()->GetId()%iwaypoint);
                        return false;
                    }
                    // Steer vNewWaypoints[iwaypoint] by xmidDelta. The resulting state is stored in xmid.
                    if( !_parameters->_neighstatefn(xmid, xmidDelta, NSO_OnlyHardConstraints) ) {
                        RAVELOG_WARN_FORMAT("env = %d: Failed to get the neighbor of waypoint %d", GetEnv()->GetId()%iwaypoint);
                        return false;
                    }

                    // Check if xmid is far from the expected point.
                    dReal dist = 0;
                    for (size_t idof = 0; idof < ndof; ++idof) {
                        dReal fExpected = 0.5*(vNewWaypoints[iwaypoint + 1][idof] + vNewWaypoints[iwaypoint][idof]);
                        dReal fError = fExpected - xmid[idof];
                        dist += fError*fError;
                    }
                    if( dist > distThresh ) {
                        RAVELOG_DEBUG_FORMAT("env = %d: Adding extra midpoint between waypoints %d and %d, dist = %.15e", GetEnv()->GetId()%(iwaypoint - 1)%iwaypoint%dist);
                        vNewWaypoints.insert(vNewWaypoints.begin() + iwaypoint + 1, xmid);
                        vForceInitialChecking[iwaypoint + 1] = 1;
                        vForceInitialChecking.insert(vForceInitialChecking.begin() + iwaypoint + 1, 1);
                        nConsecutiveExpansions += 2;
                        if( nConsecutiveExpansions > nConsecutiveExpansionsAllowed ) {
                            RAVELOG_WARN_FORMAT("env = %d: Too many consecutive expansions, waypoint %d/%d is bad", GetEnv()->GetId()%iwaypoint%vNewWaypoints.size());
                            return false;
                        }
                        continue;
                    }
                    if( nConsecutiveExpansions > 0 ) {
                        nConsecutiveExpansions--;
                    }
                    iwaypoint += 1;
                }
            }
            else {
                // No _neighstatefn.
                vNewWaypoints = vWaypoints;
            }
            // Finished preparation of waypoints. Now continue to time-parameterize the path.

            OPENRAVE_ASSERT_OP(vNewWaypoints[0].size(), ==, ndof);
            std::vector<RampOptimizer::RampND>& rampndVect = _cacheRampNDVect1;
            for (size_t iwaypoint = 1; iwaypoint < vNewWaypoints.size(); ++iwaypoint) {
                OPENRAVE_ASSERT_OP(vNewWaypoints[iwaypoint].size(), ==, ndof);

                if( !_TimeParameterizeZeroVel(vNewWaypoints[iwaypoint - 1], vNewWaypoints[iwaypoint], options, rampndVect) ) {
                    RAVELOG_WARN_FORMAT("env = %d: Failed to time-parameterize path connecting waypoints %d and %d", GetEnv()->GetId()%(iwaypoint - 1)%iwaypoint);
                    return false;
                }

                if( !_parameters->verifyinitialpath && !vForceInitialChecking[iwaypoint] ) {
                    FOREACH(itrampnd, rampndVect) {
                        itrampnd->constraintChecked = true;
                    }
                }

                // Keep track of zero-velocity waypoints
                dReal duration = 0;
                FOREACHC(itrampnd, rampndVect) {
                    duration += itrampnd->GetDuration();
                    parabolicpath.AppendRampND(*itrampnd);
                }
                if( _zeroVelPoints.size() == 0 ) {
                    _zeroVelPoints.push_back(duration);
                }
                else {
                    _zeroVelPoints.push_back(_zeroVelPoints.back() + duration);
                }
            }
            _zeroVelPoints.pop_back(); // now containing all zero-velocity points except the start and the end
        }

        return true;
    }

    /// \brief Interpolate two given waypoints with a trajectory which starts and ends with zero
    /// velocities. Manip constraints (if available) is also taken care of by gradually scaling
    /// vellimits and accellimits down until the constraints are no longer violated. Therefore, the
    /// output trajectory (rampnd) is guaranteed to feasible.
    bool _TimeParameterizeZeroVel(const std::vector<dReal>& x0VectIn, const std::vector<dReal>& x1VectIn, int options, std::vector<RampOptimizer::RampND>& rampndVectOut)
    {
        // Cache
        std::vector<dReal> &x0Vect = _cacheX0Vect1, &x1Vect = _cacheX1Vect1, &v0Vect = _cacheV0Vect1, &v1Vect = _cacheV1Vect1;
        std::vector<dReal> &vellimits = _cacheVellimits, &accellimits = _cacheAccelLimits;
        vellimits = _parameters->_vConfigVelocityLimit;
        accellimits = _parameters->_vConfigAccelerationLimit;

        // For debugging
        std::stringstream sss;
        sss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        std::vector<dReal>& aVect = _cacheAVect1;
        
        RampOptimizer::CheckReturn retseg(0);
        size_t numTries = 30; // number of times allowed to scale down vellimits and accellimits
        for (size_t itry = 0; itry < numTries; ++itry) {
            bool res = _interpolator.ComputeZeroVelNDTrajectory(x0VectIn, x1VectIn, vellimits, accellimits, rampndVectOut);
            BOOST_ASSERT(res);

            size_t irampnd = 0;
            rampndVectOut[0].GetX0Vect(x0Vect);
            rampndVectOut[0].GetV0Vect(v0Vect);
            FOREACHC(itrampnd, rampndVectOut) {
                irampnd = itrampnd - rampndVectOut.begin();

                itrampnd->GetX1Vect(x1Vect);
                itrampnd->GetV1Vect(v1Vect);

                retseg = SegmentFeasible2(x0Vect, x1Vect, v0Vect, v1Vect, itrampnd->GetDuration(), options, _cacheRampNDVectOut1);
                if( 0 ) {
                    sss.str("");
                    sss.clear();
                    sss << "x0 = [";
                    SerializeValues(sss, x0Vect);
                    sss << "]; x1 = [";
                    SerializeValues(sss, x1Vect);
                    sss << "]; v0 = [";
                    SerializeValues(sss, v0Vect);
                    sss << "]; v1 = [";
                    SerializeValues(sss, v1Vect);
                    sss << "];";
                    RAVELOG_WARN(sss.str());
                }
                
                if( retseg.retcode != 0 ) {
                    break;
                }
                if( retseg.bDifferentVelocity ) {
                    RAVELOG_WARN_FORMAT("env = %d: SegmentFeasible2 returns different final velocities", GetEnv()->GetId());
                    retseg.retcode = CFO_FinalValuesNotReached;
                    break;
                }
                x0Vect.swap(x1Vect);
                v0Vect.swap(v1Vect);
            }
            if( retseg.retcode == 0 ) {
                break;
            }
            else if( retseg.retcode == CFO_CheckTimeBasedConstraints ) {
                RAVELOG_VERBOSE_FORMAT("env = %d: scaling vellimits and accellimits by %.15e, itry = %d", GetEnv()->GetId()%retseg.fTimeBasedSurpassMult%itry);
                RampOptimizer::ScaleVector(vellimits, retseg.fTimeBasedSurpassMult);
                RampOptimizer::ScaleVector(accellimits, retseg.fTimeBasedSurpassMult);
            }
            else {
                std::stringstream ss;
                ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                ss << "x0 = [";
                SerializeValues(ss, x0Vect);
                ss << "]; x1 = [";
                SerializeValues(ss, x1Vect);
                ss << "]; v0 = [";
                SerializeValues(ss, v0Vect);
                ss << "]; v1 = [";
                SerializeValues(ss, v1Vect);
                ss << "]; deltatime=" << (rampndVectOut[irampnd].GetDuration());
                RAVELOG_WARN_FORMAT("env = %d: SegmentFeasibile2 returned error 0x%x; %s, giving up....", GetEnv()->GetId()%retseg.retcode%ss.str());
                return false;
            }                
        }
        if( retseg.retcode != 0 ) {
            return false;
        }
        return true;
    }
    
    /// \brief Return the number of successful shortcut.
    int _Shortcut(RampOptimizer::ParabolicPath& parabolicpath, int numIters, RampOptimizer::RandomNumberGeneratorBase* rng, dReal minTimeStep)
    {
        int numShortcuts = 0;

        uint32_t fileindex;
        if( !!_logginguniformsampler ) {
            fileindex = _logginguniformsampler->SampleSequenceOneUInt32();
        }
        else {
            fileindex = RaveRandomInt();
        }
        fileindex = fileindex%_fileIndexMod;
        _DumpParabolicPath(parabolicpath, _dumplevel, fileindex, 0);
        
        return numShortcuts;
    }

    void _DumpParabolicPath(RampOptimizer::ParabolicPath& parabolicpath, DebugLevel level=Level_Verbose, uint32_t fileindex=10000, int option=-1) const
    {
        if( !IS_DEBUGLEVEL(level) ) {
            return;
        }
        if( fileindex == 10000 ) {
            // No particular index given. Randomly choose one.
            if( !!_logginguniformsampler ) {
                fileindex  = _logginguniformsampler->SampleSequenceOneUInt32();
            }
            else {
                fileindex = RaveRandomInt();
            }
            fileindex = fileindex%_fileIndexMod;
        }

        std::string filename;
        if( option == 0 ) {
            filename = str(boost::format("%s/parabolicpath%d.beforeshortcut.xml")%RaveGetHomeDirectory()%fileindex);
        }
        else if( option == 1 ) {
            filename = str(boost::format("%s/parabolicpath%d.aftershortcut.xml")%RaveGetHomeDirectory()%fileindex);
        }
        else {
            filename = str(boost::format("%s/parabolicpath%d.xml")%RaveGetHomeDirectory()%fileindex);
        }
        ofstream f(filename.c_str());
        f << std::setprecision(RampOptimizer::g_nPrec);
        parabolicpath.Serialize(f);
        RAVELOG_DEBUG_FORMAT("Wrote a parabolicpath to %s (duration = %.15e)", filename%parabolicpath.GetDuration());
        return;
    }

    std::string _DumpTrajectory(TrajectoryBasePtr ptraj, DebugLevel level)
    {
        if( IS_DEBUGLEVEL(level) ) {
            std::string filename = _DumpTrajectory(ptraj);
            RavePrintfA(str(boost::format("env = %d: wrote trajectory to %s")%GetEnv()->GetId()%filename), level);
            return filename;
        }
        else {
            return std::string();
        }
    }

    std::string _DumpTrajectory(TrajectoryBasePtr ptraj)
    {
        uint32_t randNum;
        if( !!_logginguniformsampler ) {
            randNum = _logginguniformsampler->SampleSequenceOneUInt32();
        }
        else {
            randNum = RaveRandomInt();
        }
        std::string filename = str(boost::format("%s/parabolicsmoother%d.traj.xml")%RaveGetHomeDirectory()%(randNum%1000));
        ofstream f(filename.c_str());
        f << std::setprecision(RampOptimizer::g_nPrec);
        ptraj->serialize(f);
        return filename;
    }
    
    /// Members
    ConstraintTrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _uniformsampler;        ///< used for planning, seed is controlled
    ConstraintFilterReturnPtr _constraintreturn;
    MyRampNDFeasibilityChecker _feasibilitychecker;
    boost::shared_ptr<ManipConstraintChecker2> _manipconstraintchecker;
    TrajectoryBasePtr _pdummytraj;
    PlannerProgress _progress;
    bool _bUsePerturbation;
    bool _bmanipconstraints;
    std::vector<dReal> _zeroVelPoints; ///< keeps track of original (zero-velocity) waypoints
    RampOptimizer::ParabolicInterpolator _interpolator;

    // for logging
    SpaceSamplerBasePtr _logginguniformsampler; ///< used for logging, seed is randomly set
    uint32_t _fileIndexMod; ///< maximum number of trajectory index allowed when saving
    DebugLevel _dumplevel;  ///< minimum debug level which triggers trajectory saving

    /// Caching stuff
    RampOptimizer::ParabolicPath _cacheparabolicpath;
    std::vector<dReal> _cacheWaypoints; ///< stores concatenated waypoints obtained from the input trajectory
    std::vector<std::vector<dReal> > _cacheWaypointVect; ///< each element is a vector storing a waypoint
    std::vector<dReal> _cacheX0Vect, _cacheX1Vect, _cacheV0Vect, _cacheV1Vect, _cacheTVect;
    RampOptimizer::RampND _cacheRampND, _cacheRemRampND;
    std::vector<RampOptimizer::RampND> _cacheRampNDVect;    ///< handles the finalized set of RampNDs before writing to OpenRAVE trajectory
    std::vector<RampOptimizer::RampND> _cacheRampNDVectOut; ///< used to handle output from Check function in PlanPath

    // in SegmentFeasible2
    RampOptimizer::RampND _cacheRampNDSeg;

    // in _SetMileStones
    std::vector<std::vector<dReal> > _cacheNewWaypointsVect;
    std::vector<RampOptimizer::RampND> _cacheRampNDVect1; ///< this one will also be used later in _Shortcut

    // in _TimeParameterizeZeroVel
    std::vector<dReal> _cacheX0Vect1, _cacheX1Vect1, _cacheV0Vect1, _cacheV1Vect1;
    std::vector<dReal> _cacheVellimits, _cacheAccelLimits; ///< stores current velocity and acceleration limits, also used in _Shortcut
    std::vector<RampOptimizer::RampND> _cacheRampNDVectOut1; ///< stores output from the check function, also used in _Shortcut
    std::vector<dReal> _cacheAVect1; // for debugging
    
#ifdef SMOOTHER_TIMING_DEBUG
    // Statistics
    size_t _nCallsCheckManip;
    dReal _totalTimeCheckManip;
    uint32_t _tStartCheckManip, _tEndCheckManip;

    size_t _nCallInterpolator;
    dReal _totalTimeInterpolator;
    uint32_t _tStartInterpolator, _tEndInterpolator;
#endif

}; // end class ParabolicSmoother2

PlannerBasePtr CreateParabolicSmoother2(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new ParabolicSmoother2(penv, sinput));
}

} // end namespace rplanners

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(RampOptimizer::Ramp)
BOOST_TYPEOF_REGISTER_TYPE(RampOptimizer::ParabolicCurve)
BOOST_TYPEOF_REGISTER_TYPE(RampOptimizer::RampND)
BOOST_TYPEOF_REGISTER_TYPE(RampOptimizer::ParabolicPath)
#endif
