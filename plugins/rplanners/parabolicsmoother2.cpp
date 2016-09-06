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

#include "manipconstraints2.h"
#include "rampoptimizer/interpolation.h"
#include "rampoptimizer/parabolicpath.h"

namespace rplanners {

namespace RampOptimizer = RampOptimizerInternal;

class ParabolicSmoother2 : public PlannerBase, public RampOptimizer::FeasibilityCheckerBase, public RampOptimizer::RandomNumberGeneratorBase {

    class MyParabolicCurvesNDFeasibilityChecker : public RampOptimizer::ParabolicCurvesNDFeasibilityChecker {
public:
        MyParabolicCurvesNDFeasibilityChecker(RampOptimizer::FeasibilityCheckerBase* feas) : RampOptimizer::ParabolicCurvesNDFeasibilityChecker(feas) {
        }

        // Check2 checks constraints at all switch points and on all segments connecting two consecutive switch points
        RampOptimizer::CheckReturn Check2(const RampOptimizer::ParabolicCurvesND& curvesnd, int options, std::vector<RampOptimizer::ParabolicCurvesND>& curvesndVectOut) {

            // Only set constraintCheckedVect to all true if all necessary constraints are checked
            if ((options & constraintmask) == constraintmask) {
                curvesnd.constraintChecked = true;
            }
            OPENRAVE_ASSERT_OP(tol.size(), ==, curvesnd.ndof);
            for (size_t i = 0; i < tol.size(); ++i) {
                OPENRAVE_ASSERT_OP(tol[i], >, 0);
            }

            vswitchtimes = curvesnd.switchpointsList;

            // Check boundary configurations
            RampOptimizer::CheckReturn ret0 = feas->ConfigFeasible2(curvesnd.x0Vect, curvesnd.v0Vect, options);
            if (ret0.retcode != 0) {
                return ret0;
            }

            // curvesnd.EvalPos(curvesnd.duration, q1);
            RampOptimizer::CheckReturn ret1 = feas->ConfigFeasible2(curvesnd.x1Vect, curvesnd.v1Vect, options);
            if (ret1.retcode != 0) {
                return ret1;
            }

            // Check if configurations are feasible at all switch times
            _vsearchsegments.resize(vswitchtimes.size(), 0);
            for (size_t i = 0; i < _vsearchsegments.size(); ++i) {
                _vsearchsegments[i] = i;
            }
            int midIndex = _vsearchsegments.size()/2;
            std::swap(_vsearchsegments[0], _vsearchsegments[midIndex]); // put the mid point as the first point to be considered

            for (size_t i = 0; i < vswitchtimes.size(); ++i) {
                dReal switchtime = vswitchtimes[_vsearchsegments[i]];
                curvesnd.EvalPos(switchtime, q0);
                if (feas->NeedDerivativeForFeasibility()) {
                    curvesnd.EvalVel(switchtime, dq0);
                }
                RampOptimizer::CheckReturn retconf = feas->ConfigFeasible2(q0, dq0, options);
                if (retconf.retcode != 0) {
                    return retconf;
                }
            }

            curvesndVectOut.resize(0);

            // Now sequentially check each segment (ParabolicCurvesND between each two consecutive switch points)
            q0 = curvesnd.x0Vect;
            dq0 = curvesnd.v0Vect;
            q1.resize(q0.size());
            dq1.resize(dq0.size());
            for (size_t iswitch = 1; iswitch < vswitchtimes.size(); ++iswitch) {
                curvesnd.EvalPos(vswitchtimes[iswitch], q1);
                dReal elapsedTime = vswitchtimes[iswitch] - vswitchtimes[iswitch - 1];

                if (feas->NeedDerivativeForFeasibility()) {
                    // Due to constraints, configurations may be modified inside
                    // SegmentFeasible2. Therefore, dq1 might not be consistent with q0, q1, and
                    // dq0.
                    curvesnd.EvalVel(vswitchtimes[iswitch], dq1); // original dq1

                    dReal expectedElapsedTime = 0;
                    dReal totalWeight = 0;
                    for (size_t idof = 0; idof < curvesnd.ndof; ++idof) {
                        dReal avgVel = 0.5*(dq0[idof] + dq1[idof]);
                        if (RaveFabs(avgVel) > g_fEpsilon) {
                            dReal fWeight = RaveFabs(q1[idof] - q0[idof]);
                            expectedElapsedTime += fWeight*(q1[idof] - q0[idof])/avgVel;
                            totalWeight += fWeight;
                        }
                    }

                    if (totalWeight > g_fEpsilon) {
                        // Compute a better elapsed time
                        dReal newElapsedTime = expectedElapsedTime/totalWeight;
                        // Check elapsed time consistency
                        if (RaveFabs(newElapsedTime) > RampOptimizer::epsilon) {
                            // The new elapsed time is consistent with the data
                            // RAVELOG_VERBOSE_FORMAT("changing the segment elapsed time: %.15e -> %.15e; diff = %.15e", elapsedTime%newElapsedTime%(newElapsedTime - elapsedTime));
                            elapsedTime = newElapsedTime;
                            if (elapsedTime > g_fEpsilon) {
                                dReal iElapsedTime = 1/elapsedTime;
                                for (size_t idof = 0; idof < curvesnd.ndof; ++idof) {
                                    dq1[idof] = 2*iElapsedTime*(q1[idof] - q0[idof]) - dq0[idof];
                                }
                            }
                            else {
                                dq1 = dq0;
                            }
                        }
                    }
                }

                RampOptimizer::CheckReturn retseg = feas->SegmentFeasible2(q0, q1, dq0, dq1, elapsedTime, options, segmentCurvesNDVect);
                if (retseg.retcode != 0) {
                    return retseg;
                }

                if (segmentCurvesNDVect.size() > 0) {
                    if (IS_DEBUGLEVEL(Level_Verbose)) {
                        for (size_t idof = 0; idof < q0.size(); ++idof) {
                            if (RaveFabs(q1[idof] - segmentCurvesNDVect.back().x1Vect[idof]) > RampOptimizer::epsilon) {
                                RAVELOG_VERBOSE_FORMAT("ParabolicCurvesND end point does not finish at the desired position (idof = %d, diff = %.15f)", idof%(q1[idof] - segmentCurvesNDVect.back().x1Vect[idof]));
                            }
                            if (RaveFabs(dq1[idof] - segmentCurvesNDVect.back().v1Vect[idof]) > RampOptimizer::epsilon) {
                                RAVELOG_VERBOSE_FORMAT("ParabolicCurvesND end point does not finish at the desired velocity (idof = %d, diff = %.15f)", idof%(dq1[idof] - segmentCurvesNDVect.back().v1Vect[idof]));
                            }
                        }
                    }

                    // Append segmentCurvesNDVect to curvesndVectOut
                    curvesndVectOut.insert(curvesndVectOut.end(), segmentCurvesNDVect.begin(), segmentCurvesNDVect.end());
                    q0 = curvesndVectOut.back().x1Vect;
                    dq0 = curvesndVectOut.back().v1Vect;
                }
            }

            // Note that q0 and dq0 is actually the final joint values and velocities
            bool bDifferentVelocity = false;
            for (size_t idof = 0; idof < q0.size(); ++idof) {
                if (RaveFabs(curvesnd.x1Vect[idof] - q0[idof]) > RampOptimizer::epsilon) {
                    RAVELOG_VERBOSE_FORMAT("ParabolicCurvesND end point does not finish at the desired position (idof = %d, diff = %.15f), so rejecting", idof%(curvesnd.x1Vect[idof] - q0[idof]));
                    return RampOptimizer::CheckReturn(CFO_FinalValuesNotReached);
                }
                if (RaveFabs(curvesnd.v1Vect[idof] - dq0[idof]) > RampOptimizer::epsilon) {
                    RAVELOG_VERBOSE_FORMAT("ParabolicCurvesND end point does not finish at the desired velocity (idof = %d, diff = %.15f), so reforming", idof%(curvesnd.v1Vect[idof] - dq0[idof]));
                    bDifferentVelocity = true;
                }
            }
            RampOptimizer::CheckReturn finalret(0);
            finalret.bDifferentVelocity = bDifferentVelocity;
            return finalret;
        }

private:
        std::vector<dReal> vswitchtimes;
        std::vector<dReal> q0, q1, dq0, dq1;
        std::vector<uint8_t> _vsearchsegments;
        std::vector<RampOptimizer::ParabolicCurvesND> segmentCurvesNDVect;

    }; // end class MyParabolicCurvesNDFeasibilityChecker


////////////////////////////////////////////////////////////////////////////////////////////////////
public:
    ParabolicSmoother2(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv), _feasibilitychecker(this) {
        __description = "";
        _bmanipconstraints = false;
        _constraintreturn.reset(new ConstraintFilterReturn());
        _logginguniformsampler = RaveCreateSpaceSampler(GetEnv(), "mt19937");
        if (!!_logginguniformsampler) {
            _logginguniformsampler->SetSeed(utils::GetMicroTime());
        }
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params) {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        _parameters->copy(params);
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters) {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        isParameters >> *_parameters;
        return _InitPlan();
    }

    bool _InitPlan() {
        if (_parameters->_nMaxIterations <= 0) {
            _parameters->_nMaxIterations = 100;
        }

        _bUsePerturnation = true;
        _bmanipconstraints = (_parameters->manipname.size() > 0) && (_parameters->maxmanipspeed > 0 || _parameters->maxmanipaccel > 0);

        // Initialize workspace constraints on manipulators
        if (_bmanipconstraints) {
            if (!_manipconstraintchecker) {
                _manipconstraintchecker.reset(new ManipConstraintChecker2(GetEnv()));
            }
            _manipconstraintchecker->Init(_parameters->manipname, _parameters->_configurationspecification, _parameters->maxmanipspeed, _parameters->maxmanipaccel);
        }

        // Initialize a uniform sampler
        if (!_uniformsampler) {
            _uniformsampler = RaveCreateSpaceSampler(GetEnv(), "mt19937");
        }
        _uniformsampler->SetSeed(_parameters->_nRandomGeneratorSeed);

        _fileIndexMod = 10000; // for trajectory saving
        _dumplevel = Level_Verbose;
#ifdef SMOOTHER_TIMING_DEBUG
        // Statistics
        _nCallsCheckManip = 0;
        _totalTimeCheckManip = 0;
        _nCallsInterpolation = 0;
        _totalTimeInterpolation = 0;
#endif
        return !!_uniformsampler;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj) {
        BOOST_ASSERT(!!_parameters && !!ptraj);

        // We need at least two waypoints for retiming.
        if (ptraj->GetNumWaypoints() < 2) {
            return PS_Failed;
        }

        if (IS_DEBUGLEVEL(Level_Verbose)) {
            // Store the trajectory
            uint32_t randNum;
            if (!!_logginguniformsampler) {
                randNum = _logginguniformsampler->SampleSequenceOneUInt32();
            }
            else {
                randNum = RaveRandomInt();
            }
            string filename = str(boost::format("%s/parabolicsmoother2-%d.parameters.xml")%RaveGetHomeDirectory()%(randNum%1000));
            ofstream f(filename.c_str());
            f << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
            f << *_parameters;
            RAVELOG_VERBOSE_FORMAT("ParabolicSmoother2: parameters saved to %s", filename);
        }
        _DumpTrajectory(ptraj, Level_Verbose);

        // Save velocities
        std::vector<KinBody::KinBodyStateSaverPtr> vstatesavers;
        std::vector<KinBodyPtr> vusedbodies;
        _parameters->_configurationspecification.ExtractUsedBodies(GetEnv(), vusedbodies);
        if (vusedbodies.size() == 0) {
            RAVELOG_WARN("There is no used bodies in this configuration\n");
        }
        FOREACH(itbody, vusedbodies) {
            KinBody::KinBodyStateSaverPtr statesaver;
            if ((*itbody)->IsRobot()) {
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

        // Initialize a ParabolicPath
        RampOptimizer::ParabolicPath& parabolicpath = _cacheparabolicpath;
        parabolicpath.Clear();
        OPENRAVE_ASSERT_OP(parameters->_vConfigVelocityLimit.size(), ==, parameters->_vConfigAccelerationLimit.size());
        OPENRAVE_ASSERT_OP((int) parameters->_vConfigVelocityLimit.size(), ==, parameters->GetDOF());
        parabolicpath.Initialize(parameters->_vConfigLowerLimit, parameters->_vConfigUpperLimit, parameters->_vConfigVelocityLimit, parameters->_vConfigAccelerationLimit);
        OPENRAVE_ASSERT_OP((int) parabolicpath.ndof, ==, parameters->GetDOF());

        // Retrieve waypoints
        bool bPathIsPerfectlyModeled = false; // will be true if the initial interpolation is linear or quadratic
        std::vector<dReal> q(_parameters->GetDOF());
        std::vector<dReal>& waypoints = _cacheWaypoints;

        if (_parameters->_hastimestamps && itcompatposgroup->interpolation == "quadratic") {
            RAVELOG_VERBOSE("The initial trajectory is piecewise quadratic");
            // Assume that the traj has velocity data and is consistent. Convert the original trajectory into a ParabolicPath
            ptraj->GetWaypoint(0, _cachex0Vect, posSpec);
            ptraj->GetWaypoint(0, _cachev0Vect, velSpec);

            // size_t iramp = 0;
            for (size_t i = 0; i + 1 < ptraj->GetNumWaypoints(); ++i) {
                ptraj->GetWaypoint(i + 1, _cachetVect, timeSpec);
                if (_cachetVect.at(0) > g_fEpsilonLinear) {
                    ptraj->GetWaypoint(i + 1, _cachex1Vect, posSpec);
                    ptraj->GetWaypoint(i + 1, _cachev1Vect, velSpec);
                    _cacheCurvesND.SetSegment(_cachex0Vect, _cachex1Vect, _cachev0Vect, _cachev1Vect, _cachetVect.at(0));
                    parabolicpath.AppendParabolicCurvesND(_cacheCurvesND);
                    _cachex0Vect.swap(_cachex1Vect);
                    _cachev0Vect.swap(_cachev1Vect);
                }
            }
            bPathIsPerfectlyModeled = true;
        }
        else if (_parameters->_hastimestamps && itcompatposgroup->interpolation == "cubic") {
            RAVELOG_VERBOSE("The initial trajectory is piecewise cubic");
            // Assume that the traj has velocity data and is consistent. Convert the original trajectory into a ParabolicPath
            ptraj->GetWaypoint(0, _cachex0Vect, posSpec);
            ptraj->GetWaypoint(0, _cachev0Vect, velSpec);

            // size_t iramp = 0;
            for (size_t i = 0; i + 1 < ptraj->GetNumWaypoints(); ++i) {
                ptraj->GetWaypoint(i + 1, _cachetVect, timeSpec);
                if (_cachetVect.at(0) > g_fEpsilonLinear) {
                    ptraj->GetWaypoint(i + 1, _cachex1Vect, posSpec);
                    ptraj->GetWaypoint(i + 1, _cachev1Vect, velSpec);

                    dReal iDeltaTime = 1.0/_cachetVect.at(0);
                    dReal iDeltaTime2 = iDeltaTime*iDeltaTime;
                    bool isParabolic = true;
                    for (size_t j = 0; j < _cachex0Vect.size(); ++j) {
                        dReal coeff = (2.0*iDeltaTime*(_cachex0Vect[j] - _cachex1Vect[j]) + _cachev0Vect[j] + _cachev1Vect[j])*iDeltaTime2;
                        if (RaveFabs(coeff) > 1e-5) {
                            isParabolic = false;
                        }
                    }

                    if (isParabolic) {
                        if (!_parameters->verifyinitialpath) {
                            _cacheCurvesND.SetSegment(_cachex0Vect, _cachex1Vect, _cachev0Vect, _cachev1Vect, _cachetVect.at(0));
                            _cacheCurvesND.constraintChecked = true;
                        }
                    }
                    else {
                        // We only check time-based constraints since the path is likely to be modified when shortcutting.
                        if (!_TimeParameterizeZeroVel(_cachex0Vect, _cachex1Vect, CFO_CheckTimeBasedConstraints, _cacheCurvesND)) {
                            RAVELOG_WARN("Failed to initialize from cubic waypoints");
                            _DumpTrajectory(ptraj, Level_Debug);
                            return PS_Failed;
                        }
                    }

                    parabolicpath.AppendParabolicCurvesND(_cacheCurvesND);
                    _cachex0Vect.swap(_cachex1Vect);
                    _cachev0Vect.swap(_cachev1Vect);
                }
            }
        }
        else {
            RAVELOG_VERBOSE_FORMAT("The initial trajectory is %s", itcompatposgroup->interpolation);
            if (itcompatposgroup->interpolation.size() == 0 || itcompatposgroup->interpolation == "linear") {
                bPathIsPerfectlyModeled = true;
            }
            // Interpolation is linear or unknown
            std::vector<std::vector<dReal> > &vWaypoints = _cacheWaypointsVect;
            vWaypoints.resize(0);
            if (vWaypoints.capacity() < ptraj->GetNumWaypoints()) {
                vWaypoints.reserve(ptraj->GetNumWaypoints());
            }

            // waypoints stores concatenated waypoints
            ptraj->GetWaypoints(0, ptraj->GetNumWaypoints(), waypoints, _parameters->_configurationspecification);

            // Iterate through all waypoints and remove the redundant (collinear) ones
            dReal collinearThresh = 1e-14;//RampOptimizer::Sqr(10*RampOptimizer::epsilon);
            for (size_t iwaypoint = 0; iwaypoint < ptraj->GetNumWaypoints(); ++iwaypoint) {
                std::copy(waypoints.begin() + iwaypoint*_parameters->GetDOF(), waypoints.begin() + (iwaypoint + 1)*_parameters->GetDOF(), q.begin());

                if (vWaypoints.size() >= 2) {
                    // Check if the new point is collinear with the previous one.
                    const std::vector<dReal> &x0 = vWaypoints[vWaypoints.size() - 2];
                    const std::vector<dReal> &x1 = vWaypoints[vWaypoints.size() - 1];
                    dReal dotProduct = 0, x0Length2 = 0, x1Length2 = 0;

                    for (size_t idof = 0; idof < q.size(); ++idof) {
                        dReal dx0 = x0[idof] - q[idof];
                        dReal dx1 = x1[idof] - q[idof];
                        dotProduct += dx0*dx1;
                        x0Length2 += dx0*dx0;
                        x1Length2 += dx1*dx1;
                    }
                    if (RaveFabs(dotProduct*dotProduct - x0Length2*x1Length2) < collinearThresh) {
                        // Points are collinear
                        vWaypoints.back() = q;
                        continue;
                    }
                }

                // Check if the point is not the same as the previous one (is this necessary?)
                if (vWaypoints.size() > 0) {
                    dReal d = 0;
                    for (size_t idof = 0; idof < q.size(); ++idof) {
                        d += RaveFabs(q[idof] - vWaypoints.back().at(idof));
                    }
                    if (d <= q.size()*std::numeric_limits<dReal>::epsilon()) {
                        continue;
                    }
                }

                vWaypoints.push_back(q);
            }

            // Time-parameterize the waypoints
            if (!_SetMileStones(vWaypoints, parabolicpath)) {
                RAVELOG_WARN_FORMAT("Failed to initialize from %s waypoints", itcompatposgroup->interpolation);
                _DumpTrajectory(ptraj, Level_Debug);
                return PS_Failed;
            }
            RAVELOG_DEBUG_FORMAT("Finished initializing %s waypoints (via _SetMilestones)", itcompatposgroup->interpolation);
            RAVELOG_DEBUG_FORMAT("numwaypoints: %d -> %d", ptraj->GetNumWaypoints()%vWaypoints.size());
            // _DumpParabolicPath(parabolicpath, Level_Verbose);
        }

        // Do not check constraints again if we already did
        if (!_parameters->verifyinitialpath && bPathIsPerfectlyModeled) {
            // Disable verification
            FOREACH(itcurvesnd, parabolicpath.curvesndVect) {
                itcurvesnd->constraintChecked = true;
            }
        }

        // Main planning loop (shortcutting + converting ParabolicPath to OpenRAVE trajectory)
        try {
            _bUsePerturnation = true;
            RAVELOG_DEBUG_FORMAT("env = %d: Initial ParabolicPath duration = %.15e, pointtolerance = %.15e, manipname = %s, maxmanipspeed = %.15e, maxmanipaccel = %.15e",
                                 GetEnv()->GetId()%parabolicpath.duration%_parameters->_pointtolerance%parameters->manipname%parameters->maxmanipspeed%parameters->maxmanipaccel);
            _feasibilitychecker.tol = parameters->_vConfigResolution;

            FOREACH(it, _feasibilitychecker.tol) {
                *it *= parameters->_pointtolerance;
            }

            _progress._iteration = 0;
            if (_CallCallbacks(_progress) == PA_Interrupt) {
                return PS_Interrupted;
            }

            // Perform shortcutting here
            int numShortcuts = 0;
            if (!!parameters->_setstatevaluesfn || !!parameters->_setstatefn) {
                numShortcuts = _Shortcut(parabolicpath, parameters->_nMaxIterations, this, parameters->_fStepLength*0.99);
                if (numShortcuts < 0) {
                    return PS_Interrupted;
                }
            }

            if (numShortcuts > 0) {
                _DumpTrajectory(ptraj, Level_Verbose);
            }

            ++_progress._iteration;
            if (_CallCallbacks(_progress) == PA_Interrupt) {
                return PS_Interrupted;
            }

            // Finished shortcutting. Start writing the trajectory into OpenRAVE format.
            ConfigurationSpecification newSpec = posSpec;
            newSpec.AddDerivativeGroups(1, true);
            int waypointOffset = newSpec.AddGroup("iswaypoint", 1, "next");

            int timeOffset = -1;
            FOREACH(itgroup, newSpec._vgroups) {
                if (itgroup->name == "deltatime") {
                    timeOffset = itgroup->offset;
                }
                else if (velSpec.FindCompatibleGroup(*itgroup) != velSpec._vgroups.end()) {
                    itgroup->interpolation = "linear";
                }
                else if (posSpec.FindCompatibleGroup(*itgroup) != posSpec._vgroups.end()) {
                    itgroup->interpolation = "quadratic";
                }
            }

            // We are writing the data into _pdummytraj (not ptraj)
            if (!_pdummytraj || (_pdummytraj->GetXMLId() != ptraj->GetXMLId())) {
                _pdummytraj = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
            }
            _pdummytraj->Init(newSpec);

            // Separate all acceleration switch points into individual points
            waypoints.resize(newSpec.GetDOF()); // reuse _cacheWaypoints
            FOREACH(itcurvesnd, parabolicpath.curvesndVect) {
                OPENRAVE_ASSERT_OP((int) itcurvesnd->ndof, ==, _parameters->GetDOF());
            }

            // Insert the first waypoint
            RAVELOG_DEBUG("Start inserting the first waypoint");

            ConfigurationSpecification::ConvertData(waypoints.begin(), newSpec, parabolicpath.curvesndVect.front().x0Vect.begin(), posSpec, 1, GetEnv(), true);
            ConfigurationSpecification::ConvertData(waypoints.begin(), newSpec, parabolicpath.curvesndVect.front().v0Vect.begin(), velSpec, 1, GetEnv(), false);
            waypoints.at(waypointOffset) = 1;
            waypoints.at(timeOffset) = 0;
            _pdummytraj->Insert(_pdummytraj->GetNumWaypoints(), waypoints);

            std::vector<dReal> &vswitchtimes = _cacheSwitchtimes;
            std::vector<dReal> vConfig;

            RampOptimizer::ParabolicCurvesND curvesndTrimmed;
            std::vector<RampOptimizer::ParabolicCurvesND> &tempCurvesNDVect = _cacheCurvesNDVectOut;
            dReal fTrimEdgesTime = parameters->_fStepLength*2; // ignore collisions during [0, fTrimEdgesTime] and [fTrimEdgesTime, duration]
            dReal fExpectedDuration = 0;
            dReal durationDiscrepancyThresh = 0.01;

            for (size_t icurvesnd = 0; icurvesnd < parabolicpath.curvesndVect.size(); ++icurvesnd) {
                RampOptimizer::ParabolicCurvesND &curvesnd = parabolicpath.curvesndVect[icurvesnd];

                tempCurvesNDVect.resize(1);
                tempCurvesNDVect[0] = curvesnd;
                ++_progress._iteration;

                // Check constraints if not yet checked
                if (!curvesnd.constraintChecked) {
                    // Trim out the very first and last segment of the trajectory.
                    curvesndTrimmed = curvesnd;
                    bool bTrimmedFront = false;
                    bool bTrimmedBack = false;
                    bool bCheck = true;
                    RampOptimizer::ParabolicCurvesND frontCurvesND, backCurvesND;

                    if (icurvesnd == 0) {
                        if (curvesnd.duration <= fTrimEdgesTime + g_fEpsilonLinear) {
                            // This initial curvesnd is too short so ignore checking
                            bCheck = false;
                        }
                        else {
                            frontCurvesND = curvesnd;
                            frontCurvesND.Cut(fTrimEdgesTime, curvesndTrimmed);
                            bTrimmedFront = true;
                        }
                    }
                    else if (icurvesnd + 1 == parabolicpath.curvesndVect.size()) {
                        if (curvesnd.duration <= fTrimEdgesTime + g_fEpsilonLinear) {
                            // This final curvesnd is too short so ignore checking
                            bCheck = false;
                        }
                        else {
                            curvesndTrimmed.Cut(curvesnd.duration - fTrimEdgesTime, backCurvesND);
                            bTrimmedBack = true;
                        }
                    }

                    _bUsePerturnation = false;
                    std::vector<RampOptimizer::ParabolicCurvesND> curvesndVectOut;
                    if (bCheck) {
                        RampOptimizer::CheckReturn checkret = _feasibilitychecker.Check2(curvesndTrimmed, 0xffff, curvesndVectOut);
                        // RampOptimizer::CheckReturn checkret = _feasibilitychecker.Check2(curvesndTrimmed, 0xffff, tempCurvesNDVect);

                        if (checkret.retcode != 0) {
                            RAVELOG_DEBUG_FORMAT("env = %d: Check2 for ParabolicCurvesND %d/%d returns 0x%x", GetEnv()->GetId()%icurvesnd%(parabolicpath.curvesndVect.size())%checkret.retcode);

                            bool bSuccess = false;
                            if (1) {
                                // Try to stretch the duration of this ParabolicCurvesND in hopes of
                                // fixing manipulator constraints violation.
                                dReal newDuration = curvesndTrimmed.duration;
                                dReal timeIncrement = 0.05*newDuration;
                                size_t maxTries = 30; // try to go up to about 1.5X duration

                                RampOptimizer::ParabolicCurvesND tempCurvesND;

                                for (size_t iDilate = 0; iDilate < maxTries; ++iDilate) {
#ifdef SMOOTHER_TIMING_DEBUG
                                    _nCallsInterpolation += 1;
                                    _tStartInterpolation = utils::GetMicroTime();
#endif
                                    bool result = RampOptimizer::InterpolateNDFixedDuration(curvesndTrimmed.x0Vect, curvesndTrimmed.x1Vect, curvesndTrimmed.v0Vect, curvesndTrimmed.v1Vect, newDuration, parameters->_vConfigLowerLimit, parameters->_vConfigUpperLimit, parameters->_vConfigVelocityLimit, parameters->_vConfigAccelerationLimit, tempCurvesND);
#ifdef SMOOTHER_TIMING_DEBUG
                                    _tEndInterpolation = utils::GetMicroTime();
                                    _totalTimeInterpolation += 0.000001f*(float)(_tEndInterpolation - _tStartInterpolation);
#endif
                                    if (result) {
                                        // Stretching is successful.

                                        RAVELOG_VERBOSE_FORMAT("prev duration = %.15e; new duration = %.15e", curvesndTrimmed.duration%tempCurvesND.duration);
                                        RampOptimizer::CheckReturn newcurvesndret = _feasibilitychecker.Check2(tempCurvesND, 0xffff, tempCurvesNDVect);
                                        if (newcurvesndret.retcode == 0) {
                                            // The new ParabolicCurvesND passes the test
                                            if (bTrimmedFront) {
                                                tempCurvesNDVect.insert(tempCurvesNDVect.begin(), frontCurvesND);
                                            }
                                            else if (bTrimmedBack) {
                                                tempCurvesNDVect.push_back(backCurvesND);
                                            }
                                            bSuccess = true;
                                            break;
                                        }
                                    }

                                    // If reach here, the interpolation is unsuccessful or constraints checking fails.
                                    if (iDilate > 2) {
                                        newDuration += timeIncrement;
                                    }
                                    else {
                                        newDuration += RampOptimizer::epsilon;
                                    }
                                }

                            }
                            else {
                                // TODO: try different heuristics
                            }

                            if (!bSuccess) {
                                if (IS_DEBUGLEVEL(Level_Verbose)) {
                                    std::stringstream ss;
                                    std::string separator = "";
                                    ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
                                    ss << "x0 = [";
                                    FOREACHC(itvalue, curvesndTrimmed.x0Vect) {
                                        ss << separator << *itvalue;
                                        separator = ", ";
                                    }
                                    ss << "]; x1 = [";
                                    separator = "";
                                    FOREACHC(itvalue, curvesndTrimmed.x1Vect) {
                                        ss << separator << *itvalue;
                                        separator = ", ";
                                    }
                                    ss << "]; v0 = [";
                                    separator = "";
                                    FOREACHC(itvalue, curvesndTrimmed.v0Vect) {
                                        ss << separator << *itvalue;
                                        separator = ", ";
                                    }
                                    ss << "]; v1 = [";
                                    separator = "";
                                    FOREACHC(itvalue, curvesndTrimmed.v1Vect) {
                                        ss << separator << *itvalue;
                                        separator = ", ";
                                    }
                                    ss << "]; deltatime = " << curvesndTrimmed.duration;
                                    RAVELOG_WARN_FORMAT("env = %d: original ParabolicCurvesND %d/%d does not satisfy constraints. retcode = 0x%x. %s", GetEnv()->GetId()%icurvesnd%parabolicpath.curvesndVect.size()%checkret.retcode%ss.str());
                                }
                                else {
                                    RAVELOG_WARN_FORMAT("env = %d: original ParabolicCurvesND %d/%d does not satisfy constraints. retcode = 0x%x", GetEnv()->GetId()%icurvesnd%parabolicpath.curvesndVect.size()%checkret.retcode);
                                }
                                _DumpTrajectory(ptraj, Level_Debug);
                                return PS_Failed;
                            }
                        }
                    }
                    _bUsePerturnation = true; // re-enable
                    ++_progress._iteration;

                    if (_CallCallbacks(_progress) == PA_Interrupt) {
                        return PS_Interrupted;
                    }

                } // finished checking constraints

                // Insert configurations at each switchpoint into OpenRAVE trajectory
                FOREACH(itcurvesnd, tempCurvesNDVect) {
                    fExpectedDuration += itcurvesnd->duration;
                    vswitchtimes.resize(0);
                    if (_parameters->_outputaccelchanges) {
                        vswitchtimes = itcurvesnd->switchpointsList;
                    }
                    else {
                        vswitchtimes.push_back(0);
                        vswitchtimes.push_back(itcurvesnd->duration);
                    }
                    waypoints.resize(newSpec.GetDOF()*(vswitchtimes.size() - 1));

                    std::vector<dReal>::iterator ittargetdata = waypoints.begin();
                    dReal prevTime = 0;
                    // Skip the first switch point at t = 0
                    for (size_t iswitch = 1; iswitch < vswitchtimes.size(); ++iswitch) {
                        itcurvesnd->EvalPos(vswitchtimes[iswitch], vConfig);
                        ConfigurationSpecification::ConvertData(ittargetdata, newSpec, vConfig.begin(), posSpec, 1, GetEnv(), true);
                        itcurvesnd->EvalVel(vswitchtimes[iswitch], vConfig);
                        ConfigurationSpecification::ConvertData(ittargetdata, newSpec, vConfig.begin(), velSpec, 1, GetEnv(), false);

                        *(ittargetdata + timeOffset) = vswitchtimes[iswitch] - prevTime;
                        *(ittargetdata + waypointOffset) = dReal(iswitch + 1 == vswitchtimes.size());
                        ittargetdata += newSpec.GetDOF();
                        prevTime = vswitchtimes[iswitch];
                    }
                    _pdummytraj->Insert(_pdummytraj->GetNumWaypoints(), waypoints);
                }

                if (IS_DEBUGLEVEL(Level_Verbose)) {
                    // If Verbose, do tigher bound checking
                    OPENRAVE_ASSERT_OP(RaveFabs(fExpectedDuration - _pdummytraj->GetDuration()), <, 0.1*durationDiscrepancyThresh);
                }

            } // finished checking every ParabolicCurvesND in parabolicpath

            OPENRAVE_ASSERT_OP(RaveFabs(fExpectedDuration - _pdummytraj->GetDuration()), <, durationDiscrepancyThresh);
            RAVELOG_DEBUG_FORMAT("env = %d: after shortcutting %d times: parabolicpath waypoints = %d; traj waypoints = %d; traj duration = %.15e", GetEnv()->GetId()%numShortcuts%parabolicpath.curvesndVect.size()%_pdummytraj->GetNumWaypoints()%_pdummytraj->GetDuration());

            ptraj->Swap(_pdummytraj);
        }
        catch (const std::exception &ex) {
            _DumpTrajectory(ptraj, Level_Debug);
            RAVELOG_WARN_FORMAT("env = %d: ParabolicSmoother2 failed at iter = %d: %s", GetEnv()->GetId()%_progress._iteration%ex.what());
            return PS_Failed;
        }
        RAVELOG_DEBUG_FORMAT("env = %d: path optimizing - computation time = %f s.", GetEnv()->GetId()%(0.001f*(float)(utils::GetMilliTime() - baseTime)));
        if (IS_DEBUGLEVEL(Level_Debug)) {
            RAVELOG_DEBUG_FORMAT("env = %d: Start sampling the trajectory (verification purpose) after shortcutting", GetEnv()->GetId());
            // Actually _VerifySampling() gets called every time we sample a trajectory. The function
            // already checks _Validate* at every traj point. Therefore, in order to just verify, we
            // need to call ptraj->Sample just once.
            std::vector<dReal> dummy;
            try {
                ptraj->Sample(dummy, 0);
                RAVELOG_DEBUG("env = %d: Sampling for verification successful", GetEnv()->GetId());
            }
            catch (const std::exception& ex) {
                RAVELOG_WARN_FORMAT("env = %d: Sampling for verification failed: %s", GetEnv()->GetId()%ex.what());
                _DumpTrajectory(ptraj, Level_Debug);
            }
            _DumpTrajectory(ptraj, Level_Debug);
        }
#ifdef SMOOTHER_TIMING_DEBUG
        RAVELOG_DEBUG_FORMAT("measured %d interpolations; total exectime = %.15e; time/iter = %.15e", _nCallsInterpolation%_totalTimeInterpolation%(_totalTimeInterpolation/_nCallsInterpolation));
        RAVELOG_DEBUG_FORMAT("measured %d checkmanips; total exectime = %.15e; time/iter = %.15e", _nCallsCheckManip%_totalTimeCheckManip%(_totalTimeCheckManip/_nCallsCheckManip));
#endif
        return _ProcessPostPlanners(RobotBasePtr(), ptraj);
    }

    /// \brief ConfigFeasible checks if the configuration (q0, dq0) is satisfying all constraints
    /// (via CheckPathAllConstraints). Return retcode.
    virtual int ConfigFeasible(const std::vector<dReal> &q0, const std::vector<dReal> &dq0, int options) {
        if (_bUsePerturnation) {
            options |= CFO_CheckWithPerturbation;
        }

        try {
            return _parameters->CheckPathAllConstraints(q0, q0, dq0, dq0, 0, IT_OpenStart, options);
        }
        catch (const std::exception& ex) {
            RAVELOG_WARN_FORMAT("env = %d, rrtparams path constraints threw an exception: %s", GetEnv()->GetId()%ex.what());
            return 0xffff;
        }
    }

    /// \brief ConfigFeasible2 does the same thing as ConfigFeasible. The difference is that it
    /// returns RampOptimizer::CheckReturn. checkret.fTimeBasedSurpassMult is also set to 0.8 if the
    /// configuration (q0, dq0) is not feasible.
    virtual RampOptimizer::CheckReturn ConfigFeasible2(const std::vector<dReal> &q0, const std::vector<dReal> &dq0, int options) {
        if (_bUsePerturnation) {
            options |= CFO_CheckWithPerturbation;
        }

        try {
            int ret = _parameters->CheckPathAllConstraints(q0, q0, dq0, dq0, 0, IT_OpenStart, options);
            RampOptimizer::CheckReturn checkret(ret);
            if (ret == CFO_CheckTimeBasedConstraints) {
                checkret.fTimeBasedSurpassMult = 0.8;
            }
            return checkret;
        }
        catch (const std::exception& ex) {
            RAVELOG_WARN_FORMAT("env = %d, rrtparams path constraints threw an exception: %s", GetEnv()->GetId()%ex.what());
            return 0xffff;
        }
    }

    /// \brief Check if the segment connecting (q0, dq0) and (q1, dq1) is feasible. The function
    /// internally calls CheckPathAllConstraints to do the checking. Initially the input
    /// configurations are connectible with one acceleraiton (therefore, they are consistent when
    /// calling SetSegment). However, if we check manipulator constraints, the path is modified so
    /// as to comply with the constraints. In this case many intermediate waypoints are introduced,
    /// so we check consistency of the set of waypoints introduced as well. In both case, the
    /// verified trajectories are stored in curvesndVectOut.
    virtual RampOptimizer::CheckReturn SegmentFeasible2(const std::vector<dReal> &q0, const std::vector<dReal> &q1, const std::vector<dReal> &dq0, const std::vector<dReal> &dq1, dReal timeElapsed, int options, std::vector<RampOptimizer::ParabolicCurvesND>& curvesndVectOut) {
        size_t ndof = q0.size();

        if (timeElapsed <= g_fEpsilon) {
            curvesndVectOut.resize(1);
            curvesndVectOut[0].SetZeroDuration(q0, dq0);
            return ConfigFeasible2(q0, dq0, options);
        }

        curvesndVectOut.resize(0);
        if (_bUsePerturnation) {
            options |= CFO_CheckWithPerturbation;
        }

        bool bExpectedModifiedConfigurations = (_parameters->fCosManipAngleThresh > -1 + g_fEpsilonLinear);
        if (bExpectedModifiedConfigurations || _bmanipconstraints) {
            options |= CFO_FillCheckedConfiguration;
            _constraintreturn->Clear();
        }

        try {
            int ret = _parameters->CheckPathAllConstraints(q0, q1, dq0, dq1, timeElapsed, IT_OpenStart, options, _constraintreturn);
            if (ret != 0) {
                RampOptimizer::CheckReturn checkret(ret);
                if (ret == CFO_CheckTimeBasedConstraints) {
                    checkret.fTimeBasedSurpassMult = 0.8;
                }
                return checkret;
            }
        }
        catch (const std::exception& ex) {
            RAVELOG_WARN_FORMAT("env = %d: CheckPathAllConstraints threw an exception: %s", GetEnv()->GetId()%ex.what());
            return RampOptimizer::CheckReturn(0xffff);
        }

        // Configurations (between (q0, dq0) and (q1, dq1)) may have been modified inside
        // CheckPathAllConstraints. If that is the case, the path connecting (q0, dq0) and (q1, dq1)
        // is not directly a direct parabolic interpolation anymore.
        if (bExpectedModifiedConfigurations && (_constraintreturn->_configurationtimes.size() > 0)) {
            // There were some modifications from CheckPathAllConstraints. Check consistency of the
            // output of CheckPathAllConstraints
            OPENRAVE_ASSERT_OP(_constraintreturn->_configurations.size(), ==, _constraintreturn->_configurationtimes.size()*ndof);

            std::vector<dReal> curVel = dq0, newVel(ndof);
            std::vector<dReal> curPos = q0, newPos(ndof);

            // _constraintreturn->_configurationtimes[0] is actuall the end of the first segment since interval is IT_OpenStart
            std::vector<dReal>::const_iterator it = _constraintreturn->_configurations.begin();
            dReal curTime = 0;

            for (size_t itime = 0; itime < _constraintreturn->_configurationtimes.size(); ++itime, it += ndof) {
                std::copy(it, it + ndof, newPos.begin());
                dReal deltaTime = _constraintreturn->_configurationtimes[itime] - curTime;
                if (deltaTime > g_fEpsilon) {
                    dReal invDeltaTime = 1.0/deltaTime;
                    // Compute the next velocity for each DOF as well as check consistency
                    for (size_t idof = 0; idof < ndof; ++idof) {
                        newVel[idof] = 2.0*(newPos[idof] - curPos[idof])*invDeltaTime - curVel[idof];
                        // Check the velocity limit
                        if (RaveFabs(newVel[idof]) > _parameters->_vConfigVelocityLimit.at(idof) + RampOptimizer::epsilon) {
                            if (0.9*_parameters->_vConfigVelocityLimit.at(idof) < 0.1*RaveFabs(newVel[idof])) {
                                // Warn if the velocity is really too high
                                RAVELOG_WARN_FORMAT("the new velocity for dof %d is too high. |%.15e| > %.15e", idof%newVel[idof]%_parameters->_vConfigVelocityLimit.at(idof));
                            }
                            // Report the discrepancy
                            RAVELOG_VERBOSE_FORMAT("retcode = 0x4; idof = %d; newvel[%d] = %.15e; vellimit = %.15e; g_fEpsilon = %.15e", idof%idof%newVel[idof]%_parameters->_vConfigVelocityLimit.at(idof)%RampOptimizer::epsilon);
                            return RampOptimizer::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9*_parameters->_vConfigVelocityLimit.at(idof)/RaveFabs(newVel[idof]));
                        }
                    }
                    // The next velocity is ok.
                    _cacheCurvesNDSeg.SetSegment(curPos, newPos, curVel, newVel, deltaTime);

                    // Now check the acceleration.
                    bool bAccelChanged = false;
                    for (size_t idof = 0; idof < ndof; ++idof) {
                        for (size_t jramp = 0; jramp < _cacheCurvesNDSeg.curves[idof].ramps.size(); ++jramp) {
                            if (_cacheCurvesNDSeg.curves[idof].ramps[jramp].a < -_parameters->_vConfigAccelerationLimit[idof]) {
                                _cacheCurvesNDSeg.curves[idof].ramps[jramp].a = -_parameters->_vConfigAccelerationLimit[idof];
                                bAccelChanged = true;
                            }
                            else if (_cacheCurvesNDSeg.curves[idof].ramps[jramp].a > _parameters->_vConfigAccelerationLimit[idof]) {
                                _cacheCurvesNDSeg.curves[idof].ramps[jramp].a = _parameters->_vConfigAccelerationLimit[idof];
                                bAccelChanged = true;
                            }
                        }
                    }
                    if (bAccelChanged) {
                        RampOptimizer::ParabolicCheckReturn parabolicret = RampOptimizer::CheckParabolicCurvesND(_cacheCurvesNDSeg, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, _parameters->_vConfigVelocityLimit, _parameters->_vConfigAccelerationLimit, curPos, newPos, curVel, newVel);
                        if (parabolicret != RampOptimizer::PCR_Normal) {
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

                            RAVELOG_WARN_FORMAT("env = %d: the output ParabolicCuvresND becomes invalid (ret = %x) after fixing accelerations. %s", GetEnv()->GetId()%parabolicret%ss.str());
                            return RampOptimizer::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9);
                        }
                    }
                    _cacheCurvesNDSeg.constraintChecked = true;

                    curvesndVectOut.push_back(_cacheCurvesNDSeg);
                    curTime = _constraintreturn->_configurationtimes[itime];
                    curPos.swap(newPos);
                    curVel.swap(newVel);
                }
            }

            // Since the configurations are modified, we need to make sure that the last
            // configuration ends at the desired values.
            for (size_t idof = 0; idof < ndof; ++idof) {
                if (RaveFabs(curPos[idof] - q1[idof]) + g_fEpsilon > RampOptimizer::epsilon) {
                    RAVELOG_WARN_FORMAT("env = %d: discrepancy at the last configuration: curPos[%d] (%.15e) != q1[%d] (%.15e)", GetEnv()->GetId()%idof%curPos[idof]%idof%q1[idof]);
                    return RampOptimizer::CheckReturn(CFO_StateSettingError);
                }
            }

            // How about the last velocity?
        }

        if (curvesndVectOut.size() == 0) {
            // No previous modification.
            _cacheCurvesNDSeg.SetSegment(q0, q1, dq0, dq1, timeElapsed);
            _cacheCurvesNDSeg.constraintChecked = true;
            curvesndVectOut.push_back(_cacheCurvesNDSeg);
        }

        if (_bmanipconstraints && (options & CFO_CheckTimeBasedConstraints)) {
            try {
#ifdef SMOOTHER_TIMING_DEBUG
                _nCallsCheckManip += 1;
                _tStartCheckManip = utils::GetMicroTime();
#endif
                RampOptimizer::CheckReturn retmanip = _manipconstraintchecker->CheckManipConstraints3(curvesndVectOut);
#ifdef SMOOTHER_TIMING_DEBUG
                _tEndCheckManip = utils::GetMicroTime();
                _totalTimeCheckManip += 0.000001f*(float)(_tEndCheckManip - _tStartCheckManip);
#endif
                if (retmanip.retcode != 0) {
                    RAVELOG_VERBOSE_FORMAT("CheckManipConstraints2 returns retcode = %d", retmanip.retcode);
                    return retmanip;
                }
            }
            catch (const std::exception& ex) {
                RAVELOG_VERBOSE_FORMAT("CheckManipConstraints2 (modified = %d) threw an exception: %s", ((int) bExpectedModifiedConfigurations)%ex.what());
                return 0xffff;
            }
        }

        return RampOptimizer::CheckReturn(0);
    }

    virtual dReal Rand() {
        return _uniformsampler->SampleSequenceOneReal(IT_OpenEnd);
    }

    virtual bool NeedDerivativeForFeasibility() {
        return true;
    }

protected:
    /// \brief Time-parameterize the ordered set of waypoints to a trajectory that stops at every
    /// waypoint. _SetMilestones also adds some extra waypoints to the original set if any two
    /// consecutive waypoints are too far apart.
    bool _SetMileStones(const std::vector<std::vector<dReal> > &vWaypoints, RampOptimizer::ParabolicPath &parabolicpath) {
        _zeroVelPoints.resize(0);
        if (_zeroVelPoints.capacity() < vWaypoints.size()) {
            _zeroVelPoints.reserve(vWaypoints.size());
        }

        size_t ndof = _parameters->GetDOF();
        parabolicpath.Clear();

        RAVELOG_VERBOSE_FORMAT("env = %d: initial num waypoints = %d", GetEnv()->GetId()%vWaypoints.size());

        if (vWaypoints.size() == 1) {
            RampOptimizer::ParabolicCurvesND &curvesnd = _cacheCurvesND1;
            curvesnd.SetConstant(vWaypoints[0]);
            parabolicpath.AppendParabolicCurvesND(curvesnd);
        }
        else if (vWaypoints.size() > 1) {
            int options = CFO_CheckTimeBasedConstraints;
            // options = options|CFO_FillCheckedConfiguration;
            if (!_parameters->verifyinitialpath) {
                options = options & (~CFO_CheckEnvCollisions) & (~CFO_CheckSelfCollisions);
                RAVELOG_VERBOSE_FORMAT("env = %d: Initial path verification is disabled using options = 0x%x", GetEnv()->GetId()%options);
            }

            std::vector<dReal> vZero(ndof, 0); // zero vector

            // In many cases when there are manipulator constraints, the midpoint 0.5*(x0 + x1) will
            // not satisfy the constraints. Instead of returning failure, we try to recompute a
            // better midpoint.
            std::vector<std::vector<dReal> > &vNewWaypoints = _cacheNewWaypointsVect;
            std::vector<uint8_t> vForceInitialChecking(vWaypoints.size(), 0);

            if (!!_parameters->_neighstatefn) {
                std::vector<dReal> xmid(ndof), xmidDelta(ndof);
                vNewWaypoints = vWaypoints;
                dReal distThresh = 0.00001;
                int nConsecutiveExpansionsAllowed = 10;

                int nConsecutiveExpansions = 0; // too many consecutive expansion is not good. If that occurs, we return failure.
                size_t iwaypoint = 0;
                while (iwaypoint + 1 < vNewWaypoints.size()) {
                    for (size_t idof = 0; idof < ndof; ++idof) {
                        xmidDelta[idof] = 0.5*(vNewWaypoints[iwaypoint + 1][idof] - vNewWaypoints[iwaypoint][idof]);
                    }

                    xmid = vNewWaypoints[iwaypoint];
                    if (_parameters->SetStateValues(xmid) != 0) {
                        RAVELOG_WARN_FORMAT("env = %d: Could not set values at waypoint %d/%d", GetEnv()->GetId()%iwaypoint%(vNewWaypoints.size()));
                        return false;
                    }

                    if (!_parameters->_neighstatefn(xmid, xmidDelta, NSO_OnlyHardConstraints)) {
                        RAVELOG_WARN_FORMAT("env = %d: Failed to get the neightbor of waypoint %d/%d", GetEnv()->GetId()%iwaypoint%(vNewWaypoints.size()));
                        return false;
                    }

                    // Check if xmid (computed from _neightstatefn) is far from what we expect
                    dReal dist = 0;
                    for (size_t idof = 0; idof < ndof; ++idof) {
                        dReal fExpected = 0.5*(vNewWaypoints[iwaypoint + 1][idof] + vNewWaypoints[iwaypoint][idof]);
                        dReal fError = fExpected - xmid[idof];
                        dist += fError*fError;
                    }
                    if (dist > distThresh) {
                        RAVELOG_DEBUG_FORMAT("env = %d: Adding extra midpoint between waypoints %d and %d since dist = %.15e", GetEnv()->GetId()%iwaypoint%(iwaypoint + 1)%dist);
                        OPENRAVE_ASSERT_OP(xmid.size(), ==, ndof);
                        vNewWaypoints.insert(vNewWaypoints.begin() + iwaypoint + 1, xmid);
                        vForceInitialChecking[iwaypoint + 1] = 1;
                        vForceInitialChecking.insert(vForceInitialChecking.begin() + iwaypoint + 1, 1);
                        nConsecutiveExpansions += 2;
                        if (nConsecutiveExpansions > nConsecutiveExpansionsAllowed) {
                            RAVELOG_WARN_FORMAT("env = %d: Too many consecutive expansions, waypoint %d/%d is bad", GetEnv()->GetId()%iwaypoint%(vNewWaypoints.size()));
                            return false;
                        }
                        continue;
                    }
                    if (nConsecutiveExpansions > 0) {
                        nConsecutiveExpansions--;
                    }

                    iwaypoint += 1;
                }
            }
            else {
                // No _neightstatefn. Do nothing.
                vNewWaypoints = vWaypoints;
            }
            // Finished preparation of waypoints. Now continue to time-parameterize the path.

            RampOptimizer::ParabolicCurvesND &curvesnd = _cacheCurvesND1;
            OPENRAVE_ASSERT_OP(vNewWaypoints[0].size(), ==, ndof);
            for (size_t iwaypoint = 1; iwaypoint < vNewWaypoints.size(); ++iwaypoint) {
                OPENRAVE_ASSERT_OP(vNewWaypoints[iwaypoint].size(), ==, ndof);

                if (!_TimeParameterizeZeroVel(vNewWaypoints[iwaypoint - 1], vNewWaypoints[iwaypoint], options, curvesnd)) {
                    return false;
                }

                if (!_parameters->verifyinitialpath && !vForceInitialChecking.at(iwaypoint)) {
                    curvesnd.constraintChecked = true;
                }

                // Keep track of zero-velocity waypoints
                if (_zeroVelPoints.size() == 0) {
                    _zeroVelPoints.push_back(curvesnd.duration);
                }
                else {
                    _zeroVelPoints.push_back(_zeroVelPoints.back() + curvesnd.duration);
                }

                // Store the computed ParabolicCurvesND in the ParabolicPath
                parabolicpath.AppendParabolicCurvesND(curvesnd);
            }
            _zeroVelPoints.pop_back(); // zeroVelPoints contains every zero-velocity waypoints except the start and the end.
        }

        return true;
    }

    /// \brief _TimeParameterizeZeroVel does interpolation with zero terminal velocities. It also
    /// checks manipulator constraints and lower the velocity and acceleration limits accordingly if
    /// the constraints are violated. Therefore, the trajectory in curvesndOut is guaranteed to
    /// satisfy all constraints.
    bool _TimeParameterizeZeroVel(const std::vector<dReal> &x0VectIn, const std::vector<dReal> &x1VectIn, int options, RampOptimizer::ParabolicCurvesND &curvesndOut) {
        std::vector<dReal> &x0Vect = _cachex0Vect2, &x1Vect = _cachex1Vect2, &v0Vect = _cachev0Vect2, &v1Vect = _cachev1Vect2;
        std::vector<dReal> &vellimits = _cachevellimits, &accellimits = _cacheaccellimits;
        vellimits = _parameters->_vConfigVelocityLimit;
        accellimits = _parameters->_vConfigAccelerationLimit;

        RampOptimizer::CheckReturn retseg(0);
        size_t numTries = 30;
        for (size_t itry = 0; itry < numTries; ++itry) {
            bool res = RampOptimizer::InterpolateZeroVelND(x0VectIn, x1VectIn, vellimits, accellimits, curvesndOut);
            RAMP_OPTIM_ASSERT(res);

            x0Vect = curvesndOut.x0Vect;
            v0Vect = curvesndOut.v0Vect;

            size_t iswitch = 1;
            size_t nswitches = curvesndOut.switchpointsList.size(); // switchpointsList includes zero.
            for (iswitch = 1; iswitch < nswitches; ++iswitch) {
                curvesndOut.EvalPos(curvesndOut.switchpointsList[iswitch], x1Vect);
                curvesndOut.EvalVel(curvesndOut.switchpointsList[iswitch], v1Vect);

                // Check the segment between the two switch points
                retseg = SegmentFeasible2(x0Vect, x1Vect, v0Vect, v1Vect, curvesndOut.switchpointsList[iswitch] - curvesndOut.switchpointsList[iswitch - 1], options, _cacheCurvesNDVect1);

                if (retseg.retcode != 0) {
                    break;
                }
                if (retseg.bDifferentVelocity) {
                    RAVELOG_WARN("SegmentFeasible2 sets bDifferentVelocity to true");
                    retseg.retcode = CFO_FinalValuesNotReached;
                    break;
                }
                x0Vect.swap(x1Vect);
                v0Vect.swap(v1Vect);
            }
            if (retseg.retcode == 0) {
                // Everything is fine. Stop iterating.
                break;
            }
            else if (retseg.retcode == CFO_CheckTimeBasedConstraints) {
                // The manipulator is probably moving too fast. Suppress the velocity and
                // acceleration limits and try again.
                RAVELOG_VERBOSE_FORMAT("env = %d: slowing down by %.15e since it is too fast, itry = %d", GetEnv()->GetId()%retseg.fTimeBasedSurpassMult%itry);
                for (size_t j = 0; j < vellimits.size(); ++j) {
                    vellimits[j] *= retseg.fTimeBasedSurpassMult;
                    accellimits[j] *= retseg.fTimeBasedSurpassMult;
                }
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
                ss << "]; deltatime=" << (curvesndOut.switchpointsList[iswitch] - curvesndOut.switchpointsList[iswitch - 1]);
                RAVELOG_WARN_FORMAT("Calling SegmentFeasibile2 between switchpoints %d and %d returned error 0x%x; %s giving up....", (iswitch - 1)%iswitch%retseg.retcode%ss.str());
                return false;
            }
        }
        if (retseg.retcode != 0) {
            return false;
        }
        return true;
    }

    int _Shortcut(RampOptimizer::ParabolicPath &parabolicpath, int numIters, RampOptimizer::RandomNumberGeneratorBase* rng, dReal minTimeStep) {

        uint32_t fileindex; // we save trajectory instances before and after shortcutting using the same index
        if (!!_logginguniformsampler) {
            fileindex = _logginguniformsampler->SampleSequenceOneUInt32();
        }
        else {
            fileindex = RaveRandomInt();
        }
        fileindex = fileindex%_fileIndexMod;
        _DumpParabolicPath(parabolicpath, _dumplevel, fileindex, 0); // save the trajectory before shortcutting (option = 0)

        std::vector<RampOptimizer::ParabolicCurvesND> &curvesndVect = parabolicpath.curvesndVect; // just for convenience

        // Caching stuff
        RampOptimizer::ParabolicCurvesND &shortcutCurvesND1 = _cacheCurvesND1, &shortcutCurvesND2 = _cacheCurvesND2;
        std::vector<RampOptimizer::ParabolicCurvesND> &shortcutCurvesNDVect1 = _cacheCurvesNDVect1, &shortcutCurvesNDVect2 = _cacheCurvesNDVect2;
        shortcutCurvesNDVect1.resize(0);
        shortcutCurvesNDVect2.resize(0);
        // RampOptimizer::ParabolicPath &shortcutpath = _cacheShortcutPath;
        // shortcutpath.Initialize(parabolicpath.xminVect, parabolicpath.xmaxVect, parabolicpath.vmVect, parabolicpath.amVect);
        // shortcutpath.Clear();

        int numShortcuts = 0;
        std::vector<dReal> x0, x1, v0, v1;
        const dReal tOriginal = parabolicpath.duration;
        dReal tTotal = parabolicpath.duration; // keeps track of the total duration of the shortcut trajectory

        std::vector<dReal> &vellimits = _cachevellimits, &accellimits = _cacheaccellimits;
        vellimits.resize(_parameters->_vConfigVelocityLimit.size());
        accellimits.resize(_parameters->_vConfigAccelerationLimit.size());

        int numSlowDowns = 0; // counts the number of times we slow down the trajectory (via vel/accel scaling) because of manip constraints
        dReal fiSearchVelAccelMult = 1.0/_parameters->fSearchVelAccelMult; // magic constant
        dReal fStartTimeVelMult = 1.0; // this is the multiplier for scaling down the *initial* velocity in each shortcut iteration. If manip constraints
                                       // or dynamic constraints are used, then this will track the most recent successful multiplier. The idea is that if the
                                       // recent successful multiplier is some low value, say 0.1, it is unlikely that using the full vel/accel limits, i.e.,
                                       // multiplier = 1.0, will succeed the next time
        dReal fStartTimeAccelMult = 1.0;

        // Parameters & variables for early shortcut termination
        size_t nItersFromPrevSuccessful = 0; // keeps track of the most recent successful shortcut iteration
        size_t nCutoffIters = 100;           // we stop shortcutting if no progress has been made in the past nCutoffIters iterations

        dReal score = 1.0;                   // if the current iteration is successful, we calculate a score
        dReal currentBestScore = 1.0;        // keeps track of the best shortcut score so far
        dReal iCurrentBestScore = 1.0;
        dReal cutoffRatio = 1e-3;            // we stop shortcutting if the progress made is considered too little (score/currentBestScore < cutoffRatio)

        dReal specialShortcutWeight = 0.1;      // if the sampled number is less than this weight, we sample t0 and t1 around a zerovelpoint
                                                // (instead of randomly sample in the whole range) to try to shortcut and remove it.
        dReal specialShortcutCutoffTime = 0.75;

        int iters = 0;
        for (iters = 0; iters < numIters; ++iters) {
            if (tTotal < minTimeStep) {
                RAVELOG_VERBOSE_FORMAT("env = %d; tTotal = %.15e is too short to continue shortcutting", GetEnv()->GetId()%tTotal);
                break;
            }

            // Sample t0 and t1. We could possibly add some heuristics here to get higher quality
            // shortcuts
            dReal t0, t1;
            if (iters == 0) {
                t0 = 0;
                t1 = tTotal;
            }
            else if ( (_zeroVelPoints.size() > 0 && rng->Rand() <= specialShortcutWeight) || (numIters - iters <= (int)_zeroVelPoints.size()) ) {
                /* We consider shortcutting around a zerovelpoint (the time instant of an original
                   waypoint which has not yet been shortcut) when there are some zerovelpoints left
                   and either
                   - the random number falls below the threshold, or
                   - there are not so many shortcut iterations left (compared to the number of zerovelpoints)
                 */
                size_t index = _uniformsampler->SampleSequenceOneUInt32()%_zeroVelPoints.size();
                dReal t = _zeroVelPoints[index];
                t0 = t - rng->Rand()*min(specialShortcutCutoffTime, t);
                t1 = t + rng->Rand()*min(specialShortcutCutoffTime, tTotal - t);
            }
            else {
                // Proceed normally
                t0 = rng->Rand()*tTotal;
                t1 = rng->Rand()*tTotal;
                if (t0 > t1) {
                    RampOptimizer::Swap(t0, t1);
                }
            }
            if (t1 - t0 < minTimeStep) {
                // The sampled t0 and t1 are too close to be useful.
                continue;
            }

            // Locate the ParabolicCurvesNDs where t0 and t1 fall in
            int i0 = std::upper_bound(parabolicpath.mainSwitchpoints.begin(), parabolicpath.mainSwitchpoints.end() - 1, t0) - parabolicpath.mainSwitchpoints.begin() - 1;
            int i1 = std::upper_bound(parabolicpath.mainSwitchpoints.begin(), parabolicpath.mainSwitchpoints.end() - 1, t1) - parabolicpath.mainSwitchpoints.begin() - 1;

            uint32_t iIterProgress = 0; // used for debugging purposes
            // Perform shortcutting
            try {
                dReal u0 = t0 - parabolicpath.mainSwitchpoints.at(i0);
                dReal u1 = t1 - parabolicpath.mainSwitchpoints.at(i1);
                OPENRAVE_ASSERT_OP(u0, >=, 0);
                OPENRAVE_ASSERT_OP(u0, <=, curvesndVect[i0].duration + RampOptimizer::epsilon);
                OPENRAVE_ASSERT_OP(u1, >=, 0);
                OPENRAVE_ASSERT_OP(u1, <=, curvesndVect[i1].duration + RampOptimizer::epsilon);

                u0 = RampOptimizer::Min(u0, curvesndVect[i0].duration);
                u1 = RampOptimizer::Min(u1, curvesndVect[i1].duration);

                curvesndVect[i0].EvalPos(u0, x0);
                if (_parameters->SetStateValues(x0) != 0) {
                    continue;
                }
                iIterProgress +=  0x10000000;
                _parameters->_getstatefn(x0);
                iIterProgress += 0x10000000;

                curvesndVect[i1].EvalPos(u1, x1);
                if (_parameters->SetStateValues(x1) != 0) {
                    continue;
                }
                iIterProgress +=  0x10000000;
                _parameters->_getstatefn(x1);

                curvesndVect[i0].EvalVel(u0, v0);
                curvesndVect[i1].EvalVel(u1, v1);
                ++_progress._iteration;

                vellimits = _parameters->_vConfigVelocityLimit;
                accellimits = _parameters->_vConfigAccelerationLimit;

                if (0) {
                    // Modify vellimits and accellimits according to manipulator constraints (if any)
                    if (_bmanipconstraints && !!_manipconstraintchecker) {
                        if (_parameters->SetStateValues(x0) != 0) {
                            RAVELOG_VERBOSE("SetStateValues at x0 error\n");
                            continue;
                        }
                        _manipconstraintchecker->GetMaxVelocitiesAccelerations(v0, vellimits, accellimits);

                        if (_parameters->SetStateValues(x1) != 0) {
                            RAVELOG_VERBOSE("SetStateValues at x1 error\n");
                            continue;
                        }
                        _manipconstraintchecker->GetMaxVelocitiesAccelerations(v1, vellimits, accellimits);
                    }
                }

                for (size_t j = 0; j < _parameters->_vConfigVelocityLimit.size(); ++j) {
                    // Adjust vellimits and accellimits
                    dReal fminvel = max(RaveFabs(v0[j]), RaveFabs(v1[j]));
                    if (vellimits[j] < fminvel) {
                        vellimits[j] = fminvel;
                    }
                    else {
                        dReal f = max(fminvel, fStartTimeVelMult * _parameters->_vConfigVelocityLimit[j]);
                        if (vellimits[j] > f) {
                            vellimits[j] = f;
                        }
                    }
                    {
                        dReal f = fStartTimeAccelMult * _parameters->_vConfigAccelerationLimit[j];
                        if (accellimits[j] > f) {
                            accellimits[j] = f;
                        }
                    }
                }

                dReal fCurVelMult = fStartTimeVelMult;
                dReal fCurAccelMult = fStartTimeAccelMult;
                RAVELOG_VERBOSE_FORMAT("env = %d: iter = %d/%d, start shortcutting from t0 = %.15e to t1 = %.15e", GetEnv()->GetId()%iters%numIters%t0%t1);

                bool bSuccess = false;
                size_t maxSlowDownTries = 4;
                for (size_t iSlowDown = 0; iSlowDown < maxSlowDownTries; ++iSlowDown) {
#ifdef SMOOTHER_TIMING_DEBUG
                    _nCallsInterpolation += 1;
                    _tStartInterpolation = utils::GetMicroTime();
#endif
                    bool res = RampOptimizer::InterpolateArbitraryVelND(x0, x1, v0, v1, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, vellimits, accellimits, shortcutCurvesND1, false);
#ifdef SMOOTHER_TIMING_DEBUG
                    _tEndInterpolation = utils::GetMicroTime();
                    _totalTimeInterpolation += 0.000001f*(float)(_tEndInterpolation - _tStartInterpolation);
#endif
                    iIterProgress += 0x1000;
                    if (!res) {
                        RAVELOG_DEBUG_FORMAT("env = %d: shortcut iter = %d/%d, initial interpolation failed.\n", GetEnv()->GetId()%iters%numIters);
                        break;
                    }

                    // Check if shortcutcurvesnd makes a significant improvement
                    if (shortcutCurvesND1.duration + minTimeStep > t1 - t0) {
                        RAVELOG_DEBUG_FORMAT("env = %d: shortcut iter = %d/%d, rejecting shortcut from t0 = %.15e to t1 = %.15e, %.15e > %.15e, minTimeStep = %.15e, final trajectory duration = %.15e s.\n",
                                             GetEnv()->GetId()%iters%numIters%t0%t1%shortcutCurvesND1.duration%(t1 - t0)%minTimeStep%(parabolicpath.duration));
                        break;
                    }

                    if (_CallCallbacks(_progress) == PA_Interrupt) {
                        return -1;
                    }
                    iIterProgress += 0x1000;

                    RampOptimizer::CheckReturn retcheck(0);
                    iIterProgress += 0x10;

                    do { // Start checking constraints. shortcutpath.curvesndVect will be filled in here.
                        if (_parameters->SetStateValues(shortcutCurvesND1.x1Vect) != 0) {
                            std::string x1string;
                            VectorToString(shortcutCurvesND1.x1Vect, x1string, "x1Vect");
                            RAVELOG_DEBUG_FORMAT("env = %d: shortcut iter = %d/%d, cannot set state: %s.\n", GetEnv()->GetId()%iters%numIters%x1string);
                            retcheck.retcode = CFO_StateSettingError;
                            break;
                        }
                        _parameters->_getstatefn(shortcutCurvesND1.x1Vect);
                        iIterProgress += 0x10;

                        retcheck = _feasibilitychecker.Check2(shortcutCurvesND1, 0xffff, shortcutCurvesNDVect1);
                        iIterProgress += 0x10;

                        if (retcheck.retcode != 0) {
                            // shortcut does not pass DynamicsCollisionConstraint::Check
                            RAVELOG_DEBUG_FORMAT("env = %d: shortcut iter = %d/%d, iSlowDown = %d, shortcut does not pass Check2, retcode = 0x%x.\n", GetEnv()->GetId()%iters%numIters%iSlowDown%retcheck.retcode);
                            break;
                        }

                        // CheckPathAllConstraints (called viaSegmentFeasible2 inside Check2) may be
                        // modifying the original shortcutcurvesnd due to constraints. Therefore, we
                        // have to reset vellimits and accellimits so that they are above those of
                        // the modified trajectory.
                        for (size_t icurvesnd = 0; icurvesnd + 1 < shortcutCurvesNDVect1.size(); ++icurvesnd) {
                            for (size_t jdof = 0; jdof < shortcutCurvesNDVect1[icurvesnd].ndof; ++jdof) {
                                dReal fminvel = max(RaveFabs(shortcutCurvesNDVect1[icurvesnd].v0Vect[jdof]), RaveFabs(shortcutCurvesNDVect1[icurvesnd].v1Vect[jdof]));
                                if (vellimits[jdof] < fminvel) {
                                    vellimits[jdof] = fminvel;
                                }
                            }
                        }

                        // Check consistency
                        if (IS_DEBUGLEVEL(Level_Verbose)) {
                            for (size_t icurvesnd = 0; icurvesnd + 1 < shortcutCurvesNDVect1.size(); ++icurvesnd) {
                                for (size_t jdof = 0; jdof < shortcutCurvesNDVect1[icurvesnd].ndof; ++jdof) {
                                    OPENRAVE_ASSERT_OP(RaveFabs(shortcutCurvesNDVect1[icurvesnd].x1Vect[jdof] - shortcutCurvesNDVect1[icurvesnd + 1].x0Vect[jdof]), <=, RampOptimizer::epsilon);
                                    OPENRAVE_ASSERT_OP(RaveFabs(shortcutCurvesNDVect1[icurvesnd].v1Vect[jdof] - shortcutCurvesNDVect1[icurvesnd + 1].v0Vect[jdof]), <=, RampOptimizer::epsilon);
                                }
                            }
                        }

                        // The interpolated segment passes constraints checking. Now see if it is modified such that it ends with different velocity.
                        if (retcheck.bDifferentVelocity && shortcutCurvesNDVect1.size() > 0) {
                            RAVELOG_VERBOSE_FORMAT("env = %d: new shortcut is *not* aligned with boundary values after running Check2. Start fixing the last segment.", GetEnv()->GetId());
                            // Modification inside Check2 results in the shortcut trajectory not ending at the desired velocity v1.
                            dReal allowedStretchTime = (t1 - t0) - (shortcutCurvesND1.duration + minTimeStep); // the time that this segment is allowed to stretch out such that it is still a useful shortcut
#ifdef SMOOTHER_TIMING_DEBUG
                            _nCallsInterpolation += 1;
                            _tStartInterpolation = utils::GetMicroTime();
#endif
                            bool res = RampOptimizer::InterpolateArbitraryVelND(shortcutCurvesNDVect1.back().x0Vect, x1, shortcutCurvesNDVect1.back().v0Vect, v1, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, vellimits, accellimits, shortcutCurvesND2, true);
#ifdef SMOOTHER_TIMING_DEBUG
                            _tEndInterpolation = utils::GetMicroTime();
                            _totalTimeInterpolation += 0.000001f*(float)(_tEndInterpolation - _tStartInterpolation);
#endif
                            if (!res) {
                                // This may be because we cannot the fix joint limit violation.
                                RAVELOG_WARN_FORMAT("env = %d: failed to InterpolateArbitraryVelND to correct the final velocity", GetEnv()->GetId());
                                retcheck.retcode = CFO_FinalValuesNotReached;
                                break;
                            }
                            if (RaveFabs(shortcutCurvesND2.duration - shortcutCurvesNDVect1.back().duration) > allowedStretchTime) {
                                RAVELOG_DEBUG_FORMAT("env = %d: shortcutCurvesND2 duration is too long to be useful(%.15e s.)", GetEnv()->GetId()%shortcutCurvesND2.duration);
                                retcheck.retcode = CFO_FinalValuesNotReached;
                                break;
                            }

                            retcheck = _feasibilitychecker.Check2(shortcutCurvesND2, 0xffff, shortcutCurvesNDVect2);
                            if (retcheck.retcode != 0) {
                                RAVELOG_VERBOSE_FORMAT("env = %d: final segment fixing failed. retcode = 0x%x", GetEnv()->GetId()%retcheck.retcode);
                            }
                            else if (retcheck.bDifferentVelocity) {
                                RAVELOG_WARN_FORMAT("env = %d: after final segment fixing, shortcutCurvesND2 does not end at the desired velocity", GetEnv()->GetId());
                                retcheck.retcode = CFO_FinalValuesNotReached;
                                break;
                            }
                            else {
                                // Otherwise, this segment is good.
                                RAVELOG_VERBOSE_FORMAT("env = %d: final velocity correction for the last segment successful", GetEnv()->GetId());
                                shortcutCurvesNDVect1.pop_back();
                                // Append the checked curve to the end
                                shortcutCurvesNDVect1.insert(shortcutCurvesNDVect1.end(), shortcutCurvesNDVect2.begin(), shortcutCurvesNDVect2.end());
                                // Another consistency checking
                                if (IS_DEBUGLEVEL(Level_Verbose)) {
                                    for (size_t icurvesnd = 0; icurvesnd + 1 < shortcutCurvesNDVect1.size(); ++icurvesnd) {
                                        for (size_t jdof = 0; jdof < shortcutCurvesNDVect1[icurvesnd].ndof; ++jdof) {
                                            OPENRAVE_ASSERT_OP(RaveFabs(shortcutCurvesNDVect1[icurvesnd].x1Vect[jdof] - shortcutCurvesNDVect1[icurvesnd + 1].x0Vect[jdof]), <=, RampOptimizer::epsilon);
                                            OPENRAVE_ASSERT_OP(RaveFabs(shortcutCurvesNDVect1[icurvesnd].v1Vect[jdof] - shortcutCurvesNDVect1[icurvesnd + 1].v0Vect[jdof]), <=, RampOptimizer::epsilon);
                                        }
                                    }
                                }
                                break;
                            }
                        }
                        else {
                            RAVELOG_VERBOSE_FORMAT("env = %d: new shortcut is aligned with boundary values after running Check2", GetEnv()->GetId());
                            break;
                        }
                    } while (0);
                    // Finished checking constraints. Now see what retcheck.retcode is
                    iIterProgress += 0x1000;

                    if (retcheck.retcode == 0) {
                        // Shortcut is successful
                        bSuccess = true;
                        break;
                    }
                    else if (retcheck.retcode == CFO_CheckTimeBasedConstraints) {
                        // CFO_CheckTimeBasedConstraints can be returned because of two things: torque limit violation and manip constraint violation

                        // Scale down vellimits and/or accellimits
                        if (_bmanipconstraints && _manipconstraintchecker) {
                            // Scale down vellimits and accellimits independently according to the violated constraint (manipspeed/manipaccel)
                            if (iSlowDown == 0) {
                                // Try computing estimates of vellimits and accellimits before scaling down
                                if (_parameters->SetStateValues(x0) != 0) {
                                    RAVELOG_VERBOSE("state setting error");
                                    break;
                                }
                                _manipconstraintchecker->GetMaxVelocitiesAccelerations(v0, vellimits, accellimits);

                                if (_parameters->SetStateValues(x1) != 0) {
                                    RAVELOG_VERBOSE("state setting error");
                                    break;
                                }
                                _manipconstraintchecker->GetMaxVelocitiesAccelerations(v1, vellimits, accellimits);

                                for (size_t j = 0; j < _parameters->_vConfigVelocityLimit.size(); ++j) {
                                    dReal fMinVel = max(RaveFabs(v0[j]), RaveFabs(v1[j]));
                                    if (vellimits[j] < fMinVel) {
                                        vellimits[j] = fMinVel;
                                    }
                                }
                            }
                            else {
                                // After computing the new vellimits and accellimits and they don't work, we gradually scale vellimits/accellimits down
                                dReal fVelMult, fAccelMult;
                                if (retcheck.fMaxManipSpeed > _parameters->maxmanipspeed) {
                                    // Manipspeed is violated. We don't scale down accellimts.
                                    fVelMult = retcheck.fTimeBasedSurpassMult;
                                    fCurVelMult *= fVelMult;
                                    if (fCurVelMult < 0.01) {
                                        RAVELOG_VERBOSE_FORMAT("env = %d: shortcut iter = %d/%d: fCurVelMult is too small (%.15e). continue to the next iteration", GetEnv()->GetId()%iters%numIters%fCurVelMult);
                                        break;
                                    }
                                    for (size_t j = 0; j < vellimits.size(); ++j) {
                                        dReal fMinVel = max(RaveFabs(v0[j]), RaveFabs(v1[j]));
                                        vellimits[j] = max(fMinVel, fVelMult * vellimits[j]);
                                    }
                                }

                                if (retcheck.fMaxManipAccel > _parameters->maxmanipaccel) {
                                    // Manipaccel is violated. We scale both vellimits and accellimits down.
                                    fAccelMult = retcheck.fTimeBasedSurpassMult;
                                    fCurAccelMult *= fAccelMult;
                                    if (fCurAccelMult < 0.01) {
                                        RAVELOG_VERBOSE_FORMAT("env = %d: shortcut iter = %d/%d: fCurAccelMult is too small (%.15e). continue to the next iteration", GetEnv()->GetId()%iters%numIters%fCurAccelMult);
                                        break;
                                    }
                                    {
                                        fVelMult = RaveSqrt(fAccelMult); // larger scaling factor, less reduction. Use a square root here since the velocity has the factor t while the acceleration has t^2
                                        fCurVelMult *= fVelMult;
                                        if (fCurVelMult < 0.01) {
                                            RAVELOG_VERBOSE_FORMAT("env = %d: shortcut iter = %d/%d: fCurVelMult is too small (%.15e). continue to the next iteration", GetEnv()->GetId()%iters%numIters%fCurVelMult);
                                            break;
                                        }
                                        for (size_t j = 0; j < vellimits.size(); ++j) {
                                            dReal fMinVel = max(RaveFabs(v0[j]), RaveFabs(v1[j]));
                                            vellimits[j] = max(fMinVel, fVelMult * vellimits[j]);
                                        }
                                    }
                                    for (size_t j = 0; j < accellimits.size(); ++j) {
                                        accellimits[j] *= fAccelMult;
                                    }
                                }

                                numSlowDowns += 1;
                                RAVELOG_VERBOSE_FORMAT("env = %d: fTimeBasedSurpassMult = %.15e; fCurVelMult = %.15e; fCurAccelMult = %.15e", GetEnv()->GetId()%retcheck.fTimeBasedSurpassMult%fCurVelMult%fCurAccelMult);
                            }
                        }
                        else {
                            // Scale down vellimits and accellimits using the normal procedure
                            fCurVelMult *= retcheck.fTimeBasedSurpassMult;
                            fCurAccelMult *= retcheck.fTimeBasedSurpassMult;
                            if (fCurVelMult < 0.01) {
                                RAVELOG_VERBOSE_FORMAT("env = %d: shortcut iter = %d/%d: fCurVelMult is too small (%.15e). continue to the next iteration", GetEnv()->GetId()%iters%numIters%fCurVelMult);
                                break;
                            }
                            if (fCurAccelMult < 0.01) {
                                RAVELOG_VERBOSE_FORMAT("env = %d: shortcut iter = %d/%d: fCurAccelMult is too small (%.15e). continue to the next iteration", GetEnv()->GetId()%iters%numIters%fCurAccelMult);
                                break;
                            }

                            numSlowDowns += 1;
                            for (size_t j = 0; j < vellimits.size(); ++j) {
                                dReal fMinVel =  max(RaveFabs(v0[j]), RaveFabs(v1[j]));
                                vellimits[j] = max(fMinVel, retcheck.fTimeBasedSurpassMult * vellimits[j]);
                                accellimits[j] *= retcheck.fTimeBasedSurpassMult;
                            }
                        }
                    }
                    else {
                        RAVELOG_VERBOSE_FORMAT("env = %d: shortcut iter = %d/%d, rejecting shortcut due to constraint 0x%x", GetEnv()->GetId()%iters%numIters%retcheck.retcode);
                        break;
                    }
                    iIterProgress += 0x1000;
                } // finished slowing down the shortcut

                if (!bSuccess) {
                    // Shortcut failed. Continue to the next shortcut iteration.
                    continue;
                }

                if (shortcutCurvesNDVect1.size() == 0) {
                    RAVELOG_WARN("shortcutpath is empty!\n");
                    continue;
                }

                // Now this shortcut is really successful
                ++numShortcuts;

                // Keep track of zero-velocity waypoints
                dReal segmentEndTime = 0;
                for (std::vector<RampOptimizer::ParabolicCurvesND>::const_iterator itcurvesnd = shortcutCurvesNDVect1.begin(); itcurvesnd != shortcutCurvesNDVect1.end(); ++itcurvesnd) {
                    segmentEndTime += itcurvesnd->duration;
                }
                dReal diff = (t1 - t0) - segmentEndTime;

                size_t writeIndex = 0;
                for (size_t readIndex = 0; readIndex < _zeroVelPoints.size(); ++readIndex) {
                    if (_zeroVelPoints[readIndex] <= t0) {
                        writeIndex += 1;
                    }
                    else if (_zeroVelPoints[readIndex] <= t1) {
                        // Do nothing
                    }
                    else {
                        _zeroVelPoints[writeIndex++] = _zeroVelPoints[readIndex] - diff;
                    }
                }
                // RAVELOG_DEBUG_FORMAT("zeroVelPoints.size(): %d -> %d", zeroVelPoints.size()%cacheZeroVelPoints.size());
                _zeroVelPoints.resize(writeIndex);

                // Keep track of the multipliers
                fStartTimeVelMult = min(1.0, fCurVelMult * fiSearchVelAccelMult);
                fStartTimeAccelMult = min(1.0, fCurAccelMult * fiSearchVelAccelMult);

                // Now replace the original trajectory segment by the shortcut
                parabolicpath.ReplaceSegment(t0, t1, shortcutCurvesNDVect1);
                iIterProgress += 0x10000000;

                // Check consistency
                if (IS_DEBUGLEVEL(Level_Verbose)) {
                    for (size_t icurvesnd = 0; icurvesnd + 1 < curvesndVect.size(); ++icurvesnd) {
                        for (size_t jdof = 0; jdof < curvesndVect[icurvesnd].ndof; ++jdof) {
                            OPENRAVE_ASSERT_OP(RaveFabs(curvesndVect[icurvesnd].x1Vect[jdof] - curvesndVect[icurvesnd + 1].x0Vect[jdof]), <=, RampOptimizer::epsilon);
                            OPENRAVE_ASSERT_OP(RaveFabs(curvesndVect[icurvesnd].v1Vect[jdof] - curvesndVect[icurvesnd + 1].v0Vect[jdof]), <=, RampOptimizer::epsilon);
                        }
                    }
                }
                iIterProgress += 0x10000000;

                tTotal = parabolicpath.duration;
                RAVELOG_DEBUG_FORMAT("env = %d: shortcut iter = %d/%d successful, numSlowDowns = %d, tTotal = %.15e", GetEnv()->GetId()%iters%numIters%numSlowDowns%tTotal);

                // Calculate the score
                score = diff/nItersFromPrevSuccessful;
                if (score > currentBestScore) {
                    currentBestScore = score;
                    iCurrentBestScore = 1.0/currentBestScore;
                }
                nItersFromPrevSuccessful = 0;

                if ((score*iCurrentBestScore < cutoffRatio) && (numShortcuts > 5)) {
                    // We have already shortcut for a bit (numShortcuts > 5). The progress made in
                    // this iteration is below the curoff ratio. If we continue, it is unlikely that
                    // we will make much more progress. So stop here.
                    break;
                }
            }
            catch (const std::exception &ex) {
                RAVELOG_WARN_FORMAT("env = %d: An exception happened during shortcut iteration progress = 0x%x: %s", GetEnv()->GetId()%iIterProgress%ex.what());
            }
        }

        // Report status
        if (iters == numIters) {
            RAVELOG_DEBUG_FORMAT("Finished at shortcut iter=%d (normal exit), successful=%d, slowdowns=%d, endTime: %.15e -> %.15e; diff = %.15e", iters%numShortcuts%numSlowDowns%tOriginal%tTotal%(tOriginal - tTotal));
        }
        else if (score*iCurrentBestScore < cutoffRatio) {
            RAVELOG_DEBUG_FORMAT("Finished at shortcut iter=%d (current score falls below %.15e), successful=%d, slowdowns=%d, endTime: %.15e -> %.15e; diff = %.15e", iters%cutoffRatio%numShortcuts%numSlowDowns%tOriginal%tTotal%(tOriginal - tTotal));
        }
        else if (nItersFromPrevSuccessful > nCutoffIters) {
            RAVELOG_DEBUG_FORMAT("Finished at shortcut iter=%d (did not make progress in the last %d iterations), successful=%d, slowdowns=%d, endTime: %.15e -> %.15e; diff = %.15e", iters%nCutoffIters%numShortcuts%numSlowDowns%tOriginal%tTotal%(tOriginal - tTotal));
        }
        _DumpParabolicPath(parabolicpath, _dumplevel, fileindex, 1);

        return numShortcuts;
    }

    std::string _DumpTrajectory(TrajectoryBasePtr traj, DebugLevel level) {
        if (IS_DEBUGLEVEL(level)) {
            std::string filename = _DumpTrajectory(traj);
            RavePrintfA(str(boost::format("env = %d: wrote trajectory to %s")%GetEnv()->GetId()%filename), level);
            return filename;
        }
        return std::string();
    }

    std::string _DumpTrajectory(TrajectoryBasePtr traj) {
        uint32_t randnum;
        if( !!_logginguniformsampler ) {
            randnum = _logginguniformsampler->SampleSequenceOneUInt32();
        }
        else {
            randnum = RaveRandomInt();
        }
        string filename = str(boost::format("%s/parabolicsmoother%d.traj.xml")%RaveGetHomeDirectory()%(randnum%1000));
        ofstream f(filename.c_str());
        // f << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);     /// have to do this or otherwise precision gets lost
        f << std::setprecision(RampOptimizer::prec);
        traj->serialize(f);
        return filename;
    }

    void _DumpParabolicPath(RampOptimizer::ParabolicPath &path, DebugLevel level=Level_Verbose, uint32_t fileindex=100000, int option=-1) const {
        if (!IS_DEBUGLEVEL(level)) {
            return;
        }

        if (fileindex == 100000) {
            // No particular index given, randomly choose one
            uint32_t randnum;
            if (!!_logginguniformsampler) {
                randnum = _logginguniformsampler->SampleSequenceOneUInt32();
            }
            else {
                randnum = RaveRandomInt();
            }
            fileindex = randnum%_fileIndexMod;
        }

        std::string filename;
        if (option == 0) {
            filename = str(boost::format("%s/parabolicpath%d.beforeshortcut.xml")%RaveGetHomeDirectory()%fileindex);
        }
        else if (option == 1) {
            filename = str(boost::format("%s/parabolicpath%d.aftershortcut.xml")%RaveGetHomeDirectory()%fileindex);
        }
        else {
            filename = str(boost::format("%s/parabolicpath%d.xml")%RaveGetHomeDirectory()%fileindex);
        }
        ofstream f(filename.c_str());
        f << std::setprecision(RampOptimizer::prec);
        path.Serialize(f);
        RAVELOG_DEBUG_FORMAT("Wrote a parabolic path to %s (duration = %.15e)", filename%path.duration);
        return;
    }

    void VectorToString(const std::vector<dReal> &v, std::string &s, const char* name) {
        s = "";
        s += name;
        s += " = [";
        std::string separator = "";
        FOREACH(tempit, v) {
            s += str(boost::format("%s%.15e")%separator%*tempit);
            separator = ", ";
        }
        s += "]; ";
        return;
    }

    // Members
    ConstraintTrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _uniformsampler; ///< used for planning, seed is controlled
    SpaceSamplerBasePtr _logginguniformsampler; ///< used for logging, seed is random
    ConstraintFilterReturnPtr _constraintreturn;
    MyParabolicCurvesNDFeasibilityChecker _feasibilitychecker;
    boost::shared_ptr<ManipConstraintChecker2> _manipconstraintchecker;

    TrajectoryBasePtr _pdummytraj;
    PlannerProgress _progress;
    bool _bUsePerturnation;
    bool _bmanipconstraints;

    std::vector<dReal> _zeroVelPoints; // keep track of original waypoints

    // Caching stuff
    // Used in PlanPath
    RampOptimizer::ParabolicPath _cacheparabolicpath;
    std::vector<dReal> _cachex0Vect, _cachex1Vect, _cachev0Vect, _cachev1Vect, _cachetVect;
    std::vector<dReal> _cacheWaypoints; // a vector of concatenated waypoints (size = ndof*nwaypoints)
    std::vector<std::vector<dReal> > _cacheWaypointsVect; // a vector for waypoints (size = nwaypoints)
    RampOptimizer::ParabolicCurvesND _cacheCurvesND;
    std::vector<dReal> _cacheSwitchtimes;
    std::vector<RampOptimizer::ParabolicCurvesND> _cacheCurvesNDVectOut;

    // Used in _SetMilestones
    std::vector<std::vector<dReal> > _cacheNewWaypointsVect;
    RampOptimizer::ParabolicCurvesND _cacheCurvesND1; // this will also be used in _Shortcut

    // Used in _TimeParameterizeZeroVel
    std::vector<dReal> _cachex0Vect2, _cachex1Vect2, _cachev0Vect2, _cachev1Vect2;
    std::vector<dReal> _cachevellimits, _cacheaccellimits; // these vectors are also used later in _Shortcut
    std::vector<RampOptimizer::ParabolicCurvesND> _cacheCurvesNDVect1; // this vector is also used later in _Shortcut

    // Used in _Shortcut
    RampOptimizer::ParabolicPath _cacheShortcutPath;
    std::vector<RampOptimizer::ParabolicCurvesND> _cacheCurvesNDVect2;
    RampOptimizer::ParabolicCurvesND _cacheCurvesND2;

    // Used in SegmentFeasible2
    RampOptimizer::ParabolicCurvesND _cacheCurvesNDSeg;

    uint32_t _fileIndexMod; // maximum number of trajectory indices allowed
    DebugLevel _dumplevel;

#ifdef SMOOTHER_TIMING_DEBUG
    // Statistics
    size_t _nCallsCheckManip;
    dReal _totalTimeCheckManip;
    uint32_t _tStartCheckManip, _tEndCheckManip;

    size_t _nCallsInterpolation;
    dReal _totalTimeInterpolation;
    uint32_t _tStartInterpolation, _tEndInterpolation;;
#endif

};

PlannerBasePtr CreateParabolicSmoother2(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new ParabolicSmoother2(penv, sinput));
}

} // end namespace rplanners

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(RampOptimizer::Ramp)
BOOST_TYPEOF_REGISTER_TYPE(RampOptimizer::ParabolicCurve)
BOOST_TYPEOF_REGISTER_TYPE(RampOptimizer::ParabolicCurvesND)
#endif
