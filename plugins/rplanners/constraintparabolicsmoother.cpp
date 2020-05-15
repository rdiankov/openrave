// -*- coding: utf-8 -*-
// Copyright (C) 2012-2013 Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "rplanners.h" // openraveplugindefs + _(msgid)
#include <fstream>

#include <openrave/planningutils.h>

#include "ParabolicPathSmooth/DynamicPath.h"
#include "mergewaypoints.h"


namespace ParabolicRamp = ParabolicRampInternal;

class ConstraintParabolicSmoother : public PlannerBase, public ParabolicRamp::FeasibilityCheckerBase, public ParabolicRamp::RandomNumberGeneratorBase
{
    struct ManipConstraintInfo
    {
        // the end-effector link of the manipulator
        KinBody::LinkPtr plink;
        // the points to check for in the end effector coordinate system
        // for now the checked points are the vertices of the bounding box
        // but this can be more general, e.g. the vertices of the convex hull
        std::list<Vector> checkpoints;
    };


    // Compute the AABB that encloses all the links in linklist
    AABB ComputeEnclosingAABB(std::list<KinBody::LinkPtr> linklist){
        Vector vmin, vmax;
        bool binitialized=false;
        AABB ab;
        FOREACHC(itlink,linklist) {
            ab = (*itlink)->ComputeAABB(); // AABB of the link in the global coordinates
            if((ab.extents.x == 0)&&(ab.extents.y == 0)&&(ab.extents.z == 0)) {
                continue;
            }
            Vector vnmin = ab.pos - ab.extents;
            Vector vnmax = ab.pos + ab.extents;
            if( !binitialized ) {
                vmin = vnmin;
                vmax = vnmax;
                binitialized = true;
            }
            else {
                if( vmin.x > vnmin.x ) {
                    vmin.x = vnmin.x;
                }
                if( vmin.y > vnmin.y ) {
                    vmin.y = vnmin.y;
                }
                if( vmin.z > vnmin.z ) {
                    vmin.z = vnmin.z;
                }
                if( vmax.x < vnmax.x ) {
                    vmax.x = vnmax.x;
                }
                if( vmax.y < vnmax.y ) {
                    vmax.y = vnmax.y;
                }
                if( vmax.z < vnmax.z ) {
                    vmax.z = vnmax.z;
                }
            }
        }
        BOOST_ASSERT(binitialized); // the linklist must contain at least the end effector
        ab.pos = (dReal)0.5 * (vmin + vmax);
        ab.extents = vmax - ab.pos;
        return ab;
    }


    void AABBtoCheckPoints(AABB ab, Transform T, std::list<Vector>& checkpoints){
        dReal signextents[24] = {1,1,1,   1,1,-1,  1,-1,1,   1,-1,-1,  -1,1,1,   -1,1,-1,  -1,-1,1,   -1,-1,-1};
        Vector incr;
        Transform Tinv = T.inverse();
        checkpoints.resize(0);
        for(int i=0; i<8; i++) {
            incr[0] = ab.extents[0] * signextents[3*i+0];
            incr[1] = ab.extents[1] * signextents[3*i+1];
            incr[2] = ab.extents[2] * signextents[3*i+2];
            checkpoints.push_back( Tinv*(ab.pos + incr));
        }
    }


public:
    ConstraintParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\nConstraint-based smoothing with `Indiana University Intelligent Motion Laboratory <http://www.iu.edu/~motion/software.html>`_ parabolic smoothing library (Kris Hauser).\n\n**Note:** The original trajectory will not be preserved at all, don't use this if the robot has to hit all points of the trajectory.\n";
        _bCheckControllerTimeStep = true;
        //_distancechecker = RaveCreateCollisionChecker(penv, "pqp");
        //OPENRAVE_ASSERT_FORMAT0(!!_distancechecker, "need pqp distance checker", ORE_Assert);
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        _parameters->copy(params);
        _probot = pbase;
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        isParameters >> *_parameters;
        _probot = pbase;
        return _InitPlan();
    }

    bool _InitPlan()
    {
        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 100;
        }
        // always a step length!
        if( _parameters->_fStepLength <= g_fEpsilonLinear ) {
            _parameters->_fStepLength = 0.001;
        }

        // // workspace constraints on links
        // _constraintreturn.reset(new ConstraintFilterReturn());
        // _listCheckManips.clear();
        // if( _parameters->maxlinkspeed > 0 || _parameters->maxlinkaccel ) {
        //     // extract links from _parameters->_configurationspecification?
        //     //_listCheckManipss
        //     //_parameters->_configurationspecification
        // }

        _bmanipconstraints = _parameters->maxmanipspeed>0 || _parameters->maxmanipaccel>0;

        // initialize workspace constraints on manipulators
        if(_bmanipconstraints) {
            _listCheckManips.clear();
            std::vector<KinBodyPtr> listUsedBodies;
            std::set<KinBody::LinkPtr> setCheckedManips;
            _parameters->_configurationspecification.ExtractUsedBodies(GetEnv(), listUsedBodies);
            FOREACH(itbody, listUsedBodies) {
                KinBodyPtr pbody = *itbody;
                if( pbody->IsRobot() ) {
                    RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
                    std::vector<int> vuseddofindices, vusedconfigindices;
                    _parameters->_configurationspecification.ExtractUsedIndices(probot, vuseddofindices, vusedconfigindices);
                    // go through every manipulator and check whether it depends on vusedindices
                    FOREACH(itmanip, probot->GetManipulators()) {
                        KinBody::LinkPtr endeffector = (*itmanip)->GetEndEffector();
                        if( setCheckedManips.find(endeffector) != setCheckedManips.end() ) {
                            // already checked, so don't do anything
                            continue;
                        }
                        bool manipisaffected = false;
                        FOREACH(itdofindex, vuseddofindices) {
                            if( probot->DoesDOFAffectLink(*itdofindex, endeffector->GetIndex()) ) {
                                manipisaffected = true;
                                break;
                            }
                        }
                        if (!manipisaffected) {
                            // manip is not affected by trajectory, so don't do anything
                            continue;
                        }

                        // Insert all child links of endeffector
                        std::list<KinBody::LinkPtr> globallinklist;
                        std::vector<KinBody::LinkPtr> vchildlinks;
                        (*itmanip)->GetChildLinks(vchildlinks);
                        FOREACH(itlink,vchildlinks){
                            globallinklist.push_back(*itlink);
                        }
                        // Insert all links of all bodies that the endeffector is grabbing
                        std::vector<KinBodyPtr> grabbedbodies;
                        probot->GetGrabbed(grabbedbodies);
                        FOREACH(itbody,grabbedbodies){
                            if((*itmanip)->IsGrabbing(**itbody)) {
                                FOREACH(itlink,(*itbody)->GetLinks()){
                                    globallinklist.push_back(*itlink);
                                }
                            }
                        }
                        // Compute the enclosing AABB and add its vertices to the checkpoints
                        AABB enclosingaabb = ComputeEnclosingAABB(globallinklist);
                        _listCheckManips.push_back(ManipConstraintInfo());
                        _listCheckManips.back().plink = endeffector;
                        std::list<Vector> checkpoints;
                        AABBtoCheckPoints(enclosingaabb,endeffector->GetTransform(),checkpoints);
                        _listCheckManips.back().checkpoints = checkpoints;
                        setCheckedManips.insert(endeffector);
                    }
                }
            }
        }

        // Seet seed
        if( !_uniformsampler ) {
            _uniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
            OPENRAVE_ASSERT_FORMAT0(!!_uniformsampler, "need mt19937 space samplers", ORE_Assert);
        }
        _uniformsampler->SetSeed(_parameters->_nRandomGeneratorSeed);


        // check and update minswitchtime
        _parameters->minswitchtime = ComputeStepSizeCeiling(_parameters->minswitchtime, _parameters->_fStepLength);
        if(_bCheckControllerTimeStep) {
            _parameters->minswitchtime = max(_parameters->minswitchtime, 5*_parameters->_fStepLength);
        }

        return true;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        BOOST_ASSERT(!!_parameters && !!ptraj);
        if( ptraj->GetNumWaypoints() < 2 ) {
            return PlannerStatus(PS_Failed);
        }

        //Writing the incoming traj
        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            string filename = str(boost::format("%s/inittraj%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%10000));
            RAVELOG_VERBOSE_FORMAT("Writing original traj to %s", filename);
            ofstream f(filename.c_str());
            f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
            ptraj->serialize(f);
        }

        RobotBase::RobotStateSaverPtr statesaver;
        if( !!_probot ) {
            statesaver.reset(new RobotBase::RobotStateSaver(_probot));
        }

        uint32_t basetime = utils::GetMilliTime();

        ConfigurationSpecification posspec = _parameters->_configurationspecification;
        //_setstatefn = posspec.GetSetFn(GetEnv());
        ConfigurationSpecification velspec = posspec.ConvertToVelocitySpecification();
        //_setvelstatefn = velspec.GetSetFn(GetEnv());
        ConfigurationSpecification timespec;
        timespec.AddDeltaTimeGroup();

        vector<dReal> vtrajpoints;
        ptraj->GetWaypoints(0,ptraj->GetNumWaypoints(),vtrajpoints,posspec);

        ParabolicRamp::RampFeasibilityChecker checker(this);
        checker.tol = _parameters->_vConfigResolution;
        FOREACH(it, checker.tol) {
            *it *= _parameters->_pointtolerance;
        }
        checker.constraintsmask = CFO_CheckEnvCollisions|CFO_CheckSelfCollisions|CFO_CheckTimeBasedConstraints|CFO_CheckUserConstraints;
        RAVELOG_VERBOSE_FORMAT("minswitchtime = %f, steplength=%f\n",_parameters->minswitchtime%_parameters->_fStepLength);

        try {
            //  Convert to ramps
            std::list<ParabolicRamp::ParabolicRampND> ramps,ramps2;
            std::vector<ConfigurationSpecification::Group>::const_iterator itcompatposgroup = ptraj->GetConfigurationSpecification().FindCompatibleGroup(posspec._vgroups.at(0), false);
            OPENRAVE_ASSERT_FORMAT(itcompatposgroup != ptraj->GetConfigurationSpecification()._vgroups.end(), "failed to find group %s in passed in trajectory", posspec._vgroups.at(0).name, ORE_InvalidArguments);

            // Case 1 : Initial traj is quadratic
            // assumes that the traj has velocity data and is consistent, so convert the original trajectory in a sequence of ramps, and preserve velocity
            if (_parameters->_hastimestamps && itcompatposgroup->interpolation == "quadratic" ) {
                RAVELOG_VERBOSE("Initial traj is piecewise quadratic\n");
                vector<dReal> x0, x1, dx0, dx1, ramptime;
                ptraj->GetWaypoint(0,x0,posspec);
                ptraj->GetWaypoint(0,dx0,velspec);
                for(size_t i=0; i+1<ptraj->GetNumWaypoints(); i++) {
                    ptraj->GetWaypoint(i+1,ramptime,timespec);
                    if (ramptime.at(0) > g_fEpsilonLinear) {
                        ptraj->GetWaypoint(i+1,x1,posspec);
                        ptraj->GetWaypoint(i+1,dx1,velspec);
                        ramps.push_back(ParabolicRamp::ParabolicRampND());
                        ramps.back().SetPosVelTime(x0,dx0,x1,dx1,ramptime.at(0));
                        /*if( !checker.Check(ramps.back())) {
                            RAVELOG_WARN("ramp %d failed\n", i);
                            ramps.back().SetPosVelTime(x0,dx0,x1,dx1,2.0*ramptime.at(0));
                            checker.Check(ramps.back());
                            }*/
                        x0.swap(x1);
                        dx0.swap(dx1);
                    }
                }
                // Change timing of ramps so that they satisfy minswitchtime, _fStepLength, and dynamics and collision constraints

                // Disable usePerturbation for this particular stage
                int options = 0xffff;
                // Reset all ramps
                FOREACH(itramp, ramps) {
                    itramp->modified = true;
                }
                mergewaypoints::PrintRamps(ramps,_parameters,false);
                // Try first fixing trajectory ends (for traj comming from previous jittering operation)
                bool res = mergewaypoints::FixRampsEnds(ramps,ramps2, _parameters,checker,options);
                if(!res) {
                    RAVELOG_DEBUG("First or last two ramps could not be fixed, try something more general...\n");
                    // More general algorithm
                    dReal upperbound = 2;
                    dReal stepsize = 0.1;
                    res = mergewaypoints::IterativeMergeRampsNoDichotomy(ramps,ramps2, _parameters, upperbound, stepsize, _bCheckControllerTimeStep, _uniformsampler,checker,options);
                }
                if(!res) {
                    std::string filename = _DumpTrajectory(ptraj, Level_Verbose);
                    std::string description = _("Could not obtain a feasible trajectory from initial quadratic trajectory\n");
                    RAVELOG_WARN(description);
                    return PlannerStatus(description, PS_Failed);
                }
                RAVELOG_DEBUG("Cool: obtained a feasible trajectory from initial quadratic trajectory\n");
                ramps.swap(ramps2);
            }
            else {
                // Case 2 : Initial traj is linear
                RAVELOG_VERBOSE("Initial traj is piecewise linear\n");
                ParabolicRamp::Vector q(_parameters->GetDOF());
                vector<ParabolicRamp::Vector> path;
                path.reserve(ptraj->GetNumWaypoints());
                // Group waypoints that are on the same segment
                for(size_t i = 0; i < ptraj->GetNumWaypoints(); ++i) {
                    std::copy(vtrajpoints.begin()+i*_parameters->GetDOF(),vtrajpoints.begin()+(i+1)*_parameters->GetDOF(),q.begin());
                    if( path.size() >= 2 ) {
                        // check if collinear by taking angle
                        const ParabolicRamp::Vector& x0 = path[path.size()-2];
                        const ParabolicRamp::Vector& x1 = path[path.size()-1];
                        dReal dotproduct=0,x0length2=0,x1length2=0;
                        for(size_t i = 0; i < q.size(); ++i) {
                            dReal dx0=x0[i]-q[i];
                            dReal dx1=x1[i]-q[i];
                            dotproduct += dx0*dx1;
                            x0length2 += dx0*dx0;
                            x1length2 += dx1*dx1;
                        }
                        dReal ferror = RaveFabs(dotproduct * dotproduct - x0length2*x1length2);
                        if( ferror < 100*ParabolicRamp::EpsilonX*ParabolicRamp::EpsilonX ) {
                            path.back() = q;
                            continue;
                        }
                    }
                    // check if the point is not the same as the previous point
                    if( path.size() > 0 ) {
                        bool bIsClose = true;
                        for(size_t i = 0; i < q.size(); ++i) {
                            if( RaveFabs(q[i]-path.back().at(i)) > ParabolicRamp::EpsilonX ) {
                                bIsClose = false;
                            }
                        }
                        if( bIsClose ) {
                            continue;
                        }
                    }
                    path.push_back(q);
                }

                // Convert to unitary ramps which are linear in config space
                // and guarantees that that two waypoints are at least _parameters->minswitchtime away
                // Note that this should never fail if initial traj is collision-free
                // and if dynamics is always satisfied at zero velocity
                if( !SetMilestones(ramps, path,checker) ) {
                    _DumpTrajectory(ptraj, Level_Verbose);
                    return PlannerStatus(PS_Failed);
                }
            }

            //  Shortcutting
            // Break into unitary ramps
            mergewaypoints::BreakIntoUnitaryRamps(ramps);

            // Sanity check before any shortcutting
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                RAVELOG_VERBOSE("Sanity check before starting shortcutting...\n");
                int options = 0xffff;
                int iramp = 0;
                FOREACHC(itramp,ramps){
                    if(checker.Check(*itramp,options) != 0) {
                        _DumpTrajectory(ptraj, Level_Verbose);
                        std::string description = str(boost::format(_("Ramp %d/%d of original traj invalid"))%iramp%ramps.size());
                        RAVELOG_WARN(description);
                        return PlannerStatus(description, PS_Failed);
                    }
                    ++iramp;
                }
            }

            // Reset all ramps
            FOREACH(itramp, ramps) {
                itramp->modified = false;
            }

            // Log before shortcutting
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                mergewaypoints::PrintRamps(ramps,_parameters,_bCheckControllerTimeStep);
            }
            dReal totaltime = mergewaypoints::ComputeRampsDuration(ramps);
            RAVELOG_DEBUG_FORMAT("initial ramps=%d, duration=%f, pointtolerance=%f", ramps.size()%totaltime%_parameters->_pointtolerance);
            int numshortcuts=0;
            if( totaltime > _parameters->_fStepLength ) {

                // Start shortcutting
                RAVELOG_DEBUG("Start shortcutting\n");
                _progress._iteration=0;
                dReal besttime = 1e10;
                std::list<ParabolicRamp::ParabolicRampND> bestramps,initramps;
                initramps = ramps;
                for(int rep=0; rep<_parameters->nshortcutcycles; rep++) {
                    ramps = initramps;
                    RAVELOG_VERBOSE_FORMAT("Start shortcut cycle %d\n",rep);
                    numshortcuts = Shortcut(ramps, _parameters->_nMaxIterations,checker, this);
                    totaltime = mergewaypoints::ComputeRampsDuration(ramps);
                    if(totaltime < besttime) {
                        bestramps = ramps;
                        besttime = totaltime;
                    }
                }
                ramps = bestramps;

                RAVELOG_DEBUG("End shortcutting\n");
                if( numshortcuts < 0 ) {
                    // interrupted
                    return PlannerStatus(PS_Interrupted);
                }


                // Log after shortcutting
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    RAVELOG_VERBOSE("Ramps after shortcutting\n");
                    mergewaypoints::PrintRamps(ramps,_parameters,_bCheckControllerTimeStep);
                }
                totaltime = mergewaypoints::ComputeRampsDuration(ramps);

                //  Further merge if possible

                int options = 0xffff;
                dReal upperbound = 1.05;
                std::list<ParabolicRamp::ParabolicRampND> resramps;
                bool resmerge = mergewaypoints::FurtherMergeRamps(ramps,resramps, _parameters, upperbound, _bCheckControllerTimeStep, _uniformsampler,checker,options);
                if(resmerge) {
                    RAVELOG_DEBUG("Great, could further merge ramps!!\n");
                    size_t nbrampsbefore = mergewaypoints::CountUnitaryRamps(ramps);
                    size_t nbrampsafter = mergewaypoints::CountUnitaryRamps(resramps);
                    dReal qualitybefore = mergewaypoints::ComputeRampQuality(ramps);
                    dReal qualityafter = mergewaypoints::ComputeRampQuality(resramps);
                    dReal totaltime2 = mergewaypoints::ComputeRampsDuration(resramps);
                    RAVELOG_DEBUG("Nb ramps %d --> %d\n",nbrampsbefore,nbrampsafter);
                    RAVELOG_DEBUG("Quality %f --> %f\n",qualitybefore,qualityafter);
                    RAVELOG_DEBUG("Duration %f --> %f\n",totaltime,totaltime2);
                    totaltime = totaltime2;
                    ramps.swap(resramps);
                    mergewaypoints::PrintRamps(ramps,_parameters,_bCheckControllerTimeStep);
                }
                else{
                    RAVELOG_DEBUG("Could not further merge ramps\n");
                }
            }
            else {
                RAVELOG_DEBUG_FORMAT("ramps are so close (%fs), so no need to shortcut", totaltime);
            }


            PlannerStatus status = ConvertRampsToOpenRAVETrajectory(ramps, &checker, ptraj->GetXMLId());
            if( status.GetStatusCode() == PS_Interrupted ) {
                return status;
            }
            else if( !(status.GetStatusCode() & PS_HasSolution) ) {
                _DumpTrajectory(ptraj, Level_Verbose);
                return status;
            }

            if( _dummytraj->GetNumWaypoints() == 0 ) {
                // most likely a trajectory with the same start and end points were given, so return a similar trajectory
                // get the first point
                vtrajpoints.resize(posspec.GetDOF());
                ptraj->GetWaypoint(0,vtrajpoints,posspec);
                _dummytraj->Insert(0, vtrajpoints, posspec);
            }
            OPENRAVE_ASSERT_OP(status.GetStatusCode(), ==, PS_HasSolution);
            OPENRAVE_ASSERT_OP(RaveFabs(totaltime-_dummytraj->GetDuration()),<,0.001);
            RAVELOG_DEBUG_FORMAT("after shortcutting %d times: path waypoints=%d, traj waypoints=%d, traj time=%fs", numshortcuts%ramps.size()%_dummytraj->GetNumWaypoints()%totaltime);
            ptraj->Swap(_dummytraj);
        }
        catch (const std::exception& ex) {
            _DumpTrajectory(ptraj, Level_Verbose);
            std::string description = str(boost::format(_("parabolic planner failed: %s"))% ex.what());
            RAVELOG_WARN(description);
            return PlannerStatus(description, PS_Failed);

        }

        RAVELOG_DEBUG(str(boost::format("path optimizing - computation time=%fs\n")%(0.001f*(float)(utils::GetMilliTime()-basetime))));
        return PlannerStatus(PS_HasSolution);
    }

    std::string _DumpTrajectory(TrajectoryBasePtr traj, DebugLevel level)
    {
        if( IS_DEBUGLEVEL(level) ) {
            std::string filename = _DumpTrajectory(traj);
            RavePrintfA(str(boost::format("wrote parabolicsmoothing trajectory to %s")%filename), level);
            return filename;
        }
        return std::string();
    }
    std::string _DumpTrajectory(TrajectoryBasePtr traj)
    {
        // store the trajectory
        string filename = str(boost::format("%s/parabolicsmoother%d.traj.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%1000));
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        traj->serialize(f);
        return filename;
    }

    /// \brief converts ramps to an openrave trajectory structure. If return is PS_HasSolution, _dummytraj has the converted result
    PlannerStatus ConvertRampsToOpenRAVETrajectory(const std::list<ParabolicRamp::ParabolicRampND>& ramps, ParabolicRamp::RampFeasibilityChecker* pchecker, const std::string& sTrajectoryXMLId=std::string())
    {
        const ConfigurationSpecification& posspec = _parameters->_configurationspecification;
        ConfigurationSpecification velspec = posspec.ConvertToVelocitySpecification();
        ConfigurationSpecification newspec = _parameters->_configurationspecification;
        newspec.AddDerivativeGroups(1,true);
        int waypointoffset = newspec.AddGroup("iswaypoint", 1, "next");

        int timeoffset=-1;
        FOREACH(itgroup,newspec._vgroups) {
            if( itgroup->name == "deltatime" ) {
                timeoffset = itgroup->offset;
            }
            else if( velspec.FindCompatibleGroup(*itgroup) != velspec._vgroups.end() ) {
                itgroup->interpolation = "linear";
            }
            else if( posspec.FindCompatibleGroup(*itgroup) != posspec._vgroups.end() ) {
                itgroup->interpolation = "quadratic";
            }
        }

        // have to write to another trajectory
        if( !_dummytraj || (sTrajectoryXMLId.size() > 0 && _dummytraj->GetXMLId() != sTrajectoryXMLId) ) {
            _dummytraj = RaveCreateTrajectory(GetEnv(), sTrajectoryXMLId);
        }
        _dummytraj->Init(newspec);

        if( ramps.size() == 0 ) {
            return PlannerStatus(PS_HasSolution);
        }

        // separate all the acceleration switches into individual points
        _vtrajpointscache.resize(newspec.GetDOF());
        ConfigurationSpecification::ConvertData(_vtrajpointscache.begin(),newspec,ramps.front().x0.begin(),posspec,1,GetEnv(),true);
        ConfigurationSpecification::ConvertData(_vtrajpointscache.begin(),newspec,ramps.front().dx0.begin(),velspec,1,GetEnv(),false);
        _vtrajpointscache.at(waypointoffset) = 1;
        _vtrajpointscache.at(timeoffset) = 0;
        _dummytraj->Insert(_dummytraj->GetNumWaypoints(),_vtrajpointscache);
        ParabolicRamp::Vector vconfig;
        // TODO ramps are unitary so don't have to track switch points
        FOREACH(itrampnd,ramps) {
            // double-check the current ramps
            //                if(_parameters->verifyinitialpath) {
            if(!!pchecker) {
                // part of original trajectory which might not have been processed with perturbations, so ignore them
                int options = 0xffff; // no perturbation
                if( pchecker->Check(*itrampnd,options) != 0) {
                    // unfortunately happens sometimes when the robot is close to corners.. not sure if returning is failing is the right solution here..

                    std::string description = "original ramp does not satisfy constraints!\n";
                    RAVELOG_WARN(description);
                    return PlannerStatus(description, PS_Failed);
                }
                _progress._iteration+=1;
                if( _CallCallbacks(_progress) == PA_Interrupt ) {
                    return PlannerStatus(PS_Interrupted);
                }
            }

            if( itrampnd->ramps.at(0).tswitch1 > 0 && itrampnd->ramps.at(0).tswitch1 < itrampnd->endTime-ParabolicRamp::EpsilonT ) {
                std::string description = "ramp is not unitary\n";
                RAVELOG_WARN(description);
                return PlannerStatus(description, PS_Failed);
            }
            if( itrampnd->ramps.at(0).tswitch2 > 0 && itrampnd->ramps.at(0).tswitch2 < itrampnd->endTime-ParabolicRamp::EpsilonT ) {
                std::string description = "ramp is not unitary\n";
                RAVELOG_WARN(description);
                return PlannerStatus(description, PS_Failed);
            }

            _vtrajpointscache.resize(newspec.GetDOF());
            vector<dReal>::iterator ittargetdata = _vtrajpointscache.begin();
            ConfigurationSpecification::ConvertData(ittargetdata,newspec,itrampnd->x1.begin(),posspec,1,GetEnv(),true);
            ConfigurationSpecification::ConvertData(ittargetdata,newspec,itrampnd->dx1.begin(),velspec,1,GetEnv(),false);
            *(ittargetdata+timeoffset) = itrampnd->endTime;
            *(ittargetdata+waypointoffset) = 1;
            ittargetdata += newspec.GetDOF();
            _dummytraj->Insert(_dummytraj->GetNumWaypoints(),_vtrajpointscache);
        }

        return PlannerStatus(PS_HasSolution);
    }

    bool SetMilestones(std::list<ParabolicRamp::ParabolicRampND>& ramps, const vector<ParabolicRamp::Vector>& x, ParabolicRamp::RampFeasibilityChecker& check){

        ramps.clear();
        if(x.size()==1) {
            ramps.push_back(ParabolicRamp::ParabolicRampND());
            ramps.front().SetConstant(x[0]);
        }
        else if( x.size() > 1 ) {
            int options = 0xffff; // no perturbation
            if(!_parameters->verifyinitialpath) {
                RAVELOG_VERBOSE("Initial path verification is disabled (in SetMilestones)\n");
                options = options & (~CFO_CheckEnvCollisions) & (~CFO_CheckSelfCollisions); // no collision checking
            }
            for(size_t i=0; i+1<x.size(); i++) {
                std::list<ParabolicRamp::ParabolicRampND> tmpramps0, tmpramps1;
                bool cansetmilestone = mergewaypoints::ComputeLinearRampsWithConstraints(tmpramps0,x[i],x[i+1],_parameters,check,options);
                if( !cansetmilestone ) {
                    RAVELOG_INFO_FORMAT("linear ramp %d-%d (of %d) failed to pass constraints", i%(i+1)%x.size());
                    return false;
                }
                dReal tmpduration = mergewaypoints::ComputeRampsDuration(tmpramps0);
                if( tmpduration > 0 ) {
                    if(_bCheckControllerTimeStep) {
                        bool canscale = mergewaypoints::ScaleRampsTime(tmpramps0,tmpramps1,ComputeStepSizeCeiling(tmpduration,_parameters->_fStepLength*2)/tmpduration,false,_parameters);
                        BOOST_ASSERT(canscale);
                        ramps.splice(ramps.end(),tmpramps1);
                    }
                    else{
                        ramps.splice(ramps.end(),tmpramps0);
                    }
                }
            }
        }
        return true;
    }

// Check whether the shortcut end points (t1,t2) are similar to already attempted shortcut end points
    bool HasBeenAttempted(dReal t1,dReal t2, std::list<std::pair<dReal,dReal> >& attemptedlist, dReal shortcutinnovationthreshold){
        if(attemptedlist.size()==0) {
            return false;
        }
        std::list<std::pair<dReal,dReal> >::iterator it = attemptedlist.begin();
        while(it!=attemptedlist.end()) {
            if(RaveFabs(it->first-t1)<=shortcutinnovationthreshold+g_fEpsilonLinear && RaveFabs(it->second-t2)<=shortcutinnovationthreshold+g_fEpsilonLinear) {
                return true;
            }
            it++;
        }
        return false;
    }

    // const std::list<dReal>& t1list, const std::list<dReal>&t2list,dReal shortcutinnovationthreshold){

    //     if(t1list.size()==0) {
    //         return false;
    //     }
    //     std::list<dReal>::iterator it1 = t1list.begin(), it2 = t2list.begin();
    //     while(it1!=t1list.end()) {
    //         if(RaveFabs(*it1-t1)<=shortcutinnovationthreshold+g_fEpsilonLinear && RaveFabs(*it2-t2)<=shortcutinnovationthreshold+g_fEpsilonLinear) {
    //             return true;
    //         }
    //         it1++;
    //         it2++;
    //     }
    //     return false;
    // }

    void UpdateAttemptedList(std::list<std::pair<dReal,dReal> >& attemptedlist, dReal tbeginmod, dReal tendmod, dReal delta){
        bool smart=true;
        if(!smart) {
            attemptedlist.resize(0);
        }
        else{
            std::list<std::pair<dReal,dReal> >::iterator it = attemptedlist.begin();
            while(it!=attemptedlist.end()) {
                if(it->second<=tbeginmod) {
                    it++;
                }
                else if(it->first>=tendmod+delta) {
                    it->first -= delta;
                    it->second -= delta;
                    it++;
                }
                else {
                    it = attemptedlist.erase(it);
                }
            }
        }
    }


    // Perform the shortcuts
    int Shortcut(std::list<ParabolicRamp::ParabolicRampND>&ramps, int numIters, ParabolicRamp::RampFeasibilityChecker& check, ParabolicRamp::RandomNumberGeneratorBase* rng)
    {
        ParabolicRamp::Vector qstart = ramps.begin()->x0;
        ParabolicRamp::Vector qgoal = ramps.back().x1;
        int rejected = 0;
        int shortcuts = 0;
        std::list<ParabolicRamp::ParabolicRampND> saveramps;
        //std::map<int, std::list<int> > mapTestedTimeRanges; // (starttime, list of end times) pairs where the end times are always > than starttime
        std::vector<dReal> rampStartTime; rampStartTime.resize(ramps.size());
        std::list<std::pair<dReal,dReal> > attemptedlist;
        dReal durationbeforeshortcut=0;
        int i = 0;
        FOREACH(itramp, ramps) {
            rampStartTime[i++] = durationbeforeshortcut;
            durationbeforeshortcut += itramp->endTime;
        }
        ParabolicRamp::Vector x0,x1,dx0,dx1;
        std::list<ParabolicRamp::ParabolicRampND> intermediate;
        std::list<ParabolicRamp::ParabolicRampND>::iterator itramp1, itramp2;

        dReal fStepLength = _parameters->_fStepLength;
        dReal shortcutinnovationthreshold = 0;
        dReal fimprovetimethresh = _parameters->_fStepLength;

        // Iterative shortcutting
        for(int iters=0; iters<numIters; iters++) {

            dReal t1=rng->Rand()*durationbeforeshortcut,t2=rng->Rand()*durationbeforeshortcut;
            if(t1 > t2) {
                std::swap(t1,t2);
            }
            if( iters == 0 ) {
                t1 = 0;
                t2 = durationbeforeshortcut;
            }
            else {
                // round t1 and t2 to the closest multiple of fStepLength
                t1 = floor(t1/fStepLength+0.5)*fStepLength;
                t2 = floor(t2/fStepLength+0.5)*fStepLength;
                // check whether the shortcut is close to a previously attempted shortcut
                if(HasBeenAttempted(t1,t2,attemptedlist,shortcutinnovationthreshold)) {
                    RAVELOG_VERBOSE_FORMAT("Iter %d: Shortcut (%f,%f) already attempted, ramps=%d\n",iters%t1%t2%ramps.size());
                    rejected++;
                    continue;
                }
                else{
                    RAVELOG_VERBOSE_FORMAT("Iter %d: Attempt shortcut (%f,%f), ramps=%d...\n",iters%t1%t2%ramps.size());
//                    if( ramps.size() <= 10 && iters < 50 ) {
//                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
//                            ConvertRampsToOpenRAVETrajectory(ramps, NULL);
//                            string filename = str(boost::format("%s/smoothingiter%d.xml")%RaveGetHomeDirectory()%iters);
//                            RAVELOG_VERBOSE_FORMAT("writing current ramps to %s", filename);
//                            ofstream f(filename.c_str());
//                            f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
//                            _dummytraj->serialize(f);
//                        }
//                    }
                    attemptedlist.emplace_back(t1, t2);
                }
            }
            int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
            int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
            if(i1 == i2) {
                RAVELOG_VERBOSE("... Same ramp\n");
                continue;
            }

            _progress._iteration+=1;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return -1;
            }

            dReal u1 = t1-rampStartTime[i1];
            dReal u2 = t2-rampStartTime[i2];
            itramp1 = ramps.begin();
            advance(itramp1, i1);
            itramp2 = ramps.begin();
            advance(itramp2, i2);
            PARABOLIC_RAMP_ASSERT(u1 >= 0);
            PARABOLIC_RAMP_ASSERT(u1 <= itramp1->endTime+ParabolicRamp::EpsilonT);
            PARABOLIC_RAMP_ASSERT(u2 >= 0);
            PARABOLIC_RAMP_ASSERT(u2 <= itramp2->endTime+ParabolicRamp::EpsilonT);
            u1 = min(u1,itramp1->endTime);
            u2 = min(u2,itramp2->endTime);
            itramp1->Evaluate(u1,x0);
            itramp2->Evaluate(u2,x1);
            itramp1->Derivative(u1,dx0);
            itramp2->Derivative(u2,dx1);

            int options = 0xffff;
            // If we run mergewaypoints afterwards, no need to check for collision at this stage
            if(_parameters->minswitchtime > 0) {
                options = options &(~CFO_CheckEnvCollisions) & (~CFO_CheckSelfCollisions);
            }
            bool res = mergewaypoints::ComputeQuadraticRampsWithConstraints(intermediate,x0,dx0,x1,dx1,t2-t1, _parameters, check, options);

            if(!res) {
                RAVELOG_VERBOSE("... Could not SolveMinTime\n");
                continue;
            }

            // no idea what a good time improvement delta  is... _parameters->_fStepLength?
            dReal newramptime = 0;
            FOREACH(itramp, intermediate) {
                newramptime += itramp->endTime;
                itramp->modified = true;
            }



            if( newramptime+fimprovetimethresh >= t2-t1 ) {
                // reject since it didn't make significant improvement
                RAVELOG_VERBOSE("... Duration did not improve (even before merge)\n");
                RAVELOG_VERBOSE("shortcut iter=%d rejected time=%fs\n", iters, durationbeforeshortcut-(t2-t1)+newramptime);
                continue;
            }

            saveramps = ramps;
            //perform shortcut
            itramp1->TrimBack(itramp1->endTime-u1);
            itramp1->x1 = intermediate.front().x0;
            itramp1->dx1 = intermediate.front().dx0;
            itramp1->modified = true;
            itramp2->TrimFront(u2);
            itramp2->x0 = intermediate.back().x1;
            itramp2->dx0 = intermediate.back().dx1;
            itramp2->modified = true;

            //replace intermediate ramps
            ++itramp1;
            ramps.erase(itramp1, itramp2);

            mergewaypoints::BreakIntoUnitaryRamps(intermediate);
            ramps.splice(itramp2, intermediate);

            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                //check for consistency
                itramp1 = ramps.begin();
                itramp2 = ramps.begin(); ++itramp2;
                while(itramp2 != ramps.end() ) {
                    PARABOLIC_RAMP_ASSERT(itramp1->x1 == itramp2->x0);
                    PARABOLIC_RAMP_ASSERT(itramp1->dx1 == itramp2->dx0);
                    itramp1=itramp2;
                    ++itramp2;
                }
            }

            if(_parameters->minswitchtime == 0) {
                // Do not check for minswitchtime neither controllertime
                shortcuts++;
            }
            else{
                // Merge waypoints
                std::list<ParabolicRamp::ParabolicRampND> resramps;
                dReal upperbound = (durationbeforeshortcut-fimprovetimethresh)/mergewaypoints::ComputeRampsDuration(ramps);
                // Do not check collision during merge, check later
                int options = 0xffff & (~CFO_CheckEnvCollisions) & (~CFO_CheckSelfCollisions);


                bool resmerge = mergewaypoints::IterativeMergeRamps(ramps,resramps, _parameters, upperbound, _bCheckControllerTimeStep, _uniformsampler,check,options);

                if(!resmerge) {
                    RAVELOG_VERBOSE("... Could not merge\n");
                }
                else{
                    // Merge succeeded
                    // Check whether shortcut-and-merge improves the time duration
                    dReal durationaftermerge = mergewaypoints::ComputeRampsDuration(resramps);
                    if(durationaftermerge>durationbeforeshortcut-fimprovetimethresh) {
                        resmerge = false;
                        RAVELOG_VERBOSE("... Duration did not significantly improve after merger\n");
                    }
                    else{
                        // Now check for collision only for the ramps that have been modified
                        // Perturbations are applied when a configuration is outside the "radius" of start and goal config
                        int itx = 0;
                        dReal radius_around_endpoints = 0.1;
                        options = 0xffff;
                        FOREACH(itramp, resramps) {
                            if(!itramp->modified) {
                                continue;
                            }
                            if(!mergewaypoints::SpecialCheckRamp(*itramp,qstart,qgoal,radius_around_endpoints,_parameters,check,options)) {
                                RAVELOG_VERBOSE_FORMAT("... Collision for ramp %d after merge\n",itx);
                                resmerge = false;
                                break;
                            }
                            itx++;
                        }
                        if(resmerge) {
                            ramps.swap(resramps);
                            dReal tbeginmod = -1, tendmod = -1, tcur = 0;
                            FOREACH(itramp, ramps) {
                                if(itramp->modified) {
                                    if(tbeginmod == -1) {
                                        tbeginmod = tcur;
                                    }
                                    tcur += itramp->endTime;
                                    tendmod = tcur;
                                }
                                itramp->modified = false;
                            }
                            shortcuts++;
                            UpdateAttemptedList(attemptedlist,tbeginmod,tendmod,durationbeforeshortcut-durationaftermerge);
                            RAVELOG_VERBOSE_FORMAT("... Duration after shortcut and merger: %f\n",durationaftermerge);
                        }
                    } // end collision check
                } // end post-mergewaypoint checks

                if(!resmerge) {
                    // restore the ramps
                    ramps = saveramps;
                }
            } // end mergewaypoints

            //revise the timing
            rampStartTime.resize(ramps.size());
            durationbeforeshortcut=0;
            i = 0;
            FOREACH(itramp, ramps) {
                rampStartTime[i++] = durationbeforeshortcut;
                durationbeforeshortcut += itramp->endTime;
            }
        } // end shortcutting loop

        //RAVELOG_VERBOSE("Rejected: %d\n", rejected);
        return shortcuts;
    }


    virtual int ConfigFeasible(const ParabolicRamp::Vector& a, const ParabolicRamp::Vector& da, int options)
    {
        return _parameters->CheckPathAllConstraints(a,a, da, da, 0, IT_OpenStart, options);
    }

/** \brief return true if all the links in _listCheckManipssatisfy the acceleration and velocity constraints

   |w x (R x_i) + v| <= thresh

 */
    // virtual bool _CheckConstraintManips() const {
    //     FOREACHC(itinfo, _listCheckManips) {
    //     }
    //     return true;
    // }


    /*class GravitySaver
       {
       public:
        GravitySaver(EnvironmentBasePtr penv) : _penv(penv) {
            _vgravity = penv->GetPhysicsEngine()->GetGravity();
        }
        ~GravitySaver() {
            _penv->GetPhysicsEngine()->SetGravity(_vgravity);
        }
        EnvironmentBasePtr _penv;
        Vector _vgravity;
        };*/

    bool CheckManipConstraints(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b, const ParabolicRamp::Vector& da,const ParabolicRamp::Vector& db, dReal timeelapsed){
        ParabolicRamp::ParabolicRampND ramp;
        if(timeelapsed<=g_fEpsilonLinear) {
            return true;
        }
        else {
            //GravitySaver gravitysaver(GetEnv());
            //GetEnv()->GetPhysicsEngine()->SetGravity(Vector());
            ramp.SetPosVelTime(a,da,b,db,timeelapsed);
            vector<dReal> ac, dofaccelerations, qfill, vfill;
            ramp.Accel(timeelapsed/2,ac);
            bool res = true;
            FOREACH(it,_constraintreturn->_configurationtimes){
                vector<dReal> qc,vc;
                ramp.Evaluate(*it,qc);
                ramp.Derivative(*it,vc);
                // Compute the velocity and accel of the end effector COM
                FOREACH(itmanipinfo,_listCheckManips){
                    KinBodyPtr probot = itmanipinfo->plink->GetParent();
                    std::vector<int> vuseddofindices, vconfigindices;
                    _parameters->_configurationspecification.ExtractUsedIndices(probot, vuseddofindices, vconfigindices);
                    qfill.resize(vuseddofindices.size());
                    vfill.resize(vuseddofindices.size());
                    for(size_t idof = 0; idof < vuseddofindices.size(); ++idof) {
                        qfill[idof] = qc.at(vconfigindices.at(idof));
                    }
                    std::vector<std::pair<Vector,Vector> > endeffvels,endeffaccs;
                    Vector endeffvellin,endeffvelang,endeffacclin,endeffaccang;
                    int endeffindex = itmanipinfo->plink->GetIndex();
                    KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);

                    // Set robot to new state
                    probot->SetDOFValues(qfill, KinBody::CLA_CheckLimits, vuseddofindices);
                    probot->SetDOFVelocities(vfill, KinBody::CLA_CheckLimits, vuseddofindices);
                    probot->GetLinkVelocities(endeffvels);
                    //AccelerationMapPtr externalaccelerations(new AccelerationMap());
                    probot->GetLinkAccelerations(dofaccelerations,endeffaccs);
                    endeffvellin = endeffvels.at(endeffindex).first;
                    endeffvelang = endeffvels.at(endeffindex).second;
                    endeffacclin = endeffaccs.at(endeffindex).first;
                    endeffaccang = endeffaccs.at(endeffindex).second;
                    Transform R = itmanipinfo->plink->GetTransform();
                    // For each point in checkpoints, compute its vel and acc and check whether they satisfy the manipulator constraints
                    //int itx=0;
                    FOREACH(itpoint,itmanipinfo->checkpoints){
                        Vector point = R.rotate(*itpoint);
                        if(_parameters->maxmanipspeed>0) {
                            Vector vpoint = endeffvellin + endeffvelang.cross(point);
                            if(sqrt(vpoint.lengthsqr3())>_parameters->maxmanipspeed) {
                                res = false;
                                break;
                            }
                        }
                        if(_parameters->maxmanipaccel>0) {
                            Vector apoint = endeffacclin + endeffvelang.cross(endeffvelang.cross(point)) + endeffaccang.cross(point);
                            if(sqrt(apoint.lengthsqr3())>_parameters->maxmanipaccel) {
                                res = false;
                                break;
                            }
                        }
                    }
                    if(!res) {
                        return false;
                    }
                }
            }
            return true;
        }
    }

    virtual int SegmentFeasible(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b, const ParabolicRamp::Vector& da,const ParabolicRamp::Vector& db, dReal timeelapsed, int options)
    {
        //_parameters->_setstatefn(a);
        if(_bmanipconstraints) {
            options = options | CFO_FillCheckedConfiguration;
            _constraintreturn.reset(new ConstraintFilterReturn());
        }
        int pathreturn = _parameters->CheckPathAllConstraints(a,b,da, db, timeelapsed, IT_OpenStart, options, _constraintreturn);
        if( pathreturn != 0 ) {
            if( pathreturn & CFO_CheckTimeBasedConstraints ) {
                // time-related
            }
            return pathreturn;
        }
        // Test for collision and/or dynamics has succeeded, now test for manip constraint
        if( _bmanipconstraints && (options & CFO_CheckTimeBasedConstraints) ) {
            if( !CheckManipConstraints(a,b,da, db, timeelapsed) ) {
                return CFO_CheckTimeBasedConstraints;
            }
        }
        return 0;
    }

/*
   def ComputeDistanceToEnvironment(const std::vector<dReal>& vdofvalues):
    """robot state is not saved and environment is not locked
    """
    env=self.robot.GetEnv()
    pqpchecker = RaveCreateCollisionChecker(env,'pqp')
    if pqpchecker is not None:
        oldchecker = env.GetCollisionChecker()
        try:
            env.SetCollisionChecker(pqpchecker)
            with CollisionOptionsStateSaver(pqpchecker, CollisionOptions.Distance|CollisionOptions.Contacts):
                self.robot.GetGrabbed()
                checklinks = set()
                for manip in self.robot.GetManipulators():
                    for childlink in  manip.GetChildLinks():
                        checklinks.add(childlink)
                for link in self.robot.GetLinks():
                    if not link in checklinks:
                        link.Enable(False)
                report=CollisionReport()
                distancevalues=[]
                for dofvalues in dofvaluess:
                    self.robot.SetDOFValues(dofvalues)
                    check = env.CheckCollision(self.robot, report=report)
                    distancevalues.append([report.minDistance,report.contacts[0]])
                return distancevalues

        finally:
            env.SetCollisionChecker(oldchecker)
 */

    virtual bool NeedDerivativeForFeasibility()
    {
        // always enable since CheckPathAllConstraints needs to interpolate quadratically
        return true;
    }

    virtual ParabolicRamp::Real Rand()
    {
        return _uniformsampler->SampleSequenceOneReal(IT_OpenEnd);
    }

protected:
    ConstraintTrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _uniformsampler;
    RobotBasePtr _probot;
    CollisionCheckerBasePtr _distancechecker;
    ConstraintFilterReturnPtr _constraintreturn;
    //boost::shared_ptr<ConfigurationSpecification::SetConfigurationStateFn> _setstatefn, _setvelstatefn;
    //boost::shared_ptr<ConfigurationSpecification::GetConfigurationStateFn> _getstatefn, _getvelstatefn;
    std::vector<dReal> _vtrajpointscache;

    std::list< ManipConstraintInfo > _listCheckManips;
    TrajectoryBasePtr _dummytraj;
    bool _bCheckControllerTimeStep; ///< if set to true (default), then constraints all switch points to be a multiple of _parameters->_fStepLength
    bool _bmanipconstraints; /// if true, check workspace manip constraints
    PlannerProgress _progress;

private:
    std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> > __tempramps1d;
};


PlannerBasePtr CreateConstraintParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new ConstraintParabolicSmoother(penv,sinput));
}

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(ParabolicRamp::ParabolicRamp1D)
BOOST_TYPEOF_REGISTER_TYPE(ParabolicRamp::ParabolicRampND)
#endif
