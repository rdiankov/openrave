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
#include "plugindefs.h"
#include <fstream>

#include <openrave/planningutils.h>

#include "ParabolicPathSmooth/DynamicPath.h"
#include "mergewaypoints.h"


namespace ParabolicRamp = ParabolicRampInternal;

class ConstraintParabolicSmoother : public PlannerBase, public ParabolicRamp::FeasibilityCheckerBase, public ParabolicRamp::RandomNumberGeneratorBase
{
    struct ManipConstraintInfo
    {
        KinBody::LinkPtr plink;  // the end-effector link of the manipulator
        std::list<Vector> checkpoints;  // the points to check for in the link coordinate system
    };


    AABB ComputeGlobalAABB(std::list<KinBody::LinkPtr> linklist){
        Vector vmin, vmax;
        bool binitialized=false;
        AABB ab;
        FOREACHC(itlink,linklist) {
            ab = (*itlink)->ComputeAABB();
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
        BOOST_ASSERT(binitialized); // the linklist must at least contain the end effector
        ab.pos = (dReal)0.5 * (vmin + vmax);
        ab.extents = vmax - ab.pos;
        return ab;
    }


    void AABBtoCheckPoints(AABB ab, Transform T, std::list<Vector>& checkpoints){
        Vector incr;
        Transform Tinv = T.inverse();
        checkpoints.resize(0);
        for(int i=0; i<8; i++) {
            incr = ab.extents;
            switch(i) {
            case 0:
                break;
            case 1:
                incr.z = -incr.z;
                break;
            case 2:
                incr.y = -incr.y;
                break;
            case 3:
                incr.y = -incr.y;
                incr.z = -incr.z;
                break;
            case 4:
                incr.x = -incr.x;
                break;
            case 5:
                incr.x = -incr.x;
                incr.z = -incr.z;
                break;
            case 6:
                incr.x = -incr.x;
                incr.y = -incr.y;
                break;
            default:
                incr.x = -incr.x;
                incr.y = -incr.y;
                incr.z = -incr.z;
            }
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
                    // go through every manipulator and see if it depends on vusedindices
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
                            if((*itmanip)->IsGrabbing(*itbody)) {
                                FOREACH(itlink,(*itbody)->GetLinks()){
                                    globallinklist.push_back(*itlink);
                                }
                            }
                        }
                        AABB globalaabb = ComputeGlobalAABB(globallinklist);
                        // add the manip constraints
                        _listCheckManips.push_back(ManipConstraintInfo());
                        _listCheckManips.back().plink = endeffector;
                        std::list<Vector> checkpoints;
                        AABBtoCheckPoints(globalaabb,endeffector->GetTransform(),checkpoints);
                        // cout << "[";
                        // FOREACH(itcp, checkpoints) {
                        //     cout << "["<< itcp->x << "," << itcp->y << "," << itcp->z << "],\n";
                        // }
                        // cout <<"]\n";
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

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        BOOST_ASSERT(!!_parameters && !!ptraj);
        if( ptraj->GetNumWaypoints() < 2 ) {
            return PS_Failed;
        }

        _inittraj = ptraj;

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
        _setstatefn = posspec.GetSetFn(GetEnv());
        ConfigurationSpecification velspec = posspec.ConvertToVelocitySpecification();
        _setvelstatefn = velspec.GetSetFn(GetEnv());
        ConfigurationSpecification timespec;
        timespec.AddDeltaTimeGroup();

        vector<dReal> vtrajpoints;
        ptraj->GetWaypoints(0,ptraj->GetNumWaypoints(),vtrajpoints,posspec);

        try {
            ParabolicRamp::Vector tol = _parameters->_vConfigResolution;
            FOREACH(it,tol) {
                *it *= _parameters->_pointtolerance;
            }
            ParabolicRamp::RampFeasibilityChecker checker(this,tol);

            RAVELOG_VERBOSE_FORMAT("minswitchtime = %f, steplength=%f\n",_parameters->minswitchtime%_parameters->_fStepLength);


            /////////////////////////////////////////////////////////////////////////
            /////////////////////////  Convert to ramps /////////////////////////////
            /////////////////////////////////////////////////////////////////////////

            std::list<ParabolicRamp::ParabolicRampND> ramps,ramps2;

            std::vector<ConfigurationSpecification::Group>::const_iterator itcompatposgroup = ptraj->GetConfigurationSpecification().FindCompatibleGroup(posspec._vgroups.at(0), false);
            OPENRAVE_ASSERT_FORMAT(itcompatposgroup != ptraj->GetConfigurationSpecification()._vgroups.end(), "failed to find group %s in passed in trajectory", posspec._vgroups.at(0).name, ORE_InvalidArguments);


            ////////////////// Case 1 : Initial traj is quadratic ////////////////////

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
                dReal upperbound = 2 * mergewaypoints::ComputeRampsDuration(ramps);
                dReal stepsize = 0.1;
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
                    res = mergewaypoints::IterativeMergeRampsNoDichotomy(ramps,ramps2, _parameters, upperbound, stepsize, _bCheckControllerTimeStep, _uniformsampler,checker,options);
                }
                if(!res) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("Could not obtain a feasible trajectory from initial quadratic trajectory",ORE_Assert);
                }
                RAVELOG_DEBUG("Cool: obtained a feasible trajectory from initial quadratic trajectory\n");
                ramps.swap(ramps2);
            }


            ////////////////// Case 2 : Initial traj is linear ////////////////////////

            else {
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
                        if( RaveFabs(dotproduct * dotproduct - x0length2*x1length2) < 1e-8 ) {
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
                SetMilestones(ramps, path,checker);
            }


            /////////////////////////////////////////////////////////////////////////
            ////////////////////////////  Shortcutting //////////////////////////////
            /////////////////////////////////////////////////////////////////////////

            // Break into unitary ramps
            mergewaypoints::BreakIntoUnitaryRamps(ramps);

            // Sanity check before any shortcutting
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                RAVELOG_VERBOSE("Sanity check before starting shortcutting...\n");
                int options = 0xffff;
                FOREACHC(itramp,ramps){
                    if(!checker.Check(*itramp,options)) {
                        string filename = str(boost::format("%s/failedsmoothing%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%10000));
                        RAVELOG_WARN(str(boost::format("Original traj invalid, writing to %s")%filename));
                        ofstream f(filename.c_str());
                        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                        ptraj->serialize(f);
                        throw OPENRAVE_EXCEPTION_FORMAT0("Original ramps invalid!", ORE_Assert);
                    }
                }
            }

            // Reset all ramps
            FOREACH(itramp, ramps) {
                itramp->modified = false;
            }

            // Log before shortcutting
            //RAVELOG_DEBUG("Ramps before shortcutting\n");
            mergewaypoints::PrintRamps(ramps,_parameters,_bCheckControllerTimeStep);
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
                    RAVELOG_DEBUG_FORMAT("Start shortcut cycle %d\n",rep);
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
                    return PS_Interrupted;
                }


                // Log after shortcutting
                RAVELOG_DEBUG("Ramps after shortcutting\n");
                mergewaypoints::PrintRamps(ramps,_parameters,_bCheckControllerTimeStep);
                totaltime = mergewaypoints::ComputeRampsDuration(ramps);



                //////////////////////////////////////////////////////////////////////////
                //////////////////////  Further merge if possible ////////////////////////
                //////////////////////////////////////////////////////////////////////////

                int options = 0xffff;
                dReal upperbound = totaltime * 1.05;
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


            ///////////////////////////////////////////////////////////////////////////
            ////////////////// Convert back to Rave Trajectory ////////////////////////
            ///////////////////////////////////////////////////////////////////////////

            ConfigurationSpecification newspec = posspec;
            newspec.AddDerivativeGroups(1,true);
            int waypointoffset = newspec.AddGroup("iswaypoint", 1, "next");

            if( ramps.size() == 0 ) {
                // most likely a trajectory with the same start and end points were given, so return a similar trajectory
                // get the first point
                vtrajpoints.resize(posspec.GetDOF());
                ptraj->GetWaypoint(0,vtrajpoints,posspec);
                ptraj->Init(newspec);
                ptraj->Insert(0, vtrajpoints, posspec);
                return PS_HasSolution;
            }

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
            if( !_dummytraj || _dummytraj->GetXMLId() != ptraj->GetXMLId() ) {
                _dummytraj = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
            }
            _dummytraj->Init(newspec);


            // separate all the acceleration switches into individual points
            vtrajpoints.resize(newspec.GetDOF());
            ConfigurationSpecification::ConvertData(vtrajpoints.begin(),newspec,ramps.front().x0.begin(),posspec,1,GetEnv(),true);
            ConfigurationSpecification::ConvertData(vtrajpoints.begin(),newspec,ramps.front().dx0.begin(),velspec,1,GetEnv(),false);
            vtrajpoints.at(waypointoffset) = 1;
            vtrajpoints.at(timeoffset) = 0;
            _dummytraj->Insert(_dummytraj->GetNumWaypoints(),vtrajpoints);
            ParabolicRamp::Vector vconfig;
            // TODO ramps are unitary so don't have to track switch points
            FOREACH(itrampnd,ramps) {
                // double-check the current ramps
                if(_parameters->verifyinitialpath) {
                    // part of original trajectory which might not have been processed with perturbations, so ignore them
                    int options = 0xffff; // no perturbation
                    if( !checker.Check(*itrampnd,options)) {
                        throw OPENRAVE_EXCEPTION_FORMAT0("original ramp does not satisfy constraints!", ORE_Assert);
                    }
                    _progress._iteration+=1;
                    if( _CallCallbacks(_progress) == PA_Interrupt ) {
                        return PS_Interrupted;
                    }
                }

                if( itrampnd->ramps.at(0).tswitch1 > 0 && itrampnd->ramps.at(0).tswitch1 < itrampnd->endTime-g_fEpsilonLinear ) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("ramp is not unitary", ORE_Assert);
                }
                if( itrampnd->ramps.at(0).tswitch2 > 0 && itrampnd->ramps.at(0).tswitch2 < itrampnd->endTime-g_fEpsilonLinear ) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("ramp is not unitary", ORE_Assert);
                }

                vtrajpoints.resize(newspec.GetDOF());
                vector<dReal>::iterator ittargetdata = vtrajpoints.begin();
                ConfigurationSpecification::ConvertData(ittargetdata,newspec,itrampnd->x1.begin(),posspec,1,GetEnv(),true);
                ConfigurationSpecification::ConvertData(ittargetdata,newspec,itrampnd->dx1.begin(),velspec,1,GetEnv(),false);
                *(ittargetdata+timeoffset) = itrampnd->endTime;
                *(ittargetdata+waypointoffset) = 1;
                ittargetdata += newspec.GetDOF();
                _dummytraj->Insert(_dummytraj->GetNumWaypoints(),vtrajpoints);
            }

            OPENRAVE_ASSERT_OP(RaveFabs(totaltime-_dummytraj->GetDuration()),<,0.001);
            RAVELOG_DEBUG_FORMAT("after shortcutting %d times: path waypoints=%d, traj waypoints=%d, traj time=%fs", numshortcuts%ramps.size()%_dummytraj->GetNumWaypoints()%totaltime);
            ptraj->Swap(_dummytraj);
        }
        catch (const std::exception& ex) {
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                string filename = str(boost::format("%s/failedsmoothing%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%10000));
                RAVELOG_WARN(str(boost::format("parabolic planner failed: %s, writing original trajectory to %s")%ex.what()%filename));
                ofstream f(filename.c_str());
                f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                ptraj->serialize(f);
            }
            else {
                RAVELOG_WARN(str(boost::format("parabolic planner failed: %s")%ex.what()));
            }
            return PS_Failed;
        }
        RAVELOG_DEBUG(str(boost::format("path optimizing - computation time=%fs\n")%(0.001f*(float)(utils::GetMilliTime()-basetime))));
        return PS_HasSolution;
    }


    void SetMilestones(std::list<ParabolicRamp::ParabolicRampND>& ramps, const vector<ParabolicRamp::Vector>& x, ParabolicRamp::RampFeasibilityChecker& check){

        ramps.clear();
        if(x.size()==1) {
            ramps.push_back(ParabolicRamp::ParabolicRampND());
            ramps.front().SetConstant(x[0]);
        }
        else if( x.size() > 1 ) {
            int options = 0xffff; // no perturbation
            if(!_parameters->verifyinitialpath) {
                RAVELOG_WARN("Initial path verification is disabled (in SetMilestones)\n");
                options = options & (~CFO_CheckEnvCollisions) & (~CFO_CheckSelfCollisions); // no collision checking
            }
            for(size_t i=0; i+1<x.size(); i++) {
                std::list<ParabolicRamp::ParabolicRampND> tmpramps0, tmpramps1;
                bool cansetmilestone = mergewaypoints::ComputeLinearRampsWithConstraints(tmpramps0,x[i],x[i+1],_parameters,check,options);
                if( !cansetmilestone ) {
                    string filename = str(boost::format("%s/inittraj%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%10000));
                    RAVELOG_DEBUG_FORMAT("Writing original traj to %s", filename);
                    ofstream f(filename.c_str());
                    f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                    _inittraj->serialize(f);
                    throw OPENRAVE_EXCEPTION_FORMAT("linear ramp %d-%d failed to pass constraints", i%(i+1), ORE_Assert);
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
    }

// Check whether the shortcut end points (t1,t2) are similar to already attempted shortcut end points
    bool hasbeenattempted(dReal t1,dReal t2,std::list<dReal>& t1list,std::list<dReal>& t2list,dReal shortcutinnovationthreshold){

        if(t1list.size()==0) {
            return false;
        }
        std::list<dReal>::iterator it1 = t1list.begin(), it2 = t2list.begin();
        while(it1!=t1list.end()) {
            if(RaveFabs(*it1-t1)<=shortcutinnovationthreshold+g_fEpsilonLinear && RaveFabs(*it2-t2)<=shortcutinnovationthreshold+g_fEpsilonLinear) {
                return true;
            }
            it1++;
            it2++;
        }
        return false;
    }

// Perform the shortcuts
    int Shortcut(std::list<ParabolicRamp::ParabolicRampND>&ramps, int numIters, ParabolicRamp::RampFeasibilityChecker& check,ParabolicRamp::RandomNumberGeneratorBase* rng)
    {
        int shortcuts = 0;
        std::list<ParabolicRamp::ParabolicRampND> saveramps;
        //std::map<int, std::list<int> > mapTestedTimeRanges; // (starttime, list of end times) pairs where the end times are always > than starttime
        std::vector<dReal> rampStartTime; rampStartTime.resize(ramps.size());
        std::list<dReal> t1list, t2list;
        dReal currenttrajduration=0;
        int i = 0;
        FOREACH(itramp, ramps) {
            rampStartTime[i++] = currenttrajduration;
            currenttrajduration += itramp->endTime;
        }
        ParabolicRamp::Vector x0,x1,dx0,dx1;
        std::list<ParabolicRamp::ParabolicRampND> intermediate;
        std::list<ParabolicRamp::ParabolicRampND>::iterator itramp1, itramp2;

        dReal fStepLength = _parameters->_fStepLength;
        dReal shortcutinnovationthreshold = 0;
        dReal fimprovetimethresh = _parameters->_fStepLength;

        // Iterative shortcutting
        for(int iters=0; iters<numIters; iters++) {

            dReal t1=rng->Rand()*currenttrajduration,t2=rng->Rand()*currenttrajduration;
            if( iters == 0 ) {
                t1 = 0;
                t2 = currenttrajduration;
            }
            else {
                // round t1 and t2 to the closest multiple of fStepLength
                t1 = floor(t1/fStepLength+0.5)*fStepLength;
                t2 = floor(t2/fStepLength+0.5)*fStepLength;
                // check whether the shortcut is close to a previously attempted shortcut
                if(hasbeenattempted(t1,t2,t1list,t2list,shortcutinnovationthreshold)) {
                    RAVELOG_VERBOSE_FORMAT("Iter %d: Shortcut (%f,%f) already attempted\n",iters%t1%t2);
                    continue;
                }
                else{
                    RAVELOG_VERBOSE_FORMAT("Iter %d: Attempt shortcut (%f,%f)...\n",iters%t1%t2);
                    t1list.push_back(t1);
                    t2list.push_back(t2);
                }
            }
            if(t1 > t2) {
                std::swap(t1,t2);
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
                RAVELOG_VERBOSE("shortcut iter=%d rejected time=%fs\n", iters, currenttrajduration-(t2-t1)+newramptime);
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
                dReal upperbound = currenttrajduration-fimprovetimethresh;
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
                    if(durationaftermerge>currenttrajduration-fimprovetimethresh) {
                        resmerge = false;
                        RAVELOG_VERBOSE("... Duration did not significantly improve after merger\n");
                    }
                    else{
                        // Now check for collision (with perturbation), only for the ramps that have been modified
                        int itx = 0;
                        options = 0xffff | CFO_CheckWithPerturbation;
                        FOREACH(itramp, resramps) {
                            if(!itramp->modified) {
                                continue;
                            }
                            bool passed = true;
                            if (itx==0) {
                                passed = mergewaypoints::SpecialCheckRamp(*itramp,check,1,options);
                            }
                            else if (itx==(int)resramps.size()-1) {
                                passed = mergewaypoints::SpecialCheckRamp(*itramp,check,-1,options);
                            }
                            else {
                                passed = check.Check(*itramp,options);
                            }
                            if(!passed) {
                                RAVELOG_VERBOSE_FORMAT("... Collision for ramp %d after merge\n",itx);
                                resmerge = false;
                                break;
                            }
                            itx++;
                        }
                        if(resmerge) {
                            ramps.swap(resramps);
                            FOREACH(itramp, ramps) {
                                itramp->modified = false;
                            }
                            shortcuts++;
                            RAVELOG_DEBUG_FORMAT("... Duration after shortcut and merger: %f\n",durationaftermerge);
                            t1list.resize(0);
                            t2list.resize(0);
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
            currenttrajduration=0;
            i = 0;
            FOREACH(itramp, ramps) {
                rampStartTime[i++] = currenttrajduration;
                currenttrajduration += itramp->endTime;
            }
        } // end shortcutting loop

        return shortcuts;
    }


    virtual bool ConfigFeasible(const ParabolicRamp::Vector& a, const ParabolicRamp::Vector& da, int options)
    {
        if( _parameters->CheckPathAllConstraints(a,a, da, da, 0, IT_OpenStart, options) != 0 ) {
            return false;
        }
        return true;
    }

/** \brief return true if all the links in _listCheckManipssatisfy the acceleration and velocity constraints

   |w x (R x_i) + v| <= thresh

 */
    // virtual bool _CheckConstraintManips() const {
    //     FOREACHC(itinfo, _listCheckManips) {
    //     }
    //     return true;
    // }


    // Small function to deal with used indices
    void FillNonUsedDofs(RobotBasePtr probot, std::vector<dReal>& vconfigvalues, std::vector<dReal>& vdofvalues){
        vdofvalues.resize(probot->GetDOF());
        std::vector<int> vuseddofindices, vconfigindices;
        _parameters->_configurationspecification.ExtractUsedIndices(probot, vuseddofindices, vconfigindices);
        for(size_t iuseddof = 0; iuseddof < vuseddofindices.size(); ++iuseddof) {
            vdofvalues[vuseddofindices[iuseddof]] = vconfigvalues[vconfigindices[iuseddof]];
        }
    }


    bool CheckManipConstraints(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b, const ParabolicRamp::Vector& da,const ParabolicRamp::Vector& db, dReal timeelapsed){
        ParabolicRamp::ParabolicRampND ramp;
        if(timeelapsed>g_fEpsilonLinear) {
            ramp.SetPosVelTime(a,da,b,db,timeelapsed);
            // Compute manually the acceleration values since ramp.Accel is not trustworthy at the borders
            vector<dReal> ac;
            ramp.Accel(timeelapsed/2,ac);
            bool res = true;
            FOREACH(it,_constraintreturn->_configurationtimes){
                vector<dReal> qc,vc;
                ramp.Evaluate(*it,qc);
                ramp.Derivative(*it,vc);
                // Compute the velocity and accel of the end effector COM
                FOREACH(itmanipinfo,_listCheckManips){
                    RobotBasePtr probot = RaveInterfaceCast<RobotBase>(itmanipinfo->plink->GetParent());
                    Vector gravity = probot->GetEnv()->GetPhysicsEngine()->GetGravity();
                    vector<dReal> qfill,vfill,afill;
                    FillNonUsedDofs(probot,qc,qfill);
                    FillNonUsedDofs(probot,vc,vfill);
                    FillNonUsedDofs(probot,ac,afill);
                    std::vector<std::pair<Vector,Vector> > endeffvels,endeffaccs;
                    Vector endeffvellin,endeffvelang,endeffacclin,endeffaccang;
                    std::vector<dReal> q0,v0;
                    int endeffindex = itmanipinfo->plink->GetIndex();
                    probot->GetDOFValues(q0);
                    probot->GetDOFVelocities(v0);
                    // Set robot to new state
                    probot->SetDOFValues(qfill);
                    probot->SetDOFVelocities(vfill);
                    probot->GetLinkVelocities(endeffvels);
                    probot->GetLinkAccelerations(afill,endeffaccs);
                    endeffvellin = endeffvels[endeffindex].first;
                    endeffvelang = endeffvels[endeffindex].second;
                    endeffacclin = endeffaccs[endeffindex].first + gravity;
                    endeffaccang = endeffaccs[endeffindex].second;
                    Transform R = itmanipinfo->plink->GetTransform();
                    // For each point in checkpoints, compute its vel and acc and check whether they satisfy the manipulator constraints
                    FOREACH(itpoint,itmanipinfo->checkpoints){
                        Vector point = R.rot*(*itpoint);
                        if(_parameters->maxmanipspeed>0) {
                            Vector vpoint = endeffvellin + endeffvelang.cross(point);
                            if(sqrt(vpoint.lengthsqr3()>_parameters->maxmanipspeed)) {
                                res = false;
                                break;
                            }
                        }
                        if(_parameters->maxmanipaccel>0) {
                            Vector apoint = endeffacclin + endeffvelang.cross(endeffvelang.cross(point)) + endeffaccang.cross(point);
                            if(sqrt(apoint.lengthsqr3()>_parameters->maxmanipaccel)) {
                                //cout << "Accel violation: " << sqrt(apoint.lengthsqr3()) << "\n";
                                res = false;
                                break;
                            }
                        }
                    }
                    // Restore initial state of the robot
                    probot->SetActiveDOFValues(q0);
                    probot->SetActiveDOFVelocities(v0);
                    if(!res) {
                        return false;
                    }
                }
            }
            return true;
        }
        // Check config only
        return true;
    }

    virtual bool SegmentFeasible(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b, const ParabolicRamp::Vector& da,const ParabolicRamp::Vector& db, dReal timeelapsed, int options)
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
            return false;
        }
        // Test for collision and/or dynamics has succeeded, now test for manip constraint
        return !_bmanipconstraints || CheckManipConstraints(a,b,da, db, timeelapsed);
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
    boost::shared_ptr<ConfigurationSpecification::SetConfigurationStateFn> _setstatefn, _setvelstatefn;
//boost::shared_ptr<ConfigurationSpecification::GetConfigurationStateFn> _getstatefn, _getvelstatefn;

    std::list< ManipConstraintInfo > _listCheckManips;
    TrajectoryBasePtr _dummytraj,_inittraj;
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
