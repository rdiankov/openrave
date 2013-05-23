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
    struct LinkConstraintInfo
    {
        KinBody::LinkPtr plink;
        AABB ablocal; // local aabb of the link
    };

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
        _bUsePerturbation = true;
//        if( !_distancechecker->InitEnvironment() ) {
//            return false;
//        }
        _listCheckLinks.clear();
        if( _parameters->maxlinkspeed > 0 || _parameters->maxlinkaccel ) {
            // extract links from _parameters->_configurationspecification?
            //_listCheckLinks
            //_parameters->_configurationspecification
        }
        if( !_uniformsampler ) {
            _uniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
            OPENRAVE_ASSERT_FORMAT0(!!_uniformsampler, "need mt19937 space samplers", ORE_Assert);
        }

        // check and update minswitchtime
        _parameters->minswitchtime = ComputeStepSizeCeiling(_parameters->minswitchtime, _parameters->_fStepLength);
        if(_bCheckControllerTimeStep) {
            _parameters->minswitchtime = max(_parameters->minswitchtime, 5*_parameters->_fStepLength);
        }

        _uniformsampler->SetSeed(_parameters->_randomgeneratorseed);
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


        //Writing the incoming traj
        string filename = str(boost::format("%s/inittraj%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%10000));
        RAVELOG_WARN(str(boost::format("Writing original traj to %s")%filename));
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        ptraj->serialize(f);


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

        vector<ParabolicRamp::Vector> path;
        path.reserve(ptraj->GetNumWaypoints());
        vector<dReal> vtrajpoints;
        ptraj->GetWaypoints(0,ptraj->GetNumWaypoints(),vtrajpoints,posspec);

        try {
            ParabolicRamp::Vector tol = _parameters->_vConfigResolution;
            FOREACH(it,tol) {
                *it *= _parameters->_pointtolerance;
            }
            ParabolicRamp::RampFeasibilityChecker checker(this,tol);

            _bUsePerturbation = true;

            RAVELOG_DEBUG_FORMAT("Minswitchtime = %f\n",_parameters->minswitchtime);
            RAVELOG_DEBUG_FORMAT("Controller timestep = %f\n",_parameters->_fStepLength);



            /////////////////////////////////////////////////////////////////////////
            /////////////////////////  Convert to ramps /////////////////////////////
            /////////////////////////////////////////////////////////////////////////

            std::list<ParabolicRamp::ParabolicRampND> ramps,ramps2;

            std::vector<ConfigurationSpecification::Group>::const_iterator itcompatposgroup = ptraj->GetConfigurationSpecification().FindCompatibleGroup(posspec._vgroups.at(0), false);
            OPENRAVE_ASSERT_FORMAT(itcompatposgroup != ptraj->GetConfigurationSpecification()._vgroups.end(), "failed to find group %s in passed in trajectory", posspec._vgroups.at(0).name, ORE_InvalidArguments);


            ////////////////// Case 1 : Initial traj is quadratic ////////////////////

            // assumes that the traj has velocity data and is consistent, so convert the original trajectory in a sequence of ramps, and preserve velocity
            if (_parameters->_hastimestamps && itcompatposgroup->interpolation == "quadratic" ) {
                RAVELOG_DEBUG("Initial traj is piecewise quadratic\n");
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
                dReal upperbound = 10 * mergewaypoints::ComputeRampsDuration(ramps);
                dReal stepsize = 0.1;
                bool docheck = true;
                // Disable usePerturbation for this particular stage
                bool saveUsePerturbation = _bUsePerturbation;
                _bUsePerturbation = false;
                // Reset all ramps
                FOREACH(itramp, ramps) {
                    itramp->modified = true;
                }
                mergewaypoints::PrintRamps(ramps,_parameters,false);
                bool res = mergewaypoints::FixRampsEnds(ramps,ramps2, _parameters,checker);
                if(!res) {
                    res = mergewaypoints::IterativeMergeRampsNoDichotomy(ramps,ramps2, _parameters, upperbound, stepsize, _bCheckControllerTimeStep, _uniformsampler,checker,docheck);
                }
                _bUsePerturbation = saveUsePerturbation;
                // Reset all ramps
                FOREACH(itramp, ramps2) {
                    itramp->modified = false;
                }
                ramps.swap(ramps2);
                if(!res) {
                    throw OPENRAVE_EXCEPTION_FORMAT0("Could not obtain a feasible trajectory from initial quadratic trajectory",ORE_Assert);
                    return PS_Failed;
                }
                RAVELOG_DEBUG("Cool: obtained a feasible trajectory from initial quadratic trajectory\n");
            }


            ////////////////// Case 2 : Initial traj is linear ////////////////////////

            else {
                RAVELOG_DEBUG("Initial traj is piecewise linear\n");
                ParabolicRamp::Vector q(_parameters->GetDOF());
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
                        dReal d = 0;
                        for(size_t i = 0; i < q.size(); ++i) {
                            d += RaveFabs(q[i]-path.back().at(i));
                        }
                        if( d <= q.size()*std::numeric_limits<dReal>::epsilon() ) {
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
                bool saveUsePerturbation = _bUsePerturbation;
                _bUsePerturbation = false;
                FOREACHC(itramp,ramps){
                    if(!checker.Check(*itramp)) {

                        string filename = str(boost::format("%s/failedsmoothing%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%10000));
                        RAVELOG_WARN(str(boost::format("Original traj invalid, writing to %s")%filename));
                        ofstream f(filename.c_str());
                        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                        ptraj->serialize(f);
                        throw OPENRAVE_EXCEPTION_FORMAT0("Original ramps invalid!", ORE_Assert);
                    }
                }
                _bUsePerturbation = saveUsePerturbation;
            }

            // Reset all ramps
            FOREACH(itramp, ramps) {
                itramp->modified = false;
            }

            // Log before shortcutting
            RAVELOG_DEBUG("Ramps before shortcutting\n");
            mergewaypoints::PrintRamps(ramps,_parameters,_bCheckControllerTimeStep);
            dReal totaltime = mergewaypoints::ComputeRampsDuration(ramps);
            RAVELOG_DEBUG_FORMAT("initial path size=%d, duration=%f, pointtolerance=%f", path.size()%totaltime%_parameters->_pointtolerance);


            // Start shortcutting
            RAVELOG_DEBUG("\nStart shortcutting\n");
            _progress._iteration=0;
            int numshortcuts=0;
            dReal besttime = 1e10;
            std::list<ParabolicRamp::ParabolicRampND> bestramps,initramps;
            initramps = ramps;
            for(int rep=0; rep<_parameters->nshortcutcycles; rep++) {
                ramps = initramps;
                RAVELOG_DEBUG_FORMAT("\nStart shortcut cycle %d\n",rep);
                numshortcuts = Shortcut(ramps, _parameters->_nMaxIterations,checker, this);
                totaltime = mergewaypoints::ComputeRampsDuration(ramps);
                if(totaltime < besttime) {
                    bestramps = ramps;
                    besttime = totaltime;
                }
            }
            ramps = bestramps;

            RAVELOG_DEBUG("End shortcutting\n\n");
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

            bool docheck = true;
            dReal upperbound = totaltime * 1.05;
            std::list<ParabolicRamp::ParabolicRampND> resramps;
            bool resmerge = mergewaypoints::FurtherMergeRamps(ramps,resramps, _parameters, upperbound, _bCheckControllerTimeStep, _uniformsampler,checker,docheck);
            if(resmerge) {
                RAVELOG_DEBUG("Great, could further merge ramps!!\n");
                size_t nbrampsbefore = mergewaypoints::CountUnitaryRamps(ramps);
                size_t nbrampsafter = mergewaypoints::CountUnitaryRamps(resramps);
                dReal qualitybefore = mergewaypoints::quality(ramps);
                dReal qualityafter = mergewaypoints::quality(resramps);
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


            ///////////////////////////////////////////////////////////////////////////
            ////////////////// Convert back to Rave Trajectory ////////////////////////
            ///////////////////////////////////////////////////////////////////////////

            BOOST_ASSERT( ramps.size() > 0 );
            ConfigurationSpecification newspec = posspec;
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
                if(true) {
                    // part of original trajectory which might not have been processed with perturbations, so ignore them
                    _bUsePerturbation = false;
                    if( !checker.Check(*itrampnd)) {
                        throw OPENRAVE_EXCEPTION_FORMAT0("original ramp does not satisfy constraints!", ORE_Assert);
                    }
                    _bUsePerturbation = true; // re-enable
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


    inline bool SolveMinTimeWithConstraints(const ParabolicRamp::Vector& x0,const ParabolicRamp::Vector& dx0,const ParabolicRamp::Vector& x1,const ParabolicRamp::Vector& dx1, dReal curtime, ParabolicRamp::RampFeasibilityChecker& check, bool docheck, std::list<ParabolicRamp::ParabolicRampND>& rampsout)
    {
        __tempramps1d.resize(0);
        dReal mintime = ParabolicRamp::SolveMinTimeBounded(x0,dx0,x1,dx1, _parameters->_vConfigAccelerationLimit, _parameters->_vConfigVelocityLimit, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, __tempramps1d, _parameters->_multidofinterp);
        if(mintime < 0 || mintime > curtime ) {
            return false;
        }
        rampsout.clear();
        CombineRamps(__tempramps1d,rampsout);
        bool feas=true;
        if(docheck) {
            FOREACH(itramp, rampsout) {
                if(!check.Check(*itramp)) {
                    feas=false;
                    break;
                }
            }
        }
        return feas;
    }

    void SetMilestones(std::list<ParabolicRamp::ParabolicRampND>& ramps, const vector<ParabolicRamp::Vector>& x, ParabolicRamp::RampFeasibilityChecker& check){
        bool savebUsePerturbation = _bUsePerturbation;
        _bUsePerturbation = false;
        ramps.clear();
        if(x.size()==1) {
            ramps.push_back(ParabolicRamp::ParabolicRampND());
            ramps.front().SetConstant(x[0]);
        }
        else if( !x.empty() ) {
            for(size_t i=0; i+1<x.size(); i++) {
                ParabolicRamp::ParabolicRampND newramp;
                bool cansetmilestone = mergewaypoints::ComputeStraightRamp(newramp,x[i],x[i+1],_parameters,check);
                BOOST_ASSERT(cansetmilestone);
                std::list<ParabolicRamp::ParabolicRampND> tmpramps0, tmpramps1;
                tmpramps0.resize(0);
                tmpramps0.push_back(newramp);
                mergewaypoints::BreakIntoUnitaryRamps(tmpramps0);
                if(_bCheckControllerTimeStep) {
                    bool canscale = mergewaypoints::ScaleRampsTime(tmpramps0,tmpramps1,ComputeStepSizeCeiling(newramp.endTime,_parameters->_fStepLength*2)/newramp.endTime,false,_parameters);
                    BOOST_ASSERT(canscale);
                    ramps.splice(ramps.end(),tmpramps1);
                }
                else{
                    ramps.splice(ramps.end(),tmpramps0);
                }
            }
        }
        _bUsePerturbation = savebUsePerturbation;
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
        std::list<dReal> t1list,t2list;
        t1list.resize(0);
        t2list.resize(0);

        std::vector<dReal> rampStartTime; rampStartTime.resize(ramps.size());
        dReal currenttrajduration=0;
        int i = 0;
        FOREACH(itramp, ramps) {
            rampStartTime[i++] = currenttrajduration;
            currenttrajduration += itramp->endTime;
        }
        ParabolicRamp::Vector x0,x1,dx0,dx1;
        std::list<ParabolicRamp::ParabolicRampND> intermediate;
        std::list<ParabolicRamp::ParabolicRampND>::iterator itramp1, itramp2;

        dReal contrtime = _parameters->_fStepLength;
        dReal shortcutinnovationthreshold = 0.1;

        // Iterative shortcutting
        for(int iters=0; iters<numIters; iters++) {

            dReal t1=rng->Rand()*currenttrajduration,t2=rng->Rand()*currenttrajduration;
            if( iters == 0 ) {
                t1 = 0;
                t2 = currenttrajduration;
            }
            else {
                // round t1 and t2 to the closest multiple of fStepLength
                t1 = floor(t1/contrtime+0.5)*contrtime;
                t2 = floor(t2/contrtime+0.5)*contrtime;
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

            // Check collision at this stage only if we don't run mergewaypoints afterwards
            bool docheck = _parameters->minswitchtime == 0;
            bool res=SolveMinTimeWithConstraints(x0,dx0,x1,dx1,t2-t1, check, docheck, intermediate);

            if(!res) {
                RAVELOG_VERBOSE("... Could not SolveMinTime\n");
                continue;
            }

            // no idea what a good time improvement delta  is... _parameters->_fStepLength?
            dReal fimprovetimethresh = _parameters->_fStepLength;
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
                bool docheck = false;
                bool resmerge = mergewaypoints::IterativeMergeRamps(ramps,resramps, _parameters, upperbound, _bCheckControllerTimeStep, _uniformsampler,check,docheck);

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
                        // Now check for collision, only for the ramps that have been modified
                        int itx = 0;
                        FOREACH(itramp, resramps) {
                            if(itramp->modified && (!check.Check(*itramp))) {
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


    virtual bool ConfigFeasible(const ParabolicRamp::Vector& a, const ParabolicRamp::Vector& da)
    {
        if( _bUsePerturbation ) {
            // have to also test with tolerances!
            boost::array<dReal,3> perturbations = {{ 0,_parameters->_pointtolerance,-_parameters->_pointtolerance}};
            ParabolicRamp::Vector anew(a.size());
            FOREACH(itperturbation,perturbations) {
                for(size_t i = 0; i < a.size(); ++i) {
                    anew[i] = a[i] + *itperturbation * _parameters->_vConfigResolution.at(i);
                    if( anew[i] < _parameters->_vConfigLowerLimit.at(i) ) {
                        anew[i] = _parameters->_vConfigLowerLimit.at(i);
                    }
                    if( anew[i] > _parameters->_vConfigUpperLimit.at(i) ) {
                        anew[i] = _parameters->_vConfigUpperLimit.at(i);
                    }
                }
                (*_setstatefn)(anew);
                if( _parameters->CheckPathAllConstraints(a,a,da,da,0,IT_OpenStart) <= 0 ) {
                    return false;
                }
            }
        }
        else {
            if( _parameters->CheckPathAllConstraints(a,a, da, da, 0, IT_OpenStart) <= 0 ) {
                return false;
            }
        }
        return true;
    }

    /** \brief return true if all the links in _listCheckLinks satisfy the acceleration and velocity constraints

       |w x (R x_i) + v| <= thresh

     */
    virtual bool _CheckConstraintLinks() const {
        FOREACHC(itinfo, _listCheckLinks) {
        }
        return true;
    }

    virtual bool SegmentFeasible(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b, const ParabolicRamp::Vector& da,const ParabolicRamp::Vector& db, dReal timeelapsed)
    {
        if( _bUsePerturbation ) {
            // have to also test with tolerances!
            boost::array<dReal,3> perturbations = {{ 0,_parameters->_pointtolerance,-_parameters->_pointtolerance}};
            ParabolicRamp::Vector anew(a.size()), bnew(b.size());
            FOREACH(itperturbation,perturbations) {
                for(size_t i = 0; i < a.size(); ++i) {
                    anew[i] = a[i] + *itperturbation * _parameters->_vConfigResolution.at(i);
                    if( anew[i] < _parameters->_vConfigLowerLimit.at(i) ) {
                        anew[i] = _parameters->_vConfigLowerLimit.at(i);
                    }
                    if( anew[i] > _parameters->_vConfigUpperLimit.at(i) ) {
                        anew[i] = _parameters->_vConfigUpperLimit.at(i);
                    }
                    bnew[i] = b[i] + *itperturbation * _parameters->_vConfigResolution.at(i);
                    if( bnew[i] < _parameters->_vConfigLowerLimit.at(i) ) {
                        bnew[i] = _parameters->_vConfigLowerLimit.at(i);
                    }
                    if( bnew[i] > _parameters->_vConfigUpperLimit.at(i) ) {
                        bnew[i] = _parameters->_vConfigUpperLimit.at(i);
                    }
                }
                //(*_setstatefn)(anew);
                if( _parameters->CheckPathAllConstraints(anew,bnew,da, db, timeelapsed, IT_OpenStart) <= 0 ) {
                    return false;
                }
            }
        }
        else {
            //_parameters->_setstatefn(a);
            int pathreturn = _parameters->CheckPathAllConstraints(a,b,da, db, timeelapsed, IT_OpenStart);
            if( pathreturn <= 0 ) {
                if( pathreturn & 0x40000000 ) {
                    // time-related
                }
                return false;
            }
        }
        return true;
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
        return _parameters->maxlinkspeed > 0 || _parameters->maxlinkaccel > 0 || _parameters->velocitydistancethresh > 0;
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
    boost::shared_ptr<ConfigurationSpecification::SetConfigurationStateFn> _setstatefn, _setvelstatefn;
//boost::shared_ptr<ConfigurationSpecification::GetConfigurationStateFn> _getstatefn, _getvelstatefn;

    std::list< LinkConstraintInfo > _listCheckLinks;
    TrajectoryBasePtr _dummytraj;
    bool _bUsePerturbation; ///< if true, perterbs joint values a little before testing them for the constraints
    bool _bCheckControllerTimeStep; ///< if set to true (default), then constraints all switch points to be a multiple of _parameters->_fStepLength
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
