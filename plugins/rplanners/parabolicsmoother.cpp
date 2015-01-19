// -*- coding: utf-8 -*-
// Copyright (C) 2012-2015 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "openraveplugindefs.h"
#include <fstream>

#include <openrave/planningutils.h>

#include "ParabolicPathSmooth/DynamicPath.h"

namespace ParabolicRamp = ParabolicRampInternal;

class ParabolicSmoother : public PlannerBase, public ParabolicRamp::FeasibilityCheckerBase, public ParabolicRamp::RandomNumberGeneratorBase
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


    /// \brief Compute the AABB that encloses all the links in linklist with respect to a coordinate system Tparent
    ///
    /// \param tparent is most likely the end effector of a manipulator
    AABB ComputeEnclosingAABB(std::list<KinBody::LinkPtr> linklist, const Transform& tparent)
    {
        Vector vmin, vmax;
        bool binitialized=false;
        Transform tparentinv = tparent.inverse();
        FOREACHC(itlink,linklist) {
            AABB ablink = (*itlink)->ComputeLocalAABB(); // AABB of the link in its local coordinates
            Transform tdelta = tparentinv * (*itlink)->GetTransform();
            TransformMatrix tmdelta(tdelta);
            Vector vabsextents(RaveFabs(tmdelta.m[0])*ablink.extents[0] + RaveFabs(tmdelta.m[1])*ablink.extents[1] + RaveFabs(tmdelta.m[2])*ablink.extents[2],
                               RaveFabs(tmdelta.m[4])*ablink.extents[0] + RaveFabs(tmdelta.m[5])*ablink.extents[1] + RaveFabs(tmdelta.m[6])*ablink.extents[2],
                               RaveFabs(tmdelta.m[8])*ablink.extents[0] + RaveFabs(tmdelta.m[9])*ablink.extents[1] + RaveFabs(tmdelta.m[10])*ablink.extents[2]);
            Vector vcenter = tdelta*ablink.pos;
            Vector vnmin = vcenter - vabsextents;
            Vector vnmax = vcenter + vabsextents;
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
        AABB ab;
        ab.pos = (dReal)0.5 * (vmin + vmax);
        ab.extents = vmax - ab.pos;
        return ab;
    }

    /// \brief given a world AABB oriented, return its 8 vertices
    void ConvertAABBtoCheckPoints(AABB ab, std::list<Vector>& checkpoints)
    {
        dReal signextents[24] = {1,1,1,   1,1,-1,  1,-1,1,   1,-1,-1,  -1,1,1,   -1,1,-1,  -1,-1,1,   -1,-1,-1};
        Vector incr;
        //Transform Tinv = T.inverse();
        checkpoints.resize(0);
        for(int i=0; i<8; i++) {
            incr[0] = ab.extents[0] * signextents[3*i+0];
            incr[1] = ab.extents[1] * signextents[3*i+1];
            incr[2] = ab.extents[2] * signextents[3*i+2];
            checkpoints.push_back(ab.pos + incr);
        }
    }

public:
    ParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nInterface to `Indiana University Intelligent Motion Laboratory <http://www.iu.edu/~motion/software.html>`_ parabolic smoothing library (Kris Hauser).\n\n**Note:** The original trajectory will not be preserved at all, don't use this if the robot has to hit all points of the trajectory.\n";
        _bmanipconstraints = false;
        _constraintreturn.reset(new ConstraintFilterReturn());
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

        _bmanipconstraints = _parameters->manipname.size() > 0 && (_parameters->maxmanipspeed>0 || _parameters->maxmanipaccel>0);
        _listCheckManips.clear();

        // initialize workspace constraints on manipulators
        if(_bmanipconstraints ) {
            std::vector<KinBodyPtr> listUsedBodies;
            std::set<KinBody::LinkPtr> setCheckedManips;
            _parameters->_configurationspecification.ExtractUsedBodies(GetEnv(), listUsedBodies);
            FOREACH(itbody, listUsedBodies) {
                KinBodyPtr pbody = *itbody;
                if( pbody->IsRobot() ) {
                    RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
                    RobotBase::ManipulatorPtr pmanip = probot->GetManipulator(_parameters->manipname);
                    if( !!pmanip ) {
                        KinBody::LinkPtr endeffector = pmanip->GetEndEffector();
                        // Insert all child links of endeffector
                        std::list<KinBody::LinkPtr> globallinklist;
                        std::vector<KinBody::LinkPtr> vchildlinks;
                        pmanip->GetChildLinks(vchildlinks);
                        FOREACH(itlink,vchildlinks) {
                            globallinklist.push_back(*itlink);
                        }
                        // Insert all links of all bodies that the endeffector is grabbing
                        std::vector<KinBodyPtr> grabbedbodies;
                        probot->GetGrabbed(grabbedbodies);
                        FOREACH(itbody,grabbedbodies) {
                            if(pmanip->IsGrabbing(*itbody)) {
                                FOREACH(itlink,(*itbody)->GetLinks()) {
                                    globallinklist.push_back(*itlink);
                                }
                            }
                        }
                        // Compute the enclosing AABB and add its vertices to the checkpoints
                        AABB enclosingaabb = ComputeEnclosingAABB(globallinklist, endeffector->GetTransform());
                        _listCheckManips.push_back(ManipConstraintInfo());
                        _listCheckManips.back().plink = endeffector;
                        ConvertAABBtoCheckPoints(enclosingaabb, _listCheckManips.back().checkpoints);
                        setCheckedManips.insert(endeffector);
                    }

                    //std::vector<int> vuseddofindices, vusedconfigindices;
                    //_parameters->_configurationspecification.ExtractUsedIndices(probot, vuseddofindices, vusedconfigindices);
                    // go through every manipulator and check whether it depends on vusedindices
//                    FOREACH(itmanip, probot->GetManipulators()) {
//
//                        if( setCheckedManips.find(endeffector) != setCheckedManips.end() ) {
//                            // already checked, so don't do anything
//                            continue;
//                        }
//                        bool manipisaffected = false;
//                        FOREACH(itdofindex, vuseddofindices) {
//                            if( probot->DoesDOFAffectLink(*itdofindex, endeffector->GetIndex()) ) {
//                                manipisaffected = true;
//                                break;
//                            }
//                        }
//                        if (!manipisaffected) {
//                            // manip is not affected by trajectory, so don't do anything
//                            continue;
//                        }
//
//
//                    }
                }
            }
        }

        if( !_uniformsampler ) {
            _uniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        }
        _uniformsampler->SetSeed(_parameters->_nRandomGeneratorSeed);
        return !!_uniformsampler;
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

        _DumpTrajectory(ptraj, Level_Verbose);

        // save velocities
        std::vector<KinBody::KinBodyStateSaverPtr> vstatesavers;
        std::vector<KinBodyPtr> vusedbodies;
        _parameters->_configurationspecification.ExtractUsedBodies(GetEnv(), vusedbodies);
        if( vusedbodies.size() == 0 ) {
            RAVELOG_WARN("there are no used bodies in this configuration\n");
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

        uint32_t basetime = utils::GetMilliTime();
        ConfigurationSpecification posspec = _parameters->_configurationspecification;
        ConfigurationSpecification velspec = posspec.ConvertToVelocitySpecification();
        ConfigurationSpecification timespec;
        timespec.AddDeltaTimeGroup();

        std::vector<ConfigurationSpecification::Group>::const_iterator itcompatposgroup = ptraj->GetConfigurationSpecification().FindCompatibleGroup(posspec._vgroups.at(0), false);
        OPENRAVE_ASSERT_FORMAT(itcompatposgroup != ptraj->GetConfigurationSpecification()._vgroups.end(), "failed to find group %s in passed in trajectory", posspec._vgroups.at(0).name, ORE_InvalidArguments);

        ConstraintTrajectoryTimingParametersConstPtr parameters = boost::dynamic_pointer_cast<ConstraintTrajectoryTimingParameters const>(GetParameters());

        ParabolicRamp::DynamicPath dynamicpath;
        dynamicpath.Init(parameters->_vConfigVelocityLimit,parameters->_vConfigAccelerationLimit);
        dynamicpath._multidofinterp = _parameters->_multidofinterp;
        dynamicpath.SetJointLimits(parameters->_vConfigLowerLimit,parameters->_vConfigUpperLimit);

        ParabolicRamp::Vector q(_parameters->GetDOF());
        vector<dReal> vtrajpoints;
        if (_parameters->_hastimestamps && itcompatposgroup->interpolation == "quadratic" ) {
            RAVELOG_VERBOSE("Initial traj is piecewise quadratic\n");
            // assumes that the traj has velocity data and is consistent, so convert the original trajectory in a sequence of ramps, and preserve velocity
            vector<dReal> x0, x1, dx0, dx1, ramptime;
            ptraj->GetWaypoint(0,x0,posspec);
            ptraj->GetWaypoint(0,dx0,velspec);
            dynamicpath.ramps.resize(ptraj->GetNumWaypoints()-1);
            size_t iramp = 0;
            for(size_t i=0; i+1<ptraj->GetNumWaypoints(); i++) {
                ptraj->GetWaypoint(i+1,ramptime,timespec);
                if (ramptime.at(0) > g_fEpsilonLinear) {
                    ptraj->GetWaypoint(i+1,x1,posspec);
                    ptraj->GetWaypoint(i+1,dx1,velspec);
                    dynamicpath.ramps[iramp].SetPosVelTime(x0,dx0,x1,dx1,ramptime.at(0));
                    x0.swap(x1);
                    dx0.swap(dx1);
                    iramp += 1;
                }
//                else {
//                    RAVELOG_WARN("there is no ramp time, so making a linear ramp\n");
//                    dynamicpath.ramps[i].x0 = x0;
//                    dynamicpath.ramps[i].dx0 = dx0;
//                    ptraj->GetWaypoint(i+1,dynamicpath.ramps[i].x1,posspec);
//                    ptraj->GetWaypoint(i+1,dynamicpath.ramps[i].dx1,velspec);
//                    bool res=dynamicpath.ramps[i].SolveMinTimeLinear(_parameters->_vConfigAccelerationLimit, _parameters->_vConfigVelocityLimit);
//                    PARABOLIC_RAMP_ASSERT(res && dynamicpath.ramps[i].IsValid());
//                    x0 = dynamicpath.ramps[i].x1;
//                    dx0 = dynamicpath.ramps[i].dx0;
//                }
            }
            dynamicpath.ramps.resize(iramp);
        }
        else {
            vector<ParabolicRamp::Vector> path;
            path.reserve(ptraj->GetNumWaypoints());
            // linear piecewise trajectory
            ptraj->GetWaypoints(0,ptraj->GetNumWaypoints(),vtrajpoints,_parameters->_configurationspecification);
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
                    if( RaveFabs(dotproduct * dotproduct - x0length2*x1length2) < 100*ParabolicRamp::EpsilonX*ParabolicRamp::EpsilonX ) {
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
            //dynamicpath.SetMilestones(path);   //now the trajectory starts and stops at every milestone
            if( !_SetMilestones(dynamicpath.ramps, path) ) {
                RAVELOG_WARN("failed to initialize ramps\n");
                return PS_Failed;
            }
        }

        if( !_parameters->verifyinitialpath ) {
            // disable verification
            FOREACH(itramp, dynamicpath.ramps) {
                itramp->constraintchecked = 1;
            }
        }

        try {
            _bUsePerturbation = true;
            RAVELOG_DEBUG_FORMAT("env=%d, initial path size=%d, duration=%f, pointtolerance=%f, multidof=%d, maxmanipspeed=%f, maxmanipaccel=%f", GetEnv()->GetId()%dynamicpath.ramps.size()%dynamicpath.GetTotalTime()%parameters->_pointtolerance%parameters->_multidofinterp%parameters->maxmanipspeed%parameters->maxmanipaccel);
            ParabolicRamp::Vector tol = parameters->_vConfigResolution;
            FOREACH(it,tol) {
                *it *= parameters->_pointtolerance;
            }
            ParabolicRamp::RampFeasibilityChecker checker(this,tol);

            _progress._iteration = 0;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return PS_Interrupted;
            }

            int numshortcuts=0;
            if( !!parameters->_setstatevaluesfn || !!parameters->_setstatefn ) {
                // no idea what a good mintimestep is... _parameters->_fStepLength*0.5?
                //numshortcuts = dynamicpath.Shortcut(parameters->_nMaxIterations,checker,this, parameters->_fStepLength*0.99);
                numshortcuts = _Shortcut(dynamicpath, parameters->_nMaxIterations,checker,this, parameters->_fStepLength*0.99);
                if( numshortcuts < 0 ) {
                    return PS_Interrupted;
                }
            }

            ++_progress._iteration;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return PS_Interrupted;
            }

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
            ConfigurationSpecification::ConvertData(vtrajpoints.begin(),newspec,dynamicpath.ramps.at(0).x0.begin(), posspec,1,GetEnv(),true);
            ConfigurationSpecification::ConvertData(vtrajpoints.begin(),newspec,dynamicpath.ramps.at(0).dx0.begin(),velspec,1,GetEnv(),false);
            vtrajpoints.at(waypointoffset) = 1;
            vtrajpoints.at(timeoffset) = 0;
            _dummytraj->Insert(_dummytraj->GetNumWaypoints(),vtrajpoints);
            vector<dReal> vswitchtimes;
            ParabolicRamp::Vector vconfig;
            std::vector<ParabolicRamp::ParabolicRampND> temprampsnd;
            ParabolicRamp::ParabolicRampND rampndtrimmed;
            dReal fTrimEdgesTime = parameters->_fStepLength*2; // 2 controller timesteps is enough?
            for(size_t irampindex = 0; irampindex < dynamicpath.ramps.size(); ++irampindex) {
                const ParabolicRamp::ParabolicRampND& rampnd = dynamicpath.ramps[irampindex];
                temprampsnd.resize(1);
                temprampsnd[0] = rampnd;
                // double-check the current ramps, ignore first and last ramps since they connect to the initial and goal positions, and those most likely they cannot be fixed .
                if(!rampnd.constraintchecked ) {
                    //(irampindex > 0 && irampindex+1 < dynamicpath.ramps.size())
                    rampndtrimmed = rampnd;
                    bool bTrimmed = false;
                    bool bCheck = true;
                    if( irampindex == 0 ) {
                        if( rampnd.endTime <= fTrimEdgesTime+g_fEpsilonLinear ) {
                            // ramp is too short so ignore checking
                            bCheck = false;
                        }
                        else {
                            // don't check points close to the initial configuration because of jittering
                            rampndtrimmed.TrimFront(fTrimEdgesTime);
                            bTrimmed = true;
                        }
                    }
                    else if( irampindex+1 == dynamicpath.ramps.size() ) {
                        if( rampnd.endTime <= fTrimEdgesTime+g_fEpsilonLinear ) {
                            // ramp is too short so ignore checking
                            bCheck = false;
                        }
                        else {
                            // don't check points close to the final configuration because of jittering
                            rampndtrimmed.TrimBack(fTrimEdgesTime);
                            bTrimmed = true;
                        }
                    }
                    // part of original trajectory which might not have been processed with perturbations, so ignore perturbations
                    _bUsePerturbation = false;
                    if( bCheck && checker.Check(rampndtrimmed) != 0 ) {
                        std::vector<std::vector<ParabolicRamp::ParabolicRamp1D> > tempramps1d;
                        // try to time scale, perhaps collision and dynamics will change
                        // go all the way up to 2.0 multiplier: 1.05*1.1*1.15*1.2*1.25 ~= 2
                        bool bSuccess = false;
                        dReal mult = 1.05;
                        dReal endTime = rampndtrimmed.endTime;
                        for(size_t idilate = 0; idilate < 5; ++idilate ) {
                            tempramps1d.resize(0);
                            endTime *= mult;
                            if( ParabolicRamp::SolveAccelBounded(rampndtrimmed.x0, rampndtrimmed.dx0, rampndtrimmed.x1, rampndtrimmed.dx1, endTime,  parameters->_vConfigAccelerationLimit, parameters->_vConfigVelocityLimit, parameters->_vConfigLowerLimit, parameters->_vConfigUpperLimit, tempramps1d, _parameters->_multidofinterp) ) {
                                temprampsnd.resize(0);
                                CombineRamps(tempramps1d, temprampsnd);
                                if( irampindex == 0 ) {
                                    temprampsnd[0].TrimFront(fTrimEdgesTime);
                                }
                                else if( irampindex+1 == dynamicpath.ramps.size() ) {
                                    temprampsnd[0].TrimBack(fTrimEdgesTime);
                                }
                                bool bHasBadRamp=false;
                                FOREACH(itnewrampnd, temprampsnd) {
                                    if( checker.Check(*itnewrampnd) != 0 ) {
                                        bHasBadRamp = true;
                                        break;
                                    }
                                }
                                if( !bHasBadRamp ) {
                                    if( bTrimmed ) {
                                        // have to retime the original ramp without trimming
                                        if( !ParabolicRamp::SolveAccelBounded(rampnd.x0, rampnd.dx0, rampnd.x1, rampnd.dx1, endTime,  parameters->_vConfigAccelerationLimit, parameters->_vConfigVelocityLimit, parameters->_vConfigLowerLimit, parameters->_vConfigUpperLimit, tempramps1d, _parameters->_multidofinterp) ) {
                                            break;
                                        }
                                        temprampsnd.resize(0);
                                        CombineRamps(tempramps1d, temprampsnd);
                                    }
                                    bSuccess = true;
                                    break;
                                }
                                mult += 0.05;
                            }
                        }
                        if( !bSuccess ) {
                            RAVELOG_WARN_FORMAT("original ramp %d is in collision!", irampindex);
                            _DumpTrajectory(ptraj, Level_Verbose);
                            return PS_Failed;
                        }
                    }
                    _bUsePerturbation = true; // re-enable
                    ++_progress._iteration;
                    if( _CallCallbacks(_progress) == PA_Interrupt ) {
                        return PS_Interrupted;
                    }
                }

                FOREACH(itrampnd2, temprampsnd) {
                    vswitchtimes.resize(0);
                    vswitchtimes.push_back(itrampnd2->endTime);
                    if( _parameters->_outputaccelchanges ) {
                        FOREACHC(itramp,itrampnd2->ramps) {
                            vector<dReal>::iterator it;
                            if( itramp->tswitch1 != 0 ) {
                                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
                                if( it != vswitchtimes.end() && *it != itramp->tswitch1) {
                                    vswitchtimes.insert(it,itramp->tswitch1);
                                }
                            }
                            if( itramp->tswitch1 != itramp->tswitch2 && itramp->tswitch2 != 0 ) {
                                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
                                if( it != vswitchtimes.end() && *it != itramp->tswitch2 ) {
                                    vswitchtimes.insert(it,itramp->tswitch2);
                                }
                            }
                            if( itramp->ttotal != itramp->tswitch2 && itramp->ttotal != 0 ) {
                                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->ttotal);
                                if( it != vswitchtimes.end() && *it != itramp->ttotal ) {
                                    vswitchtimes.insert(it,itramp->ttotal);
                                }
                            }
                        }
                    }
                    vtrajpoints.resize(newspec.GetDOF()*vswitchtimes.size());
                    vector<dReal>::iterator ittargetdata = vtrajpoints.begin();
                    dReal prevtime = 0;
                    for(size_t i = 0; i < vswitchtimes.size(); ++i) {
                        rampnd.Evaluate(vswitchtimes[i],vconfig);
                        ConfigurationSpecification::ConvertData(ittargetdata,newspec,vconfig.begin(),posspec,1,GetEnv(),true);
                        rampnd.Derivative(vswitchtimes[i],vconfig);
                        ConfigurationSpecification::ConvertData(ittargetdata,newspec,vconfig.begin(),velspec,1,GetEnv(),false);
                        *(ittargetdata+timeoffset) = vswitchtimes[i]-prevtime;
                        *(ittargetdata+waypointoffset) = dReal(i+1==vswitchtimes.size());
                        ittargetdata += newspec.GetDOF();
                        prevtime = vswitchtimes[i];
                    }
                    _dummytraj->Insert(_dummytraj->GetNumWaypoints(),vtrajpoints);
                }
            }

            OPENRAVE_ASSERT_OP(RaveFabs(dynamicpath.GetTotalTime()-_dummytraj->GetDuration()),<,0.001);
            RAVELOG_DEBUG(str(boost::format("after shortcutting %d times: path waypoints=%d, traj waypoints=%d, traj time=%fs")%numshortcuts%dynamicpath.ramps.size()%_dummytraj->GetNumWaypoints()%dynamicpath.GetTotalTime()));
            ptraj->Swap(_dummytraj);
        }
        catch (const std::exception& ex) {
            _DumpTrajectory(ptraj, Level_Verbose);
            RAVELOG_WARN_FORMAT("parabolic planner failed: %s", ex.what());
            return PS_Failed;
        }
        RAVELOG_DEBUG_FORMAT("path optimizing - computation time=%fs\n", (0.001f*(float)(utils::GetMilliTime()-basetime)));
        return _ProcessPostPlanners(RobotBasePtr(),ptraj);
    }

    virtual int ConfigFeasible(const ParabolicRamp::Vector& a, const ParabolicRamp::Vector& da, int options)
    {
        if( _bUsePerturbation ) {
            options |= CFO_CheckWithPerturbation;
        }
        return _parameters->CheckPathAllConstraints(a,a, da, da, 0, IT_OpenStart, options);
    }

    virtual int SegmentFeasible(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b, const ParabolicRamp::Vector& da,const ParabolicRamp::Vector& db, dReal timeelapsed, int options)
    {
        if( _bUsePerturbation ) {
            options |= CFO_CheckWithPerturbation;
        }
        if(_bmanipconstraints) {
            options |= CFO_FillCheckedConfiguration;
            _constraintreturn->Clear();
        }
        try {
            int ret = _parameters->CheckPathAllConstraints(a,b,da, db, timeelapsed, IT_OpenStart, options, _constraintreturn);
            if( ret != 0 ) {
                return ret;
            }
        }
        catch(const std::exception& ex) {
            // some constraints assume initial conditions for a and b are followed, however at this point a and b are sa
            RAVELOG_VERBOSE_FORMAT("rrtparams path constraints threw an exception: %s", ex.what());
            return 0xffff; // could be anything
        }
        // Test for collision and/or dynamics has succeeded, now test for manip constraint
        if( _bmanipconstraints && (options & CFO_CheckTimeBasedConstraints) ) {
            if( !_CheckManipConstraints(a,b,da, db, timeelapsed) ) {
                return CFO_CheckTimeBasedConstraints;
            }
        }
        return 0;
    }

    virtual ParabolicRamp::Real Rand()
    {
        return _uniformsampler->SampleSequenceOneReal(IT_OpenEnd);
    }

    virtual bool NeedDerivativeForFeasibility()
    {
        // always enable since CheckPathAllConstraints needs to interpolate quadratically
        return true;
    }

protected:
    /// \brief converts a path of linear points to a ramp that initially satisfies the constraints
    bool _SetMilestones(std::vector<ParabolicRamp::ParabolicRampND>& ramps, const vector<ParabolicRamp::Vector>& vpath)
    {
        ramps.clear();
        if(vpath.size()==1) {
            ramps.push_back(ParabolicRamp::ParabolicRampND());
            ramps.front().SetConstant(vpath[0]);
        }
        else if( vpath.size() > 1 ) {
            // only check time based constraints since most of the collision checks here will change due to a different path. however it's important to have the ramp start with reasonable velocities/accelerations.
            int options = CFO_CheckTimeBasedConstraints;
            if(!_parameters->verifyinitialpath) {
                RAVELOG_VERBOSE("Initial path verification is disabled (in SetMilestones)\n");
                options = options & (~CFO_CheckEnvCollisions) & (~CFO_CheckSelfCollisions); // no collision checking
            }
            ramps.resize(vpath.size()-1);
            std::vector<dReal> vzero(vpath.at(0).size(), 0.0);
            std::vector<dReal> vellimits, accellimits;
            std::vector<dReal> vswitchtimes;
            std::vector<dReal> x0, x1, dx0, dx1;
            for(size_t i=0; i+1<vpath.size(); i++) {
                ParabolicRamp::ParabolicRampND& ramp = ramps[i];
                ramp.x0 = vpath[i];
                ramp.x1 = vpath[i+1];
                ramp.dx0 = vzero;
                ramp.dx1 = vzero;
                vellimits = _parameters->_vConfigVelocityLimit;
                accellimits = _parameters->_vConfigAccelerationLimit;
                dReal fmult = 0.9;
                int retseg = -1;
                for(size_t itry = 0; itry < 30; ++itry) {
                    bool res=ramp.SolveMinTimeLinear(vellimits, accellimits);
                    _ExtractSwitchTimes(ramp, vswitchtimes);
                    ramp.Evaluate(0, x0);
                    ramp.Derivative(0, dx0);
                    dReal fprevtime = 0;
                    for(size_t iswitch = 0; iswitch < vswitchtimes.size(); ++iswitch) {
                        ramp.Evaluate(vswitchtimes.at(iswitch), x1);
                        ramp.Derivative(vswitchtimes.at(iswitch), dx1);
                        retseg = SegmentFeasible(x0, x1, dx0, dx1, vswitchtimes.at(iswitch) - fprevtime, options);
                        if( retseg != 0 ) {
                            break;
                        }
                        x0.swap(x1);
                        dx0.swap(dx1);
                        fprevtime = vswitchtimes[iswitch];
                    }
                    if( retseg == 0 ) {
                        break;
                    }
                    if( retseg == CFO_CheckTimeBasedConstraints ) {
                        // slow the ramp down and try again
                        for(size_t j = 0; j < vellimits.size(); ++j) {
                            vellimits.at(j) *= fmult;
                            accellimits.at(j) *= fmult;
                        }
                    }
                    else {
                        RAVELOG_WARN_FORMAT("initial ramp returned error 0x%x, giving up....", retseg);
                        return retseg;
                    }
                }
                if( retseg != 0 ) {
                    // couldn't find anything...
                    return false;
                }
                if( !_parameters->verifyinitialpath ) {
                    // disable future verification
                    ramp.constraintchecked = 1;
                }
            }
        }
        return true;
    }
    
    int _Shortcut(ParabolicRamp::DynamicPath& dynamicpath, int numIters,ParabolicRamp::RampFeasibilityChecker& check,ParabolicRamp::RandomNumberGeneratorBase* rng, dReal mintimestep)
    {
        std::vector<ParabolicRamp::ParabolicRampND>& ramps = dynamicpath.ramps;
        int shortcuts = 0;
        vector<dReal> rampStartTime(ramps.size());
        dReal endTime=0;
        for(size_t i=0; i<ramps.size(); i++) {
            rampStartTime[i] = endTime;
            endTime += ramps[i].endTime;
        }
        ParabolicRamp::Vector x0, x1, dx0, dx1;
        ParabolicRamp::DynamicPath intermediate;
        std::vector<dReal> vellimits(_parameters->_vConfigVelocityLimit.size()), accellimits(_parameters->_vConfigAccelerationLimit.size());
        dReal fSearchVelAccelMult = _parameters->fSearchVelAccelMult; // for slowing down when timing constraints
        dReal fstarttimemult = 1.0; // the start velocity/accel multiplier for the velocity and acceleration computations. If manip speed/accel or dynamics constraints are used, then this will track the last successful multipler. Basically if the last successful one is 0.1, it's very unlikely than a muliplier of 0.8 will meet the constraints the next time.
        for(int iters=0; iters<numIters; iters++) {
            dReal t1=rng->Rand()*endTime,t2=rng->Rand()*endTime;
            if( iters == 0 ) {
                t1 = 0;
                t2 = endTime;
            }
            if(t1 > t2) {
                ParabolicRamp::Swap(t1,t2);
            }
            int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
            int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
            if(i1 == i2) {
                continue;
            }
            //same ramp
            dReal u1 = t1-rampStartTime[i1];
            dReal u2 = t2-rampStartTime[i2];
            PARABOLIC_RAMP_ASSERT(u1 >= 0);
            PARABOLIC_RAMP_ASSERT(u1 <= ramps[i1].endTime+ParabolicRamp::EpsilonT);
            PARABOLIC_RAMP_ASSERT(u2 >= 0);
            PARABOLIC_RAMP_ASSERT(u2 <= ramps[i2].endTime+ParabolicRamp::EpsilonT);
            u1 = ParabolicRamp::Min(u1,ramps[i1].endTime);
            u2 = ParabolicRamp::Min(u2,ramps[i2].endTime);
            ramps[i1].Evaluate(u1,x0);
            if( _parameters->SetStateValues(x0) != 0 ) {
                continue;
            }
            _parameters->_getstatefn(x0);
            ramps[i2].Evaluate(u2,x1);
            if( _parameters->SetStateValues(x1) != 0 ) {
                continue;
            }
            _parameters->_getstatefn(x1);
            ramps[i1].Derivative(u1,dx0);
            ramps[i2].Derivative(u2,dx1);
            ++_progress._iteration;

            bool bsuccess = false;

            for(size_t j = 0; j < _parameters->_vConfigVelocityLimit.size(); ++j) {
                // have to watch out that velocities don't drop under dx0 & dx1!
                dReal fminvel = max(RaveFabs(dx0[j]), RaveFabs(dx1[j]));
                vellimits[j] = max(_parameters->_vConfigVelocityLimit[j]*fstarttimemult, fminvel);
                accellimits[j] = _parameters->_vConfigAccelerationLimit[j]*fstarttimemult;
            }
            dReal fcurmult = fstarttimemult;
            for(size_t islowdowntry = 0; islowdowntry < 8; ++islowdowntry ) {
                bool res=ParabolicRamp::SolveMinTime(x0, dx0, x1, dx1, accellimits, vellimits, _parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit, intermediate, _parameters->_multidofinterp);
                if(!res) {
                    break;
                }
                // check the new ramp time makes significant steps
                dReal newramptime = intermediate.GetTotalTime();
                if( newramptime+mintimestep > t2-t1 ) {
                    // reject since it didn't make significant improvement
                    RAVELOG_VERBOSE_FORMAT("shortcut iter=%d rejected time=%fs\n", iters%(endTime-(t2-t1)+newramptime));
                    break;
                }

                if( _CallCallbacks(_progress) == PA_Interrupt ) {
                    return -1;
                }

                int retcheck = 0;
                for(size_t i=0; i<intermediate.ramps.size(); i++) {
                    if( i > 0 ) {
                        intermediate.ramps[i].x0 = intermediate.ramps[i-1].x1; // to remove noise?
                        intermediate.ramps[i].dx0 = intermediate.ramps[i-1].dx1; // to remove noise?
                    }
                    if( _parameters->SetStateValues(intermediate.ramps[i].x1) != 0 ) {
                        retcheck = CFO_StateSettingError;
                        break;
                    }
                    _parameters->_getstatefn(intermediate.ramps[i].x1);
                    // have to resolve for the ramp since the positions might have changed?
    //                for(size_t j = 0; j < intermediate.rams[i].x1.size(); ++j) {
    //                    intermediate.ramps[i].SolveFixedSwitchTime();
    //                }
                    retcheck = check.Check(intermediate.ramps[i]);
                    if( retcheck != 0) {
                        break;
                    }
                }
                if(retcheck == 0) {
                    bsuccess = true;
                    break;
                }

                if( retcheck == CFO_CheckTimeBasedConstraints ) {
                    RAVELOG_VERBOSE_FORMAT("shortcut iter=%d, slow down ramp", iters);
                    for(size_t j = 0; j < vellimits.size(); ++j) {
                        // have to watch out that velocities don't drop under dx0 & dx1!
                        dReal fminvel = max(RaveFabs(dx0[j]), RaveFabs(dx1[j]));
                        vellimits[j] = max(vellimits[j]*fSearchVelAccelMult, fminvel);
                        accellimits[j] *= fSearchVelAccelMult;
                        fcurmult *= fSearchVelAccelMult;
                    }
                }
                else {
                    RAVELOG_VERBOSE_FORMAT("shortcut iter=%d rejected due to constraints 0x%x", iters%retcheck);
                    break;
                }
            }

            if( !bsuccess ) {
                continue;
            }

            fstarttimemult = min(1.0, fcurmult/fSearchVelAccelMult); // the new start time mult should be increased by one timemult

            //perform shortcut
            shortcuts++;
            ramps[i1].TrimBack(ramps[i1].endTime-u1);
            ramps[i1].x1 = intermediate.ramps.front().x0;
            ramps[i1].dx1 = intermediate.ramps.front().dx0;
            ramps[i2].TrimFront(u2);
            ramps[i2].x0 = intermediate.ramps.back().x1;
            ramps[i2].dx0 = intermediate.ramps.back().dx1;

            //replace intermediate ramps
            for(int i=0; i<i2-i1-1; i++) {
                ramps.erase(ramps.begin()+i1+1);
            }
            ramps.insert(ramps.begin()+i1+1,intermediate.ramps.begin(),intermediate.ramps.end());

            //check for consistency
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                for(size_t i=0; i+1<ramps.size(); i++) {
                    for(size_t j = 0; j < ramps[i].x1.size(); ++j) {
                        OPENRAVE_ASSERT_OP(RaveFabs(ramps[i].x1[j]-ramps[i+1].x0[j]), <=, g_fEpsilon);
                        OPENRAVE_ASSERT_OP(RaveFabs(ramps[i].dx1[j]-ramps[i+1].dx0[j]), <=, g_fEpsilon);
                    }
                }
            }

            //revise the timing
            rampStartTime.resize(ramps.size());
            endTime=0;
            for(size_t i=0; i<ramps.size(); i++) {
                rampStartTime[i] = endTime;
                endTime += ramps[i].endTime;
            }
            RAVELOG_VERBOSE("shortcut iter=%d endTime=%f\n",iters,endTime);
        }
        return shortcuts;
    }

    /** \brief return true if all the links in _listCheckManips satisfy the acceleration and velocity constraints

       |w x (R x_i) + v| <= thresh
     */
    bool _CheckManipConstraints(const ParabolicRamp::Vector& a,const ParabolicRamp::Vector& b, const ParabolicRamp::Vector& da,const ParabolicRamp::Vector& db, dReal timeelapsed)
    {
        ParabolicRamp::ParabolicRampND ramp;
        if(timeelapsed<=g_fEpsilonLinear) {
            return true;
        }
        else {
            ramp.SetPosVelTime(a,da,b,db,timeelapsed);
            vector<dReal> ac, dofaccelerations, qfill, vfill;
            ramp.Accel(timeelapsed/2,ac);
            bool res = true;
            dReal maxmanipspeed=0, maxmanipaccel=0;
            FOREACH(it,_constraintreturn->_configurationtimes) {
                vector<dReal> qc,vc;
                ramp.Evaluate(*it,qc);
                ramp.Derivative(*it,vc);
                // Compute the velocity and accel of the end effector COM
                FOREACH(itmanipinfo,_listCheckManips) {
                    KinBodyPtr probot = itmanipinfo->plink->GetParent();
                    std::vector<int> vuseddofindices, vconfigindices;
                    _parameters->_configurationspecification.ExtractUsedIndices(probot, vuseddofindices, vconfigindices);
                    qfill.resize(vuseddofindices.size());
                    vfill.resize(vuseddofindices.size());
                    for(size_t idof = 0; idof < vuseddofindices.size(); ++idof) {
                        qfill[idof] = qc.at(vconfigindices.at(idof));
                        vfill[idof] = vc.at(vconfigindices.at(idof));
                    }
                    std::vector<std::pair<Vector,Vector> > endeffvels,endeffaccs;
                    Vector endeffvellin,endeffvelang,endeffacclin,endeffaccang;
                    int endeffindex = itmanipinfo->plink->GetIndex();
                    KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);

                    // Set robot to new state
                    probot->SetDOFValues(qfill, KinBody::CLA_CheckLimits, vuseddofindices);
                    probot->SetDOFVelocities(vfill, KinBody::CLA_CheckLimits, vuseddofindices);
                    probot->GetLinkVelocities(endeffvels);
                    probot->GetLinkAccelerations(dofaccelerations,endeffaccs);
                    endeffvellin = endeffvels.at(endeffindex).first;
                    endeffvelang = endeffvels.at(endeffindex).second;
                    endeffacclin = endeffaccs.at(endeffindex).first;
                    endeffaccang = endeffaccs.at(endeffindex).second;
                    Transform R = itmanipinfo->plink->GetTransform();
                    // For each point in checkpoints, compute its vel and acc and check whether they satisfy the manipulator constraints
                    FOREACH(itpoint,itmanipinfo->checkpoints) {
                        Vector point = R.rotate(*itpoint);
                        if(_parameters->maxmanipspeed>0) {
                            Vector vpoint = endeffvellin + endeffvelang.cross(point);
                            dReal manipspeed = RaveSqrt(vpoint.lengthsqr3());
                            if( maxmanipspeed < manipspeed ) {
                                maxmanipspeed = manipspeed;
                            }
                            if( manipspeed > _parameters->maxmanipspeed) {
                                res = false;
                                break;
                            }
                        }
                        if(_parameters->maxmanipaccel>0) {
                            Vector apoint = endeffacclin + endeffvelang.cross(endeffvelang.cross(point)) + endeffaccang.cross(point);
                            dReal manipaccel = RaveSqrt(apoint.lengthsqr3());
                            if( maxmanipaccel < manipaccel ) {
                                maxmanipaccel = manipaccel;
                            }
                            if(manipaccel > _parameters->maxmanipaccel) {
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

    /// \brief extracts the unique switch points for every 1D ramp. 0 is *not* included. endtime is included.
    void _ExtractSwitchTimes(const ParabolicRamp::ParabolicRampND& rampnd, std::vector<dReal>& vswitchtimes)
    {
        vswitchtimes.resize(0);
        vswitchtimes.push_back(rampnd.endTime);
        FOREACHC(itramp,rampnd.ramps) {
            vector<dReal>::iterator it;
            if( itramp->tswitch1 != 0 ) {
                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch1);
                if( it != vswitchtimes.end() && *it != itramp->tswitch1) {
                    vswitchtimes.insert(it,itramp->tswitch1);
                }
            }
            if( itramp->tswitch1 != itramp->tswitch2 && itramp->tswitch2 != 0 ) {
                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->tswitch2);
                if( it != vswitchtimes.end() && *it != itramp->tswitch2 ) {
                    vswitchtimes.insert(it,itramp->tswitch2);
                }
            }
            if( itramp->ttotal != itramp->tswitch2 && itramp->ttotal != 0 ) {
                it = lower_bound(vswitchtimes.begin(),vswitchtimes.end(),itramp->ttotal);
                if( it != vswitchtimes.end() && *it != itramp->ttotal ) {
                    vswitchtimes.insert(it,itramp->ttotal);
                }
            }
        }
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

    ConstraintTrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _uniformsampler;
    ConstraintFilterReturnPtr _constraintreturn;
    std::list< ManipConstraintInfo > _listCheckManips; ///< the manipulators and the points on their end efffectors to check for velocity and acceleration constraints
    TrajectoryBasePtr _dummytraj;
    PlannerProgress _progress;
    bool _bUsePerturbation;
    bool _bmanipconstraints; /// if true, check workspace manip constraints
};


PlannerBasePtr CreateParabolicSmoother(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PlannerBasePtr(new ParabolicSmoother(penv,sinput));
}

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(ParabolicRamp::ParabolicRamp1D)
BOOST_TYPEOF_REGISTER_TYPE(ParabolicRamp::ParabolicRampND)
#endif
