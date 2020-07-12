// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_TRAJECTORY_RETIMER
#define OPENRAVE_TRAJECTORY_RETIMER

#include "openraveplugindefs.h"
#include "manipconstraints.h"

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave_plugins_rplanners", msgid)

namespace rplanners {

class TrajectoryRetimer : public PlannerBase
{
protected:
    class GroupInfo
    {
public:
        GroupInfo(int degree, const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group &gvel) : degree(degree), gpos(gpos), gvel(gvel), orgposoffset(-1), orgveloffset(-1) {
        }
        virtual ~GroupInfo() {
        }
        int degree;
        const ConfigurationSpecification::Group& gpos, &gvel;
        int orgposoffset, orgveloffset;
        std::vector<dReal> _vConfigVelocityLimit, _vConfigAccelerationLimit, _vConfigLowerLimit, _vConfigUpperLimit;
        // optional
        std::vector<dReal> _vConfigJerkLimit;
    };
    typedef boost::shared_ptr<GroupInfo> GroupInfoPtr;
    typedef boost::shared_ptr<GroupInfo const> GroupInfoConstPtr;

public:
    TrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\nTrajectory re-timing without modifying any of the points. Overwrites the velocities and timestamps.";
        _bmanipconstraints = false;
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        params->Validate();
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        _parameters->copy(params);
        // reset the cache
        _cachedoldspec = ConfigurationSpecification();
        _cachednewspec = ConfigurationSpecification();
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new ConstraintTrajectoryTimingParameters());
        isParameters >> *_parameters;
        _parameters->Validate();
        return _InitPlan();
    }

    virtual bool _InitPlan()
    {
        _timeoffset = -1;
        if( (int)_parameters->_vConfigVelocityLimit.size() != _parameters->GetDOF() ) {
            return false;
        }
        _vimaxvel.resize(_parameters->_vConfigVelocityLimit.size());
        for(size_t i = 0; i < _vimaxvel.size(); ++i) {
            _vimaxvel[i] = 1/_parameters->_vConfigVelocityLimit[i];
        }
        _vimaxaccel.resize(_parameters->_vConfigAccelerationLimit.size());
        for(size_t i = 0; i < _vimaxaccel.size(); ++i) {
            _vimaxaccel[i] = 1/_parameters->_vConfigAccelerationLimit[i];
        }
        _bmanipconstraints = _parameters->manipname.size() > 0 && (_parameters->maxmanipspeed>0 || _parameters->maxmanipaccel>0);
        // initialize workspace constraints on manipulators
        if(_bmanipconstraints ) {
            if( !_manipconstraintchecker ) {
                _manipconstraintchecker.reset(new ManipConstraintChecker(GetEnv()));
            }
            _manipconstraintchecker->Init(_parameters->manipname, _parameters->_configurationspecification, _parameters->maxmanipspeed, _parameters->maxmanipaccel);
        }

        return _SupportInterpolation();
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        // TODO there's a lot of info that is being recomputed which could be cached depending on the configurationspace of the incoming trajectory
        BOOST_ASSERT(!!_parameters && !!ptraj && ptraj->GetEnv()==GetEnv());
        BOOST_ASSERT(_parameters->GetDOF() == _parameters->_configurationspecification.GetDOF());
        std::vector<ConfigurationSpecification::Group>::const_iterator itoldgrouptime = ptraj->GetConfigurationSpecification().FindCompatibleGroup("deltatime",false);
        if( _parameters->_hastimestamps && itoldgrouptime == ptraj->GetConfigurationSpecification()._vgroups.end() ) {
            std::string description = str(boost::format("env=%d, trajectory does not have timestamps, even though parameters say timestamps are needed")%GetEnv()->GetId());
            RAVELOG_WARN(description);
            return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
        }
        size_t numpoints = ptraj->GetNumWaypoints();
        if( numpoints == 0 ) {
            // there's nothing to retime...
            std::string description = str(boost::format("env=%d, there's nothing to retime")%GetEnv()->GetId());
            return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
        }
        ConfigurationSpecification velspec = _parameters->_configurationspecification.ConvertToVelocitySpecification();
        if( _parameters->_hasvelocities ) {
            // check that all velocity groups are there
            FOREACH(itgroup,velspec._vgroups) {
                if(ptraj->GetConfigurationSpecification().FindCompatibleGroup(*itgroup,true) == ptraj->GetConfigurationSpecification()._vgroups.end() ) {
                    std::string description = str(boost::format("env=%d, trajectory does not have velocity group '%s', even though parameters say is needed")%GetEnv()->GetId()%itgroup->name);
                    RAVELOG_WARN(description);
                    return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
                }
            }
        }

        ConfigurationSpecification newspec = _parameters->_configurationspecification;
        newspec.AddDerivativeGroups(1,false);
        //bool bWritingAcceleration = false;
        if( _parameters->_interpolation == "quadric" || _parameters->_interpolation == "quintic" ) {
            newspec.AddDerivativeGroups(2,false);
            //bWritingAcceleration = true;
        }
        newspec.AddDeltaTimeGroup();
        ptraj->GetWaypoints(0,numpoints,_vdiffdata, _parameters->_configurationspecification);
        // check values close to the limits and clamp them, this hopefully helps the retimers that just do simpler <= and >= checks
        for(size_t i = 0; i < _vdiffdata.size(); i += _parameters->GetDOF()) {
            for(int j = 0; j < _parameters->GetDOF(); ++j) {
                dReal lower = _parameters->_vConfigLowerLimit.at(j), upper = _parameters->_vConfigUpperLimit.at(j);
                if( _vdiffdata.at(i+j) < lower ) {
                    if( _vdiffdata.at(i+j) < lower-g_fEpsilonJointLimit ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, lower limit for traj point %d dof %d is not followed (%.15e < %.15e)"),GetEnv()->GetId()%(i/_parameters->GetDOF())%j%_vdiffdata.at(i+j)%lower,ORE_InconsistentConstraints);
                    }
                    _vdiffdata.at(i+j) = lower;
                }
                else if( _vdiffdata.at(i+j) > upper ) {
                    if( _vdiffdata.at(i+j) > upper+g_fEpsilonJointLimit ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, upper limit for traj point %d dof %d is not followed (%.15e > %.15e)"),GetEnv()->GetId()%(i/_parameters->GetDOF())%j%_vdiffdata.at(i+j)%upper,ORE_InconsistentConstraints);
                    }
                    _vdiffdata.at(i+j) = upper;
                }
            }
        }

        _vdata.resize(numpoints*newspec.GetDOF());
        std::fill(_vdata.begin(), _vdata.end(), 0);
        ConfigurationSpecification::ConvertData(_vdata.begin(), newspec, _vdiffdata.begin(), _parameters->_configurationspecification, numpoints, GetEnv());
        int degree = 1;

//        {
//            string filename = str(boost::format("%s/failedsmoothing%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%10000));
//            RAVELOG_WARN(str(boost::format("writing original trajectory to %s")%filename));
//            ofstream f(filename.c_str());
//            f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
//            ptraj->serialize(f);
//        }

        if( numpoints > 1 ) {
            // analyze the configuration space and set the necessary converters
            const string& posinterpolation = _parameters->_interpolation;
            if( _cachedoldspec != _parameters->_configurationspecification || posinterpolation != _cachedposinterpolation ) {
                _listgroupinfo.clear();
                _listmintimefns.clear();
                _listvelocityfns.clear();
                _listcheckvelocityfns.clear();
                _listwritefns.clear();
                _cachednewspec = newspec;
                _cachedoldspec = _parameters->_configurationspecification;
                _cachedposinterpolation = posinterpolation;
                string velinterpolation = ConfigurationSpecification::GetInterpolationDerivative(posinterpolation);
                string accelinterpolation = ConfigurationSpecification::GetInterpolationDerivative(velinterpolation);
                const boost::array<std::string,3> supportedgroups = {{"joint_values", "affine_transform", "ikparam_values"}};
                for(size_t i = 0; i < _cachednewspec._vgroups.size(); ++i) {
                    ConfigurationSpecification::Group& gpos = _cachednewspec._vgroups[i];
                    size_t igrouptype;
                    for(igrouptype = 0; igrouptype < supportedgroups.size(); ++igrouptype) {
                        if( gpos.name.size() >= supportedgroups[igrouptype].size() && gpos.name.substr(0,supportedgroups[igrouptype].size()) == supportedgroups[igrouptype] ) {
                            break;
                        }
                    }
                    if( igrouptype >= supportedgroups.size() ) {
                        continue;
                    }

                    // group is supported
                    std::vector<ConfigurationSpecification::Group>::const_iterator itgroup = _cachedoldspec.FindCompatibleGroup(gpos);
                    BOOST_ASSERT(itgroup != _cachedoldspec._vgroups.end());
                    int orgposoffset = itgroup->offset;
                    BOOST_ASSERT(orgposoffset+gpos.dof <= _parameters->GetDOF());
                    std::vector<ConfigurationSpecification::Group>::iterator itvelgroup = _cachednewspec._vgroups.begin()+(_cachednewspec.FindTimeDerivativeGroup(gpos)-_cachednewspec._vgroups.begin());
                    BOOST_ASSERT(itvelgroup != _cachednewspec._vgroups.end());
                    std::vector<ConfigurationSpecification::Group>::const_iterator itaccelgroupc = _cachednewspec.FindTimeDerivativeGroup(*itvelgroup);
                    _listgroupinfo.push_back(CreateGroupInfo(degree, _cachednewspec, gpos, *itvelgroup));
                    _listgroupinfo.back()->orgposoffset = orgposoffset;
                    _listgroupinfo.back()->_vConfigVelocityLimit = std::vector<dReal>(_parameters->_vConfigVelocityLimit.begin()+itgroup->offset, _parameters->_vConfigVelocityLimit.begin()+itgroup->offset+itgroup->dof);
                    _listgroupinfo.back()->_vConfigAccelerationLimit = std::vector<dReal>(_parameters->_vConfigAccelerationLimit.begin()+itgroup->offset, _parameters->_vConfigAccelerationLimit.begin()+itgroup->offset+itgroup->dof);
                    _listgroupinfo.back()->_vConfigLowerLimit = std::vector<dReal>(_parameters->_vConfigLowerLimit.begin()+itgroup->offset, _parameters->_vConfigLowerLimit.begin()+itgroup->offset+itgroup->dof);
                    _listgroupinfo.back()->_vConfigUpperLimit = std::vector<dReal>(_parameters->_vConfigUpperLimit.begin()+itgroup->offset, _parameters->_vConfigUpperLimit.begin()+itgroup->offset+itgroup->dof);

                    itgroup = _cachedoldspec.FindCompatibleGroup(*itvelgroup);
                    if( itgroup != _cachedoldspec._vgroups.end() ) {
                        // velocity is optional
                        _listgroupinfo.back()->orgveloffset = itgroup->offset;
                    }

                    stringstream ss(gpos.name.substr(supportedgroups[igrouptype].size()));
                    string bodyname;
                    int affinedofs=0;
                    IkParameterizationType iktype=IKP_None;
                    if( igrouptype == 1 ) {
                        ss >> bodyname >> affinedofs;
                    }
                    else if( igrouptype == 2 ) {
                        int niktype=0;
                        ss >> niktype;
                        iktype = static_cast<IkParameterizationType>(niktype);
                    }

                    {
                        // if _parameters->_hastimestamps, then use this for double checking that times are feasible
                        if( igrouptype == 0 ) {
                            _listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeJointValues,this,_listgroupinfo.back(),_1,_2,_3,_4));
                        }
                        else if( igrouptype == 1 ) {
                            _listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeAffine,this,_listgroupinfo.back(),affinedofs,_1,_2,_3,_4));
                        }
                        else if( igrouptype == 2 ) {
                            _listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeIk,this,_listgroupinfo.back(),iktype,_1,_2,_3,_4));
                        }
                    }

                    if( igrouptype == 0 ) {
                        if( _parameters->_hasvelocities ) {
                            _listcheckvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_CheckJointValues,this,_listgroupinfo.back(),_1,_2,_3));
                        }
                        else {
                            _listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesJointValues,this,_listgroupinfo.back(),_1,_2,_3));
                        }
                        _listwritefns.push_back(boost::bind(&TrajectoryRetimer::_WriteJointValues,this,_listgroupinfo.back(),_1,_2,_3));
                    }
                    else if( igrouptype == 1 ) {
                        if( _parameters->_hasvelocities ) {
                            _listcheckvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_CheckAffine,this,_listgroupinfo.back(),affinedofs,_1,_2,_3));
                        }
                        else {
                            _listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesAffine,this,_listgroupinfo.back(),affinedofs,_1,_2,_3));
                        }
                        _listwritefns.push_back(boost::bind(&TrajectoryRetimer::_WriteAffine,this,_listgroupinfo.back(),affinedofs,_1,_2,_3));
                    }
                    else if( igrouptype == 2 ) {
                        if( _parameters->_hasvelocities ) {
                            _listcheckvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_CheckIk,this,_listgroupinfo.back(),iktype,_1,_2,_3));
                        }
                        else {
                            _listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesIk,this,_listgroupinfo.back(),iktype,_1,_2,_3));
                        }
                        _listwritefns.push_back(boost::bind(&TrajectoryRetimer::_WriteIk,this,_listgroupinfo.back(),iktype,_1,_2,_3));
                    }

                    gpos.interpolation = posinterpolation;
                    itvelgroup->interpolation = velinterpolation;
                    if( itaccelgroupc != _cachednewspec._vgroups.end() ) {
                        _cachednewspec._vgroups.at(itaccelgroupc-_cachednewspec._vgroups.begin()).interpolation = accelinterpolation;
                    }
                }
                // compute the timestamps and velocities
                _timeoffset = -1;
                FOREACH(itgroup,_cachednewspec._vgroups) {
                    if( itgroup->name == "deltatime" ) {
                        _timeoffset = itgroup->offset;
                    }
                }
            }
            else {
                FOREACH(it, _listgroupinfo) {
                    ResetCachedGroupInfo(*it);
                }
            }


            int dof = _cachednewspec.GetDOF();
            std::vector<dReal>::iterator itdata = _vdata.begin();
            _vdata.at(_timeoffset) = 0;
            // set velocities to 0
            FOREACH(it, _listgroupinfo) {
                int offset = (*it)->gvel.offset;
                for(int j = 0; j < (*it)->gvel.dof; ++j) {
                    _vdata.at(offset+j) = 0; // initial
                    _vdata.at(_vdata.size()-dof+offset+j) = 0; // end
                }
            }

            // get the diff states
            vector<dReal>& vprev = _vtempdata0; vprev.resize(_cachedoldspec.GetDOF());
            vector<dReal>& vnext = _vtempdata1; vnext.resize(_cachedoldspec.GetDOF());
            std::copy(_vdiffdata.end()-_cachedoldspec.GetDOF(),_vdiffdata.end(),vnext.begin());
            for(size_t i = numpoints-1; i > 0; --i) {
                std::copy(_vdiffdata.begin()+(i-1)*_cachedoldspec.GetDOF(),_vdiffdata.begin()+(i)*_cachedoldspec.GetDOF(),vprev.begin());
                _parameters->_diffstatefn(vnext,vprev);
                std::copy(vnext.begin(),vnext.end(),_vdiffdata.begin()+i*_cachedoldspec.GetDOF());
                vnext = vprev;
            }

            if( _parameters->_hastimestamps ) {
                ptraj->GetWaypoints(0, numpoints, _vtempdata0,*itoldgrouptime);
                ConfigurationSpecification::ConvertData(_vdata.begin(), _cachednewspec, _vtempdata0.begin(), *itoldgrouptime, numpoints, GetEnv(), false);
            }
            if( _parameters->_hasvelocities ) {
                ptraj->GetWaypoints(0,numpoints,_vtempdata0,velspec);
                ConfigurationSpecification::ConvertData(_vdata.begin(),_cachednewspec,_vtempdata0.begin(),velspec,numpoints,GetEnv(),false);
            }
            try {
                std::vector<dReal>::iterator itorgdiff = _vdiffdata.begin()+_cachedoldspec.GetDOF();
                std::vector<dReal>::iterator itdataprev = itdata;
                itdata += dof;
                for(size_t i = 1; i < numpoints; ++i, itdata += dof, itorgdiff += _cachedoldspec.GetDOF()) {
                    dReal mintime = 0;
                    bool bUseEndVelocity = i+1==numpoints;
                    if( _parameters->_hastimestamps && _parameters->_hasvelocities ) {
                        // positions, velocities, and timestamps already filled, so check everything
                        FOREACH(itfn, _listcheckvelocityfns) {
                            if( !(*itfn)(itdataprev, itdata, 7) ) {
                                std::string description = str(boost::format("env=%d, point %d/%d has unreachable velocity")%GetEnv()->GetId()%i%numpoints);
                                RAVELOG_VERBOSE(description);
                                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                                    (*itfn)(itdataprev, itdata, 7);
                                }
                                return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
                            }
                        }
                    }
                    else {
                        FOREACH(itmin, _listmintimefns) {
                            dReal fgrouptime = (*itmin)(itorgdiff, itdataprev, itdata,bUseEndVelocity);
                            if( fgrouptime < 0 ) {
                                std::string description = str(boost::format("env=%d, point %d/%d has uncomputable minimum time, possibly due to boundary constraints")%GetEnv()->GetId()%i%numpoints);
                                RAVELOG_VERBOSE(description);
                                return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
                            }

                            if( _parameters->_fStepLength > 0 ) {
                                if( fgrouptime < _parameters->_fStepLength ) {
                                    fgrouptime = _parameters->_fStepLength;
                                }
                                else {
                                    fgrouptime = std::ceil(fgrouptime/_parameters->_fStepLength-g_fEpsilonJointLimit)*_parameters->_fStepLength;
                                }
                            }
                            if( mintime < fgrouptime ) {
                                mintime = fgrouptime;
                            }
                        }
                        if( _parameters->_hastimestamps ) {
                            if( *(itdata+_timeoffset) < mintime-g_fEpsilonJointLimit ) {
                                // this is a commonly occuring message in planning
                                std::string description = str(boost::format("env=%d, point %d/%d has unreachable minimum time %e > %e")%GetEnv()->GetId()%i%numpoints%(*(itdata+_timeoffset))%mintime);
                                RAVELOG_VERBOSE(description);
                                return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
                            }
                        }
                        else {
                            *(itdata+_timeoffset) = mintime;
                        }
                        if( _parameters->_hasvelocities ) {
                            FOREACH(itfn,_listcheckvelocityfns) {
                                if( !(*itfn)(itdataprev, itdata, 6) ) {
                                    std::string description = str(boost::format("env=%d, point %d/%d has unreachable velocity")%GetEnv()->GetId()%i%numpoints);
                                    RAVELOG_WARN(description);
                                    return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
                                }
                            }
                        }
                        else {
                            // given the mintime, fill the velocities
                            FOREACH(itfn,_listvelocityfns) {
                                (*itfn)(itorgdiff, itdataprev, itdata);
                            }
                        }
                    }
                    FOREACH(itfn,_listwritefns) {
                        // because the initial time for each ramp could have been stretched to accomodate other points, it is possible for this to fail
                        if( !(*itfn)(itorgdiff, itdataprev, itdata) ) {
                            std::string description = str(boost::format("env=%d, point %d/%d has unreachable new time %es, probably due to acceleration limtis violated.")%GetEnv()->GetId()%i%numpoints%(*(itdata+_timeoffset)));
                            RAVELOG_VERBOSE(description);
                            return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
                        }
                    }
                    itdataprev = itdata;
                }
            }
            catch (const std::exception& ex) {
                string filename = str(boost::format("%s/failedsmoothing%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%10000));
                std::string description = str(boost::format("env=%d, parabolic planner failed: %s, writing original trajectory to %s")%GetEnv()->GetId()%ex.what()%filename);
                RAVELOG_WARN(description);
                ofstream f(filename.c_str());
                f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                ptraj->serialize(f);
                return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
            }
        }
        else {
            // one point, but still need to set interpolation
            FOREACH(itgroup,_cachednewspec._vgroups) {
                itgroup->interpolation = _parameters->_interpolation;
            }
        }

        _WriteTrajectory(ptraj,_cachednewspec, _vdata);
        // happens too often for debug message?
        //RAVELOG_VERBOSE(str(boost::format("env=%d, %s path duration=%es, timestep=%es")%GetEnv()->GetId()%GetXMLId()%ptraj->GetDuration()%_parameters->_fStepLength));
        return OPENRAVE_PLANNER_STATUS(PS_HasSolution);
    }

protected:
    // method to be overriden by individual timing types

    /// \brief createa s group info
    virtual GroupInfoPtr CreateGroupInfo(int degree, const ConfigurationSpecification& spec, const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group &gvel) {
        return GroupInfoPtr(new GroupInfo(degree, gpos, gvel));
    }
    /// \brief resets any cached data in the group info
    virtual void ResetCachedGroupInfo(GroupInfoPtr g) {
    }

    virtual bool _SupportInterpolation() = 0;

    /// \brief compute the minimum time to achieve the point. returns a mintime>=0 if successeeded, otherwise returns value < 0.
    virtual dReal _ComputeMinimumTimeJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) = 0;
    /// \brief given the delta time, compute the velocities in the data
    virtual void _ComputeVelocitiesJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) = 0;
    /// \brief given the computed deltatime and velocities at each point, check position, velocity, and acceleration limits
    /// checkoptions. If 1 checks positions. If 2, checks velocities, If 4, checks accelerations
    virtual bool _CheckJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata, int checkoptions=0xffffffff) {
        return true;
    }
    virtual bool _WriteJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        return true;
    }

    virtual dReal _ComputeMinimumTimeAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) = 0;
    virtual void _ComputeVelocitiesAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) = 0;
    virtual bool _CheckAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata, int checkoptions=0xffffffff) {
        return true;
    }
    virtual bool _WriteAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        return true;
    }

    // speed of rotations is always the speed of the angle along the minimum rotation
    // speed of translations is always the combined xyz speed
    virtual dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) = 0;
    virtual void _ComputeVelocitiesIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) = 0;
    virtual bool _CheckIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata, int checkoptions=0xffffffff) {
        return true;
    }
    virtual bool _WriteIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        return true;
    }

    virtual void _WriteTrajectory(TrajectoryBasePtr ptraj, const ConfigurationSpecification& newspec, const std::vector<dReal>& data) {
        ptraj->Init(newspec);
        ptraj->Insert(0,data);
    }

    ConstraintTrajectoryTimingParametersPtr _parameters;
    boost::shared_ptr<ManipConstraintChecker> _manipconstraintchecker;

    // caching
    ConfigurationSpecification _cachedoldspec, _cachednewspec; ///< the configuration specification that the cached structures have been set for
    std::string _cachedposinterpolation;
    std::list< boost::function<dReal(std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,bool) > > _listmintimefns;
    std::list< boost::function<void(std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,std::vector<dReal>::iterator) > > _listvelocityfns;
    std::list< boost::function<bool(std::vector<dReal>::const_iterator,std::vector<dReal>::iterator, int) > > _listcheckvelocityfns;
    std::list< boost::function<bool(std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,std::vector<dReal>::iterator) > > _listwritefns;
    std::vector<dReal> _vimaxvel, _vimaxaccel;
    std::vector<dReal> _vdiffdata, _vdata;
    int _timeoffset;
    std::list<GroupInfoPtr> _listgroupinfo;
    vector<dReal> _vtempdata0, _vtempdata1;

    bool _bmanipconstraints; /// if true, check workspace manip constraints
};

} // end namespace rplanners

#endif
