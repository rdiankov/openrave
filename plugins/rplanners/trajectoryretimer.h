// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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

#include "plugindefs.h"

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
        vector<dReal> _vConfigVelocityLimit, _vConfigAccelerationLimit, _vConfigLowerLimit, _vConfigUpperLimit;
    };
    typedef boost::shared_ptr<GroupInfo> GroupInfoPtr;
    typedef boost::shared_ptr<GroupInfo const> GroupInfoConstPtr;

public:
    TrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\nTrajectory re-timing without modifying any of the points. Overwrites the velocities and timestamps.";
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        params->Validate();
        _parameters.reset(new TrajectoryTimingParameters());
        _parameters->copy(params);
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new TrajectoryTimingParameters());
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
        return _SupportInterpolation();
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        // TODO there's a lot of info that is being recomputed which could be cached depending on the configurationspace of the incoming trajectory
        BOOST_ASSERT(!!_parameters && !!ptraj && ptraj->GetEnv()==GetEnv());
        BOOST_ASSERT(_parameters->GetDOF() == _parameters->_configurationspecification.GetDOF());
        std::vector<ConfigurationSpecification::Group>::const_iterator itoldgrouptime = ptraj->GetConfigurationSpecification().FindCompatibleGroup("deltatime",false);
        if( _parameters->_hastimestamps && itoldgrouptime == ptraj->GetConfigurationSpecification()._vgroups.end() ) {
            RAVELOG_WARN("trajectory does not have timestamps, even though parameters say timestamps are needed\n");
            return PS_Failed;
        }
        size_t numpoints = ptraj->GetNumWaypoints();
        const ConfigurationSpecification& oldspec = _parameters->_configurationspecification;
        ConfigurationSpecification velspec = oldspec.ConvertToVelocitySpecification();
        if( _parameters->_hasvelocities ) {
            // check that all velocity groups are there
            FOREACH(itgroup,velspec._vgroups) {
                if(ptraj->GetConfigurationSpecification().FindCompatibleGroup(*itgroup,true) == ptraj->GetConfigurationSpecification()._vgroups.end() ) {
                    RAVELOG_WARN(str(boost::format("trajectory does not have velocity group '%s', even though parameters say is needed")%itgroup->name));
                    return PS_Failed;
                }
            }
        }

        ConfigurationSpecification newspec = oldspec;
        newspec.AddDerivativeGroups(1,true);
        vector<dReal> vdiffdata, data;
        ptraj->GetWaypoints(0,numpoints,vdiffdata,oldspec);
        // check values close to the limits and clamp them, this hopefully helps the retimers that just do simpler <= and >= checks
        for(size_t i = 0; i < vdiffdata.size(); i += oldspec.GetDOF()) {
            for(int j = 0; j < oldspec.GetDOF(); ++j) {
                dReal lower = _parameters->_vConfigLowerLimit.at(j), upper = _parameters->_vConfigUpperLimit.at(j);
                if( vdiffdata.at(i+j) < lower ) {
                    if( vdiffdata.at(i+j) < lower-g_fEpsilonJointLimit ) {
                        throw OPENRAVE_EXCEPTION_FORMAT("lower limit for traj point %d dof %d is not followed (%.15e < %.15e)",(i/oldspec.GetDOF())%j%vdiffdata.at(i+j)%lower,ORE_InconsistentConstraints);
                    }
                    vdiffdata.at(i+j) = lower;
                }
                else if( vdiffdata.at(i+j) > upper ) {
                    if( vdiffdata.at(i+j) > upper+g_fEpsilonJointLimit ) {
                        throw OPENRAVE_EXCEPTION_FORMAT("upper limit for traj point %d dof %d is not followed (%.15e > %.15e)",(i/oldspec.GetDOF())%j%vdiffdata.at(i+j)%upper,ORE_InconsistentConstraints);
                    }
                    vdiffdata.at(i+j) = upper;
                }
            }
        }

        data.resize(numpoints*newspec.GetDOF(),0);
        ConfigurationSpecification::ConvertData(data.begin(),newspec,vdiffdata.begin(),oldspec,numpoints,GetEnv());
        int degree = 1;
        string posinterpolation = _parameters->_interpolation;
        string velinterpolation;
        if( posinterpolation == "quadratic" ) {
            velinterpolation = "linear";
        }
        else if( posinterpolation == "linear" ) {
            velinterpolation = "next";
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("what is deriv of %s?",posinterpolation,ORE_InvalidArguments);
        }

        if( numpoints > 1 ) {
            // get the diff states
            vector<dReal> vprev(oldspec.GetDOF()), vnext(oldspec.GetDOF());
            std::copy(vdiffdata.end()-oldspec.GetDOF(),vdiffdata.end(),vnext.begin());
            for(size_t i = numpoints-1; i > 0; --i) {
                std::copy(vdiffdata.begin()+(i-1)*oldspec.GetDOF(),vdiffdata.begin()+(i)*oldspec.GetDOF(),vprev.begin());
                _parameters->_diffstatefn(vnext,vprev);
                std::copy(vnext.begin(),vnext.end(),vdiffdata.begin()+i*oldspec.GetDOF());
                vnext = vprev;
            }

            // analyze the configuration space and set the necessary converters
            _listgroupinfo.clear();
            list< boost::function<dReal(std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,bool) > > listmintimefns;
            list< boost::function<void(std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,std::vector<dReal>::iterator) > > listvelocityfns;
            list< boost::function<bool(std::vector<dReal>::const_iterator,std::vector<dReal>::iterator) > > listcheckvelocityfns;
            list< boost::function<bool(std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator) > > listwritefns;
            const boost::array<std::string,3> supportedgroups = {{"joint_values", "affine_transform", "ikparam_values"}};
            for(size_t i = 0; i < newspec._vgroups.size(); ++i) {
                ConfigurationSpecification::Group& gpos = newspec._vgroups[i];
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
                std::vector<ConfigurationSpecification::Group>::const_iterator itgroup = oldspec.FindCompatibleGroup(gpos);
                BOOST_ASSERT(itgroup != oldspec._vgroups.end());
                int orgposoffset = itgroup->offset;
                BOOST_ASSERT(orgposoffset+gpos.dof <= _parameters->GetDOF());
                std::vector<ConfigurationSpecification::Group>::iterator itvelgroup = newspec._vgroups.begin()+(newspec.FindTimeDerivativeGroup(gpos)-newspec._vgroups.begin());
                BOOST_ASSERT(itvelgroup != newspec._vgroups.end());
                _listgroupinfo.push_back(CreateGroupInfo(degree,gpos,*itvelgroup));
                _listgroupinfo.back()->orgposoffset = orgposoffset;
                _listgroupinfo.back()->_vConfigVelocityLimit = std::vector<dReal>(_parameters->_vConfigVelocityLimit.begin()+itgroup->offset, _parameters->_vConfigVelocityLimit.begin()+itgroup->offset+itgroup->dof);
                _listgroupinfo.back()->_vConfigAccelerationLimit = std::vector<dReal>(_parameters->_vConfigAccelerationLimit.begin()+itgroup->offset, _parameters->_vConfigAccelerationLimit.begin()+itgroup->offset+itgroup->dof);
                _listgroupinfo.back()->_vConfigLowerLimit = std::vector<dReal>(_parameters->_vConfigLowerLimit.begin()+itgroup->offset, _parameters->_vConfigLowerLimit.begin()+itgroup->offset+itgroup->dof);
                _listgroupinfo.back()->_vConfigUpperLimit = std::vector<dReal>(_parameters->_vConfigUpperLimit.begin()+itgroup->offset, _parameters->_vConfigUpperLimit.begin()+itgroup->offset+itgroup->dof);

                itgroup = oldspec.FindCompatibleGroup(*itvelgroup);
                if( itgroup != oldspec._vgroups.end() ) {
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
                        listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeJointValues,this,_listgroupinfo.back(),_1,_2,_3,_4));
                    }
                    else if( igrouptype == 1 ) {
                        listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeAffine,this,_listgroupinfo.back(),affinedofs,_1,_2,_3,_4));
                    }
                    else if( igrouptype == 2 ) {
                        listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeIk,this,_listgroupinfo.back(),iktype,_1,_2,_3,_4));
                    }
                }

                if( igrouptype == 0 ) {
                    if( _parameters->_hasvelocities ) {
                        listcheckvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_CheckVelocitiesJointValues,this,_listgroupinfo.back(),_1,_2));
                    }
                    else {
                        listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesJointValues,this,_listgroupinfo.back(),_1,_2,_3));
                    }
                    listwritefns.push_back(boost::bind(&TrajectoryRetimer::_WriteJointValues,this,_listgroupinfo.back(),_1,_2,_3));
                }
                else if( igrouptype == 1 ) {
                    if( _parameters->_hasvelocities ) {
                        listcheckvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_CheckVelocitiesAffine,this,_listgroupinfo.back(),affinedofs,_1,_2));
                    }
                    else {
                        listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesAffine,this,_listgroupinfo.back(),affinedofs,_1,_2,_3));
                    }
                    listwritefns.push_back(boost::bind(&TrajectoryRetimer::_WriteAffine,this,_listgroupinfo.back(),affinedofs,_1,_2,_3));
                }
                else if( igrouptype == 2 ) {
                    if( _parameters->_hasvelocities ) {
                        listcheckvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_CheckVelocitiesIk,this,_listgroupinfo.back(),iktype,_1,_2));
                    }
                    else {
                        listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesIk,this,_listgroupinfo.back(),iktype,_1,_2,_3));
                    }
                    listwritefns.push_back(boost::bind(&TrajectoryRetimer::_WriteIk,this,_listgroupinfo.back(),iktype,_1,_2,_3));
                }

                gpos.interpolation = posinterpolation;
                itvelgroup->interpolation = velinterpolation;
            }

            // compute the timestamps and velocities
            _timeoffset = -1;
            FOREACH(itgroup,newspec._vgroups) {
                if( itgroup->name == "deltatime" ) {
                    _timeoffset = itgroup->offset;
                }
            }

            int dof = newspec.GetDOF();
            std::vector<dReal>::iterator itdata = data.begin();
            data.at(_timeoffset) = 0;
            // set velocities to 0
            FOREACH(it, _listgroupinfo) {
                int offset = (*it)->gvel.offset;
                for(int j = 0; j < (*it)->gvel.dof; ++j) {
                    data.at(offset+j) = 0; // initial
                    data.at(data.size()-dof+offset+j) = 0; // end
                }
            }

            if( _parameters->_hastimestamps ) {
                vector<dReal> vdeltatimes;
                ptraj->GetWaypoints(0,numpoints,vdeltatimes,*itoldgrouptime);
                ConfigurationSpecification::ConvertData(data.begin(),newspec,vdeltatimes.begin(),*itoldgrouptime,numpoints,GetEnv(),false);
            }
            if( _parameters->_hasvelocities ) {
                vector<dReal> vvelocities;
                ptraj->GetWaypoints(0,numpoints,vvelocities,velspec);
                ConfigurationSpecification::ConvertData(data.begin(),newspec,vvelocities.begin(),velspec,numpoints,GetEnv(),false);
            }
            try {
                std::vector<dReal>::iterator itorgdiff = vdiffdata.begin()+oldspec.GetDOF();
                std::vector<dReal>::iterator itdataprev = itdata;
                itdata += dof;
                for(size_t i = 1; i < numpoints; ++i, itdata += dof, itorgdiff += oldspec.GetDOF()) {
                    dReal mintime = 0;
                    bool bUseEndVelocity = i+1==numpoints;
                    FOREACH(itmin,listmintimefns) {
                        dReal fgrouptime = (*itmin)(itorgdiff, itdataprev, itdata,bUseEndVelocity);
                        if( fgrouptime < 0 ) {
                            RAVELOG_DEBUG(str(boost::format("point %d/%d has uncomputable minimum time, possibly due to boundary constraints")%i%numpoints));
                            return PS_Failed;
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
                            RAVELOG_VERBOSE(str(boost::format("point %d/%d has unreachable minimum time %e > %e")%i%numpoints%(*(itdata+_timeoffset))%mintime));
                            return PS_Failed;
                        }
                    }
                    else {
                        *(itdata+_timeoffset) = mintime;
                    }
                    if( _parameters->_hasvelocities ) {
                        FOREACH(itfn,listcheckvelocityfns) {
                            if( !(*itfn)(itdataprev, itdata) ) {
                                RAVELOG_WARN(str(boost::format("point %d/%d has unreachable velocity")%i%numpoints));
                                return PS_Failed;
                            }
                        }
                    }
                    else {
                        // given the mintime, fill the velocities
                        FOREACH(itfn,listvelocityfns) {
                            (*itfn)(itorgdiff, itdataprev, itdata);
                        }
                    }

                    FOREACH(itfn,listwritefns) {
                        // because the initial time for each ramp could have been stretched to accomodate other points, it is possible for this to fail
                        if( !(*itfn)(itorgdiff, itdataprev, itdata) ) {
                            RAVELOG_DEBUG(str(boost::format("point %d/%d has unreachable new time %es, probably due to acceleration limtis violated.")%i%numpoints%(*(itdata+_timeoffset))));
                            return PS_Failed;
                        }
                    }
                    itdataprev = itdata;
                }
            }
            catch (const std::exception& ex) {
                string filename = str(boost::format("%s/failedsmoothing%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%10000));
                RAVELOG_WARN(str(boost::format("parabolic planner failed: %s, writing original trajectory to %s")%ex.what()%filename));
                ofstream f(filename.c_str());
                f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                ptraj->serialize(f);
                return PS_Failed;
            }
        }
        else {
            // one point, but still need to set interpolation
            FOREACH(itgroup,newspec._vgroups) {
                itgroup->interpolation = posinterpolation;
            }
        }

        _WriteTrajectory(ptraj,newspec, data);
        // happens too often for debug message?
        RAVELOG_VERBOSE(str(boost::format("%s path duration=%es, timestep=%es")%GetXMLId()%ptraj->GetDuration()%_parameters->_fStepLength));
        return PS_HasSolution;
    }

protected:
    // method to be overriden by individual timing types

    virtual GroupInfoPtr CreateGroupInfo(int degree, const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group &gvel) {
        return GroupInfoPtr(new GroupInfo(degree, gpos, gvel));
    }

    virtual bool _SupportInterpolation() = 0;
    virtual dReal _ComputeMinimumTimeJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) = 0;
    virtual void _ComputeVelocitiesJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) = 0;
    virtual bool _CheckVelocitiesJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        return true;
    }
    virtual bool _WriteJointValues(GroupInfoConstPtr info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata) {
        return true;
    }

    virtual dReal _ComputeMinimumTimeAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) = 0;
    virtual void _ComputeVelocitiesAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) = 0;
    virtual bool _CheckVelocitiesAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        return true;
    }
    virtual bool _WriteAffine(GroupInfoConstPtr info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata) {
        return true;
    }

    virtual dReal _ComputeMinimumTimeIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) = 0;
    virtual void _ComputeVelocitiesIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) = 0;
    virtual bool _CheckVelocitiesIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) {
        return true;
    }
    virtual bool _WriteIk(GroupInfoConstPtr info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata) {
        return true;
    }

    virtual void _WriteTrajectory(TrajectoryBasePtr ptraj, const ConfigurationSpecification& newspec, const std::vector<dReal>& data) {
        ptraj->Init(newspec);
        ptraj->Insert(0,data);
    }

    TrajectoryTimingParametersPtr _parameters;
    std::vector<dReal> _vimaxvel, _vimaxaccel;
    int _timeoffset;
    std::list<GroupInfoPtr> _listgroupinfo;
};

#endif
