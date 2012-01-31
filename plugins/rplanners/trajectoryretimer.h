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

class TrajectoryRetimer : public PlannerBase
{
protected:
    struct GroupInfo
    {
        GroupInfo(int degree, const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group &gvel) : degree(degree), gpos(gpos), gvel(gvel) {
        }
        int degree;
        const ConfigurationSpecification::Group& gpos, &gvel;
        int orgdofoffset;
    };

public:
    TrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\nTrajectory re-timing without modifying any of the points. Overwrites the velocities and timestamps.";
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new TrajectoryTimingParameters());
        _parameters->copy(params);
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new TrajectoryTimingParameters());
        isParameters >> *_parameters;
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
        BOOST_ASSERT(!!_parameters && !!ptraj && ptraj->GetEnv()==GetEnv());
        BOOST_ASSERT(_parameters->GetDOF() == _parameters->_configurationspecification.GetDOF());
        size_t numpoints = ptraj->GetNumWaypoints();
        const ConfigurationSpecification& oldspec = _parameters->_configurationspecification;
        ConfigurationSpecification newspec = ptraj->GetConfigurationSpecification();
        newspec.AddVelocityGroups(true);
        vector<dReal> vdiffdata, data;
        ptraj->GetWaypoints(0,numpoints,vdiffdata,oldspec);
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
            list<GroupInfo> listgroupinfo;
            list< boost::function<dReal(std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,bool) > > listmintimefns;
            list< boost::function<void(std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,std::vector<dReal>::iterator) > > listvelocityfns;
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
                int orgdofoffset = oldspec.FindCompatibleGroup(gpos)->offset;
                BOOST_ASSERT(orgdofoffset+gpos.dof <= _parameters->GetDOF());
                std::vector<ConfigurationSpecification::Group>::iterator itvelgroup = newspec._vgroups.begin()+(newspec.FindTimeDerivativeGroup(gpos)-newspec._vgroups.begin());
                BOOST_ASSERT(itvelgroup != newspec._vgroups.end());
                listgroupinfo.push_back(GroupInfo(degree,gpos,*itvelgroup));
                listgroupinfo.back().orgdofoffset = orgdofoffset;

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

                if( !_parameters->_hastimestamps ) {
                    if( igrouptype == 0 ) {
                        listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeJointValues,this,boost::ref(listgroupinfo.back()),_1,_2,_3,_4));
                    }
                    else if( igrouptype == 1 ) {
                        listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeAffine,this,boost::ref(listgroupinfo.back()),affinedofs,_1,_2,_3,_4));
                    }
                    else if( igrouptype == 2 ) {
                        listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeIk,this,boost::ref(listgroupinfo.back()),iktype,_1,_2,_3,_4));
                    }
                }

                if( igrouptype == 0 ) {
                    listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesJointValues,this,boost::ref(listgroupinfo.back()),_1,_2,_3));
                }
                else if( igrouptype == 1 ) {
                    listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesAffine,this,boost::ref(listgroupinfo.back()),affinedofs,_1,_2,_3));
                }
                else if( igrouptype == 2 ) {
                    listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesIk,this,boost::ref(listgroupinfo.back()),iktype,_1,_2,_3));
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
            FOREACH(it, listgroupinfo) {
                int offset = it->gvel.offset;
                for(int j = 0; j < it->gvel.dof; ++j) {
                    data.at(offset+j) = 0; // initial
                    data.at(data.size()-dof+offset+j) = 0; // end
                }
            }
            std::vector<dReal>::iterator itorgdiff = vdiffdata.begin()+oldspec.GetDOF();
            std::vector<dReal>::iterator itdataprev = itdata;
            itdata += dof;
            for(size_t i = 1; i < numpoints; ++i, itdata += dof, itorgdiff += oldspec.GetDOF()) {
                dReal mintime = 0;
                bool bUseEndVelocity = i+1==numpoints;
                FOREACH(itmin,listmintimefns) {
                    dReal time = (*itmin)(itorgdiff, itdataprev, itdata,bUseEndVelocity);
                    if( mintime < time ) {
                        mintime = time;
                    }
                }
                if( _parameters->_hastimestamps ) {
                    if( *(itdata+_timeoffset) < mintime ) {
                        RAVELOG_WARN(str(boost::format("point %d has unreachable minimum time %f > %f, changing...")%i%(*(itdata+_timeoffset))%mintime));
                        *(itdata+_timeoffset) = mintime;
                    }
                }
                else {
                    *(itdata+_timeoffset) = mintime;
                }
                if( !bUseEndVelocity ) {
                    // given the mintime, fill the velocities
                    FOREACH(itfn,listvelocityfns) {
                        (*itfn)(itorgdiff, itdataprev, itdata);
                    }
                }
                itdataprev = itdata;
            }
        }
        else {
            // one point, but still need to set interpolation
            FOREACH(itgroup,newspec._vgroups) {
                itgroup->interpolation = posinterpolation;
            }
        }

        // finally initialize the output trajectory
        ptraj->Init(newspec);
        ptraj->Insert(0,data);
        RAVELOG_DEBUG(str(boost::format("TrajectoryRetimer path length=%fs")%ptraj->GetDuration()));
        return PS_HasSolution;
    }

protected:
    // method to be overriden by individual timing types

    virtual bool _SupportInterpolation() = 0;
    virtual dReal _ComputeMinimumTimeJointValues(GroupInfo& info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) = 0;
    virtual void _ComputeVelocitiesJointValues(GroupInfo& info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) = 0;
    virtual dReal _ComputeMinimumTimeAffine(GroupInfo& info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) = 0;
    virtual void _ComputeVelocitiesAffine(GroupInfo& info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) = 0;
    virtual dReal _ComputeMinimumTimeIk(GroupInfo& info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity) = 0;
    virtual void _ComputeVelocitiesIk(GroupInfo& info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata) = 0;

    TrajectoryTimingParametersPtr _parameters;
    vector<dReal> _vimaxvel, _vimaxaccel;
    int _timeoffset;
};

#endif
