// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "rplanners.h"

class TrajectoryRetimer : public PlannerBase
{
public:
    TrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\ntrajectory re-timing without modifying any of the points. Overwrites the velocities and timestamps.";
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
        return _parameters->_interpolation == "linear";
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        BOOST_ASSERT(!!_parameters && !!ptraj && ptraj->GetEnv()==GetEnv());
        BOOST_ASSERT(_parameters->GetDOF() == ptraj->GetConfigurationSpecification().GetDOF());
        size_t numpoints = ptraj->GetNumWaypoints();
        const ConfigurationSpecification& oldspec = ptraj->GetConfigurationSpecification();
        ConfigurationSpecification newspec = oldspec;
        newspec.AddVelocityGroups(true);
        vector<dReal> vdiffdata, data;
        ptraj->GetWaypoints(0,numpoints,vdiffdata);
        data.resize(numpoints*newspec.GetDOF(),0);
        ConfigurationSpecification::ConvertData(data.begin(),newspec,vdiffdata.begin(),oldspec,numpoints,GetEnv());

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
            list< boost::function<dReal(std::vector<dReal>::const_iterator,std::vector<dReal>::const_iterator,bool) > > listmintimefns;
            list< boost::function<void(std::vector<dReal>::const_iterator,std::vector<dReal>::iterator) > > listvelocityfns;
            list< std::vector<ConfigurationSpecification::Group>::iterator > listvelocitygroups;
            for(size_t i = 0; i < newspec._vgroups.size(); ++i) {
                ConfigurationSpecification::Group& gpos = newspec._vgroups[i];
                if( gpos.name.size() >= 12 && gpos.name.substr(0,12) == "joint_values" ) {
                    int orgdofoffset = oldspec.FindCompatibleGroup(gpos)->offset;
                    BOOST_ASSERT(orgdofoffset+gpos.dof <= _parameters->GetDOF());
                    std::vector<ConfigurationSpecification::Group>::iterator itvelgroup = newspec._vgroups.begin()+(newspec.FindTimeDerivativeGroup(gpos)-newspec._vgroups.begin());
                    BOOST_ASSERT(itvelgroup != newspec._vgroups.end());
                    listvelocitygroups.push_back(itvelgroup);
                    if( _parameters->_interpolation == "linear" ) {
                        if( !_parameters->_hastimestamps ) {
                            listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeLinearJointValues,this,boost::ref(gpos),boost::ref(*itvelgroup),orgdofoffset,_1,_2,_3));
                        }
                        listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesLinearJointValues,this,boost::ref(gpos),boost::ref(*itvelgroup),orgdofoffset,_1,_2));
                        gpos.interpolation = "linear";
                        itvelgroup->interpolation = "next";
                    }
                }
                else if( gpos.name.size() >= 16 && gpos.name.substr(0,16) == "affine_transform" ) {
                    int orgdofoffset = oldspec.FindCompatibleGroup(gpos)->offset;
                    BOOST_ASSERT(orgdofoffset+gpos.dof <= _parameters->GetDOF());
                    std::vector<ConfigurationSpecification::Group>::iterator itvelgroup = newspec._vgroups.begin()+(newspec.FindTimeDerivativeGroup(gpos)-newspec._vgroups.begin());
                    BOOST_ASSERT(itvelgroup != newspec._vgroups.end());
                    listvelocitygroups.push_back(itvelgroup);
                    stringstream ss(gpos.name.substr(16));
                    string bodyname;
                    int affinedofs=0;
                    ss >> bodyname >> affinedofs;
                    if( _parameters->_interpolation == "linear" ) {
                        if( !_parameters->_hastimestamps ) {
                            listmintimefns.push_back(boost::bind(&TrajectoryRetimer::_ComputeMinimumTimeLinearAffine,this,boost::ref(gpos),boost::ref(*itvelgroup),orgdofoffset,affinedofs,_1,_2,_3));
                        }
                        listvelocityfns.push_back(boost::bind(&TrajectoryRetimer::_ComputeVelocitiesLinearAffine,this,boost::ref(gpos),boost::ref(*itvelgroup),orgdofoffset,affinedofs,_1,_2));
                        gpos.interpolation = "linear";
                        itvelgroup->interpolation = "next";
                    }
                }
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
            FOREACH(it, listvelocitygroups) {
                int offset = (*it)->offset;
                for(int j = 0; j < (*it)->dof; ++j) {
                    data.at(offset+j) = 0; // initial
                    data.at(data.size()-dof+offset+j) = 0; // end
                }
            }
            std::vector<dReal>::iterator itorgdiff = vdiffdata.begin()+oldspec.GetDOF();
            itdata += dof;
            for(size_t i = 1; i < numpoints; ++i, itdata += dof, itorgdiff += oldspec.GetDOF()) {
                dReal mintime = 0;
                bool bUseEndVelocity = i+1==numpoints;
                FOREACH(itmin,listmintimefns) {
                    dReal time = (*itmin)(itorgdiff, itdata,bUseEndVelocity);
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
                        (*itfn)(itorgdiff, itdata);
                    }
                }
            }
        }
        else {
            // one point, but still need to set interpolation
            FOREACH(itgroup,newspec._vgroups) {
                itgroup->interpolation = "linear";
            }
        }

        // finally initialize the output trajectory
        ptraj->Init(newspec);
        ptraj->Insert(0,data);
        return PS_HasSolution;
    }

protected:

    dReal _ComputeMinimumTimeLinearJointValues(const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group& gvel, int orgdofoffset, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itpoint, bool bUseEndVelocity)
    {
        dReal bestmintime = 0;
        for(int i = 0; i < gpos.dof; ++i) {
            dReal mintime = RaveFabs(*(itorgdiff+orgdofoffset+i)*_vimaxvel.at(orgdofoffset+i));
            bestmintime = max(bestmintime,mintime);
        }
        return bestmintime;
    }

    void _ComputeVelocitiesLinearJointValues(const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group& gvel, int orgdofoffset, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::iterator itpoint)
    {
        dReal invdeltatime = 1.0 / *(itpoint+_timeoffset);
        for(int i = 0; i < gpos.dof; ++i) {
            *(itpoint+gvel.offset+i) = *(itorgdiff+orgdofoffset+i)*invdeltatime;
        }
    }

    dReal _ComputeMinimumTimeLinearAffine(const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group& gvel, int orgdofoffset, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itpoint, bool bUseEndVelocity)
    {
        dReal bestmintime = 0;
        const boost::array<DOFAffine,4> testdofs={{DOF_X,DOF_Y,DOF_Z,DOF_RotationAxis}};
        FOREACHC(itdof,testdofs) {
            if( affinedofs & *itdof ) {
                int index = RaveGetIndexFromAffineDOF(affinedofs,*itdof);
                dReal mintime = RaveFabs(*(itorgdiff+index)*_vimaxvel.at(orgdofoffset+index));
                bestmintime = max(bestmintime,mintime);
            }
        }
        if( affinedofs & DOF_RotationQuat ) {
            int index = RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationQuat);
            Vector qprev,qnext;
            for(int i = 0; i < 4; ++i) {
                qnext[i] = *(itpoint+gpos.offset+index+i);
                qprev[i] = qnext[i] - *(itorgdiff+index+i);
            }
            dReal mintime = RaveAcos(min(dReal(1),RaveFabs(qprev.dot(qnext))))/_vimaxvel.at(orgdofoffset+index);
            bestmintime = max(bestmintime,mintime);
        }
        if( affinedofs & DOF_Rotation3D ) {
            RAVELOG_WARN("_ComputeMinimumTimeLinearAffine does not support DOF_Rotation3D\n");
        }
        return bestmintime;
    }

    void _ComputeVelocitiesLinearAffine(const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group& gvel, int orgdofoffset, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::iterator itpoint)
    {
        dReal invdeltatime = 1.0 / *(itpoint+_timeoffset);
        const boost::array<DOFAffine,4> testdofs={{DOF_X,DOF_Y,DOF_Z,DOF_RotationAxis}};
        FOREACHC(itdof,testdofs) {
            if( affinedofs & *itdof ) {
                int index = RaveGetIndexFromAffineDOF(affinedofs,*itdof);
                *(itpoint+gvel.offset+index) = *(itorgdiff+orgdofoffset+index)*invdeltatime;
            }
        }
        if( affinedofs & DOF_RotationQuat ) {
            int index = RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationQuat);
            for(int i = 0; i < 4; ++i) {
                *(itpoint+gvel.offset+index+i) = *(itorgdiff+index+i)*invdeltatime;
            }
        }
        if( affinedofs & DOF_Rotation3D ) {
            RAVELOG_WARN("_ComputeMinimumTimeLinearAffine does not support DOF_Rotation3D\n");
        }
    }

    TrajectoryTimingParametersPtr _parameters;
    vector<dReal> _vimaxvel, _vimaxaccel;
    int _timeoffset;
};


PlannerBasePtr CreateTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new TrajectoryRetimer(penv, sinput));
}
