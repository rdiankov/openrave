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
#include "plugindefs.h"
#include "plannerparameters.h"

class LinearTrajectoryRetimer : public PlannerBase
{
    struct GroupInfo
    {
        GroupInfo(int degree, const ConfigurationSpecification::Group& gpos, const ConfigurationSpecification::Group &gvel) : degree(degree), gpos(gpos), gvel(gvel) {
        }
        int degree;
        const ConfigurationSpecification::Group& gpos, &gvel;
        int orgdofoffset;
    };

public:
    LinearTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
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
        string posinterpolation = "linear";
        string velinterpolation = "next";

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
                        listmintimefns.push_back(boost::bind(&LinearTrajectoryRetimer::_ComputeMinimumTimeJointValues,this,boost::ref(listgroupinfo.back()),_1,_2,_3,_4));
                    }
                    else if( igrouptype == 1 ) {
                        listmintimefns.push_back(boost::bind(&LinearTrajectoryRetimer::_ComputeMinimumTimeAffine,this,boost::ref(listgroupinfo.back()),affinedofs,_1,_2,_3,_4));
                    }
                    else if( igrouptype == 2 ) {
                        listmintimefns.push_back(boost::bind(&LinearTrajectoryRetimer::_ComputeMinimumTimeIk,this,boost::ref(listgroupinfo.back()),iktype,_1,_2,_3,_4));
                    }
                }

                if( igrouptype == 0 ) {
                    listvelocityfns.push_back(boost::bind(&LinearTrajectoryRetimer::_ComputeVelocitiesJointValues,this,boost::ref(listgroupinfo.back()),_1,_2,_3));
                }
                else if( igrouptype == 1 ) {
                    listvelocityfns.push_back(boost::bind(&LinearTrajectoryRetimer::_ComputeVelocitiesAffine,this,boost::ref(listgroupinfo.back()),affinedofs,_1,_2,_3));
                }
                else if( igrouptype == 2 ) {
                    listvelocityfns.push_back(boost::bind(&LinearTrajectoryRetimer::_ComputeVelocitiesIk,this,boost::ref(listgroupinfo.back()),iktype,_1,_2,_3));
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
                itgroup->interpolation = "linear";
            }
        }

        // finally initialize the output trajectory
        ptraj->Init(newspec);
        ptraj->Insert(0,data);
        RAVELOG_DEBUG(str(boost::format("LinearTrajectoryRetimer path length=%fs")%ptraj->GetDuration()));
        return PS_HasSolution;
    }

protected:
    dReal _ComputeMinimumTimeJointValues(GroupInfo& info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity)
    {
        dReal bestmintime = 0;
        for(int i = 0; i < info.gpos.dof; ++i) {
            dReal mintime = RaveFabs(*(itorgdiff+info.orgdofoffset+i)*_vimaxvel.at(info.orgdofoffset+i));
            bestmintime = max(bestmintime,mintime);
        }
        return bestmintime;
    }

    void _ComputeVelocitiesJointValues(GroupInfo& info, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata)
    {
        dReal invdeltatime = 1.0 / *(itdata+_timeoffset);
        for(int i = 0; i < info.gpos.dof; ++i) {
            *(itdata+info.gvel.offset+i) = *(itorgdiff+info.orgdofoffset+i)*invdeltatime;
        }
    }

    dReal _ComputeMinimumTimeAffine(GroupInfo& info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity)
    {
        dReal bestmintime = 0;
        const boost::array<DOFAffine,4> testdofs={{DOF_X,DOF_Y,DOF_Z,DOF_RotationAxis}};
        dReal distxyz = 0;
        dReal fivel=0;
        FOREACHC(itdof,testdofs) {
            if( affinedofs & *itdof ) {
                int index = RaveGetIndexFromAffineDOF(affinedofs,*itdof);
                dReal f = *(itorgdiff+index);
                distxyz += f*f;
                fivel = _vimaxvel.at(info.orgdofoffset+index);
            }
        }
        if( distxyz > 0 ) {
            dReal mintime = RaveSqrt(distxyz)*fivel;
            bestmintime = max(bestmintime,mintime);
        }
        if( affinedofs & DOF_RotationAxis ) {
            int index = RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationAxis);
            dReal mintime = RaveFabs(*(itorgdiff+index)*_vimaxvel.at(info.orgdofoffset+index));
            bestmintime = max(bestmintime,mintime);
        }
        else if( affinedofs & DOF_RotationQuat ) {
            int index = RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationQuat);
            Vector qprev,qnext;
            for(int i = 0; i < 4; ++i) {
                qnext[i] = *(itdata+info.gpos.offset+index+i);
                qprev[i] = *(itdataprev+info.gpos.offset+index+i);
            }
            dReal mintime = RaveAcos(min(dReal(1),RaveFabs(qprev.dot(qnext))))*_vimaxvel.at(info.orgdofoffset+index);
            bestmintime = max(bestmintime,mintime);
        }
        else if( affinedofs & DOF_Rotation3D ) {
            RAVELOG_WARN("_ComputeMinimumTimeAffine does not support DOF_Rotation3D\n");
        }
        return bestmintime;
    }

    void _ComputeVelocitiesAffine(GroupInfo& info, int affinedofs, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata)
    {
        dReal invdeltatime = 1.0 / *(itdata+_timeoffset);
        const boost::array<DOFAffine,4> testdofs={{DOF_X,DOF_Y,DOF_Z,DOF_RotationAxis}};
        FOREACHC(itdof,testdofs) {
            if( affinedofs & *itdof ) {
                int index = RaveGetIndexFromAffineDOF(affinedofs,*itdof);
                *(itdata+info.gvel.offset+index) = *(itorgdiff+info.orgdofoffset+index)*invdeltatime;
            }
        }
        if( affinedofs & DOF_RotationQuat ) {
            int index = RaveGetIndexFromAffineDOF(affinedofs,DOF_RotationQuat);
            for(int i = 0; i < 4; ++i) {
                *(itdata+info.gvel.offset+index+i) = *(itorgdiff+index+i)*invdeltatime;
            }
        }
        else if( affinedofs & DOF_Rotation3D ) {
            RAVELOG_WARN("_ComputeMinimumTimeAffine does not support DOF_Rotation3D\n");
        }
    }

    dReal _ComputeMinimumTimeIk(GroupInfo& info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::const_iterator itdata, bool bUseEndVelocity)
    {
        IkParameterization ikparamprev, ikparam;
        ikparamprev.Set(itdataprev,iktype);
        ikparam.Set(itdata,iktype);
        switch(iktype) {
        case IKP_Transform6D: {
            dReal quatmintime = RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetTransform6D().rot.dot(ikparam.GetTransform6D().rot))))*_vimaxvel.at(info.orgdofoffset+0);
            dReal transmintime = RaveSqrt((ikparamprev.GetTransform6D().trans-ikparam.GetTransform6D().trans).lengthsqr3())*_vimaxvel.at(info.orgdofoffset+4);
            return max(quatmintime,transmintime);
        }
        case IKP_Rotation3D:
            return RaveAcos(min(dReal(1),RaveFabs(ikparamprev.GetRotation3D().dot(ikparam.GetRotation3D()))))*_vimaxvel.at(info.orgdofoffset+0);
        case IKP_Translation3D:
            return RaveSqrt((ikparamprev.GetTranslation3D()-ikparam.GetTranslation3D()).lengthsqr3())*_vimaxvel.at(info.orgdofoffset);
        case IKP_Direction3D: {
            return RaveAcos(min(dReal(1),ikparamprev.GetDirection3D().dot3(ikparam.GetDirection3D())))*_vimaxvel.at(info.orgdofoffset);
        }
        case IKP_Ray4D: {
            Vector pos0 = ikparamprev.GetRay4D().pos - ikparamprev.GetRay4D().dir*ikparamprev.GetRay4D().dir.dot(ikparamprev.GetRay4D().pos);
            Vector pos1 = ikparam.GetRay4D().pos - ikparam.GetRay4D().dir*ikparam.GetRay4D().dir.dot(ikparam.GetRay4D().pos);
            dReal fcos = ikparamprev.GetRay4D().dir.dot(ikparam.GetRay4D().dir);
            dReal facos = fcos >= 1 ? 0 : RaveAcos(fcos);
            return max(facos*_vimaxvel.at(info.orgdofoffset),(pos0-pos1).lengthsqr3()*_vimaxvel.at(info.orgdofoffset+3));
        }
        case IKP_Lookat3D:
            return RaveSqrt((ikparamprev.GetLookat3D()-ikparam.GetLookat3D()).lengthsqr3())*_vimaxvel.at(info.orgdofoffset);
        case IKP_TranslationDirection5D: {
            dReal dirmintime = RaveAcos(min(dReal(1),ikparamprev.GetTranslationDirection5D().dir.dot3(ikparam.GetTranslationDirection5D().dir)))*_vimaxvel.at(info.orgdofoffset);
            dReal transmintime = RaveSqrt((ikparamprev.GetTranslationDirection5D().pos-ikparam.GetTranslationDirection5D().pos).lengthsqr3())*_vimaxvel.at(info.orgdofoffset+3);
            return max(dirmintime,transmintime);
        }
        case IKP_TranslationXY2D:
            return RaveSqrt((ikparamprev.GetTranslationXY2D()-ikparam.GetTranslationXY2D()).lengthsqr2())*_vimaxvel.at(info.orgdofoffset+0);
        case IKP_TranslationXYOrientation3D: {
            dReal angmintime = ANGLE_DIFF(ikparam.GetTranslationXYOrientation3D().z,ikparamprev.GetTranslationXYOrientation3D().z)*_vimaxvel.at(info.orgdofoffset+2);
            dReal transmintime = RaveSqrt((ikparamprev.GetTranslationXYOrientation3D()-ikparam.GetTranslationXYOrientation3D()).lengthsqr2())*_vimaxvel.at(info.orgdofoffset+0);
            return max(angmintime,transmintime);
        }
        case IKP_TranslationLocalGlobal6D: {
            dReal transmintime0 = RaveSqrt((ikparamprev.GetTranslationLocalGlobal6D().first-ikparam.GetTranslationLocalGlobal6D().first).lengthsqr3())*_vimaxvel.at(info.orgdofoffset+0);
            dReal transmintime1 = RaveSqrt((ikparamprev.GetTranslationLocalGlobal6D().second-ikparam.GetTranslationLocalGlobal6D().second).lengthsqr3())*_vimaxvel.at(info.orgdofoffset+3);
            return max(transmintime0,transmintime1);
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("does not support parameterization 0x%x", ikparam.GetType(),ORE_InvalidArguments);
        }
    }

    void _ComputeVelocitiesIk(GroupInfo& info, IkParameterizationType iktype, std::vector<dReal>::const_iterator itorgdiff, std::vector<dReal>::const_iterator itdataprev, std::vector<dReal>::iterator itdata)
    {
        dReal invdeltatime = 1.0 / *(itdata+_timeoffset);
        // can probably do better...
        for(int i = 0; i < info.gpos.dof; ++i) {
            *(itdata+info.gvel.offset+i) = *(itorgdiff+info.orgdofoffset+i)*invdeltatime;
        }
    }

    TrajectoryTimingParametersPtr _parameters;
    vector<dReal> _vimaxvel, _vimaxaccel;
    int _timeoffset;
};


PlannerBasePtr CreateLinearTrajectoryRetimer(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new LinearTrajectoryRetimer(penv, sinput));
}
