// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#include "libopenrave.h"
#include <boost/lexical_cast.hpp>
#include <openrave/planningutils.h>
#include <openrave/xmlreaders.h>

namespace OpenRAVE {

TrajectoryBase::TrajectoryBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Trajectory,penv)
{
}

void TrajectoryBase::serialize(std::ostream& O, int options) const
{
    O << "<trajectory type=\"" << GetXMLId() << "\">" << endl << GetConfigurationSpecification();
    O << "<data count=\"" << GetNumWaypoints() << "\">" << endl;
    std::vector<dReal> data;
    GetWaypoints(0,GetNumWaypoints(),data);
    FOREACHC(it,data){
        O << *it << " ";
    }
    O << "</data>" << endl;
    if( GetDescription().size() > 0 ) {
        O << "<description><![CDATA[" << GetDescription() << "]]></description>" << endl;
    }
    if( GetReadableInterfaces().size() > 0 ) {
        xmlreaders::StreamXMLWriterPtr writer(new xmlreaders::StreamXMLWriter("readable"));
        FOREACHC(it, GetReadableInterfaces()) {
            BaseXMLWriterPtr newwriter = writer->AddChild(it->first);
            it->second->Serialize(newwriter,options);
        }
        writer->Serialize(O);
    }
    O << "</trajectory>" << endl;
}

InterfaceBasePtr TrajectoryBase::deserialize(std::istream& I)
{
    stringbuf buf;
    stringstream::streampos pos = I.tellg();
    I.get(buf, 0); // get all the data, yes this is inefficient, not sure if there anyway to search in streams
    BOOST_ASSERT(!!I);

    string pbuf = buf.str();
    const char* p = strcasestr(pbuf.c_str(), "</trajectory>");
    int ppsize=-1;
    if( p != NULL ) {
        I.clear();
        ppsize=(p-pbuf.c_str())+13;
        I.seekg((size_t)pos+ppsize);
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT("error, failed to find </trajectory> in %s",buf.str(),ORE_InvalidArguments);
    }
    xmlreaders::TrajectoryReader reader(GetEnv(),shared_trajectory());
    LocalXML::ParseXMLData(BaseXMLReaderPtr(&reader,utils::null_deleter()), pbuf.c_str(), ppsize);
    return shared_from_this();
}

void TrajectoryBase::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    InterfaceBase::Clone(preference,cloningoptions);
    TrajectoryBaseConstPtr r = RaveInterfaceConstCast<TrajectoryBase>(preference);
    Init(r->GetConfigurationSpecification());
    vector<dReal> data;
    r->GetWaypoints(0,r->GetNumWaypoints(),data);
    Insert(0,data);
}

void TrajectoryBase::Sample(std::vector<dReal>& data, dReal time, const ConfigurationSpecification& spec) const
{
    RAVELOG_VERBOSE(str(boost::format("TrajectoryBase::Sample: calling slow implementation %s")%GetXMLId()));
    vector<dReal> vinternaldata;
    Sample(vinternaldata,time);
    data.resize(spec.GetDOF());
    ConfigurationSpecification::ConvertData(data.begin(),spec,vinternaldata.begin(),GetConfigurationSpecification(),1,GetEnv());
}

void TrajectoryBase::GetWaypoints(size_t startindex, size_t endindex, std::vector<dReal>& data, const ConfigurationSpecification& spec) const
{
    RAVELOG_VERBOSE(str(boost::format("TrajectoryBase::GetWaypoints: calling slow implementation %s")%GetXMLId()));
    vector<dReal> vinternaldata;
    GetWaypoints(startindex,endindex,vinternaldata);
    data.resize(spec.GetDOF()*(endindex-startindex),0);
    if( startindex < endindex ) {
        ConfigurationSpecification::ConvertData(data.begin(),spec,vinternaldata.begin(),GetConfigurationSpecification(),endindex-startindex,GetEnv());
    }
}

// Old API

bool TrajectoryBase::SampleTrajectory(dReal time, TrajectoryBase::Point& tp) const
{
    std::vector<dReal> data;
    Sample(data,time);
    tp.q.resize(0);
    tp.trans = Transform();
    GetConfigurationSpecification().ExtractTransform(tp.trans,data.begin(),KinBodyConstPtr());
    FOREACHC(itgroup,GetConfigurationSpecification()._vgroups) {
        if( itgroup->name.size() >= 12 && itgroup->name.substr(0,12) == string("joint_values") ) {
            tp.q.resize(itgroup->dof);
            std::copy(data.begin()+itgroup->offset,data.begin()+itgroup->offset+itgroup->dof,tp.q.begin());
        }
    }
    return true;
}

const std::vector<TrajectoryBase::Point>& TrajectoryBase::GetPoints() const
{
    __vdeprecatedpoints.resize(GetNumWaypoints());
    std::vector<dReal> vdata;
    GetWaypoints(0,GetNumWaypoints(),vdata);
    ConfigurationSpecification::Group joint_values, affine_transform;
    int affinedofs=0;
    FOREACHC(itgroup,GetConfigurationSpecification()._vgroups) {
        if( itgroup->name.size() >= 12 && itgroup->name.substr(0,12) == string("joint_values") ) {
            joint_values = *itgroup;
        }
        else if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == string("affine_transform") ) {
            affine_transform = *itgroup;
            stringstream ss(affine_transform.name);
            string semantic, robotname, dofs;
            ss >> semantic >> robotname >> dofs;
            if( !!ss ) {
                affinedofs = boost::lexical_cast<int>(dofs);
            }
        }
    }

    for(size_t i = 0; i < __vdeprecatedpoints.size(); ++i) {
        TrajectoryBase::Point& tp = __vdeprecatedpoints[i];
        tp.q.resize(joint_values.dof);
        std::vector<dReal>::iterator itdata = vdata.begin()+GetConfigurationSpecification().GetDOF()*i;
        std::copy(itdata+joint_values.offset,itdata+joint_values.offset+joint_values.dof,tp.q.begin());
        tp.trans = Transform();
        RaveGetTransformFromAffineDOFValues(tp.trans,itdata+affine_transform.offset,affinedofs);
    }

    return __vdeprecatedpoints;
}

void TrajectoryBase::AddPoint(const Point& p)
{
    ConfigurationSpecification spec;
    spec._vgroups.reserve(4);
    std::vector<dReal> v; v.reserve(p.q.size()+p.qdot.size()+8);
    int dof = 0;
    if( p.q.size() > 0 ) {
        spec._vgroups.push_back(ConfigurationSpecification::Group());
        spec._vgroups.back().name = "joint_values";
        spec._vgroups.back().offset = dof;
        spec._vgroups.back().dof = p.q.size();
        v.resize(dof+spec._vgroups.back().dof);
        for(size_t i = 0; i < p.q.size(); ++i) {
            v[dof+i] = p.q[i];
        }
        dof += p.q.size();
    }
    if( p.qdot.size() > 0 ) {
        BOOST_ASSERT(p.qdot.size() == p.q.size());
        spec._vgroups.push_back(ConfigurationSpecification::Group());
        spec._vgroups.back().name = "joint_velocities";
        spec._vgroups.back().offset = dof;
        spec._vgroups.back().dof = p.qdot.size();
        v.resize(dof+spec._vgroups.back().dof);
        for(size_t i = 0; i < p.q.size(); ++i) {
            v[dof+i] = p.qdot[i];
        }
        dof += p.qdot.size();
    }

    spec._vgroups.push_back(ConfigurationSpecification::Group());
    spec._vgroups.back().name = str(boost::format("affine_transform __dummy__ %d")%DOF_Transform);
    spec._vgroups.back().offset = dof;
    spec._vgroups.back().dof = RaveGetAffineDOF(DOF_Transform);
    v.resize(dof+spec._vgroups.back().dof);
    RaveGetAffineDOFValuesFromTransform(v.begin()+dof,p.trans,DOF_Transform);
    dof += spec._vgroups.back().dof;

    spec._vgroups.push_back(ConfigurationSpecification::Group());
    spec._vgroups.back().name = "deltatime";
    spec._vgroups.back().offset = dof;
    spec._vgroups.back().dof = dof;
    v.resize(dof+spec._vgroups.back().dof);
    v.at(dof) = p.time;
    dof += 1;
    if( GetConfigurationSpecification().GetDOF() == 0 ) {
        Init(spec);
    }
    Insert(GetNumWaypoints(),v,spec);
}

bool TrajectoryBase::CalcTrajTiming(RobotBasePtr probot, int interp,  bool autocalc, bool activedof, dReal fmaxvelmult)
{
    if( activedof ) {
        planningutils::RetimeActiveDOFTrajectory(shared_trajectory(),probot, !autocalc,fmaxvelmult);
    }
    else if( !!probot ) {
        RobotBase::RobotStateSaver saver(probot);
        vector<int> indices(probot->GetDOF());
        for(int i = 0; i < probot->GetDOF(); ++i) {
            indices[i] = i;
        }
        probot->SetActiveDOFs(indices);
        planningutils::RetimeActiveDOFTrajectory(shared_trajectory(),probot,!autocalc,fmaxvelmult);
    }
    else {
        RAVELOG_WARN("CalcTrajTiming failed, need to specify robot\n");
    }
    return true;
}

} // end namespace OpenRAVE
