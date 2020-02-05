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

void TrajectoryBase::deserialize(std::istream& I)
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
        throw OPENRAVE_EXCEPTION_FORMAT(_("error, failed to find </trajectory> in %s"),buf.str(),ORE_InvalidArguments);
    }
    xmlreaders::TrajectoryReader readerdata(GetEnv(),shared_trajectory());
    LocalXML::ParseXMLData(readerdata, pbuf.c_str(), ppsize);
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

void TrajectoryBase::Sample(std::vector<dReal>& data, dReal time, const ConfigurationSpecification& spec, bool reintializeData) const
{
    RAVELOG_VERBOSE(str(boost::format("TrajectoryBase::Sample: calling slow implementation %s")%GetXMLId()));
    vector<dReal> vinternaldata;
    Sample(vinternaldata,time);
    if( reintializeData ) {
        data.resize(0);
    }
    data.resize(spec.GetDOF(),0);
    ConfigurationSpecification::ConvertData(data.begin(),spec,vinternaldata.begin(),GetConfigurationSpecification(),1,GetEnv(),reintializeData);
}

void TrajectoryBase::SamplePoints(std::vector<dReal>& data, const std::vector<dReal>& times) const
{
    std::vector<dReal> tempdata;
    int dof = GetConfigurationSpecification().GetDOF();
    data.resize(dof*times.size());
    std::vector<dReal>::iterator itdata = data.begin();
    for(size_t i = 0; i < times.size(); ++i, itdata += dof) {
        Sample(tempdata, times[i]);
        std::copy(tempdata.begin(), tempdata.end(), itdata);
    }
}

void TrajectoryBase::SamplePoints(std::vector<dReal>& data, const std::vector<dReal>& times, const ConfigurationSpecification& spec) const
{
    std::vector<dReal> tempdata;
    data.resize(spec.GetDOF()*times.size());
    std::vector<dReal>::iterator itdata = data.begin();
    for(size_t i = 0; i < times.size(); ++i, itdata += spec.GetDOF()) {
        Sample(tempdata, times[i], spec);
        std::copy(tempdata.begin(), tempdata.end(), itdata);
    }
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

} // end namespace OpenRAVE
