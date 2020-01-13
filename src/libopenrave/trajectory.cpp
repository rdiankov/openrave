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

// To distinguish between binary and XML trajectory files
static const uint16_t MAGIC_NUMBER = 0x62ff;
static const uint16_t VERSION_NUMBER = 0x0001;  // Version number for serialization

TrajectoryBase::TrajectoryBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Trajectory,penv)
{
}

/* Helper functions for binary trajectory file writing */
inline void WriteBinaryUInt16(std::ostream& f, uint16_t value)
{
    f.write((const char*) &value, sizeof(value));
}

inline void WriteBinaryUInt32(std::ostream& f, uint32_t value)
{
    f.write((const char*) &value, sizeof(value));
}

inline void WriteBinaryInt(std::ostream& f, int value)
{
    f.write((const char*) &value, sizeof(value));
}

inline void WriteBinaryString(std::ostream& f, const std::string& s)
{
    const uint16_t length = (uint16_t) s.length();
    WriteBinaryUInt16(f, length);
    if (length > 0)
    {
        f.write(s.c_str(), length);
    }
}

inline void WriteBinaryVector(std::ostream&f, const std::vector<dReal>& v)
{
    // Indicate number of data points
    const uint32_t numDataPoints = v.size();
    WriteBinaryUInt32(f, numDataPoints);

    // Write vector memory block to binary file
    const uint64_t vectorLengthBytes = numDataPoints*sizeof(dReal);
    f.write((const char*) &v[0], vectorLengthBytes);
}

/* Helper functions for binary trajectory file reading */
inline bool ReadBinaryUInt16(std::istream& f, uint16_t& value)
{
    f.read((char*) &value, sizeof(value));
    return !!f;
}

inline bool ReadBinaryUInt32(std::istream& f, uint32_t& value)
{
    f.read((char*) &value, sizeof(value));
    return !!f;
}

inline bool ReadBinaryInt(std::istream& f, int& value)
{
    f.read((char*) &value, sizeof(value));
    return !!f;
}

inline bool ReadBinaryString(std::istream& f, std::string& s)
{
    uint16_t length = 0;
    ReadBinaryUInt16(f, length);
    if (length > 0)
    {
        s.resize(length);
        f.read(&s[0], length);
    }
    else
    {
        s.clear();
    }
    return !!f;
}

inline bool ReadBinaryVector(std::istream& f, std::vector<dReal>& v)
{
    // Get number of data points
    uint32_t numDataPoints = 0;
    ReadBinaryUInt32(f, numDataPoints);
    v.resize(numDataPoints);

    // Load binary directly to vector
    const uint64_t vectorLengthBytes = numDataPoints*sizeof(dReal);
    f.read((char*) &v[0], vectorLengthBytes);

    return !!f;
}

// New feature: Store trajectory file in binary
void TrajectoryBase::serialize(std::ostream& O, int options) const
{
    // NOTE: Ignore 'options' argument for now

    // Write binary file header
    WriteBinaryUInt16(O, MAGIC_NUMBER);
    WriteBinaryUInt16(O, VERSION_NUMBER);

    /* Store meta-data */

    // Indicate size of meta data
    const ConfigurationSpecification& spec = this->GetConfigurationSpecification();
    const uint16_t numGroups = spec._vgroups.size();
    WriteBinaryUInt16(O, numGroups);

    FOREACHC(itgroup, spec._vgroups)
    {
        WriteBinaryString(O, itgroup->name);   // Writes group name
        WriteBinaryInt(O, itgroup->offset);    // Writes offset
        WriteBinaryInt(O, itgroup->dof);       // Writes dof
        WriteBinaryString(O, itgroup->interpolation);  // Writes interpolation
    }

    /* Store data waypoints */
    std::vector<dReal> trajectoryData;
    this->GetWaypoints(0, this->GetNumWaypoints(), trajectoryData);
    WriteBinaryVector(O, trajectoryData);
}

InterfaceBasePtr TrajectoryBase::deserialize(std::istream& I)
{
    // Check whether binary or XML file
    stringstream::streampos beginningPosition = I.tellg();  // Save old position
    uint16_t binaryFileHeader = 0;
    ReadBinaryUInt16(I, binaryFileHeader);

    // Read binary trajectory files
    if (binaryFileHeader == MAGIC_NUMBER)
    {   
        uint16_t versionNumber = 0;
        ReadBinaryUInt16(I, versionNumber);

        if (versionNumber != VERSION_NUMBER)
        {
            // Throw assertion (for future versions)
        }
            
        /* Read metadata */

        // Read number of groups
        uint16_t numGroups = 0;
        ReadBinaryUInt16(I, numGroups);

        ConfigurationSpecification spec;
        spec._vgroups.resize(numGroups);
        FOREACH(itgroup, spec._vgroups)
        {
            ReadBinaryString(I, itgroup->name);             // Read group name
            ReadBinaryInt(I, itgroup->offset);              // Read offset
            ReadBinaryInt(I, itgroup->dof);                 // Read dof
            ReadBinaryString(I, itgroup->interpolation);    // Read interpolation
        }
        this->Init(spec);

        /* Read trajectory data */
        std::vector<dReal> trajectoryData;
        ReadBinaryVector(I, trajectoryData);
        this->Insert(this->GetNumWaypoints(), trajectoryData);
        return shared_from_this();
    }
    // Backwards compatible with old XML trajectory files
    
    stringbuf buf;
    I.seekg((size_t) beginningPosition);    // Move back to old position
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
