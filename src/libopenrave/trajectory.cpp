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
            // some readable are not xml readable and does not get serialized here
            if( !!it->second ) {
                BaseXMLWriterPtr newwriter = writer->AddChild(it->first);
                it->second->SerializeXML(newwriter,options);
            }
        }
        writer->Serialize(O);
    }
    O << "</trajectory>" << endl;
}

void TrajectoryBase::deserialize(std::istream& I)
{
    stringbuf buf;
    stringstream::pos_type pos = I.tellg();
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
    xmlreaders::ParseXMLData(readerdata, pbuf.c_str(), ppsize);
}

void TrajectoryBase::DeserializeFromRawData(const uint8_t* pdata, size_t nDataSize)
{
    xmlreaders::TrajectoryReader readerdata(GetEnv(),shared_trajectory());
    xmlreaders::ParseXMLData(readerdata, (const char*)pdata, nDataSize);
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

/// \brief generator of range from start to stop
template<typename T>
class RangeGenerator
{
 public:
    RangeGenerator(T step, T start, T stop, bool ensureLast)
        : _index(0), _numPoints(std::ceil((stop - start)/step)), _start(start), _step(step), _stop(stop), _ensureLast(ensureLast)
    {
    }

    /// \brief resets current index to start
    void Reset()
    {
        _index = 0;
    }

    /// \brief return current value and iterate to next value
    /// \return current value
    T GetAndIncrement()
    {
        {
            const T value = _start + _step * _index;
            const bool lessThanStop = _index < _numPoints;
            if (lessThanStop) {
                ++_index;
                return value;
            }
        }
        if (_index == _numPoints) {
            if (_IsPaddedByEnsureLast()) {
                ++_index;
                return _stop;
            }
        }
        throw std::out_of_range("index=" + std::to_string(_index) + " is out of size=" + std::to_string(GetSize()) + ", ensureLast=" + std::to_string(_ensureLast));
    }

    /// \brief size of range
    size_t GetSize() const
    {
        return _numPoints + _IsPaddedByEnsureLast();
    }

 private:
    bool _IsPaddedByEnsureLast() const
    {
        return _ensureLast && (_start + _step * (_numPoints - 1)) < _stop;
    }

    size_t _index; ///< current index, can go up to _numPoints if _ensureLast is true and other conditions are met.
    const size_t _numPoints; ///< number of data points. excludes padding added by _ensureLast
    const T _start, _step, _stop; ///< defines linearly interpolated range from start to stop separated by step size.
    const bool _ensureLast; ///< if true, _stop is guaranteed to be returned by GetAndIncrement
};

template <typename T>
void TrajectoryBase::_SamplePointsInRange(std::vector<dReal>& data, RangeGenerator<T>& timeRange) const
{
    std::vector<dReal> tempdata;
    int dof = GetConfigurationSpecification().GetDOF();
    data.resize(dof*timeRange.GetSize());
    std::vector<dReal>::iterator itdata = data.begin();
    for(size_t i = 0; i < timeRange.GetSize(); ++i, itdata += dof) {
        const dReal time = timeRange.GetAndIncrement();
        Sample(tempdata, time);
        std::copy(tempdata.begin(), tempdata.end(), itdata);
    }
}

template <typename T>
void TrajectoryBase::_SamplePointsInRange(std::vector<dReal>& data, RangeGenerator<T>& timeRange, const ConfigurationSpecification& spec) const
{
    std::vector<dReal> tempdata;
    int dof = GetConfigurationSpecification().GetDOF();
    data.resize(dof*timeRange.GetSize());
    std::vector<dReal>::iterator itdata = data.begin();
    for(size_t i = 0; i < timeRange.GetSize(); ++i, itdata += dof) {
        const dReal time = timeRange.GetAndIncrement();
        Sample(tempdata, time, spec);
        std::copy(tempdata.begin(), tempdata.end(), itdata);
    }
}

void TrajectoryBase::SamplePointsSameDeltaTime(std::vector<dReal>& data, dReal deltatime, bool ensureLastPoint) const
{
    RangeGenerator<dReal> timeRange(deltatime, 0, GetDuration(), ensureLastPoint);
    return _SamplePointsInRange(data, timeRange);
}

void TrajectoryBase::SamplePointsSameDeltaTime(std::vector<dReal>& data, dReal deltatime, bool ensureLastPoint, const ConfigurationSpecification& spec) const
{
    RangeGenerator<dReal> timeRange(deltatime, 0, GetDuration(), ensureLastPoint);
    return _SamplePointsInRange(data, timeRange, spec);
}

void TrajectoryBase::SampleRangeSameDeltaTime(std::vector<dReal>& data, dReal deltatime, dReal startTime, dReal stopTime, bool ensureLastPoint) const
{
    RangeGenerator<dReal> timeRange(deltatime, startTime, stopTime, ensureLastPoint);
    return _SamplePointsInRange(data, timeRange);
}

void TrajectoryBase::SampleRangeSameDeltaTime(std::vector<dReal>& data, dReal deltatime, dReal startTime, dReal stopTime, bool ensureLastPoint, const ConfigurationSpecification& spec) const
{
    RangeGenerator<dReal> timeRange(deltatime, startTime, stopTime, ensureLastPoint);
    return _SamplePointsInRange(data, timeRange, spec);
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
