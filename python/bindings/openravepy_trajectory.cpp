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
#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

class PyTrajectoryBase : public PyInterfaceBase
{
protected:
    TrajectoryBasePtr _ptrajectory;
public:
    PyTrajectoryBase(TrajectoryBasePtr pTrajectory, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pTrajectory, pyenv),_ptrajectory(pTrajectory) {
    }
    virtual ~PyTrajectoryBase() {
    }

    void Init(PyConfigurationSpecificationPtr pyspec) {
        _ptrajectory->Init(openravepy::GetConfigurationSpecification(pyspec));
    }

    void Insert(size_t index, object odata)
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        _ptrajectory->Insert(index,vdata);
    }

    void Insert(size_t index, object odata, bool bOverwrite)
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        _ptrajectory->Insert(index,vdata,bOverwrite);
    }

    void Insert(size_t index, object odata, PyConfigurationSpecificationPtr pyspec)
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        _ptrajectory->Insert(index,vdata,openravepy::GetConfigurationSpecification(pyspec));
    }

    void Insert(size_t index, object odata, PyConfigurationSpecificationPtr pyspec, bool bOverwrite)
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        _ptrajectory->Insert(index,vdata,openravepy::GetConfigurationSpecification(pyspec),bOverwrite);
    }

    void Remove(size_t startindex, size_t endindex)
    {
        _ptrajectory->Remove(startindex,endindex);
    }

    object Sample(dReal time) const
    {
        vector<dReal> values;
        _ptrajectory->Sample(values,time);
        return toPyArray(values);
    }

    object Sample(dReal time, PyConfigurationSpecificationPtr pyspec) const
    {
        vector<dReal> values;
        _ptrajectory->Sample(values,time,openravepy::GetConfigurationSpecification(pyspec));
        return toPyArray(values);
    }

    object GetConfigurationSpecification() const {
        return object(openravepy::toPyConfigurationSpecification(_ptrajectory->GetConfigurationSpecification()));
    }

    size_t GetNumWaypoints() const {
        return _ptrajectory->GetNumWaypoints();
    }

    object GetWaypoints(size_t startindex, size_t endindex) const
    {
        vector<dReal> values;
        _ptrajectory->GetWaypoints(startindex,endindex,values);
        return toPyArray(values);
    }

    object GetWaypoints(size_t startindex, size_t endindex, PyConfigurationSpecificationPtr pyspec) const
    {
        vector<dReal> values;
        _ptrajectory->GetWaypoints(startindex,endindex,values,openravepy::GetConfigurationSpecification(pyspec));
        return toPyArray(values);
    }

    object GetWaypoint(int index) const
    {
        vector<dReal> values;
        _ptrajectory->GetWaypoint(index,values);
        return toPyArray(values);
    }

    object GetWaypoint(int index, PyConfigurationSpecificationPtr pyspec) const
    {
        vector<dReal> values;
        _ptrajectory->GetWaypoint(index,values,openravepy::GetConfigurationSpecification(pyspec));
        return toPyArray(values);
    }

    dReal GetDuration() const {
        return _ptrajectory->GetDuration();
    }


    void deserialize(const string& s)
    {
        std::stringstream ss(s);
        _ptrajectory->deserialize(ss);
    }

    object serialize(int options)
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _ptrajectory->serialize(ss,options);
        return object(ss.str());
    }

    bool Read(const string& s, object probot) {
        RAVELOG_WARN("Trajectory.Read deprecated please use Trajerctory.deserialize\n");
        deserialize(s);
        return true;
    }

    object Write(int options) {
        RAVELOG_WARN("Trajectory.Write deprecated please use Trajerctory.serialize\n");
        return serialize(options);
    }

    TrajectoryBasePtr GetTrajectory() {
        return _ptrajectory;
    }
};

namespace openravepy {

TrajectoryBasePtr GetTrajectory(PyTrajectoryBasePtr pytrajectory)
{
    return !pytrajectory ? TrajectoryBasePtr() : pytrajectory->GetTrajectory();
}

PyInterfaceBasePtr toPyTrajectory(TrajectoryBasePtr ptrajectory, PyEnvironmentBasePtr pyenv)
{
    return !ptrajectory ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyTrajectoryBase(ptrajectory,pyenv));
}

PyEnvironmentBasePtr toPyEnvironment(PyTrajectoryBasePtr pytraj)
{
    return pytraj->GetEnv();
}

PyTrajectoryBasePtr RaveCreateTrajectory(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    TrajectoryBasePtr p = OpenRAVE::RaveCreateTrajectory(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyTrajectoryBasePtr();
    }
    return PyTrajectoryBasePtr(new PyTrajectoryBase(p,pyenv));
}

void init_openravepy_trajectory()
{
    void (PyTrajectoryBase::*Insert1)(size_t,object) = &PyTrajectoryBase::Insert;
    void (PyTrajectoryBase::*Insert2)(size_t,object,bool) = &PyTrajectoryBase::Insert;
    void (PyTrajectoryBase::*Insert3)(size_t,object,PyConfigurationSpecificationPtr) = &PyTrajectoryBase::Insert;
    void (PyTrajectoryBase::*Insert4)(size_t,object,PyConfigurationSpecificationPtr,bool) = &PyTrajectoryBase::Insert;
    object (PyTrajectoryBase::*Sample1)(dReal) const = &PyTrajectoryBase::Sample;
    object (PyTrajectoryBase::*Sample2)(dReal, PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::Sample;
    object (PyTrajectoryBase::*GetWaypoints1)(size_t,size_t) const = &PyTrajectoryBase::GetWaypoints;
    object (PyTrajectoryBase::*GetWaypoints2)(size_t,size_t,PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::GetWaypoints;
    object (PyTrajectoryBase::*GetWaypoint1)(int) const = &PyTrajectoryBase::GetWaypoint;
    object (PyTrajectoryBase::*GetWaypoint2)(int,PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::GetWaypoint;
    class_<PyTrajectoryBase, boost::shared_ptr<PyTrajectoryBase>, bases<PyInterfaceBase> >("Trajectory", DOXY_CLASS(TrajectoryBase), no_init)
    .def("Init",&PyTrajectoryBase::Init,args("spec"),DOXY_FN(TrajectoryBase,Init))
    .def("Insert",Insert1,args("index","data"),DOXY_FN(TrajectoryBase,Init "size_t; const std::vector; bool"))
    .def("Insert",Insert2,args("index","data","overwrite"),DOXY_FN(TrajectoryBase,Init "size_t; const std::vector; bool"))
    .def("Insert",Insert3,args("index","data","spec"),DOXY_FN(TrajectoryBase,Init "size_t; const std::vector; const ConfigurationSpecification&; bool"))
    .def("Insert",Insert4,args("index","data","spec","overwrite"),DOXY_FN(TrajectoryBase,Init "size_t; const std::vector; const ConfigurationSpecification&; bool"))
    .def("Remove",&PyTrajectoryBase::Remove,args("startindex","endindex"),DOXY_FN(TrajectoryBase,Remove))
    .def("Sample",Sample1,args("time"),DOXY_FN(TrajectoryBase,Sample "std::vector; dReal"))
    .def("Sample",Sample2,args("time","spec"),DOXY_FN(TrajectoryBase,Sample "std::vector; dReal; spec"))
    .def("GetConfigurationSpecification",&PyTrajectoryBase::GetConfigurationSpecification,DOXY_FN(TrajectoryBase,GetConfigurationSpecification))
    .def("GetNumWaypoints",&PyTrajectoryBase::GetNumWaypoints,DOXY_FN(TrajectoryBase,GetNumWaypoints))
    .def("GetWaypoints",GetWaypoints1,args("startindex","endindex"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector"))
    .def("GetWaypoints",GetWaypoints2,args("startindex","endindex","spec"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector, const ConfigurationSpecification&"))
    .def("GetWaypoint",GetWaypoint1,args("index"),DOXY_FN(TrajectoryBase, GetWaypoint "size_t; std::vector"))
    .def("GetWaypoint",GetWaypoint2,args("index","spec"),DOXY_FN(TrajectoryBase, GetWaypoint "size_t; std::vector; const ConfigurationSpecification&"))
    .def("GetDuration",&PyTrajectoryBase::GetDuration,DOXY_FN(TrajectoryBase, GetDuration))
    .def("serialize",&PyTrajectoryBase::serialize,args("options"),DOXY_FN(TrajectoryBase,serialize))
    .def("deserialize",&PyTrajectoryBase::deserialize,args("data"),DOXY_FN(TrajectoryBase,deserialize))
    .def("Write",&PyTrajectoryBase::Write,args("options"),DOXY_FN(TrajectoryBase,Write))
    .def("Read",&PyTrajectoryBase::Read,args("data","robot"),DOXY_FN(TrajectoryBase,Read))
    ;

    def("RaveCreateTrajectory",openravepy::RaveCreateTrajectory,args("env","name"),DOXY_FN1(RaveCreateTrajectory));
}

}
