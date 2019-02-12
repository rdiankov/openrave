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

#include <boost/python/slice.hpp> // slice objects

namespace openravepy {

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
        _ptrajectory->Sample(values,time,openravepy::GetConfigurationSpecification(pyspec), true);
        return toPyArray(values);
    }

    object SampleFromPrevious(object odata, dReal time, PyConfigurationSpecificationPtr pyspec) const
    {
        vector<dReal> vdata = ExtractArray<dReal>(odata);
        _ptrajectory->Sample(vdata,time,openravepy::GetConfigurationSpecification(pyspec), false);
        return toPyArray(vdata);
    }

    object SamplePoints2D(object otimes) const
    {
        vector<dReal> values;
        std::vector<dReal> vtimes = ExtractArray<dReal>(otimes);
        _ptrajectory->SamplePoints(values,vtimes);

        int numdof = _ptrajectory->GetConfigurationSpecification().GetDOF();
        npy_intp dims[] = { npy_intp(values.size()/numdof), npy_intp(numdof) };
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        if( values.size() > 0 ) {
            memcpy(PyArray_DATA(pypos), &values[0], values.size()*sizeof(values[0]));
        }
        return static_cast<numeric::array>(handle<>(pypos));
    }

    object SamplePoints2D(object otimes, PyConfigurationSpecificationPtr pyspec) const
    {
        vector<dReal> values;
        ConfigurationSpecification spec = openravepy::GetConfigurationSpecification(pyspec);
        std::vector<dReal> vtimes = ExtractArray<dReal>(otimes);
        _ptrajectory->SamplePoints(values, vtimes, spec);

        npy_intp dims[] = { npy_intp(values.size()/spec.GetDOF()), npy_intp(spec.GetDOF()) };
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        if( values.size() > 0 ) {
            memcpy(PyArray_DATA(pypos), &values[0], values.size()*sizeof(values[0]));
        }
        return static_cast<numeric::array>(handle<>(pypos));
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

    // similar to GetWaypoints except returns a 2D array, one row for every waypoint
    object GetWaypoints2D(size_t startindex, size_t endindex) const
    {
        vector<dReal> values;
        _ptrajectory->GetWaypoints(startindex,endindex,values);
        int numdof = _ptrajectory->GetConfigurationSpecification().GetDOF();
        npy_intp dims[] = { npy_intp(values.size()/numdof), npy_intp(numdof) };
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        if( values.size() > 0 ) {
            memcpy(PyArray_DATA(pypos), &values[0], values.size()*sizeof(values[0]));
        }
        return static_cast<numeric::array>(handle<>(pypos));
    }

    object __getitem__(int index) const
    {
        return GetWaypoint(index);
    }

    object __getitem__(slice indices) const
    {
        vector<int>vindices;
        int len = _ptrajectory->GetNumWaypoints();
        int step = !IS_PYTHONOBJECT_NONE(indices.step()) ? extract<int>(indices.step()) : 1;
        int start = !IS_PYTHONOBJECT_NONE(indices.start()) ? extract<int>(indices.start()) : step>0 ? 0 : len-1;
        int stop = !IS_PYTHONOBJECT_NONE(indices.stop()) ? extract<int>(indices.stop()) : step>0 ? len : -1;
        if(step==0) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("step cannot be 0"),ORE_InvalidArguments);
        }
        for(int i=start;step>0 ? i<stop : i>stop;i+=step) {
            vindices.push_back(i);
        }

        vector<dReal> values;
        _ptrajectory->GetWaypoint(0,values);
        int numdof = _ptrajectory->GetConfigurationSpecification().GetDOF();
        npy_intp dims[] = { npy_intp(vindices.size()), npy_intp(numdof) };
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        int waypointSize = values.size()*sizeof(values[0]);
        for(int i=0;i<vindices.size();i++) {
            _ptrajectory->GetWaypoint(vindices[i],values);
            memcpy(PyArray_BYTES(pypos)+(i*waypointSize), &values[0], waypointSize);
        }
        return static_cast<numeric::array>(handle<>(pypos));
    }

    object GetAllWaypoints2D() const
    {
        return GetWaypoints2D(0, _ptrajectory->GetNumWaypoints());
    }

    object GetWaypoints2D(size_t startindex, size_t endindex, PyConfigurationSpecificationPtr pyspec) const
    {
        vector<dReal> values;
        ConfigurationSpecification spec = openravepy::GetConfigurationSpecification(pyspec);
        _ptrajectory->GetWaypoints(startindex,endindex,values,spec);
        npy_intp dims[] = { npy_intp(values.size()/spec.GetDOF()), npy_intp(spec.GetDOF()) };
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        if( values.size() > 0 ) {
            memcpy(PyArray_DATA(pypos), &values[0], values.size()*sizeof(values[0]));
        }
        return static_cast<numeric::array>(handle<>(pypos));
    }

    object GetAllWaypoints2D(PyConfigurationSpecificationPtr pyspec) const
    {
        return GetWaypoints2D(0, _ptrajectory->GetNumWaypoints(), pyspec);
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

    size_t GetFirstWaypointIndexAfterTime(dReal time) const
    {
        return _ptrajectory->GetFirstWaypointIndexAfterTime(time);
    }

    dReal GetDuration() const {
        return _ptrajectory->GetDuration();
    }

    PyTrajectoryBasePtr deserialize(const string& s)
    {
        std::stringstream ss(s);
        InterfaceBasePtr p = _ptrajectory->deserialize(ss);
        return PyTrajectoryBasePtr(new PyTrajectoryBase(RaveInterfaceCast<TrajectoryBase>(p),_pyenv));
    }

    object serialize(object ooptions=object())
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _ptrajectory->serialize(ss,pyGetIntFromPy(ooptions,0));
        return object(ss.str());
    }

    bool Read(const string& s, object probot) {
        RAVELOG_WARN("Trajectory.Read deprecated please use Trajerctory.deserialize\n");
        deserialize(s);
        return true;
    }

    object Write(object options) {
        RAVELOG_WARN("Trajectory.Write deprecated please use Trajerctory.serialize\n");
        return serialize(options);
    }

    TrajectoryBasePtr GetTrajectory() {
        return _ptrajectory;
    }
};

TrajectoryBasePtr GetTrajectory(object o)
{
    extract<PyTrajectoryBasePtr> pytrajectory(o);
    if( pytrajectory.check() ) {
        return GetTrajectory((PyTrajectoryBasePtr)pytrajectory);
    }
    return TrajectoryBasePtr();
}

TrajectoryBasePtr GetTrajectory(PyTrajectoryBasePtr pytrajectory)
{
    return !pytrajectory ? TrajectoryBasePtr() : pytrajectory->GetTrajectory();
}

PyInterfaceBasePtr toPyTrajectory(TrajectoryBasePtr ptrajectory, PyEnvironmentBasePtr pyenv)
{
    return !ptrajectory ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyTrajectoryBase(ptrajectory,pyenv));
}

object toPyTrajectory(TrajectoryBasePtr ptraj, object opyenv)
{
    extract<PyEnvironmentBasePtr> pyenv(opyenv);
    if( pyenv.check() ) {
        return object(toPyTrajectory(ptraj,(PyEnvironmentBasePtr)pyenv));
    }
    return object();
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

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(serialize_overloads, serialize, 0, 1)

void init_openravepy_trajectory()
{
    void (PyTrajectoryBase::*Insert1)(size_t,object) = &PyTrajectoryBase::Insert;
    void (PyTrajectoryBase::*Insert2)(size_t,object,bool) = &PyTrajectoryBase::Insert;
    void (PyTrajectoryBase::*Insert3)(size_t,object,PyConfigurationSpecificationPtr) = &PyTrajectoryBase::Insert;
    void (PyTrajectoryBase::*Insert4)(size_t,object,PyConfigurationSpecificationPtr,bool) = &PyTrajectoryBase::Insert;
    object (PyTrajectoryBase::*Sample1)(dReal) const = &PyTrajectoryBase::Sample;
    object (PyTrajectoryBase::*Sample2)(dReal, PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::Sample;
    object (PyTrajectoryBase::*SamplePoints2D1)(object) const = &PyTrajectoryBase::SamplePoints2D;
    object (PyTrajectoryBase::*SamplePoints2D2)(object, PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::SamplePoints2D;
    object (PyTrajectoryBase::*GetWaypoints1)(size_t,size_t) const = &PyTrajectoryBase::GetWaypoints;
    object (PyTrajectoryBase::*GetWaypoints2)(size_t,size_t,PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::GetWaypoints;
    object (PyTrajectoryBase::*GetWaypoints2D1)(size_t,size_t) const = &PyTrajectoryBase::GetWaypoints2D;
    object (PyTrajectoryBase::*GetWaypoints2D2)(size_t,size_t,PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::GetWaypoints2D;
    object (PyTrajectoryBase::*GetAllWaypoints2D1)() const = &PyTrajectoryBase::GetAllWaypoints2D;
    object (PyTrajectoryBase::*GetAllWaypoints2D2)(PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::GetAllWaypoints2D;
    object (PyTrajectoryBase::*GetWaypoint1)(int) const = &PyTrajectoryBase::GetWaypoint;
    object (PyTrajectoryBase::*GetWaypoint2)(int,PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::GetWaypoint;
    object (PyTrajectoryBase::*__getitem__1)(int) const = &PyTrajectoryBase::__getitem__;
    object (PyTrajectoryBase::*__getitem__2)(slice) const = &PyTrajectoryBase::__getitem__;
    class_<PyTrajectoryBase, boost::shared_ptr<PyTrajectoryBase>, bases<PyInterfaceBase> >("Trajectory", DOXY_CLASS(TrajectoryBase), no_init)
    .def("Init",&PyTrajectoryBase::Init,args("spec"),DOXY_FN(TrajectoryBase,Init))
    .def("Insert",Insert1,args("index","data"),DOXY_FN(TrajectoryBase,Insert "size_t; const std::vector; bool"))
    .def("Insert",Insert2,args("index","data","overwrite"),DOXY_FN(TrajectoryBase,Insert "size_t; const std::vector; bool"))
    .def("Insert",Insert3,args("index","data","spec"),DOXY_FN(TrajectoryBase,Insert "size_t; const std::vector; const ConfigurationSpecification; bool"))
    .def("Insert",Insert4,args("index","data","spec","overwrite"),DOXY_FN(TrajectoryBase,Insert "size_t; const std::vector; const ConfigurationSpecification; bool"))
    .def("Remove",&PyTrajectoryBase::Remove,args("startindex","endindex"),DOXY_FN(TrajectoryBase,Remove))
    .def("Sample",Sample1,args("time"),DOXY_FN(TrajectoryBase,Sample "std::vector; dReal"))
    .def("Sample",Sample2,args("time","spec"),DOXY_FN(TrajectoryBase,Sample "std::vector; dReal; const ConfigurationSpecification"))
    .def("SampleFromPrevious",&PyTrajectoryBase::SampleFromPrevious,args("data","time","spec"),DOXY_FN(TrajectoryBase,Sample "std::vector; dReal; const ConfigurationSpecification"))
    .def("SamplePoints2D",SamplePoints2D1,args("times"),DOXY_FN(TrajectoryBase,SamplePoints2D "std::vector; std::vector"))
    .def("SamplePoints2D",SamplePoints2D2,args("times","spec"),DOXY_FN(TrajectoryBase,SamplePoints2D "std::vector; std::vector; const ConfigurationSpecification"))
    .def("GetConfigurationSpecification",&PyTrajectoryBase::GetConfigurationSpecification,DOXY_FN(TrajectoryBase,GetConfigurationSpecification))
    .def("GetNumWaypoints",&PyTrajectoryBase::GetNumWaypoints,DOXY_FN(TrajectoryBase,GetNumWaypoints))
    .def("GetWaypoints",GetWaypoints1,args("startindex","endindex"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector"))
    .def("GetWaypoints",GetWaypoints2,args("startindex","endindex","spec"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector, const ConfigurationSpecification&"))
    .def("GetWaypoints2D",GetWaypoints2D1,args("startindex","endindex"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector"))
    .def("GetWaypoints2D",GetWaypoints2D2,args("startindex","endindex","spec"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector, const ConfigurationSpecification&"))
    .def("GetAllWaypoints2D",GetAllWaypoints2D1,DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector"))
    .def("GetAllWaypoints2D",GetAllWaypoints2D2,args("spec"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector, const ConfigurationSpecification&"))
    .def("GetWaypoint",GetWaypoint1,args("index"),DOXY_FN(TrajectoryBase, GetWaypoint "int; std::vector"))
    .def("GetWaypoint",GetWaypoint2,args("index","spec"),DOXY_FN(TrajectoryBase, GetWaypoint "int; std::vector; const ConfigurationSpecification"))
    .def("GetFirstWaypointIndexAfterTime",&PyTrajectoryBase::GetFirstWaypointIndexAfterTime, DOXY_FN(TrajectoryBase, GetFirstWaypointIndexAfterTime))
    .def("GetDuration",&PyTrajectoryBase::GetDuration,DOXY_FN(TrajectoryBase, GetDuration))
    .def("serialize",&PyTrajectoryBase::serialize,serialize_overloads(args("options"),DOXY_FN(TrajectoryBase,serialize)))
    .def("deserialize",&PyTrajectoryBase::deserialize,args("data"),DOXY_FN(TrajectoryBase,deserialize))
    .def("Write",&PyTrajectoryBase::Write,args("options"),DOXY_FN(TrajectoryBase,Write))
    .def("Read",&PyTrajectoryBase::Read,args("data","robot"),DOXY_FN(TrajectoryBase,Read))
    .def("__len__",&PyTrajectoryBase::GetNumWaypoints,DOXY_FN(TrajectoryBase,__len__))
    .def("__getitem__",__getitem__1,args("index"),DOXY_FN(TrajectoryBase, __getitem__ "int"))
    .def("__getitem__",__getitem__2,args("indices"),DOXY_FN(TrajectoryBase, __getitem__ "slice"))
    ;

    def("RaveCreateTrajectory",openravepy::RaveCreateTrajectory,args("env","name"),DOXY_FN1(RaveCreateTrajectory));
}

}
