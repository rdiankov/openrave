// -*- Coding: utf-8 -*-
// Copyright (C) 2014 Rosen Diankov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/// \author Rosen Diankov
#include "configurationcachetree.h"
#include <pyconfig.h>
#include <numpy/arrayobject.h>
#include <openrave/xmlreaders.h>

#ifdef USE_PYBIND11_PYTHON_BINDINGS
#ifdef _
#undef _
#endif // _
#define OPENRAVE_BININGS_PYARRAY
#include <pybind11/pybind11.h>
namespace py = pybind11;
#else // USE_PYBIND11_PYTHON_BINDINGS
#define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle
#define BOOST_PYTHON_MAX_ARITY 16
#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/docstring_options.hpp>
namespace py = boost::python;
#endif // USE_PYBIND11_PYTHON_BINDINGS

#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_module.h>
using py::object;
using py::extract;
using py::extract_;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::init;
using py::scope_; // py::object if USE_PYBIND11_PYTHON_BINDINGS
using py::scope;
using py::args;
using py::return_value_policy;

#ifndef USE_PYBIND11_PYTHON_BINDINGS
using py::no_init;
using py::bases;
using py::copy_const_reference;
using py::docstring_options;
using py::pickle_suite;
using py::manage_new_object;
using py::def;
#endif // USE_PYBIND11_PYTHON_BINDINGS
namespace numeric = py::numeric;

using openravepy::ConvertStringToUnicode;

using namespace OpenRAVE;

namespace py = boost::python;
using py::object;
using py::extract;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::no_init;
using py::bases;
using py::init;
using py::scope;
using py::args;
using py::return_value_policy;
using py::copy_const_reference;
using py::docstring_options;
using py::pickle_suite;
using py::manage_new_object;
using py::def;
namespace numeric = py::numeric;

// declared from openravepy_int
namespace openravepy {
Transform ExtractTransform(const object& oraw);
TransformMatrix ExtractTransformMatrix(const object& oraw);
object toPyArray(const TransformMatrix& t);
object toPyArray(const Transform& t);

XMLReadablePtr ExtractXMLReadable(object o);
object toPyXMLReadable(XMLReadablePtr p);
bool ExtractIkParameterization(object o, IkParameterization& ikparam);
object toPyIkParameterization(const IkParameterization& ikparam);
object toPyIkParameterization(const std::string& serializeddata);

object toPyPlannerParameters(PlannerBase::PlannerParametersPtr params);

object toPyEnvironment(object);
object toPyKinBody(KinBodyPtr, object opyenv);
object toPyKinBodyLink(KinBody::LinkPtr plink, object opyenv);

//EnvironmentBasePtr GetEnvironment(py::object);
TrajectoryBasePtr GetTrajectory(object);
KinBodyPtr GetKinBody(object);
KinBody::LinkPtr GetKinBodyLink(object);
RobotBasePtr GetRobot(object o);
RobotBase::ManipulatorPtr GetRobotManipulator(object);
PlannerBase::PlannerParametersConstPtr GetPlannerParametersConst(object);
CollisionReportPtr GetCollisionReport(object);

EnvironmentBasePtr GetEnvironment(object o);
} // namespace openravepy

namespace configurationcachepy {

inline std::string _ExtractStringSafe(py::object ostring)
{
    if( ostring == object() ) {
        return std::string();
    }
    else {
        return extract<std::string>(ostring);
    }
}

inline py::object ConvertStringToUnicode(const std::string& s)
{
    return py::object(py::handle<>(PyUnicode_Decode(s.c_str(),s.size(), "utf-8", NULL)));
}

template <typename T>
inline RaveVector<T> ExtractVector3Type(const object& o)
{
    return RaveVector<T>(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]));
}

template <typename T>
inline std::vector<T> ExtractArray(const object& o)
{
    std::vector<T> v(len(o));
    for(size_t i = 0; i < v.size(); ++i) {
        v[i] = extract<T>(o[i]);
    }
    return v;
}

inline numeric::array toPyArrayN(const float* pvalues, size_t N)
{
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("f4"));
    }
    npy_intp dims[] = {npy_intp(N)};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, PyArray_FLOAT);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(float));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const float* pvalues, std::vector<npy_intp>& dims)
{
    if( dims.size() == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("f4"));
    }
    size_t totalsize = 1;
    FOREACH(it,dims) {
        totalsize *= *it;
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("f4"));
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(),&dims[0], PyArray_FLOAT);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,totalsize*sizeof(float));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const double* pvalues, size_t N)
{
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("f8"));
    }
    npy_intp dims[] = {npy_intp(N)};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, PyArray_DOUBLE);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(double));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const double* pvalues, std::vector<npy_intp>& dims)
{
    if( dims.size() == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("f8"));
    }
    size_t totalsize = 1;
    FOREACH(it,dims) {
        totalsize *= *it;
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("f8"));
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(),&dims[0], PyArray_DOUBLE);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,totalsize*sizeof(double));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const uint8_t* pvalues, std::vector<npy_intp>& dims)
{
    if( dims.size() == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("u1"));
    }
    size_t totalsize = 1;
    for(size_t i = 0; i < dims.size(); ++i) {
        totalsize *= dims[i];
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("u1"));
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(),&dims[0], PyArray_UINT8);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,totalsize*sizeof(uint8_t));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const uint8_t* pvalues, size_t N)
{
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("u1"));
    }
    npy_intp dims[] = {npy_intp(N)};
    PyObject *pyvalues = PyArray_SimpleNew(1,&dims[0], PyArray_UINT8);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(uint8_t));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const int* pvalues, size_t N)
{
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("i4"));
    }
    npy_intp dims[] = {npy_intp(N)};
    PyObject *pyvalues = PyArray_SimpleNew(1,&dims[0], PyArray_INT32);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(int));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const uint32_t* pvalues, size_t N)
{
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(py::list()).astype("u4"));
    }
    npy_intp dims[] = {npy_intp(N)};
    PyObject *pyvalues = PyArray_SimpleNew(1,&dims[0], PyArray_UINT32);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(uint32_t));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

template <typename T>
inline numeric::array toPyArray(const std::vector<T>& v)
{
    if( v.size() == 0 ) {
        return toPyArrayN((T*)NULL,0);
    }
    return toPyArrayN(&v[0],v.size());
}

class PyConfigurationCache
{
public:
    PyConfigurationCache(object pyrobot)
    {
        _pyenv = openravepy::toPyEnvironment(pyrobot);
        _cache.reset(new configurationcache::ConfigurationCache(openravepy::GetRobot(pyrobot)));
    }
    virtual ~PyConfigurationCache(){
    }

    int InsertConfigurationDist(object ovalues, object pyreport, dReal dist)
    {
        return _cache->InsertConfiguration(openravepy::ExtractArray<dReal>(ovalues), openravepy::GetCollisionReport(pyreport), dist);
    }

    int InsertConfiguration(object ovalues, object pyreport)
    {
        return _cache->InsertConfiguration(openravepy::ExtractArray<dReal>(ovalues), openravepy::GetCollisionReport(pyreport));
    }

    object CheckCollision(object ovalues)
    {
        KinBody::LinkConstPtr crobotlink, ccollidinglink;
        dReal closestdist=0;
        int ret = _cache->CheckCollision(openravepy::ExtractArray<dReal>(ovalues), crobotlink, ccollidinglink, closestdist);
        KinBody::LinkPtr robotlink, collidinglink;
        if( !!crobotlink ) {
            robotlink = crobotlink->GetParent()->GetLinks().at(crobotlink->GetIndex());
        }
        if( !!ccollidinglink ) {
            collidinglink = ccollidinglink->GetParent()->GetLinks().at(ccollidinglink->GetIndex());
        }
        return py::make_tuple(ret, closestdist, py::make_tuple(openravepy::toPyKinBodyLink(robotlink, _pyenv), openravepy::toPyKinBodyLink(collidinglink, _pyenv)));
    }

    void Reset()
    {
        _cache->Reset();
    }

    object GetDOFValues()
    {
        std::vector<dReal> values;
        _cache->GetDOFValues(values);
        return openravepy::toPyArray(values);
    }

    int GetNumNodes() {
        return _cache->GetNumNodes();
    }

    void SetCollisionThresh(dReal colthresh)
    {
        _cache->SetCollisionThresh(colthresh);
    }

    void SetFreeSpaceThresh(dReal freespacethresh)
    {
        _cache->SetFreeSpaceThresh(freespacethresh);
    }

    void SetWeights(object oweights)
    {
        _cache->SetWeights(openravepy::ExtractArray<dReal>(oweights));
    }

    void SetInsertionDistanceMult(dReal indist)
    {
        _cache->SetInsertionDistanceMult(indist);
    }

    dReal GetCollisionThresh()
    {
        return _cache->GetCollisionThresh();
    }

    dReal GetFreeSpaceThresh()
    {
        return _cache->GetFreeSpaceThresh();
    }


    dReal GetInsertionDistanceMult()
    {
        return _cache->GetInsertionDistanceMult();
    }

    object GetRobot() {
        return openravepy::toPyKinBody(_cache->GetRobot(), _pyenv);
    }

    bool Validate() {
        return _cache->Validate();
    }

    object GetNodeValues() {
        std::vector<dReal> values;
        _cache->GetNodeValues(values);
        return openravepy::toPyArray(values);
    }

    object FindNearestNode(object ovalues, dReal dist) {
        std::pair<std::vector<dReal>, dReal> nn = _cache->FindNearestNode(openravepy::ExtractArray<dReal>(ovalues), dist);
        if( nn.first.empty() ) {
            return py::none_(); // didn't find anything
        }
        else {
            return py::make_tuple(toPyArray(nn.first), nn.second);
        }
    }

    dReal ComputeDistance(object oconfi, object oconff) {
        return _cache->ComputeDistance(openravepy::ExtractArray<dReal>(oconfi), openravepy::ExtractArray<dReal>(oconff));
    }

protected:
    object _pyenv;
    configurationcache::ConfigurationCachePtr _cache;
};

typedef OPENRAVE_SHARED_PTR<PyConfigurationCache> PyConfigurationCachePtr;

} // end namespace configurationcachepy

OPENRAVE_PYTHON_MODULE(openravepy_configurationcache)
{
    using namespace configurationcachepy;
    import_array();
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.attr("__doc__") = "The module contains configuration cache bindings for openravepy\n";
#else
    scope().attr("__doc__") = "The module contains configuration cache bindings for openravepy\n";
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals; // "..."_a
    class_<PyConfigurationCache, PyConfigurationCachePtr >(m, "ConfigurationCache")
    .def(init<object>(), "robot"_a)
#else
    class_<PyConfigurationCache, PyConfigurationCachePtr >("ConfigurationCache", no_init)
    .def(init<object>(args("robot")))
#endif // USE_PYBIND11_PYTHON_BINDINGS
    .def("InsertConfigurationDist",&PyConfigurationCache::InsertConfigurationDist, PY_ARGS("values","report","dist") "Doc of InsertConfigurationDist")
    .def("InsertConfiguration",&PyConfigurationCache::InsertConfiguration, PY_ARGS("values", "report") "Doc of InsertConfiguration")
    .def("CheckCollision",&PyConfigurationCache::CheckCollision, PY_ARGS("values") "Doc of CheckCollision")
    .def("Reset",&PyConfigurationCache::Reset)
    .def("GetDOFValues",&PyConfigurationCache::GetDOFValues)
    .def("GetNumNodes",&PyConfigurationCache::GetNumNodes)
    .def("SetCollisionThresh",&PyConfigurationCache::SetCollisionThresh, PY_ARGS("colthresh") "Doc of SetCollisionThresh")
    .def("SetFreeSpaceThresh",&PyConfigurationCache::SetFreeSpaceThresh, PY_ARGS("freespacethresh") "Doc of SetFreeSpaceThresh")
    .def("SetWeights",&PyConfigurationCache::SetWeights, PY_ARGS("weights") "Doc of SetWeights")
    .def("SetInsertionDistanceMult",&PyConfigurationCache::SetInsertionDistanceMult, PY_ARGS("indist") "Doc of SetInsertionDistanceMult")
    .def("GetRobot",&PyConfigurationCache::GetRobot)
    .def("GetNumNodes",&PyConfigurationCache::GetNumNodes)
    .def("Validate", &PyConfigurationCache::Validate)
    .def("GetNodeValues", &PyConfigurationCache::GetNodeValues)
    .def("FindNearestNode", &PyConfigurationCache::FindNearestNode)
    .def("ComputeDistance", &PyConfigurationCache::ComputeDistance)

    .def("GetCollisionThresh", &PyConfigurationCache::GetCollisionThresh)
    .def("GetFreeSpaceThresh", &PyConfigurationCache::GetFreeSpaceThresh)
    .def("GetInsertionDistanceMult", &PyConfigurationCache::GetInsertionDistanceMult)
    ;
}
