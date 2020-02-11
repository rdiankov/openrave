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

namespace configurationcachepy {

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
            return py::make_tuple(openravepy::toPyArray(nn.first), nn.second);
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
