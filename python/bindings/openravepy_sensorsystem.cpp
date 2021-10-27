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
#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_environmentbase.h>

namespace openravepy {

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

class PySensorSystemBase : public PyInterfaceBase
{
    friend class PyEnvironmentBase;
private:
    SensorSystemBasePtr _psensorsystem;
public:
    PySensorSystemBase(SensorSystemBasePtr psensorsystem, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(psensorsystem, pyenv), _psensorsystem(psensorsystem) {
    }
    virtual ~PySensorSystemBase() {
    }

    SensorSystemBasePtr GetSensorSystem() {
        return _psensorsystem;
    }
};

SensorSystemBasePtr GetSensorSystem(PySensorSystemBasePtr pySensorSystem)
{
    return !pySensorSystem ? SensorSystemBasePtr() : pySensorSystem->GetSensorSystem();
}

PyInterfaceBasePtr toPySensorSystem(SensorSystemBasePtr pSensorSystem, PyEnvironmentBasePtr pyenv)
{
    return !pSensorSystem ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PySensorSystemBase(pSensorSystem,pyenv));
}

PySensorSystemBasePtr RaveCreateSensorSystem(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    SensorSystemBasePtr p = OpenRAVE::RaveCreateSensorSystem(GetEnvironment(pyenv), name);
    if( !p ) {
        return PySensorSystemBasePtr();
    }
    return PySensorSystemBasePtr(new PySensorSystemBase(p,pyenv));
}

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_sensorsystem(py::module& m)
#else
void init_openravepy_sensorsystem()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals; // "..."_a
    class_<PySensorSystemBase, OPENRAVE_SHARED_PTR<PySensorSystemBase>, PyInterfaceBase>(m, "SensorSystem", DOXY_CLASS(SensorSystemBase))
    .def(init<SensorSystemBasePtr, PyEnvironmentBasePtr>(), "sensorsystem"_a, "env"_a)
#else
    class_<PySensorSystemBase, OPENRAVE_SHARED_PTR<PySensorSystemBase>, bases<PyInterfaceBase> >("SensorSystem", DOXY_CLASS(SensorSystemBase), no_init)
#endif
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveCreateSensorSystem", openravepy::RaveCreateSensorSystem, PY_ARGS("env","name") DOXY_FN1(RaveCreateSensorSystem));
#else
    def("RaveCreateSensorSystem",openravepy::RaveCreateSensorSystem, PY_ARGS("env","name") DOXY_FN1(RaveCreateSensorSystem));
#endif
}

}
