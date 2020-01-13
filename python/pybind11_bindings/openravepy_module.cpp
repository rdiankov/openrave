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
#include <openravepy/openravepy_environmentbase.h>
#include <openravepy/openravepy_module.h>

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

PyModuleBase::PyModuleBase(ModuleBasePtr pmodule, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pmodule, pyenv), _pmodule(pmodule) {}
PyModuleBase::~PyModuleBase() {}
ModuleBasePtr PyModuleBase::GetModule() { return _pmodule; }
void PyModuleBase::Destroy() {
    _pmodule->Destroy();
}
bool PyModuleBase::SimulationStep(dReal fElapsedTime) {
    return _pmodule->SimulationStep(fElapsedTime);
}

ModuleBasePtr GetModule(PyModuleBasePtr pymodule)
{
    return !pymodule ? ModuleBasePtr() : pymodule->GetModule();
}

PyInterfaceBasePtr toPyModule(ModuleBasePtr pmodule, PyEnvironmentBasePtr pyenv)
{
    return !pmodule ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyModuleBase(pmodule,pyenv));
}

PyModuleBasePtr RaveCreateModule(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    ModuleBasePtr p = OpenRAVE::RaveCreateModule(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyModuleBasePtr();
    }
    return PyModuleBasePtr(new PyModuleBase(p,pyenv));
}

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_module(py::module& m)
#else
void init_openravepy_module()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals; // "..."_a
    class_<PyModuleBase, OPENRAVE_SHARED_PTR<PyModuleBase>, PyInterfaceBase>(m, "Module", DOXY_CLASS(ModuleBase))
    .def(init<ModuleBasePtr, PyEnvironmentBasePtr>(), "module"_a, "env"_a)
#else
    class_<PyModuleBase, OPENRAVE_SHARED_PTR<PyModuleBase>, bases<PyInterfaceBase> >("Module", DOXY_CLASS(ModuleBase), no_init)
#endif
    .def("SimulationStep",&PyModuleBase::SimulationStep, DOXY_FN(ModuleBase,"SimulationStep"))
    .def("Destroy",&PyModuleBase::Destroy, DOXY_FN(ModuleBase,"Destroy"))
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveCreateModule",openravepy::RaveCreateModule, PY_ARGS("env","name") DOXY_FN1(RaveCreateModule));
    m.def("RaveCreateProblem",openravepy::RaveCreateModule, PY_ARGS("env","name") DOXY_FN1(RaveCreateModule));
    m.def("RaveCreateProblemInstance",openravepy::RaveCreateModule, PY_ARGS("env","name") DOXY_FN1(RaveCreateModule));
#else
    def("RaveCreateModule",openravepy::RaveCreateModule, PY_ARGS("env","name") DOXY_FN1(RaveCreateModule));
    def("RaveCreateProblem",openravepy::RaveCreateModule, PY_ARGS("env","name") DOXY_FN1(RaveCreateModule));
    def("RaveCreateProblemInstance",openravepy::RaveCreateModule, PY_ARGS("env","name") DOXY_FN1(RaveCreateModule));
#endif
}

}
