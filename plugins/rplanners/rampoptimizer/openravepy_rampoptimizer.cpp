// -*- Coding: utf-8 -*-
// Copyright (C) 2021 Puttichai Lertkultanon
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
#include "ramp.h"
#include <pyconfig.h>
#include <numpy/arrayobject.h>

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

using py::extract;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::init;
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

#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_module.h>

namespace rampoptimizer = OpenRAVE::RampOptimizerInternal;
typedef OpenRAVE::dReal dReal;

namespace rampoptimizerpy {

typedef boost::shared_ptr<rampoptimizer::Ramp> RampPtr;

class PyRamp {
public:
    PyRamp()
    {
        _pramp.reset(new rampoptimizer::Ramp());
    }

    PyRamp(dReal v0, dReal a, dReal duration, dReal x0)
    {
        _pramp.reset(new rampoptimizer::Ramp(v0, a, duration, x0));
    }

private:
    RampPtr _pramp;
};
typedef boost::shared_ptr<PyRamp> PyRampPtr;

} // end namespace rampoptimizerpy

OPENRAVE_PYTHON_MODULE(openravepy_rampoptimizer)
{
    using namespace rampoptimizerpy;

    // expansion of the macro `import_array()` in
    // numpy/core/include/numpy/__multiarray_api.h
    if (_import_array() < 0) {
        PyErr_Print();
        PyErr_SetString(PyExc_ImportError, "numpy.core.multiarray failed to import");
        return;
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyRamp, PyRampPtr>(m, "Ramp", "RampOptimizer::Ramp wrapper")
    .def(init<>())
    .def(init<dReal, dReal, dReal, dReal>(), PY_ARGS("v0", "a", "duration", "x0"))
#else
    class_<PyRamp, PyRampPtr>("Ramp", "RampOptimizer::Ramp wrapper", no_init)
    .def(init<>())
    .def(init<dReal, dReal, dReal, dReal>(py::args("v0", "a", "duration", "x0")))
#endif
    ;
}
