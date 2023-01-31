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

#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_module.h>
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
using OpenRAVE::dReal;

namespace rampoptimizerpy {

typedef boost::shared_ptr<rampoptimizer::Ramp> RampPtr;

class PyRamp;
typedef boost::shared_ptr<PyRamp> PyRampPtr;

#pragma GCC diagnostic ignored "-Wshadow"   // parameters seems to intentionally overlap w/ class members, no reason to fix that

class PyRamp {
public:
    PyRamp()
    {
        _pramp.reset(new rampoptimizer::Ramp());
        _PostProcess();
    }

    PyRamp(dReal v0, dReal a, dReal duration, dReal x0)
    {
        _pramp.reset(new rampoptimizer::Ramp(v0, a, duration, x0));
        _PostProcess();
    }

    dReal EvalPos(dReal t) const
    {
        return _pramp->EvalPos(t);
    }

    dReal EvalVel(dReal t) const
    {
        return _pramp->EvalVel(t);
    }

    dReal EvalAcc() const
    {
        return _pramp->EvalAcc();
    }

    py::object GetPeaks() const
    {
        dReal bmin, bmax;
        _pramp->GetPeaks(bmin, bmax);
        return py::make_tuple(bmin, bmax);
    }

    py::object GetPeaks(const dReal ta, const dReal tb) const
    {
        dReal bmin, bmax;
        _pramp->GetPeaks(ta, tb, bmin, bmax);
        return py::make_tuple(bmin, bmax);
    }

    void Initialize(dReal v0, dReal a, dReal duration, dReal x0)
    {
        _pramp->Initialize(v0, a, duration, x0);
        _PostProcess();
    }

    void SetInitialValue(dReal newx0)
    {
        _pramp->SetInitialValue(newx0);
        _PostProcess();
    }

    void UpdateDuration(dReal newDuration)
    {
        _pramp->UpdateDuration(newDuration);
        _PostProcess();
    }

    void Copy(PyRampPtr inputRamp)
    {
        _pramp->Copy(*(inputRamp->_pramp));
        _PostProcess();
    }

    void Cut(dReal t, PyRampPtr remRamp)
    {
        _pramp->Cut(t, *(remRamp->_pramp));
        _PostProcess();
    }

    void TrimFront(dReal t)
    {
        _pramp->TrimFront(t);
        _PostProcess();
    }

    void TrimBack(dReal t)
    {
        _pramp->TrimBack(t);
        _PostProcess();
    }

    dReal v0;
    dReal a;
    dReal duration;
    dReal x0;
    dReal x1;
    dReal d;
    dReal v1;
private:
    void _PostProcess()
    {
        v0 = _pramp->v0;
        a = _pramp->a;
        duration = _pramp->duration;
        x0 = _pramp->x0;
        x1 = _pramp->x1;
        d = _pramp->d;
        v1 = _pramp->v1;
    }

    RampPtr _pramp;
};

} // end namespace rampoptimizerpy

OPENRAVE_PYTHON_MODULE(openravepy_rampoptimizer)
{
    using namespace rampoptimizerpy;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace pybind11::literals;
#endif

    // expansion of the macro `import_array()` in
    // numpy/core/include/numpy/__multiarray_api.h
    if (_import_array() < 0) {
        PyErr_Print();
        PyErr_SetString(PyExc_ImportError, "numpy.core.multiarray failed to import");
        return;
    }

    /* Usage from python

      from openravepy import openravepy_rampoptimizer as rampoptimizer

     */

    py::object (PyRamp::*getpeaks1)() const = &PyRamp::GetPeaks;
    py::object (PyRamp::*getpeaks2)(const dReal, const dReal) const = &PyRamp::GetPeaks;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyRamp, PyRampPtr>(m, "Ramp", "RampOptimizer::Ramp wrapper")
    .def(init<>())
    .def(init<dReal, dReal, dReal, dReal>(), "v0"_a, "a"_a, "duration"_a, "x0"_a)
#else
    class_<PyRamp, PyRampPtr>("Ramp", "RampOptimizer::Ramp wrapper", no_init)
    .def(init<>())
    .def(init<dReal, dReal, dReal, dReal>(py::args("v0", "a", "duration", "x0")))
#endif
    .def_readonly("v0", &PyRamp::v0)
    .def_readonly("a", &PyRamp::a)
    .def_readonly("duration", &PyRamp::duration)
    .def_readonly("x0", &PyRamp::x0)
    .def_readonly("v1", &PyRamp::v1)
    .def_readonly("d", &PyRamp::d)
    .def_readonly("x1", &PyRamp::x1)
    .def("EvalPos", &PyRamp::EvalPos, PY_ARGS("t") "Evaluate position of this ramp at the given time t")
    .def("EvalVel", &PyRamp::EvalVel, PY_ARGS("t") "Evaluate velocity of this ramp at the given time t")
    .def("EvalAcc", &PyRamp::EvalAcc, "Evaluate acceleration of this ramp")
    .def("GetPeaks", getpeaks1, "")
    .def("GetPeaks", getpeaks2, PY_ARGS("ta", "tb") "")
    .def("Initialize", &PyRamp::Initialize, PY_ARGS("v0", "a", "duration", "x0") "")
    .def("SetInitialValue", &PyRamp::SetInitialValue, PY_ARGS("newx0") "")
    .def("UpdateDuration", &PyRamp::UpdateDuration, PY_ARGS("newDuration") "")
    .def("Copy", &PyRamp::Copy, PY_ARGS("outRamp") "")
    .def("Cut", &PyRamp::Cut, PY_ARGS("t", "remRamp") "")
    .def("TrimFront", &PyRamp::TrimFront, PY_ARGS("t") "")
    .def("TrimBack", &PyRamp::TrimBack, PY_ARGS("t") "")
    ;
}
