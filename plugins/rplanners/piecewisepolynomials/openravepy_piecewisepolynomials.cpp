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
#include "polynomialtrajectory.h"
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

namespace piecewisepolynomials = OpenRAVE::PiecewisePolynomialsInternal;
typedef OpenRAVE::dReal dReal;

namespace piecewisepolynomialspy {

typedef boost::shared_ptr<piecewisepolynomials::Coordinate> CoordinatePtr;
typedef boost::shared_ptr<piecewisepolynomials::Polynomial> PolynomialPtr;
typedef boost::shared_ptr<piecewisepolynomials::Chunk> ChunkPtr;
typedef boost::shared_ptr<piecewisepolynomials::PiecewisePolynomialTrajectory> PiecewisePolynomialTrajectoryPtr;

class PyCoordinate;
class PyPolynomial;
class PyChunk;
class PyPiecewisePolynomialTrajectory;
typedef boost::shared_ptr<PyCoordinate> PyCoordinatePtr;
typedef boost::shared_ptr<PyPolynomial> PyPolynomialPtr;
typedef boost::shared_ptr<PyChunk> PyChunkPtr;
typedef boost::shared_ptr<PyPiecewisePolynomialTrajectory> PyPiecewisePolynomialTrajectoryPtr;

class PyCoordinate {
public:
    PyCoordinate()
    {
        _pcoord.reset(new piecewisepolynomials::Coordinate());
        _PostProcess();
    }
    PyCoordinate(dReal p, dReal v)
    {
        _pcoord.reset(new piecewisepolynomials::Coordinate(p, v));
        _PostProcess();
    }

    std::string __repr__()
    {
        return boost::str(boost::format("Coordinate(%f, %f)")%point%value);
    }

    std::string __str__()
    {
        return boost::str(boost::format("Coordinate(%f, %f)")%point%value);
    }

    bool __eq__(PyCoordinatePtr c)
    {
        return !!c && (point == c->point) && (value == c->value);
    }

    bool __ne__(PyCoordinatePtr c)
    {
        return !!c || (point != c->point) || (value != c->value);
    }

    dReal point;
    dReal value;
private:
    void _PostProcess()
    {
        point = _pcoord->point;
        value = _pcoord->value;
    }

    CoordinatePtr _pcoord;
}; // end class PyCoordinate

class PyPolynomial {
public:
    PyPolynomial()
    {
        _ppolynomial.reset(new piecewisepolynomials::Polynomial());
    }
    PyPolynomial(const py::object ocoeffs)
    {
        std::vector<dReal> inputCoeffs = openravepy::ExtractArray<dReal>(ocoeffs);
        _ppolynomial.reset(new piecewisepolynomials::Polynomial(inputCoeffs));
        _PostProcess();
    }

    void Initialize(const py::object ocoeffs)
    {
        std::vector<dReal> inputCoeffs = openravepy::ExtractArray<dReal>(ocoeffs);
        _ppolynomial->Initialize(inputCoeffs);
        _PostProcess();
    }

    void PadCoefficients(size_t newdegree)
    {
        _ppolynomial->PadCoefficients(newdegree);
        _PostProcess();
    }

    void UpdateInitialValue(dReal c0)
    {
        _ppolynomial->UpdateInitialValue(c0);
        _PostProcess();
    }

    dReal Eval(dReal t) const
    {
        return _ppolynomial->Eval(t);
    }

    dReal Evald1(dReal t) const
    {
        return _ppolynomial->Evald1(t);
    }

    dReal Evald2(dReal t) const
    {
        return _ppolynomial->Evald2(t);
    }

    dReal Evald3(dReal t) const
    {
        return _ppolynomial->Evald3(t);
    }

    dReal Evaldn(dReal t, size_t n) const
    {
        return _ppolynomial->Evaldn(t, n);
    }

    py::list GetExtrema() const
    {
        py::list oExtrema;
        FOREACHC(itextremum, _ppolynomial->vcextrema) {
            oExtrema.append( PyCoordinate(itextremum->point, itextremum->value) );
        }
        return oExtrema;
    }

    py::list FindAllLocalExtrema(size_t ideriv) const
    {
        std::vector<piecewisepolynomials::Coordinate> vcoord;
        _ppolynomial->FindAllLocalExtrema(ideriv, vcoord);
        py::list oExtrema;
        FOREACHC(itextremum, vcoord) {
            oExtrema.append( PyCoordinate(itextremum->point, itextremum->value) );
        }
        return oExtrema;
    }

    std::string __repr__()
    {
        return _srep;
    }

    std::string __str__()
    {
        return _srep;
    }

private:
    void _PostProcess()
    {
        _srep.clear();
        _srep += "Polynomial([";
        FOREACHC(itcoeff, _ppolynomial->vcoeffs) {
            _srep += std::to_string(*itcoeff);
            _srep += ", ";
        }
        _srep += "])";
    }

    PolynomialPtr _ppolynomial;
    std::string _srep; // string representation of this polynomial
}; // end class PyPolynomial


} // end namespace piecewisepolynomialspy

OPENRAVE_PYTHON_MODULE(openravepy_piecewisepolynomials)
{
    using namespace piecewisepolynomialspy;

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

       from openravepy import openravepy_piecewisepolynomials as piecewisepolynomials

     */

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyCoordinate, PyCoordinatePtr>(m, "Coordinate", "PiecewisePolynomials::Coordinate wrapper")
    .def(init<>())
    .def(init<dReal, dReal>(), "point"_a, "value"_a)
#else
    class_<PyCoordinate, PyCoordinatePtr>("Coordinate", "PiecewisePolynomials::Coordinate wrapper", no_init)
    .def(init<>())
    .def(init<dReal, dReal>(py::args("point", "value")))
#endif
    .def_readonly("point", &PyCoordinate::point)
    .def_readonly("value", &PyCoordinate::value)
    .def("__repr__", &PyCoordinate::__repr__)
    .def("__str__", &PyCoordinate::__str__)
    .def("__eq__", &PyCoordinate::__eq__)
    .def("__ne__", &PyCoordinate::__ne__)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPolynomial, PyPolynomialPtr>(m, "Polynomial", "PiecewisePolynomials::Polynomial wrapper")
    .def(init<>())
    .def(init<py::object>(), "coeffs"_a)
#else
    class_<PyPolynomial, PyPolynomialPtr>("Polynomial", "PiecewisePolynomials::Polynomial wrapper", no_init)
    .def(init<>())
    .def(init<py::object>(py::args("coeffs")))
#endif
    .def("Eval", &PyPolynomial::Eval, PY_ARGS("t") "Evaluate the value of this polynomial at the given parameter t")
    .def("Evald1", &PyPolynomial::Evald1, PY_ARGS("t") "Evaluate the first derivative of this polynomial at the given parameter t")
    .def("Evald2", &PyPolynomial::Evald2, PY_ARGS("t") "Evaluate the second derivative of this polynomial at the given parameter t")
    .def("Evald3", &PyPolynomial::Evald3, PY_ARGS("t") "Evaluate the third derivative of this polynomial at the given parameter t")
    .def("Evaldn", &PyPolynomial::Evald3, PY_ARGS("t") "Evaluate the n-th derivative of this polynomial at the given parameter t")
    .def("GetExtrema", &PyPolynomial::GetExtrema, "Return the list of extrema of this polynomial")
    .def("FindAllLocalExtrema", &PyPolynomial::FindAllLocalExtrema, PY_ARGS("ideriv") "Return the list of extrema of the i-th derivative of this polynomial")
    .def("__repr__", &PyPolynomial::__repr__)
    .def("__str__", &PyPolynomial::__str__)
    ;
}
