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

private:
    void _PostProcess()
    {
        point = _pcoord->point;
        value = _pcoord->value;
    }

public:
    dReal point;
    dReal value;
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
    PyPolynomial(const std::vector<dReal>& coeffs)
    {
        _ppolynomial.reset(new piecewisepolynomials::Polynomial(coeffs));
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

    py::object Serialize() const
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        _ppolynomial->Serialize(ss);
        return py::to_object(ss.str());
    }

    void Deserialize(const std::string s)
    {
        std::stringstream ss(s);
        _ppolynomial->Deserialize(ss);
        _PostProcess();
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

public:
    PolynomialPtr _ppolynomial;
    std::string _srep; // string representation of this polynomial
}; // end class PyPolynomial

std::vector<piecewisepolynomials::Polynomial> ExtractArrayPolynomials(const py::object ov)
{
    size_t numPolynomials = len(ov);
    std::vector<piecewisepolynomials::Polynomial> v;
    v.reserve(numPolynomials);
    for(size_t ipoly = 0; ipoly < numPolynomials; ++ipoly ) {
        PyPolynomialPtr ppypoly = py::extract<PyPolynomialPtr>(ov[ipoly]);
        if( !!ppypoly ) {
            v.push_back(*(ppypoly->_ppolynomial));
        }
        else {
            RAVELOG_ERROR_FORMAT("failed to get polynomial at index=%d", ipoly);
        }
    }
    return v;
}

class PyChunk {
public:
    PyChunk()
    {
        _pchunk.reset(new piecewisepolynomials::Chunk());
        _PostProcess();
    }
    PyChunk(const dReal duration, const py::object ovpolynomials)
    {
        std::vector<piecewisepolynomials::Polynomial> vpolynomials = ExtractArrayPolynomials(ovpolynomials);
        _pchunk.reset(new piecewisepolynomials::Chunk(duration, vpolynomials));
        _PostProcess();
    }
    PyChunk(const dReal duration, const std::vector<piecewisepolynomials::Polynomial>& vpolynomials)
    {
        _pchunk.reset(new piecewisepolynomials::Chunk(duration, vpolynomials));
        _PostProcess();
    }

    void Initialize(const dReal duration, const py::object ovpolynomials)
    {
        std::vector<piecewisepolynomials::Polynomial> vpolynomials = ExtractArrayPolynomials(ovpolynomials);
        _pchunk->Initialize(duration, vpolynomials);
        _PostProcess();
    }

    void UpdateInitialValues(const py::object ovinitialvalues)
    {
        std::vector<dReal> vinitialvalues = openravepy::ExtractArray<dReal>(ovinitialvalues);
        _pchunk->UpdateInitialValues(vinitialvalues);
        _PostProcess();
    }

    PyChunkPtr Cut(dReal t)
    {
        piecewisepolynomials::Chunk remChunk;
        _pchunk->Cut(t, remChunk);
        PyChunkPtr pyRemChunk(new PyChunk());
        pyRemChunk->_pchunk->Initialize(remChunk.duration, remChunk.vpolynomials);
        return pyRemChunk;
    }

    py::object Eval(dReal t) const
    {
        std::vector<dReal> res;
        _pchunk->Eval(t, res);
        return openravepy::toPyArray(res);
    }

    py::object Evald1(dReal t) const
    {
        std::vector<dReal> res;
        _pchunk->Evald1(t, res);
        return openravepy::toPyArray(res);
    }

    py::object Evald2(dReal t) const
    {
        std::vector<dReal> res;
        _pchunk->Evald2(t, res);
        return openravepy::toPyArray(res);
    }

    py::object Evald3(dReal t) const
    {
        std::vector<dReal> res;
        _pchunk->Evald3(t, res);
        return openravepy::toPyArray(res);
    }

    py::object Evaldn(dReal t, size_t n) const
    {
        std::vector<dReal> res;
        _pchunk->Evaldn(t, n, res);
        return openravepy::toPyArray(res);
    }

    void SetConstant(const py::object ox0Vect, const dReal duration, const size_t degree)
    {
        std::vector<dReal> x0Vect = openravepy::ExtractArray<dReal>(ox0Vect);
        _pchunk->SetConstant(x0Vect, duration, degree);
        _PostProcess();
    }

    py::object Serialize() const
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        _pchunk->Serialize(ss);
        return py::to_object(ss.str());
    }

    void Deserialize(const std::string s)
    {
        std::stringstream ss(s);
        _pchunk->Deserialize(ss);
        _PostProcess();
    }

    py::object GetPolynomials() const
    {
        py::list pypolynomials;
        for( size_t ipoly = 0; ipoly < dof; ++ipoly ) {
            pypolynomials.append(PyPolynomialPtr(new PyPolynomial(_pchunk->vpolynomials[ipoly].vcoeffs)));
        }
        return pypolynomials;
    }

    PyPolynomialPtr GetPolynomial(int index) const
    {
        return PyPolynomialPtr(new PyPolynomial(_pchunk->vpolynomials[index].vcoeffs));
    }

private:
    void _PostProcess()
    {
        degree = _pchunk->degree;
        dof = _pchunk->dof;
        duration = _pchunk->duration;
    }

public:
    ChunkPtr _pchunk;

    size_t degree;
    size_t dof;
    dReal duration;
}; // end class PyChunk

std::vector<piecewisepolynomials::Chunk> ExtractArrayChunks(const py::object ov)
{
    size_t numChunks = len(ov);
    std::vector<piecewisepolynomials::Chunk> v;
    v.reserve(numChunks);
    for(size_t ichunk = 0; ichunk < numChunks; ++ichunk ) {
        PyChunkPtr ppychunk = py::extract<PyChunkPtr>(ov[ichunk]);
        if( !!ppychunk ) {
            v.push_back(*(ppychunk->_pchunk));
        }
        else {
            RAVELOG_ERROR_FORMAT("failed to get chunk at index=%d", ichunk);
        }
    }
    return v;
}

class PyPiecewisePolynomialTrajectory {
public:
    PyPiecewisePolynomialTrajectory()
    {
        _ptraj.reset(new piecewisepolynomials::PiecewisePolynomialTrajectory());
        _PostProcess();
    }
    PyPiecewisePolynomialTrajectory(const py::object ovchunks)
    {
        std::vector<piecewisepolynomials::Chunk> vchunks = ExtractArrayChunks(ovchunks);
        _ptraj.reset(new piecewisepolynomials::PiecewisePolynomialTrajectory(vchunks));
        _PostProcess();
    }

    void Initialize(const py::object ovchunks)
    {
        std::vector<piecewisepolynomials::Chunk> vchunks = ExtractArrayChunks(ovchunks);
        _ptraj->Initialize(vchunks);
        _PostProcess();
    }

    py::object Eval(dReal t) const
    {
        std::vector<dReal> res;
        _ptraj->Eval(t, res);
        return openravepy::toPyArray(res);
    }

    py::object Evald1(dReal t) const
    {
        std::vector<dReal> res;
        _ptraj->Evald1(t, res);
        return openravepy::toPyArray(res);
    }

    py::object Evald2(dReal t) const
    {
        std::vector<dReal> res;
        _ptraj->Evald2(t, res);
        return openravepy::toPyArray(res);
    }

    py::object Evald3(dReal t) const
    {
        std::vector<dReal> res;
        _ptraj->Evald3(t, res);
        return openravepy::toPyArray(res);
    }

    py::object Evaldn(dReal t, size_t n) const
    {
        std::vector<dReal> res;
        _ptraj->Evaldn(t, n, res);
        return openravepy::toPyArray(res);
    }

    py::object FindChunkIndex(dReal t) const
    {
        size_t index;
        dReal remainder;
        _ptraj->FindChunkIndex(t, index, remainder);
        return py::make_tuple(index, remainder);
    }

    py::object Serialize() const
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        _ptraj->Serialize(ss);
        return py::to_object(ss.str());
    }

    void Deserialize(std::string s)
    {
        std::stringstream ss(s);
        _ptraj->Deserialize(ss);
        _PostProcess();
    }

    void ReplaceSegment(dReal t0, dReal t1, const py::object ovchunks)
    {
        std::vector<piecewisepolynomials::Chunk> vchunks = ExtractArrayChunks(ovchunks);
        _ptraj->ReplaceSegment(t0, t1, vchunks);
        _PostProcess();
    }

    void Reset()
    {
        _ptraj->Reset();
    }

    py::object GetChunks() const
    {
        py::list pychunks;
        for( size_t ichunk = 0; ichunk < dof; ++ichunk ) {
            piecewisepolynomials::Chunk& chunk = _ptraj->vchunks[ichunk];
            pychunks.append(PyChunkPtr(new PyChunk(chunk.duration, chunk.vpolynomials)));
        }
        return pychunks;
    }

    PyChunkPtr GetChunk(int index) const
    {
        piecewisepolynomials::Chunk& chunk = _ptraj->vchunks[index];
        return PyChunkPtr(new PyChunk(chunk.duration, chunk.vpolynomials));
    }

private:
    void _PostProcess()
    {
        degree = _ptraj->degree;
        dof = _ptraj->dof;
        duration = _ptraj->duration;
    }

public:
    size_t degree;
    size_t dof;
    dReal duration;

    PiecewisePolynomialTrajectoryPtr _ptraj;

}; // end class PyPiecewisePolynomialTrajectory

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
    .def("Initialize", &PyPolynomial::Initialize, PY_ARGS("coeffs") "Reinitialize this polynomial with the given coefficients.")
    .def("PadCoefficients", &PyPolynomial::PadCoefficients, PY_ARGS("newdegree") "Append zeros to the coefficient vectors")
    .def("Eval", &PyPolynomial::Eval, PY_ARGS("t") "Evaluate the value of this polynomial at the given parameter t")
    .def("Evald1", &PyPolynomial::Evald1, PY_ARGS("t") "Evaluate the first derivative of this polynomial at the given parameter t")
    .def("Evald2", &PyPolynomial::Evald2, PY_ARGS("t") "Evaluate the second derivative of this polynomial at the given parameter t")
    .def("Evald3", &PyPolynomial::Evald3, PY_ARGS("t") "Evaluate the third derivative of this polynomial at the given parameter t")
    .def("Evaldn", &PyPolynomial::Evald3, PY_ARGS("t", "n") "Evaluate the n-th derivative of this polynomial at the given parameter t")
    .def("GetExtrema", &PyPolynomial::GetExtrema, "Return the list of extrema of this polynomial")
    .def("FindAllLocalExtrema", &PyPolynomial::FindAllLocalExtrema, PY_ARGS("ideriv") "Return the list of extrema of the i-th derivative of this polynomial")
    .def("Serialize", &PyPolynomial::Serialize, "Serialize this polynomial into string")
    .def("Deserialize", &PyPolynomial::Deserialize, PY_ARGS("s") "Deserialize a polynomial from the given string")
    .def("__repr__", &PyPolynomial::__repr__)
    .def("__str__", &PyPolynomial::__str__)
    ; // end class_ PyPolynomial

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyChunk, PyChunkPtr>(m, "Chunk", "PiecewisePolynomials::Chunk wrapper")
    .def(init<>())
    .def(init<dReal, py::object>(), "duration"_a, "vpolynomials"_a)
#else
    class_<PyChunk, PyChunkPtr>("Chunk", "PiecewisePolynomials::Chunk wrapper", no_init)
    .def(init<>())
    .def(init<dReal, py::object>(py::args("duration", "vpolynomials")))
#endif
    .def_readonly("degree", &PyChunk::degree)
    .def_readonly("dof", &PyChunk::dof)
    .def_readonly("duration", &PyChunk::duration)
    .def("Initialize", &PyChunk::Initialize, PY_ARGS("duration", "vpolynomials") "Reinitialize this chunk with the given duration and polynomials.")
    .def("Cut", &PyChunk::Cut, PY_ARGS("t") "Cut this chunk into two halves. The left half (from t = 0 to t = t) is stored in this chunk. The right half is returned.")
    .def("Eval", &PyChunk::Eval, PY_ARGS("t") "Evaluate the value of this chunk at the given parameter t")
    .def("Evald1", &PyChunk::Evald1, PY_ARGS("t") "Evaluate the first derivative of this chunk at the given parameter t")
    .def("Evald2", &PyChunk::Evald2, PY_ARGS("t") "Evaluate the second derivative of this chunk at the given parameter t")
    .def("Evald3", &PyChunk::Evald3, PY_ARGS("t") "Evaluate the third derivative of this chunk at the given parameter t")
    .def("Evaldn", &PyChunk::Evald3, PY_ARGS("t", "n") "Evaluate the n-th derivative of this chunk at the given parameter t")
    .def("SetConstant", &PyChunk::SetConstant, PY_ARGS("x0Vect", "duration", "degree") "Initialize this chunk with constant polynomials")
    .def("Serialize", &PyChunk::Serialize, "Serialize this chunk into string")
    .def("Deserialize", &PyChunk::Deserialize, PY_ARGS("s") "Deserialize a chunk from the given string")
    .def("GetPolynomials", &PyChunk::GetPolynomials, "Return a list of polynomials from this chunk")
    .def("GetPolynomial", &PyChunk::GetPolynomial, PY_ARGS("index") "Return the polynomial at the given index")
    ; // end class_ PyChunk

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPiecewisePolynomialTrajectory, PyPiecewisePolynomialTrajectoryPtr>(m, "PiecewisePolynomialTrajectory", "PiecewisePolynomials::PiecewisePolynomialTrajectory wrapper")
    .def(init<>())
    .def(init<py::object>(), "chunks"_a)
#else
    class_<PyPiecewisePolynomialTrajectory, PyPiecewisePolynomialTrajectoryPtr>("PiecewisePolynomialTrajectory", "PiecewisePolynomials::PiecewisePolynomialTrajectory wrapper", no_init)
    .def(init<>())
    .def(init<py::object>(py::args("chunks")))
#endif
    .def("Initialize", &PyPiecewisePolynomialTrajectory::Initialize, PY_ARGS("chunks") "Reinitialize this trajectory with the given chunks.")
    .def("Eval", &PyPiecewisePolynomialTrajectory::Eval, PY_ARGS("t") "Evaluate the value of this trajectory at the given parameter t")
    .def("Evald1", &PyPiecewisePolynomialTrajectory::Evald1, PY_ARGS("t") "Evaluate the first derivative of this trajectory at the given parameter t")
    .def("Evald2", &PyPiecewisePolynomialTrajectory::Evald2, PY_ARGS("t") "Evaluate the second derivative of this trajectory at the given parameter t")
    .def("Evald3", &PyPiecewisePolynomialTrajectory::Evald3, PY_ARGS("t") "Evaluate the third derivative of this trajectory at the given parameter t")
    .def("Evaldn", &PyPiecewisePolynomialTrajectory::Evald3, PY_ARGS("t", "n") "Evaluate the n-th derivative of this trajectory at the given parameter t")
    .def("FindChunkIndex", &PyPiecewisePolynomialTrajectory::FindChunkIndex, PY_ARGS("t") "Find the index of the chunk in which the given time t falls into. Also compute the remainder of that chunk.")
    .def("Serialize", &PyPiecewisePolynomialTrajectory::Serialize, "Serialize this trajectory into string")
    .def("Deserialize", &PyPiecewisePolynomialTrajectory::Deserialize, PY_ARGS("s") "Deserialize a trajectory from the given string")
    .def("GetChunks", &PyPiecewisePolynomialTrajectory::GetChunks, "Return a list of chunks from this trajectory")
    .def("GetChunk", &PyPiecewisePolynomialTrajectory::GetChunk, PY_ARGS("index") "Return the chunk at the given index")
    ; // end class_ PyPiecewisePolynomialTrajectory
}
