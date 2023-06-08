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
#include "interpolatorbase.h"
#include "cubicinterpolator.h"
#include "quinticinterpolator.h"
#include "generalrecursiveinterpolator.h"
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

namespace piecewisepolynomials = OpenRAVE::PiecewisePolynomialsInternal;
using OpenRAVE::dReal;

namespace piecewisepolynomialspy {

class PyCoordinate;
class PyPolynomial;
class PyPiecewisePolynomial;
class PyChunk;
class PyPiecewisePolynomialTrajectory;
class PyInterpolator;
class PyGeneralRecursiveInterpolator;
class PyPolynomialChecker;
typedef boost::shared_ptr<PyCoordinate> PyCoordinatePtr;
typedef boost::shared_ptr<PyPolynomial> PyPolynomialPtr;
typedef boost::shared_ptr<PyPiecewisePolynomial> PyPiecewisePolynomialPtr;
typedef boost::shared_ptr<PyChunk> PyChunkPtr;
typedef boost::shared_ptr<PyPiecewisePolynomialTrajectory> PyPiecewisePolynomialTrajectoryPtr;
typedef boost::shared_ptr<PyInterpolator> PyInterpolatorPtr;
typedef boost::shared_ptr<PyGeneralRecursiveInterpolator> PyGeneralRecursiveInterpolatorPtr;
typedef boost::shared_ptr<PyPolynomialChecker> PyPolynomialCheckerPtr;

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
    piecewisepolynomials::CoordinatePtr _pcoord;
}; // end class PyCoordinate

class PyPolynomial {
public:
    PyPolynomial()
    {
        _ppolynomial.reset(new piecewisepolynomials::Polynomial());
    }
    PyPolynomial(const dReal duration_, const py::object ocoeffs_)
    {
        std::vector<dReal> inputCoeffs = openravepy::ExtractArray<dReal>(ocoeffs_);
        _ppolynomial.reset(new piecewisepolynomials::Polynomial(duration_, inputCoeffs));
        _PostProcess();
    }
    PyPolynomial(const dReal duration_, const std::vector<dReal>& coeffs_)
    {
        _ppolynomial.reset(new piecewisepolynomials::Polynomial(duration_, coeffs_));
        _PostProcess();
    }
    PyPolynomial(const piecewisepolynomials::Polynomial& polynomial)
    {
        _ppolynomial.reset(new piecewisepolynomials::Polynomial(polynomial.duration, polynomial.vcoeffs));
        _PostProcess();
    }

    void Initialize(const dReal duration_, const py::object ocoeffs_)
    {
        std::vector<dReal> inputCoeffs = openravepy::ExtractArray<dReal>(ocoeffs_);
        _ppolynomial->Initialize(duration_, inputCoeffs);
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

    void UpdateDuration(dReal duration_)
    {
        _ppolynomial->UpdateDuration(duration_);
        _PostProcess();
    }

    void Reparameterize(const dReal t0)
    {
        _ppolynomial->Reparameterize(t0);
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

    PyPolynomialPtr Differentiate(const size_t ideriv) const
    {
        piecewisepolynomials::Polynomial newPoly = _ppolynomial->Differentiate(ideriv);
        return PyPolynomialPtr(new PyPolynomial(newPoly));
    }

    dReal Evali1(dReal t, const dReal c) const
    {
        return _ppolynomial->Evali1(t, c);
    }

    PyPolynomialPtr Integrate(const dReal c) const
    {
        piecewisepolynomials::Polynomial newPoly = _ppolynomial->Integrate(c);
        return PyPolynomialPtr(new PyPolynomial(newPoly));
    }

    py::list GetExtrema() const
    {
        py::list oExtrema;
        FOREACHC(itextremum, _ppolynomial->GetExtrema()) {
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

    py::list GetCoefficients() const
    {
        py::list ocoeffs;
        FOREACHC(itcoeff, _ppolynomial->vcoeffs) {
            ocoeffs.append( *itcoeff );
        }
        return ocoeffs;
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
        duration = _ppolynomial->duration;
        displacement = _ppolynomial->displacement;

        _srep.clear();
        _srep += "Polynomial(" + std::to_string(duration) + ", [";
        FOREACHC(itcoeff, _ppolynomial->vcoeffs) {
            _srep += std::to_string(*itcoeff);
            _srep += ", ";
        }
        _srep += "])";
    }

public:
    piecewisepolynomials::PolynomialPtr _ppolynomial;
    dReal duration;
    dReal displacement;
    std::string _srep; // string representation of this polynomial
}; // end class PyPolynomial

std::vector<piecewisepolynomials::Polynomial> ExtractArrayPolynomials(const py::object ov)
{
    size_t numPolynomials = len(ov);
    std::vector<piecewisepolynomials::Polynomial> v;
    v.reserve(numPolynomials);
    for(size_t ipoly = 0; ipoly < numPolynomials; ++ipoly ) {
        PyPolynomialPtr ppypoly = py::extract<PyPolynomialPtr>(ov[py::to_object(ipoly)]);
        if( !!ppypoly ) {
            v.push_back(*(ppypoly->_ppolynomial));
        }
        else {
            RAVELOG_ERROR_FORMAT("failed to get polynomial at index=%d", ipoly);
        }
    }
    return v;
}

class PyPiecewisePolynomial {
public:
    PyPiecewisePolynomial()
    {
        _ppwpoly.reset(new piecewisepolynomials::PiecewisePolynomial());
        _PostProcess();
    }
    PyPiecewisePolynomial(const py::object ovpolynomials)
    {
        std::vector<piecewisepolynomials::Polynomial> vpolynomials = ExtractArrayPolynomials(ovpolynomials);
        _ppwpoly.reset(new piecewisepolynomials::PiecewisePolynomial(vpolynomials));
        _PostProcess();
    }
    PyPiecewisePolynomial(const piecewisepolynomials::PiecewisePolynomial& pwpoly)
    {
        std::vector<piecewisepolynomials::Polynomial> polynomials = pwpoly.GetPolynomials(); // make a copy
        _ppwpoly.reset(new piecewisepolynomials::PiecewisePolynomial(polynomials));
        _PostProcess();
    }

    void Initialize(const py::object ovpolynomials)
    {
        std::vector<piecewisepolynomials::Polynomial> vpolynomials = ExtractArrayPolynomials(ovpolynomials);
        _ppwpoly.reset(new piecewisepolynomials::PiecewisePolynomial(vpolynomials));
        _PostProcess();
    }

    void UpdateInitialValue(dReal c0)
    {
        _ppwpoly->UpdateInitialValue(c0);
        _PostProcess();
    }

    py::object FindPolynomialIndex(dReal t) const
    {
        size_t index;
        dReal remainder;
        _ppwpoly->FindPolynomialIndex(t, index, remainder);
        return py::make_tuple(index, remainder);
    }

    dReal Eval(dReal t) const
    {
        return _ppwpoly->Eval(t);
    }

    dReal Evald1(dReal t) const
    {
        return _ppwpoly->Evald1(t);
    }

    dReal Evald2(dReal t) const
    {
        return _ppwpoly->Evald2(t);
    }

    dReal Evald3(dReal t) const
    {
        return _ppwpoly->Evald3(t);
    }

    dReal Evaldn(dReal t, size_t n) const
    {
        return _ppwpoly->Evaldn(t, n);
    }

    dReal Evali1(dReal t, const dReal c) const
    {
        return _ppwpoly->Evali1(t, c);
    }

    py::object Serialize() const
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
        _ppwpoly->Serialize(ss);
        return py::to_object(ss.str());
    }

    void Deserialize(const std::string s)
    {
        std::stringstream ss(s);
        _ppwpoly->Deserialize(ss);
        _PostProcess();
    }

    PyPiecewisePolynomialPtr Differentiate(const size_t ideriv) const
    {
        piecewisepolynomials::PiecewisePolynomial newPWPoly = _ppwpoly->Differentiate(ideriv);
        return PyPiecewisePolynomialPtr(new PyPiecewisePolynomial(newPWPoly));
    }

    PyPiecewisePolynomialPtr Integrate(const dReal c) const
    {
        piecewisepolynomials::PiecewisePolynomial newPWPoly = _ppwpoly->Integrate(c);
        return PyPiecewisePolynomialPtr(new PyPiecewisePolynomial(newPWPoly));
    }

    // void Cut(dReal t, PiecewisePolynomial &remPWPolynomial);

    // void TrimFront(dReal t);

    // void TrimBack(dReal t);

    py::object GetPolynomials() const
    {
        py::list pypolynomials;
        const std::vector<piecewisepolynomials::Polynomial>& vpolynomials = _ppwpoly->GetPolynomials();
        FOREACHC(itpoly, vpolynomials) {
            pypolynomials.append(PyPolynomialPtr(new PyPolynomial(*itpoly)));
        }
        return pypolynomials;
    }

    PyPolynomialPtr GetPolynomial(size_t index) const
    {
        return PyPolynomialPtr(new PyPolynomial(_ppwpoly->GetPolynomial(index)));
    }

    dReal GetDuration() const
    {
        return _ppwpoly->GetDuration();
    }

    size_t GetNumPolynomials() const
    {
        return _ppwpoly->GetPolynomials().size();
    }

    PyPolynomialPtr ExtractPolynomial(const dReal t0, const dReal t1) const
    {
        piecewisepolynomials::Polynomial poly = _ppwpoly->ExtractPolynomial(t0, t1);
        return PyPolynomialPtr(new PyPolynomial(poly));
    }

private:
    void _PostProcess()
    {
        duration = _ppwpoly->GetDuration();
    }

public:
    piecewisepolynomials::PiecewisePolynomialPtr _ppwpoly;
    dReal duration;
}; // end class PyPiecewisePolynomial

class PyChunk {
public:
    PyChunk()
    {
        _pchunk.reset(new piecewisepolynomials::Chunk());
        _PostProcess();
    }
    PyChunk(const dReal duration_, const py::object ovpolynomials)
    {
        std::vector<piecewisepolynomials::Polynomial> vpolynomials = ExtractArrayPolynomials(ovpolynomials);
        _pchunk.reset(new piecewisepolynomials::Chunk(duration_, vpolynomials));
        _PostProcess();
    }
    PyChunk(const dReal duration_, const std::vector<piecewisepolynomials::Polynomial>& vpolynomials)
    {
        _pchunk.reset(new piecewisepolynomials::Chunk(duration_, vpolynomials));
        _PostProcess();
    }

    void Initialize(const dReal duration_, const py::object ovpolynomials)
    {
        std::vector<piecewisepolynomials::Polynomial> vpolynomials = ExtractArrayPolynomials(ovpolynomials);
        _pchunk->Initialize(duration_, vpolynomials);
        _PostProcess();
    }

    void UpdateInitialValues(const py::object ovinitialvalues)
    {
        std::vector<dReal> vinitialvalues = openravepy::ExtractArray<dReal>(ovinitialvalues);
        _pchunk->UpdateInitialValues(vinitialvalues);
        _PostProcess();
    }

    void UpdateDuration(const dReal duration_)
    {
        _pchunk->UpdateDuration(duration_);
        _PostProcess();
    }

    PyChunkPtr Cut(dReal t)
    {
        piecewisepolynomials::Chunk remChunk;
        _pchunk->Cut(t, remChunk);
        _PostProcess();

        PyChunkPtr pyRemChunk(new PyChunk());
        pyRemChunk->_Initialize(remChunk);
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

    void SetConstant(const py::object ox0Vect, const dReal duration_, const size_t degree_)
    {
        std::vector<dReal> x0Vect = openravepy::ExtractArray<dReal>(ox0Vect);
        _pchunk->SetConstant(x0Vect, duration_, degree_);
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
            pypolynomials.append(PyPolynomialPtr(new PyPolynomial(_pchunk->vpolynomials[ipoly])));
        }
        return pypolynomials;
    }

    PyPolynomialPtr GetPolynomial(int index) const
    {

        return PyPolynomialPtr(new PyPolynomial(_pchunk->vpolynomials.at(index)));
    }

private:
    void _Initialize(const piecewisepolynomials::Chunk& chunk)
    {
        _pchunk->Initialize(chunk.duration, chunk.vpolynomials);
        _PostProcess();
    }

    void _PostProcess()
    {
        degree = _pchunk->degree;
        dof = _pchunk->dof;
        duration = _pchunk->duration;
    }

public:
    piecewisepolynomials::ChunkPtr _pchunk;

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
        PyChunkPtr ppychunk = py::extract<PyChunkPtr>(ov[py::to_object(ichunk)]);
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

    size_t GetNumChunks() const
    {
        return _ptraj->vchunks.size();
    }

    py::object GetChunks() const
    {
        py::list pychunks;
        for( size_t ichunk = 0; ichunk < _ptraj->vchunks.size(); ++ichunk ) {
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

    piecewisepolynomials::PiecewisePolynomialTrajectoryPtr _ptraj;

}; // end class PyPiecewisePolynomialTrajectory

class PyInterpolator {
public:
    PyInterpolator()
    {
    }
    PyInterpolator(const std::string& interpolatorname, size_t ndof, int envid)
    {
        Initialize(interpolatorname, ndof, envid);
        _PostProcess();
    }

    void Initialize(const std::string& interpolatorname, size_t ndof, int envid)
    {
        if( interpolatorname == "quinticinterpolator" ) {
            _pinterpolator.reset(new piecewisepolynomials::QuinticInterpolator(ndof, envid));
        }
        else if( interpolatorname == "cubicinterpolator" ) {
            _pinterpolator.reset(new piecewisepolynomials::CubicInterpolator(ndof, envid));
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("Invalid interpolatorname %s", interpolatorname, ORE_InvalidArguments);
        }
        _PostProcess();
    }

    const std::string GetDescription() const
    {
        return _pinterpolator->GetDescription();
    }

    PyPiecewisePolynomialPtr Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(dReal x0, dReal x1,
                                                                                     dReal vm, dReal am, dReal jm)
    {
        piecewisepolynomials::PiecewisePolynomial pwpoly;
        piecewisepolynomials::PolynomialCheckReturn ret = _pinterpolator->Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration(x0, x1, vm, am, jm, pwpoly);
        if( ret == piecewisepolynomials::PCR_Normal ) {
            return PyPiecewisePolynomialPtr(new PyPiecewisePolynomial(pwpoly));
        }
        else {
            return nullptr;
        }
    }

    PyPiecewisePolynomialPtr Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration(dReal x0, dReal x1,
                                                                                          dReal v0, dReal v1,
                                                                                          dReal a0, dReal a1,
                                                                                          dReal xmin, dReal xmax,
                                                                                          dReal vm, dReal am, dReal jm)
    {
        piecewisepolynomials::PiecewisePolynomial pwpoly;
        piecewisepolynomials::PolynomialCheckReturn ret = _pinterpolator->Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration(x0, x1, v0, v1, a0, a1, xmin, xmax, vm, am, jm, pwpoly);
        if( ret == piecewisepolynomials::PCR_Normal ) {
            return PyPiecewisePolynomialPtr(new PyPiecewisePolynomial(pwpoly));
        }
        else {
            RAVELOG_DEBUG_FORMAT("interpolation failed with ret=%s", piecewisepolynomials::GetPolynomialCheckReturnString(ret));
            return nullptr;
        }
    }

    PyPiecewisePolynomialPtr Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(dReal x0, dReal x1,
                                                                                      dReal v0, dReal v1,
                                                                                      dReal a0, dReal a1, dReal T,
                                                                                      dReal xmin, dReal xmax,
                                                                                      dReal vm, dReal am, dReal jm)
    {
        piecewisepolynomials::PiecewisePolynomial pwpoly;
        piecewisepolynomials::PolynomialCheckReturn ret = _pinterpolator->Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration(x0, x1, v0, v1, a0, a1, T, xmin, xmax, vm, am, jm, pwpoly);
        if( ret == piecewisepolynomials::PCR_Normal ) {
            return PyPiecewisePolynomialPtr(new PyPiecewisePolynomial(pwpoly));
        }
        else {
            RAVELOG_DEBUG_FORMAT("interpolation failed with ret=%s", piecewisepolynomials::GetPolynomialCheckReturnString(ret));
            return nullptr;
        }
    }

    py::object ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration(const py::object ox0Vect, const py::object ox1Vect,
                                                                       const py::object ovmVect, const py::object oamVect, const py::object ojmVect)
    {
        std::vector<dReal> vmVect = openravepy::ExtractArray<dReal>(ovmVect);
        std::vector<dReal> amVect = openravepy::ExtractArray<dReal>(oamVect);
        std::vector<dReal> jmVect = openravepy::ExtractArray<dReal>(ojmVect);
        std::vector<dReal> x0Vect = openravepy::ExtractArray<dReal>(ox0Vect);
        std::vector<dReal> x1Vect = openravepy::ExtractArray<dReal>(ox1Vect);
        std::vector<piecewisepolynomials::Chunk> vchunks;
        piecewisepolynomials::PolynomialCheckReturn ret = _pinterpolator->ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration(x0Vect, x1Vect, vmVect, amVect, jmVect, vchunks);
        if( ret != piecewisepolynomials::PCR_Normal ) {
            RAVELOG_DEBUG_FORMAT("interpolation failed with ret=%s", piecewisepolynomials::GetPolynomialCheckReturnString(ret));
            return py::none_();
        }

        py::list pychunks;
        for( size_t ichunk = 0; ichunk < vchunks.size(); ++ichunk ) {
            piecewisepolynomials::Chunk& chunk = vchunks[ichunk];
            pychunks.append(PyChunkPtr(new PyChunk(chunk.duration, chunk.vpolynomials)));
        }
        return pychunks;
    }

    py::object ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(const py::object ox0Vect, const py::object ox1Vect,
                                                                        const py::object ov0Vect, const py::object ov1Vect,
                                                                        const py::object oa0Vect, const py::object oa1Vect,
                                                                        const dReal T,
                                                                        const py::object oxminVect, const py::object oxmaxVect,
                                                                        const py::object ovmVect, const py::object oamVect,
                                                                        const py::object ojmVect)
    {
        std::vector<dReal> xminVect = openravepy::ExtractArray<dReal>(oxminVect);
        std::vector<dReal> xmaxVect = openravepy::ExtractArray<dReal>(oxmaxVect);
        std::vector<dReal> vmVect = openravepy::ExtractArray<dReal>(ovmVect);
        std::vector<dReal> amVect = openravepy::ExtractArray<dReal>(oamVect);
        std::vector<dReal> jmVect = openravepy::ExtractArray<dReal>(ojmVect);

        std::vector<dReal> x0Vect = openravepy::ExtractArray<dReal>(ox0Vect);
        std::vector<dReal> x1Vect = openravepy::ExtractArray<dReal>(ox1Vect);
        std::vector<dReal> v0Vect = openravepy::ExtractArray<dReal>(ov0Vect);
        std::vector<dReal> v1Vect = openravepy::ExtractArray<dReal>(ov1Vect);
        std::vector<dReal> a0Vect = openravepy::ExtractArray<dReal>(oa0Vect);
        std::vector<dReal> a1Vect = openravepy::ExtractArray<dReal>(oa1Vect);
        std::vector<piecewisepolynomials::Chunk> vchunks;
        piecewisepolynomials::PolynomialCheckReturn ret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, T, xminVect, xmaxVect, vmVect, amVect, jmVect, vchunks);
        if( ret != piecewisepolynomials::PCR_Normal ) {
            RAVELOG_DEBUG_FORMAT("interpolation failed with ret=%s", piecewisepolynomials::GetPolynomialCheckReturnString(ret));
            return py::none_();
        }

        py::list pychunks;
        for( size_t ichunk = 0; ichunk < vchunks.size(); ++ichunk ) {
            piecewisepolynomials::Chunk& chunk = vchunks[ichunk];
            pychunks.append(PyChunkPtr(new PyChunk(chunk.duration, chunk.vpolynomials)));
        }
        return pychunks;
    }

    py::object ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(const py::object ox0Vect, const py::object ox1Vect,
                                                                            const py::object ov0Vect, const py::object ov1Vect,
                                                                            const py::object oa0Vect, const py::object oa1Vect,
                                                                            const py::object oxminVect, const py::object oxmaxVect,
                                                                            const py::object ovmVect, const py::object oamVect,
                                                                            const py::object ojmVect, const dReal T)
    {
        std::vector<dReal> xminVect = openravepy::ExtractArray<dReal>(oxminVect);
        std::vector<dReal> xmaxVect = openravepy::ExtractArray<dReal>(oxmaxVect);
        std::vector<dReal> vmVect = openravepy::ExtractArray<dReal>(ovmVect);
        std::vector<dReal> amVect = openravepy::ExtractArray<dReal>(oamVect);
        std::vector<dReal> jmVect = openravepy::ExtractArray<dReal>(ojmVect);

        std::vector<dReal> x0Vect = openravepy::ExtractArray<dReal>(ox0Vect);
        std::vector<dReal> x1Vect = openravepy::ExtractArray<dReal>(ox1Vect);
        std::vector<dReal> v0Vect = openravepy::ExtractArray<dReal>(ov0Vect);
        std::vector<dReal> v1Vect = openravepy::ExtractArray<dReal>(ov1Vect);
        std::vector<dReal> a0Vect = openravepy::ExtractArray<dReal>(oa0Vect);
        std::vector<dReal> a1Vect = openravepy::ExtractArray<dReal>(oa1Vect);
        std::vector<piecewisepolynomials::Chunk> vchunks;
        piecewisepolynomials::PolynomialCheckReturn ret = _pinterpolator->ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration(x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect, xminVect, xmaxVect, vmVect, amVect, jmVect, T, vchunks);
        if( ret != piecewisepolynomials::PCR_Normal ) {
            RAVELOG_DEBUG_FORMAT("interpolation failed with ret=%s", piecewisepolynomials::GetPolynomialCheckReturnString(ret));
            return py::none_();
        }

        py::list pychunks;
        for( size_t ichunk = 0; ichunk < vchunks.size(); ++ichunk ) {
            piecewisepolynomials::Chunk& chunk = vchunks[ichunk];
            pychunks.append(PyChunkPtr(new PyChunk(chunk.duration, chunk.vpolynomials)));
        }
        return pychunks;
    }

private:
    void _PostProcess()
    {
        _ndof = _pinterpolator->ndof;
        _envid = _pinterpolator->envid;
        _xmlid = _pinterpolator->GetXMLId();
    }

    piecewisepolynomials::InterpolatorBasePtr _pinterpolator;
    size_t _ndof;
    int _envid;
    std::string _xmlid;
}; // end class PyInterpolator

class PyGeneralRecursiveInterpolator {
public:
    PyGeneralRecursiveInterpolator()
    {
    }
    PyGeneralRecursiveInterpolator(int envid)
    {
        _pinterpolator.reset(new piecewisepolynomials::GeneralRecursiveInterpolator(envid));
    }

    PyPiecewisePolynomialPtr Compute1DTrajectory(const size_t degree,
                                                 const py::object oInitialState, const py::object oFinalState,
                                                 const py::object oLowerBounds, const py::object oUpperBounds,
                                                 const dReal fixedDuration)
    {
        std::vector<dReal> initialState = openravepy::ExtractArray<dReal>(oInitialState);
        std::vector<dReal> finalState = openravepy::ExtractArray<dReal>(oFinalState);
        std::vector<dReal> lowerBounds = openravepy::ExtractArray<dReal>(oLowerBounds);
        std::vector<dReal> upperBounds = openravepy::ExtractArray<dReal>(oUpperBounds);
        piecewisepolynomials::PiecewisePolynomial pwpoly;
        piecewisepolynomials::PolynomialCheckReturn ret = _pinterpolator->Compute1DTrajectory(degree, initialState, finalState, lowerBounds, upperBounds, fixedDuration, pwpoly);
        if( ret != piecewisepolynomials::PolynomialCheckReturn::PCR_Normal ) {
            return nullptr;
        }

        return PyPiecewisePolynomialPtr(new PyPiecewisePolynomial(pwpoly));
    }

private:
    piecewisepolynomials::GeneralRecursiveInterpolatorPtr _pinterpolator;
}; // end class PyGeneralRecursiveInterpolator

class PyPolynomialChecker {
public:
    PyPolynomialChecker()
    {
        _pchecker.reset(new piecewisepolynomials::PolynomialChecker());
    }
    PyPolynomialChecker(size_t ndof, int envid)
    {
        _pchecker.reset(new piecewisepolynomials::PolynomialChecker(ndof, envid));
    }

    void Initialize(size_t ndof, int envid)
    {
        _pchecker->Initialize(ndof, envid);
    }

    uint8_t CheckPolynomial(const py::object opolynomial, const dReal xmin, const dReal xmax, const dReal vm, const dReal am, const dReal jm,
                            const dReal x0, const dReal x1, const dReal v0, const dReal v1, const dReal a0, const dReal a1)
    {
        PyPolynomialPtr ppypoly = py::extract<PyPolynomialPtr>(opolynomial);
        piecewisepolynomials::PolynomialCheckReturn ret = _pchecker->CheckPolynomial(*ppypoly->_ppolynomial, xmin, xmax, vm, am, jm, x0, x1, v0, v1, a0, a1);
        return ret;
    }

    uint8_t CheckPiecewisePolynomial(const py::object opwpolynomial, const dReal xmin, const dReal xmax, const dReal vm, const dReal am, const dReal jm,
                                     const dReal x0, const dReal x1, const dReal v0, const dReal v1, const dReal a0, const dReal a1)
    {
        PyPiecewisePolynomialPtr ppypwpoly = py::extract<PyPiecewisePolynomialPtr>(opwpolynomial);
        piecewisepolynomials::PolynomialCheckReturn ret = _pchecker->CheckPiecewisePolynomial(*ppypwpoly->_ppwpoly, xmin, xmax, vm, am, jm, x0, x1, v0, v1, a0, a1);
        return ret;
    }

    uint8_t CheckChunk(const py::object opychunk, const py::object oxminVect, const py::object oxmaxVect, const py::object ovmVect, const py::object oamVect, const py::object ojmVect,
                       const py::object ox0Vect, const py::object ox1Vect, const py::object ov0Vect, const py::object ov1Vect, const py::object oa0Vect, const py::object oa1Vect)
    {
        PyChunkPtr ppychunk = py::extract<PyChunkPtr>(opychunk);
        std::vector<dReal> xminVect = openravepy::ExtractArray<dReal>(oxminVect);
        std::vector<dReal> xmaxVect = openravepy::ExtractArray<dReal>(oxmaxVect);
        std::vector<dReal> vmVect = openravepy::ExtractArray<dReal>(ovmVect);
        std::vector<dReal> amVect = openravepy::ExtractArray<dReal>(oamVect);
        std::vector<dReal> jmVect = openravepy::ExtractArray<dReal>(ojmVect);
        std::vector<dReal> x0Vect = openravepy::ExtractArray<dReal>(ox0Vect);
        std::vector<dReal> x1Vect = openravepy::ExtractArray<dReal>(ox1Vect);
        std::vector<dReal> v0Vect = openravepy::ExtractArray<dReal>(ov0Vect);
        std::vector<dReal> v1Vect = openravepy::ExtractArray<dReal>(ov1Vect);
        std::vector<dReal> a0Vect = openravepy::ExtractArray<dReal>(oa0Vect);
        std::vector<dReal> a1Vect = openravepy::ExtractArray<dReal>(oa1Vect);
        piecewisepolynomials::PolynomialCheckReturn ret = _pchecker->CheckChunk(*ppychunk->_pchunk, xminVect, xmaxVect, vmVect, amVect, jmVect,
                                                                                x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect);
        return ret;
    }

    uint8_t CheckChunks(const py::object ovchunks, const py::object oxminVect, const py::object oxmaxVect, const py::object ovmVect, const py::object oamVect, const py::object ojmVect,
                        const py::object ox0Vect, const py::object ox1Vect, const py::object ov0Vect, const py::object ov1Vect, const py::object oa0Vect, const py::object oa1Vect)
    {
        std::vector<piecewisepolynomials::Chunk> vchunks = ExtractArrayChunks(ovchunks);
        std::vector<dReal> xminVect = openravepy::ExtractArray<dReal>(oxminVect);
        std::vector<dReal> xmaxVect = openravepy::ExtractArray<dReal>(oxmaxVect);
        std::vector<dReal> vmVect = openravepy::ExtractArray<dReal>(ovmVect);
        std::vector<dReal> amVect = openravepy::ExtractArray<dReal>(oamVect);
        std::vector<dReal> jmVect = openravepy::ExtractArray<dReal>(ojmVect);
        std::vector<dReal> x0Vect = openravepy::ExtractArray<dReal>(ox0Vect);
        std::vector<dReal> x1Vect = openravepy::ExtractArray<dReal>(ox1Vect);
        std::vector<dReal> v0Vect = openravepy::ExtractArray<dReal>(ov0Vect);
        std::vector<dReal> v1Vect = openravepy::ExtractArray<dReal>(ov1Vect);
        std::vector<dReal> a0Vect = openravepy::ExtractArray<dReal>(oa0Vect);
        std::vector<dReal> a1Vect = openravepy::ExtractArray<dReal>(oa1Vect);
        piecewisepolynomials::PolynomialCheckReturn ret = _pchecker->CheckChunks(vchunks, xminVect, xmaxVect, vmVect, amVect, jmVect,
                                                                                 x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect);
        return ret;
    }

    uint8_t CheckPiecewisePolynomialTrajectory(const py::object otraj, const py::object oxminVect, const py::object oxmaxVect, const py::object ovmVect, const py::object oamVect, const py::object ojmVect,
                                               const py::object ox0Vect, const py::object ox1Vect, const py::object ov0Vect, const py::object ov1Vect, const py::object oa0Vect, const py::object oa1Vect)
    {
        PyPiecewisePolynomialTrajectoryPtr ppytraj = py::extract<PyPiecewisePolynomialTrajectoryPtr>(otraj);
        std::vector<dReal> xminVect = openravepy::ExtractArray<dReal>(oxminVect);
        std::vector<dReal> xmaxVect = openravepy::ExtractArray<dReal>(oxmaxVect);
        std::vector<dReal> vmVect = openravepy::ExtractArray<dReal>(ovmVect);
        std::vector<dReal> amVect = openravepy::ExtractArray<dReal>(oamVect);
        std::vector<dReal> jmVect = openravepy::ExtractArray<dReal>(ojmVect);
        std::vector<dReal> x0Vect = openravepy::ExtractArray<dReal>(ox0Vect);
        std::vector<dReal> x1Vect = openravepy::ExtractArray<dReal>(ox1Vect);
        std::vector<dReal> v0Vect = openravepy::ExtractArray<dReal>(ov0Vect);
        std::vector<dReal> v1Vect = openravepy::ExtractArray<dReal>(ov1Vect);
        std::vector<dReal> a0Vect = openravepy::ExtractArray<dReal>(oa0Vect);
        std::vector<dReal> a1Vect = openravepy::ExtractArray<dReal>(oa1Vect);
        piecewisepolynomials::PolynomialCheckReturn ret = _pchecker->CheckPiecewisePolynomialTrajectory(*ppytraj->_ptraj, xminVect, xmaxVect, vmVect, amVect, jmVect,
                                                                                                        x0Vect, x1Vect, v0Vect, v1Vect, a0Vect, a1Vect);
        return ret;
    }

    piecewisepolynomials::PolynomialCheckerPtr _pchecker;

}; // end class PyPolynomialChecker

} // end namespace piecewisepolynomialspy

#ifndef USE_PYBIND11_PYTHON_BINDINGS

#endif // USE_PYBIND11_PYTHON_BINDINGS

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
    .def(init<dReal, py::object>(), "duration"_a, "coeffs"_a)
#else
    class_<PyPolynomial, PyPolynomialPtr>("Polynomial", "PiecewisePolynomials::Polynomial wrapper", no_init)
    .def(init<>())
    .def(init<dReal, py::object>(py::args("duration", "coeffs")))
#endif
    .def_readonly("duration", &PyPolynomial::duration)
    .def_readonly("displacement", &PyPolynomial::displacement)
    .def("Initialize", &PyPolynomial::Initialize, PY_ARGS("duration", "coeffs") "Reinitialize this polynomial with the given coefficients.")
    .def("UpdateInitialValue", &PyPolynomial::UpdateInitialValue, PY_ARGS("c0") "Update the weakest term coefficient")
    .def("UpdateDuration", &PyPolynomial::UpdateDuration, PY_ARGS("duration") "Update the duration")
    .def("Reparameterize", &PyPolynomial::Reparameterize, PY_ARGS("t0") "Doc of Polynomial::Reparameterize")
    .def("PadCoefficients", &PyPolynomial::PadCoefficients, PY_ARGS("newdegree") "Append zeros to the coefficient vectors")
    .def("Eval", &PyPolynomial::Eval, PY_ARGS("t") "Evaluate the value of this polynomial at the given parameter t")
    .def("Evald1", &PyPolynomial::Evald1, PY_ARGS("t") "Evaluate the first derivative of this polynomial at the given parameter t")
    .def("Evald2", &PyPolynomial::Evald2, PY_ARGS("t") "Evaluate the second derivative of this polynomial at the given parameter t")
    .def("Evald3", &PyPolynomial::Evald3, PY_ARGS("t") "Evaluate the third derivative of this polynomial at the given parameter t")
    .def("Evaldn", &PyPolynomial::Evaldn, PY_ARGS("t", "n") "Evaluate the n-th derivative of this polynomial at the given parameter t")
    .def("Differentiate", &PyPolynomial::Differentiate,PY_ARGS("n") "Return the polynomial d^n/dt^n p(t) where p is this polynomial")
    .def("Evali1", &PyPolynomial::Evali1, PY_ARGS("t", "c") "Evaluate the first integral of this polynomial at the given parameter t with integration constant c.")
    .def("Integrate", &PyPolynomial::Integrate,PY_ARGS("c") "Return polynomial q = integrate x=0 to x=t p(x) where p is this polynomial and q(0) = c.")
    .def("GetExtrema", &PyPolynomial::GetExtrema, "Return the list of extrema of this polynomial")
    .def("FindAllLocalExtrema", &PyPolynomial::FindAllLocalExtrema, PY_ARGS("ideriv") "Return the list of extrema of the i-th derivative of this polynomial")
    .def("GetCoefficients", &PyPolynomial::GetCoefficients, "Return the list of all coefficients (weakest term first)")
    .def("Serialize", &PyPolynomial::Serialize, "Serialize this polynomial into string")
    .def("Deserialize", &PyPolynomial::Deserialize, PY_ARGS("s") "Deserialize a polynomial from the given string")
    .def("__repr__", &PyPolynomial::__repr__)
    .def("__str__", &PyPolynomial::__str__)
    ; // end class_ PyPolynomial

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPiecewisePolynomial, PyPiecewisePolynomialPtr>(m, "PiecewisePolynomial", "PiecewisePolynomials::PiecewisePolynomial wrapper")
    .def(init<>())
    .def(init<py::object>(), "polynomials"_a)
#else
    class_<PyPiecewisePolynomial, PyPiecewisePolynomialPtr>("PiecewisePolynomial", "PiecewisePolynomials::PiecewisePolynomial wrapper", no_init)
    .def(init<>())
    .def(init<py::object>(py::args("polynomials")))
#endif
    .def_readonly("duration", &PyPiecewisePolynomial::duration)
    .def("Initialize", &PyPiecewisePolynomial::Initialize, PY_ARGS("vpolynomials") "Reinitialize this piecewise polynomial with the given polynomials.")
    .def("UpdateInitialValues", &PyPiecewisePolynomial::UpdateInitialValue, PY_ARGS("c0") "Update the weakest term coefficient of polynomials in this piecewise polynomial.")
    .def("Eval", &PyPiecewisePolynomial::Eval, PY_ARGS("t") "Evaluate the value of this piecewise polynomial at the given parameter t")
    .def("Evald1", &PyPiecewisePolynomial::Evald1, PY_ARGS("t") "Evaluate the first derivative of this piecewise polynomial at the given parameter t")
    .def("Evald2", &PyPiecewisePolynomial::Evald2, PY_ARGS("t") "Evaluate the second derivative of this piecewise polynomial at the given parameter t")
    .def("Evald3", &PyPiecewisePolynomial::Evald3, PY_ARGS("t") "Evaluate the third derivative of this piecewise polynomial at the given parameter t")
    .def("Evaldn", &PyPiecewisePolynomial::Evaldn, PY_ARGS("t", "n") "Evaluate the n-th derivative of this piecewise polynomial at the given parameter t")
    .def("Evali1", &PyPiecewisePolynomial::Evali1, PY_ARGS("t", "c") "Evaluate the first integral of this piecewise polynomial at the given parameter t with integration constant c")
    .def("Serialize", &PyPiecewisePolynomial::Serialize, "Serialize this piecewise polynomial into string")
    .def("Deserialize", &PyPiecewisePolynomial::Deserialize, PY_ARGS("s") "Deserialize a piecewise polynomial from the given string")
    .def("FindPolynomialIndex", &PyPiecewisePolynomial::FindPolynomialIndex, PY_ARGS("t") "Find the index of the polynomial q that t falls into and also compute the remainder so that p(t) = q(remainder)")
    .def("Differentiate", &PyPiecewisePolynomial::Differentiate,PY_ARGS("n") "Return the polynomial d^n/dt^n p(t) where p is this polynomial")
    .def("Integrate", &PyPiecewisePolynomial::Integrate,PY_ARGS("c") "Return polynomial q = integrate x=0 to x=t p(x) where p is this polynomial and q(0) = c.")
    .def("GetPolynomials", &PyPiecewisePolynomial::GetPolynomials, "Return a list of polynomials from this piecewise polynomial")
    .def("GetPolynomial", &PyPiecewisePolynomial::GetPolynomial, PY_ARGS("index") "Return the polynomial at the given index")
    .def("GetNumPolynomials", &PyPiecewisePolynomial::GetNumPolynomials, "Return the number of polynomials")
    .def("ExtractPolynomial", &PyPiecewisePolynomial::ExtractPolynomial, PY_ARGS("t0", "t1") "Return a polynomial representing a segment starting from t0 and ending at t1.")
    ; // end class_ PyPiecewisePolynomial

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
    .def("UpdateInitialValues", &PyChunk::UpdateInitialValues, PY_ARGS("vinitialvalues") "Update the weakest term coefficient of polynomials in this chunk.")
    .def("UpdateDuration", &PyChunk::UpdateDuration, PY_ARGS("duration") "Update the duration of this chunk")
    .def("Cut", &PyChunk::Cut, PY_ARGS("t") "Cut this chunk into two halves. The left half (from t = 0 to t = t) is stored in this chunk. The right half is returned.")
    .def("Eval", &PyChunk::Eval, PY_ARGS("t") "Evaluate the value of this chunk at the given parameter t")
    .def("Evald1", &PyChunk::Evald1, PY_ARGS("t") "Evaluate the first derivative of this chunk at the given parameter t")
    .def("Evald2", &PyChunk::Evald2, PY_ARGS("t") "Evaluate the second derivative of this chunk at the given parameter t")
    .def("Evald3", &PyChunk::Evald3, PY_ARGS("t") "Evaluate the third derivative of this chunk at the given parameter t")
    .def("Evaldn", &PyChunk::Evaldn, PY_ARGS("t", "n") "Evaluate the n-th derivative of this chunk at the given parameter t")
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
    .def_readonly("degree", &PyPiecewisePolynomialTrajectory::degree)
    .def_readonly("dof", &PyPiecewisePolynomialTrajectory::dof)
    .def_readonly("duration", &PyPiecewisePolynomialTrajectory::duration)
    .def("Initialize", &PyPiecewisePolynomialTrajectory::Initialize, PY_ARGS("chunks") "Reinitialize this trajectory with the given chunks.")
    .def("Eval", &PyPiecewisePolynomialTrajectory::Eval, PY_ARGS("t") "Evaluate the value of this trajectory at the given parameter t")
    .def("Evald1", &PyPiecewisePolynomialTrajectory::Evald1, PY_ARGS("t") "Evaluate the first derivative of this trajectory at the given parameter t")
    .def("Evald2", &PyPiecewisePolynomialTrajectory::Evald2, PY_ARGS("t") "Evaluate the second derivative of this trajectory at the given parameter t")
    .def("Evald3", &PyPiecewisePolynomialTrajectory::Evald3, PY_ARGS("t") "Evaluate the third derivative of this trajectory at the given parameter t")
    .def("Evaldn", &PyPiecewisePolynomialTrajectory::Evaldn, PY_ARGS("t", "n") "Evaluate the n-th derivative of this trajectory at the given parameter t")
    .def("FindChunkIndex", &PyPiecewisePolynomialTrajectory::FindChunkIndex, PY_ARGS("t") "Find the index of the chunk in which the given time t falls into. Also compute the remainder of that chunk.")
    .def("Serialize", &PyPiecewisePolynomialTrajectory::Serialize, "Serialize this trajectory into string")
    .def("Deserialize", &PyPiecewisePolynomialTrajectory::Deserialize, PY_ARGS("s") "Deserialize a trajectory from the given string")
    .def("GetNumChunks", &PyPiecewisePolynomialTrajectory::GetNumChunks, "Return the number of chunks in this trajectory")
    .def("GetChunks", &PyPiecewisePolynomialTrajectory::GetChunks, "Return a list of chunks from this trajectory")
    .def("GetChunk", &PyPiecewisePolynomialTrajectory::GetChunk, PY_ARGS("index") "Return the chunk at the given index")
    ; // end class_ PyPiecewisePolynomialTrajectory

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyInterpolator, PyInterpolatorPtr>(m, "Interpolator", "wrapper for interpolators")
    .def(init<>())
    .def(init<const std::string&, size_t, int>(),
         "interpolatorname"_a,
         "ndof"_a,
         "envid"_a)
#else
    class_<PyInterpolator, PyInterpolatorPtr>("Interpolator", "wrapper for interpolators")
    .def(init<>())
    .def(init<const std::string&, size_t, int>(py::args("interpolatorname", "ndof", "envid")))
#endif
    .def("Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration", &PyInterpolator::Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration, PY_ARGS("x0", "x1", "vm", "am", "jm") "Docs of Compute1DTrajectoryZeroTimeDerivativesOptimizedDuration")
    .def("Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration", &PyInterpolator::Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration, PY_ARGS("x0", "x1", "v0", "v1", "a0", "a1", "xmin", "xmax", "vm", "am", "jm") "Docs of Compute1DTrajectoryArbitraryTimeDerivativesOptimizedDuration")
    .def("Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration", &PyInterpolator::Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration, PY_ARGS("x0", "x1", "v0", "v1", "a0", "a1", "T", "xmin", "xmax", "vm", "am", "jm") "Docs of Compute1DTrajectoryArbitraryTimeDerivativesFixedDuration")
    .def("ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration", &PyInterpolator::ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration, PY_ARGS("x0Vect", "x1Vect", "vmVect", "amVect", "jmVect") "Docs of ComputeNDTrajectoryZeroTimeDerivativesOptimizedDuration")
    .def("ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration", &PyInterpolator::ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration, PY_ARGS("x0Vect", "x1Vect", "v0Vect", "v1Vect", "a0Vect", "a1Vect", "T", "xminVect", "xmaxVect", "vmVect", "amVect", "jmVect") "Docs of ComputeNDTrajectoryArbitraryTimeDerivativesFixedDuration")
    .def("ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration", &PyInterpolator::ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration, PY_ARGS("x0Vect", "x1Vect", "v0Vect", "v1Vect", "a0Vect", "a1Vect", "xminVect", "xmaxVect", "vmVect", "amVect", "jmVect", "T") "Docs of ComputeNDTrajectoryArbitraryTimeDerivativesOptimizedDuration")
    ; // end class_ PyInterpolator

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyGeneralRecursiveInterpolator, PyGeneralRecursiveInterpolatorPtr>(m, "GeneralRecursiveInterpolator", "wrapper for general-recursive-interpolators")
    .def(init<>())
    .def(init<int>(), "envid"_a)
#else
    class_<PyGeneralRecursiveInterpolator, PyGeneralRecursiveInterpolatorPtr>("GeneralRecursiveInterpolator", "wrapper for general-recursive-interpolators")
    .def(init<>())
    .def(init<int>(py::args("envid")))
#endif
    .def("Compute1DTrajectory", &PyGeneralRecursiveInterpolator::Compute1DTrajectory, PY_ARGS("degree", "initialState", "finalState", "lowerBounds", "upperBounds", "fixedDuration") "Docs of Compute1DTrajectory")
    ; // end class_ PyGeneralRecursiveInterpolator

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPolynomialChecker, PyPolynomialCheckerPtr>(m, "PolynomialChecker", "wrapper for polynomial checkers")
    .def(init<>())
    .def(init<size_t, int>(),
         "ndof"_a,
         "envid"_a)
#else
    class_<PyPolynomialChecker, PyPolynomialCheckerPtr>("PolynomialChecker", "wrapper for polynomial checkers")
    .def(init<>())
    .def(init<size_t, int>(py::args("ndof", "envid")))
#endif
    .def("Initialize", &PyPolynomialChecker::Initialize, PY_ARGS("ndof", "envid") "Initialize this checker")
    .def("CheckPolynomial", &PyPolynomialChecker::CheckPolynomial,
         PY_ARGS("polynomial", "xmin", "xmax", "vm", "am", "jm", "x0", "x1", "v0", "v1", "a0", "a1") "Check if the given polynomial respects all the limits.")
    .def("CheckPiecewisePolynomial", &PyPolynomialChecker::CheckPiecewisePolynomial,
         PY_ARGS("pwpolynomial", "xmin", "xmax", "vm", "am", "jm", "x0", "x1", "v0", "v1", "a0", "a1") "Check if the given piecewise polynomial is consistent and respects all the limits.")
    .def("CheckChunk", &PyPolynomialChecker::CheckChunk,
         PY_ARGS("chunk", "xminVect", "xmaxVect", "vmVect", "amVect", "jmVect", "x0Vect", "x1Vect", "v0Vect", "v1Vect", "a0Vect", "a1Vect") "Check if the given chunk is consistent and respects all the limits.")
    .def("CheckChunks", &PyPolynomialChecker::CheckChunks,
         PY_ARGS("pwptraj", "xminVect", "xmaxVect", "vmVect", "amVect", "jmVect", "x0Vect", "x1Vect", "v0Vect", "v1Vect", "a0Vect", "a1Vect") "Check if the given sequence of chunks is consistent and respects all the limits.")
    .def("CheckPiecewisePolynomialTrajectory", &PyPolynomialChecker::CheckPiecewisePolynomialTrajectory,
         PY_ARGS("pwptraj", "xminVect", "xmaxVect", "vmVect", "amVect", "jmVect", "x0Vect", "x1Vect", "v0Vect", "v1Vect", "a0Vect", "a1Vect") "Check if the given piecewise polynomial trajectory is consistent and respects all the limits.")
    ; // end class_ PyPolynomialChecker

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    enum_<piecewisepolynomials::PolynomialCheckReturn>(m, "PolynomialCheckReturn", py::arithmetic() DOXY_ENUM(piecewisepolynomials::PolynomialCheckReturn))
#else
    enum_<piecewisepolynomials::PolynomialCheckReturn>("PolynomialCheckReturn" DOXY_ENUM(piecewisepolynomials::PolynomialCheckReturn))
#endif
    .value("PCR_Normal", piecewisepolynomials::PCR_Normal)
    .value("PCR_PositionLimitsViolation", piecewisepolynomials::PCR_PositionLimitsViolation)
    .value("PCR_VelocityLimitsViolation", piecewisepolynomials::PCR_VelocityLimitsViolation)
    .value("PCR_AccelerationLimitsViolation", piecewisepolynomials::PCR_AccelerationLimitsViolation)
    .value("PCR_JerkLimitsViolation", piecewisepolynomials::PCR_JerkLimitsViolation)
    .value("PCR_NegativeDuration", piecewisepolynomials::PCR_NegativeDuration)
    .value("PCR_PositionDiscrepancy", piecewisepolynomials::PCR_PositionDiscrepancy)
    .value("PCR_VelocityDiscrepancy", piecewisepolynomials::PCR_VelocityDiscrepancy)
    .value("PCR_AccelerationDiscrepancy", piecewisepolynomials::PCR_AccelerationDiscrepancy)
    .value("PCR_DurationDiscrepancy", piecewisepolynomials::PCR_DurationDiscrepancy)
    .value("PCR_DurationTooLong", piecewisepolynomials::PCR_DurationTooLong)
    .value("PCR_GenericError", piecewisepolynomials::PCR_GenericError)
    ;
}
