// -*- coding: utf-8 -*-
// Copyright (C) 2016 Puttichai Lertkultanon
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
/// \author Puttichai Lertkultanon
#include "rampoptimizer/ramp.h"
#include "openraveplugindefs.h" // defining FOREACH

#define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle
#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/docstring_options.hpp>
#include <pyconfig.h>
#include <numpy/arrayobject.h>
#include <openrave/xmlreaders.h>

#include <iostream>
#include <boost/format.hpp>

using namespace OpenRAVE;
using namespace boost::python;
using namespace RampOptimizerInternal;

typedef OpenRAVE::dReal Real;

namespace rplanners {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Copied from openravepy_configurationcache.cpp
template <typename T>
inline std::vector<T> ExtractArray(const object& o) {
    std::vector<T> v(len(o));
    for(size_t i = 0; i < v.size(); ++i) {
        v[i] = extract<T>(o[i]);
    }
    return v;
}

inline numeric::array toPyArrayN(const float* pvalues, size_t N) {
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f4"));
    }
    npy_intp dims[] = {npy_intp(N)};
    PyObject *pyvalues = PyArray_SimpleNew(1, dims, PyArray_FLOAT);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, N*sizeof(float));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const float* pvalues, std::vector<npy_intp>& dims) {
    if( dims.size() == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f4"));
    }
    size_t totalsize = 1;
    FOREACH(it, dims) {
        totalsize *= *it;
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f4"));
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(), &dims[0], PyArray_FLOAT);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, totalsize*sizeof(float));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const double* pvalues, size_t N) {
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f8"));
    }
    npy_intp dims[] = {npy_intp(N)};
    PyObject *pyvalues = PyArray_SimpleNew(1, dims, PyArray_DOUBLE);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, N*sizeof(double));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const double* pvalues, std::vector<npy_intp>& dims) {
    if( dims.size() == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f8"));
    }
    size_t totalsize = 1;
    FOREACH(it, dims) {
        totalsize *= *it;
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("f8"));
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(), &dims[0], PyArray_DOUBLE);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, totalsize*sizeof(double));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const uint8_t* pvalues, std::vector<npy_intp>& dims) {
    if( dims.size() == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("u1"));
    }
    size_t totalsize = 1;
    for(size_t i = 0; i < dims.size(); ++i) {
        totalsize *= dims[i];
    }
    if( totalsize == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("u1"));
    }
    PyObject *pyvalues = PyArray_SimpleNew(dims.size(), &dims[0], PyArray_UINT8);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, totalsize*sizeof(uint8_t));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const uint8_t* pvalues, size_t N) {
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("u1"));
    }
    npy_intp dims[] = {npy_intp(N)};
    PyObject *pyvalues = PyArray_SimpleNew(1, &dims[0], PyArray_UINT8);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, N*sizeof(uint8_t));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const int* pvalues, size_t N) {
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("i4"));
    }
    npy_intp dims[] = {npy_intp(N)};
    PyObject *pyvalues = PyArray_SimpleNew(1, &dims[0], PyArray_INT32);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, N*sizeof(int));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline numeric::array toPyArrayN(const uint32_t* pvalues, size_t N) {
    if( N == 0 ) {
        return static_cast<numeric::array>(numeric::array(boost::python::list()).astype("u4"));
    }
    npy_intp dims[] = {npy_intp(N)};
    PyObject *pyvalues = PyArray_SimpleNew(1, &dims[0], PyArray_UINT32);
    if( pvalues != NULL ) {
        memcpy(PyArray_DATA(pyvalues), pvalues, N*sizeof(uint32_t));
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

template <typename T>
inline numeric::array toPyArray(const std::vector<T>& v) {
    if( v.size() == 0 ) {
        return toPyArrayN((T*)NULL, 0);
    }
    return toPyArrayN(&v[0], v.size());
}
////////////////////////////////////////////////////////////////////////////////////////////////////
typedef boost::shared_ptr<Ramp> RampPtr;
typedef boost::shared_ptr<ParabolicCurve> CurvePtr;

class PyRamp {
public:
    PyRamp() {
        pramp.reset(new Ramp());
    }
    PyRamp(Real v0_, Real a_, Real duration_, Real x0_) {
        pramp.reset(new Ramp(v0_, a_, duration_, x0_));
        v0 = v0_;
        a = a_;
        duration = duration_;
        x0 = x0_;
        d = pramp->d;
        v1 = pramp->v1;
    }
    PyRamp(Real v0_, Real a_, Real duration_) {
        pramp.reset(new Ramp(v0_, a_, duration_, 0));
        v0 = v0_;
        a = a_;
        duration = duration_;
        x0 = pramp->x0;
        d = pramp->d;
        v1 = pramp->v1;
    }
    ~PyRamp() {
    }

    void Initialize(Real v0_, Real a_, Real duration_, Real x0_) {
        BOOST_ASSERT(!!pramp);
        pramp->Initialize(v0_, a_, duration_, x0_);
        v0 = v0_;
        a = a_;
        duration = duration_;
        x0 = x0_;
        d = pramp->d;
        v1 = pramp->v1;
    }

    void Initialize(Real v0_, Real a_, Real duration_) {
        BOOST_ASSERT(!!pramp);
        pramp->Initialize(v0_, a_, duration_, 0);
        v0 = v0_;
        a = a_;
        duration = duration_;
        x0 = pramp->x0;
        d = pramp->d;
        v1 = pramp->v1;
    }

    Real EvalPos(Real t) const {
        return pramp->EvalPos(t);
    }

    Real EvalVel(Real t) const {
        return pramp->EvalVel(t);
    }

    Real EvalAcc(Real t) const {
        return pramp->EvalAcc(t);
    }

    void UpdateDuration(Real newDuration) {
        pramp->UpdateDuration(newDuration);
        duration = newDuration;
    }

    // Members
    RampPtr pramp;
    Real v0, v1, a, x0, d, duration;
}; // end class PyRamp

typedef boost::shared_ptr<PyRamp> PyRampPtr;

class PyParabolicCurve {
public:
    PyParabolicCurve() {
        // pcurve = new ParabolicCurve();
        pcurve.reset(new ParabolicCurve());
    }
    PyParabolicCurve(object pyRampsArray) {
        std::vector<PyRampPtr> vpramps = ExtractArray<PyRampPtr>(pyRampsArray);
        Initialize(vpramps);
    }
    ~PyParabolicCurve() {
    }

    // Initialize PyParabolicCurve from an std::vector of PyRampPtrs
    void Initialize(std::vector<PyRampPtr> vpramps) {
        std::vector<Ramp> ramps(vpramps.size());
        for (size_t i = 0; i < vpramps.size(); ++i) {
            ramps[i] = Ramp(vpramps[i]->v0, vpramps[i]->a, vpramps[i]->duration, vpramps[i]->x0);
        }
        pcurve.reset(new ParabolicCurve(ramps));

        v0 = pcurve->ramps.front().v0;
        duration = pcurve->duration;
        x0 = pcurve->ramps.front().x0;
        d = pcurve->d;
        v1 = pcurve->ramps.back().v1;
        switchpointsList.reserve(pcurve->switchpointsList.size());
        switchpointsList = pcurve->switchpointsList;
    }

    void Append(PyParabolicCurve anotherCurve) {
        BOOST_ASSERT(!anotherCurve._IsEmpty());

        switchpointsList.reserve(pcurve->switchpointsList.size() + anotherCurve.switchpointsList.size());
        pcurve->Append(*(anotherCurve.pcurve));
        // Update members
        v1 = pcurve->ramps.back().v1;
        duration = pcurve->duration;
        d = pcurve->d;
        switchpointsList = pcurve->switchpointsList;
    }

    Real EvalPos(Real t) const {
        return pcurve->EvalPos(t);
    }

    Real EvalVel(Real t) const {
        return pcurve->EvalVel(t);
    }

    Real EvalAcc(Real t) const {
        return pcurve->EvalAcc(t);
    }

    bool _IsEmpty() const {
        return pcurve->ramps.size() == 0;
    }

    size_t _GetLength() const {
        return pcurve->ramps.size();
    }

    PyRamp _GetRamp(size_t index) {
        std::vector<Ramp>::const_iterator it = pcurve->ramps.begin() + index;
        PyRamp pyramp(it->v0, it->a, it->duration, it->x0);
        // Check soundness
        BOOST_ASSERT(pyramp.v0 == it->v0);
        BOOST_ASSERT(pyramp.v1 == it->v1);
        BOOST_ASSERT(pyramp.a == it->a);
        BOOST_ASSERT(pyramp.x0 == it->x0);
        BOOST_ASSERT(pyramp.d == it->d);
        BOOST_ASSERT(pyramp.duration == it->duration);
        return pyramp;
    }

    object _GetSwitchpointsList() {
        return toPyArray(switchpointsList);
    }

    // Members
    CurvePtr pcurve;
    Real v0;
    Real duration;
    Real x0;
    Real v1;
    Real d;
    std::vector<Real> switchpointsList;
}; // end class PyParabolicCurve

typedef boost::shared_ptr<PyParabolicCurve> PyCurvePtr;

class PyParabolicCurvesND {
public:
    PyParabolicCurvesND() {

    }
    PyParabolicCurvesND(object pyCurvesArray) {
        std::vector<PyCurvePtr> vpcurves = ExtractArray<PyCurvePtr>(pyCurvesArray);
        Initialize(vpcurves);
    }
    ~PyParabolicCurvesND() {
    }

    // Initialize PyParabolicCurvesND from an std::vector of PyCurvePtrs
    void Initialize(std::vector<PyCurvePtr> vpcurves) {
        ndof = (int) vpcurves.size();
        std::vector<ParabolicCurve> curves(vpcurves.size());
        for (int i = 0; i < ndof; ++i) {
            curves[i] = *(vpcurves[i]->pcurve);
        }
        pcurvesnd.reset(new ParabolicCurvesND(curves));

        duration = pcurvesnd->duration;

        x0Vect = pcurvesnd->x0Vect;
        dVect = pcurvesnd->dVect;
        v0Vect = pcurvesnd->v0Vect;
        v1Vect = pcurvesnd->v1Vect;
        switchpointsList = pcurvesnd->switchpointsList;
    }

    object EvalPos(Real t) {
        std::vector<Real> xVect(ndof);
        for (int i = 0; i < ndof; ++i) {
            xVect[i] = pcurvesnd->curves[i].EvalPos(t);
        }
        return toPyArray(xVect);
    }

    object EvalVel(Real t) {
        std::vector<Real> vVect(ndof);
        for (int i = 0; i < ndof; ++i) {
            vVect[i] = pcurvesnd->curves[i].EvalVel(t);
        }
        return toPyArray(vVect);
    }

    object EvalAcc(Real t) {
        std::vector<Real> aVect(ndof);
        for (int i = 0; i < ndof; ++i) {
            aVect[i] = pcurvesnd->curves[i].EvalAcc(t);
        }
        return toPyArray(aVect);
    }

    object _GetSwitchpointsList() {
        return toPyArray(switchpointsList);
    }

    object _Getx0Vect() {
        return toPyArray(x0Vect);
    }

    object _GetdVect() {
        return toPyArray(dVect);
    }

    object _Getv0Vect() {
        return toPyArray(v0Vect);
    }

    object _Getv1Vect() {
        return toPyArray(v1Vect);
    }

    // Members
    boost::shared_ptr<ParabolicCurvesND> pcurvesnd;
    std::vector<Real> x0Vect;
    std::vector<Real> v0Vect;
    std::vector<Real> v1Vect;
    std::vector<Real> dVect;
    Real duration;
    int ndof;
    std::vector<Real> switchpointsList;

}; //end class PyParabolicCurvesND

} // end namespace rplanners


BOOST_PYTHON_MODULE(openravepy_rplanners) {

    using namespace RampOptimizerInternal;
    using namespace rplanners;
    using namespace boost::python;
    import_array(); // have to put this here in order to export vector as numpy array

    ////////////////////////////////////////////////////////////////////////////////
    // PyRamp
    void (PyRamp::*pyrampinit1)(Real, Real, Real, Real) = &PyRamp::Initialize;
    void (PyRamp::*pyrampinit2)(Real, Real, Real) = &PyRamp::Initialize;

    class_<PyRamp>("PyRamp", init<>())
    .def(init<Real, Real, Real, Real>())
    .def(init<Real, Real, Real>())
    .def_readonly("v0", &PyRamp::v0)
    .def_readonly("a", &PyRamp::a)
    .def_readonly("duration", &PyRamp::duration)
    .def_readonly("x0", &PyRamp::x0)
    .def_readonly("v1", &PyRamp::v1)
    .def_readonly("d", &PyRamp::d)
    .def("Initialize", pyrampinit1)
    .def("Initialize", pyrampinit2)
    .def("EvalPos", &PyRamp::EvalPos)
    .def("EvalVel", &PyRamp::EvalVel)
    .def("EvalAcc", &PyRamp::EvalAcc)
    ;

    ////////////////////////////////////////////////////////////////////////////////
    // PyParabolicCurve
    class_<PyParabolicCurve>("PyParabolicCurve", init<>())
    .def(init<object>())
    .def_readonly("v0", &PyParabolicCurve::v0)
    .def_readonly("duration", &PyParabolicCurve::duration)
    .def_readonly("x0", &PyParabolicCurve::x0)
    .def_readonly("v1", &PyParabolicCurve::v1)
    .def_readonly("d", &PyParabolicCurve::d)
    .add_property("switchpointsList", &PyParabolicCurve::_GetSwitchpointsList)
    .add_property("isEmpty", &PyParabolicCurve::_IsEmpty)
    .def("Initialize", &PyParabolicCurve::Initialize)
    .def("Append", &PyParabolicCurve::Append)
    .def("EvalPos", &PyParabolicCurve::EvalPos)
    .def("EvalVel", &PyParabolicCurve::EvalVel)
    .def("EvalAcc", &PyParabolicCurve::EvalAcc)
    .def("__len__", &PyParabolicCurve::_GetLength)
    .def("__getitem__", &PyParabolicCurve::_GetRamp)
    ;

    ////////////////////////////////////////////////////////////////////////////////
    // PyParabolicCurvesND
    class_<PyParabolicCurvesND>("PyParabolicCurvesND", init<>())
    .def(init<object>())
    .def_readonly("ndof", &PyParabolicCurvesND::ndof)
    .def_readonly("duration", &PyParabolicCurvesND::duration)
    .add_property("switchpointsList", &PyParabolicCurvesND::_GetSwitchpointsList)
    .add_property("x0Vect", &PyParabolicCurvesND::_Getx0Vect)
    .add_property("dVect", &PyParabolicCurvesND::_GetdVect)
    .add_property("v0Vect", &PyParabolicCurvesND::_Getv0Vect)
    .add_property("v1Vect", &PyParabolicCurvesND::_Getv1Vect)
    .def("EvalPos", &PyParabolicCurvesND::EvalPos)
    .def("EvalVel", &PyParabolicCurvesND::EvalVel)
    .def("EvalAcc", &PyParabolicCurvesND::EvalAcc)
    ;

    // ////////////////////////////////////////////////////////////////////////////////
    // // Ramp
    // void (Ramp::*printrampinfo1)() const = &Ramp::PrintInfo;
    // void (Ramp::*printrampinfo2)(std::string) const = &Ramp::PrintInfo;

    // class_<Ramp>("Ramp", init<>())
    // .def(init<Real, Real, Real, Real>())
    // .def_readonly("v0", &Ramp::v0)
    // .def_readonly("a", &Ramp::a)
    // .def_readonly("duration", &Ramp::duration)
    // .def_readonly("x0", &Ramp::x0)
    // .def_readonly("v1", &Ramp::v1)
    // .def_readonly("d", &Ramp::d)
    // .def("EvalPos", &Ramp::EvalPos)
    // .def("EvalVel", &Ramp::EvalVel)
    // .def("EvalAcc", &Ramp::EvalAcc)
    // .def("PrintInfo", printrampinfo1)
    // .def("PrintInfo", printrampinfo2)
    // ;

    // ////////////////////////////////////////////////////////////////////////////////
    // // ParabolicCurve
    // void (ParabolicCurve::*printcurveinfo1)() const = &ParabolicCurve::PrintInfo;
    // void (ParabolicCurve::*printcurveinfo2)(std::string) const = &ParabolicCurve::PrintInfo;

    // class_<ParabolicCurve>("ParabolicCurve", init<>())
    // .def(init<std::vector<Ramp> >())
    // .def_readonly("x0", &ParabolicCurve::x0)
    // .def_readonly("duration", &ParabolicCurve::duration)
    // .def_readonly("d", &ParabolicCurve::d)
    // .def("EvalPos", &ParabolicCurve::EvalPos)
    // .def("EvalVel", &ParabolicCurve::EvalVel)
    // .def("EvalAcc", &ParabolicCurve::EvalAcc)
    // .def("PrintInfo", printcurveinfo1)
    // .def("PrintInfo", printcurveinfo2)
    // ;
} // end BOOST_PYTHON_MODULE rplanners
