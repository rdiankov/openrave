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
#include "openravepy_int.h"

class PySpaceSamplerBase : public PyInterfaceBase
{
protected:
    SpaceSamplerBasePtr _pspacesampler;
public:
    PySpaceSamplerBase(SpaceSamplerBasePtr pspacesampler, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pspacesampler, pyenv), _pspacesampler(pspacesampler) {
    }
    virtual ~PySpaceSamplerBase() {
    }

    SpaceSamplerBasePtr GetSpaceSampler() {
        return _pspacesampler;
    }

    void SetSeed(uint32_t seed) {
        _pspacesampler->SetSeed(seed);
    }
    void SetSpaceDOF(int dof) {
        _pspacesampler->SetSpaceDOF(dof);
    }
    int GetDOF() {
        return _pspacesampler->GetDOF();
    }
    int GetNumberOfValues() {
        return _pspacesampler->GetNumberOfValues();
    }

    bool Supports(SampleDataType type) {
        return _pspacesampler->Supports(type);
    }

    object GetLimits(SampleDataType type)
    {
        if( type == SDT_Real ) {
            vector<dReal> vlower, vupper;
            _pspacesampler->GetLimits(vlower,vupper);
            return boost::python::make_tuple(toPyArray(vlower),toPyArray(vupper));
        }
        else if( type == SDT_Uint32 ) {
            vector<uint32_t> vlower, vupper;
            _pspacesampler->GetLimits(vlower,vupper);
            return boost::python::make_tuple(toPyArray(vlower),toPyArray(vupper));
        }
        throw OPENRAVE_EXCEPTION_FORMAT("%d sampling type not supported",type,ORE_InvalidArguments);
    }

    object SampleSequence(SampleDataType type, size_t num,IntervalType interval)
    {
        if( type == SDT_Real ) {
            std::vector<dReal> samples;
            _pspacesampler->SampleSequence(samples,num,interval);
            return _ReturnSamples(samples);
        }
        else if( type == SDT_Uint32 ) {
            std::vector<uint32_t> samples;
            _pspacesampler->SampleSequence(samples,num);
            return _ReturnSamples(samples);
        }
        throw OPENRAVE_EXCEPTION_FORMAT("%d sampling type not supported",type,ORE_InvalidArguments);
    }

    object SampleSequence(SampleDataType type, size_t num)
    {
        if( type == SDT_Real ) {
            std::vector<dReal> samples;
            _pspacesampler->SampleSequence(samples,num);
            return _ReturnSamples(samples);
        }
        else if( type == SDT_Uint32 ) {
            std::vector<uint32_t> samples;
            _pspacesampler->SampleSequence(samples,num);
            return _ReturnSamples(samples);
        }
        throw OPENRAVE_EXCEPTION_FORMAT("%d sampling type not supported",type,ORE_InvalidArguments);
    }

    object SampleComplete(SampleDataType type, size_t num,IntervalType interval)
    {
        if( type == SDT_Real ) {
            std::vector<dReal> samples;
            _pspacesampler->SampleComplete(samples,num,interval);
            return _ReturnSamples(samples);
        }
        else if( type == SDT_Uint32 ) {
            std::vector<uint32_t> samples;
            _pspacesampler->SampleComplete(samples,num);
            return _ReturnSamples(samples);
        }
        throw OPENRAVE_EXCEPTION_FORMAT("%d sampling type not supported",type,ORE_InvalidArguments);
    }

    object SampleComplete(SampleDataType type, size_t num)
    {
        if( type == SDT_Real ) {
            std::vector<dReal> samples;
            _pspacesampler->SampleComplete(samples,num);
            return _ReturnSamples(samples);
        }
        else if( type == SDT_Uint32 ) {
            std::vector<uint32_t> samples;
            _pspacesampler->SampleComplete(samples,num);
            return _ReturnSamples(samples);
        }
        throw OPENRAVE_EXCEPTION_FORMAT("%d sampling type not supported",type,ORE_InvalidArguments);
    }

protected:
    object _ReturnSamples(const std::vector<dReal>&samples)
    {
        int dim = _pspacesampler->GetNumberOfValues();
        npy_intp dims[] = { samples.size()/dim,dim};
        PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        memcpy(PyArray_DATA(pyvalues),&samples.at(0),samples.size()*sizeof(samples[0]));
        return static_cast<numeric::array>(handle<>(pyvalues));
    }

    object _ReturnSamples(const std::vector<uint32_t>&samples)
    {
        int dim = _pspacesampler->GetNumberOfValues();
        npy_intp dims[] = { samples.size()/dim,dim};
        PyObject *pyvalues = PyArray_SimpleNew(2,dims, PyArray_UINT32);
        memcpy(PyArray_DATA(pyvalues),&samples.at(0),samples.size()*sizeof(samples[0]));
        return static_cast<numeric::array>(handle<>(pyvalues));
    }
};

namespace openravepy {

SpaceSamplerBasePtr GetSpaceSampler(PySpaceSamplerBasePtr pyspacesampler)
{
    return !pyspacesampler ? SpaceSamplerBasePtr() : pyspacesampler->GetSpaceSampler();
}

PyInterfaceBasePtr toPySpaceSampler(SpaceSamplerBasePtr pspacesampler, PyEnvironmentBasePtr pyenv)
{
    return !pspacesampler ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PySpaceSamplerBase(pspacesampler,pyenv));
}

PySpaceSamplerBasePtr RaveCreateSpaceSampler(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    SpaceSamplerBasePtr p = OpenRAVE::RaveCreateSpaceSampler(GetEnvironment(pyenv), name);
    if( !p ) {
        return PySpaceSamplerBasePtr();
    }
    return PySpaceSamplerBasePtr(new PySpaceSamplerBase(p,pyenv));
}

void init_openravepy_spacesampler()
{
    {
        object (PySpaceSamplerBase::*SampleSequence1)(SampleDataType,size_t,IntervalType) = &PySpaceSamplerBase::SampleSequence;
        object (PySpaceSamplerBase::*SampleSequence2)(SampleDataType,size_t) = &PySpaceSamplerBase::SampleSequence;
        object (PySpaceSamplerBase::*SampleComplete1)(SampleDataType,size_t,IntervalType) = &PySpaceSamplerBase::SampleComplete;
        object (PySpaceSamplerBase::*SampleComplete2)(SampleDataType,size_t) = &PySpaceSamplerBase::SampleComplete;
        scope spacesampler = class_<PySpaceSamplerBase, boost::shared_ptr<PySpaceSamplerBase>, bases<PyInterfaceBase> >("SpaceSampler", DOXY_CLASS(SpaceSamplerBase), no_init)
                             .def("SetSeed",&PySpaceSamplerBase::SetSeed, args("seed"), DOXY_FN(SpaceSamplerBase,SetSeed))
                             .def("SetSpaceDOF",&PySpaceSamplerBase::SetSpaceDOF, args("dof"), DOXY_FN(SpaceSamplerBase,SetSpaceDOF))
                             .def("GetDOF",&PySpaceSamplerBase::GetDOF, DOXY_FN(SpaceSamplerBase,GetDOF))
                             .def("GetNumberOfValues",&PySpaceSamplerBase::GetNumberOfValues, args("seed"), DOXY_FN(SpaceSamplerBase,GetNumberOfValues))
                             .def("Supports",&PySpaceSamplerBase::Supports, args("seed"), DOXY_FN(SpaceSamplerBase,Supports))
                             .def("GetLimits",&PySpaceSamplerBase::GetLimits, args("seed"), DOXY_FN(SpaceSamplerBase,GetLimits))
                             .def("SampleSequence",SampleSequence1, args("num","interval"), DOXY_FN(SpaceSamplerBase,SampleSequence "std::vector<dReal>; size_t; IntervalType"))
                             .def("SampleSequence",SampleSequence2, args("num"), DOXY_FN(SpaceSamplerBase,SampleSequence "std::vector<uint32_t>; size_t"))
                             .def("SampleComplete",SampleComplete1, args("num","interval"), DOXY_FN(SpaceSamplerBase,SampleComplete "std::vector<dReal>; size_t; IntervalType"))
                             .def("SampleComplete",SampleComplete2, args("num"), DOXY_FN(SpaceSamplerBase,SampleComplete "std::vector<uint32_t>; size_t"))
        ;
    }

    def("RaveCreateSpaceSampler",openravepy::RaveCreateSpaceSampler,args("env","name"),DOXY_FN1(RaveCreateSpaceSampler));
}

}
