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
#include "include/openravepy_configurationspecification.h"

#include <openrave/planningutils.h>

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
namespace numeric = py::numeric;
#endif // USE_PYBIND11_PYTHON_BINDINGS

class PyIkParameterization
{
public:
    PyIkParameterization() {
    }
    PyIkParameterization(const string &s) {
        stringstream ss(s);
        ss >> _param;
    }
    PyIkParameterization(object o, IkParameterizationType type)
    {
        switch(type) {
        case IKP_Transform6D: SetTransform6D(o); break;
        case IKP_Rotation3D: SetRotation3D(o); break;
        case IKP_Translation3D: SetTranslation3D(o); break;
        case IKP_Direction3D: SetDirection3D(o); break;
        case IKP_Ray4D: SetRay4D(extract<OPENRAVE_SHARED_PTR<PyRay> >(o)); break;
        case IKP_Lookat3D: SetLookat3D(o); break;
        case IKP_TranslationDirection5D: SetTranslationDirection5D(extract<OPENRAVE_SHARED_PTR<PyRay> >(o)); break;
        case IKP_TranslationXY2D: SetTranslationXY2D(o); break;
        case IKP_TranslationXYOrientation3D: SetTranslationXYOrientation3D(o); break;
        case IKP_TranslationLocalGlobal6D: SetTranslationLocalGlobal6D(o[0],o[1]); break;
        case IKP_TranslationXAxisAngle4D: SetTranslationXAxisAngle4D(o[0],extract<dReal>(o[1])); break;
        case IKP_TranslationYAxisAngle4D: SetTranslationYAxisAngle4D(o[0],extract<dReal>(o[1])); break;
        case IKP_TranslationZAxisAngle4D: SetTranslationZAxisAngle4D(o[0],extract<dReal>(o[1])); break;
        case IKP_TranslationXAxisAngleZNorm4D: SetTranslationXAxisAngleZNorm4D(o[0],extract<dReal>(o[1])); break;
        case IKP_TranslationYAxisAngleXNorm4D: SetTranslationYAxisAngleXNorm4D(o[0],extract<dReal>(o[1])); break;
        case IKP_TranslationZAxisAngleYNorm4D: SetTranslationZAxisAngleYNorm4D(o[0],extract<dReal>(o[1])); break;
        default: throw OPENRAVE_EXCEPTION_FORMAT(_("incorrect ik parameterization type 0x%x"), type, ORE_InvalidArguments);
        }
    }
    PyIkParameterization(OPENRAVE_SHARED_PTR<PyIkParameterization> pyikparam) {
        _param = pyikparam->_param;
    }
    PyIkParameterization(const IkParameterization &ikparam) : _param(ikparam) {
    }
    virtual ~PyIkParameterization() {
    }

    IkParameterizationType GetType() {
        return _param.GetType();
    }

    int GetDOF() {
        return _param.GetDOF();
    }

    int GetDOF(object o) {
        extract_<PyIkParameterization*> pyik(o);
        if( pyik.check() ) {
            return ((PyIkParameterization*)pyik)->_param.GetDOF();
        }
        extract_<OPENRAVE_SHARED_PTR<PyIkParameterization> > pyikptr(o);
        if( pyikptr.check() ) {
            return ((OPENRAVE_SHARED_PTR<PyIkParameterization>)pyikptr)->_param.GetDOF();
        }
        return IkParameterization::GetDOF((IkParameterizationType)extract<IkParameterizationType>(o));
    }

    int GetNumberOfValues() {
        return _param.GetNumberOfValues();
    }

    int GetNumberOfValues(object o) {
        extract_<PyIkParameterization*> pyik(o);
        if( pyik.check() ) {
            return ((PyIkParameterization*)pyik)->_param.GetNumberOfValues();
        }
        extract_<OPENRAVE_SHARED_PTR<PyIkParameterization> > pyikptr(o);
        if( pyikptr.check() ) {
            return ((OPENRAVE_SHARED_PTR<PyIkParameterization>)pyikptr)->_param.GetNumberOfValues();
        }
        return IkParameterization::GetNumberOfValues((IkParameterizationType)extract<IkParameterizationType>(o));
    }

    object GetConfigurationSpecification() {
        return py::to_object(openravepy::toPyConfigurationSpecification(_param.GetConfigurationSpecification()));
    }

    object GetConfigurationSpecification(object ointerpolation, const std::string& robotname="", const std::string& manipname="") {
        extract_<PyIkParameterization*> pyik(ointerpolation);
        if( pyik.check() ) {
            return py::to_object(openravepy::toPyConfigurationSpecification(((PyIkParameterization*)pyik)->_param.GetConfigurationSpecification(std::string(), robotname, manipname)));
        }
        extract_<OPENRAVE_SHARED_PTR<PyIkParameterization> > pyikptr(ointerpolation);
        if( pyikptr.check() ) {
            return py::to_object(openravepy::toPyConfigurationSpecification(((OPENRAVE_SHARED_PTR<PyIkParameterization>)pyikptr)->_param.GetConfigurationSpecification(std::string(), robotname, manipname)));
        }
        extract_<IkParameterizationType> pyiktype(ointerpolation);
        if( pyiktype.check() ) {
            return py::to_object(openravepy::toPyConfigurationSpecification(IkParameterization::GetConfigurationSpecification((IkParameterizationType)pyiktype, std::string(), robotname, manipname)));
        }
        return py::to_object(openravepy::toPyConfigurationSpecification(_param.GetConfigurationSpecification((std::string)extract<std::string>(ointerpolation), robotname, manipname)));
    }

    static object GetConfigurationSpecificationFromType(IkParameterizationType iktype, const std::string& interpolation="", const std::string& robotname="", const std::string& manipname="")
    {
        return py::to_object(openravepy::toPyConfigurationSpecification(IkParameterization::GetConfigurationSpecification(iktype,interpolation, robotname, manipname)));
    }

    void SetTransform6D(object o) {
        _param.SetTransform6D(ExtractTransform(o));
    }
    void SetRotation3D(object o) {
        _param.SetRotation3D(ExtractVector4(o));
    }
    void SetTranslation3D(object o) {
        _param.SetTranslation3D(ExtractVector3(o));
    }
    void SetDirection3D(object o) {
        _param.SetDirection3D(ExtractVector3(o));
    }
    void SetRay4D(OPENRAVE_SHARED_PTR<PyRay> ray) {
        _param.SetRay4D(ray->r);
    }
    void SetLookat3D(object o) {
        _param.SetLookat3D(ExtractVector3(o));
    }
    void SetTranslationDirection5D(OPENRAVE_SHARED_PTR<PyRay> ray) {
        _param.SetTranslationDirection5D(ray->r);
    }
    void SetTranslationXY2D(object o) {
        _param.SetTranslationXY2D(ExtractVector2(o));
    }
    void SetTranslationXYOrientation3D(object o) {
        _param.SetTranslationXYOrientation3D(ExtractVector3(o));
    }
    void SetTranslationLocalGlobal6D(object olocaltrans, object otrans) {
        _param.SetTranslationLocalGlobal6D(ExtractVector3(olocaltrans),ExtractVector3(otrans));
    }
    void SetTranslationXAxisAngle4D(object otrans, dReal angle) {
        _param.SetTranslationXAxisAngle4D(ExtractVector3(otrans),angle);
    }
    void SetTranslationYAxisAngle4D(object otrans, dReal angle) {
        _param.SetTranslationYAxisAngle4D(ExtractVector3(otrans),angle);
    }
    void SetTranslationZAxisAngle4D(object otrans, dReal angle) {
        _param.SetTranslationZAxisAngle4D(ExtractVector3(otrans),angle);
    }
    void SetTranslationXAxisAngleZNorm4D(object otrans, dReal angle) {
        _param.SetTranslationXAxisAngleZNorm4D(ExtractVector3(otrans),angle);
    }
    void SetTranslationYAxisAngleXNorm4D(object otrans, dReal angle) {
        _param.SetTranslationYAxisAngleXNorm4D(ExtractVector3(otrans),angle);
    }
    void SetTranslationZAxisAngleYNorm4D(object otrans, dReal angle) {
        _param.SetTranslationZAxisAngleYNorm4D(ExtractVector3(otrans),angle);
    }

    object GetTransform6D() {
        return ReturnTransform(_param.GetTransform6D());
    }
    object GetTransform6DPose() {
        return toPyArray(_param.GetTransform6D());
    }
    object GetRotation3D() {
        return toPyVector4(_param.GetRotation3D());
    }
    object GetTranslation3D() {
        return toPyVector3(_param.GetTranslation3D());
    }
    object GetDirection3D() {
        return toPyVector3(_param.GetDirection3D());
    }
    PyRay GetRay4D() {
        return PyRay(_param.GetRay4D());
    }
    object GetLookat3D() {
        return toPyVector3(_param.GetLookat3D());
    }
    PyRay GetTranslationDirection5D() {
        return PyRay(_param.GetTranslationDirection5D());
    }
    object GetTranslationXY2D() {
        return toPyVector2(_param.GetTranslationXY2D());
    }
    object GetTranslationXYOrientation3D() {
        return toPyVector3(_param.GetTranslationXYOrientation3D());
    }
    object GetTranslationLocalGlobal6D() {
        return py::make_tuple(toPyVector3(_param.GetTranslationLocalGlobal6D().first),toPyVector3(_param.GetTranslationLocalGlobal6D().second));
    }
    object GetTranslationXAxisAngle4D() {
        std::pair<Vector,dReal> p = _param.GetTranslationXAxisAngle4D();
        return py::make_tuple(toPyVector3(p.first), py::to_object(p.second));
    }
    object GetTranslationYAxisAngle4D() {
        std::pair<Vector,dReal> p = _param.GetTranslationYAxisAngle4D();
        return py::make_tuple(toPyVector3(p.first), py::to_object(p.second));
    }
    object GetTranslationZAxisAngle4D() {
        std::pair<Vector,dReal> p = _param.GetTranslationZAxisAngle4D();
        return py::make_tuple(toPyVector3(p.first), py::to_object(p.second));
    }
    object GetTranslationXAxisAngleZNorm4D() {
        std::pair<Vector,dReal> p = _param.GetTranslationXAxisAngleZNorm4D();
        return py::make_tuple(toPyVector3(p.first), py::to_object(p.second));
    }
    object GetTranslationYAxisAngleXNorm4D() {
        std::pair<Vector,dReal> p = _param.GetTranslationYAxisAngleXNorm4D();
        return py::make_tuple(toPyVector3(p.first), py::to_object(p.second));
    }
    object GetTranslationZAxisAngleYNorm4D() {
        std::pair<Vector,dReal> p = _param.GetTranslationZAxisAngleYNorm4D();
        return py::make_tuple(toPyVector3(p.first), py::to_object(p.second));
    }
    dReal ComputeDistanceSqr(OPENRAVE_SHARED_PTR<PyIkParameterization> pyikparam)
    {
        return _param.ComputeDistanceSqr(pyikparam->_param);
    }

    object Transform(object otrans) const
    {
        return toPyIkParameterization(ExtractTransform(otrans) * _param);
    }

    void SetCustomValues(const std::string& name, object ovalues)
    {
        _param.SetCustomValues(name,ExtractArray<dReal>(ovalues));
    }

    void SetCustomValue(const std::string& name, dReal value)
    {
        _param.SetCustomValue(name,value);
    }

    object GetCustomValues(const std::string& name)
    {
        std::vector<dReal> values;
        if( _param.GetCustomValues(name,values) ) {
            return toPyArray(values);
        }
        return py::object();
    }

    object GetCustomDataMap()
    {
        py::dict odata;
        FOREACHC(it, _param.GetCustomDataMap()) {
            odata[it->first] = toPyArray(it->second);
        }
        return odata;
    }

    size_t ClearCustomValues(const std::string& name=std::string())
    {
        return _param.ClearCustomValues(name);
    }

    object GetValues() const
    {
        vector<dReal> values(_param.GetNumberOfValues());
        _param.GetValues(values.begin());
        return toPyArray(values);
    }

    void SetValues(object ovalues, IkParameterizationType iktype)
    {
        vector<dReal> vsetvalues = ExtractArray<dReal>(ovalues);
        _param.Set(vsetvalues.begin(),iktype);
    }

    void MultiplyTransform(object otrans)
    {
        _param.MultiplyTransform(ExtractTransform(otrans));
    }

    void MultiplyTransformRight(object otrans)
    {
        _param.MultiplyTransformRight(ExtractTransform(otrans));
    }

    string __repr__() {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        ss << _param;
        return boost::str(boost::format("IkParameterization('%s')")%ss.str());
    }
    string __str__() {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        ss << _param;
        return ss.str();
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    PyIkParameterizationPtr __mul__(object otrans)
    {
        return PyIkParameterizationPtr(new PyIkParameterization(_param * ExtractTransform(otrans)));
    }

    PyIkParameterizationPtr __rmul__(object otrans)
    {
        return PyIkParameterizationPtr(new PyIkParameterization(ExtractTransform(otrans) * _param));
    }

    IkParameterization _param;
};

bool ExtractIkParameterization(object o, IkParameterization& ikparam) {
    extract_<PyIkParameterizationPtr > pyikparam(o);
    if( pyikparam.check() ) {
        ikparam = ((PyIkParameterizationPtr)pyikparam)->_param;
        return true;
    }
    return false;
}


object toPyIkParameterization(const IkParameterization &ikparam)
{
    return py::to_object(PyIkParameterizationPtr(new PyIkParameterization(ikparam)));
}

object toPyIkParameterization(const std::string& serializeddata)
{
    return py::to_object(PyIkParameterizationPtr(new PyIkParameterization(serializeddata)));
}

#ifndef USE_PYBIND11_PYTHON_BINDINGS
class IkParameterization_pickle_suite : public pickle_suite
{
public:
    static py::tuple getinitargs(const PyIkParameterization &r)
    {
        std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        ss << r._param;
        return py::make_tuple(ss.str());
    }
};
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_FUNCTION_OVERLOADS(GetConfigurationSpecificationFromType_overloads, PyIkParameterization::GetConfigurationSpecificationFromType, 1, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetConfigurationSpecification_overloads, GetConfigurationSpecification, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ClearCustomValues_overloads, ClearCustomValues, 0, 1)
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_ikparameterization(py::module& m)
#else
void init_openravepy_ikparameterization()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals;  // "..."_a
    object iktype = enum_<IkParameterizationType>(m, "IkParameterizationType" DOXY_ENUM(IkParameterizationType))
#else
    object iktype = enum_<IkParameterizationType>("IkParameterizationType" DOXY_ENUM(IkParameterizationType))
#endif
                    .value("Transform6D",IKP_Transform6D)
                    .value("Rotation3D",IKP_Rotation3D)
                    .value("Translation3D",IKP_Translation3D)
                    .value("Direction3D",IKP_Direction3D)
                    .value("Ray4D",IKP_Ray4D)
                    .value("Lookat3D",IKP_Lookat3D)
                    .value("TranslationDirection5D",IKP_TranslationDirection5D)
                    .value("TranslationXY2D",IKP_TranslationXY2D)
                    .value("TranslationXYOrientation3D",IKP_TranslationXYOrientation3D)
                    .value("TranslationLocalGlobal6D",IKP_TranslationLocalGlobal6D)
                    .value("TranslationXAxisAngle4D",IKP_TranslationXAxisAngle4D)
                    .value("TranslationYAxisAngle4D",IKP_TranslationYAxisAngle4D)
                    .value("TranslationZAxisAngle4D",IKP_TranslationZAxisAngle4D)
                    .value("TranslationXAxisAngleZNorm4D",IKP_TranslationXAxisAngleZNorm4D)
                    .value("TranslationYAxisAngleXNorm4D",IKP_TranslationYAxisAngleXNorm4D)
                    .value("TranslationZAxisAngleYNorm4D",IKP_TranslationZAxisAngleYNorm4D)
                    // velocity
                    .value("VelocityDataBit",IKP_VelocityDataBit)
                    .value("Transform6DVelocity",IKP_Transform6DVelocity)
                    .value("Rotation3DVelocity",IKP_Rotation3DVelocity)
                    .value("Translation3DVelocity",IKP_Translation3DVelocity)
                    .value("Direction3DVelocity",IKP_Direction3DVelocity)
                    .value("Ray4DVelocity",IKP_Ray4DVelocity)
                    .value("Lookat3DVelocity",IKP_Lookat3DVelocity)
                    .value("TranslationDirection5DVelocity",IKP_TranslationDirection5DVelocity)
                    .value("TranslationXY2DVelocity",IKP_TranslationXY2DVelocity)
                    .value("TranslationXYOrientation3DVelocity",IKP_TranslationXYOrientation3DVelocity)
                    .value("TranslationLocalGlobal6DVelocity",IKP_TranslationLocalGlobal6DVelocity)
                    .value("TranslationXAxisAngle4DVelocity",IKP_TranslationXAxisAngle4DVelocity)
                    .value("TranslationYAxisAngle4DVelocity",IKP_TranslationYAxisAngle4DVelocity)
                    .value("TranslationZAxisAngle4DVelocity",IKP_TranslationZAxisAngle4DVelocity)
                    .value("TranslationXAxisAngleZNorm4DVelocity",IKP_TranslationXAxisAngleZNorm4DVelocity)
                    .value("TranslationYAxisAngleXNorm4DVelocity",IKP_TranslationYAxisAngleXNorm4DVelocity)
                    .value("TranslationZAxisAngleYNorm4DVelocity",IKP_TranslationZAxisAngleYNorm4DVelocity)
                    // other
                    .value("UniqueIdMask",IKP_UniqueIdMask)
                    .value("CustomDataBit",IKP_CustomDataBit)
    ;

    {
        int (PyIkParameterization::*getdof1)() = &PyIkParameterization::GetDOF;
        int (PyIkParameterization::*getdof2)(object) = &PyIkParameterization::GetDOF;
        int (*getdofstatic)(IkParameterizationType) = IkParameterization::GetDOF;
        int (PyIkParameterization::*getnumberofvalues1)() = &PyIkParameterization::GetNumberOfValues;
        int (PyIkParameterization::*getnumberofvalues2)(object) = &PyIkParameterization::GetNumberOfValues;
        int (*getnumberofvaluesstatic)(IkParameterizationType) = IkParameterization::GetNumberOfValues;
        object (PyIkParameterization::*GetConfigurationSpecification1)() = &PyIkParameterization::GetConfigurationSpecification;
        object (PyIkParameterization::*GetConfigurationSpecification2)(object, const std::string&, const std::string&) = &PyIkParameterization::GetConfigurationSpecification;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        scope_ ikparameterization = class_<PyIkParameterization, PyIkParameterizationPtr >(m, "IkParameterization", DOXY_CLASS(IkParameterization))
                                   .def(init<object, IkParameterizationType>(), "primitive"_a, "type"_a)
                                   .def(init<string>(), "str"_a)
                                   .def(init<OPENRAVE_SHARED_PTR<PyIkParameterization>>(), "ikparam"_a)
#else
        scope_ ikparameterization = class_<PyIkParameterization, PyIkParameterizationPtr >("IkParameterization", DOXY_CLASS(IkParameterization))
                                   .def(init<object,IkParameterizationType>(py::args("primitive","type")))
                                   .def(init<string>(py::args("str")))
                                   .def(init<OPENRAVE_SHARED_PTR<PyIkParameterization> >(py::args("ikparam")))
#endif
                                   .def("GetType",&PyIkParameterization::GetType, DOXY_FN(IkParameterization,GetType))
                                   .def("SetTransform6D",&PyIkParameterization::SetTransform6D, PY_ARGS("transform") DOXY_FN(IkParameterization,SetTransform6D))
                                   .def("SetRotation3D",&PyIkParameterization::SetRotation3D, PY_ARGS("quat") DOXY_FN(IkParameterization,SetRotation3D))
                                   .def("SetTranslation3D",&PyIkParameterization::SetTranslation3D, PY_ARGS("pos") DOXY_FN(IkParameterization,SetTranslation3D))
                                   .def("SetDirection3D",&PyIkParameterization::SetDirection3D, PY_ARGS("dir") DOXY_FN(IkParameterization,SetDirection3D))
                                   .def("SetRay4D",&PyIkParameterization::SetRay4D, PY_ARGS("quat") DOXY_FN(IkParameterization,SetRay4D))
                                   .def("SetLookat3D",&PyIkParameterization::SetLookat3D, PY_ARGS("pos") DOXY_FN(IkParameterization,SetLookat3D))
                                   .def("SetTranslationDirection5D",&PyIkParameterization::SetTranslationDirection5D, PY_ARGS("quat") DOXY_FN(IkParameterization,SetTranslationDirection5D))
                                   .def("SetTranslationXY2D",&PyIkParameterization::SetTranslationXY2D, PY_ARGS("pos") DOXY_FN(IkParameterization,SetTranslationXY2D))
                                   .def("SetTranslationXYOrientation3D",&PyIkParameterization::SetTranslationXYOrientation3D, PY_ARGS("posangle") DOXY_FN(IkParameterization,SetTranslationXYOrientation3D))
                                   .def("SetTranslationLocalGlobal6D",&PyIkParameterization::SetTranslationLocalGlobal6D, PY_ARGS("localpos","pos") DOXY_FN(IkParameterization,SetTranslationLocalGlobal6D))
                                   .def("SetTranslationXAxisAngle4D",&PyIkParameterization::SetTranslationXAxisAngle4D, PY_ARGS("translation","angle") DOXY_FN(IkParameterization,SetTranslationXAxisAngle4D))
                                   .def("SetTranslationYAxisAngle4D",&PyIkParameterization::SetTranslationYAxisAngle4D, PY_ARGS("translation","angle") DOXY_FN(IkParameterization,SetTranslationYAxisAngle4D))
                                   .def("SetTranslationZAxisAngle4D",&PyIkParameterization::SetTranslationZAxisAngle4D, PY_ARGS("translation","angle") DOXY_FN(IkParameterization,SetTranslationZAxisAngle4D))
                                   .def("SetTranslationXAxisAngleZNorm4D",&PyIkParameterization::SetTranslationXAxisAngleZNorm4D, PY_ARGS("translation","angle") DOXY_FN(IkParameterization,SetTranslationXAxisAngleZNorm4D))
                                   .def("SetTranslationYAxisAngleXNorm4D",&PyIkParameterization::SetTranslationYAxisAngleXNorm4D, PY_ARGS("translation","angle") DOXY_FN(IkParameterization,SetTranslationYAxisAngleXNorm4D))
                                   .def("SetTranslationZAxisAngleYNorm4D",&PyIkParameterization::SetTranslationZAxisAngleYNorm4D, PY_ARGS("translation","angle") DOXY_FN(IkParameterization,SetTranslationZAxisAngleYNorm4D))
                                   .def("GetTransform6D",&PyIkParameterization::GetTransform6D, DOXY_FN(IkParameterization,GetTransform6D))
                                   .def("GetTransform6DPose",&PyIkParameterization::GetTransform6DPose, DOXY_FN(IkParameterization,GetTransform6D))
                                   .def("GetRotation3D",&PyIkParameterization::GetRotation3D, DOXY_FN(IkParameterization,GetRotation3D))
                                   .def("GetTranslation3D",&PyIkParameterization::GetTranslation3D, DOXY_FN(IkParameterization,GetTranslation3D))
                                   .def("GetDirection3D",&PyIkParameterization::GetDirection3D, DOXY_FN(IkParameterization,GetDirection3D))
                                   .def("GetRay4D",&PyIkParameterization::GetRay4D, DOXY_FN(IkParameterization,GetRay4D))
                                   .def("GetLookat3D",&PyIkParameterization::GetLookat3D, DOXY_FN(IkParameterization,GetLookat3D))
                                   .def("GetTranslationDirection5D",&PyIkParameterization::GetTranslationDirection5D, DOXY_FN(IkParameterization,GetTranslationDirection5D))
                                   .def("GetTranslationXY2D",&PyIkParameterization::GetTranslationXY2D, DOXY_FN(IkParameterization,GetTranslationXY2D))
                                   .def("GetTranslationXYOrientation3D",&PyIkParameterization::GetTranslationXYOrientation3D, DOXY_FN(IkParameterization,GetTranslationXYOrientation3D))
                                   .def("GetTranslationLocalGlobal6D",&PyIkParameterization::GetTranslationLocalGlobal6D, DOXY_FN(IkParameterization,GetTranslationLocalGlobal6D))
                                   .def("GetTranslationXAxisAngle4D",&PyIkParameterization::GetTranslationXAxisAngle4D, DOXY_FN(IkParameterization,GetTranslationXAxisAngle4D))
                                   .def("GetTranslationYAxisAngle4D",&PyIkParameterization::GetTranslationYAxisAngle4D, DOXY_FN(IkParameterization,GetTranslationYAxisAngle4D))
                                   .def("GetTranslationZAxisAngle4D",&PyIkParameterization::GetTranslationZAxisAngle4D, DOXY_FN(IkParameterization,GetTranslationZAxisAngle4D))
                                   .def("GetTranslationXAxisAngleZNorm4D",&PyIkParameterization::GetTranslationXAxisAngleZNorm4D, DOXY_FN(IkParameterization,GetTranslationXAxisAngleZNorm4D))
                                   .def("GetTranslationYAxisAngleXNorm4D",&PyIkParameterization::GetTranslationYAxisAngleXNorm4D, DOXY_FN(IkParameterization,GetTranslationYAxisAngleXNorm4D))
                                   .def("GetTranslationZAxisAngleYNorm4D",&PyIkParameterization::GetTranslationZAxisAngleYNorm4D, DOXY_FN(IkParameterization,GetTranslationZAxisAngleYNorm4D))
                                   .def("GetDOF", getdof1, DOXY_FN(IkParameterization,GetDOF))
                                   .def("GetDOF", getdof2, PY_ARGS("type") DOXY_FN(IkParameterization,GetDOF))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                                   .def_static("GetDOFFromType", getdofstatic, PY_ARGS("type") DOXY_FN(IkParameterization,GetDOF))
#else
                                   .def("GetDOFFromType", getdofstatic, PY_ARGS("type") DOXY_FN(IkParameterization,GetDOF))
                                   .staticmethod("GetDOFFromType")
#endif
                                   .def("GetNumberOfValues", getnumberofvalues1, DOXY_FN(IkParameterization,GetNumberOfValues))
                                   .def("GetNumberOfValues", getnumberofvalues2, PY_ARGS("type") DOXY_FN(IkParameterization,GetNumberOfValues))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                                   .def_static("GetNumberOfValuesFromType", getnumberofvaluesstatic, PY_ARGS("type") DOXY_FN(IkParameterization,GetNumberOfValues))
#else
                                   .def("GetNumberOfValuesFromType", getnumberofvaluesstatic, PY_ARGS("type") DOXY_FN(IkParameterization,GetNumberOfValues))
                                   .staticmethod("GetNumberOfValuesFromType")
#endif
                                   .def("GetConfigurationSpecification", GetConfigurationSpecification1, DOXY_FN(IkParameterization,GetConfigurationSpecification))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                                   .def("GetConfigurationSpecification", GetConfigurationSpecification2,
                                    "interpolation"_a,
                                    "robotname"_a = "",
                                    "manipname"_a = "",
                                    DOXY_FN(IkParameterization,GetConfigurationSpecification)
                                    )
#else
                                   .def("GetConfigurationSpecification", GetConfigurationSpecification2, GetConfigurationSpecification_overloads(PY_ARGS("interpolation", "robotname", "manipname") DOXY_FN(IkParameterization,GetConfigurationSpecification)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                                   .def_static("GetConfigurationSpecificationFromType", PyIkParameterization::GetConfigurationSpecificationFromType,
                                    "type"_a,
                                    "interpolation"_a = "",
                                    "robotname"_a = "",
                                    "manipname"_a = "",
                                    DOXY_FN(IkParameterization,GetConfigurationSpecification)
                                    )
#else
                                   .def("GetConfigurationSpecificationFromType", PyIkParameterization::GetConfigurationSpecificationFromType, GetConfigurationSpecificationFromType_overloads(PY_ARGS("type","interpolation","robotname","manipname") DOXY_FN(IkParameterization,GetConfigurationSpecification)))
                                   .staticmethod("GetConfigurationSpecificationFromType")
#endif
                                   .def("ComputeDistanceSqr",&PyIkParameterization::ComputeDistanceSqr,DOXY_FN(IkParameterization,ComputeDistanceSqr))
                                   .def("Transform",&PyIkParameterization::Transform,"Returns a new parameterization with transformed by the transformation T (T * ik)")
                                   .def("MultiplyTransform",&PyIkParameterization::MultiplyTransform,DOXY_FN(IkParameterization,MultiplyTransform))
                                   .def("MultiplyTransformRight",&PyIkParameterization::MultiplyTransformRight,DOXY_FN(IkParameterization,MultiplyTransformRight))
                                   .def("GetValues",&PyIkParameterization::GetValues, DOXY_FN(IkParameterization,GetValues))
                                   .def("SetValues",&PyIkParameterization::SetValues, PY_ARGS("values","type") DOXY_FN(IkParameterization,SetValues))
                                   .def("GetCustomDataMap",&PyIkParameterization::GetCustomDataMap, DOXY_FN(IkParameterization,GetCustomDataMap))
                                   .def("GetCustomValues",&PyIkParameterization::GetCustomValues, PY_ARGS("name") DOXY_FN(IkParameterization,GetCustomValues))
                                   .def("SetCustomValues",&PyIkParameterization::SetCustomValues, PY_ARGS("name","values") DOXY_FN(IkParameterization,SetCustomValues))
                                   .def("SetCustomValue",&PyIkParameterization::SetCustomValue, PY_ARGS("name","value") DOXY_FN(IkParameterization,SetCustomValue))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                                   .def("ClearCustomValues", &PyIkParameterization::ClearCustomValues,
                                        "name"_a = "",
                                        DOXY_FN(IkParameterization, ClearCustomValues)
                                    )
#else
                                   .def("ClearCustomValues",&PyIkParameterization::ClearCustomValues,ClearCustomValues_overloads(PY_ARGS("name") DOXY_FN(IkParameterization,ClearCustomValues)))
#endif
                                   .def("__str__",&PyIkParameterization::__str__)
                                   .def("__unicode__",&PyIkParameterization::__unicode__)
                                   .def("__repr__",&PyIkParameterization::__repr__)
                                   .def("__mul__",&PyIkParameterization::__mul__)
                                   .def("__rmul__",&PyIkParameterization::__rmul__)
#ifndef USE_PYBIND11_PYTHON_BINDINGS
                                   .def_pickle(IkParameterization_pickle_suite())
#endif
        ;
        ikparameterization.attr("Type") = iktype;
    }
}

} // end namespace openravepy
