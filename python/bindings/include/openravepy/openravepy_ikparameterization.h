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
#ifndef OPENRAVEPY_IKPARAMETERIZATION_H
#define OPENRAVEPY_IKPARAMETERIZATION_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class OPENRAVEPY_API PyIkParameterization
{
public:
    PyIkParameterization();
    PyIkParameterization(const string &s);
    PyIkParameterization(object o, IkParameterizationType type);
    PyIkParameterization(OPENRAVE_SHARED_PTR<PyIkParameterization> pyikparam);
    PyIkParameterization(const IkParameterization &ikparam);
    virtual ~PyIkParameterization();

    IkParameterizationType GetType() const;
    std::string GetId() const;
    py::str GetName() const;

    int GetDOF();

    int GetDOF(object o);

    int GetNumberOfValues();

    int GetNumberOfValues(object o);

    PyConfigurationSpecificationPtr GetConfigurationSpecification();

    PyConfigurationSpecificationPtr GetConfigurationSpecification(object ointerpolation, const std::string& robotname="", const std::string& manipname="");

    static PyConfigurationSpecificationPtr GetConfigurationSpecificationFromType(IkParameterizationType iktype, const std::string& interpolation="", const std::string& robotname="", const std::string& manipname="");
    void SetTransform6D(object o);
    void SetRotation3D(object o);
    void SetTranslation3D(object o);
    void SetDirection3D(object o);
    void SetRay4D(OPENRAVE_SHARED_PTR<PyRay> ray);
    void SetLookat3D(object o);
    void SetTranslationDirection5D(OPENRAVE_SHARED_PTR<PyRay> ray);
    void SetTranslationXY2D(object o);
    void SetTranslationXYOrientation3D(object o);
    void SetTranslationLocalGlobal6D(object olocaltrans, object otrans);
    void SetTranslationXAxisAngle4D(object otrans, dReal angle);
    void SetTranslationYAxisAngle4D(object otrans, dReal angle);
    void SetTranslationZAxisAngle4D(object otrans, dReal angle);
    void SetTranslationXAxisAngleZNorm4D(object otrans, dReal angle);
    void SetTranslationYAxisAngleXNorm4D(object otrans, dReal angle);
    void SetTranslationZAxisAngleYNorm4D(object otrans, dReal angle);

    py::array_t<dReal> GetTransform6D();
    py::array_t<dReal> GetTransform6DPose();
    py::array_t<dReal> GetRotation3D();
    py::array_t<dReal> GetTranslation3D();
    py::array_t<dReal> GetDirection3D();
    PyRay GetRay4D();
    py::array_t<dReal> GetLookat3D();
    PyRay GetTranslationDirection5D();
    py::array_t<dReal> GetTranslationXY2D();
    py::array_t<dReal> GetTranslationXYOrientation3D();
    py::tuple GetTranslationLocalGlobal6D();
    py::tuple GetTranslationXAxisAngle4D();
    py::tuple GetTranslationYAxisAngle4D();
    py::tuple GetTranslationZAxisAngle4D();
    py::tuple GetTranslationXAxisAngleZNorm4D();
    py::tuple GetTranslationYAxisAngleXNorm4D();
    py::tuple GetTranslationZAxisAngleYNorm4D();
    dReal ComputeDistanceSqr(PyIkParameterizationPtr pyikparam);

    PyIkParameterizationPtr Transform(object otrans) const;

    void SetCustomValues(const std::string& name, object ovalues);

    void SetCustomValue(const std::string& name, dReal value);

    object GetCustomValues(const std::string& name);

    object GetCustomDataMap();

    size_t ClearCustomValues(const std::string& name=std::string());

    object GetValues() const;

    void SetValues(object ovalues, IkParameterizationType iktype);

    void MultiplyTransform(object otrans);

    void MultiplyTransformRight(object otrans);

    py::dict SerializeJSON(dReal fUnitScale=1.0);
    void DeserializeJSON(py::object obj, dReal fUnitScale=1.0);

    std::string __repr__();
    std::string __str__();
    py::str __unicode__();

    PyIkParameterizationPtr __mul__(object otrans);

    PyIkParameterizationPtr __rmul__(object otrans);

    IkParameterization _param;

private:
    void _Update(const IkParameterization& ikparam);
};

struct IkParameterizationInitializer
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    IkParameterizationInitializer(py::module& m_);
    void init_openravepy_ikparameterization();
    py::module& m;
#else
    IkParameterizationInitializer();
    void init_openravepy_ikparameterization();
#endif
};

} // namespace openravepy

#endif // OPENRAVEPY_IKPARAMETERIZATION_H
