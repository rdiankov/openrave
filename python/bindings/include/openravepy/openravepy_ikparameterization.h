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

class PyIkParameterization
{
public:
    PyIkParameterization();
    PyIkParameterization(const string &s);
    PyIkParameterization(object o, IkParameterizationType type);
    PyIkParameterization(OPENRAVE_SHARED_PTR<PyIkParameterization> pyikparam);
    PyIkParameterization(const IkParameterization &ikparam);
    virtual ~PyIkParameterization();

    IkParameterizationType GetType();

    int GetDOF();

    int GetDOF(object o);

    int GetNumberOfValues();

    int GetNumberOfValues(object o);

    object GetConfigurationSpecification();

    object GetConfigurationSpecification(object ointerpolation, const std::string& robotname="", const std::string& manipname="");

    static object GetConfigurationSpecificationFromType(IkParameterizationType iktype, const std::string& interpolation="", const std::string& robotname="", const std::string& manipname="");
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

    object GetTransform6D();
    object GetTransform6DPose();
    object GetRotation3D();
    object GetTranslation3D();
    object GetDirection3D();
    PyRay GetRay4D();
    object GetLookat3D();
    PyRay GetTranslationDirection5D();
    object GetTranslationXY2D();
    object GetTranslationXYOrientation3D();
    object GetTranslationLocalGlobal6D();
    object GetTranslationXAxisAngle4D();
    object GetTranslationYAxisAngle4D();
    object GetTranslationZAxisAngle4D();
    object GetTranslationXAxisAngleZNorm4D();
    object GetTranslationYAxisAngleXNorm4D();
    object GetTranslationZAxisAngleYNorm4D();
    dReal ComputeDistanceSqr(OPENRAVE_SHARED_PTR<PyIkParameterization> pyikparam);

    object Transform(object otrans) const;

    void SetCustomValues(const std::string& name, object ovalues);

    void SetCustomValue(const std::string& name, dReal value);

    object GetCustomValues(const std::string& name);

    object GetCustomDataMap();

    size_t ClearCustomValues(const std::string& name=std::string());

    object GetValues() const;

    void SetValues(object ovalues, IkParameterizationType iktype);

    void MultiplyTransform(object otrans);

    void MultiplyTransformRight(object otrans);

    py::object SerializeJSON(dReal fUnitScale=1.0);
    void DeserializeJSON(py::object obj, dReal fUnitScale=1.0);

    std::string __repr__();
    std::string __str__();
    object __unicode__();

    PyIkParameterizationPtr __mul__(object otrans);

    PyIkParameterizationPtr __rmul__(object otrans);

    IkParameterization _param;

private:
    void _Update(const IkParameterization& ikparam);
};

} // namespace openravepy

#endif // OPENRAVEPY_IKPARAMETERIZATION_H
