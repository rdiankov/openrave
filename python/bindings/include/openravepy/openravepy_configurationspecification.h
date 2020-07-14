// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVEPY_INTERNAL_CONFIGURATIONSPECIFICATION_H
#define OPENRAVEPY_INTERNAL_CONFIGURATIONSPECIFICATION_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class PyConfigurationSpecification : public OPENRAVE_ENABLE_SHARED_FROM_THIS<PyConfigurationSpecification>
{
public:
    PyConfigurationSpecification();
    PyConfigurationSpecification(const std::string &s);
    PyConfigurationSpecification(const ConfigurationSpecification& spec);
    PyConfigurationSpecification(const ConfigurationSpecification::Group& g);
    PyConfigurationSpecification(PyConfigurationSpecificationPtr pyspec);
    virtual ~PyConfigurationSpecification();

    void DeserializeJSON(py::object obj);
    py::object SerializeJSON();

    int GetDOF() const;

    bool IsValid() const;

    const ConfigurationSpecification::Group& GetGroupFromName(const std::string& name);

    object FindCompatibleGroup(const std::string& name, bool exactmatch) const;

    object FindTimeDerivativeGroup(const std::string& name, bool exactmatch) const;

//    ConfigurationSpecification GetTimeDerivativeSpecification(int timederivative) const;

    void ResetGroupOffsets();

    void AddVelocityGroups(bool adddeltatime);

    void AddDerivativeGroups(int deriv, bool adddeltatime);

    int AddDeltaTimeGroup();

    int AddGroup(const std::string& name, int dof, const std::string& interpolation);

    int AddGroup(const ConfigurationSpecification::Group& g);

    int RemoveGroups(const std::string& groupname, bool exactmatch=true);

    PyConfigurationSpecificationPtr ConvertToVelocitySpecification() const;

    PyConfigurationSpecificationPtr ConvertToDerivativeSpecification(uint32_t timederivative) const;

    PyConfigurationSpecificationPtr GetTimeDerivativeSpecification(int timederivative) const;

    object ExtractTransform(object otransform, object odata, PyKinBodyPtr pybody, int timederivative=0) const;

    object ExtractIkParameterization(object odata, int timederivative=0, const std::string& robotname="", const std::string& manipulatorname="") const;

    object ExtractAffineValues(object odata, PyKinBodyPtr pybody, int affinedofs, int timederivative=0) const;

    object ExtractJointValues(object odata, PyKinBodyPtr pybody, object oindices, int timederivative=0) const;

    object ExtractDeltaTime(object odata) const;

    bool InsertJointValues(object odata, object ovalues, PyKinBodyPtr pybody, object oindices, int timederivative=0) const;

    bool InsertDeltaTime(object odata, dReal deltatime);

    py::list ExtractUsedBodies(PyEnvironmentBasePtr pyenv);

    object ExtractUsedIndices(PyKinBodyPtr pybody);

//
//    static void ConvertGroupData(std::vector<dReal>::iterator ittargetdata, size_t targetstride, const Group& gtarget, std::vector<dReal>::const_iterator itsourcedata, size_t sourcestride, const Group& gsource, size_t numpoints, EnvironmentBaseConstPtr penv);
//
    // source spec is the current configurationspecification spec
    object ConvertData(PyConfigurationSpecificationPtr pytargetspec, object osourcedata, size_t numpoints, PyEnvironmentBasePtr pyenv, bool filluninitialized = true);

    object ConvertDataFromPrevious(object otargetdata, PyConfigurationSpecificationPtr pytargetspec, object osourcedata, size_t numpoints, PyEnvironmentBasePtr pyenv);

    py::list GetGroups();

    bool __eq__(PyConfigurationSpecificationPtr p);
    bool __ne__(PyConfigurationSpecificationPtr p);

    PyConfigurationSpecificationPtr __add__(PyConfigurationSpecificationPtr r);

    PyConfigurationSpecificationPtr __iadd__(PyConfigurationSpecificationPtr r);

    std::string __repr__();
    std::string __str__();
    object __unicode__();

    // members
    ConfigurationSpecification _spec;

private:
    void _Update(const ConfigurationSpecification& spec);
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_CONFIGURATIONSPECIFICATION_H
