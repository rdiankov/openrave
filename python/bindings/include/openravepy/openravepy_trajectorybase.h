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
#ifndef OPENRAVEPY_INTERNAL_TRAJECTORYBASE_H
#define OPENRAVEPY_INTERNAL_TRAJECTORYBASE_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class PyTrajectoryBase : public PyInterfaceBase
{
protected:
    TrajectoryBasePtr _ptrajectory;
public:
    PyTrajectoryBase(TrajectoryBasePtr pTrajectory, PyEnvironmentBasePtr pyenv);
    virtual ~PyTrajectoryBase();
    void Init(PyConfigurationSpecificationPtr pyspec);

    void Insert(size_t index, object odata);

    void Insert(size_t index, object odata, bool bOverwrite);

    void Insert(size_t index, object odata, PyConfigurationSpecificationPtr pyspec);

    void Insert(size_t index, object odata, PyConfigurationSpecificationPtr pyspec, bool bOverwrite);

    void Remove(size_t startindex, size_t endindex);

    object Sample(dReal time) const;

    object Sample(dReal time, PyConfigurationSpecificationPtr pyspec) const;

    object SampleFromPrevious(object odata, dReal time, PyConfigurationSpecificationPtr pyspec) const;

    object SamplePoints2D(object otimes) const;

    object SamplePoints2D(object otimes, PyConfigurationSpecificationPtr pyspec) const;

    object GetConfigurationSpecification() const;

    size_t GetNumWaypoints() const;

    object GetWaypoints(size_t startindex, size_t endindex) const;

    object GetWaypoints(size_t startindex, size_t endindex, PyConfigurationSpecificationPtr pyspec) const;

    // similar to GetWaypoints except returns a 2D array, one row for every waypoint
    object GetWaypoints2D(size_t startindex, size_t endindex) const;

    object __getitem__(int index) const;

    object __getitem__(py::slice indices) const;

    object GetAllWaypoints2D() const;

    object GetWaypoints2D(size_t startindex, size_t endindex, PyConfigurationSpecificationPtr pyspec) const;

    object GetAllWaypoints2D(PyConfigurationSpecificationPtr pyspec) const;

    object GetWaypoint(int index) const;

    object GetWaypoint(int index, PyConfigurationSpecificationPtr pyspec) const;

    size_t GetFirstWaypointIndexAfterTime(dReal time) const;

    dReal GetDuration() const;

    void deserialize(const string& s);

    object serialize(object options=py::none_());

    void SaveToFile(const std::string& filename, object options=py::none_());

    void LoadFromFile(const std::string& filename);
    
    TrajectoryBasePtr GetTrajectory();

    // functions that explictly initialize ConfigurationSpecification with ConfigurationSpecification::Group
    void Init(OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup);
    void Insert(size_t index, object odata, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup);
    void Insert(size_t index, object odata, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup, bool bOverwrite);
    object Sample(dReal time, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    object SampleFromPrevious(object odata, dReal time, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    object SamplePoints2D(object otimes, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    object GetWaypoints(size_t startindex, size_t endindex, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    object GetWaypoints2D(size_t startindex, size_t endindex, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    object GetAllWaypoints2D(OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    object GetWaypoint(int index, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_TRAJECTORYBASE_H
