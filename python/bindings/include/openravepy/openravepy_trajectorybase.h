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

/// \brief wrapper around TrajectoryBase.
/// This class is not multi-thread safe in general, however concurrent read operations (Sample and GetWaypoints methods) are supported.
class OPENRAVEPY_API PyTrajectoryBase : public PyInterfaceBase
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

    py::array_t<dReal> Sample(dReal time) const;

    py::array_t<dReal> Sample(dReal time, PyConfigurationSpecificationPtr pyspec) const;

    py::array_t<dReal> SampleFromPrevious(object odata, dReal time, PyConfigurationSpecificationPtr pyspec) const;

    py::array_t<dReal> SamplePoints2D(object otimes) const;

    py::array_t<dReal> SamplePoints2D(object otimes, PyConfigurationSpecificationPtr pyspec) const;

    py::array_t<dReal> SamplePointsSameDeltaTime2D(dReal deltatime, bool ensureLastPoint) const;

    py::array_t<dReal> SamplePointsSameDeltaTime2D(dReal deltatime, bool ensureLastPoint, PyConfigurationSpecificationPtr pyspec) const;

    py::array_t<dReal> SampleRangeSameDeltaTime2D(dReal deltatime, dReal startTime, dReal stopTime, bool ensureLastPoint) const;

    py::array_t<dReal> SampleRangeSameDeltaTime2D(dReal deltatime, dReal startTime, dReal stopTime, bool ensureLastPoint, PyConfigurationSpecificationPtr pyspec) const;

    PyConfigurationSpecificationPtr GetConfigurationSpecification() const;

    size_t GetNumWaypoints() const;

    py::array_t<dReal> GetWaypoints(size_t startindex, size_t endindex) const;

    py::array_t<dReal> GetWaypoints(size_t startindex, size_t endindex, PyConfigurationSpecificationPtr pyspec) const;

    // similar to GetWaypoints except returns a 2D array, one row for every waypoint
    py::array_t<dReal> GetWaypoints2D(size_t startindex, size_t endindex) const;

    py::array_t<dReal> __getitem__(int index) const;

    py::array_t<dReal> __getitem__(py::slice indices) const;

    py::array_t<dReal> GetAllWaypoints2D() const;

    py::array_t<dReal> GetWaypoints2D(size_t startindex, size_t endindex, PyConfigurationSpecificationPtr pyspec) const;

    py::array_t<dReal> GetAllWaypoints2D(PyConfigurationSpecificationPtr pyspec) const;

    py::array_t<dReal> GetWaypoint(int index) const;

    py::array_t<dReal> GetWaypoint(int index, PyConfigurationSpecificationPtr pyspec) const;

    size_t GetFirstWaypointIndexAfterTime(dReal time) const;

    dReal GetDuration() const;

    void deserialize(const string& s);

    py::bytes serialize(object options=py::none_());

    void SaveToFile(const std::string& filename, object options=py::none_());

    void LoadFromFile(const std::string& filename);
    
    TrajectoryBasePtr GetTrajectory();

    // functions that explictly initialize ConfigurationSpecification with ConfigurationSpecification::Group
    void Init(OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup);
    void Insert(size_t index, object odata, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup);
    void Insert(size_t index, object odata, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup, bool bOverwrite);
    py::array_t<dReal> Sample(dReal time, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    py::array_t<dReal> SampleFromPrevious(object odata, dReal time, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    py::array_t<dReal> SamplePoints2D(object otimes, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    py::array_t<dReal> SamplePointsSameDeltaTime2D(dReal deltatime, bool ensureLastPoint, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    py::array_t<dReal> SampleRangeSameDeltaTime2D(dReal deltatime, dReal startTime, dReal stopTime, bool ensureLastPoint, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    py::array_t<dReal> GetWaypoints(size_t startindex, size_t endindex, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    py::array_t<dReal> GetWaypoints2D(size_t startindex, size_t endindex, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    py::array_t<dReal> GetAllWaypoints2D(OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;
    py::array_t<dReal> GetWaypoint(int index, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> pygroup) const;

private:
    static thread_local std::vector<dReal> _vdataCache, _vtimesCache; ///< caches to avoid memory allocation. TLS to suppport concurrent data read ( getting waypoint, sampling and so on ) from multiple threads.
};

struct TrajectoryBaseInitializer
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    TrajectoryBaseInitializer(py::module& m_);
    void init_openravepy_trajectory();
    py::module& m;
#else
    TrajectoryBaseInitializer();
    void init_openravepy_trajectory();
#endif
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_TRAJECTORYBASE_H
