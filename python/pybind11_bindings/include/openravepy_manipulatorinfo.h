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
#ifndef OPENRAVEPY_INTERNAL_MANIPULATORINFO_H
#define OPENRAVEPY_INTERNAL_MANIPULATORINFO_H

#define NO_IMPORT_ARRAY
#include "../openravepy_int.h"

namespace openravepy {
using py::object;

class PyManipulatorInfo
{
public:
    PyManipulatorInfo();
    PyManipulatorInfo(const RobotBase::ManipulatorInfo& info);

    RobotBase::ManipulatorInfoPtr GetManipulatorInfo() const;

    object _name, _sBaseLinkName, _sEffectorLinkName;
    object _tLocalTool;
    object _vChuckingDirection;
    object _vdirection;
    std::string _sIkSolverXMLId;
    object _vGripperJointNames;
};

class PyAttachedSensorInfo
{
public:
    PyAttachedSensorInfo();
    PyAttachedSensorInfo(const RobotBase::AttachedSensorInfo& info);

    RobotBase::AttachedSensorInfoPtr GetAttachedSensorInfo() const;

    object _name, _linkname;
    object _trelative;
    object _sensorname;
    PySensorGeometryPtr _sensorgeometry;
};

class PyConnectedBodyInfo
{
public:
    PyConnectedBodyInfo();
    PyConnectedBodyInfo(const RobotBase::ConnectedBodyInfo& info, PyEnvironmentBasePtr pyenv);

    RobotBase::ConnectedBodyInfoPtr GetConnectedBodyInfo() const;

    object _name;
    object _linkname;
    object _trelative;
    object _url;
    object _linkInfos;
    object _jointInfos;
    object _manipulatorInfos;
    object _attachedSensorInfos;

};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_MANIPULATORINFO_H