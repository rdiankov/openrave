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

class PySensorSystemBase : public PyInterfaceBase
{
    friend class PyEnvironmentBase;
private:
    SensorSystemBasePtr _psensorsystem;
public:
    PySensorSystemBase(SensorSystemBasePtr psensorsystem, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(psensorsystem, pyenv), _psensorsystem(psensorsystem) {
    }
    virtual ~PySensorSystemBase() {
    }

    SensorSystemBasePtr GetSensorSystem() {
        return _psensorsystem;
    }
};

namespace openravepy {

SensorSystemBasePtr GetSensorSystem(PySensorSystemBasePtr pySensorSystem)
{
    return !pySensorSystem ? SensorSystemBasePtr() : pySensorSystem->GetSensorSystem();
}

PyInterfaceBasePtr toPySensorSystem(SensorSystemBasePtr pSensorSystem, PyEnvironmentBasePtr pyenv)
{
    return !pSensorSystem ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PySensorSystemBase(pSensorSystem,pyenv));
}

PySensorSystemBasePtr RaveCreateSensorSystem(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    SensorSystemBasePtr p = OpenRAVE::RaveCreateSensorSystem(GetEnvironment(pyenv), name);
    if( !p ) {
        return PySensorSystemBasePtr();
    }
    return PySensorSystemBasePtr(new PySensorSystemBase(p,pyenv));
}

void init_openravepy_sensorsystem()
{
    class_<PySensorSystemBase, boost::shared_ptr<PySensorSystemBase>, bases<PyInterfaceBase> >("SensorSystem", DOXY_CLASS(SensorSystemBase), no_init);

    def("RaveCreateSensorSystem",openravepy::RaveCreateSensorSystem,args("env","name"),DOXY_FN1(RaveCreateSensorSystem));
}

}
