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

class PyTrajectoryBase : public PyInterfaceBase
{
protected:
    TrajectoryBasePtr _ptrajectory;
public:
    PyTrajectoryBase(TrajectoryBasePtr pTrajectory, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pTrajectory, pyenv),_ptrajectory(pTrajectory) {
    }
    virtual ~PyTrajectoryBase() {
    }

    void deserialize(const string& s)
    {
        std::stringstream ss(s);
        _ptrajectory->deserialize(ss);
    }

    object serialize(int options)
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _ptrajectory->serialize(ss,options);
        return object(ss.str());
    }

    bool Read(const string& s, object probot) {
        RAVELOG_WARN("Trajectory.Read deprecated please use Trajerctory.deserialize\n");
        deserialize(s);
        return true;
    }

    object Write(int options) {
        RAVELOG_WARN("Trajectory.Write deprecated please use Trajerctory.serialize\n");
        return serialize(options);
    }

    TrajectoryBasePtr GetTrajectory() {
        return _ptrajectory;
    }
};

namespace openravepy {

TrajectoryBasePtr GetTrajectory(PyTrajectoryBasePtr pytrajectory)
{
    return !pytrajectory ? TrajectoryBasePtr() : pytrajectory->GetTrajectory();
}

PyInterfaceBasePtr toPyTrajectory(TrajectoryBasePtr ptrajectory, PyEnvironmentBasePtr pyenv)
{
    return !ptrajectory ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyTrajectoryBase(ptrajectory,pyenv));
}

PyTrajectoryBasePtr RaveCreateTrajectory(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    TrajectoryBasePtr p = OpenRAVE::RaveCreateTrajectory(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyTrajectoryBasePtr();
    }
    return PyTrajectoryBasePtr(new PyTrajectoryBase(p,pyenv));
}

void init_openravepy_trajectory()
{
    class_<PyTrajectoryBase, boost::shared_ptr<PyTrajectoryBase>, bases<PyInterfaceBase> >("Trajectory", DOXY_CLASS(TrajectoryBase), no_init)
    .def("serialize",&PyTrajectoryBase::serialize,args("options"),DOXY_FN(TrajectoryBase,serialize))
    .def("deserialize",&PyTrajectoryBase::deserialize,args("data"),DOXY_FN(TrajectoryBase,deserialize))
    .def("Write",&PyTrajectoryBase::Write,args("options"),DOXY_FN(TrajectoryBase,Write))
    .def("Read",&PyTrajectoryBase::Read,args("data","robot"),DOXY_FN(TrajectoryBase,Read))
    ;

    def("RaveCreateTrajectory",openravepy::RaveCreateTrajectory,args("env","name"),DOXY_FN1(RaveCreateTrajectory));
}

}
