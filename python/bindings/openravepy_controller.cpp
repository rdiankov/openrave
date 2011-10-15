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

class PyControllerBase : public PyInterfaceBase
{
protected:
    ControllerBasePtr _pcontroller;
public:
    PyControllerBase(ControllerBasePtr pcontroller, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pcontroller, pyenv), _pcontroller(pcontroller) {
    }
    virtual ~PyControllerBase() {
    }

    ControllerBasePtr GetController() {
        return _pcontroller;
    }

    bool Init(PyRobotBasePtr pyrobot, const string& args)
    {
        RAVELOG_WARN("PyControllerBase::Init(robot,args) deprecated!\n");
        CHECK_POINTER(pyrobot);
        RobotBasePtr probot = openravepy::GetRobot(pyrobot);
        std::vector<int> dofindices;
        for(int i = 0; i < probot->GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        return _pcontroller->Init(probot, dofindices,1);
    }

    bool Init(PyRobotBasePtr pyrobot, object odofindices, int nControlTransformation)
    {
        CHECK_POINTER(pyrobot);
        vector<int> dofindices = ExtractArray<int>(odofindices);
        return _pcontroller->Init(openravepy::GetRobot(pyrobot),dofindices,nControlTransformation);
    }

    object GetControlDOFIndices() {
        return toPyArray(_pcontroller->GetControlDOFIndices());
    }
    int IsControlTransformation() {
        return _pcontroller->IsControlTransformation();
    }
    object GetRobot()
    {
        return object(openravepy::toPyRobot(_pcontroller->GetRobot(),_pyenv));
    }

    void Reset(int options) {
        _pcontroller->Reset(options);
    }

    bool SetDesired(object o)
    {
        vector<dReal> values = ExtractArray<dReal>(o);
        if( values.size() == 0 ) {
            throw openrave_exception("no values specified");
        }
        return _pcontroller->SetDesired(values);
    }

    bool SetDesired(object o, object otransform)
    {
        if( otransform == object() ) {
            return SetDesired(o);
        }
        return _pcontroller->SetDesired(ExtractArray<dReal>(o),TransformConstPtr(new Transform(ExtractTransform(otransform))));
    }

    bool SetPath(PyTrajectoryBasePtr pytraj)
    {
        CHECK_POINTER(pytraj);
        return _pcontroller->SetPath (openravepy::GetTrajectory(pytraj));
    }

    void SimulationStep(dReal fTimeElapsed) {
        _pcontroller->SimulationStep(fTimeElapsed);
    }

    bool IsDone() {
        return _pcontroller->IsDone();
    }
    dReal GetTime() {
        return _pcontroller->GetTime();
    }

    object GetVelocity()
    {
        vector<dReal> velocity;
        _pcontroller->GetVelocity(velocity);
        return toPyArray(velocity);
    }

    object GetTorque()
    {
        vector<dReal> torque;
        _pcontroller->GetTorque(torque);
        return toPyArray(torque);
    }
};

namespace openravepy {

ControllerBasePtr GetController(PyControllerBasePtr pycontroller)
{
    return !pycontroller ? ControllerBasePtr() : pycontroller->GetController();
}

PyInterfaceBasePtr toPyController(ControllerBasePtr pcontroller, PyEnvironmentBasePtr pyenv)
{
    return !pcontroller ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyControllerBase(pcontroller,pyenv));
}

PyControllerBasePtr RaveCreateController(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    ControllerBasePtr p = OpenRAVE::RaveCreateController(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyControllerBasePtr();
    }
    return PyControllerBasePtr(new PyControllerBase(p,pyenv));
}

void init_openravepy_controller()
{
    {
        bool (PyControllerBase::*init1)(PyRobotBasePtr,const string &) = &PyControllerBase::Init;
        bool (PyControllerBase::*init2)(PyRobotBasePtr,object,int) = &PyControllerBase::Init;
        bool (PyControllerBase::*setdesired1)(object) = &PyControllerBase::SetDesired;
        bool (PyControllerBase::*setdesired2)(object,object) = &PyControllerBase::SetDesired;
        class_<PyControllerBase, boost::shared_ptr<PyControllerBase>, bases<PyInterfaceBase> >("Controller", DOXY_CLASS(ControllerBase), no_init)
        .def("Init",init1, DOXY_FN(ControllerBase,Init))
        .def("Init",init2, args("robot","dofindices","controltransform"), DOXY_FN(ControllerBase,Init))
        .def("GetControlDOFIndices",&PyControllerBase::GetControlDOFIndices,DOXY_FN(ControllerBase,GetControlDOFIndices))
        .def("IsControlTransformation",&PyControllerBase::IsControlTransformation, DOXY_FN(ControllerBase,IsControlTransformation))
        .def("GetRobot",&PyControllerBase::GetRobot, DOXY_FN(ControllerBase,GetRobot))
        .def("Reset",&PyControllerBase::Reset, DOXY_FN(ControllerBase,Reset))
        .def("SetDesired",setdesired1, args("values"), DOXY_FN(ControllerBase,SetDesired))
        .def("SetDesired",setdesired2, args("values","transform"), DOXY_FN(ControllerBase,SetDesired))
        .def("SetPath",&PyControllerBase::SetPath, DOXY_FN(ControllerBase,SetPath))
        .def("SimulationStep",&PyControllerBase::SimulationStep, DOXY_FN(ControllerBase,SimulationStep "dReal"))
        .def("IsDone",&PyControllerBase::IsDone, DOXY_FN(ControllerBase,IsDone))
        .def("GetTime",&PyControllerBase::GetTime, DOXY_FN(ControllerBase,GetTime))
        .def("GetVelocity",&PyControllerBase::GetVelocity, DOXY_FN(ControllerBase,GetVelocity))
        .def("GetTorque",&PyControllerBase::GetTorque, DOXY_FN(ControllerBase,GetTorque))
        ;
    }

    def("RaveCreateController",openravepy::RaveCreateController,args("env","name"),DOXY_FN1(RaveCreateController));
}

}
