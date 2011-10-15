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

class PyModuleBase : public PyInterfaceBase
{
protected:
    ModuleBasePtr _pmodule;
public:
    PyModuleBase(ModuleBasePtr pmodule, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pmodule, pyenv), _pmodule(pmodule) {
    }
    virtual ~PyModuleBase() {
    }
    ModuleBasePtr GetModule() {
        return _pmodule;
    }

    bool SimulationStep(dReal fElapsedTime) {
        return _pmodule->SimulationStep(fElapsedTime);
    }
};

namespace openravepy {

ModuleBasePtr GetModule(PyModuleBasePtr pymodule)
{
    return !pymodule ? ModuleBasePtr() : pymodule->GetModule();
}

PyInterfaceBasePtr toPyModule(ModuleBasePtr pmodule, PyEnvironmentBasePtr pyenv)
{
    return !pmodule ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyModuleBase(pmodule,pyenv));
}

PyModuleBasePtr RaveCreateModule(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    ModuleBasePtr p = OpenRAVE::RaveCreateModule(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyModuleBasePtr();
    }
    return PyModuleBasePtr(new PyModuleBase(p,pyenv));
}

void init_openravepy_module()
{
    class_<PyModuleBase, boost::shared_ptr<PyModuleBase>, bases<PyInterfaceBase> >("Module", DOXY_CLASS(ModuleBase), no_init)
    .def("SimulationStep",&PyModuleBase::SimulationStep, DOXY_FN(ModuleBase,"SimulationStep"))
    ;

    def("RaveCreateModule",openravepy::RaveCreateModule,args("env","name"),DOXY_FN1(RaveCreateModule));
    def("RaveCreateProblem",openravepy::RaveCreateModule,args("env","name"),DOXY_FN1(RaveCreateModule));
    def("RaveCreateProblemInstance",openravepy::RaveCreateModule,args("env","name"),DOXY_FN1(RaveCreateModule));
}

}
