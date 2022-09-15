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
#ifndef OPENRAVEPY_INTERNAL_IKSOLVERBASE_H
#define OPENRAVEPY_INTERNAL_IKSOLVERBASE_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class PyIkReturn
{
public:
    PyIkReturn(const IkReturn& ret);
    PyIkReturn(IkReturnPtr pret);
    PyIkReturn(IkReturnAction action);
    IkReturnAction GetAction();
    object GetSolution();
    object GetUserData();
    object GetMapData(const std::string& key);
    object GetMapDataDict();

    void SetUserData(PyUserData pdata);
    void SetSolution(object osolution);
    void SetMapKeyValue(const std::string& key, object ovalues);

    IkReturn _ret;
};

typedef OPENRAVE_SHARED_PTR<PyIkReturn> PyIkReturnPtr;

class PyIkSolverBase : public PyInterfaceBase
{
protected:
    IkSolverBasePtr _pIkSolver;

    static IkReturn _CallCustomFilter(object fncallback, PyEnvironmentBasePtr pyenv, IkSolverBasePtr pIkSolver, std::vector<dReal>& values, RobotBase::ManipulatorConstPtr pmanip, const IkParameterization& ikparam);

public:
    PyIkSolverBase(IkSolverBasePtr pIkSolver, PyEnvironmentBasePtr pyenv);
    virtual ~PyIkSolverBase();

    IkSolverBasePtr GetIkSolver();

    int GetNumFreeParameters() const;
    object GetFreeParameters() const;

    PyIkReturnPtr Solve(object oparam, object oq0, int filteroptions);

    object SolveAll(object oparam, int filteroptions);

    PyIkReturnPtr Solve(object oparam, object oq0, object oFreeParameters, int filteroptions);

    object SolveAll(object oparam, object oFreeParameters, int filteroptions);

    PyIkReturnPtr CallFilters(object oparam);

    bool Supports(IkParameterizationType type);

    object RegisterCustomFilter(int priority, object fncallback);
};
} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_IKSOLVERBASE_H