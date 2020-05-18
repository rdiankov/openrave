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
#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_environmentbase.h>
#include <openravepy/openravepy_posturedescriber.h> // PyPostureDescriber
#include <openrave/posturedescriber.h> // PostureDescriber
#include <openrave/utils.h>

namespace openravepy {

using py::object;
using py::extract;
using py::extract_;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::init;
using py::scope_; // py::object if USE_PYBIND11_PYTHON_BINDINGS
using py::scope;
using py::args;
using py::return_value_policy;

#ifndef USE_PYBIND11_PYTHON_BINDINGS
using py::no_init;
using py::bases;
using py::copy_const_reference;
using py::docstring_options;
using py::pickle_suite;
using py::manage_new_object;
using py::def;
#endif // USE_PYBIND11_PYTHON_BINDINGS

using ManipulatorPtr = OpenRAVE::RobotBase::ManipulatorPtr;
using LinkPtr = OpenRAVE::RobotBase::LinkPtr;
using OpenRAVE::PostureDescriberBase;
using OpenRAVE::PostureDescriberBasePtr;
using OpenRAVE::PostureDescriber;
using OpenRAVE::PostureDescriberPtr;

namespace numeric = py::numeric;

PyPostureDescriber::PyPostureDescriber(PostureDescriberBasePtr pDescriber, PyEnvironmentBasePtr pyenv)
    : PyInterfaceBase(pDescriber, pyenv),
    _pDescriber(pDescriber) {
}

PyPostureDescriber::~PyPostureDescriber() {}

PostureDescriberBasePtr PyPostureDescriber::GetPostureDescriber() const { return _pDescriber; }

bool PyPostureDescriber::Supports(PyLinkPtr pBaseLink, PyLinkPtr pEndEffectorLink) const {
    const std::array<LinkPtr, 2> kinematicsChain {pBaseLink->GetLink(), pEndEffectorLink->GetLink()};
    return _pDescriber->Supports(kinematicsChain);
}

bool PyPostureDescriber::Supports(PyRobotBase::PyManipulatorPtr pmanip) const {
    return _pDescriber->Supports(pmanip->GetManipulator());
}

bool PyPostureDescriber::Init(PyLinkPtr pBaseLink, PyLinkPtr pEndEffectorLink) {
    const std::array<LinkPtr, 2> kinematicsChain {pBaseLink->GetLink(), pEndEffectorLink->GetLink()};
    return _pDescriber->Init(kinematicsChain);
}

bool PyPostureDescriber::Init(PyRobotBase::PyManipulatorPtr pmanip) {
    return _pDescriber->Init(pmanip->GetManipulator());
}

PyPostureDescriberPtr GetPostureDescriber(PyRobotBase::PyManipulatorPtr pymanip) {
    const ManipulatorPtr pmanip = pymanip->GetManipulator();
    const RobotBasePtr probot = pmanip->GetRobot();
    const EnvironmentBasePtr penv = probot->GetEnv();
    PostureDescriberBasePtr pDescriber(new PostureDescriber(penv));
    if(!pDescriber->Supports(pmanip)) {
        return PyPostureDescriberPtr();
    }
    pDescriber->Init(pmanip);
    probot->SetPostureDescriber(pmanip, pDescriber);

    PyPostureDescriberPtr pyDescriber(new PyPostureDescriber(pDescriber, toPyEnvironment(pymanip->GetRobot())));
    return pyDescriber;
}

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_posturedescriber(py::module& m)
#else
void init_openravepy_posturedescriber()
#endif
{
    bool (PyPostureDescriber::*InitWithTwoLinks)(PyLinkPtr, PyLinkPtr)       = &PyPostureDescriber::Init;
    bool (PyPostureDescriber::*InitWithManip)(PyRobotBase::PyManipulatorPtr) = &PyPostureDescriber::Init;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPostureDescriber, OPENRAVE_SHARED_PTR<PyPostureDescriber>, PyInterfaceBase>(m, "PostureDescriber", DOXY_CLASS(PostureDescriberBase))
#else
    class_<PyPostureDescriber, OPENRAVE_SHARED_PTR<PyPostureDescriber>, bases<PyInterfaceBase> >("PostureDescriber", DOXY_CLASS(PostureDescriberBase), no_init)
#endif
    .def("Init", InitWithTwoLinks, PY_ARGS("baselink", "eelink") DOXY_FN(PostureDescriberBase, Init "const std::array<RobotBase::LinkPtr, 2>& kinematicsChain"))
    .def("Init", InitWithManip,    PY_ARGS("manipulator")        DOXY_FN(PostureDescriberBase, Init "const RobotBase::ManipulatorPtr& pmanip"))
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("GetPostureDescriber", GetPostureDescriber, PY_ARGS("manip") DOXY_FN1(GetPostureDescriber));
#else
    def("GetPostureDescriber", GetPostureDescriber, PY_ARGS("manip") DOXY_FN1(GetPostureDescriber));
#endif
}

} // namespace openravepy
