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
using OpenRAVE::RaveCreatePostureDescriber;

namespace numeric = py::numeric;

template <typename T>
py::list StdVecToPyList(const std::vector<T>& v) {
    py::list l;
    const size_t N = v.size();
    for(size_t i = 0; i < N; i++) {
        l.append(v[i]);
    }
    return l;
};

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

object PyPostureDescriber::ComputePostureStates()
{
    // return (_pDescriber->ComputePostureStates(_posturestates)) ? toPyArray<uint16_t>(_posturestates) : py::empty_array_astype<uint16_t>();
    return StdVecToPyList<uint16_t>(_pDescriber->ComputePostureStates(_posturestates) ? _posturestates : std::vector<uint16_t>());
}

object PyPostureDescriber::ComputePostureStates(object pyjointvalues)
{
    const std::vector<dReal> vjointvalues = ExtractArray<dReal>(pyjointvalues);
    // return (_pDescriber->ComputePostureStates(_posturestates, vjointvalues)) ? toPyArray<uint16_t>(_posturestates) : py::empty_array_astype<uint16_t>();
    return StdVecToPyList<uint16_t>(_pDescriber->ComputePostureStates(_posturestates, vjointvalues) ? _posturestates : std::vector<uint16_t>());
}

PyPostureDescriberPtr GetPostureDescriber(PyRobotBase::PyManipulatorPtr pymanip) {
    const ManipulatorPtr pmanip = pymanip->GetManipulator();
    const RobotBasePtr probot = pmanip->GetRobot();
    const EnvironmentBasePtr penv = probot->GetEnv();
    const std::string interfacename = "posturedescriber";
    std::vector<int> armindices;
    // fe743742269c7dbfe548cb1f3412f658
    const std::string chainhash = ComputeKinematicsChainHash(pmanip, armindices);
    // posturedescriber.motoman-gp8l.fe743742269c7dbfe548cb1f3412f658.L0.L6
    const std::string describername = interfacename + "." + probot->GetName() + "." + chainhash + "." + pmanip->GetBase()->GetName() + "." + pmanip->GetEndEffector()->GetName();
    PostureDescriberBasePtr pDescriber = RaveCreatePostureDescriber(penv, interfacename + " " + describername);
    if(!pDescriber->Supports(pmanip)) {
        return PyPostureDescriberPtr();
    }
    pDescriber->Init(pmanip);
    probot->SetPostureDescriber(pmanip, pDescriber); // set explicitly
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
    bool (PyPostureDescriber::*SupportsWithTwoLinks)(PyLinkPtr, PyLinkPtr)       const = &PyPostureDescriber::Supports;
    bool (PyPostureDescriber::*SupportsWithManip)(PyRobotBase::PyManipulatorPtr) const = &PyPostureDescriber::Supports;
    object (PyPostureDescriber::*ComputePostureStates)()                      = &PyPostureDescriber::ComputePostureStates;
    object (PyPostureDescriber::*ComputePostureStatesWithJointValues)(object) = &PyPostureDescriber::ComputePostureStates;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPostureDescriber, OPENRAVE_SHARED_PTR<PyPostureDescriber>, PyInterfaceBase>(m, "PostureDescriber", DOXY_CLASS(PostureDescriberBase))
#else
    class_<PyPostureDescriber, OPENRAVE_SHARED_PTR<PyPostureDescriber>, bases<PyInterfaceBase> >("PostureDescriber", DOXY_CLASS(PostureDescriberBase), no_init)
#endif
    .def("Supports",             SupportsWithTwoLinks,                PY_ARGS("baselink", "eelink") DOXY_FN(PostureDescriberBase, Supports "const std::array<RobotBase::LinkPtr, 2>& kinematicsChain"))
    .def("Supports",             SupportsWithManip,                   PY_ARGS("manipulator")        DOXY_FN(PostureDescriberBase, Supports "const RobotBase::ManipulatorPtr& pmanip"))
    .def("Init",                 InitWithTwoLinks,                    PY_ARGS("baselink", "eelink") DOXY_FN(PostureDescriberBase, Init "const std::array<RobotBase::LinkPtr, 2>& kinematicsChain"))
    .def("Init",                 InitWithManip,                       PY_ARGS("manipulator")        DOXY_FN(PostureDescriberBase, Init "const RobotBase::ManipulatorPtr& pmanip"))
    .def("ComputePostureStates", ComputePostureStates,                                              DOXY_FN(PostureDescriberBase, ComputePostureStates ""))
    .def("ComputePostureStates", ComputePostureStatesWithJointValues, PY_ARGS("jointvalues")        DOXY_FN(PostureDescriberBase, ComputePostureStates ""))
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("GetPostureDescriber", GetPostureDescriber, PY_ARGS("manip") DOXY_FN1(GetPostureDescriber));
#else
    def("GetPostureDescriber", GetPostureDescriber, PY_ARGS("manip") DOXY_FN1(GetPostureDescriber));
#endif
}

} // namespace openravepy
