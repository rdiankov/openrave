// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Guangning Tan, Kei Usui, Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
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
using PyManipulatorPtr = PyRobotBase::PyManipulatorPtr;

namespace numeric = py::numeric;

PyPostureDescriber::PyPostureDescriber(PostureDescriberBasePtr pDescriber, PyEnvironmentBasePtr pyenv)
    : PyInterfaceBase(pDescriber, pyenv),
    _pDescriber(pDescriber) {
}

PyPostureDescriber::~PyPostureDescriber() {
}

PostureDescriberBasePtr PyPostureDescriber::GetPostureDescriber() const {
    return _pDescriber;
}

bool PyPostureDescriber::Supports(PyLinkPtr pBaseLink, PyLinkPtr pEndEffectorLink) const {
    const LinkPair kinematicsChain {pBaseLink->GetLink(), pEndEffectorLink->GetLink()};
    return _pDescriber->Supports(kinematicsChain);
}

bool PyPostureDescriber::Supports(PyManipulatorPtr pmanip) const {
    return _pDescriber->Supports(pmanip->GetManipulator());
}

bool PyPostureDescriber::Init(PyLinkPtr pBaseLink, PyLinkPtr pEndEffectorLink) {
    const LinkPair kinematicsChain {pBaseLink->GetLink(), pEndEffectorLink->GetLink()};
    return _pDescriber->Init(kinematicsChain);
}

bool PyPostureDescriber::Init(PyManipulatorPtr pmanip) {
    return _pDescriber->Init(pmanip->GetManipulator());
}

object PyPostureDescriber::ComputePostureStates()
{
    // return (_pDescriber->ComputePostureStates(_posturestates)) ? toPyArray<PostureStateInt>(_posturestates) : py::empty_array_astype<PostureStateInt>();
    return StdVectorToPyList<PostureStateInt>(_pDescriber->ComputePostureStates(_posturestates) ? _posturestates : std::vector<PostureStateInt>());
}

object PyPostureDescriber::ComputePostureStates(object pydofvalues)
{
    const std::vector<dReal> dofvalues = ExtractArray<dReal>(pydofvalues);
    // return (_pDescriber->ComputePostureStates(_posturestates, dofvalues)) ? toPyArray<PostureStateInt>(_posturestates) : py::empty_array_astype<PostureStateInt>();
    return StdVectorToPyList<PostureStateInt>(_pDescriber->ComputePostureStates(_posturestates, dofvalues) ? _posturestates : std::vector<PostureStateInt>());
}

PyPostureDescriberPtr GeneratePostureDescriber(const PyManipulatorPtr& pymanip) {
    const ManipulatorPtr pmanip = pymanip->GetManipulator();
    const RobotBasePtr probot = pmanip->GetRobot();
    const EnvironmentBasePtr penv = probot->GetEnv();
    const std::string interfacename = "posturedescriber"; // POSTUREDESCRIBER_CLASS_NAME
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
    bool (PyPostureDescriber::*InitWithTwoLinks)(PyLinkPtr, PyLinkPtr)           = &PyPostureDescriber::Init;
    bool (PyPostureDescriber::*InitWithManip)(PyManipulatorPtr)                  = &PyPostureDescriber::Init;
    bool (PyPostureDescriber::*SupportsWithTwoLinks)(PyLinkPtr, PyLinkPtr) const = &PyPostureDescriber::Supports;
    bool (PyPostureDescriber::*SupportsWithManip)(PyManipulatorPtr)        const = &PyPostureDescriber::Supports;
    object (PyPostureDescriber::*ComputePostureStates)()                         = &PyPostureDescriber::ComputePostureStates;
    object (PyPostureDescriber::*ComputePostureStatesWithJointValues)(object)    = &PyPostureDescriber::ComputePostureStates;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPostureDescriber, OPENRAVE_SHARED_PTR<PyPostureDescriber>, PyInterfaceBase>(m, "PostureDescriber", DOXY_CLASS(PostureDescriberBase))
#else
    class_<PyPostureDescriber, OPENRAVE_SHARED_PTR<PyPostureDescriber>, bases<PyInterfaceBase> >("PostureDescriber", DOXY_CLASS(PostureDescriberBase), no_init)
#endif
    // We may not want the user to call with these interfaces.
    /*
       .def("Supports",             SupportsWithTwoLinks,                PY_ARGS("baselink", "eelink") DOXY_FN(PostureDescriberBase, Supports "const std::array<RobotBase::LinkPtr, 2>& kinematicsChain"))
       .def("Supports",             SupportsWithManip,                   PY_ARGS("manipulator")        DOXY_FN(PostureDescriberBase, Supports "const RobotBase::ManipulatorPtr& pmanip"))
       .def("Init",                 InitWithTwoLinks,                    PY_ARGS("baselink", "eelink") DOXY_FN(PostureDescriberBase, Init "const std::array<RobotBase::LinkPtr, 2>& kinematicsChain"))
       .def("Init",                 InitWithManip,                       PY_ARGS("manipulator")        DOXY_FN(PostureDescriberBase, Init "const RobotBase::ManipulatorPtr& pmanip"))
     */
    .def("ComputePostureStates", ComputePostureStates,                                            DOXY_FN(PostureDescriberBase, ComputePostureStates ""))
    .def("ComputePostureStates", ComputePostureStatesWithJointValues, PY_ARGS("dofvalues")        DOXY_FN(PostureDescriberBase, ComputePostureStates "const std::vector<double>& dofvalues"))
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("GeneratePostureDescriber", GeneratePostureDescriber, PY_ARGS("manip") DOXY_FN1(GeneratePostureDescriber));
#else
    def("GeneratePostureDescriber", GeneratePostureDescriber, PY_ARGS("manip") DOXY_FN1(GeneratePostureDescriber));
#endif
}

} // namespace openravepy
