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
    if(!_pDescriber->ComputePostureStates(_posturestates)) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Failed to compute posture", ORE_Failed);
    }
    return StdVectorToPyList<PostureStateInt>(_posturestates);
}

object PyPostureDescriber::ComputePostureStates(object pydofvalues)
{
    const std::vector<dReal> dofvalues = ExtractArray<dReal>(pydofvalues);
    if(!_pDescriber->ComputePostureStates(_posturestates, dofvalues)) {
        std::stringstream ss;
        if(!dofvalues.empty()) {
            ss << dofvalues[0];
        }
        for(size_t idof = 1; idof < dofvalues.size(); ++idof) {
            ss << ", " << dofvalues[idof];
        }
        throw OPENRAVE_EXCEPTION_FORMAT("Failed to compute posture for dof values: %s", ss.str(), ORE_Failed);
    }
    return StdVectorToPyList<PostureStateInt>(_posturestates);
}

std::string PyPostureDescriber::GetMapDataKey() const {
    return _pDescriber->GetMapDataKey();
}

py::dict PyPostureDescriber::Explain(object pystates) const {
    py::dict d;
    std::stringstream ssout, ssin;
    ssin << "GetSupportType";
    if(!_pDescriber->SendCommand(ssout, ssin)) {
        RAVELOG_WARN("Unsupported posture type, cannot explain");
        return d;
    }
    uint16_t supporttype = 0;
    ssout >> supporttype;
    std::vector<std::string> vfeatures;
    switch(supporttype) {
        case 1: {
            vfeatures = {"shoulder", "elbow", "wrist"};
            break;
        }
        case 2: {
            vfeatures = {"shoulder", "elbow"};
            break;
        }
        case 3: {
            vfeatures = {"elbow"};
            break;
        }
        default: {
            RAVELOG_WARN("Unsupported posture type, cannot explain");
            return d;
        }
    }

    std::vector<PostureStateInt> vstates = ExtractArray<PostureStateInt>(pystates);
    const std::set<PostureStateInt> sstates(begin(vstates), end(vstates));
    vstates = std::vector<PostureStateInt>(begin(sstates), end(sstates));
    for(const PostureStateInt state : vstates) {
        py::list l;
        int pow2 = 1 << (vfeatures.size() - 1);
        for(const std::string& feature : vfeatures) {
            py::list p;
            p.append(feature);
            p.append(state & pow2 ? 1 : 0);
            pow2 >>= 1;
            l.append(p);
        }
        d[state] = l;
    }
    return d;
}

PyPostureDescriberPtr GeneratePostureDescriber(const PyManipulatorPtr& pymanip) {
    return GeneratePostureDescriber(pymanip, "posturedescriber"); // default
}

PyPostureDescriberPtr GeneratePostureDescriber(const PyManipulatorPtr& pymanip, const std::string& interfacename) {
    const ManipulatorPtr pmanip = pymanip->GetManipulator();
    const RobotBasePtr probot = pmanip->GetRobot();
    const EnvironmentBasePtr penv = probot->GetEnv();
    std::vector<int> armindices;
    // fe743742269c7dbfe548cb1f3412f658
    const std::string chainhash = ComputeKinematicsChainHash(pmanip, armindices);
    const LinkPair linkpair = ExtractEssentialKinematicsChain(pmanip);
    // posturedescriber.motoman-gp8l.fe743742269c7dbfe548cb1f3412f658.L0.L6
    const std::string describername = interfacename + "." + probot->GetName() + "." + chainhash + "." + linkpair[0]->GetName() + "." + linkpair[1]->GetName();
    const PostureDescriberBasePtr pDescriber = RaveCreatePostureDescriber(penv, interfacename + " " + describername);
    if(pDescriber == nullptr) {
        RAVELOG_WARN_FORMAT("Can not generate posture describer interface \"%s\" for manpulator %s of robot %s", interfacename % pmanip->GetName() % probot->GetName());
    }
    if(!pDescriber->Supports(pmanip)) {
        RAVELOG_WARN_FORMAT("Posture describer interface \"%s\" does not support manpulator %s of robot %s", interfacename % pmanip->GetName() % probot->GetName());
        return PyPostureDescriberPtr();
    }
    if(!pDescriber->Init(pmanip)) {
        RAVELOG_WARN_FORMAT("Cannot initialize posture describer interface \"%s\" for manpulator %s of robot %s", interfacename % pmanip->GetName() % probot->GetName());
        return PyPostureDescriberPtr();
    }
    probot->SetPostureDescriber(pmanip, pDescriber); // set explicitly
    PyPostureDescriberPtr pyDescriber(new PyPostureDescriber(pDescriber, toPyEnvironment(pymanip->GetRobot())));
    return pyDescriber;
}

PostureDescriberBasePtr GetPostureDescriber(PyPostureDescriberPtr pydescriber)
{
    return !pydescriber ? PostureDescriberBasePtr() : pydescriber->GetPostureDescriber();
}

PyInterfaceBasePtr toPyPostureDescriber(PostureDescriberBasePtr pdescriber, PyEnvironmentBasePtr pyenv)
{
    return !pdescriber ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyPostureDescriber(pdescriber,pyenv));
}

PyPostureDescriberPtr RaveCreatePostureDescriber(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    PostureDescriberBasePtr p = OpenRAVE::RaveCreatePostureDescriber(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyPostureDescriberPtr();
    }
    return PyPostureDescriberPtr(new PyPostureDescriber(p, pyenv));
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
    .def("Supports",             SupportsWithTwoLinks               , PY_ARGS("baselink", "eelink") DOXY_FN(PostureDescriberBase, Supports "const std::array<RobotBase::LinkPtr, 2>& kinematicsChain"))
    .def("Supports",             SupportsWithManip                  , PY_ARGS("manipulator")        DOXY_FN(PostureDescriberBase, Supports "const RobotBase::ManipulatorPtr& pmanip"))
    .def("Init",                 InitWithTwoLinks                   , PY_ARGS("baselink", "eelink") DOXY_FN(PostureDescriberBase, Init "const std::array<RobotBase::LinkPtr, 2>& kinematicsChain"))
    .def("Init",                 InitWithManip                      , PY_ARGS("manipulator")        DOXY_FN(PostureDescriberBase, Init "const RobotBase::ManipulatorPtr& pmanip"))

    .def("ComputePostureStates", ComputePostureStates               ,                               DOXY_FN(PostureDescriberBase, ComputePostureStates ""))
    .def("ComputePostureStates", ComputePostureStatesWithJointValues, PY_ARGS("dofvalues")          DOXY_FN(PostureDescriberBase, ComputePostureStates "const std::vector<double>& dofvalues"))
    .def("GetMapDataKey"       , &PyPostureDescriber::GetMapDataKey ,                               DOXY_FN(PostureDescriberBase, GetMapDataKey ""))
    .def("Explain"             , &PyPostureDescriber::Explain       , PY_ARGS("posturestates")      DOXY_FN(PostureDescriberBase, Explain ""))
    ;

    PyPostureDescriberPtr (*GeneratePostureDescriberDefault    )(const PyManipulatorPtr&                    ) = &GeneratePostureDescriber;
    PyPostureDescriberPtr (*GeneratePostureDescriberByInterface)(const PyManipulatorPtr&, const std::string&) = &GeneratePostureDescriber;
    PyPostureDescriberPtr (*openravepyRaveCreatePostureDescriber)(PyEnvironmentBasePtr pyenv, const std::string& name) = &RaveCreatePostureDescriber;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("GeneratePostureDescriber"  , GeneratePostureDescriberDefault       , PY_ARGS("manip")                  DOXY_FN1(GeneratePostureDescriber));
    m.def("GeneratePostureDescriber"  , GeneratePostureDescriberByInterface   , PY_ARGS("manip", "interfacename") DOXY_FN1(GeneratePostureDescriber));
    m.def("RaveCreatePostureDescriber", openravepyRaveCreatePostureDescriber  , PY_ARGS("env", "name")            DOXY_FN1(RaveCreatePostureDescriber));
#else
    def("GeneratePostureDescriber"  , GeneratePostureDescriberDefault       , PY_ARGS("manip")                  DOXY_FN1(GeneratePostureDescriber));
    def("GeneratePostureDescriber"  , GeneratePostureDescriberByInterface   , PY_ARGS("manip", "interfacename") DOXY_FN1(GeneratePostureDescriber));
    def("RaveCreatePostureDescriber", openravepyRaveCreatePostureDescriber  , PY_ARGS("env", "name")            DOXY_FN1(RaveCreatePostureDescriber));
#endif
}

} // namespace openravepy
