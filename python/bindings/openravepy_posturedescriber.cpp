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
#include <openravepy/openravepy_posturedescriber.h> // PyPostureDescriberBase
#include <openrave/utils.h>
#include <openrave/openravejson.h> // openravejson

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

PyPostureDescriberBase::PyPostureDescriberBase(PostureDescriberBasePtr pDescriber, PyEnvironmentBasePtr pyenv)
    : PyInterfaceBase(pDescriber, pyenv),
    _pDescriber(pDescriber) {
}

PyPostureDescriberBase::~PyPostureDescriberBase() {
}

PostureDescriberBasePtr PyPostureDescriberBase::GetPostureDescriber() const {
    return _pDescriber;
}

bool PyPostureDescriberBase::Supports(PyLinkPtr pBaseLink, PyLinkPtr pEndEffectorLink) const {
    const LinkPair kinematicsChain {pBaseLink->GetLink(), pEndEffectorLink->GetLink()};
    return _pDescriber->Supports(kinematicsChain);
}

bool PyPostureDescriberBase::Supports(PyManipulatorPtr pmanip) const {
    return _pDescriber->Supports(pmanip->GetManipulator());
}

bool PyPostureDescriberBase::Init(PyLinkPtr pBaseLink, PyLinkPtr pEndEffectorLink) {
    const LinkPair kinematicsChain {pBaseLink->GetLink(), pEndEffectorLink->GetLink()};
    return _pDescriber->Init(kinematicsChain);
}

bool PyPostureDescriberBase::Init(PyManipulatorPtr pmanip) {
    return _pDescriber->Init(pmanip->GetManipulator());
}

object PyPostureDescriberBase::ComputePostureStates()
{
    if(!_pDescriber->ComputePostureStates(_posturestates)) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Failed to compute posture", ORE_Failed);
    }
    return StdVectorToPyList<PostureStateInt>(_posturestates);
}

object PyPostureDescriberBase::ComputePostureStates(object pydofvalues)
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

std::string PyPostureDescriberBase::GetMapDataKey() const {
    return _pDescriber->GetMapDataKey();
}

py::object PyPostureDescriberBase::Interpret(const PostureStateInt state) const {
    rapidjson::Document rIn, rOut;
    openravejson::SetJsonValueByKey(rIn, "posturestate", state, rIn.GetAllocator());
    _pDescriber->SendJSONCommand("Interpret", rIn, rOut);
    return toPyObject(rOut);
}

PyPostureDescriberBasePtr GeneratePostureDescriber(const PyManipulatorPtr& pymanip, std::string interfacename, const bool load) {
    const ManipulatorPtr pmanip = pymanip->GetManipulator();
    const RobotBasePtr probot = pmanip->GetRobot();
    const EnvironmentBasePtr penv = probot->GetEnv();
    const LinkPair linkpair = ExtractEssentialKinematicsChain(pmanip); // if extraction fails, i.e. no revolute joint, then linkpair is {nullptr, nullptr}
    if(linkpair[0] == nullptr || linkpair[1] == nullptr) {
        RAVELOG_WARN_FORMAT("The essential kinematics chain is not valid (linkpair contains nullptr) for manpulator %s of robot %s",
                            pmanip->GetName() % probot->GetName());
        return PyPostureDescriberBasePtr();
    }
    // fe743742269c7dbfe548cb1f3412f658
    std::vector<int> armindices;
    const std::string chainhash = ComputeKinematicsChainHash(linkpair, armindices);    
    // posturedescriber.motoman-gp8l.fe743742269c7dbfe548cb1f3412f658.L0.L6
    if(interfacename.empty()) {
        interfacename = "posturedescriber"; ///< default to OpenRAVE's posture describer
    }
    const std::string describername = interfacename + "." + probot->GetName() + "." + chainhash + "." + linkpair[0]->GetName() + "." + linkpair[1]->GetName();
    const PostureDescriberBasePtr pDescriber = RaveCreatePostureDescriber(penv, interfacename + " " + describername);
    if(pDescriber == nullptr) {
        RAVELOG_WARN_FORMAT("Can not generate posture describer interface \"%s\" for manpulator %s of robot %s", interfacename % pmanip->GetName() % probot->GetName());
    }
    if(!pDescriber->Supports(pmanip)) {
        RAVELOG_WARN_FORMAT("Posture describer interface \"%s\" does not support manpulator %s of robot %s", interfacename % pmanip->GetName() % probot->GetName());
        return PyPostureDescriberBasePtr();
    }
    if(!pDescriber->Init(pmanip)) {
        RAVELOG_WARN_FORMAT("Cannot initialize posture describer interface \"%s\" for manpulator %s of robot %s", interfacename % pmanip->GetName() % probot->GetName());
        return PyPostureDescriberBasePtr();
    }
    if(load) {
        probot->SetPostureDescriber(pmanip, pDescriber); // load automatically
    }
    PyPostureDescriberBasePtr pyDescriber(new PyPostureDescriberBase(pDescriber, toPyEnvironment(pymanip->GetRobot())));
    return pyDescriber;
}

PostureDescriberBasePtr GetPostureDescriber(PyPostureDescriberBasePtr pydescriber)
{
    return !pydescriber ? PostureDescriberBasePtr() : pydescriber->GetPostureDescriber();
}

PyInterfaceBasePtr toPyPostureDescriberBase(PostureDescriberBasePtr pDescriber, PyEnvironmentBasePtr pyenv)
{
    return !pDescriber ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyPostureDescriberBase(pDescriber,pyenv));
}
    
object toPyTrajectory(PostureDescriberBasePtr pDescriber, object opyenv)
{
    extract_<PyEnvironmentBasePtr> pyenv(opyenv);
    if( pyenv.check() ) {
        return py::to_object(toPyPostureDescriberBase(pDescriber, (PyEnvironmentBasePtr)pyenv));
    }
    return py::none_();
}

PyPostureDescriberBasePtr RaveCreatePostureDescriber(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    PostureDescriberBasePtr p = OpenRAVE::RaveCreatePostureDescriber(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyPostureDescriberBasePtr();
    }
    return PyPostureDescriberBasePtr(new PyPostureDescriberBase(p, pyenv));
}

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_FUNCTION_OVERLOADS(GeneratePostureDescriber_overloads, GeneratePostureDescriber, 1, 3)
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_posturedescriber(py::module& m)
#else
void init_openravepy_posturedescriber()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals;  // "..."_a
#endif

    bool (PyPostureDescriberBase::*InitWithTwoLinks)(PyLinkPtr, PyLinkPtr)           = &PyPostureDescriberBase::Init;
    bool (PyPostureDescriberBase::*InitWithManip)(PyManipulatorPtr)                  = &PyPostureDescriberBase::Init;
    bool (PyPostureDescriberBase::*SupportsWithTwoLinks)(PyLinkPtr, PyLinkPtr) const = &PyPostureDescriberBase::Supports;
    bool (PyPostureDescriberBase::*SupportsWithManip)(PyManipulatorPtr)        const = &PyPostureDescriberBase::Supports;
    object (PyPostureDescriberBase::*ComputePostureStates)()                         = &PyPostureDescriberBase::ComputePostureStates;
    object (PyPostureDescriberBase::*ComputePostureStatesWithJointValues)(object)    = &PyPostureDescriberBase::ComputePostureStates;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPostureDescriberBase, OPENRAVE_SHARED_PTR<PyPostureDescriberBase>, PyInterfaceBase>(m, "PostureDescriber", DOXY_CLASS(PostureDescriberBase))
#else
    class_<PyPostureDescriberBase, OPENRAVE_SHARED_PTR<PyPostureDescriberBase>, bases<PyInterfaceBase> >("PostureDescriber", DOXY_CLASS(PostureDescriberBase), no_init)
#endif
    .def("Supports",             SupportsWithTwoLinks                   , PY_ARGS("baselink", "eelink") DOXY_FN(PostureDescriberBase, Supports "const std::array<RobotBase::LinkPtr, 2>& kinematicsChain"))
    .def("Supports",             SupportsWithManip                      , PY_ARGS("manipulator")        DOXY_FN(PostureDescriberBase, Supports "const RobotBase::ManipulatorPtr& pmanip"))
    .def("Init",                 InitWithTwoLinks                       , PY_ARGS("baselink", "eelink") DOXY_FN(PostureDescriberBase, Init "const std::array<RobotBase::LinkPtr, 2>& kinematicsChain"))
    .def("Init",                 InitWithManip                          , PY_ARGS("manipulator")        DOXY_FN(PostureDescriberBase, Init "const RobotBase::ManipulatorPtr& pmanip"))

    .def("ComputePostureStates", ComputePostureStates                   ,                               DOXY_FN(PostureDescriberBase, ComputePostureStates ""))
    .def("ComputePostureStates", ComputePostureStatesWithJointValues    , PY_ARGS("dofvalues")          DOXY_FN(PostureDescriberBase, ComputePostureStates "const std::vector<double>& dofvalues"))
    .def("GetMapDataKey"       , &PyPostureDescriberBase::GetMapDataKey ,                               DOXY_FN(PostureDescriberBase, GetMapDataKey ""))
    .def("Interpret"           , &PyPostureDescriberBase::Interpret     , PY_ARGS("posturestate")       DOXY_FN(PostureDescriberBase, Interpret ""))
    ;

    PyPostureDescriberBasePtr (*openravepyRaveCreatePostureDescriber)(PyEnvironmentBasePtr pyenv, const std::string& name) = &RaveCreatePostureDescriber;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("GeneratePostureDescriber", &GeneratePostureDescriber,
          "manip"_a,
          "interfacename"_a = "",
          "load"_a = false,
          DOXY_FN1(GeneratePostureDescriber)
    );
    m.def("RaveCreatePostureDescriber", openravepyRaveCreatePostureDescriber          , PY_ARGS("env", "name")                    DOXY_FN1(RaveCreatePostureDescriber));
#else
    def("GeneratePostureDescriber"  , GeneratePostureDescriber, GeneratePostureDescriber_overloads(PY_ARGS("manip", "interfacename", "load") DOXY_FN1(GeneratePostureDescriber)));
    def("RaveCreatePostureDescriber", openravepyRaveCreatePostureDescriber          , PY_ARGS("env", "name")                    DOXY_FN1(RaveCreatePostureDescriber));
#endif
}

} // namespace openravepy
