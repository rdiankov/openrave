// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include <openravepy/openravepy_kinbody.h>
#include <openravepy/openravepy_collisionreport.h>
#include <openravepy/openravepy_controllerbase.h>
#include <openravepy/openravepy_configurationspecification.h>
#include <openravepy/openravepy_jointinfo.h>
#include <openravepy/openravepy_environmentbase.h>
#include <openravepy/openravepy_iksolverbase.h>
#include <openravepy/openravepy_manipulatorinfo.h>
#include <openravepy/openravepy_robotbase.h>

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

namespace numeric = py::numeric;

PyManipulatorInfo::PyManipulatorInfo() {
    _tLocalTool = ReturnTransform(Transform());
    _vChuckingDirection = py::empty_array_astype<dReal>();
    _vdirection = toPyVector3(Vector(0,0,1));
    _vGripperJointNames = py::list();
}

PyManipulatorInfo::PyManipulatorInfo(const RobotBase::ManipulatorInfo& info) {
    _Update(info);
}

void PyManipulatorInfo::_Update(const RobotBase::ManipulatorInfo& info) {
    _name = ConvertStringToUnicode(info._name);
    _sBaseLinkName = ConvertStringToUnicode(info._sBaseLinkName);
    _sEffectorLinkName = ConvertStringToUnicode(info._sEffectorLinkName);
    _tLocalTool = ReturnTransform(info._tLocalTool);
    _vChuckingDirection = toPyArray(info._vChuckingDirection);
    _vdirection = toPyVector3(info._vdirection);
    _sIkSolverXMLId = info._sIkSolverXMLId;
    py::list vGripperJointNames;
    FOREACHC(itname, info._vGripperJointNames) {
        vGripperJointNames.append(ConvertStringToUnicode(*itname));
    }
    _vGripperJointNames = vGripperJointNames;
}

RobotBase::ManipulatorInfoPtr PyManipulatorInfo::GetManipulatorInfo() const
{
    RobotBase::ManipulatorInfoPtr pinfo(new RobotBase::ManipulatorInfo());
    pinfo->_name = py::extract<std::string>(_name);
    pinfo->_sBaseLinkName = py::extract<std::string>(_sBaseLinkName);
    pinfo->_sEffectorLinkName = py::extract<std::string>(_sEffectorLinkName);
    pinfo->_tLocalTool = ExtractTransform(_tLocalTool);
    pinfo->_vChuckingDirection = ExtractArray<dReal>(_vChuckingDirection);
    pinfo->_vdirection = ExtractVector3(_vdirection);
    pinfo->_sIkSolverXMLId = _sIkSolverXMLId;
    pinfo->_vGripperJointNames = ExtractArray<std::string>(_vGripperJointNames);
    return pinfo;
}

object PyManipulatorInfo::SerializeJSON(dReal fUnitScale, object options)
{
    rapidjson::Document doc;
    RobotBase::ManipulatorInfoPtr pInfo = GetManipulatorInfo();
    pInfo->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(options, 0));
    return toPyObject(doc);
}

void PyManipulatorInfo::DeserializeJSON(object obj, dReal fUnitScale)
{
    rapidjson::Document doc;
    toRapidJSONValue(obj, doc, doc.GetAllocator());
    RobotBase::ManipulatorInfo info;
    info.DeserializeJSON(doc, fUnitScale);
    _Update(info);
}

PyManipulatorInfoPtr toPyManipulatorInfo(const RobotBase::ManipulatorInfo& manipulatorinfo)
{
    return PyManipulatorInfoPtr(new PyManipulatorInfo(manipulatorinfo));
}

PyAttachedSensorInfo::PyAttachedSensorInfo() {
}

PyAttachedSensorInfo::PyAttachedSensorInfo(const RobotBase::AttachedSensorInfo& info) {
    _Update(info);
}

void PyAttachedSensorInfo::_Update(const RobotBase::AttachedSensorInfo& info) {
    _name = ConvertStringToUnicode(info._name);
    _linkname = ConvertStringToUnicode(info._linkname);
    _trelative = ReturnTransform(info._trelative);
    _sensorname = ConvertStringToUnicode(info._sensorname);
    _sensorgeometry = toPySensorGeometry(info._sensorgeometry);
}

RobotBase::AttachedSensorInfoPtr PyAttachedSensorInfo::GetAttachedSensorInfo() const
{
    RobotBase::AttachedSensorInfoPtr pinfo(new RobotBase::AttachedSensorInfo());
    pinfo->_name = py::extract<std::string>(_name);
    pinfo->_linkname = py::extract<std::string>(_linkname);
    pinfo->_trelative = ExtractTransform(_trelative);
    pinfo->_sensorname = py::extract<std::string>(_sensorname);
    if(!!_sensorgeometry){
        pinfo->_sensorgeometry = _sensorgeometry->GetGeometry();
    }
    return pinfo;
}

object PyAttachedSensorInfo::SerializeJSON(dReal fUnitScale, object options)
{
    rapidjson::Document doc;
    RobotBase::AttachedSensorInfoPtr pInfo = GetAttachedSensorInfo();
    pInfo->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(options, 0));
    return toPyObject(doc);
}

void PyAttachedSensorInfo::DeserializeJSON(object obj, dReal fUnitScale)
{
    rapidjson::Document doc;
    toRapidJSONValue(obj, doc, doc.GetAllocator());
    RobotBase::AttachedSensorInfo info;
    info.DeserializeJSON(doc, fUnitScale);
    _Update(info);
}

PyAttachedSensorInfoPtr toPyAttachedSensorInfo(const RobotBase::AttachedSensorInfo& attachedSensorinfo)
{
    return PyAttachedSensorInfoPtr(new PyAttachedSensorInfo(attachedSensorinfo));
}

PyConnectedBodyInfo::PyConnectedBodyInfo() {
}

PyConnectedBodyInfo::PyConnectedBodyInfo(const RobotBase::ConnectedBodyInfo& info) {
    _Update(info);
}

void PyConnectedBodyInfo::_Update(const RobotBase::ConnectedBodyInfo& info)
{
    _name = ConvertStringToUnicode(info._name);
    _linkname = ConvertStringToUnicode(info._linkname);
    _trelative = ReturnTransform(info._trelative);
    _uri = ConvertStringToUnicode(info._uri);

    py::list linkInfos;
    FOREACH(itlinkinfo, info._vLinkInfos) {
        linkInfos.append(toPyLinkInfo(**itlinkinfo));
    }
    _linkInfos = linkInfos;

    py::list jointInfos;
    FOREACH(itjointinfo, info._vJointInfos) {
        jointInfos.append(toPyJointInfo(**itjointinfo));
    }
    _jointInfos = jointInfos;

    py::list manipulatorInfos;
    FOREACH(itmanipulatorinfo, info._vManipulatorInfos) {
        manipulatorInfos.append(toPyManipulatorInfo(**itmanipulatorinfo));
    }
    _manipulatorInfos = manipulatorInfos;

    py::list attachedSensorInfos;
    FOREACH(itattachedSensorinfo, info._vAttachedSensorInfos) {
        attachedSensorInfos.append(toPyAttachedSensorInfo(**itattachedSensorinfo));
    }
    _attachedSensorInfos = attachedSensorInfos;
}

RobotBase::ConnectedBodyInfoPtr PyConnectedBodyInfo::GetConnectedBodyInfo() const
{
    RobotBase::ConnectedBodyInfoPtr pinfo(new RobotBase::ConnectedBodyInfo());
    pinfo->_name = py::extract<std::string>(_name);
    pinfo->_linkname = py::extract<std::string>(_linkname);
    pinfo->_trelative = ExtractTransform(_trelative);
    pinfo->_uri = py::extract<std::string>(_uri);
    // extract all the infos
    return pinfo;
}

object PyConnectedBodyInfo::SerializeJSON(dReal fUnitScale, object options)
{
    rapidjson::Document doc;
    RobotBase::ConnectedBodyInfoPtr pInfo = GetConnectedBodyInfo();
    pInfo->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(options, 0));
    return toPyObject(doc);
}

void PyConnectedBodyInfo::DeserializeJSON(object obj, dReal fUnitScale)
{
    rapidjson::Document doc;
    toRapidJSONValue(obj, doc, doc.GetAllocator());
    RobotBase::ConnectedBodyInfo info;
    info.DeserializeJSON(doc, fUnitScale);
    _Update(info);
}

PyConnectedBodyInfoPtr toPyConnectedBodyInfo(const RobotBase::ConnectedBodyInfo& connectedBodyInfo)
{
    return PyConnectedBodyInfoPtr(new PyConnectedBodyInfo(connectedBodyInfo));
}

RobotBasePtr PyRobotBase::GetRobot() {
    return _probot;
}

PyRobotBase::PyManipulator::PyManipulator(RobotBase::ManipulatorPtr pmanip, PyEnvironmentBasePtr pyenv) : _pmanip(pmanip),_pyenv(pyenv) {
}
PyRobotBase::PyManipulator::~PyManipulator() {
}

RobotBase::ManipulatorPtr PyRobotBase::PyManipulator::GetManipulator() const {
    return _pmanip;
}

object PyRobotBase::PyManipulator::GetTransform() const {
    return ReturnTransform(_pmanip->GetTransform());
}

object PyRobotBase::PyManipulator::GetTransformPose() const {
    return toPyArray(_pmanip->GetTransform());
}

object PyRobotBase::PyManipulator::GetVelocity() const {
    std::pair<Vector, Vector> velocity;
    velocity = _pmanip->GetVelocity();
    boost::array<dReal,6> v = {{ velocity.first.x, velocity.first.y, velocity.first.z, velocity.second.x, velocity.second.y, velocity.second.z}};
    return toPyArray<dReal,6>(v);
}

object PyRobotBase::PyManipulator::GetName() const {
    return ConvertStringToUnicode(_pmanip->GetName());
}

void PyRobotBase::PyManipulator::SetName(const std::string& s) {
    _pmanip->SetName(s);
}

PyRobotBasePtr PyRobotBase::PyManipulator::GetRobot() {
    return PyRobotBasePtr(new PyRobotBase(_pmanip->GetRobot(),_pyenv));
}

bool PyRobotBase::PyManipulator::SetIkSolver(PyIkSolverBasePtr iksolver) {
    return _pmanip->SetIkSolver(openravepy::GetIkSolver(iksolver));
}
object PyRobotBase::PyManipulator::GetIkSolver() {
    return py::to_object(openravepy::toPyIkSolver(_pmanip->GetIkSolver(),_pyenv));
}

object PyRobotBase::PyManipulator::GetBase() {
    return toPyKinBodyLink(_pmanip->GetBase(),_pyenv);
}
object PyRobotBase::PyManipulator::GetEndEffector() {
    return toPyKinBodyLink(_pmanip->GetEndEffector(),_pyenv);
}
void PyRobotBase::PyManipulator::ReleaseAllGrabbed() {
    _pmanip->ReleaseAllGrabbed();
}
object PyRobotBase::PyManipulator::GetGraspTransform() {
    RAVELOG_WARN("Robot.Manipulator.GetGraspTransform deprecated, use GetLocalToolTransform\n");
    return ReturnTransform(_pmanip->GetLocalToolTransform());
}
object PyRobotBase::PyManipulator::GetLocalToolTransform() {
    return ReturnTransform(_pmanip->GetLocalToolTransform());
}
object PyRobotBase::PyManipulator::GetLocalToolTransformPose() {
    return toPyArray(_pmanip->GetLocalToolTransform());
}
void PyRobotBase::PyManipulator::SetLocalToolTransform(object otrans) {
    _pmanip->SetLocalToolTransform(ExtractTransform(otrans));
}
void PyRobotBase::PyManipulator::SetLocalToolDirection(object odirection) {
    _pmanip->SetLocalToolDirection(ExtractVector3(odirection));
}
void PyRobotBase::PyManipulator::SetClosingDirection(object oclosingdirection)
{
    RAVELOG_WARN("SetClosingDirection is deprecated, use SetChuckingDirection\n");
    _pmanip->SetChuckingDirection(ExtractArray<dReal>(oclosingdirection));
}
void PyRobotBase::PyManipulator::SetChuckingDirection(object ochuckingdirection)
{
    _pmanip->SetChuckingDirection(ExtractArray<dReal>(ochuckingdirection));
}
py::array_int PyRobotBase::PyManipulator::GetGripperJoints() {
    RAVELOG_DEBUG("GetGripperJoints is deprecated, use GetGripperIndices\n");
    return toPyArray<int>(_pmanip->GetGripperIndices());
}
py::array_int PyRobotBase::PyManipulator::GetGripperIndices() {
    return toPyArray(_pmanip->GetGripperIndices());
}
py::array_int PyRobotBase::PyManipulator::GetArmJoints() {
    RAVELOG_DEBUG("GetArmJoints is deprecated, use GetArmIndices\n");
    return toPyArray(_pmanip->GetArmIndices());
}
py::array_int PyRobotBase::PyManipulator::GetArmIndices() {
    return toPyArray(_pmanip->GetArmIndices());
}
object PyRobotBase::PyManipulator::GetArmDOFValues()
{
    if( _pmanip->GetArmDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _pmanip->GetArmDOFValues(values);
    return toPyArray(values);
}
object PyRobotBase::PyManipulator::GetGripperDOFValues()
{
    if( _pmanip->GetGripperDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _pmanip->GetGripperDOFValues(values);
    return toPyArray(values);
}
int PyRobotBase::PyManipulator::GetArmDOF() {
    return _pmanip->GetArmDOF();
}
int PyRobotBase::PyManipulator::GetGripperDOF() {
    return _pmanip->GetGripperDOF();
}
object PyRobotBase::PyManipulator::GetClosingDirection() {
    RAVELOG_WARN("GetClosingDirection is deprecated, use GetChuckingDirection\n");
    return toPyArray(_pmanip->GetChuckingDirection());
}
object PyRobotBase::PyManipulator::GetChuckingDirection() {
    return toPyArray(_pmanip->GetChuckingDirection());
}
object PyRobotBase::PyManipulator::GetDirection() {
    return toPyVector3(_pmanip->GetLocalToolDirection());
}
object PyRobotBase::PyManipulator::GetLocalToolDirection() {
    return toPyVector3(_pmanip->GetLocalToolDirection());
}
bool PyRobotBase::PyManipulator::IsGrabbing(PyKinBodyPtr pbody) {
    return _pmanip->IsGrabbing(*pbody->GetBody());
}

int PyRobotBase::PyManipulator::GetNumFreeParameters() const {
    RAVELOG_WARN("Manipulator::GetNumFreeParameters() is deprecated\n");
    return _pmanip->GetIkSolver()->GetNumFreeParameters();
}

object PyRobotBase::PyManipulator::GetFreeParameters() const {
    RAVELOG_WARN("Manipulator::GetFreeParameters() is deprecated\n");
    if( _pmanip->GetIkSolver()->GetNumFreeParameters() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _pmanip->GetIkSolver()->GetFreeParameters(values);
    return toPyArray(values);
}

bool PyRobotBase::PyManipulator::_FindIKSolution(const IkParameterization& ikparam, std::vector<dReal>& solution, int filteroptions, bool releasegil) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolution(ikparam,solution,filteroptions);

}
bool PyRobotBase::PyManipulator::_FindIKSolution(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, std::vector<dReal>& solution, int filteroptions, bool releasegil) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolution(ikparam,vFreeParameters, solution,filteroptions);
}
bool PyRobotBase::PyManipulator::_FindIKSolution(const IkParameterization& ikparam, int filteroptions, IkReturn& ikreturn, bool releasegil) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolution(ikparam,filteroptions,IkReturnPtr(&ikreturn,utils::null_deleter()));
}
bool PyRobotBase::PyManipulator::_FindIKSolution(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturn& ikreturn, bool releasegil) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolution(ikparam,vFreeParameters, filteroptions,IkReturnPtr(&ikreturn,utils::null_deleter()));
}

bool PyRobotBase::PyManipulator::_FindIKSolutions(const IkParameterization& ikparam, std::vector<std::vector<dReal> >& solutions, int filteroptions, bool releasegil) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolutions(ikparam,solutions,filteroptions);
}
bool PyRobotBase::PyManipulator::_FindIKSolutions(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal> >& solutions, int filteroptions, bool releasegil) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolutions(ikparam,vFreeParameters,solutions,filteroptions);
}
bool PyRobotBase::PyManipulator::_FindIKSolutions(const IkParameterization& ikparam, int filteroptions, std::vector<IkReturnPtr>& vikreturns, bool releasegil) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolutions(ikparam,filteroptions,vikreturns);
}
bool PyRobotBase::PyManipulator::_FindIKSolutions(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& vikreturns, bool releasegil) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolutions(ikparam,vFreeParameters,filteroptions,vikreturns);
}

object PyRobotBase::PyManipulator::FindIKSolution(object oparam, int filteroptions, bool ikreturn, bool releasegil) const
{
    IkParameterization ikparam;
    EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
    if( ExtractIkParameterization(oparam,ikparam) ) {
        if( ikreturn ) {
            IkReturn ikreturn(IKRA_Reject);
            _FindIKSolution(ikparam,filteroptions,ikreturn,releasegil);
            return openravepy::toPyIkReturn(ikreturn);
        }
        else {
            std::vector<dReal> solution;
            if( !_FindIKSolution(ikparam,solution,filteroptions,releasegil) ) {
                return py::none_();
            }
            return toPyArray(solution);
        }
    }
    // assume transformation matrix
    else {
        if( ikreturn ) {
            IkReturn ikreturn(IKRA_Reject);
            _FindIKSolution(ExtractTransform(oparam),filteroptions,ikreturn,releasegil);
            return openravepy::toPyIkReturn(ikreturn);
        }
        else {
            std::vector<dReal> solution;
            if( !_FindIKSolution(ExtractTransform(oparam),solution,filteroptions,releasegil) ) {
                return py::none_();
            }
            return toPyArray(solution);
        }
    }
}

object PyRobotBase::PyManipulator::FindIKSolution(object oparam, object freeparams, int filteroptions, bool ikreturn, bool releasegil) const
{
    std::vector<dReal> vfreeparams = ExtractArray<dReal>(freeparams);
    IkParameterization ikparam;
    EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
    if( ExtractIkParameterization(oparam,ikparam) ) {
        if( ikreturn ) {
            IkReturn ikreturn(IKRA_Reject);
            _FindIKSolution(ikparam,vfreeparams,filteroptions,ikreturn,releasegil);
            return openravepy::toPyIkReturn(ikreturn);
        }
        else {
            std::vector<dReal> solution;
            if( !_FindIKSolution(ikparam,vfreeparams,solution,filteroptions,releasegil) ) {
                return py::none_();
            }
            return toPyArray(solution);
        }
    }
    // assume transformation matrix
    else {
        if( ikreturn ) {
            IkReturn ikreturn(IKRA_Reject);
            _FindIKSolution(ExtractTransform(oparam),vfreeparams,filteroptions,ikreturn,releasegil);
            return openravepy::toPyIkReturn(ikreturn);
        }
        else {
            std::vector<dReal> solution;
            if( !_FindIKSolution(ExtractTransform(oparam),vfreeparams, solution,filteroptions,releasegil) ) {
                return py::none_();
            }
            return toPyArray(solution);
        }
    }
}

object PyRobotBase::PyManipulator::FindIKSolutions(object oparam, int filteroptions, bool ikreturn, bool releasegil) const
{
    IkParameterization ikparam;
    EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
    if( ikreturn ) {
        std::vector<IkReturnPtr> vikreturns;
        if( ExtractIkParameterization(oparam,ikparam) ) {
            if( !_FindIKSolutions(ikparam,filteroptions,vikreturns,releasegil) ) {
                return py::list();
            }
        }
        // assume transformation matrix
        else if( !_FindIKSolutions(ExtractTransform(oparam),filteroptions,vikreturns,releasegil) ) {
            return py::list();
        }

        py::list oikreturns;
        FOREACH(it,vikreturns) {
            oikreturns.append(openravepy::toPyIkReturn(**it));
        }
        return oikreturns;
    }
    else {
        std::vector<std::vector<dReal> > vsolutions;
        if( ExtractIkParameterization(oparam,ikparam) ) {
            if( !_FindIKSolutions(ikparam,vsolutions,filteroptions,releasegil) ) {
                return py::empty_array_astype<dReal>();
            }
        }
        // assume transformation matrix
        else if( !_FindIKSolutions(ExtractTransform(oparam),vsolutions,filteroptions,releasegil) ) {
            return py::empty_array_astype<dReal>();
        }

        npy_intp dims[] = { npy_intp(vsolutions.size()), npy_intp(_pmanip->GetArmIndices().size()) };
        PyObject *pysolutions = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* ppos = (dReal*)PyArray_DATA(pysolutions);
        FOREACH(itsol,vsolutions) {
            BOOST_ASSERT(itsol->size()==size_t(dims[1]));
            std::copy(itsol->begin(),itsol->end(),ppos);
            ppos += itsol->size();
        }
        return py::to_array_astype<dReal>(pysolutions);
    }
}

object PyRobotBase::PyManipulator::FindIKSolutions(object oparam, object freeparams, int filteroptions, bool ikreturn, bool releasegil) const
{
    std::vector<dReal> vfreeparams = ExtractArray<dReal>(freeparams);
    IkParameterization ikparam;
    EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
    if( ikreturn ) {
        std::vector<IkReturnPtr> vikreturns;
        if( ExtractIkParameterization(oparam,ikparam) ) {
            if( !_FindIKSolutions(ikparam,vfreeparams,filteroptions,vikreturns,releasegil) ) {
                return py::list();
            }
        }
        // assume transformation matrix
        else if( !_FindIKSolutions(ExtractTransform(oparam),vfreeparams,filteroptions,vikreturns,releasegil) ) {
            return py::list();
        }

        py::list oikreturns;
        FOREACH(it,vikreturns) {
            oikreturns.append(openravepy::toPyIkReturn(**it));
        }
        return oikreturns;
    }
    else {
        std::vector<std::vector<dReal> > vsolutions;
        if( ExtractIkParameterization(oparam,ikparam) ) {
            if( !_FindIKSolutions(ikparam,vfreeparams,vsolutions,filteroptions,releasegil) ) {
                return py::empty_array_astype<dReal>();
            }
        }
        // assume transformation matrix
        else if( !_FindIKSolutions(ExtractTransform(oparam),vfreeparams, vsolutions,filteroptions,releasegil) ) {
            return py::empty_array_astype<dReal>();
        }

        npy_intp dims[] = { npy_intp(vsolutions.size()), npy_intp(_pmanip->GetArmIndices().size()) };
        PyObject *pysolutions = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* ppos = (dReal*)PyArray_DATA(pysolutions);
        FOREACH(itsol,vsolutions) {
            BOOST_ASSERT(itsol->size()==size_t(dims[1]));
            std::copy(itsol->begin(),itsol->end(),ppos);
            ppos += itsol->size();
        }
        return py::to_array_astype<dReal>(pysolutions);
    }
}

object PyRobotBase::PyManipulator::GetIkParameterization(object oparam, bool inworld)
{
    IkParameterization ikparam;
    if( ExtractIkParameterization(oparam,ikparam) ) {
        return toPyIkParameterization(_pmanip->GetIkParameterization(ikparam,inworld));
    }
    // must be IkParameterizationType
    return toPyIkParameterization(_pmanip->GetIkParameterization((IkParameterizationType)extract<IkParameterizationType>(oparam),inworld));
}

object PyRobotBase::PyManipulator::GetChildJoints() {
    std::vector<KinBody::JointPtr> vjoints;
    _pmanip->GetChildJoints(vjoints);
    py::list joints;
    FOREACH(itjoint,vjoints) {
        joints.append(toPyKinBodyJoint(*itjoint,_pyenv));
    }
    return joints;
}
object PyRobotBase::PyManipulator::GetChildDOFIndices() {
    std::vector<int> vdofindices;
    _pmanip->GetChildDOFIndices(vdofindices);
    py::list dofindices;
    FOREACH(itindex,vdofindices) {
        dofindices.append(*itindex);
    }
    return dofindices;
}

object PyRobotBase::PyManipulator::GetChildLinks() {
    std::vector<KinBody::LinkPtr> vlinks;
    _pmanip->GetChildLinks(vlinks);
    py::list links;
    FOREACH(itlink,vlinks) {
        links.append(toPyKinBodyLink(*itlink,_pyenv));
    }
    return links;
}

bool PyRobotBase::PyManipulator::IsChildLink(object pylink)
{
    CHECK_POINTER(pylink);
    return _pmanip->IsChildLink(*GetKinBodyLink(pylink));
}

object PyRobotBase::PyManipulator::GetIndependentLinks() {
    std::vector<KinBody::LinkPtr> vlinks;
    _pmanip->GetIndependentLinks(vlinks);
    py::list links;
    FOREACH(itlink,vlinks) {
        links.append(toPyKinBodyLink(*itlink,_pyenv));
    }
    return links;
}

object PyRobotBase::PyManipulator::GetArmConfigurationSpecification(const std::string& interpolation) const {
    return py::to_object(openravepy::toPyConfigurationSpecification(_pmanip->GetArmConfigurationSpecification(interpolation)));
}

object PyRobotBase::PyManipulator::GetIkConfigurationSpecification(IkParameterizationType iktype, const std::string& interpolation) const {
    return py::to_object(openravepy::toPyConfigurationSpecification(_pmanip->GetIkConfigurationSpecification(iktype, interpolation)));
}

bool PyRobotBase::PyManipulator::CheckEndEffectorCollision(PyCollisionReportPtr pyreport) const
{
    bool bcollision = _pmanip->CheckEndEffectorCollision(openravepy::GetCollisionReport(pyreport));
    openravepy::UpdateCollisionReport(pyreport,_pyenv);
    return bcollision;
}

bool PyRobotBase::PyManipulator::CheckEndEffectorCollision(object otrans, PyCollisionReportPtr pyreport, int numredundantsamples) const
{
    bool bCollision;
    IkParameterization ikparam;
    if( ExtractIkParameterization(otrans,ikparam) ) {
        bCollision = _pmanip->CheckEndEffectorCollision(ikparam, !pyreport ? CollisionReportPtr() : openravepy::GetCollisionReport(pyreport), numredundantsamples);
    }
    else {
        bCollision = _pmanip->CheckEndEffectorCollision(ExtractTransform(otrans),!pyreport ? CollisionReportPtr() : openravepy::GetCollisionReport(pyreport), numredundantsamples);
    }
    if( !!pyreport ) {
        openravepy::UpdateCollisionReport(pyreport,_pyenv);
    }
    return bCollision;
}

bool PyRobotBase::PyManipulator::CheckEndEffectorSelfCollision(PyCollisionReportPtr pyreport) const
{
    BOOST_ASSERT(0);
    bool bcollision = true;//_pmanip->CheckEndEffectorSelfCollision(openravepy::GetCollisionReport(pyreport));
    openravepy::UpdateCollisionReport(pyreport,_pyenv);
    return bcollision;
}

bool PyRobotBase::PyManipulator::CheckEndEffectorSelfCollision(object otrans, PyCollisionReportPtr pyreport, int numredundantsamples, bool ignoreManipulatorLinks) const
{
    bool bCollision;
    IkParameterization ikparam;
    if( ExtractIkParameterization(otrans,ikparam) ) {
        bCollision = _pmanip->CheckEndEffectorSelfCollision(ikparam, !pyreport ? CollisionReportPtr() : openravepy::GetCollisionReport(pyreport), numredundantsamples, ignoreManipulatorLinks);
    }
    else {
        bCollision = _pmanip->CheckEndEffectorSelfCollision(ExtractTransform(otrans),!pyreport ? CollisionReportPtr() : openravepy::GetCollisionReport(pyreport), numredundantsamples, ignoreManipulatorLinks);
    }
    if( !!pyreport ) {
        openravepy::UpdateCollisionReport(pyreport,_pyenv);
    }
    return bCollision;
}

bool PyRobotBase::PyManipulator::CheckIndependentCollision() const
{
    return _pmanip->CheckIndependentCollision();
}
bool PyRobotBase::PyManipulator::CheckIndependentCollision(PyCollisionReportPtr pReport) const
{
    bool bCollision = _pmanip->CheckIndependentCollision(openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

object PyRobotBase::PyManipulator::CalculateJacobian()
{
    std::vector<dReal> vjacobian;
    _pmanip->CalculateJacobian(vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _pmanip->GetArmIndices().size();
    return toPyArray(vjacobian,dims);
}

object PyRobotBase::PyManipulator::CalculateRotationJacobian()
{
    std::vector<dReal> vjacobian;
    _pmanip->CalculateRotationJacobian(vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 4; dims[1] = _pmanip->GetArmIndices().size();
    return toPyArray(vjacobian,dims);
}

object PyRobotBase::PyManipulator::CalculateAngularVelocityJacobian()
{
    std::vector<dReal> vjacobian;
    _pmanip->CalculateAngularVelocityJacobian(vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _pmanip->GetArmIndices().size();
    return toPyArray(vjacobian,dims);
}

object PyRobotBase::PyManipulator::GetInfo() {
    return py::to_object(PyManipulatorInfoPtr(new PyManipulatorInfo(_pmanip->GetInfo())));
}

std::string PyRobotBase::PyManipulator::GetStructureHash() const {
    return _pmanip->GetStructureHash();
}
std::string PyRobotBase::PyManipulator::GetKinematicsStructureHash() const {
    return _pmanip->GetKinematicsStructureHash();
}
std::string PyRobotBase::PyManipulator::GetInverseKinematicsStructureHash(IkParameterizationType iktype) const {
    return _pmanip->GetInverseKinematicsStructureHash(iktype);
}

std::string PyRobotBase::PyManipulator::__repr__() {
    return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s').GetManipulator('%s')")%RaveGetEnvironmentId(_pmanip->GetRobot()->GetEnv())%_pmanip->GetRobot()->GetName()%_pmanip->GetName());
}
std::string PyRobotBase::PyManipulator::__str__() {
    return boost::str(boost::format("<manipulator:%s, parent=%s>")%_pmanip->GetName()%_pmanip->GetRobot()->GetName());
}
object PyRobotBase::PyManipulator::__unicode__() {
    return ConvertStringToUnicode(__str__());
}
bool PyRobotBase::PyManipulator::__eq__(OPENRAVE_SHARED_PTR<PyManipulator> p) {
    return !!p && _pmanip==p->_pmanip;
}
bool PyRobotBase::PyManipulator::__ne__(OPENRAVE_SHARED_PTR<PyManipulator> p) {
    return !p || _pmanip!=p->_pmanip;
}
long PyRobotBase::PyManipulator::__hash__() {
    return static_cast<long>(uintptr_t(_pmanip.get()));
}

typedef OPENRAVE_SHARED_PTR<PyRobotBase::PyManipulator> PyManipulatorPtr;
PyManipulatorPtr PyRobotBase::_GetManipulator(RobotBase::ManipulatorPtr pmanip) {
    return !pmanip ? PyManipulatorPtr() : PyManipulatorPtr(new PyManipulator(pmanip,_pyenv));
}

PyRobotBase::PyAttachedSensor::PyAttachedSensor(RobotBase::AttachedSensorPtr pattached, PyEnvironmentBasePtr pyenv) : _pattached(pattached),_pyenv(pyenv) {
}
PyRobotBase::PyAttachedSensor::~PyAttachedSensor() {
}

RobotBase::AttachedSensorPtr PyRobotBase::PyAttachedSensor::GetAttachedSensor() const {
    return _pattached;
}
object PyRobotBase::PyAttachedSensor::GetSensor() {
    return py::to_object(openravepy::toPySensor(_pattached->GetSensor(),_pyenv));
}
object PyRobotBase::PyAttachedSensor::GetAttachingLink() const {
    return toPyKinBodyLink(_pattached->GetAttachingLink(), _pyenv);
}
object PyRobotBase::PyAttachedSensor::GetRelativeTransform() const {
    return ReturnTransform(_pattached->GetRelativeTransform());
}
object PyRobotBase::PyAttachedSensor::GetTransform() const {
    return ReturnTransform(_pattached->GetTransform());
}
object PyRobotBase::PyAttachedSensor::GetTransformPose() const {
    return toPyArray(_pattached->GetTransform());
}
PyRobotBasePtr PyRobotBase::PyAttachedSensor::GetRobot() const {
    return _pattached->GetRobot() ? PyRobotBasePtr() : PyRobotBasePtr(new PyRobotBase(_pattached->GetRobot(), _pyenv));
}
object PyRobotBase::PyAttachedSensor::GetName() const {
    return ConvertStringToUnicode(_pattached->GetName());
}

object PyRobotBase::PyAttachedSensor::GetData()
{
    return openravepy::toPySensorData(_pattached->GetSensor(),_pyenv);
}

void PyRobotBase::PyAttachedSensor::SetRelativeTransform(object transform) {
    _pattached->SetRelativeTransform(ExtractTransform(transform));
}
std::string PyRobotBase::PyAttachedSensor::GetStructureHash() const {
    return _pattached->GetStructureHash();
}

void PyRobotBase::PyAttachedSensor::UpdateInfo(SensorBase::SensorType type) {
    _pattached->UpdateInfo(type);
}

object PyRobotBase::PyAttachedSensor::UpdateAndGetInfo(SensorBase::SensorType type) {
    return py::to_object(PyAttachedSensorInfoPtr(new PyAttachedSensorInfo(_pattached->UpdateAndGetInfo(type))));
}

object PyRobotBase::PyAttachedSensor::GetInfo() {
    return py::to_object(PyAttachedSensorInfoPtr(new PyAttachedSensorInfo(_pattached->GetInfo())));
}

std::string PyRobotBase::PyAttachedSensor::__repr__() {
    return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s').GetAttachedSensor('%s')")%RaveGetEnvironmentId(_pattached->GetRobot()->GetEnv())%_pattached->GetRobot()->GetName()%_pattached->GetName());
}
std::string PyRobotBase::PyAttachedSensor::__str__() {
    return boost::str(boost::format("<attachedsensor:%s, parent=%s>")%_pattached->GetName()%_pattached->GetRobot()->GetName());
}
object PyRobotBase::PyAttachedSensor::__unicode__() {
    return ConvertStringToUnicode(__str__());
}
bool PyRobotBase::PyAttachedSensor::__eq__(OPENRAVE_SHARED_PTR<PyAttachedSensor> p) {
    return !!p && _pattached==p->_pattached;
}
bool PyRobotBase::PyAttachedSensor::__ne__(OPENRAVE_SHARED_PTR<PyAttachedSensor> p) {
    return !p || _pattached!=p->_pattached;
}
long PyRobotBase::PyAttachedSensor::__hash__() {
    return static_cast<long>(uintptr_t(_pattached.get()));
}

typedef OPENRAVE_SHARED_PTR<PyRobotBase::PyAttachedSensor> PyAttachedSensorPtr;
OPENRAVE_SHARED_PTR<PyRobotBase::PyAttachedSensor> PyRobotBase::_GetAttachedSensor(RobotBase::AttachedSensorPtr pattachedsensor)
{
    return !pattachedsensor ? PyAttachedSensorPtr() : PyAttachedSensorPtr(new PyAttachedSensor(pattachedsensor, _pyenv));
}

PyRobotBase::PyConnectedBody::PyConnectedBody(RobotBase::ConnectedBodyPtr pconnected, PyEnvironmentBasePtr pyenv) : _pconnected(pconnected),
    _pyenv(pyenv) {
}

PyRobotBase::PyConnectedBody::~PyConnectedBody() {
}

RobotBase::ConnectedBodyPtr PyRobotBase::PyConnectedBody::GetConnectedBody() const {
    return _pconnected;
}

object PyRobotBase::PyConnectedBody::GetName() {
    return ConvertStringToUnicode(_pconnected->GetName());
}

object PyRobotBase::PyConnectedBody::GetInfo() {
    return py::to_object(PyConnectedBodyInfoPtr(new PyConnectedBodyInfo(_pconnected->GetInfo())));
}

bool PyRobotBase::PyConnectedBody::SetActive(bool active) {
    return _pconnected->SetActive(active);
}

bool PyRobotBase::PyConnectedBody::IsActive() {
    return _pconnected->IsActive();
}
object PyRobotBase::PyConnectedBody::GetTransform() const {
    return ReturnTransform(_pconnected->GetTransform());
}
object PyRobotBase::PyConnectedBody::GetTransformPose() const {
    return toPyArray(_pconnected->GetTransform());
}

object PyRobotBase::PyConnectedBody::GetRelativeTransform() const {
    return ReturnTransform(_pconnected->GetRelativeTransform());
}
object PyRobotBase::PyConnectedBody::GetRelativeTransformPose() const {
    return toPyArray(_pconnected->GetRelativeTransform());
}

void PyRobotBase::PyConnectedBody::SetLinkEnable(bool enable) {
    _pconnected->SetLinkEnable(enable);
}

void PyRobotBase::PyConnectedBody::SetLinkVisible(bool visible) {
    _pconnected->SetLinkVisible(visible);
}

object PyRobotBase::PyConnectedBody::GetResolvedLinks()
{
    py::list olinks;
    std::vector<KinBody::LinkPtr> vlinks;
    _pconnected->GetResolvedLinks(vlinks);
    FOREACH(itlink, vlinks) {
        olinks.append(toPyLink(*itlink,_pyenv));
    }
    return olinks;
}

object PyRobotBase::PyConnectedBody::GetResolvedJoints()
{
    py::list ojoints;
    std::vector<KinBody::JointPtr> vjoints;
    _pconnected->GetResolvedJoints(vjoints);
    FOREACH(itjoint, vjoints) {
        ojoints.append(toPyJoint(*itjoint, _pyenv));
    }
    return ojoints;
}

object PyRobotBase::PyConnectedBody::GetResolvedManipulators()
{
    py::list omanips;
    std::vector<RobotBase::ManipulatorPtr> vmanips;
    _pconnected->GetResolvedManipulators(vmanips);
    FOREACH(itmanip, vmanips) {
        omanips.append(toPyRobotManipulator(*itmanip, _pyenv));
    }
    return omanips;
}

std::string PyRobotBase::PyConnectedBody::__repr__() {
    return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s').GetConnectedBody('%s')") %
                      RaveGetEnvironmentId(_pconnected->GetRobot()->GetEnv()) %
                      _pconnected->GetRobot()->GetName() % _pconnected->GetName());
}

std::string PyRobotBase::PyConnectedBody::__str__() {
    return boost::str(boost::format("<attachedbody:%s, parent=%s>") % _pconnected->GetName() %
                      _pconnected->GetRobot()->GetName());
}

object PyRobotBase::PyConnectedBody::__unicode__() {
    return ConvertStringToUnicode(__str__());
}

bool PyRobotBase::PyConnectedBody::__eq__(OPENRAVE_SHARED_PTR<PyConnectedBody> p) {
    return !!p && _pconnected == p->_pconnected;
}

bool PyRobotBase::PyConnectedBody::__ne__(OPENRAVE_SHARED_PTR<PyConnectedBody> p) {
    return !p || _pconnected != p->_pconnected;
}

long PyRobotBase::PyConnectedBody::__hash__() {
    return static_cast<long>(uintptr_t(_pconnected.get()));
}

typedef OPENRAVE_SHARED_PTR<PyRobotBase::PyConnectedBody> PyConnectedBodyPtr;
OPENRAVE_SHARED_PTR<PyRobotBase::PyConnectedBody> PyRobotBase::_GetConnectedBody(RobotBase::ConnectedBodyPtr pConnectedBody)
{
    return !pConnectedBody ? PyConnectedBodyPtr() : PyConnectedBodyPtr(new PyConnectedBody(pConnectedBody, _pyenv));
}

PyRobotBase::PyRobotStateSaver::PyRobotStateSaver(PyRobotBasePtr pyrobot) : _pyenv(pyrobot->GetEnv()), _state(pyrobot->GetRobot()) {
    // python should not support restoring on destruction since there's garbage collection
    _state.SetRestoreOnDestructor(false);

}
PyRobotBase::PyRobotStateSaver::PyRobotStateSaver(PyRobotBasePtr pyrobot, object options) : _pyenv(pyrobot->GetEnv()), _state(pyrobot->GetRobot(),pyGetIntFromPy(options,0)) {
    // python should not support restoring on destruction since there's garbage collection
    _state.SetRestoreOnDestructor(false);
}
PyRobotBase::PyRobotStateSaver::PyRobotStateSaver(RobotBasePtr probot, PyEnvironmentBasePtr pyenv) : _pyenv(pyenv), _state(probot) {
    // python should not support restoring on destruction since there's garbage collection
    _state.SetRestoreOnDestructor(false);
}
PyRobotBase::PyRobotStateSaver::PyRobotStateSaver(RobotBasePtr probot, PyEnvironmentBasePtr pyenv, object options) : _pyenv(pyenv), _state(probot,pyGetIntFromPy(options,0)) {
    // python should not support restoring on destruction since there's garbage collection
    _state.SetRestoreOnDestructor(false);
}
PyRobotBase::PyRobotStateSaver::~PyRobotStateSaver() {
}

object PyRobotBase::PyRobotStateSaver::GetBody() const {
    return py::to_object(toPyRobot(RaveInterfaceCast<RobotBase>(_state.GetBody()),_pyenv));
}

void PyRobotBase::PyRobotStateSaver::Restore(PyRobotBasePtr pyrobot) {
    _state.Restore(!pyrobot ? RobotBasePtr() : pyrobot->GetRobot());
}

void PyRobotBase::PyRobotStateSaver::Release() {
    _state.Release();
}

std::string PyRobotBase::PyRobotStateSaver::__str__() {
    KinBodyPtr pbody = _state.GetBody();
    if( !pbody ) {
        return "robot state empty";
    }
    return boost::str(boost::format("robot state for %s")%pbody->GetName());
}
object PyRobotBase::PyRobotStateSaver::__unicode__() {
    return ConvertStringToUnicode(__str__());
}

typedef OPENRAVE_SHARED_PTR<PyRobotBase::PyRobotStateSaver> PyRobotStateSaverPtr;

PyRobotBase::PyRobotBase(RobotBasePtr probot, PyEnvironmentBasePtr pyenv) : PyKinBody(probot,pyenv), _probot(probot) {
}
PyRobotBase::PyRobotBase(const PyRobotBase &r) : PyKinBody(r._probot,r._pyenv) {
    _probot = r._probot;
}
PyRobotBase::~PyRobotBase() {
}

bool PyRobotBase::Init(object olinkinfos, object ojointinfos, object omanipinfos, object oattachedsensorinfos, object oconnectedbodyinfos, const std::string& uri) {
    std::vector<KinBody::LinkInfoConstPtr> vlinkinfos;
    _ParseLinkInfos(olinkinfos, vlinkinfos);
    std::vector<KinBody::JointInfoConstPtr> vjointinfos;
    _ParseJointInfos(ojointinfos, vjointinfos);
    std::vector<RobotBase::ManipulatorInfoConstPtr> vmanipinfos(len(omanipinfos));
    for(size_t i = 0; i < vmanipinfos.size(); ++i) {
        PyManipulatorInfoPtr pymanip = py::extract<PyManipulatorInfoPtr>(omanipinfos[i]);
        if( !pymanip ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.ManipInfo"),ORE_InvalidArguments);
        }
        vmanipinfos[i] = pymanip->GetManipulatorInfo();
    }
    std::vector<RobotBase::AttachedSensorInfoConstPtr> vattachedsensorinfos(len(oattachedsensorinfos));
    for(size_t i = 0; i < vattachedsensorinfos.size(); ++i) {
        PyAttachedSensorInfoPtr pyattachedsensor = py::extract<PyAttachedSensorInfoPtr>(oattachedsensorinfos[i]);
        if( !pyattachedsensor ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.AttachedsensorInfo"),ORE_InvalidArguments);
        }
        vattachedsensorinfos[i] = pyattachedsensor->GetAttachedSensorInfo();
    }
    std::vector<RobotBase::ConnectedBodyInfoConstPtr> vconnectedbodyinfos(len(oconnectedbodyinfos));
    for(size_t i = 0; i < vconnectedbodyinfos.size(); ++i) {
        PyConnectedBodyInfoPtr pyconnectedbody = py::extract<PyConnectedBodyInfoPtr>(oconnectedbodyinfos[i]);
        if( !pyconnectedbody ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.AttachedsensorInfo"),ORE_InvalidArguments);
        }
        vconnectedbodyinfos[i] = pyconnectedbody->GetConnectedBodyInfo();
    }
    return _probot->Init(vlinkinfos, vjointinfos, vmanipinfos, vattachedsensorinfos, vconnectedbodyinfos, uri);
}

object PyRobotBase::GetManipulators()
{
    py::list manips;
    FOREACH(it, _probot->GetManipulators()) {
        manips.append(_GetManipulator(*it));
    }
    return manips;
}

object PyRobotBase::GetManipulators(const string& manipname)
{
    py::list manips;
    FOREACH(it, _probot->GetManipulators()) {
        if( (*it)->GetName() == manipname ) {
            manips.append(_GetManipulator(*it));
        }
    }
    return manips;
}
PyManipulatorPtr PyRobotBase::GetManipulator(const string& manipname)
{
    FOREACH(it, _probot->GetManipulators()) {
        if( (*it)->GetName() == manipname ) {
            return _GetManipulator(*it);
        }
    }
    return PyManipulatorPtr();
}

PyManipulatorPtr PyRobotBase::SetActiveManipulator(const std::string& manipname) {
    _probot->SetActiveManipulator(manipname);
    return GetActiveManipulator();
}
PyManipulatorPtr PyRobotBase::SetActiveManipulator(PyManipulatorPtr pmanip) {
    _probot->SetActiveManipulator(pmanip->GetManipulator());
    return GetActiveManipulator();
}
PyManipulatorPtr PyRobotBase::GetActiveManipulator() {
    return _GetManipulator(_probot->GetActiveManipulator());
}

PyManipulatorPtr PyRobotBase::AddManipulator(PyManipulatorInfoPtr pmanipinfo, bool removeduplicate) {
    return _GetManipulator(_probot->AddManipulator(*pmanipinfo->GetManipulatorInfo(), removeduplicate));
}
bool PyRobotBase::RemoveManipulator(PyManipulatorPtr pmanip) {
    return _probot->RemoveManipulator(pmanip->GetManipulator());
}

PyAttachedSensorPtr PyRobotBase::AddAttachedSensor(PyAttachedSensorInfoPtr pattsensorinfo, bool removeduplicate) {
    return _GetAttachedSensor(_probot->AddAttachedSensor(*pattsensorinfo->GetAttachedSensorInfo(), removeduplicate));
}
bool PyRobotBase::RemoveAttachedSensor(PyAttachedSensorPtr pyattsensor) {
    return _probot->RemoveAttachedSensor(*pyattsensor->GetAttachedSensor());
}

object PyRobotBase::GetSensors()
{
    RAVELOG_WARN("GetSensors is deprecated, please use GetAttachedSensors\n");
    return GetAttachedSensors();
}

object PyRobotBase::GetAttachedSensors()
{
    py::list sensors;
    FOREACH(itsensor, _probot->GetAttachedSensors()) {
        sensors.append(OPENRAVE_SHARED_PTR<PyAttachedSensor>(new PyAttachedSensor(*itsensor,_pyenv)));
    }
    return sensors;
}
OPENRAVE_SHARED_PTR<PyRobotBase::PyAttachedSensor> PyRobotBase::GetSensor(const std::string& sensorname)
{
    RAVELOG_WARN("GetSensor is deprecated, please use GetAttachedSensor\n");
    return GetAttachedSensor(sensorname);
}

OPENRAVE_SHARED_PTR<PyRobotBase::PyAttachedSensor> PyRobotBase::GetAttachedSensor(const std::string& sensorname)
{
    return _GetAttachedSensor(_probot->GetAttachedSensor(sensorname));
}

PyConnectedBodyPtr PyRobotBase::AddConnectedBody(PyConnectedBodyInfoPtr pConnectedBodyInfo, bool removeduplicate) {
    return _GetConnectedBody(_probot->AddConnectedBody(*pConnectedBodyInfo->GetConnectedBodyInfo(), removeduplicate));
}

bool PyRobotBase::RemoveConnectedBody(PyConnectedBodyPtr pConnectedBody) {
    return _probot->RemoveConnectedBody(*pConnectedBody->GetConnectedBody());
}

object PyRobotBase::GetConnectedBodies()
{
    py::list bodies;
    FOREACH(itbody, _probot->GetConnectedBodies()) {
        bodies.append(OPENRAVE_SHARED_PTR<PyConnectedBody>(new PyConnectedBody(*itbody, _pyenv)));
    }
    return bodies;
}

PyConnectedBodyPtr PyRobotBase::GetConnectedBody(const std::string& bodyname)
{
    FOREACH(itbody, _probot->GetConnectedBodies()) {
        if( (*itbody)->GetName() == bodyname ) {
            return _GetConnectedBody(*itbody);
        }
    }
    return PyConnectedBodyPtr();
}

object PyRobotBase::GetConnectedBodyActiveStates() const
{
    std::vector<uint8_t> activestates;
    _probot->GetConnectedBodyActiveStates(activestates);
    return toPyArray(activestates);
}

void PyRobotBase::SetConnectedBodyActiveStates(object oactivestates)
{
    std::vector<uint8_t> activestates = ExtractArray<uint8_t>(oactivestates);
    _probot->SetConnectedBodyActiveStates(activestates);
}

object PyRobotBase::GetController() const {
    CHECK_POINTER(_probot);
    return py::to_object(openravepy::toPyController(_probot->GetController(),_pyenv));
}

bool PyRobotBase::SetController(PyControllerBasePtr pController, const string& PY_ARGS) {
    RAVELOG_WARN("RobotBase::SetController(PyControllerBasePtr,PY_ARGS) is deprecated\n");
    std::vector<int> dofindices;
    for(int i = 0; i < _probot->GetDOF(); ++i) {
        dofindices.push_back(i);
    }
    return _probot->SetController(openravepy::GetController(pController),dofindices,1);
}

bool PyRobotBase::SetController(PyControllerBasePtr pController, object odofindices, int nControlTransformation) {
    CHECK_POINTER(pController);
    std::vector<int> dofindices = ExtractArray<int>(odofindices);
    return _probot->SetController(openravepy::GetController(pController),dofindices,nControlTransformation);
}

bool PyRobotBase::SetController(PyControllerBasePtr pController) {
    RAVELOG_VERBOSE("RobotBase::SetController(PyControllerBasePtr) will control all DOFs and transformation\n");
    std::vector<int> dofindices;
    for(int i = 0; i < _probot->GetDOF(); ++i) {
        dofindices.push_back(i);
    }
    return _probot->SetController(openravepy::GetController(pController),dofindices,1);
}

void PyRobotBase::SetActiveDOFs(const object& dofindices) {
    _probot->SetActiveDOFs(ExtractArray<int>(dofindices));
}
void PyRobotBase::SetActiveDOFs(const object& dofindices, int nAffineDOsBitmask) {
    _probot->SetActiveDOFs(ExtractArray<int>(dofindices), nAffineDOsBitmask);
}
void PyRobotBase::SetActiveDOFs(const object& dofindices, int nAffineDOsBitmask, object rotationaxis) {
    _probot->SetActiveDOFs(ExtractArray<int>(dofindices), nAffineDOsBitmask, ExtractVector3(rotationaxis));
}

int PyRobotBase::GetActiveDOF() const {
    return _probot->GetActiveDOF();
}
int PyRobotBase::GetAffineDOF() const {
    return _probot->GetAffineDOF();
}
int PyRobotBase::GetAffineDOFIndex(DOFAffine dof) const {
    return _probot->GetAffineDOFIndex(dof);
}

object PyRobotBase::GetAffineRotationAxis() const {
    return toPyVector3(_probot->GetAffineRotationAxis());
}
void PyRobotBase::SetAffineTranslationLimits(object lower, object upper) {
    return _probot->SetAffineTranslationLimits(ExtractVector3(lower),ExtractVector3(upper));
}
void PyRobotBase::SetAffineRotationAxisLimits(object lower, object upper) {
    return _probot->SetAffineRotationAxisLimits(ExtractVector3(lower),ExtractVector3(upper));
}
void PyRobotBase::SetAffineRotation3DLimits(object lower, object upper) {
    return _probot->SetAffineRotation3DLimits(ExtractVector3(lower),ExtractVector3(upper));
}
void PyRobotBase::SetAffineRotationQuatLimits(object quatangle) {
    return _probot->SetAffineRotationQuatLimits(ExtractVector4(quatangle));
}
void PyRobotBase::SetAffineTranslationMaxVels(object vels) {
    _probot->SetAffineTranslationMaxVels(ExtractVector3(vels));
}
void PyRobotBase::SetAffineRotationAxisMaxVels(object vels) {
    _probot->SetAffineRotationAxisMaxVels(ExtractVector3(vels));
}
void PyRobotBase::SetAffineRotation3DMaxVels(object vels) {
    _probot->SetAffineRotation3DMaxVels(ExtractVector3(vels));
}
void PyRobotBase::SetAffineRotationQuatMaxVels(dReal vels) {
    _probot->SetAffineRotationQuatMaxVels(vels);
}
void PyRobotBase::SetAffineTranslationResolution(object resolution) {
    _probot->SetAffineTranslationResolution(ExtractVector3(resolution));
}
void PyRobotBase::SetAffineRotationAxisResolution(object resolution) {
    _probot->SetAffineRotationAxisResolution(ExtractVector3(resolution));
}
void PyRobotBase::SetAffineRotation3DResolution(object resolution) {
    _probot->SetAffineRotation3DResolution(ExtractVector3(resolution));
}
void PyRobotBase::SetAffineRotationQuatResolution(dReal resolution) {
    _probot->SetAffineRotationQuatResolution(resolution);
}
void PyRobotBase::SetAffineTranslationWeights(object weights) {
    _probot->SetAffineTranslationWeights(ExtractVector3(weights));
}
void PyRobotBase::SetAffineRotationAxisWeights(object weights) {
    _probot->SetAffineRotationAxisWeights(ExtractVector4(weights));
}
void PyRobotBase::SetAffineRotation3DWeights(object weights) {
    _probot->SetAffineRotation3DWeights(ExtractVector3(weights));
}
void PyRobotBase::SetAffineRotationQuatWeights(dReal weights) {
    _probot->SetAffineRotationQuatWeights(weights);
}

object PyRobotBase::GetAffineTranslationLimits() const
{
    Vector lower, upper;
    _probot->GetAffineTranslationLimits(lower,upper);
    return py::make_tuple(toPyVector3(lower),toPyVector3(upper));
}
object PyRobotBase::GetAffineRotationAxisLimits() const
{
    Vector lower, upper;
    _probot->GetAffineRotationAxisLimits(lower,upper);
    return py::make_tuple(toPyVector3(lower),toPyVector3(upper));
}
object PyRobotBase::GetAffineRotation3DLimits() const
{
    Vector lower, upper;
    _probot->GetAffineRotation3DLimits(lower,upper);
    return py::make_tuple(toPyVector3(lower),toPyVector3(upper));
}
object PyRobotBase::GetAffineRotationQuatLimits() const
{
    return toPyVector4(_probot->GetAffineRotationQuatLimits());
}
object PyRobotBase::GetAffineTranslationMaxVels() const {
    return toPyVector3(_probot->GetAffineTranslationMaxVels());
}
object PyRobotBase::GetAffineRotationAxisMaxVels() const {
    return toPyVector3(_probot->GetAffineRotationAxisMaxVels());
}
object PyRobotBase::GetAffineRotation3DMaxVels() const {
    return toPyVector3(_probot->GetAffineRotation3DMaxVels());
}
dReal PyRobotBase::GetAffineRotationQuatMaxVels() const {
    return _probot->GetAffineRotationQuatMaxVels();
}
object PyRobotBase::GetAffineTranslationResolution() const {
    return toPyVector3(_probot->GetAffineTranslationResolution());
}
object PyRobotBase::GetAffineRotationAxisResolution() const {
    return toPyVector4(_probot->GetAffineRotationAxisResolution());
}
object PyRobotBase::GetAffineRotation3DResolution() const {
    return toPyVector3(_probot->GetAffineRotation3DResolution());
}
dReal PyRobotBase::GetAffineRotationQuatResolution() const {
    return _probot->GetAffineRotationQuatResolution();
}
object PyRobotBase::GetAffineTranslationWeights() const {
    return toPyVector3(_probot->GetAffineTranslationWeights());
}
object PyRobotBase::GetAffineRotationAxisWeights() const {
    return toPyVector4(_probot->GetAffineRotationAxisWeights());
}
object PyRobotBase::GetAffineRotation3DWeights() const {
    return toPyVector3(_probot->GetAffineRotation3DWeights());
}
dReal PyRobotBase::GetAffineRotationQuatWeights() const {
    return _probot->GetAffineRotationQuatWeights();
}

void PyRobotBase::SetActiveDOFValues(object values, uint32_t checklimits) const
{
    std::vector<dReal> vvalues = ExtractArray<dReal>(values);
    if( vvalues.size() > 0 ) {
        _probot->SetActiveDOFValues(vvalues,checklimits);
    }
    else {
        OPENRAVE_ASSERT_OP_FORMAT((int)vvalues.size(),>=,_probot->GetActiveDOF(), "not enough values %d<%d",vvalues.size()%_probot->GetActiveDOF(),ORE_InvalidArguments);
    }
}
object PyRobotBase::GetActiveDOFValues() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFValues(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFWeights() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> weights;
    _probot->GetActiveDOFWeights(weights);
    return toPyArray(weights);
}

void PyRobotBase::SetActiveDOFVelocities(object velocities, uint32_t checklimits)
{
    _probot->SetActiveDOFVelocities(ExtractArray<dReal>(velocities), checklimits);
}
object PyRobotBase::GetActiveDOFVelocities() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFVelocities(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFLimits() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::make_tuple(py::empty_array_astype<dReal>(), py::empty_array_astype<dReal>()); // always need 2 since users can do lower, upper = GetDOFLimits()
    }
    std::vector<dReal> lower, upper;
    _probot->GetActiveDOFLimits(lower,upper);
    return py::make_tuple(toPyArray(lower),toPyArray(upper));
}

object PyRobotBase::GetActiveDOFMaxVel() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFMaxVel(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFMaxAccel() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFMaxAccel(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFMaxJerk() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFMaxJerk(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFHardMaxVel() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFHardMaxVel(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFHardMaxAccel() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFHardMaxAccel(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFHardMaxJerk() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFHardMaxJerk(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFResolutions() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFResolutions(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveConfigurationSpecification(const std::string& interpolation) const {
    return py::to_object(openravepy::toPyConfigurationSpecification(_probot->GetActiveConfigurationSpecification(interpolation)));
}

object PyRobotBase::GetActiveJointIndices() {
    RAVELOG_WARN("GetActiveJointIndices deprecated. Use GetActiveDOFIndices\n"); return toPyArray(_probot->GetActiveDOFIndices());
}
object PyRobotBase::GetActiveDOFIndices() {
    return toPyArray(_probot->GetActiveDOFIndices());
}

object PyRobotBase::SubtractActiveDOFValues(object ovalues0, object ovalues1)
{
    std::vector<dReal> values0 = ExtractArray<dReal>(ovalues0);
    std::vector<dReal> values1 = ExtractArray<dReal>(ovalues1);
    BOOST_ASSERT((int)values0.size() == GetActiveDOF() );
    BOOST_ASSERT((int)values1.size() == GetActiveDOF() );
    _probot->SubtractActiveDOFValues(values0,values1);
    return toPyArray(values0);
}

object PyRobotBase::CalculateActiveJacobian(int index, object offset) const
{
    std::vector<dReal> vjacobian;
    _probot->CalculateActiveJacobian(index,ExtractVector3(offset),vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _probot->GetActiveDOF();
    return toPyArray(vjacobian,dims);
}

object PyRobotBase::CalculateActiveRotationJacobian(int index, object q) const
{
    std::vector<dReal> vjacobian;
    _probot->CalculateActiveRotationJacobian(index,ExtractVector4(q),vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 4; dims[1] = _probot->GetActiveDOF();
    return toPyArray(vjacobian,dims);
}

object PyRobotBase::CalculateActiveAngularVelocityJacobian(int index) const
{
    std::vector<dReal> vjacobian;
    _probot->CalculateActiveAngularVelocityJacobian(index,vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _probot->GetActiveDOF();
    return toPyArray(vjacobian,dims);
}

bool PyRobotBase::Grab(PyKinBodyPtr pbody) {
    CHECK_POINTER(pbody); return _probot->Grab(pbody->GetBody());
}

// since PyKinBody::Grab is overloaded with (pbody, plink) parameters, have to support both...?
bool PyRobotBase::Grab(PyKinBodyPtr pbody, object pylink_or_linkstoignore)
{
    CHECK_POINTER(pbody);
    CHECK_POINTER(pylink_or_linkstoignore);
    KinBody::LinkPtr plink = GetKinBodyLink(pylink_or_linkstoignore);
    if( !!plink ) {
        return _probot->Grab(pbody->GetBody(), plink);
    }
    // maybe it is a set?
    std::set<int> setlinkstoignore = ExtractSet<int>(pylink_or_linkstoignore);
    return _probot->Grab(pbody->GetBody(), setlinkstoignore);
}

bool PyRobotBase::Grab(PyKinBodyPtr pbody, object pylink, object linkstoignore)
{
    CHECK_POINTER(pbody);
    CHECK_POINTER(pylink);
    std::set<int> setlinkstoignore = ExtractSet<int>(linkstoignore);
    return _pbody->Grab(pbody->GetBody(), GetKinBodyLink(pylink), setlinkstoignore);
}

bool PyRobotBase::CheckLinkSelfCollision(int ilinkindex, object olinktrans, PyCollisionReportPtr pyreport)
{
    return _probot->CheckLinkSelfCollision(ilinkindex, ExtractTransform(olinktrans), !pyreport ? CollisionReportPtr() : openravepy::GetCollisionReport(pyreport));
}

bool PyRobotBase::WaitForController(float ftimeout)
{
    ControllerBasePtr pcontroller = _probot->GetController();
    if( !pcontroller ) {
        return false;
    }
    if( pcontroller->IsDone() ) {
        return true;
    }
    bool bSuccess = true;
    Py_BEGIN_ALLOW_THREADS;

    try {
        uint64_t starttime = GetMicroTime();
        uint64_t deltatime = (uint64_t)(ftimeout*1000000.0);
        while( !pcontroller->IsDone() ) {
            Sleep(1);
            if(( deltatime > 0) &&( (GetMicroTime()-starttime)>deltatime) ) {
                bSuccess = false;
                break;
            }
        }
    }
    catch(...) {
        RAVELOG_ERROR("exception raised inside WaitForController:\n");
        PyErr_Print();
        bSuccess = false;
    }

    Py_END_ALLOW_THREADS;
    return bSuccess;
}

std::string PyRobotBase::GetRobotStructureHash() const {
    return _probot->GetRobotStructureHash();
}

PyStateRestoreContextBase* PyRobotBase::CreateStateSaver(object options) {
    PyRobotStateSaverPtr saver;
    if( IS_PYTHONOBJECT_NONE(options) ) {
        saver.reset(new PyRobotStateSaver(_probot,_pyenv));
    }
    else {
        saver.reset(new PyRobotStateSaver(_probot,_pyenv,options));
    }
    return new PyStateRestoreContext<PyRobotStateSaverPtr, PyRobotBasePtr>(saver);
}

PyStateRestoreContextBase* PyRobotBase::CreateRobotStateSaver(object options) {
    return CreateStateSaver(options);
}

std::string PyRobotBase::__repr__() {
    return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s')")%RaveGetEnvironmentId(_probot->GetEnv())%_probot->GetName());
}
std::string PyRobotBase::__str__() {
    return boost::str(boost::format("<%s:%s - %s (%s)>")%RaveGetInterfaceName(_probot->GetInterfaceType())%_probot->GetXMLId()%_probot->GetName()%_probot->GetRobotStructureHash());
}
object PyRobotBase::__unicode__() {
    return ConvertStringToUnicode(__str__());
}
void PyRobotBase::__enter__()
{
    // necessary to lock physics to prevent multiple threads from interfering
    if( _listStateSavers.size() == 0 ) {
        openravepy::LockEnvironment(_pyenv);
    }
    _listStateSavers.push_back(OPENRAVE_SHARED_PTR<void>(new RobotBase::RobotStateSaver(_probot)));
}

class ManipulatorInfo_pickle_suite
#ifndef USE_PYBIND11_PYTHON_BINDINGS
 : public pickle_suite
#endif 
{
public:
    static py::tuple getstate(const PyManipulatorInfo& r)
    {
        return py::make_tuple(r._name, r._sBaseLinkName, r._sEffectorLinkName, r._tLocalTool, r._vChuckingDirection, r._vdirection, r._sIkSolverXMLId, r._vGripperJointNames);
    }
    static void setstate(PyManipulatorInfo& r, py::tuple state) {
        r._name = state[0];
        r._sBaseLinkName = state[1];
        r._sEffectorLinkName = state[2];
        r._tLocalTool = state[3];
        r._vChuckingDirection = state[4];
        r._vdirection = state[5];
        r._sIkSolverXMLId = py::extract<std::string>(state[6]);
        r._vGripperJointNames = state[7];
    }
};

RobotBasePtr GetRobot(object o)
{
    extract_<PyRobotBasePtr> pyrobot(o);
    if( pyrobot.check() ) {
        return GetRobot((PyRobotBasePtr)pyrobot);
    }
    return RobotBasePtr();
}

RobotBasePtr GetRobot(PyRobotBasePtr pyrobot)
{
    return !pyrobot ? RobotBasePtr() : pyrobot->GetRobot();
}

PyInterfaceBasePtr toPyRobot(RobotBasePtr probot, PyEnvironmentBasePtr pyenv)
{
    return !probot ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyRobotBase(probot,pyenv));
}

RobotBase::ManipulatorPtr GetRobotManipulator(object o)
{
    extract_<PyRobotBase::PyManipulatorPtr> pymanipulator(o);
    if( pymanipulator.check() ) {
        return ((PyRobotBase::PyManipulatorPtr)pymanipulator)->GetManipulator();
    }
    return RobotBase::ManipulatorPtr();
}

object toPyRobotManipulator(RobotBase::ManipulatorPtr pmanip, PyEnvironmentBasePtr pyenv)
{
    return !pmanip ? py::none_() : py::to_object(PyRobotBase::PyManipulatorPtr(new PyRobotBase::PyManipulator(pmanip,pyenv)));
}

PyRobotBasePtr RaveCreateRobot(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    RobotBasePtr p = OpenRAVE::RaveCreateRobot(openravepy::GetEnvironment(pyenv), name);
    if( !p ) {
        return PyRobotBasePtr();
    }
    return PyRobotBasePtr(new PyRobotBase(p,pyenv));
}

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkParameterization_overloads, GetIkParameterization, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckEndEffectorCollision_overloads, CheckEndEffectorCollision, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckEndEffectorSelfCollision_overloads, CheckEndEffectorSelfCollision, 1, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolution_overloads, FindIKSolution, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionFree_overloads, FindIKSolution, 3, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutions_overloads, FindIKSolutions, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionsFree_overloads, FindIKSolutions, 3, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetArmConfigurationSpecification_overloads, GetArmConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkConfigurationSpecification_overloads, GetIkConfigurationSpecification, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateRobotStateSaver_overloads, CreateRobotStateSaver, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFValues_overloads, SetActiveDOFValues, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFVelocities_overloads, SetActiveDOFVelocities, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddManipulator_overloads, AddManipulator, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddAttachedSensor_overloads, AddAttachedSensor, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddConnectedBody_overloads, AddConnectedBody, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetActiveConfigurationSpecification_overloads, GetActiveConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Restore_overloads, Restore, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Init_overloads, Init, 5,6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(UpdateInfo_overloads, UpdateInfo, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(UpdateAndGetInfo_overloads, UpdateAndGetInfo, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckLinkSelfCollision_overloads, CheckLinkSelfCollision, 2, 3)
// SerializeJSON
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyManipulatorInfo_SerializeJSON_overloads, SerializeJSON, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyAttachedSensorInfo_SerializeJSON_overloads, SerializeJSON, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyConnectedBodyInfo_SerializeJSON_overloads, SerializeJSON, 0, 2)
// DeserializeJSON
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyManipulatorInfo_DeserializeJSON_overloads, DeserializeJSON, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyAttachedSensorInfo_DeserializeJSON_overloads, DeserializeJSON, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyConnectedBodyInfo_DeserializeJSON_overloads, DeserializeJSON, 1, 2)
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_robot(py::module& m)
#else
void init_openravepy_robot()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals;  // "..."_a
    object dofaffine = enum_<DOFAffine>(m, "DOFAffine", py::arithmetic() DOXY_ENUM(DOFAffine))
#else
    object dofaffine = enum_<DOFAffine>("DOFAffine" DOXY_ENUM(DOFAffine))
#endif
                       .value("NoTransform",DOF_NoTransform)
                       .value("X",DOF_X)
                       .value("Y",DOF_Y)
                       .value("Z",DOF_Z)
                       .value("RotationAxis",DOF_RotationAxis)
                       .value("Rotation3D",DOF_Rotation3D)
                       .value("RotationQuat",DOF_RotationQuat)
                       .value("RotationMask",DOF_RotationMask)
                       .value("Transform",DOF_Transform)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    object manipulatorinfo = class_<PyManipulatorInfo, OPENRAVE_SHARED_PTR<PyManipulatorInfo> >(m, "ManipulatorInfo", DOXY_CLASS(RobotBase::ManipulatorInfo))
                             .def(init<>())
#else
    object manipulatorinfo = class_<PyManipulatorInfo, OPENRAVE_SHARED_PTR<PyManipulatorInfo> >("ManipulatorInfo", DOXY_CLASS(RobotBase::ManipulatorInfo))
#endif
                             .def_readwrite("_name",&PyManipulatorInfo::_name)
                             .def_readwrite("_sBaseLinkName",&PyManipulatorInfo::_sBaseLinkName)
                             .def_readwrite("_sEffectorLinkName",&PyManipulatorInfo::_sEffectorLinkName)
                             .def_readwrite("_tLocalTool",&PyManipulatorInfo::_tLocalTool)
                             .def_readwrite("_vChuckingDirection",&PyManipulatorInfo::_vChuckingDirection)
                             .def_readwrite("_vClosingDirection",&PyManipulatorInfo::_vChuckingDirection) // back compat
                             .def_readwrite("_vdirection",&PyManipulatorInfo::_vdirection)
                             .def_readwrite("_sIkSolverXMLId",&PyManipulatorInfo::_sIkSolverXMLId)
                             .def_readwrite("_vGripperJointNames",&PyManipulatorInfo::_vGripperJointNames)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                             .def("SerializeJSON", &PyManipulatorInfo::SerializeJSON,
                                "unitScale"_a = 1.0,
                                "options"_a = py::none_(),
                                DOXY_FN(RobotBase::ManipulatorInfo, SerializeJSON)
                             )
                             .def("DeserializeJSON", &PyManipulatorInfo::DeserializeJSON,
                                "obj"_a,
                                "unitScale"_a = 1.0,
                                DOXY_FN(RobotBase::ManipulatorInfo, DeserializeJSON)
                             )
#else
                             .def("SerializeJSON", &PyManipulatorInfo::SerializeJSON, PyManipulatorInfo_SerializeJSON_overloads(PY_ARGS("options") DOXY_FN(RobotBase::ManipulatorInfo, SerializeJSON)))
                             .def("DeserializeJSON", &PyManipulatorInfo::DeserializeJSON, PyManipulatorInfo_DeserializeJSON_overloads(PY_ARGS("obj", "unitScale") DOXY_FN(RobotBase::ManipulatorInfo, DeserializeJSON)))
#endif
                             
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                             .def(py::pickle(
                             [](const PyManipulatorInfo &pyinfo) {
                                 // __getstate__
                                 return ManipulatorInfo_pickle_suite::getstate(pyinfo);
                             },
                             [](py::tuple state) {
                                 // __setstate__
                                 /* Create a new C++ instance */
                                 PyManipulatorInfo pyinfo;
                                 ManipulatorInfo_pickle_suite::setstate(pyinfo, state);
                                 return pyinfo;
                             }
                             ))
#else                           
                             .def_pickle(ManipulatorInfo_pickle_suite())
#endif
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    object attachedsensorinfo = class_<PyAttachedSensorInfo, OPENRAVE_SHARED_PTR<PyAttachedSensorInfo> >(m, "AttachedSensorInfo", DOXY_CLASS(RobotBase::AttachedSensorInfo))
                                .def(init<>())
#else
    object attachedsensorinfo = class_<PyAttachedSensorInfo, OPENRAVE_SHARED_PTR<PyAttachedSensorInfo> >("AttachedSensorInfo", DOXY_CLASS(RobotBase::AttachedSensorInfo))
#endif
                                .def_readwrite("_name", &PyAttachedSensorInfo::_name)
                                .def_readwrite("_linkname", &PyAttachedSensorInfo::_linkname)
                                .def_readwrite("_trelative", &PyAttachedSensorInfo::_trelative)
                                .def_readwrite("_sensorname", &PyAttachedSensorInfo::_sensorname)
                                .def_readwrite("_sensorgeometry", &PyAttachedSensorInfo::_sensorgeometry)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                                .def("SerializeJSON", &PyAttachedSensorInfo::SerializeJSON,
                                    "unitScale"_a = 1.0,
                                    "options"_a = py::none_(),
                                    DOXY_FN(RobotBase::AttachedSensorInfo, SerializeJSON)
                                )
                                .def("DeserializeJSON", &PyAttachedSensorInfo::DeserializeJSON,
                                    "obj"_a,
                                    "unitScale"_a = 1.0,
                                    DOXY_FN(RobotBase::AttachedSensorInfo, DeserializeJSON)
                                )
#else
                                .def("SerializeJSON", &PyAttachedSensorInfo::SerializeJSON, PyAttachedSensorInfo_SerializeJSON_overloads(PY_ARGS("unitScale", "options") DOXY_FN(RobotBase::AttachedSensorInfo, SerializeJSON)))
                                .def("DeserializeJSON", &PyAttachedSensorInfo::DeserializeJSON, PyAttachedSensorInfo_DeserializeJSON_overloads(PY_ARGS("obj", "unitScale") DOXY_FN(RobotBase::AttachedSensorInfo, DeserializeJSON)))
#endif
                                
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    object connectedbodyinfo = class_<PyConnectedBodyInfo, OPENRAVE_SHARED_PTR<PyConnectedBodyInfo> >(m, "ConnectedBodyInfo", DOXY_CLASS(RobotBase::ConnectedBodyInfo))
                               .def(init<>())
#else
    object connectedbodyinfo = class_<PyConnectedBodyInfo, OPENRAVE_SHARED_PTR<PyConnectedBodyInfo> >("ConnectedBodyInfo", DOXY_CLASS(RobotBase::ConnectedBodyInfo))
#endif
                               .def_readwrite("_name", &PyConnectedBodyInfo::_name)
                               .def_readwrite("_linkname", &PyConnectedBodyInfo::_linkname)
                               .def_readwrite("_trelative", &PyConnectedBodyInfo::_trelative)
                               .def_readwrite("_uri", &PyConnectedBodyInfo::_uri)
                               .def_readwrite("_linkInfos", &PyConnectedBodyInfo::_linkInfos)
                               .def_readwrite("_jointInfos", &PyConnectedBodyInfo::_jointInfos)
                               .def_readwrite("_manipulatorInfos", &PyConnectedBodyInfo::_manipulatorInfos)
                               .def_readwrite("_attachedSensorInfos", &PyConnectedBodyInfo::_attachedSensorInfos)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def("SerializeJSON", &PyConnectedBodyInfo::SerializeJSON,
                                   "unitScale"_a = 1.0,
                                   "options"_a = py::none_(),
                                   DOXY_FN(RobotBase::ConnectedBodyInfo, SerializeJSON)
                               )
                               .def("DeserializeJSON", &PyConnectedBodyInfo::DeserializeJSON,
                                   "obj"_a,
                                   "unitScale"_a = 1.0,
                                   DOXY_FN(RobotBase::ConnectedBodyInfo, DeserializeJSON)
                               )
#else
                               .def("SerializeJSON", &PyConnectedBodyInfo::SerializeJSON, PyConnectedBodyInfo_SerializeJSON_overloads(PY_ARGS("unitScale", "options") DOXY_FN(RobotBase::ConnectedBodyInfo, SerializeJSON)))
                               .def("DeserializeJSON", &PyConnectedBodyInfo::DeserializeJSON, PyConnectedBodyInfo_DeserializeJSON_overloads(PY_ARGS("obj", "unitScale") DOXY_FN(RobotBase::ConnectedBodyInfo, DeserializeJSON)))
#endif
                               
    ;

    {
        void (PyRobotBase::*psetactivedofs1)(const object&) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs2)(const object&, int) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs3)(const object&, int, object) = &PyRobotBase::SetActiveDOFs;

        bool (PyRobotBase::*pgrab1)(PyKinBodyPtr) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab2)(PyKinBodyPtr,object) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab3)(PyKinBodyPtr,object,object) = &PyRobotBase::Grab;

        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator2)(const std::string&) = &PyRobotBase::SetActiveManipulator;
        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator3)(PyRobotBase::PyManipulatorPtr) = &PyRobotBase::SetActiveManipulator;

        object (PyRobotBase::*GetManipulators1)() = &PyRobotBase::GetManipulators;
        object (PyRobotBase::*GetManipulators2)(const string &) = &PyRobotBase::GetManipulators;
        bool (PyRobotBase::*setcontroller1)(PyControllerBasePtr,const string &) = &PyRobotBase::SetController;
        bool (PyRobotBase::*setcontroller2)(PyControllerBasePtr,object,int) = &PyRobotBase::SetController;
        bool (PyRobotBase::*setcontroller3)(PyControllerBasePtr) = &PyRobotBase::SetController;
        bool (PyRobotBase::*initrobot)(object, object, object, object, object, const std::string&) = &PyRobotBase::Init;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        scope_ robot = class_<PyRobotBase, OPENRAVE_SHARED_PTR<PyRobotBase>, PyKinBody>(m, "Robot", DOXY_CLASS(RobotBase))
#else
        scope_ robot = class_<PyRobotBase, OPENRAVE_SHARED_PTR<PyRobotBase>, bases<PyKinBody, PyInterfaceBase> >("Robot", DOXY_CLASS(RobotBase), no_init)
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                      .def("Init", initrobot,
                        "linkinfos"_a,
                        "jointinfos"_a,
                        "manipinfos"_a,
                        "attachedsensorinfos"_a,
                        "connectedbodyinfos"_a,
                        "uri"_a = "",
                        DOXY_FN(RobotBase, Init)
                        )
#else
                      .def("Init", initrobot, Init_overloads(PY_ARGS("linkinfos", "jointinfos", "manipinfos", "attachedsensorinfos", "connectedbodyinfos", "uri") DOXY_FN(RobotBase, Init)))
#endif
                      .def("GetManipulators",GetManipulators1, DOXY_FN(RobotBase,GetManipulators))
                      .def("GetManipulators",GetManipulators2, PY_ARGS("manipname") DOXY_FN(RobotBase,GetManipulators))
                      .def("GetManipulator",&PyRobotBase::GetManipulator,PY_ARGS("manipname") "Return the manipulator whose name matches")
                      .def("SetActiveManipulator",setactivemanipulator2, PY_ARGS("manipname") DOXY_FN(RobotBase,SetActiveManipulator "const std::string"))
                      .def("SetActiveManipulator",setactivemanipulator3,PY_ARGS("manip") "Set the active manipulator given a pointer")
                      .def("GetActiveManipulator",&PyRobotBase::GetActiveManipulator, DOXY_FN(RobotBase,GetActiveManipulator))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                      .def("AddManipulator", &PyRobotBase::AddManipulator,
                        "manipinfo"_a,
                        "removeduplicate"_a = false,
                        DOXY_FN(RobotBase, AddManipulator)
                        )
#else
                      .def("AddManipulator",&PyRobotBase::AddManipulator, AddManipulator_overloads(PY_ARGS("manipinfo", "removeduplicate") DOXY_FN(RobotBase,AddManipulator)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                      .def("AddAttachedSensor",&PyRobotBase::AddAttachedSensor,
                        "attachedsensorinfo"_a,
                        "removeduplicate"_a = false,
                        DOXY_FN(RobotBase, AddAttachedSensor)
                        )
#else
                      .def("AddAttachedSensor",&PyRobotBase::AddAttachedSensor, AddAttachedSensor_overloads(PY_ARGS("attachedsensorinfo", "removeduplicate") DOXY_FN(RobotBase,AddAttachedSensor)))
#endif
                      .def("RemoveAttachedSensor",&PyRobotBase::RemoveAttachedSensor, PY_ARGS("attsensor") DOXY_FN(RobotBase,RemoveAttachedSensor))
                      .def("RemoveManipulator",&PyRobotBase::RemoveManipulator, PY_ARGS("manip") DOXY_FN(RobotBase,RemoveManipulator))
                      .def("GetAttachedSensors",&PyRobotBase::GetAttachedSensors, DOXY_FN(RobotBase,GetAttachedSensors))
                      .def("GetAttachedSensor",&PyRobotBase::GetAttachedSensor,PY_ARGS("sensorname") "Return the attached sensor whose name matches")
                      .def("GetSensors",&PyRobotBase::GetSensors)
                      .def("GetSensor",&PyRobotBase::GetSensor,PY_ARGS("sensorname") DOXY_FN(RobotBase, GetSensor))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                      .def("AddConnectedBody",&PyRobotBase::AddConnectedBody,
                        "connectedbodyinfo"_a,
                        "removeduplicate"_a = false,
                        DOXY_FN(RobotBase, AddConnectedBody)
                        )
#else
                      .def("AddConnectedBody",&PyRobotBase::AddConnectedBody, AddConnectedBody_overloads(PY_ARGS("connectedbodyinfo", "removeduplicate") DOXY_FN(RobotBase,AddConnectedBody)))
#endif
                      .def("RemoveConnectedBody",&PyRobotBase::RemoveConnectedBody, PY_ARGS("connectedbody") DOXY_FN(RobotBase,RemoveConnectedBody))
                      .def("GetConnectedBodies",&PyRobotBase::GetConnectedBodies, DOXY_FN(RobotBase,GetConnectedBodies))
                      .def("GetConnectedBody",&PyRobotBase::GetConnectedBody, PY_ARGS("bodyname") DOXY_FN(RobotBase,GetConnectedBody))
                      .def("GetConnectedBodyActiveStates",&PyRobotBase::GetConnectedBodyActiveStates, DOXY_FN(RobotBase,GetConnectedBodyActiveStates))
                      .def("SetConnectedBodyActiveStates",&PyRobotBase::SetConnectedBodyActiveStates, DOXY_FN(RobotBase,SetConnectedBodyActiveStates))
                      .def("GetController",&PyRobotBase::GetController, DOXY_FN(RobotBase,GetController))
                      .def("SetController",setcontroller1,DOXY_FN(RobotBase,SetController))
                      .def("SetController",setcontroller2, PY_ARGS("robot","dofindices","controltransform") DOXY_FN(RobotBase,SetController))
                      .def("SetController",setcontroller3,DOXY_FN(RobotBase,SetController))
                      .def("SetActiveDOFs",psetactivedofs1, PY_ARGS("dofindices") DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int"))
                      .def("SetActiveDOFs",psetactivedofs2, PY_ARGS("dofindices","affine") DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int"))
                      .def("SetActiveDOFs",psetactivedofs3, PY_ARGS("dofindices","affine","rotationaxis") DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int; const Vector"))
                      .def("GetActiveDOF",&PyRobotBase::GetActiveDOF, DOXY_FN(RobotBase,GetActiveDOF))
                      .def("GetAffineDOF",&PyRobotBase::GetAffineDOF, DOXY_FN(RobotBase,GetAffineDOF))
                      .def("GetAffineDOFIndex",&PyRobotBase::GetAffineDOFIndex, PY_ARGS("index") DOXY_FN(RobotBase,GetAffineDOFIndex))
                      .def("GetAffineRotationAxis",&PyRobotBase::GetAffineRotationAxis, DOXY_FN(RobotBase,GetAffineRotationAxis))
                      .def("SetAffineTranslationLimits",&PyRobotBase::SetAffineTranslationLimits, PY_ARGS("lower","upper") DOXY_FN(RobotBase,SetAffineTranslationLimits))
                      .def("SetAffineRotationAxisLimits",&PyRobotBase::SetAffineRotationAxisLimits, PY_ARGS("lower","upper") DOXY_FN(RobotBase,SetAffineRotationAxisLimits))
                      .def("SetAffineRotation3DLimits",&PyRobotBase::SetAffineRotation3DLimits, PY_ARGS("lower","upper") DOXY_FN(RobotBase,SetAffineRotation3DLimits))
                      .def("SetAffineRotationQuatLimits",&PyRobotBase::SetAffineRotationQuatLimits, PY_ARGS("quatangle") DOXY_FN(RobotBase,SetAffineRotationQuatLimits))
                      .def("SetAffineTranslationMaxVels",&PyRobotBase::SetAffineTranslationMaxVels, PY_ARGS("vels") DOXY_FN(RobotBase,SetAffineTranslationMaxVels))
                      .def("SetAffineRotationAxisMaxVels",&PyRobotBase::SetAffineRotationAxisMaxVels, PY_ARGS("velocity") DOXY_FN(RobotBase,SetAffineRotationAxisMaxVels))
                      .def("SetAffineRotation3DMaxVels",&PyRobotBase::SetAffineRotation3DMaxVels, PY_ARGS("velocity") DOXY_FN(RobotBase,SetAffineRotation3DMaxVels))
                      .def("SetAffineRotationQuatMaxVels",&PyRobotBase::SetAffineRotationQuatMaxVels, PY_ARGS("velocity") DOXY_FN(RobotBase,SetAffineRotationQuatMaxVels))
                      .def("SetAffineTranslationResolution",&PyRobotBase::SetAffineTranslationResolution, PY_ARGS("resolution") DOXY_FN(RobotBase,SetAffineTranslationResolution))
                      .def("SetAffineRotationAxisResolution",&PyRobotBase::SetAffineRotationAxisResolution, PY_ARGS("resolution") DOXY_FN(RobotBase,SetAffineRotationAxisResolution))
                      .def("SetAffineRotation3DResolution",&PyRobotBase::SetAffineRotation3DResolution, PY_ARGS("resolution") DOXY_FN(RobotBase,SetAffineRotation3DResolution))
                      .def("SetAffineRotationQuatResolution",&PyRobotBase::SetAffineRotationQuatResolution, PY_ARGS("resolution") DOXY_FN(RobotBase,SetAffineRotationQuatResolution))
                      .def("SetAffineTranslationWeights",&PyRobotBase::SetAffineTranslationWeights, PY_ARGS("weights") DOXY_FN(RobotBase,SetAffineTranslationWeights))
                      .def("SetAffineRotationAxisWeights",&PyRobotBase::SetAffineRotationAxisWeights, PY_ARGS("weights") DOXY_FN(RobotBase,SetAffineRotationAxisWeights))
                      .def("SetAffineRotation3DWeights",&PyRobotBase::SetAffineRotation3DWeights, PY_ARGS("weights") DOXY_FN(RobotBase,SetAffineRotation3DWeights))
                      .def("SetAffineRotationQuatWeights",&PyRobotBase::SetAffineRotationQuatWeights, PY_ARGS("weights") DOXY_FN(RobotBase,SetAffineRotationQuatWeights))
                      .def("GetAffineTranslationLimits",&PyRobotBase::GetAffineTranslationLimits, DOXY_FN(RobotBase,GetAffineTranslationLimits))
                      .def("GetAffineRotationAxisLimits",&PyRobotBase::GetAffineRotationAxisLimits, DOXY_FN(RobotBase,GetAffineRotationAxisLimits))
                      .def("GetAffineRotation3DLimits",&PyRobotBase::GetAffineRotation3DLimits, DOXY_FN(RobotBase,GetAffineRotation3DLimits))
                      .def("GetAffineRotationQuatLimits",&PyRobotBase::GetAffineRotationQuatLimits, DOXY_FN(RobotBase,GetAffineRotationQuatLimits))
                      .def("GetAffineTranslationMaxVels",&PyRobotBase::GetAffineTranslationMaxVels, DOXY_FN(RobotBase,GetAffineTranslationMaxVels))
                      .def("GetAffineRotationAxisMaxVels",&PyRobotBase::GetAffineRotationAxisMaxVels, DOXY_FN(RobotBase,GetAffineRotationAxisMaxVels))
                      .def("GetAffineRotation3DMaxVels",&PyRobotBase::GetAffineRotation3DMaxVels, DOXY_FN(RobotBase,GetAffineRotation3DMaxVels))
                      .def("GetAffineRotationQuatMaxVels",&PyRobotBase::GetAffineRotationQuatMaxVels, DOXY_FN(RobotBase,GetAffineRotationQuatMaxVels))
                      .def("GetAffineTranslationResolution",&PyRobotBase::GetAffineTranslationResolution, DOXY_FN(RobotBase,GetAffineTranslationResolution))
                      .def("GetAffineRotationAxisResolution",&PyRobotBase::GetAffineRotationAxisResolution, DOXY_FN(RobotBase,GetAffineRotationAxisResolution))
                      .def("GetAffineRotation3DResolution",&PyRobotBase::GetAffineRotation3DResolution, DOXY_FN(RobotBase,GetAffineRotation3DResolution))
                      .def("GetAffineRotationQuatResolution",&PyRobotBase::GetAffineRotationQuatResolution, DOXY_FN(RobotBase,GetAffineRotationQuatResolution))
                      .def("GetAffineTranslationWeights",&PyRobotBase::GetAffineTranslationWeights, DOXY_FN(RobotBase,GetAffineTranslationWeights))
                      .def("GetAffineRotationAxisWeights",&PyRobotBase::GetAffineRotationAxisWeights, DOXY_FN(RobotBase,GetAffineRotationAxisWeights))
                      .def("GetAffineRotation3DWeights",&PyRobotBase::GetAffineRotation3DWeights, DOXY_FN(RobotBase,GetAffineRotation3DWeights))
                      .def("GetAffineRotationQuatWeights",&PyRobotBase::GetAffineRotationQuatWeights, DOXY_FN(RobotBase,GetAffineRotationQuatWeights))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                      .def("SetActiveDOFValues", &PyRobotBase::SetActiveDOFValues,
                        "values"_a,
                        "checklimits"_a = (int) KinBody::CLA_CheckLimits,
                        DOXY_FN(RobotBase, SetActiveDOFValues)
                        )
#else
                      .def("SetActiveDOFValues",&PyRobotBase::SetActiveDOFValues,SetActiveDOFValues_overloads(PY_ARGS("values","checklimits") DOXY_FN(RobotBase,SetActiveDOFValues)))
#endif
                      .def("GetActiveDOFValues",&PyRobotBase::GetActiveDOFValues, DOXY_FN(RobotBase,GetActiveDOFValues))
                      .def("GetActiveDOFWeights",&PyRobotBase::GetActiveDOFWeights, DOXY_FN(RobotBase,GetActiveDOFWeights))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                      .def("SetActiveDOFVelocities", &PyRobotBase::SetActiveDOFVelocities,
                        "velocities"_a,
                        "checklimits"_a = (int) KinBody::CLA_CheckLimits,
                        DOXY_FN(RobotBase, SetActiveDOFVelocities))
#else
                      .def("SetActiveDOFVelocities",&PyRobotBase::SetActiveDOFVelocities, SetActiveDOFVelocities_overloads(PY_ARGS("velocities","checklimits") DOXY_FN(RobotBase,SetActiveDOFVelocities)))
#endif
                      .def("GetActiveDOFVelocities",&PyRobotBase::GetActiveDOFVelocities, DOXY_FN(RobotBase,GetActiveDOFVelocities))
                      .def("GetActiveDOFLimits",&PyRobotBase::GetActiveDOFLimits, DOXY_FN(RobotBase,GetActiveDOFLimits))
                      .def("GetActiveDOFMaxVel",&PyRobotBase::GetActiveDOFMaxVel, DOXY_FN(RobotBase,GetActiveDOFMaxVel))
                      .def("GetActiveDOFMaxAccel",&PyRobotBase::GetActiveDOFMaxAccel, DOXY_FN(RobotBase,GetActiveDOFMaxAccel))
                      .def("GetActiveDOFMaxJerk",&PyRobotBase::GetActiveDOFMaxJerk, DOXY_FN(RobotBase,GetActiveDOFMaxJerk))
                      .def("GetActiveDOFHardMaxVel",&PyRobotBase::GetActiveDOFHardMaxVel, DOXY_FN(RobotBase,GetActiveDOFHardMaxVel))
                      .def("GetActiveDOFHardMaxAccel",&PyRobotBase::GetActiveDOFHardMaxAccel, DOXY_FN(RobotBase,GetActiveDOFHardMaxAccel))
                      .def("GetActiveDOFHardMaxJerk",&PyRobotBase::GetActiveDOFHardMaxJerk, DOXY_FN(RobotBase,GetActiveDOFHardMaxJerk))
                      .def("GetActiveDOFResolutions",&PyRobotBase::GetActiveDOFResolutions, DOXY_FN(RobotBase,GetActiveDOFResolutions))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                      .def("GetActiveConfigurationSpecification", &PyRobotBase::GetActiveConfigurationSpecification,
                        "interpolation"_a = "",
                        DOXY_FN(RobotBase, GetActiveConfigurationSpecification)
                        )
#else
                      .def("GetActiveConfigurationSpecification",&PyRobotBase::GetActiveConfigurationSpecification, GetActiveConfigurationSpecification_overloads(PY_ARGS("interpolation") DOXY_FN(RobotBase,GetActiveConfigurationSpecification)))
#endif
                      .def("GetActiveJointIndices",&PyRobotBase::GetActiveJointIndices)
                      .def("GetActiveDOFIndices",&PyRobotBase::GetActiveDOFIndices, DOXY_FN(RobotBase,GetActiveDOFIndices))
                      .def("SubtractActiveDOFValues",&PyRobotBase::SubtractActiveDOFValues, PY_ARGS("values0","values1") DOXY_FN(RobotBase,SubtractActiveDOFValues))
                      .def("CalculateActiveJacobian",&PyRobotBase::CalculateActiveJacobian, PY_ARGS("linkindex","offset") DOXY_FN(RobotBase,CalculateActiveJacobian "int; const Vector; std::vector"))
                      .def("CalculateActiveRotationJacobian",&PyRobotBase::CalculateActiveRotationJacobian, PY_ARGS("linkindex","quat") DOXY_FN(RobotBase,CalculateActiveRotationJacobian "int; const Vector; std::vector"))
                      .def("CalculateActiveAngularVelocityJacobian",&PyRobotBase::CalculateActiveAngularVelocityJacobian, PY_ARGS("linkindex") DOXY_FN(RobotBase,CalculateActiveAngularVelocityJacobian "int; std::vector"))
                      .def("Grab",pgrab1, PY_ARGS("body") DOXY_FN(RobotBase,Grab "KinBodyPtr"))
                      .def("Grab",pgrab2, PY_ARGS("body","grablink") DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr"))
                      .def("Grab",pgrab3, PY_ARGS("body","grablink", "linkstoignore") DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr; LinkPtr"))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                      .def("CheckLinkSelfCollision", &PyRobotBase::CheckLinkSelfCollision,
                        "linkindex"_a,
                        "linktrans"_a,
                        "report"_a = py::none_(), // PyCollisionReportPtr(),
                        DOXY_FN(RobotBase,CheckLinkSelfCollision)
                        )
#else
                      .def("CheckLinkSelfCollision", &PyRobotBase::CheckLinkSelfCollision, CheckLinkSelfCollision_overloads(PY_ARGS("linkindex", "linktrans", "report") DOXY_FN(RobotBase,CheckLinkSelfCollision)))
#endif
                      .def("WaitForController",&PyRobotBase::WaitForController,PY_ARGS("timeout") "Wait until the robot controller is done")
                      .def("GetRobotStructureHash",&PyRobotBase::GetRobotStructureHash, DOXY_FN(RobotBase,GetRobotStructureHash))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                      .def("CreateRobotStateSaver",&PyRobotBase::CreateRobotStateSaver,
                        "options"_a = py::none_(),
                        "Creates an object that can be entered using 'with' and returns a RobotStateSaver"
                        )
#else
                      .def("CreateRobotStateSaver",&PyRobotBase::CreateRobotStateSaver, CreateRobotStateSaver_overloads(PY_ARGS("options") "Creates an object that can be entered using 'with' and returns a RobotStateSaver")[return_value_policy<manage_new_object>()])
#endif
                      .def("__repr__", &PyRobotBase::__repr__)
                      .def("__str__", &PyRobotBase::__str__)
                      .def("__unicode__", &PyRobotBase::__unicode__)
        ;
        robot.attr("DOFAffine") = dofaffine; // deprecated (11/10/04)
        robot.attr("ManipulatorInfo") = manipulatorinfo;
        robot.attr("AttachedSensorInfo") = attachedsensorinfo;

        object (PyRobotBase::PyManipulator::*pmanipik)(object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipikf)(object, object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipiks)(object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;
        object (PyRobotBase::PyManipulator::*pmanipiksf)(object, object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;

        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision0)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision1)(object,PyCollisionReportPtr,int) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorSelfCollision0)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckEndEffectorSelfCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorSelfCollision1)(object,PyCollisionReportPtr,int,bool) const = &PyRobotBase::PyManipulator::CheckEndEffectorSelfCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision1)() const = &PyRobotBase::PyManipulator::CheckIndependentCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision2)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckIndependentCollision;

        std::string GetIkParameterization_doc = std::string(DOXY_FN(RobotBase::Manipulator,GetIkParameterization "const IkParameterization; bool")) + std::string(DOXY_FN(RobotBase::Manipulator,GetIkParameterization "IkParameterizationType; bool"));
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<PyRobotBase::PyManipulator, OPENRAVE_SHARED_PTR<PyRobotBase::PyManipulator> >(m, "Manipulator", DOXY_CLASS(RobotBase::Manipulator))
#else
        class_<PyRobotBase::PyManipulator, OPENRAVE_SHARED_PTR<PyRobotBase::PyManipulator> >("Manipulator", DOXY_CLASS(RobotBase::Manipulator), no_init)
#endif
        .def("GetEndEffectorTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetTransformPose", &PyRobotBase::PyManipulator::GetTransformPose, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetVelocity", &PyRobotBase::PyManipulator::GetVelocity, DOXY_FN(RobotBase::Manipulator,GetVelocity))
        .def("GetName",&PyRobotBase::PyManipulator::GetName, DOXY_FN(RobotBase::Manipulator,GetName))
        .def("SetName",&PyRobotBase::PyManipulator::SetName, PY_ARGS("name") DOXY_FN(RobotBase::Manipulator,SetName))
        .def("GetRobot",&PyRobotBase::PyManipulator::GetRobot, DOXY_FN(RobotBase::Manipulator,GetRobot))
        .def("SetIkSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
        .def("GetIkSolver",&PyRobotBase::PyManipulator::GetIkSolver, DOXY_FN(RobotBase::Manipulator,GetIkSolver))
        .def("SetIKSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
        .def("GetNumFreeParameters",&PyRobotBase::PyManipulator::GetNumFreeParameters, DOXY_FN(RobotBase::Manipulator,GetNumFreeParameters))
        .def("GetFreeParameters",&PyRobotBase::PyManipulator::GetFreeParameters, DOXY_FN(RobotBase::Manipulator,GetFreeParameters))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("FindIKSolution", pmanipik,
            "param"_a,
            "filteroptions"_a,
            "ikreturn"_a = false,
            "releasegil"_a = false,
            DOXY_FN(RobotBase::Manipulator, FindIKSolution "const IkParameterization; std::vector; int")
        )
#else
        .def("FindIKSolution",pmanipik,FindIKSolution_overloads(PY_ARGS("param","filteroptions","ikreturn","releasegil") DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; std::vector; int")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("FindIKSolution", pmanipikf,
            "param"_a,
            "freevalues"_a,
            "filteroptions"_a,
            "ikreturn"_a = false,
            "releasegil"_a = false,
            DOXY_FN(RobotBase::Manipulator, FindIKSolution "const IkParameterization; const std::vector; std::vector; int")
        )
#else
        .def("FindIKSolution",pmanipikf,FindIKSolutionFree_overloads(PY_ARGS("param","freevalues","filteroptions","ikreturn","releasegil") DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; const std::vector; std::vector; int")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("FindIKSolutions", pmanipiks,
            "param"_a,
            "filteroptions"_a,
            "ikreturn"_a = false,
            "releasegil"_a = false,
            DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; std::vector; int")
        )
#else
        .def("FindIKSolutions",pmanipiks,FindIKSolutions_overloads(PY_ARGS("param","filteroptions","ikreturn","releasegil") DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; std::vector; int")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("FindIKSolutions", pmanipiksf,
            "param"_a,
            "freevalues"_a,
            "filteroptions"_a,
            "ikreturn"_a = false,
            "releasegil"_a = false,
            DOXY_FN(RobotBase::Manipulator, FindIKSolutions "const IkParameterization; const std::vector; std::vector; int")
        )
#else
        .def("FindIKSolutions",pmanipiksf,FindIKSolutionsFree_overloads(PY_ARGS("param","freevalues","filteroptions","ikreturn","releasegil") DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; const std::vector; std::vector; int")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("GetIkParameterization", &PyRobotBase::PyManipulator::GetIkParameterization,
            "iktype"_a,
            "inworld"_a = true,
            GetIkParameterization_doc.c_str()
        )
#else
        .def("GetIkParameterization",&PyRobotBase::PyManipulator::GetIkParameterization, GetIkParameterization_overloads(PY_ARGS("iktype","inworld") GetIkParameterization_doc.c_str()))
#endif
        .def("GetBase",&PyRobotBase::PyManipulator::GetBase, DOXY_FN(RobotBase::Manipulator,GetBase))
        .def("GetEndEffector",&PyRobotBase::PyManipulator::GetEndEffector, DOXY_FN(RobotBase::Manipulator,GetEndEffector))
        .def("ReleaseAllGrabbed",&PyRobotBase::PyManipulator::ReleaseAllGrabbed, DOXY_FN(RobotBase::Manipulator,ReleaseAllGrabbed))
        .def("GetGraspTransform",&PyRobotBase::PyManipulator::GetGraspTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransform",&PyRobotBase::PyManipulator::GetLocalToolTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransformPose",&PyRobotBase::PyManipulator::GetLocalToolTransformPose, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransformPose))
        .def("SetLocalToolTransform",&PyRobotBase::PyManipulator::SetLocalToolTransform, PY_ARGS("transform") DOXY_FN(RobotBase::Manipulator,SetLocalToolTransform))
        .def("SetLocalToolDirection",&PyRobotBase::PyManipulator::SetLocalToolDirection, PY_ARGS("direction") DOXY_FN(RobotBase::Manipulator,SetLocalToolDirection))
        .def("SetClosingDirection",&PyRobotBase::PyManipulator::SetClosingDirection, PY_ARGS("closingdirection") DOXY_FN(RobotBase::Manipulator,SetClosingDirection))
        .def("SetChuckingDirection",&PyRobotBase::PyManipulator::SetChuckingDirection, PY_ARGS("chuckingdirection") DOXY_FN(RobotBase::Manipulator,SetChuckingDirection))
        .def("GetGripperJoints",&PyRobotBase::PyManipulator::GetGripperJoints, DOXY_FN(RobotBase::Manipulator,GetGripperIndices))
        .def("GetGripperIndices",&PyRobotBase::PyManipulator::GetGripperIndices, DOXY_FN(RobotBase::Manipulator,GetGripperIndices))
        .def("GetArmJoints",&PyRobotBase::PyManipulator::GetArmJoints, DOXY_FN(RobotBase::Manipulator,GetArmIndices))
        .def("GetArmIndices",&PyRobotBase::PyManipulator::GetArmIndices, DOXY_FN(RobotBase::Manipulator,GetArmIndices))
        .def("GetArmDOFValues",&PyRobotBase::PyManipulator::GetArmDOFValues, DOXY_FN(RobotBase::Manipulator,GetArmDOFValues))
        .def("GetGripperDOFValues",&PyRobotBase::PyManipulator::GetGripperDOFValues, DOXY_FN(RobotBase::Manipulator,GetGripperDOFValues))
        .def("GetArmDOF",&PyRobotBase::PyManipulator::GetArmDOF, DOXY_FN(RobotBase::Manipulator,GetArmDOF))
        .def("GetGripperDOF",&PyRobotBase::PyManipulator::GetGripperDOF, DOXY_FN(RobotBase::Manipulator,GetGripperDOF))
        .def("GetClosingDirection",&PyRobotBase::PyManipulator::GetClosingDirection, DOXY_FN(RobotBase::Manipulator,GetClosingDirection))
        .def("GetChuckingDirection",&PyRobotBase::PyManipulator::GetChuckingDirection, DOXY_FN(RobotBase::Manipulator,GetChuckingDirection))
        .def("GetDirection",&PyRobotBase::PyManipulator::GetDirection, DOXY_FN(RobotBase::Manipulator,GetLocalToolDirection))
        .def("GetLocalToolDirection",&PyRobotBase::PyManipulator::GetLocalToolDirection, DOXY_FN(RobotBase::Manipulator,GetLocalToolDirection))
        .def("IsGrabbing",&PyRobotBase::PyManipulator::IsGrabbing, PY_ARGS("body") DOXY_FN(RobotBase::Manipulator,IsGrabbing))
        .def("GetChildJoints",&PyRobotBase::PyManipulator::GetChildJoints, DOXY_FN(RobotBase::Manipulator,GetChildJoints))
        .def("GetChildDOFIndices",&PyRobotBase::PyManipulator::GetChildDOFIndices, DOXY_FN(RobotBase::Manipulator,GetChildDOFIndices))
        .def("GetChildLinks",&PyRobotBase::PyManipulator::GetChildLinks, DOXY_FN(RobotBase::Manipulator,GetChildLinks))
        .def("IsChildLink",&PyRobotBase::PyManipulator::IsChildLink, DOXY_FN(RobotBase::Manipulator,IsChildLink))
        .def("GetIndependentLinks",&PyRobotBase::PyManipulator::GetIndependentLinks, DOXY_FN(RobotBase::Manipulator,GetIndependentLinks))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("GetArmConfigurationSpecification", &PyRobotBase::PyManipulator::GetArmConfigurationSpecification,
            "interpolation"_a = "",
            DOXY_FN(RobotBase::Manipulator, GetArmConfigurationSpecification)
        )
#else
        .def("GetArmConfigurationSpecification",&PyRobotBase::PyManipulator::GetArmConfigurationSpecification, GetArmConfigurationSpecification_overloads(PY_ARGS("interpolation") DOXY_FN(RobotBase::Manipulator,GetArmConfigurationSpecification)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("GetIkConfigurationSpecification", &PyRobotBase::PyManipulator::GetIkConfigurationSpecification,
            "iktype"_a,
            "interpolation"_a = "",
            DOXY_FN(RobotBase::Manipulator,GetIkConfigurationSpecification)
        )
#else
        .def("GetIkConfigurationSpecification",&PyRobotBase::PyManipulator::GetIkConfigurationSpecification, GetIkConfigurationSpecification_overloads(PY_ARGS("iktype", "interpolation") DOXY_FN(RobotBase::Manipulator,GetIkConfigurationSpecification)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("CheckEndEffectorCollision", pCheckEndEffectorCollision1,
            "transform"_a,
            "report"_a = py::none_(), // PyCollisionReportPtr(),
            "numredundantsamples"_a = 0,
            DOXY_FN(RobotBase::Manipulator, CheckEndEffectorCollision)
        )
#else
        .def("CheckEndEffectorCollision",pCheckEndEffectorCollision1,CheckEndEffectorCollision_overloads(PY_ARGS("transform", "report", "numredundantsamples") DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision)))
#endif
        .def("CheckEndEffectorCollision",pCheckEndEffectorCollision0, PY_ARGS("report") DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision))
        .def("CheckEndEffectorSelfCollision",pCheckEndEffectorSelfCollision0, PY_ARGS("report") DOXY_FN(RobotBase::Manipulator,CheckEndEffectorSelfCollision))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("CheckEndEffectorSelfCollision", pCheckEndEffectorSelfCollision1,
            "transform"_a,
            "report"_a = py::none_(), // PyCollisionReportPtr(),
            "numredundantsamples"_a = 0,
            "ignoreManipulatorLinks"_a = false,
            DOXY_FN(RobotBase::Manipulator,CheckEndEffectorSelfCollision)
        )
#else
        .def("CheckEndEffectorSelfCollision",pCheckEndEffectorSelfCollision1,CheckEndEffectorSelfCollision_overloads(PY_ARGS("transform", "report", "numredundantsamples","ignoreManipulatorLinks") DOXY_FN(RobotBase::Manipulator,CheckEndEffectorSelfCollision)))
#endif
        .def("CheckIndependentCollision",pCheckIndependentCollision1, DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
        .def("CheckIndependentCollision",pCheckIndependentCollision2, PY_ARGS("report") DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
        .def("CalculateJacobian",&PyRobotBase::PyManipulator::CalculateJacobian,DOXY_FN(RobotBase::Manipulator,CalculateJacobian))
        .def("CalculateRotationJacobian",&PyRobotBase::PyManipulator::CalculateRotationJacobian,DOXY_FN(RobotBase::Manipulator,CalculateRotationJacobian))
        .def("CalculateAngularVelocityJacobian",&PyRobotBase::PyManipulator::CalculateAngularVelocityJacobian,DOXY_FN(RobotBase::Manipulator,CalculateAngularVelocityJacobian))
        .def("GetStructureHash",&PyRobotBase::PyManipulator::GetStructureHash, DOXY_FN(RobotBase::Manipulator,GetStructureHash))
        .def("GetKinematicsStructureHash",&PyRobotBase::PyManipulator::GetKinematicsStructureHash, DOXY_FN(RobotBase::Manipulator,GetKinematicsStructureHash))
        .def("GetInverseKinematicsStructureHash",&PyRobotBase::PyManipulator::GetInverseKinematicsStructureHash, PY_ARGS("iktype") DOXY_FN(RobotBase::Manipulator,GetInverseKinematicsStructureHash))
        .def("GetInfo",&PyRobotBase::PyManipulator::GetInfo, DOXY_FN(RobotBase::Manipulator,GetInfo))
        .def("__repr__",&PyRobotBase::PyManipulator::__repr__)
        .def("__str__",&PyRobotBase::PyManipulator::__str__)
        .def("__unicode__",&PyRobotBase::PyManipulator::__unicode__)
        .def("__eq__",&PyRobotBase::PyManipulator::__eq__)
        .def("__ne__",&PyRobotBase::PyManipulator::__ne__)
        .def("__hash__",&PyRobotBase::PyManipulator::__hash__)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<PyRobotBase::PyAttachedSensor, OPENRAVE_SHARED_PTR<PyRobotBase::PyAttachedSensor> >(m, "AttachedSensor", DOXY_CLASS(RobotBase::AttachedSensor))
#else
        class_<PyRobotBase::PyAttachedSensor, OPENRAVE_SHARED_PTR<PyRobotBase::PyAttachedSensor> >("AttachedSensor", DOXY_CLASS(RobotBase::AttachedSensor), no_init)
#endif
        .def("GetSensor",&PyRobotBase::PyAttachedSensor::GetSensor, DOXY_FN(RobotBase::AttachedSensor,GetSensor))
        .def("GetAttachingLink",&PyRobotBase::PyAttachedSensor::GetAttachingLink, DOXY_FN(RobotBase::AttachedSensor,GetAttachingLink))
        .def("GetRelativeTransform",&PyRobotBase::PyAttachedSensor::GetRelativeTransform, DOXY_FN(RobotBase::AttachedSensor,GetRelativeTransform))
        .def("GetTransform",&PyRobotBase::PyAttachedSensor::GetTransform, DOXY_FN(RobotBase::AttachedSensor,GetTransform))
        .def("GetTransformPose",&PyRobotBase::PyAttachedSensor::GetTransformPose, DOXY_FN(RobotBase::AttachedSensor,GetTransform))
        .def("GetRobot",&PyRobotBase::PyAttachedSensor::GetRobot, DOXY_FN(RobotBase::AttachedSensor,GetRobot))
        .def("GetName",&PyRobotBase::PyAttachedSensor::GetName, DOXY_FN(RobotBase::AttachedSensor,GetName))
        .def("GetData",&PyRobotBase::PyAttachedSensor::GetData, DOXY_FN(RobotBase::AttachedSensor,GetData))
        .def("SetRelativeTransform",&PyRobotBase::PyAttachedSensor::SetRelativeTransform, PY_ARGS("transform") DOXY_FN(RobotBase::AttachedSensor,SetRelativeTransform))
        .def("GetStructureHash",&PyRobotBase::PyAttachedSensor::GetStructureHash, DOXY_FN(RobotBase::AttachedSensor,GetStructureHash))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("UpdateInfo",&PyRobotBase::PyAttachedSensor::UpdateInfo,
            "type"_a = (int) SensorBase::ST_Invalid,
            DOXY_FN(RobotBase::AttachedSensor, UpdateInfo)
        )
#else
        .def("UpdateInfo",&PyRobotBase::PyAttachedSensor::UpdateInfo, UpdateInfo_overloads(PY_ARGS("type") DOXY_FN(RobotBase::AttachedSensor,UpdateInfo)))
#endif
        .def("GetInfo",&PyRobotBase::PyAttachedSensor::GetInfo, DOXY_FN(RobotBase::AttachedSensor,GetInfo))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("UpdateAndGetInfo", &PyRobotBase::PyAttachedSensor::UpdateAndGetInfo,
            "type"_a = (int) SensorBase::ST_Invalid,
            DOXY_FN(RobotBase::AttachedSensor, UpdateAndGetInfo)
        )
#else
        .def("UpdateAndGetInfo",&PyRobotBase::PyAttachedSensor::UpdateAndGetInfo, UpdateAndGetInfo_overloads(DOXY_FN(RobotBase::AttachedSensor,UpdateAndGetInfo)))
#endif
        .def("__str__",&PyRobotBase::PyAttachedSensor::__str__)
        .def("__repr__",&PyRobotBase::PyAttachedSensor::__repr__)
        .def("__unicode__",&PyRobotBase::PyAttachedSensor::__unicode__)
        .def("__eq__",&PyRobotBase::PyAttachedSensor::__eq__)
        .def("__ne__",&PyRobotBase::PyAttachedSensor::__ne__)
        .def("__hash__",&PyRobotBase::PyAttachedSensor::__hash__)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<PyRobotBase::PyConnectedBody, OPENRAVE_SHARED_PTR<PyRobotBase::PyConnectedBody> >(m, "ConnectedBody", DOXY_CLASS(RobotBase::ConnectedBody))
#else
        class_<PyRobotBase::PyConnectedBody, OPENRAVE_SHARED_PTR<PyRobotBase::PyConnectedBody> >("ConnectedBody", DOXY_CLASS(RobotBase::ConnectedBody), no_init)
#endif
        .def("GetName",&PyRobotBase::PyConnectedBody::GetName, DOXY_FN(RobotBase::ConnectedBody,GetName))
        .def("GetInfo",&PyRobotBase::PyConnectedBody::GetInfo, DOXY_FN(RobotBase::ConnectedBody,GetInfo))
        .def("SetActive", &PyRobotBase::PyConnectedBody::SetActive, DOXY_FN(RobotBase::ConnectedBody,SetActive))
        .def("IsActive", &PyRobotBase::PyConnectedBody::IsActive, DOXY_FN(RobotBase::ConnectedBody,IsActive))
        .def("SetLinkEnable", &PyRobotBase::PyConnectedBody::SetLinkEnable, DOXY_FN(RobotBase::ConnectedBody,SetLinkEnable))
        .def("SetLinkVisible", &PyRobotBase::PyConnectedBody::SetLinkVisible, DOXY_FN(RobotBase::ConnectedBody,SetLinkVisible))
        .def("GetTransform",&PyRobotBase::PyConnectedBody::GetTransform, DOXY_FN(RobotBase::ConnectedBody,GetTransform))
        .def("GetTransformPose",&PyRobotBase::PyConnectedBody::GetTransformPose, DOXY_FN(RobotBase::ConnectedBody,GetTransformPose))
        .def("GetRelativeTransform",&PyRobotBase::PyConnectedBody::GetRelativeTransform, DOXY_FN(RobotBase::ConnectedBody,GetRelativeTransform))
        .def("GetRelativeTransformPose",&PyRobotBase::PyConnectedBody::GetRelativeTransformPose, DOXY_FN(RobotBase::ConnectedBody,GetRelativeTransformPose))
        .def("GetResolvedLinks",&PyRobotBase::PyConnectedBody::GetResolvedLinks, DOXY_FN(RobotBase::ConnectedBody,GetResolvedLinks))
        .def("GetResolvedJoints",&PyRobotBase::PyConnectedBody::GetResolvedJoints, DOXY_FN(RobotBase::ConnectedBody,GetResolvedJoints))
        .def("GetResolvedManipulators",&PyRobotBase::PyConnectedBody::GetResolvedManipulators, DOXY_FN(RobotBase::ConnectedBody,GetResolvedManipulators))
        .def("__str__",&PyRobotBase::PyConnectedBody::__str__)
        .def("__repr__",&PyRobotBase::PyConnectedBody::__repr__)
        .def("__unicode__",&PyRobotBase::PyConnectedBody::__unicode__)
        .def("__eq__",&PyRobotBase::PyConnectedBody::__eq__)
        .def("__ne__",&PyRobotBase::PyConnectedBody::__ne__)
        .def("__hash__",&PyRobotBase::PyConnectedBody::__hash__);

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<PyRobotBase::PyRobotStateSaver, OPENRAVE_SHARED_PTR<PyRobotBase::PyRobotStateSaver> >(m, "RobotStateSaver", DOXY_CLASS(Robot::RobotStateSaver))
        .def(init<PyRobotBasePtr>(), "robot"_a)
        .def(init<PyRobotBasePtr, object>(), "robot"_a, "options"_a)
#else
        class_<PyRobotBase::PyRobotStateSaver, OPENRAVE_SHARED_PTR<PyRobotBase::PyRobotStateSaver> >("RobotStateSaver", DOXY_CLASS(Robot::RobotStateSaver), no_init)
        .def(init<PyRobotBasePtr>(py::args("robot")))
        .def(init<PyRobotBasePtr,object>(py::args("robot","options")))
#endif
        .def("GetBody",&PyRobotBase::PyRobotStateSaver::GetBody,DOXY_FN(Robot::RobotStateSaver, GetBody))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("Restore", &PyRobotBase::PyRobotStateSaver::Restore,
            "body"_a = py::none_(), // PyRobotBasePtr(),
            DOXY_FN(Robot::RobotStateSaver, Restore)
        )
#else
        .def("Restore",&PyRobotBase::PyRobotStateSaver::Restore,Restore_overloads(PY_ARGS("body") DOXY_FN(Robot::RobotStateSaver, Restore)))
#endif
        .def("Release",&PyRobotBase::PyRobotStateSaver::Release,DOXY_FN(Robot::RobotStateSaver, Release))
        .def("__str__",&PyRobotBase::PyRobotStateSaver::__str__)
        .def("__unicode__",&PyRobotBase::PyRobotStateSaver::__unicode__)
        ;
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveCreateRobot",openravepy::RaveCreateRobot, PY_ARGS("env","name") DOXY_FN1(RaveCreateRobot));
#else
    def("RaveCreateRobot",openravepy::RaveCreateRobot, PY_ARGS("env","name") DOXY_FN1(RaveCreateRobot));
#endif
}

}
