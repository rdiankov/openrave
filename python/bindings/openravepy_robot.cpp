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
    _vRestrictGraspSetNames = py::list();
}

PyManipulatorInfo::PyManipulatorInfo(const RobotBase::ManipulatorInfo& info) {
    _Update(info);
}

void PyManipulatorInfo::_Update(const RobotBase::ManipulatorInfo& info) {
    _id = ConvertStringToUnicode(info._id);
    _name = ConvertStringToUnicode(info._name);
    _sBaseLinkName = ConvertStringToUnicode(info._sBaseLinkName);
    _sIkChainEndLinkName = ConvertStringToUnicode(info._sIkChainEndLinkName);
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
    _grippername = ConvertStringToUnicode(info._grippername);
    _toolChangerConnectedBodyToolName = ConvertStringToUnicode(info._toolChangerConnectedBodyToolName);
    _toolChangerLinkName = ConvertStringToUnicode(info._toolChangerLinkName);
    py::list vRestrictGraspSetNames;
    FOREACHC(itname, info._vRestrictGraspSetNames) {
        vRestrictGraspSetNames.append(ConvertStringToUnicode(*itname));
    }
    _vRestrictGraspSetNames = vRestrictGraspSetNames;
}

RobotBase::ManipulatorInfoPtr PyManipulatorInfo::GetManipulatorInfo() const
{
    RobotBase::ManipulatorInfoPtr pinfo(new RobotBase::ManipulatorInfo());
    if( !IS_PYTHONOBJECT_NONE(_id) ) {
        pinfo->_id = py::extract<std::string>(_id);
    }
    if( !IS_PYTHONOBJECT_NONE(_name) ) {
        pinfo->_name = py::extract<std::string>(_name);
    }
    if( !IS_PYTHONOBJECT_NONE(_sBaseLinkName) ) {
        pinfo->_sBaseLinkName = py::extract<std::string>(_sBaseLinkName);
    }
    if( !IS_PYTHONOBJECT_NONE(_sIkChainEndLinkName) ) {
        pinfo->_sIkChainEndLinkName = py::extract<std::string>(_sIkChainEndLinkName);
    }
    if( !IS_PYTHONOBJECT_NONE(_sEffectorLinkName) ) {
        pinfo->_sEffectorLinkName = py::extract<std::string>(_sEffectorLinkName);
    }
    if( !IS_PYTHONOBJECT_NONE(_tLocalTool) ) {
        pinfo->_tLocalTool = ExtractTransform(_tLocalTool);
    }
    if( !IS_PYTHONOBJECT_NONE(_vChuckingDirection) ) {
        pinfo->_vChuckingDirection = ExtractArray<int>(_vChuckingDirection);
    }
    if( !IS_PYTHONOBJECT_NONE(_vdirection) ) {
        pinfo->_vdirection = ExtractVector3(_vdirection);
    }
    pinfo->_sIkSolverXMLId = _sIkSolverXMLId;
    if( !IS_PYTHONOBJECT_NONE(_vGripperJointNames) ) {
        pinfo->_vGripperJointNames = ExtractArray<std::string>(_vGripperJointNames);
    }
    else {
        pinfo->_vGripperJointNames.clear();
    }
    if( !IS_PYTHONOBJECT_NONE(_grippername) ) {
        pinfo->_grippername = py::extract<std::string>(_grippername);
    }
    else {
        RAVELOG_WARN_FORMAT("python manipulator %s has grippername that is None", pinfo->_name);
        pinfo->_grippername.clear();
    }
    if( !IS_PYTHONOBJECT_NONE(_toolChangerConnectedBodyToolName) ) {
        pinfo->_toolChangerConnectedBodyToolName = py::extract<std::string>(_toolChangerConnectedBodyToolName);
    }
    else {
        RAVELOG_WARN_FORMAT("python manipulator %s has toolChangerConnectedBodyToolName that is None", pinfo->_name);
        pinfo->_toolChangerConnectedBodyToolName.clear();
    }
    if( !IS_PYTHONOBJECT_NONE(_toolChangerLinkName) ) {
        pinfo->_toolChangerLinkName = py::extract<std::string>(_toolChangerLinkName);
    }
    else {
        RAVELOG_WARN_FORMAT("python manipulator %s has toolChangerLinkName that is None", pinfo->_name);
        pinfo->_toolChangerLinkName.clear();
    }
    if( !IS_PYTHONOBJECT_NONE(_vRestrictGraspSetNames) ) {
        pinfo->_vRestrictGraspSetNames = ExtractArray<std::string>(_vRestrictGraspSetNames);
    }
    else {
        pinfo->_vRestrictGraspSetNames.clear();
    }
    return pinfo;
}

object PyManipulatorInfo::SerializeJSON(dReal fUnitScale, object options)
{
    rapidjson::Document doc;
    RobotBase::ManipulatorInfoPtr pInfo = GetManipulatorInfo();
    pInfo->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(options, 0));
    return toPyObject(doc);
}

void PyManipulatorInfo::DeserializeJSON(object obj, dReal fUnitScale, object options)
{
    rapidjson::Document doc;
    toRapidJSONValue(obj, doc, doc.GetAllocator());
    RobotBase::ManipulatorInfo info;
    info.DeserializeJSON(doc, fUnitScale, pyGetIntFromPy(options, 0));
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
    _id = ConvertStringToUnicode(info._id);
    _name = ConvertStringToUnicode(info._name);
    _linkname = ConvertStringToUnicode(info._linkname);
    _trelative = ReturnTransform(info._trelative);
    _sensorname = ConvertStringToUnicode(info._sensorname);
    _referenceAttachedSensorName = ConvertStringToUnicode(info._referenceAttachedSensorName);
    _sensorMaker = ConvertStringToUnicode(info._sensorMaker);
    _sensorModel = ConvertStringToUnicode(info._sensorModel);
    _sensorgeometry = toPySensorGeometry(info._sensorname, info._docSensorGeometry);
}

RobotBase::AttachedSensorInfoPtr PyAttachedSensorInfo::GetAttachedSensorInfo() const
{
    RobotBase::AttachedSensorInfoPtr pinfo(new RobotBase::AttachedSensorInfo());
    if( !IS_PYTHONOBJECT_NONE(_id) ) {
        pinfo->_id = py::extract<std::string>(_id);
    }
    if( !IS_PYTHONOBJECT_NONE(_name) ) {
        pinfo->_name = py::extract<std::string>(_name);
    }
    if( !IS_PYTHONOBJECT_NONE(_linkname) ) {
        pinfo->_linkname = py::extract<std::string>(_linkname);
    }
    if( !IS_PYTHONOBJECT_NONE(_trelative) ) {
        pinfo->_trelative = ExtractTransform(_trelative);
    }
    if( !IS_PYTHONOBJECT_NONE(_sensorname) ) {
        pinfo->_sensorname = py::extract<std::string>(_sensorname);
    }
    if( !IS_PYTHONOBJECT_NONE(_referenceAttachedSensorName) ) {
        pinfo->_referenceAttachedSensorName = py::extract<std::string>(_referenceAttachedSensorName);
    }
    if( !IS_PYTHONOBJECT_NONE(_sensorMaker) ) {
        pinfo->_sensorMaker = py::extract<std::string>(_sensorMaker);
    }
    if( !IS_PYTHONOBJECT_NONE(_sensorModel) ) {
        pinfo->_sensorModel = py::extract<std::string>(_sensorModel);
    }
    rapidjson::Document docSensorGeometry;
    if(!!_sensorgeometry) {
        SensorBase::SensorGeometryPtr sensorGeometry = _sensorgeometry->GetGeometry();
        if(!!sensorGeometry) {
            sensorGeometry->SerializeJSON(docSensorGeometry, docSensorGeometry.GetAllocator());
        }
    }
    pinfo->_docSensorGeometry.Swap(docSensorGeometry);
    return pinfo;
}

object PyAttachedSensorInfo::SerializeJSON(dReal fUnitScale, object options)
{
    rapidjson::Document doc;
    RobotBase::AttachedSensorInfoPtr pInfo = GetAttachedSensorInfo();
    pInfo->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(options, 0));
    return toPyObject(doc);
}

void PyAttachedSensorInfo::DeserializeJSON(object obj, dReal fUnitScale, object options)
{
    rapidjson::Document doc;
    toRapidJSONValue(obj, doc, doc.GetAllocator());
    RobotBase::AttachedSensorInfo info;
    info.DeserializeJSON(doc, fUnitScale, pyGetIntFromPy(options, 0));
    _Update(info);
}

PyRobotBase::PyAttachedSensorPtr toPyAttachedSensor(RobotBase::AttachedSensorPtr pAttachedSensor, PyEnvironmentBasePtr pyenv)
{
    if( !!pAttachedSensor ) {
        return PyRobotBase::PyAttachedSensorPtr(new PyRobotBase::PyAttachedSensor(pAttachedSensor, pyenv));
    }
    else {
        return PyRobotBase::PyAttachedSensorPtr();
    }
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
    _id = ConvertStringToUnicode(info._id);
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

    py::list gripperInfos;
    FOREACH(itGripperInfo, info._vGripperInfos) {
        rapidjson::Document rGripperInfo;
        dReal fUnitScale=1;
        int options=0;
        (*itGripperInfo)->SerializeJSON(rGripperInfo, rGripperInfo.GetAllocator(), fUnitScale, options);
        gripperInfos.append(toPyObject(rGripperInfo));
    }
    _gripperInfos = gripperInfos;

    _bIsActive = (int)info._bIsActive;
}

RobotBase::ConnectedBodyInfoPtr PyConnectedBodyInfo::GetConnectedBodyInfo() const
{
    RobotBase::ConnectedBodyInfoPtr pinfo(new RobotBase::ConnectedBodyInfo());
    if( !IS_PYTHONOBJECT_NONE(_id) ) {
        pinfo->_id = py::extract<std::string>(_id);
    }
    if( !IS_PYTHONOBJECT_NONE(_name) ) {
        pinfo->_name = py::extract<std::string>(_name);
    }
    if( !IS_PYTHONOBJECT_NONE(_linkname) ) {
        pinfo->_linkname = py::extract<std::string>(_linkname);
    }
    if( !IS_PYTHONOBJECT_NONE(_trelative) ) {
        pinfo->_trelative = ExtractTransform(_trelative);
    }
    if( !IS_PYTHONOBJECT_NONE(_uri) ) {
        pinfo->_uri = py::extract<std::string>(_uri);
    }
    pinfo->_bIsActive = (int)_bIsActive;
    // extract all the infos
    // links
    std::vector<KinBody::LinkInfoPtr> vLinkInfo = ExtractLinkInfoArray(_linkInfos);
    pinfo->_vLinkInfos.clear();
    pinfo->_vLinkInfos.reserve(vLinkInfo.size());
    FOREACHC(it, vLinkInfo) {
        pinfo->_vLinkInfos.push_back(*it);
    }
    // joints
    std::vector<KinBody::JointInfoPtr> vJointInfos = ExtractJointInfoArray(_jointInfos);
    pinfo->_vJointInfos.clear();
    pinfo->_vJointInfos.reserve(vJointInfos.size());
    FOREACHC(it, vJointInfos) {
        pinfo->_vJointInfos.push_back(*it);
    }
    // manipulators
    std::vector<RobotBase::ManipulatorInfoPtr> vManipulatorInfos = ExtractManipulatorInfoArray(_manipulatorInfos);
    pinfo->_vManipulatorInfos.clear();
    pinfo->_vManipulatorInfos.reserve(vManipulatorInfos.size());
    FOREACHC(it, vManipulatorInfos) {
        pinfo->_vManipulatorInfos.push_back(*it);
    }
    // attachedsensors
    std::vector<RobotBase::AttachedSensorInfoPtr> vAttachedSensorInfos = ExtractAttachedSensorInfoArray(_attachedSensorInfos);
    pinfo->_vAttachedSensorInfos.clear();
    pinfo->_vAttachedSensorInfos.reserve(vAttachedSensorInfos.size());
    FOREACHC(it, vAttachedSensorInfos) {
        pinfo->_vAttachedSensorInfos.push_back(*it);
    }
    // gripperinfos
    std::vector<RobotBase::GripperInfoPtr> vGripperInfos = ExtractGripperInfoArray(_gripperInfos);
    pinfo->_vGripperInfos.clear();
    pinfo->_vGripperInfos.reserve(vGripperInfos.size());
    FOREACHC(it, vGripperInfos) {
        pinfo->_vGripperInfos.push_back(*it);
    }
    return pinfo;
}

object PyConnectedBodyInfo::SerializeJSON(dReal fUnitScale, object options)
{
    rapidjson::Document doc;
    RobotBase::ConnectedBodyInfoPtr pInfo = GetConnectedBodyInfo();
    pInfo->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(options, 0));
    return toPyObject(doc);
}

void PyConnectedBodyInfo::DeserializeJSON(object obj, dReal fUnitScale, object options)
{
    rapidjson::Document doc;
    toRapidJSONValue(obj, doc, doc.GetAllocator());
    RobotBase::ConnectedBodyInfoPtr pCurrentInfo = GetConnectedBodyInfo();
    RobotBase::ConnectedBodyInfo info = *pCurrentInfo;
    info.DeserializeJSON(doc, fUnitScale, pyGetIntFromPy(options, 0));
    _Update(info);
}

PyConnectedBodyInfoPtr toPyConnectedBodyInfo(const RobotBase::ConnectedBodyInfo& connectedBodyInfo)
{
    return PyConnectedBodyInfoPtr(new PyConnectedBodyInfo(connectedBodyInfo));
}

RobotBasePtr PyRobotBase::GetRobot() {
    return _probot;
}

PyRobotBase::PyRobotBaseInfo::PyRobotBaseInfo() : PyKinBodyInfo() {
}

PyRobotBase::PyRobotBaseInfo::PyRobotBaseInfo(const RobotBase::RobotBaseInfo& info) {
    _Update(info);
}

py::object PyRobotBase::PyRobotBaseInfo::SerializeJSON(dReal fUnitScale, py::object options) {
    rapidjson::Document doc;
    RobotBase::RobotBaseInfoPtr pInfo = GetRobotBaseInfo();
    pInfo->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(options, 0));
    return toPyObject(doc);
}

void PyRobotBase::PyRobotBaseInfo::DeserializeJSON(py::object obj, dReal fUnitScale, object options)
{
    rapidjson::Document doc;
    toRapidJSONValue(obj, doc, doc.GetAllocator());
    RobotBase::RobotBaseInfoPtr pCurrentInfo = GetRobotBaseInfo();
    RobotBase::RobotBaseInfo info = *pCurrentInfo;
    info.DeserializeJSON(doc, fUnitScale, pyGetIntFromPy(options, 0));
    _Update(info);
}

void PyRobotBase::PyRobotBaseInfo::_Update(const RobotBase::RobotBaseInfo& info) {
    PyKinBody::PyKinBodyInfo::_Update(info);
    py::list vManipulatorInfos;
    FOREACHC(itManipulatorInfo, info._vManipulatorInfos) {
        PyManipulatorInfoPtr pmanipinfo = toPyManipulatorInfo(**itManipulatorInfo);
        vManipulatorInfos.append(pmanipinfo);
    }
    _vManipulatorInfos = vManipulatorInfos;

    py::list vAttachedSensorInfos;
    FOREACHC(itAttachedSensorInfo, info._vAttachedSensorInfos) {
        PyAttachedSensorInfoPtr pattachedsensorinfo = toPyAttachedSensorInfo(**itAttachedSensorInfo);
        vAttachedSensorInfos.append(pattachedsensorinfo);
    }
    _vAttachedSensorInfos = vAttachedSensorInfos;

    py::list vConnectedBodyInfos;
    FOREACHC(itConnectedBodyInfo, info._vConnectedBodyInfos) {
        PyConnectedBodyInfoPtr pconnectedbodyinfo = toPyConnectedBodyInfo(**itConnectedBodyInfo);
        vConnectedBodyInfos.append(pconnectedbodyinfo);
    }
    _vConnectedBodyInfos = vConnectedBodyInfos;

    py::list vGripperInfos;
    FOREACHC(itGripperInfo, info._vGripperInfos) {
        rapidjson::Document rGripperInfo;
        dReal fUnitScale = 1;
        int options = 0;
        (*itGripperInfo)->SerializeJSON(rGripperInfo, rGripperInfo.GetAllocator(), fUnitScale, options);
        vGripperInfos.append(toPyObject(rGripperInfo));
    }
    _vGripperInfos = vGripperInfos;
}


std::vector<RobotBase::ManipulatorInfoPtr> ExtractManipulatorInfoArray(object pyManipList)
{
    if( IS_PYTHONOBJECT_NONE(pyManipList) ) {
        return {};
    }
    std::vector<RobotBase::ManipulatorInfoPtr> vManipulatorInfos;
    try {
        const size_t arraySize = len(pyManipList);
        vManipulatorInfos.resize(arraySize);

        for(size_t iManipulatorInfo = 0; iManipulatorInfo < arraySize; iManipulatorInfo++) {
            extract_<OPENRAVE_SHARED_PTR<PyManipulatorInfo> > pymanipinfo(pyManipList[py::to_object(iManipulatorInfo)]);
            if (pymanipinfo.check()) {
                vManipulatorInfos[iManipulatorInfo] = ((OPENRAVE_SHARED_PTR<PyManipulatorInfo>)pymanipinfo)->GetManipulatorInfo();
            }
            else{
                throw openrave_exception(_("Bad ManipulatorInfo"));
            }
        }
    }
    catch(...) {
        RAVELOG_WARN("Cannot do ExtractArray for ManipulatorInfo");
    }
    return vManipulatorInfos;
}

std::vector<RobotBase::AttachedSensorInfoPtr> ExtractAttachedSensorInfoArray(object pyAttachedSensorInfoList)
{
    if( IS_PYTHONOBJECT_NONE(pyAttachedSensorInfoList) ) {
        return {};
    }
    std::vector<RobotBase::AttachedSensorInfoPtr> vAttachedSensorInfos;
    try {
        const size_t arraySize = len(pyAttachedSensorInfoList);
        vAttachedSensorInfos.resize(arraySize);

        for(size_t iAttachedSensorInfo = 0; iAttachedSensorInfo < arraySize; iAttachedSensorInfo++) {
            extract_<OPENRAVE_SHARED_PTR<PyAttachedSensorInfo> > pyattachensensorinfo(pyAttachedSensorInfoList[py::to_object(iAttachedSensorInfo)]);
            if (pyattachensensorinfo.check()) {
                vAttachedSensorInfos[iAttachedSensorInfo] = ((OPENRAVE_SHARED_PTR<PyAttachedSensorInfo>)pyattachensensorinfo)->GetAttachedSensorInfo();
            }
            else{
                throw openrave_exception(_("Bad AttachedSensorInfo"));
            }
        }
    }
    catch(...) {
        RAVELOG_WARN("Cannot do ExtractArray for AttachedSensorInfo");
    }
    return vAttachedSensorInfos;
}

std::vector<RobotBase::ConnectedBodyInfoPtr> ExtractConnectedBodyInfoArray(object pyConnectedBodyInfoList)
{
    if( IS_PYTHONOBJECT_NONE(pyConnectedBodyInfoList) ) {
        return {};
    }
    std::vector<RobotBase::ConnectedBodyInfoPtr> vConnectedBodyInfos;
    try {
        const size_t arraySize = len(pyConnectedBodyInfoList);
        vConnectedBodyInfos.resize(arraySize);

        for(size_t iConnectedBodyInfo = 0; iConnectedBodyInfo < arraySize; iConnectedBodyInfo++) {
            extract_<OPENRAVE_SHARED_PTR<PyConnectedBodyInfo> > pyconnectedbodyinfo(pyConnectedBodyInfoList[py::to_object(iConnectedBodyInfo)]);
            if (pyconnectedbodyinfo.check()) {
                vConnectedBodyInfos[iConnectedBodyInfo] = ((OPENRAVE_SHARED_PTR<PyConnectedBodyInfo>)pyconnectedbodyinfo)->GetConnectedBodyInfo();
            }
            else{
                throw openrave_exception(_("Bad ConnectedBodyInfo"));
            }
        }
    }
    catch(...) {
        RAVELOG_WARN("Cannot do ExtractArray for ConnectedBodyInfo");
    }
    return vConnectedBodyInfos;
}

std::vector<RobotBase::GripperInfoPtr> ExtractGripperInfoArray(object pyGripperInfoList)
{
    if(IS_PYTHONOBJECT_NONE(pyGripperInfoList)) {
        return {};
    }
    std::vector<RobotBase::GripperInfoPtr> vGripperInfos;
    try {
        const size_t arraySize = len(pyGripperInfoList);
        vGripperInfos.reserve(arraySize);
        dReal fUnitScale=1;
        int options = 0;
        for(size_t iGripperInfo = 0; iGripperInfo < arraySize; iGripperInfo++) {
            RobotBase::GripperInfoPtr pInfo(new RobotBase::GripperInfo());
            rapidjson::Document rGripperInfo;
            toRapidJSONValue(pyGripperInfoList[py::to_object(iGripperInfo)], rGripperInfo, rGripperInfo.GetAllocator());
            pInfo->DeserializeJSON(rGripperInfo, fUnitScale, options);
            vGripperInfos.push_back(pInfo);
        }
    }
    catch(...) {
        RAVELOG_WARN("Cannot do ExtractArray for GripperInfo");
    }
    return vGripperInfos;
}


RobotBase::RobotBaseInfoPtr PyRobotBase::PyRobotBaseInfo::GetRobotBaseInfo() const {
    RobotBase::RobotBaseInfoPtr pInfo(new RobotBase::RobotBaseInfo);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    pInfo->_id = _id;
    pInfo->_name = _name;
    pInfo->_interfaceType = _interfaceType;
    pInfo->_uri = _uri;
    pInfo->_referenceUri = _referenceUri;
#else
    if (!IS_PYTHONOBJECT_NONE(_id)) {
        pInfo->_id = py::extract<std::string>(_id);
    }
    if (!IS_PYTHONOBJECT_NONE(_name)) {
        pInfo->_name = py::extract<std::string>(_name);
    }
    if (!IS_PYTHONOBJECT_NONE(_interfaceType)) {
        pInfo->_interfaceType = py::extract<std::string>(_interfaceType);
    }
    if (!IS_PYTHONOBJECT_NONE(_uri)) {
        pInfo->_uri = py::extract<std::string>(_uri);
    }
    if (!IS_PYTHONOBJECT_NONE(_referenceUri)) {
        pInfo->_referenceUri = py::extract<std::string>(_referenceUri);
    }
#endif
    pInfo->_isRobot = true;
    std::vector<KinBody::LinkInfoPtr> vLinkInfos = ExtractLinkInfoArray(_vLinkInfos);
    pInfo->_vLinkInfos.clear();
    pInfo->_vLinkInfos.reserve(vLinkInfos.size());
    FOREACHC(it, vLinkInfos) {
        pInfo->_vLinkInfos.push_back(*it);
    }
    std::vector<KinBody::JointInfoPtr> vJointInfos = ExtractJointInfoArray(_vJointInfos);
    pInfo->_vJointInfos.clear();
    pInfo->_vJointInfos.reserve(vJointInfos.size());
    FOREACHC(it, vJointInfos) {
        pInfo->_vJointInfos.push_back(*it);
    }
    std::vector<KinBody::GrabbedInfoPtr> vGrabbedInfos = ExtractGrabbedInfoArray(_vGrabbedInfos);
    pInfo->_vGrabbedInfos.clear();
    pInfo->_vGrabbedInfos.reserve(vGrabbedInfos.size());
    FOREACHC(it, vGrabbedInfos) {
        pInfo->_vGrabbedInfos.push_back(*it);
    }
    std::vector<RobotBase::ManipulatorInfoPtr> vManipulatorInfos = ExtractManipulatorInfoArray(_vManipulatorInfos);
    pInfo->_vManipulatorInfos.clear();
    pInfo->_vManipulatorInfos.reserve(vManipulatorInfos.size());
    FOREACHC(it, vManipulatorInfos) {
        pInfo->_vManipulatorInfos.push_back(*it);
    }
    std::vector<RobotBase::AttachedSensorInfoPtr> vAttachedSensorInfos = ExtractAttachedSensorInfoArray(_vAttachedSensorInfos);
    pInfo->_vAttachedSensorInfos.clear();
    pInfo->_vAttachedSensorInfos.reserve(vAttachedSensorInfos.size());
    FOREACHC(it, vAttachedSensorInfos) {
        pInfo->_vAttachedSensorInfos.push_back(*it);
    }
    std::vector<RobotBase::ConnectedBodyInfoPtr> vConnectedBodyInfos = ExtractConnectedBodyInfoArray(_vConnectedBodyInfos);
    pInfo->_vConnectedBodyInfos.clear();
    pInfo->_vConnectedBodyInfos.reserve(vConnectedBodyInfos.size());
    FOREACHC(it, vConnectedBodyInfos) {
        pInfo->_vConnectedBodyInfos.push_back(*it);
    }

    std::vector<RobotBase::GripperInfoPtr> vGripperInfos = ExtractGripperInfoArray(_vGripperInfos);
    pInfo->_vGripperInfos.clear();
    pInfo->_vGripperInfos.reserve(vGripperInfos.size());
    FOREACHC(it, vGripperInfos) {
        pInfo->_vGripperInfos.push_back(*it);
    }
    pInfo->_transform = ExtractTransform(_transform);
    pInfo->_dofValues = ExtractDOFValuesArray(_dofValues);
    pInfo->_mReadableInterfaces = ExtractReadableInterfaces(_readableInterfaces);
    return pInfo;
}


RobotBase::RobotBaseInfoPtr ExtractRobotBaseInfo(object obj)
{
    extract_<PyRobotBase::PyRobotBaseInfoPtr> pyRobotBaseInfo(obj);
    if (pyRobotBaseInfo.check()) {
        return (PyRobotBase::PyRobotBaseInfoPtr(pyRobotBaseInfo))->GetRobotBaseInfo();
    }
    return NULL;
}

bool PyRobotBase::InitFromRobotInfo(const object pyRobotBaseInfo)
{
    RobotBase::RobotBaseInfoPtr pRobotBaseInfo = ExtractRobotBaseInfo(pyRobotBaseInfo);
    if(!!pRobotBaseInfo) {
        return _probot->InitFromRobotInfo(*pRobotBaseInfo);
    }
    return false;
}

std::string PyRobotBase::PyRobotBaseInfo::__str__() {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    return boost::str(boost::format("<RobotBaseInfo: %s>")%_uri);
#else
    std::string uri = "";
    if (!IS_PYTHONOBJECT_NONE(_uri)) {
        uri = extract<std::string>(_uri);
    }
    return boost::str(boost::format("<RobotBaseInfo: %s")%uri);
#endif
}

py::object PyRobotBase::PyRobotBaseInfo::__unicode__() {
    return ConvertStringToUnicode(__str__());
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
    const std::pair<Vector, Vector> velocity = _pmanip->GetVelocity();
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pyvalues(6);
    py::buffer_info buf = pyvalues.request();
    dReal* pvalue = (dReal*) buf.ptr;
    pvalue[0] = velocity.first.x;
    pvalue[1] = velocity.first.y;
    pvalue[2] = velocity.first.z;
    pvalue[3] = velocity.second.x;
    pvalue[4] = velocity.second.y;
    pvalue[5] = velocity.second.z;
    return pyvalues;
#else
    boost::array<dReal,6> v = {{ velocity.first.x, velocity.first.y, velocity.first.z, velocity.second.x, velocity.second.y, velocity.second.z}};
    return toPyArray<dReal,6>(v);
#endif
}

std::string PyRobotBase::PyManipulator::GetId() const {
    return _pmanip->GetId();
}

object PyRobotBase::PyManipulator::GetName() const {
    return ConvertStringToUnicode(_pmanip->GetName());
}

object PyRobotBase::PyManipulator::GetGripperName() const {
    return ConvertStringToUnicode(_pmanip->GetGripperName());
}

object PyRobotBase::PyManipulator::GetToolChangerConnectedBodyToolName() const {
    return ConvertStringToUnicode(_pmanip->GetToolChangerConnectedBodyToolName());
}

object PyRobotBase::PyManipulator::GetToolChangerLinkName() const {
    return ConvertStringToUnicode(_pmanip->GetToolChangerLinkName());
}

object PyRobotBase::PyManipulator::GetRestrictGraspSetNames() const {
    py::list names;
    FOREACHC(itname, _pmanip->GetRestrictGraspSetNames()) {
        names.append(ConvertStringToUnicode(*itname));
    }
    return names;
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
object PyRobotBase::PyManipulator::GetIkChainEndLink() {
    return toPyKinBodyLink(_pmanip->GetIkChainEndLink(),_pyenv);
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
bool PyRobotBase::PyManipulator::_FindIKSolution(const IkParameterization& ikparam, int filteroptions, IkReturn& ikreturn, bool releasegil, IkFailureAccumulatorBasePtr paccumulator) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolution(ikparam,filteroptions,IkReturnPtr(&ikreturn,utils::null_deleter()),paccumulator);
}
bool PyRobotBase::PyManipulator::_FindIKSolution(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturn& ikreturn, bool releasegil, IkFailureAccumulatorBasePtr paccumulator) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolution(ikparam,vFreeParameters, filteroptions,IkReturnPtr(&ikreturn,utils::null_deleter()),paccumulator);
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
bool PyRobotBase::PyManipulator::_FindIKSolutions(const IkParameterization& ikparam, int filteroptions, std::vector<IkReturnPtr>& vikreturns, bool releasegil, IkFailureAccumulatorBasePtr paccumulator) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolutions(ikparam,filteroptions,vikreturns,paccumulator);
}
bool PyRobotBase::PyManipulator::_FindIKSolutions(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& vikreturns, bool releasegil, IkFailureAccumulatorBasePtr paccumulator) const
{
    openravepy::PythonThreadSaverPtr statesaver;
    if( releasegil ) {
        statesaver.reset(new openravepy::PythonThreadSaver());
    }
    return _pmanip->FindIKSolutions(ikparam,vFreeParameters,filteroptions,vikreturns,paccumulator);
}

object PyRobotBase::PyManipulator::FindIKSolution(object oparam, int filteroptions, bool ikreturn, bool releasegil, PyIkFailureAccumulatorBasePtr pyaccumulator) const
{
    IkParameterization ikparam;
    EnvironmentLock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
    if( ExtractIkParameterization(oparam,ikparam) ) {
        if( ikreturn ) {
            IkReturn ikreject(IKRA_Reject);
            IkFailureAccumulatorBasePtr paccumulator;
            if( !!pyaccumulator ) {
                paccumulator = pyaccumulator->_pIkFailureAccumulator;
            }
            _FindIKSolution(ikparam,filteroptions,ikreject,releasegil,paccumulator);
            return openravepy::toPyIkReturn(ikreject);
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
            IkReturn ikreject(IKRA_Reject);
            IkFailureAccumulatorBasePtr paccumulator;
            if( !!pyaccumulator ) {
                paccumulator = pyaccumulator->_pIkFailureAccumulator;
            }
            _FindIKSolution(ExtractTransform(oparam),filteroptions,ikreject,releasegil,paccumulator);
            return openravepy::toPyIkReturn(ikreject);
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

object PyRobotBase::PyManipulator::FindIKSolution(object oparam, object freeparams, int filteroptions, bool ikreturn, bool releasegil, PyIkFailureAccumulatorBasePtr pyaccumulator) const
{
    std::vector<dReal> vfreeparams = ExtractArray<dReal>(freeparams);
    IkParameterization ikparam;
    EnvironmentLock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
    if( ExtractIkParameterization(oparam,ikparam) ) {
        if( ikreturn ) {
            IkReturn ikreject(IKRA_Reject);
            IkFailureAccumulatorBasePtr paccumulator;
            if( !!pyaccumulator ) {
                paccumulator = pyaccumulator->_pIkFailureAccumulator;
            }
            _FindIKSolution(ikparam,vfreeparams,filteroptions,ikreject,releasegil,paccumulator);
            return openravepy::toPyIkReturn(ikreject);
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
            IkReturn ikreject(IKRA_Reject);
            IkFailureAccumulatorBasePtr paccumulator;
            if( !!pyaccumulator ) {
                paccumulator = pyaccumulator->_pIkFailureAccumulator;
            }
            _FindIKSolution(ExtractTransform(oparam),vfreeparams,filteroptions,ikreject,releasegil,paccumulator);
            return openravepy::toPyIkReturn(ikreject);
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

object PyRobotBase::PyManipulator::FindIKSolutions(object oparam, int filteroptions, bool ikreturn, bool releasegil, PyIkFailureAccumulatorBasePtr pyaccumulator) const
{
    IkParameterization ikparam;
    EnvironmentLock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
    if( ikreturn ) {
        std::vector<IkReturnPtr> vikreturns;
        IkFailureAccumulatorBasePtr paccumulator;
        if( !!pyaccumulator ) {
            paccumulator = pyaccumulator->_pIkFailureAccumulator;
        }
        if( ExtractIkParameterization(oparam,ikparam) ) {
            _FindIKSolutions(ikparam, filteroptions, vikreturns, releasegil, paccumulator);
        }
        else {
            // assume transformation matrix
            _FindIKSolutions(ExtractTransform(oparam), filteroptions, vikreturns, releasegil, paccumulator);
        }
        // When the ik failure accumulator is given, vikreturns can be non-empty even though there are no valid ik solutions.
        if( vikreturns.empty() ) {
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

        const size_t nSolutions = vsolutions.size();
        const size_t nArmIndices = _pmanip->GetArmIndices().size();
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        py::array_t<dReal> pysolutions({nSolutions, nArmIndices});
        py::buffer_info buf = pysolutions.request();
        dReal* ppos = (dReal*) buf.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
        npy_intp dims[] = { npy_intp(nSolutions), npy_intp(nArmIndices) };
        PyObject *pysolutions = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* ppos = (dReal*)PyArray_DATA(pysolutions);
#endif // USE_PYBIND11_PYTHON_BINDINGS
        for(const std::vector<dReal>& solution : vsolutions) {
            BOOST_ASSERT(solution.size() == nArmIndices);
            std::copy(begin(solution), end(solution), ppos);
            ppos += nArmIndices;
        }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        return pysolutions;
#else // USE_PYBIND11_PYTHON_BINDINGS
        return py::to_array_astype<dReal>(pysolutions);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    }
}

object PyRobotBase::PyManipulator::FindIKSolutions(object oparam, object freeparams, int filteroptions, bool ikreturn, bool releasegil, PyIkFailureAccumulatorBasePtr pyaccumulator) const
{
    std::vector<dReal> vfreeparams = ExtractArray<dReal>(freeparams);
    IkParameterization ikparam;
    EnvironmentLock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
    if( ikreturn ) {
        std::vector<IkReturnPtr> vikreturns;
        IkFailureAccumulatorBasePtr paccumulator;
        if( !!pyaccumulator ) {
            paccumulator = pyaccumulator->_pIkFailureAccumulator;
        }
        if( ExtractIkParameterization(oparam,ikparam) ) {
            _FindIKSolutions(ikparam, vfreeparams, filteroptions, vikreturns, releasegil, paccumulator);
        }
        // assume transformation matrix
        else {
            _FindIKSolutions(ExtractTransform(oparam), vfreeparams, filteroptions, vikreturns, releasegil, paccumulator);
        }
        // When paccumulator is given, vikreturns can be non-empty even though there are no valid ik solutions.
        if( vikreturns.empty() ) {
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

        const size_t nSolutions = vsolutions.size();
        const size_t nArmIndices = _pmanip->GetArmIndices().size();
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        py::array_t<dReal> pysolutions({nSolutions, nArmIndices});
        py::buffer_info buf = pysolutions.request();
        dReal* ppos = (dReal*) buf.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
        npy_intp dims[] = { npy_intp(nSolutions), npy_intp(nArmIndices) };
        PyObject *pysolutions = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* ppos = (dReal*)PyArray_DATA(pysolutions);
#endif // USE_PYBIND11_PYTHON_BINDINGS
        for(const std::vector<dReal>& solution : vsolutions) {
            BOOST_ASSERT(solution.size() == nArmIndices);
            std::copy(begin(solution), end(solution), ppos);
            ppos += nArmIndices;
        }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        return pysolutions;
#else // USE_PYBIND11_PYTHON_BINDINGS
        return py::to_array_astype<dReal>(pysolutions);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    }
}

object PyRobotBase::PyManipulator::ConvertIkParameterization(object sourceikp, object oparam, bool inworld)
{
    IkParameterization sourceikparam,ikparam;
    ExtractIkParameterization(sourceikp,sourceikparam);
    if( ExtractIkParameterization(oparam,ikparam) ) {
        return toPyIkParameterization(_pmanip->ConvertIkParameterization(sourceikparam,ikparam,inworld));
    }
    // must be IkParameterizationType
    return toPyIkParameterization(_pmanip->ConvertIkParameterization(sourceikparam,(IkParameterizationType)extract<IkParameterizationType>(oparam),inworld));
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
    CollisionReport report;
    CollisionReportPtr preport;
    if( !!pyreport ) {
        preport = CollisionReportPtr(&report,utils::null_deleter());
    }

    bool bcollision = _pmanip->CheckEndEffectorCollision(preport);
    if( !!pyreport ) {
        pyreport->Init(report);
    }
    return bcollision;
}

bool PyRobotBase::PyManipulator::CheckEndEffectorCollision(object otrans, PyCollisionReportPtr pyreport, int numredundantsamples) const
{
    CollisionReport report;
    CollisionReportPtr preport;
    if( !!pyreport ) {
        preport = CollisionReportPtr(&report,utils::null_deleter());
    }

    bool bCollision;
    IkParameterization ikparam;
    if( ExtractIkParameterization(otrans,ikparam) ) {
        bCollision = _pmanip->CheckEndEffectorCollision(ikparam, preport, numredundantsamples);
    }
    else {
        bCollision = _pmanip->CheckEndEffectorCollision(ExtractTransform(otrans),preport, numredundantsamples);
    }
    if( !!pyreport ) {
        pyreport->Init(report);
    }
    return bCollision;
}

bool PyRobotBase::PyManipulator::CheckEndEffectorSelfCollision(PyCollisionReportPtr pyreport) const
{
    CollisionReport report;
    CollisionReportPtr preport;
    if( !!pyreport ) {
        preport = CollisionReportPtr(&report,utils::null_deleter());
    }

    bool bcollision = _pmanip->CheckEndEffectorSelfCollision(preport);
    if( !!pyreport ) {
        pyreport->Init(report);
    }
    return bcollision;
}

bool PyRobotBase::PyManipulator::CheckEndEffectorSelfCollision(object otrans, PyCollisionReportPtr pyreport, int numredundantsamples, bool ignoreManipulatorLinks) const
{
    CollisionReport report;
    CollisionReportPtr preport;
    if( !!pyreport ) {
        preport = CollisionReportPtr(&report,utils::null_deleter());
    }

    bool bCollision;
    IkParameterization ikparam;
    if( ExtractIkParameterization(otrans,ikparam) ) {
        bCollision = _pmanip->CheckEndEffectorSelfCollision(ikparam, preport, numredundantsamples, ignoreManipulatorLinks);
    }
    else {
        bCollision = _pmanip->CheckEndEffectorSelfCollision(ExtractTransform(otrans), preport, numredundantsamples, ignoreManipulatorLinks);
    }
    if( !!pyreport ) {
        pyreport->Init(report);
    }
    return bCollision;
}

bool PyRobotBase::PyManipulator::CheckIndependentCollision() const
{
    return _pmanip->CheckIndependentCollision();
}
bool PyRobotBase::PyManipulator::CheckIndependentCollision(PyCollisionReportPtr pyreport) const
{
    CollisionReport report;
    CollisionReportPtr preport;
    if( !!pyreport ) {
        preport = CollisionReportPtr(&report,utils::null_deleter());
    }

    bool bCollision = _pmanip->CheckIndependentCollision(preport);
    if( !!pyreport ) {
        pyreport->Init(report);
    }
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
    return !_pattached->GetRobot() ? PyRobotBasePtr() : PyRobotBasePtr(new PyRobotBase(_pattached->GetRobot(), _pyenv));
}
std::string PyRobotBase::PyAttachedSensor::GetId() const {
    return _pattached->GetId();
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

std::string PyRobotBase::PyConnectedBody::GetId() const {
    return _pconnected->GetId();
}

object PyRobotBase::PyConnectedBody::GetName() const {
    return ConvertStringToUnicode(_pconnected->GetName());
}

object PyRobotBase::PyConnectedBody::GetInfo() {
    return py::to_object(PyConnectedBodyInfoPtr(new PyConnectedBodyInfo(_pconnected->GetInfo())));
}

bool PyRobotBase::PyConnectedBody::SetActive(int active) {
    return _pconnected->SetActive(active);
}

int PyRobotBase::PyConnectedBody::IsActive() {
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

object PyRobotBase::PyConnectedBody::GetResolvedAttachedSensors()
{
    py::list oattachedSensors;
    std::vector<RobotBase::AttachedSensorPtr> vattachedSensors;
    _pconnected->GetResolvedAttachedSensors(vattachedSensors);
    FOREACH(itattachedSensor, vattachedSensors) {
        oattachedSensors.append(toPyAttachedSensor(*itattachedSensor, _pyenv));
    }
    return oattachedSensors;
}

object PyRobotBase::PyConnectedBody::GetResolvedGripperInfos()
{
    py::list pyGripperInfos;
    std::vector<RobotBase::GripperInfoPtr> vgripperInfos;
    _pconnected->GetResolvedGripperInfos(vgripperInfos);
    FOREACH(itGripperInfo, vgripperInfos) {
        rapidjson::Document rGripperInfo;
        dReal fUnitScale=1;
        int options=0;
        (*itGripperInfo)->SerializeJSON(rGripperInfo, rGripperInfo.GetAllocator(), fUnitScale, options);
        pyGripperInfos.append(toPyObject(rGripperInfo));
    }
    return pyGripperInfos;
}

bool PyRobotBase::PyConnectedBody::CanProvideManipulator(const std::string& resolvedManipulatorName)
{
    return _pconnected->CanProvideManipulator(resolvedManipulatorName);
}

std::string PyRobotBase::PyConnectedBody::GetInfoHash()
{
    return _pconnected->GetInfoHash();
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

bool PyRobotBase::Init(object olinkinfos, object ojointinfos, object omanipinfos, object oattachedsensorinfos, const std::string& uri) {
    std::vector<KinBody::LinkInfoConstPtr> vlinkinfos;
    _ParseLinkInfos(olinkinfos, vlinkinfos);
    std::vector<KinBody::JointInfoConstPtr> vjointinfos;
    _ParseJointInfos(ojointinfos, vjointinfos);
    std::vector<RobotBase::ManipulatorInfoConstPtr> vmanipinfos(len(omanipinfos));
    for(size_t i = 0; i < vmanipinfos.size(); ++i) {
        PyManipulatorInfoPtr pymanip = py::extract<PyManipulatorInfoPtr>(omanipinfos[py::to_object(i)]);
        if( !pymanip ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.ManipulatorInfo"),ORE_InvalidArguments);
        }
        vmanipinfos[i] = pymanip->GetManipulatorInfo();
    }
    std::vector<RobotBase::AttachedSensorInfoConstPtr> vattachedsensorinfos(len(oattachedsensorinfos));
    for(size_t i = 0; i < vattachedsensorinfos.size(); ++i) {
        PyAttachedSensorInfoPtr pyattachedsensor = py::extract<PyAttachedSensorInfoPtr>(oattachedsensorinfos[py::to_object(i)]);
        if( !pyattachedsensor ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.AttachedsensorInfo"),ORE_InvalidArguments);
        }
        vattachedsensorinfos[i] = pyattachedsensor->GetAttachedSensorInfo();
    }
    return _probot->Init(vlinkinfos, vjointinfos, vmanipinfos, vattachedsensorinfos, uri);
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

object PyRobotBase::ExtractInfo(ExtractInfoOptions options) const
{
    RobotBase::RobotBaseInfo info;
    _probot->ExtractInfo(info, options);
    return py::to_object(boost::shared_ptr<PyRobotBase::PyRobotBaseInfo>(new PyRobotBase::PyRobotBaseInfo(info)));
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
    std::vector<int8_t> activestates;
    _probot->GetConnectedBodyActiveStates(activestates);
    return toPyArray(activestates);
}

void PyRobotBase::SetConnectedBodyActiveStates(object oactivestates)
{
    std::vector<int8_t> activestates = ExtractArrayInt8(oactivestates);
    _probot->SetConnectedBodyActiveStates(activestates);
}

bool PyRobotBase::AddGripperInfo(object oGripperInfo, bool removeduplicate)
{
    RobotBase::GripperInfoPtr pGripperInfo(new RobotBase::GripperInfo());
    rapidjson::Document rGripperInfo;
    toRapidJSONValue(oGripperInfo, rGripperInfo, rGripperInfo.GetAllocator());
    dReal fUnitScale=1;
    int options = 0;
    pGripperInfo->DeserializeJSON(rGripperInfo, fUnitScale, options);
    return _probot->AddGripperInfo(pGripperInfo, removeduplicate);
}

bool PyRobotBase::RemoveGripperInfo(const std::string& name)
{
    return _probot->RemoveGripperInfo(name);
}

object PyRobotBase::GetGripperInfo(const std::string& name)
{
    RobotBase::GripperInfoPtr pGripperInfo = _probot->GetGripperInfo(name);
    if( !pGripperInfo ) {
        return py::none_();
    }

    rapidjson::Document rGripperInfo;
    dReal fUnitScale=1;
    int options=0;
    pGripperInfo->SerializeJSON(rGripperInfo, rGripperInfo.GetAllocator(), fUnitScale, options);
    return toPyObject(rGripperInfo);
}

object PyRobotBase::GetGripperInfos()
{
    py::list pyGripperInfos;
    FOREACHC(itGripperInfo, _probot->GetGripperInfos()) {
        rapidjson::Document rGripperInfo;
        dReal fUnitScale=1;
        int options=0;
        (*itGripperInfo)->SerializeJSON(rGripperInfo, rGripperInfo.GetAllocator(), fUnitScale, options);
        pyGripperInfos.append(toPyObject(rGripperInfo));
    }
    return pyGripperInfos;
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
    _probot->GetActiveDOFVelocityLimits(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFMaxAccel() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFAccelerationLimits(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFMaxJerk() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFJerkLimits(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFHardMaxVel() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFHardVelocityLimits(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFHardMaxAccel() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFHardAccelerationLimits(values);
    return toPyArray(values);
}

object PyRobotBase::GetActiveDOFHardMaxJerk() const
{
    if( _probot->GetActiveDOF() == 0 ) {
        return py::empty_array_astype<dReal>();
    }
    std::vector<dReal> values;
    _probot->GetActiveDOFHardJerkLimits(values);
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
    CHECK_POINTER(pbody); return _probot->Grab(pbody->GetBody(), rapidjson::Value());
}

// since PyKinBody::Grab is overloaded with (pbody, plink) parameters, have to support both...?
bool PyRobotBase::Grab(PyKinBodyPtr pbody, object pylink_or_linkstoignore)
{
    CHECK_POINTER(pbody);
    CHECK_POINTER(pylink_or_linkstoignore);
    KinBody::LinkPtr plink = GetKinBodyLink(pylink_or_linkstoignore);
    if( !!plink ) {
        return _probot->Grab(pbody->GetBody(), plink, rapidjson::Value());
    }
    if( !IS_PYTHONOBJECT_NONE(pylink_or_linkstoignore) && len(pylink_or_linkstoignore) > 0 && IS_PYTHONOBJECT_STRING(object(pylink_or_linkstoignore[0])) ) {
        // pylink_or_linkstoignore is a list of link names to be ignored
        std::set<std::string> setlinkstoignoreString = ExtractSet<std::string>(pylink_or_linkstoignore);
        return _probot->Grab(pbody->GetBody(), setlinkstoignoreString, rapidjson::Value());
    }
    // pylink_or_linkstoignore is a list of link indices to be ignored
    std::set<int> setlinkstoignore = ExtractSet<int>(pylink_or_linkstoignore);
    return _probot->Grab(pbody->GetBody(), setlinkstoignore, rapidjson::Value());
}

bool PyRobotBase::Grab(PyKinBodyPtr pbody, object pylink, object linkstoignore, object grabbedUserData)
{
    CHECK_POINTER(pbody);
    CHECK_POINTER(pylink);
    rapidjson::Document rGrabbedUserData;
    if( !IS_PYTHONOBJECT_NONE(grabbedUserData) ) {
        toRapidJSONValue(grabbedUserData, rGrabbedUserData, rGrabbedUserData.GetAllocator());
    }

    if( !IS_PYTHONOBJECT_NONE(linkstoignore) && len(linkstoignore) > 0 && IS_PYTHONOBJECT_STRING(object(linkstoignore[0])) ) {
        // linkstoignore is a list of link names
        std::set<std::string> setlinkstoignoreString = ExtractSet<std::string>(linkstoignore);
        return _pbody->Grab(pbody->GetBody(), GetKinBodyLink(pylink), setlinkstoignoreString, rGrabbedUserData);
    }
    // linkstoignore is a list of link indices
    std::set<int> setlinkstoignoreInt = ExtractSet<int>(linkstoignore);
    return _pbody->Grab(pbody->GetBody(), GetKinBodyLink(pylink), setlinkstoignoreInt, rGrabbedUserData);
}

bool PyRobotBase::CheckLinkSelfCollision(int ilinkindex, object olinktrans, PyCollisionReportPtr pyreport)
{
    CollisionReport report;
    CollisionReportPtr preport;
    if( !!pyreport ) {
        preport = CollisionReportPtr(&report,utils::null_deleter());
    }

    bool bCollision = _probot->CheckLinkSelfCollision(ilinkindex, ExtractTransform(olinktrans), preport);
    if( !!pyreport ) {
        pyreport->Init(report);
    }
    return bCollision;
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
        return py::make_tuple(r._name, r._sBaseLinkName, r._sEffectorLinkName, r._tLocalTool, r._vChuckingDirection, r._vdirection, r._sIkSolverXMLId, r._vGripperJointNames, r._grippername, r._toolChangerConnectedBodyToolName, r._vRestrictGraspSetNames, r._sIkChainEndLinkName, r._toolChangerLinkName);
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
        if( len(state) > 8 ) {
            r._grippername = state[8];
        }
        else {
            r._grippername = py::none_();
        }
        if( len(state) > 9 ) {
            r._toolChangerConnectedBodyToolName = state[9];
        }
        else {
            r._toolChangerConnectedBodyToolName = py::none_();
        }
        if( len(state) > 10 ) {
            r._vRestrictGraspSetNames = state[10];
        }
        else {
            r._vRestrictGraspSetNames = py::none_();
        }
        if( len(state) > 11 ) {
            r._sIkChainEndLinkName = state[11];
        }
        else {
            r._sIkChainEndLinkName = py::none_();
        }
        if( len(state) > 12 ) {
            r._toolChangerLinkName = state[12];
        }
        else {
            r._toolChangerLinkName = py::none_();
        }
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
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ConvertIkParameterization_overloads, ConvertIkParameterization, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkParameterization_overloads, GetIkParameterization, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckEndEffectorCollision_overloads, CheckEndEffectorCollision, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckEndEffectorSelfCollision_overloads, CheckEndEffectorSelfCollision, 1, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolution_overloads, FindIKSolution, 2, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionFree_overloads, FindIKSolution, 3, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutions_overloads, FindIKSolutions, 2, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionsFree_overloads, FindIKSolutions, 3, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetArmConfigurationSpecification_overloads, GetArmConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkConfigurationSpecification_overloads, GetIkConfigurationSpecification, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateRobotStateSaver_overloads, CreateRobotStateSaver, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFValues_overloads, SetActiveDOFValues, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFVelocities_overloads, SetActiveDOFVelocities, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddManipulator_overloads, AddManipulator, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddAttachedSensor_overloads, AddAttachedSensor, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddConnectedBody_overloads, AddConnectedBody, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddGripperInfo_overloads, AddGripperInfo, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetActiveConfigurationSpecification_overloads, GetActiveConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Restore_overloads, Restore, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Init_overloads, Init, 4,5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(UpdateInfo_overloads, UpdateInfo, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(UpdateAndGetInfo_overloads, UpdateAndGetInfo, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckLinkSelfCollision_overloads, CheckLinkSelfCollision, 2, 3)
// SerializeJSON
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyManipulatorInfo_SerializeJSON_overloads, SerializeJSON, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyAttachedSensorInfo_SerializeJSON_overloads, SerializeJSON, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyConnectedBodyInfo_SerializeJSON_overloads, SerializeJSON, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyRobotBaseInfo_SerializeJSON_overloads, SerializeJSON, 0, 2)
// DeserializeJSON
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyManipulatorInfo_DeserializeJSON_overloads, DeserializeJSON, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyAttachedSensorInfo_DeserializeJSON_overloads, DeserializeJSON, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyConnectedBodyInfo_DeserializeJSON_overloads, DeserializeJSON, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyRobotBaseInfo_DeserializeJSON_overloads, DeserializeJSON, 1, 3)
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
    object robotbaseinfo = class_<PyRobotBase::PyRobotBaseInfo, OPENRAVE_SHARED_PTR<PyRobotBase::PyRobotBaseInfo>, PyKinBody::PyKinBodyInfo>(m, "RobotBaseInfo", DOXY_CLASS(RobotBase::RobotBaseInfo))
                           .def(init<>())
#else
    object robotbaseinfo = class_<PyRobotBase::PyRobotBaseInfo, OPENRAVE_SHARED_PTR<PyRobotBase::PyRobotBaseInfo>, bases<PyKinBody::PyKinBodyInfo> >("RobotBaseInfo", DOXY_CLASS(RobotBase::RobotBaseInfo))
#endif
                           .def_readwrite("_vManipulatorInfos",&PyRobotBase::PyRobotBaseInfo::_vManipulatorInfos)
                           .def_readwrite("_vAttachedSensorInfos",&PyRobotBase::PyRobotBaseInfo::_vAttachedSensorInfos)
                           .def_readwrite("_vConnectedBodyInfos",&PyRobotBase::PyRobotBaseInfo::_vConnectedBodyInfos)
                           .def_readwrite("_vGripperInfos",&PyRobotBase::PyRobotBaseInfo::_vGripperInfos)
                           .def("__str__",&PyRobotBase::PyRobotBaseInfo::__str__)
                           .def("__unicode__",&PyRobotBase::PyRobotBaseInfo::__unicode__)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                           .def("SerializeJSON", &PyRobotBase::PyRobotBaseInfo::SerializeJSON,
                                "unitScale"_a = 1.0,
                                "options"_a = py::none_(),
                                DOXY_FN(RobotBase::RobotBaseInfo, SerializeJSON)
                                )
                           .def("DeserializeJSON", &PyRobotBase::PyRobotBaseInfo::DeserializeJSON,
                                "obj"_a,
                                "unitScale"_a = 1.0,
                                "options"_a = py::none_(),
                                DOXY_FN(RobotBase::RobotBaseInfo, DeserializeJSON)
                                )
#else
                           .def("SerializeJSON", &PyRobotBase::PyRobotBaseInfo::SerializeJSON, PyRobotBaseInfo_SerializeJSON_overloads(PY_ARGS("unitScale", "options") DOXY_FN(RobotBase::RobotBaseInfo, SerializeJSON)))
                           .def("DeserializeJSON", &PyRobotBase::PyRobotBaseInfo::DeserializeJSON, PyRobotBaseInfo_DeserializeJSON_overloads(PY_ARGS("obj", "unitScale", "options") DOXY_FN(RobotBase::RobotBaseInfo, DeserializeJSON)))
#endif
    ;


#ifdef USE_PYBIND11_PYTHON_BINDINGS
    object manipulatorinfo = class_<PyManipulatorInfo, OPENRAVE_SHARED_PTR<PyManipulatorInfo> >(m, "ManipulatorInfo", DOXY_CLASS(RobotBase::ManipulatorInfo))
                             .def(init<>())
#else
    object manipulatorinfo = class_<PyManipulatorInfo, OPENRAVE_SHARED_PTR<PyManipulatorInfo> >("ManipulatorInfo", DOXY_CLASS(RobotBase::ManipulatorInfo))
#endif
                             .def_readwrite("_id",&PyManipulatorInfo::_id)
                             .def_readwrite("_name",&PyManipulatorInfo::_name)
                             .def_readwrite("_sBaseLinkName",&PyManipulatorInfo::_sBaseLinkName)
                             .def_readwrite("_sIkChainEndLinkName",&PyManipulatorInfo::_sIkChainEndLinkName)
                             .def_readwrite("_sEffectorLinkName",&PyManipulatorInfo::_sEffectorLinkName)
                             .def_readwrite("_tLocalTool",&PyManipulatorInfo::_tLocalTool)
                             .def_readwrite("_vChuckingDirection",&PyManipulatorInfo::_vChuckingDirection)
                             .def_readwrite("_vClosingDirection",&PyManipulatorInfo::_vChuckingDirection) // back compat
                             .def_readwrite("_vdirection",&PyManipulatorInfo::_vdirection)
                             .def_readwrite("_sIkSolverXMLId",&PyManipulatorInfo::_sIkSolverXMLId)
                             .def_readwrite("_vGripperJointNames",&PyManipulatorInfo::_vGripperJointNames)
                             .def_readwrite("_grippername",&PyManipulatorInfo::_grippername)
                             .def_readwrite("_toolChangerConnectedBodyToolName",&PyManipulatorInfo::_toolChangerConnectedBodyToolName)
                             .def_readwrite("_toolChangerLinkName",&PyManipulatorInfo::_toolChangerLinkName)
                             .def_readwrite("_vRestrictGraspSetNames",&PyManipulatorInfo::_vRestrictGraspSetNames)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                             .def("SerializeJSON", &PyManipulatorInfo::SerializeJSON,
                                  "unitScale"_a = 1.0,
                                  "options"_a = py::none_(),
                                  DOXY_FN(RobotBase::ManipulatorInfo, SerializeJSON)
                                  )
                             .def("DeserializeJSON", &PyManipulatorInfo::DeserializeJSON,
                                  "obj"_a,
                                  "unitScale"_a = 1.0,
                                  "options"_a = py::none_(),
                                  DOXY_FN(RobotBase::ManipulatorInfo, DeserializeJSON)
                                  )
#else
                             .def("SerializeJSON", &PyManipulatorInfo::SerializeJSON, PyManipulatorInfo_SerializeJSON_overloads(PY_ARGS("options") DOXY_FN(RobotBase::ManipulatorInfo, SerializeJSON)))
                             .def("DeserializeJSON", &PyManipulatorInfo::DeserializeJSON, PyManipulatorInfo_DeserializeJSON_overloads(PY_ARGS("obj", "unitScale", "options") DOXY_FN(RobotBase::ManipulatorInfo, DeserializeJSON)))
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
                             .def("__copy__", [](const PyManipulatorInfo& self){
            return self;
        })
                             .def("__deepcopy__",
                                  [](const PyManipulatorInfo &pyinfo, const py::dict& memo) {
            auto state = ManipulatorInfo_pickle_suite::getstate(pyinfo);
            PyManipulatorInfo pyinfo_new;
            ManipulatorInfo_pickle_suite::setstate(pyinfo_new, state);
            return pyinfo_new;
        }
                                  )
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
                                .def_readwrite("_id", &PyAttachedSensorInfo::_id)
                                .def_readwrite("_name", &PyAttachedSensorInfo::_name)
                                .def_readwrite("_linkname", &PyAttachedSensorInfo::_linkname)
                                .def_readwrite("_trelative", &PyAttachedSensorInfo::_trelative)
                                .def_readwrite("_sensorname", &PyAttachedSensorInfo::_sensorname)
                                .def_readwrite("_referenceAttachedSensorName",&PyAttachedSensorInfo::_referenceAttachedSensorName)
                                .def_readwrite("_sensorMaker",&PyAttachedSensorInfo::_sensorMaker)
                                .def_readwrite("_sensorModel",&PyAttachedSensorInfo::_sensorModel)
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
                                     "options"_a = py::none_(),
                                     DOXY_FN(RobotBase::AttachedSensorInfo, DeserializeJSON)
                                     )
#else
                                .def("SerializeJSON", &PyAttachedSensorInfo::SerializeJSON, PyAttachedSensorInfo_SerializeJSON_overloads(PY_ARGS("unitScale", "options") DOXY_FN(RobotBase::AttachedSensorInfo, SerializeJSON)))
                                .def("DeserializeJSON", &PyAttachedSensorInfo::DeserializeJSON, PyAttachedSensorInfo_DeserializeJSON_overloads(PY_ARGS("obj", "unitScale", "options") DOXY_FN(RobotBase::AttachedSensorInfo, DeserializeJSON)))
#endif

    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    object connectedbodyinfo = class_<PyConnectedBodyInfo, OPENRAVE_SHARED_PTR<PyConnectedBodyInfo> >(m, "ConnectedBodyInfo", DOXY_CLASS(RobotBase::ConnectedBodyInfo))
                               .def(init<>())
#else
    object connectedbodyinfo = class_<PyConnectedBodyInfo, OPENRAVE_SHARED_PTR<PyConnectedBodyInfo> >("ConnectedBodyInfo", DOXY_CLASS(RobotBase::ConnectedBodyInfo))
#endif
                               .def_readwrite("_id", &PyConnectedBodyInfo::_id)
                               .def_readwrite("_name", &PyConnectedBodyInfo::_name)
                               .def_readwrite("_linkname", &PyConnectedBodyInfo::_linkname)
                               .def_readwrite("_trelative", &PyConnectedBodyInfo::_trelative)
                               .def_readwrite("_uri", &PyConnectedBodyInfo::_uri)
                               .def_readwrite("_linkInfos", &PyConnectedBodyInfo::_linkInfos)
                               .def_readwrite("_jointInfos", &PyConnectedBodyInfo::_jointInfos)
                               .def_readwrite("_manipulatorInfos", &PyConnectedBodyInfo::_manipulatorInfos)
                               .def_readwrite("_attachedSensorInfos", &PyConnectedBodyInfo::_attachedSensorInfos)
                               .def_readwrite("_gripperInfos", &PyConnectedBodyInfo::_gripperInfos)
                               .def_readwrite("_bIsActive", &PyConnectedBodyInfo::_bIsActive)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                               .def("SerializeJSON", &PyConnectedBodyInfo::SerializeJSON,
                                    "unitScale"_a = 1.0,
                                    "options"_a = py::none_(),
                                    DOXY_FN(RobotBase::ConnectedBodyInfo, SerializeJSON)
                                    )
                               .def("DeserializeJSON", &PyConnectedBodyInfo::DeserializeJSON,
                                    "obj"_a,
                                    "unitScale"_a = 1.0,
                                    "options"_a = py::none_(),
                                    DOXY_FN(RobotBase::ConnectedBodyInfo, DeserializeJSON)
                                    )
#else
                               .def("SerializeJSON", &PyConnectedBodyInfo::SerializeJSON, PyConnectedBodyInfo_SerializeJSON_overloads(PY_ARGS("unitScale", "options") DOXY_FN(RobotBase::ConnectedBodyInfo, SerializeJSON)))
                               .def("DeserializeJSON", &PyConnectedBodyInfo::DeserializeJSON, PyConnectedBodyInfo_DeserializeJSON_overloads(PY_ARGS("obj", "unitScale", "options") DOXY_FN(RobotBase::ConnectedBodyInfo, DeserializeJSON)))
#endif

    ;

    {
        void (PyRobotBase::*psetactivedofs1)(const object&) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs2)(const object&, int) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs3)(const object&, int, object) = &PyRobotBase::SetActiveDOFs;

        bool (PyRobotBase::*pgrab1)(PyKinBodyPtr) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab3)(PyKinBodyPtr, object) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab5)(PyKinBodyPtr, object, object, object) = &PyRobotBase::Grab;

        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator2)(const std::string&) = &PyRobotBase::SetActiveManipulator;
        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator3)(PyRobotBase::PyManipulatorPtr) = &PyRobotBase::SetActiveManipulator;

        object (PyRobotBase::*GetManipulators1)() = &PyRobotBase::GetManipulators;
        object (PyRobotBase::*GetManipulators2)(const string &) = &PyRobotBase::GetManipulators;
        bool (PyRobotBase::*setcontroller1)(PyControllerBasePtr,const string &) = &PyRobotBase::SetController;
        bool (PyRobotBase::*setcontroller2)(PyControllerBasePtr,object,int) = &PyRobotBase::SetController;
        bool (PyRobotBase::*setcontroller3)(PyControllerBasePtr) = &PyRobotBase::SetController;
        bool (PyRobotBase::*initrobot)(object, object, object, object, const std::string&) = &PyRobotBase::Init;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        scope_ robot = class_<PyRobotBase, OPENRAVE_SHARED_PTR<PyRobotBase>, PyKinBody>(m, "Robot", py::dynamic_attr(), DOXY_CLASS(RobotBase))
#else
        scope_ robot = class_<PyRobotBase, OPENRAVE_SHARED_PTR<PyRobotBase>, bases<PyKinBody, PyInterfaceBase> >("Robot", DOXY_CLASS(RobotBase), no_init)
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                       .def("Init", initrobot,
                            "linkinfos"_a,
                            "jointinfos"_a,
                            "manipinfos"_a,
                            "attachedsensorinfos"_a,
                            "uri"_a = "",
                            DOXY_FN(RobotBase, Init)
                            )
#else
                       .def("Init", initrobot, Init_overloads(PY_ARGS("linkinfos", "jointinfos", "manipinfos", "attachedsensorinfos", "uri") DOXY_FN(RobotBase, Init)))
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
                       .def("InitFromRobotInfo", &PyRobotBase::InitFromRobotInfo,
                            "info"_a,
                            DOXY_FN(RobotBase, InitFromRobotInfo))
#else
                       .def("InitFromRobotInfo",&PyRobotBase::InitFromRobotInfo, DOXY_FN(RobotBase, InitFromRobotInfo))
#endif
                       .def("ExtractInfo", &PyRobotBase::ExtractInfo, DOXY_FN(RobotBase, ExtractInfo))

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
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                       .def("AddGripperInfo",&PyRobotBase::AddGripperInfo,
                            "gripperInfo"_a,
                            "removeduplicate"_a = false,
                            DOXY_FN(RobotBase, AddGripperInfo)
                            )
#else
                       .def("AddGripperInfo",&PyRobotBase::AddGripperInfo, AddGripperInfo_overloads(PY_ARGS("gripperInfo", "removeduplicate") DOXY_FN(RobotBase,AddGripperInfo)))
#endif
                       .def("RemoveGripperInfo",&PyRobotBase::RemoveGripperInfo, PY_ARGS("name") DOXY_FN(RobotBase,RemoveGripperInfo))
                       .def("GetGripperInfo",&PyRobotBase::GetGripperInfo, PY_ARGS("name") DOXY_FN(RobotBase,GetGripperInfo))
                       .def("GetGripperInfos",&PyRobotBase::GetGripperInfos, DOXY_FN(RobotBase,GetGripperInfos))
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
                       .def("Grab",pgrab3, PY_ARGS("body","grablink") DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr"))
                       .def("Grab",pgrab5, PY_ARGS("body","grablink","linkstoignore","grabbedUserData") DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr; Linkptr; rapidjson::Document"))
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

        object (PyRobotBase::PyManipulator::*pmanipik)(object, int, bool, bool, PyIkFailureAccumulatorBasePtr) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipikf)(object, object, int, bool, bool, PyIkFailureAccumulatorBasePtr) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipiks)(object, int, bool, bool, PyIkFailureAccumulatorBasePtr) const = &PyRobotBase::PyManipulator::FindIKSolutions;
        object (PyRobotBase::PyManipulator::*pmanipiksf)(object, object, int, bool, bool, PyIkFailureAccumulatorBasePtr) const = &PyRobotBase::PyManipulator::FindIKSolutions;

        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision0)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision1)(object,PyCollisionReportPtr,int) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorSelfCollision0)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckEndEffectorSelfCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorSelfCollision1)(object,PyCollisionReportPtr,int,bool) const = &PyRobotBase::PyManipulator::CheckEndEffectorSelfCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision1)() const = &PyRobotBase::PyManipulator::CheckIndependentCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision2)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckIndependentCollision;

        std::string ConvertIkParameterization_doc = std::string(DOXY_FN(RobotBase::Manipulator,ConvertIkParameterization "const IkParameterization; const IkParameterization; bool")) + std::string(DOXY_FN(RobotBase::Manipulator,ConvertIkParameterization "const IkParameterization; IkParameterizationType; bool"));
        std::string GetIkParameterization_doc = std::string(DOXY_FN(RobotBase::Manipulator,GetIkParameterization "const IkParameterization; bool")) + std::string(DOXY_FN(RobotBase::Manipulator,GetIkParameterization "IkParameterizationType; bool"));
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<PyRobotBase::PyManipulator, OPENRAVE_SHARED_PTR<PyRobotBase::PyManipulator> >(m, "Manipulator", DOXY_CLASS(RobotBase::Manipulator))
#else
        class_<PyRobotBase::PyManipulator, OPENRAVE_SHARED_PTR<PyRobotBase::PyManipulator> >("Manipulator", DOXY_CLASS(RobotBase::Manipulator), no_init)
#endif
        .def("GetTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetTransformPose", &PyRobotBase::PyManipulator::GetTransformPose, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetVelocity", &PyRobotBase::PyManipulator::GetVelocity, DOXY_FN(RobotBase::Manipulator,GetVelocity))
        .def("GetId",&PyRobotBase::PyManipulator::GetId, DOXY_FN(RobotBase::Manipulator,GetId))
        .def("GetName",&PyRobotBase::PyManipulator::GetName, DOXY_FN(RobotBase::Manipulator,GetName))
        .def("SetName",&PyRobotBase::PyManipulator::SetName, PY_ARGS("name") DOXY_FN(RobotBase::Manipulator,SetName))
        .def("GetGripperName",&PyRobotBase::PyManipulator::GetGripperName, DOXY_FN(RobotBase::Manipulator,GetGripperName))
        .def("GetToolChangerConnectedBodyToolName",&PyRobotBase::PyManipulator::GetToolChangerConnectedBodyToolName, DOXY_FN(RobotBase::Manipulator,GetToolChangerConnectedBodyToolName))
        .def("GetToolChangerLinkName",&PyRobotBase::PyManipulator::GetToolChangerLinkName, DOXY_FN(RobotBase::Manipulator,GetToolChangerLinkName))
        .def("GetRestrictGraspSetNames",&PyRobotBase::PyManipulator::GetRestrictGraspSetNames, DOXY_FN(RobotBase::Manipulator,GetRestrictGraspSetNames))
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
             "ikFailureAccumulator"_a = nullptr,
             DOXY_FN(RobotBase::Manipulator, FindIKSolution "const IkParameterization; std::vector; int; IkFailureAccumulatorBase")
             )
#else
        .def("FindIKSolution",pmanipik,FindIKSolution_overloads(PY_ARGS("param","filteroptions","ikreturn","releasegil","IkFailureAccumulatorBase") DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; std::vector; int; IkFailureAccumulatorBase")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("FindIKSolution", pmanipikf,
             "param"_a,
             "freevalues"_a,
             "filteroptions"_a,
             "ikreturn"_a = false,
             "releasegil"_a = false,
             "ikFailureAccumulator"_a = nullptr,
             DOXY_FN(RobotBase::Manipulator, FindIKSolution "const IkParameterization; const std::vector; std::vector; int; IkFailureAccumulatorBase")
             )
#else
        .def("FindIKSolution",pmanipikf,FindIKSolutionFree_overloads(PY_ARGS("param","freevalues","filteroptions","ikreturn","releasegil","IkFailureAccumulatorBase") DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; const std::vector; std::vector; int; IkFailureAccumulatorBase")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("FindIKSolutions", pmanipiks,
             "param"_a,
             "filteroptions"_a,
             "ikreturn"_a = false,
             "releasegil"_a = false,
             "ikFailureAccumulator"_a = nullptr,
             DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; std::vector; int; IkFailureAccumulatorBase")
             )
#else
        .def("FindIKSolutions",pmanipiks,FindIKSolutions_overloads(PY_ARGS("param","filteroptions","ikreturn","releasegil","ikFailureAccumulator") DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; std::vector; int; IkFailureAccumulatorBase")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("FindIKSolutions", pmanipiksf,
             "param"_a,
             "freevalues"_a,
             "filteroptions"_a,
             "ikreturn"_a = false,
             "releasegil"_a = false,
             "ikFailureAccumulator"_a = nullptr,
             DOXY_FN(RobotBase::Manipulator, FindIKSolutions "const IkParameterization; const std::vector; std::vector; int; IkFailureAccumulatorBase")
             )
#else
        .def("FindIKSolutions",pmanipiksf,FindIKSolutionsFree_overloads(PY_ARGS("param","freevalues","filteroptions","ikreturn","releasegil","ikFailureAccumulator") DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; const std::vector; std::vector; int; IkFailureAccumulatorBase")))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .def("ConvertIkParameterization", &PyRobotBase::PyManipulator::ConvertIkParameterization,
             "sourceikp"_a,
             "iktype"_a,
             "inworld"_a = true,
             ConvertIkParameterization_doc.c_str()
             )
#else
        .def("ConvertIkParameterization",&PyRobotBase::PyManipulator::ConvertIkParameterization, ConvertIkParameterization_overloads(PY_ARGS("sourceikp","iktype","inworld") ConvertIkParameterization_doc.c_str()))
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
        .def("GetIkChainEndLink",&PyRobotBase::PyManipulator::GetIkChainEndLink, DOXY_FN(RobotBase::Manipulator,GetIkChainEndLink))
        .def("GetEndEffector",&PyRobotBase::PyManipulator::GetEndEffector, DOXY_FN(RobotBase::Manipulator,GetEndEffector))
        .def("ReleaseAllGrabbed",&PyRobotBase::PyManipulator::ReleaseAllGrabbed, DOXY_FN(RobotBase::Manipulator,ReleaseAllGrabbed))
        .def("GetGraspTransform",&PyRobotBase::PyManipulator::GetGraspTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransform",&PyRobotBase::PyManipulator::GetLocalToolTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransformPose",&PyRobotBase::PyManipulator::GetLocalToolTransformPose, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransformPose))
        .def("SetLocalToolTransform",&PyRobotBase::PyManipulator::SetLocalToolTransform, PY_ARGS("transform") DOXY_FN(RobotBase::Manipulator,SetLocalToolTransform))
        .def("SetLocalToolDirection",&PyRobotBase::PyManipulator::SetLocalToolDirection, PY_ARGS("direction") DOXY_FN(RobotBase::Manipulator,SetLocalToolDirection))
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
        .def("GetId",&PyRobotBase::PyAttachedSensor::GetId, DOXY_FN(RobotBase::AttachedSensor,GetId))
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
        .def("GetId",&PyRobotBase::PyConnectedBody::GetId, DOXY_FN(RobotBase::ConnectedBody,GetId))
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
        .def("GetResolvedAttachedSensors",&PyRobotBase::PyConnectedBody::GetResolvedAttachedSensors, DOXY_FN(RobotBase::ConnectedBody,GetResolvedAttachedSensors))
        .def("GetResolvedGripperInfos",&PyRobotBase::PyConnectedBody::GetResolvedGripperInfos, DOXY_FN(RobotBase::ConnectedBody,GetResolvedGripperInfos))
        .def("CanProvideManipulator", &PyRobotBase::PyConnectedBody::CanProvideManipulator, DOXY_FN(RobotBase::ConnectedBody,CanProvideManipulator))
        .def("GetInfoHash", &PyRobotBase::PyConnectedBody::GetInfoHash, DOXY_FN(RobotBase::ConnectedBody,GetInfoHash))
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
