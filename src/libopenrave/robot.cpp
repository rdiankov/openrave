// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov (rosen.diankov@gmail.com)
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
#include "libopenrave.h"

#define CHECK_INTERNAL_COMPUTATION OPENRAVE_ASSERT_FORMAT(_nHierarchyComputed == 2, "robot %s internal structures need to be computed, current value is %d. Are you sure Environment::AddRobot/AddKinBody was called?", GetName()%_nHierarchyComputed, ORE_NotInitialized);

namespace OpenRAVE {

RobotBase::GripperInfo& RobotBase::GripperInfo::operator=(const RobotBase::GripperInfo& other)
{
    _id = other._id;
    name = other.name;
    grippertype = other.grippertype;
    gripperJointNames = other.gripperJointNames;
    rapidjson::Document docGripperInfo;
    if (other._docGripperInfo.IsObject()) {
        docGripperInfo.CopyFrom(other._docGripperInfo, docGripperInfo.GetAllocator());
    }
    _docGripperInfo.Swap(docGripperInfo);
    return *this;
}

void RobotBase::GripperInfo::Reset()
{
    _id.clear();
    name.clear();
    grippertype.clear();
    gripperJointNames.clear();
    _docGripperInfo = rapidjson::Document();
}

void RobotBase::GripperInfo::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    value.SetObject();
    if( _docGripperInfo.IsObject() ) {
        value.CopyFrom(_docGripperInfo, allocator, true); // need to copy the const strings
    }
    orjson::SetJsonValueByKey(value, "name", name, allocator);
    orjson::SetJsonValueByKey(value, "id", _id, allocator);
    orjson::SetJsonValueByKey(value, "grippertype", grippertype, allocator);
    orjson::SetJsonValueByKey(value, "gripperJointNames", gripperJointNames, allocator);
}

void RobotBase::GripperInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    orjson::LoadJsonValueByKey(value, "name", name);
    orjson::LoadJsonValueByKey(value, "id", _id);
    if( name.size() == 0 && _id.size() > 0 ) {
        name = _id;
        RAVELOG_WARN_FORMAT("gripperInfo %s got old tag 'id', when it should be 'name'", name);
    }
    if( _id.empty() ) {
        _id = name;
    }
    orjson::LoadJsonValueByKey(value, "grippertype", grippertype);
    orjson::LoadJsonValueByKey(value, "gripperJointNames", gripperJointNames);

    rapidjson::Document docGripperInfo;
    docGripperInfo.SetObject();
    if (_docGripperInfo.IsObject()) {
        // value may used for partial update, so retain original key values
        docGripperInfo.CopyFrom(_docGripperInfo, docGripperInfo.GetAllocator(), true); // need to copy the const strings
    }
    for (rapidjson::Value::ConstMemberIterator it = value.MemberBegin(); it != value.MemberEnd(); ++it) {
        const std::string& name = it->name.GetString();
        if (name == "id" || name == "name" || name == "grippertype" || name == "gripperJointNames") {
            continue;
        }
        orjson::SetJsonValueByKey(docGripperInfo, name, it->value);
    }
    _docGripperInfo.Swap(docGripperInfo);
}

void RobotBase::AttachedSensorInfo::Reset()
{
    _id.clear();
    _name.clear();
    _linkname.clear();
    _trelative = Transform();
    _sensorname.clear();
    _docSensorGeometry = rapidjson::Document();
}

void RobotBase::AttachedSensorInfo::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    value.SetObject();
    orjson::SetJsonValueByKey(value, "id", _id, allocator);
    orjson::SetJsonValueByKey(value, "name", _name, allocator);
    orjson::SetJsonValueByKey(value, "linkName", _linkname, allocator);
    orjson::SetJsonValueByKey(value, "transform", _trelative, allocator);
    orjson::SetJsonValueByKey(value, "type", _sensorname, allocator);

    if (_docSensorGeometry.IsObject() && _docSensorGeometry.MemberCount() > 0) {
        rapidjson::Value sensorGeometry;
        sensorGeometry.CopyFrom(_docSensorGeometry, allocator, true);
        value.AddMember("sensorGeometry", sensorGeometry, allocator);
    }
}

void RobotBase::AttachedSensorInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    orjson::LoadJsonValueByKey(value, "name", _name);
    orjson::LoadJsonValueByKey(value, "id", _id);
    if( _id.empty() ) {
        _id = _name;
    }
    orjson::LoadJsonValueByKey(value, "linkName", _linkname);
    orjson::LoadJsonValueByKey(value, "transform", _trelative);
    orjson::LoadJsonValueByKey(value, "type", _sensorname);

    if (value.HasMember("sensorGeometry")) {
        if (!_docSensorGeometry.IsObject()) {
            _docSensorGeometry.SetObject();
        }
        orjson::UpdateJson(_docSensorGeometry, value["sensorGeometry"]);
    }
}

RobotBase::AttachedSensor::AttachedSensor(RobotBasePtr probot) : _probot(probot)
{
}

RobotBase::AttachedSensor::AttachedSensor(RobotBasePtr probot, const AttachedSensor& sensor,int cloningoptions)
{
    *this = sensor;
    _probot = probot;
    _psensor.reset();
    pdata.reset();
    pattachedlink.reset();
    if( (cloningoptions&Clone_Sensors) && !!sensor._psensor ) {
        _psensor = RaveCreateSensor(probot->GetEnv(), sensor._psensor->GetXMLId());
        if( !!_psensor ) {
            _psensor->SetName(str(boost::format("%s:%s")%probot->GetName()%_info._name)); // need a unique targettable name
            _psensor->Clone(sensor._psensor,cloningoptions);
            if( !!_psensor ) {
                pdata = _psensor->CreateSensorData();
            }
        }
    }
    int index = LinkPtr(sensor.pattachedlink)->GetIndex();
    if((index >= 0)&&(index < (int)probot->GetLinks().size())) {
        pattachedlink = probot->GetLinks().at(index);
    }
}

RobotBase::AttachedSensor::AttachedSensor(RobotBasePtr probot, const RobotBase::AttachedSensorInfo& info)
{
    _info = info;
    _probot = probot;
    pattachedlink = probot->GetLink(_info._linkname);
    if( !!probot ) {
        _psensor = RaveCreateSensor(probot->GetEnv(), _info._sensorname);
        if( !!_psensor ) {
            _psensor->SetName(str(boost::format("%s:%s")%probot->GetName()%_info._name)); // need a unique targettable name
            if(_info._docSensorGeometry.IsObject()) {
                BaseJSONReaderPtr pReader = RaveCallJSONReader(PT_Sensor, _info._sensorname, InterfaceBasePtr(), AttributesList());
                if (!!pReader) {
                    pReader->DeserializeJSON(_info._docSensorGeometry);
                    ReadablePtr pReadable = pReader->GetReadable();
                    if (!!pReadable) {
                        SensorBase::SensorGeometryPtr sensorGeometry = OPENRAVE_DYNAMIC_POINTER_CAST<SensorBase::SensorGeometry>(pReadable);
                        _psensor->SetSensorGeometry(sensorGeometry);
                    }
                } else {
                    RAVELOG_WARN_FORMAT("failed to get json reader for sensor type \"%s\"", _info._sensorname);
                }
            }
            pdata = _psensor->CreateSensorData();
        }
    }
}

RobotBase::AttachedSensor::~AttachedSensor()
{
}

SensorBase::SensorDataPtr RobotBase::AttachedSensor::GetData() const
{
    if( !!_psensor && _psensor->GetSensorData(pdata) ) {
        return pdata;
    }
    return SensorBase::SensorDataPtr();
}

void RobotBase::AttachedSensor::SetRelativeTransform(const Transform& t)
{
    _info._trelative = t;
    GetRobot()->_PostprocessChangedParameters(Prop_SensorPlacement);
}

void RobotBase::AttachedSensor::UpdateInfo(SensorBase::SensorType type)
{
    rapidjson::Document docSensorGeometry;
    if( !!_psensor ) {
        _info._sensorname = _psensor->GetXMLId();
        SensorBase::SensorGeometryPtr sensorGeometry = boost::const_pointer_cast<SensorBase::SensorGeometry>(_psensor->GetSensorGeometry(type));
        if (!!sensorGeometry) {
            sensorGeometry->SerializeJSON(docSensorGeometry, docSensorGeometry.GetAllocator());
        }
    }
    _info._docSensorGeometry.Swap(docSensorGeometry);

    LinkPtr prealattachedlink = pattachedlink.lock();
    if( !!prealattachedlink ) {
        _info._linkname = prealattachedlink->GetName();
    }
}

void RobotBase::AttachedSensor::ExtractInfo(AttachedSensorInfo& info) const
{
    info._id = _info._id;
    info._name = _info._name;
    info._trelative = _info._trelative;
    info._sensorname.clear();
    info._linkname.clear();

    rapidjson::Document docSensorGeometry;
    if( !!_psensor ) {
        info._sensorname = _psensor->GetXMLId();
        SensorBase::SensorGeometryPtr sensorGeometry = boost::const_pointer_cast<SensorBase::SensorGeometry>(_psensor->GetSensorGeometry(SensorBase::ST_Invalid));
        if (!!sensorGeometry) {
            sensorGeometry->SerializeJSON(docSensorGeometry, docSensorGeometry.GetAllocator());
        }
    }
    info._docSensorGeometry.Swap(docSensorGeometry);

    LinkPtr prealattachedlink = pattachedlink.lock();
    if( !!prealattachedlink ) {
        info._linkname = prealattachedlink->GetName();
    }
}

UpdateFromInfoResult RobotBase::AttachedSensor::UpdateFromInfo(const RobotBase::AttachedSensorInfo& info)
{
    BOOST_ASSERT(info._id == _info._id);

    // _name
    if (GetName() != info._name) {
        return UFIR_RequireReinitialize; // no SetName function defeined now. Maybe add later.
    }

    // _linkname
    KinBody::LinkPtr attachingLink = GetAttachingLink();
    if (!!attachingLink) {
        if (attachingLink->GetName() != info._linkname) {
            return UFIR_RequireReinitialize;
        }
    }
    else if (!info._linkname.empty()) {
        return UFIR_RequireReinitialize;
    }

    // sensor name
    if (_info._sensorname != info._sensorname) {
        return UFIR_RequireReinitialize;
    }

    // sensor geometry
    if (_info._docSensorGeometry != info._docSensorGeometry) {
        return UFIR_RequireReinitialize;
    }

    // _trelative
    if (GetRelativeTransform() != info._trelative) {
        SetRelativeTransform(info._trelative);
    }

    return UFIR_Success;
}

void RobotBase::AttachedSensor::serialize(std::ostream& o, int options) const
{
    o << (pattachedlink.expired() ? -1 : LinkPtr(pattachedlink)->GetIndex()) << " ";
    SerializeRound(o,_info._trelative);
    o << (!pdata ? -1 : pdata->GetType()) << " ";
    // it is also important to serialize some of the geom parameters for the sensor (in case models are cached to it)
    if( !!_psensor ) {
        SensorBase::SensorGeometryConstPtr prawgeom = _psensor->GetSensorGeometry();
        if( !!prawgeom ) {
            switch(prawgeom->GetType()) {
            case SensorBase::ST_Laser: {
                SensorBase::LaserGeomDataConstPtr pgeom = boost::static_pointer_cast<SensorBase::LaserGeomData const>(prawgeom);
                o << pgeom->min_angle[0] << " " << pgeom->max_angle[0] << " " << pgeom->resolution[0] << " " << pgeom->max_range << " ";
                break;
            }
            case SensorBase::ST_Camera: {
                SensorBase::CameraGeomDataConstPtr pgeom = boost::static_pointer_cast<SensorBase::CameraGeomData const>(prawgeom);
                o << pgeom->KK.fx << " " << pgeom->KK.fy << " " << pgeom->KK.cx << " " << pgeom->KK.cy << " " << pgeom->width << " " << pgeom->height << " ";
                break;
            }
            default:
                // don't support yet
                break;
            }
        }
    }
}

const std::string& RobotBase::AttachedSensor::GetStructureHash() const
{
    if( __hashstructure.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_RobotSensors);
        __hashstructure = utils::GetMD5HashString(ss.str());
    }
    return __hashstructure;
}

RobotBase::RobotStateSaver::RobotStateSaver(RobotBasePtr probot, int options) : KinBodyStateSaver(probot, options), _probot(probot)
{
    if( _options & Save_ActiveDOF ) {
        vactivedofs = _probot->GetActiveDOFIndices();
        affinedofs = _probot->GetAffineDOF();
        rotationaxis = _probot->GetAffineRotationAxis();
    }
    if( _options & Save_ActiveManipulator ) {
        _pManipActive = _probot->GetActiveManipulator();
    }
    if( _options & Save_ActiveManipulatorToolTransform ) {
        _pManipActive = _probot->GetActiveManipulator();
        if( !!_pManipActive ) {
            _tActiveManipLocalTool = _pManipActive->GetLocalToolTransform();
            _vActiveManipLocalDirection = _pManipActive->GetLocalToolDirection();
            _pActiveManipIkSolver = _pManipActive->GetIkSolver();
        }
    }
    if( _options & Save_ManipulatorsToolTransform ) {
        std::vector<RobotBase::ManipulatorPtr> vmanips = probot->GetManipulators();
        _vtManipsLocalTool.resize(vmanips.size());
        _vvManipsLocalDirection.resize(vmanips.size());
        _vpManipsIkSolver.resize(vmanips.size());
        _vManipsName.resize(vmanips.size());
        for(int imanip = 0; imanip < (int)vmanips.size(); ++imanip) {
            RobotBase::ManipulatorPtr pmanip = vmanips[imanip];
            if( !!pmanip ) {
                _vtManipsLocalTool[imanip] = pmanip->GetLocalToolTransform();
                _vvManipsLocalDirection[imanip] = pmanip->GetLocalToolDirection();
                _vpManipsIkSolver[imanip] = pmanip->GetIkSolver();
                _vManipsName[imanip] = pmanip->GetName();
            }
        }
    }

    if( _options & Save_ConnectedBodies ) {
        _probot->GetConnectedBodyActiveStates(_vConnectedBodyActiveStates);
    }
}

RobotBase::RobotStateSaver::~RobotStateSaver()
{
    if( _bRestoreOnDestructor && !!_probot && _probot->GetEnvironmentId() != 0 ) {
        _RestoreRobot(_probot);
    }
}

void RobotBase::RobotStateSaver::Restore(boost::shared_ptr<RobotBase> robot)
{
    _RestoreRobot(!robot ? _probot : robot);
    KinBodyStateSaver::Restore(!robot ? KinBodyPtr(_probot) : KinBodyPtr(robot));
}

void RobotBase::RobotStateSaver::Release()
{
    _probot.reset();
    KinBodyStateSaver::Release();
}

///\brief removes the robot from the environment temporarily while in scope
class EnvironmentRobotRemover
{
public:

    EnvironmentRobotRemover(RobotBasePtr pRobot) : _bRemoved(false), _pRobot(pRobot), _pEnv(pRobot->GetEnv()) {
        _pEnv->Remove(_pRobot);
        _bRemoved = true;
    }

    ~EnvironmentRobotRemover() {
        if (_bRemoved) {
            _pEnv->Add(_pRobot, false);
            _bRemoved = false;
        }
    }

private:

    bool _bRemoved;
    RobotBasePtr _pRobot;
    EnvironmentBasePtr _pEnv;
};

void RobotBase::RobotStateSaver::_RestoreRobot(boost::shared_ptr<RobotBase> probot)
{
    if( !probot ) {
        return;
    }
    if( probot->GetEnvironmentId() == 0 ) {
        RAVELOG_WARN(str(boost::format("robot %s not added to environment, skipping restore")%probot->GetName()));
        return;
    }

    if( _options & Save_ConnectedBodies ) {
        if( _vConnectedBodyActiveStates.size() == probot->_vecConnectedBodies.size() ) {
            bool bchanged = false;
            for(size_t iconnectedbody = 0; iconnectedbody < probot->_vecConnectedBodies.size(); ++iconnectedbody) {
                if( probot->_vecConnectedBodies[iconnectedbody]->IsActive() != _vConnectedBodyActiveStates[iconnectedbody] ) {
                    bchanged = true;
                    break;
                }
            }

            if( bchanged ) {
                EnvironmentRobotRemover robotremover(probot);
                // need to restore active connected bodies
                // but first check whether anything changed
                probot->SetConnectedBodyActiveStates(_vConnectedBodyActiveStates);
            }
        }
        else {
            RAVELOG_WARN_FORMAT("env=%d, connected body states changed, so cannot save. saved num is %s, new robot num is %s", probot->GetEnv()->GetId()%_vConnectedBodyActiveStates.size()%probot->_vecConnectedBodies.size());
        }
    }

    if( _options & Save_ActiveDOF ) {
        probot->SetActiveDOFs(vactivedofs, affinedofs, rotationaxis);
    }
    if( _options & Save_ActiveManipulator ) {
        if( probot == _probot ) {
            probot->SetActiveManipulator(_pManipActive);
        }
        else {
            if( !_pManipActive ) {
                probot->SetActiveManipulator(ManipulatorPtr());
            }
            else {
                probot->SetActiveManipulator(_pManipActive->GetName());
            }
        }
    }
    if( _options & Save_ActiveManipulatorToolTransform ) {
        if( !!_pManipActive ) {
            if( probot == _probot ) {
                RobotBase::ManipulatorPtr pmanip = probot->GetManipulator(_pManipActive->GetName()); // manipulator pointers might have changed, it is always safer to re-request the current manip
                if( !!pmanip ) {
                    pmanip->SetLocalToolTransform(_tActiveManipLocalTool);
                    pmanip->SetLocalToolDirection(_vActiveManipLocalDirection);
                    pmanip->SetIkSolver(_pActiveManipIkSolver);
                }
                else {
                    RAVELOG_VERBOSE_FORMAT("failed to restore active manipulator %s coordinate system", _pManipActive->GetName());
                }
            }
            else {
                RobotBase::ManipulatorPtr pmanip = probot->GetManipulator(_pManipActive->GetName());
                if( !!pmanip ) {
                    pmanip->SetLocalToolTransform(_tActiveManipLocalTool);
                    pmanip->SetLocalToolDirection(_vActiveManipLocalDirection);
                    if( !!_pActiveManipIkSolver ) {
                        IkSolverBasePtr pnewsolver = RaveCreateIkSolver(probot->GetEnv(), _pActiveManipIkSolver->GetXMLId());
                        pnewsolver->Clone(_pActiveManipIkSolver, 0);
                        pmanip->SetIkSolver(pnewsolver);
                    }
                    else {
                        pmanip->SetIkSolver(IkSolverBasePtr());
                    }
                }
            }
        }
    }
    if( _options & Save_ManipulatorsToolTransform ) {
        if( probot == _probot ) {
            std::vector<RobotBase::ManipulatorPtr> vmanips = probot->GetManipulators();
            if(vmanips.size() == _vtManipsLocalTool.size()) {
                for(int imanip = 0; imanip < (int)vmanips.size(); ++imanip) {
                    RobotBase::ManipulatorPtr pmanip = vmanips[imanip];
                    const std::string manipName = pmanip->GetName();
                    const vector<string>::const_iterator it = find(_vManipsName.begin(), _vManipsName.end(), manipName);
                    if (it == _vManipsName.end()) {
                        RAVELOG_WARN_FORMAT("manip %s is not found in saved state. Maybe newly added?", manipName);
                        continue;
                    }
                    int indexAtSaveTime = distance(_vManipsName.cbegin(), it);
                    if (indexAtSaveTime != imanip) {
                        RAVELOG_DEBUG_FORMAT("manip %s was previously at index %d, but changed to index %d.", manipName%imanip%indexAtSaveTime);
                    }
                    if( !!pmanip ) {
                        pmanip->SetLocalToolTransform(_vtManipsLocalTool.at(indexAtSaveTime));
                        pmanip->SetLocalToolDirection(_vvManipsLocalDirection.at(indexAtSaveTime));
                        pmanip->SetIkSolver(_vpManipsIkSolver.at(indexAtSaveTime));
                    }
                }
            }
            else {
                RAVELOG_WARN(str(boost::format("failed to restore manipulators tool transform because the number of saved manipulators %i is different from the number of current manipulators %i\n")%_vtManipsLocalTool.size()%vmanips.size()));
            }
        }
    }
}

RobotBase::RobotBaseInfo& RobotBase::RobotBaseInfo::operator=(const RobotBase::RobotBaseInfo& other) {
    KinBodyInfo::operator=(other);
    _vManipulatorInfos.resize(other._vManipulatorInfos.size());
    for(size_t iManipulator = 0; iManipulator < other._vManipulatorInfos.size(); iManipulator++) {
        _vManipulatorInfos[iManipulator].reset(new ManipulatorInfo(*other._vManipulatorInfos[iManipulator]));
    }
    _vAttachedSensorInfos.resize(other._vAttachedSensorInfos.size());
    for(size_t iAttachedSensor = 0; iAttachedSensor < other._vAttachedSensorInfos.size(); iAttachedSensor++) {
        _vAttachedSensorInfos[iAttachedSensor].reset(new AttachedSensorInfo(*other._vAttachedSensorInfos[iAttachedSensor]));
    }
    _vGripperInfos.resize(other._vGripperInfos.size());
    for(size_t iGripperInfo = 0; iGripperInfo < other._vGripperInfos.size(); iGripperInfo++) {
        _vGripperInfos[iGripperInfo].reset(new GripperInfo(*other._vGripperInfos[iGripperInfo]));
    }
    _vConnectedBodyInfos.resize(other._vConnectedBodyInfos.size());
    for(size_t iConnectedBody = 0; iConnectedBody < other._vConnectedBodyInfos.size(); iConnectedBody++) {
        _vConnectedBodyInfos[iConnectedBody].reset(new ConnectedBodyInfo(*other._vConnectedBodyInfos[iConnectedBody]));
    }
    return *this;
}

bool RobotBase::RobotBaseInfo::operator==(const RobotBaseInfo& other) const {
    return KinBody::KinBodyInfo::operator==(other)
           && IsInfoVectorEqual(_vManipulatorInfos, other._vManipulatorInfos)
           && IsInfoVectorEqual(_vAttachedSensorInfos, other._vAttachedSensorInfos)
           && IsInfoVectorEqual(_vConnectedBodyInfos, other._vConnectedBodyInfos)
           && IsInfoVectorEqual(_vGripperInfos, other._vGripperInfos);
}

void RobotBase::RobotBaseInfo::Reset()
{
    KinBodyInfo::Reset();
    _vManipulatorInfos.clear();
    _vAttachedSensorInfos.clear();
    _vConnectedBodyInfos.clear();
    _vGripperInfos.clear();
}

void RobotBase::RobotBaseInfo::SerializeJSON(rapidjson::Value& rRobotBaseInfo, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    KinBody::KinBodyInfo::SerializeJSON(rRobotBaseInfo, allocator, fUnitScale, options);

    orjson::SetJsonValueByKey(rRobotBaseInfo, "isRobot", true, allocator);

    if (_vManipulatorInfos.size() > 0) {
        rapidjson::Value rManipulatorInfoValues;
        rManipulatorInfoValues.SetArray();
        rManipulatorInfoValues.Reserve(_vManipulatorInfos.size(), allocator);
        FOREACHC(it, _vManipulatorInfos) {
            rapidjson::Value manipInfoValue;
            (*it)->SerializeJSON(manipInfoValue, allocator, fUnitScale, options);
            rManipulatorInfoValues.PushBack(manipInfoValue, allocator);
        }
        rRobotBaseInfo.AddMember("tools", rManipulatorInfoValues, allocator);  // NOTICE: manipulator is changed name to tools in json scene
    }

    if (_vAttachedSensorInfos.size() > 0) {
        rapidjson::Value rAttachedSensorInfoValues;
        rAttachedSensorInfoValues.SetArray();
        rAttachedSensorInfoValues.Reserve(_vAttachedSensorInfos.size(), allocator);
        FOREACHC(it, _vAttachedSensorInfos) {
            rapidjson::Value attachedSensorInfoValue;
            (*it)->SerializeJSON(attachedSensorInfoValue, allocator, fUnitScale, options);
            rAttachedSensorInfoValues.PushBack(attachedSensorInfoValue, allocator);
        }
        rRobotBaseInfo.AddMember("attachedSensors", rAttachedSensorInfoValues, allocator);
    }

    if (_vConnectedBodyInfos.size() > 0) {
        rapidjson::Value rConnectedBodyInfoValues;
        rConnectedBodyInfoValues.SetArray();
        rConnectedBodyInfoValues.Reserve(_vConnectedBodyInfos.size(), allocator);
        FOREACHC(it, _vConnectedBodyInfos) {
            rapidjson::Value connectedBodyInfoValue;
            (*it)->SerializeJSON(connectedBodyInfoValue, allocator, fUnitScale, options);
            rConnectedBodyInfoValues.PushBack(connectedBodyInfoValue, allocator);
        }
        rRobotBaseInfo.AddMember("connectedBodies", rConnectedBodyInfoValues, allocator);
    }

    if (_vGripperInfos.size() > 0) {
        rapidjson::Value rGripperInfoValues;
        rGripperInfoValues.SetArray();
        rGripperInfoValues.Reserve(_vGripperInfos.size(), allocator);
        FOREACHC(it, _vGripperInfos) {
            rapidjson::Value gripperInfoValue;
            (*it)->SerializeJSON(gripperInfoValue, allocator, fUnitScale, options);
            rGripperInfoValues.PushBack(gripperInfoValue, allocator);
        }
        rRobotBaseInfo.AddMember("gripperInfos", rGripperInfoValues, allocator);
    }
}

void RobotBase::RobotBaseInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    KinBodyInfo::DeserializeJSON(value, fUnitScale, options);

    if (value.HasMember("tools")) {
        _vManipulatorInfos.reserve(value["tools"].Size() + _vManipulatorInfos.size());
        for (rapidjson::Value::ConstValueIterator it = value["tools"].Begin(); it != value["tools"].End(); ++it) {
            const rapidjson::Value& manipulatorValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(manipulatorValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(manipulatorValue, "name");
            }
            UpdateOrCreateInfo(manipulatorValue, id, _vManipulatorInfos, fUnitScale, options);
        }
    }

    if (value.HasMember("attachedSensors")) {
        _vAttachedSensorInfos.reserve(value["attachedSensors"].Size() + _vAttachedSensorInfos.size());
        for (rapidjson::Value::ConstValueIterator it = value["attachedSensors"].Begin(); it != value["attachedSensors"].End(); ++it) {
            const rapidjson::Value& attachedSensorValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(attachedSensorValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(attachedSensorValue, "name");
            }
            UpdateOrCreateInfo(attachedSensorValue, id, _vAttachedSensorInfos, fUnitScale, options);
        }
    }

    if (value.HasMember("connectedBodies")) {
        _vConnectedBodyInfos.reserve(value["connectedBodies"].Size() + _vConnectedBodyInfos.size());
        for (rapidjson::Value::ConstValueIterator it = value["connectedBodies"].Begin(); it != value["connectedBodies"].End(); ++it) {
            const rapidjson::Value& connectedBodyValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(connectedBodyValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(connectedBodyValue, "name");
            }
            UpdateOrCreateInfo(connectedBodyValue, id, _vConnectedBodyInfos, fUnitScale, options);
        }
    }

    if (value.HasMember("gripperInfos")) {
        _vGripperInfos.reserve(value["gripperInfos"].Size() + _vGripperInfos.size());
        for (rapidjson::Value::ConstValueIterator it = value["gripperInfos"].Begin(); it != value["gripperInfos"].End(); ++it) {
            const rapidjson::Value& gripperInfoValue = *it;
            std::string id = orjson::GetStringJsonValueByKey(gripperInfoValue, "id");
            if (id.empty()) {
                id = orjson::GetStringJsonValueByKey(gripperInfoValue, "name");
            }
            UpdateOrCreateInfo(gripperInfoValue, id, _vGripperInfos, fUnitScale, options);
        }
    }
}

void RobotBase::RobotBaseInfo::_DeserializeReadableInterface(const std::string& id, const rapidjson::Value& value) {
    BaseJSONReaderPtr pReader = RaveCallJSONReader(PT_Robot, id, RobotBasePtr(), AttributesList());
    if (!!pReader) {
        pReader->DeserializeJSON(value);
        ReadablePtr pReadable = pReader->GetReadable();
        if (!!pReadable) {
            _mReadableInterfaces[id] = pReadable;
        }
    }
    else if (value.IsString()) {
        StringReadablePtr pReadable(new StringReadable(id, value.GetString()));
        _mReadableInterfaces[id] = pReadable;
    }
}

RobotBase::RobotBase(EnvironmentBasePtr penv) : KinBody(PT_Robot, penv)
{
    _nAffineDOFs = 0;
    _nActiveDOF = -1;
    vActvAffineRotationAxis = Vector(0,0,1);

    //set limits for the affine DOFs
    _vTranslationLowerLimits = Vector(-100,-100,-100);
    _vTranslationUpperLimits = Vector(100,100,100);
    _vTranslationMaxVels = Vector(1.0f,1.0f,1.0f);
    _vTranslationResolutions = Vector(0.001f,0.001f,0.001f);
    _vTranslationWeights = Vector(2.0f,2.0f,2.0f);

    // rotation axis has infinite movement, so make sure the limits are big
    _vRotationAxisLowerLimits = Vector(-10000,-10000,-10000,10000);
    _vRotationAxisUpperLimits = Vector(10000,10000,10000,10000);

    _vRotationAxisMaxVels = Vector(0.4f,0.4f,0.4f,0.4f);
    _vRotationAxisResolutions = Vector(0.01f,0.01f,0.01f,0.01f);
    _vRotationAxisWeights = Vector(2.0f,2.0f,2.0f,2.0f);

    _vRotation3DLowerLimits = Vector(-10000,-10000,-10000);
    _vRotation3DUpperLimits = Vector(10000,10000,10000);
    _vRotation3DMaxVels = Vector(0.07f,0.07f,0.07f);
    _vRotation3DResolutions = Vector(0.01f,0.01f,0.01f);
    _vRotation3DWeights = Vector(1.0f,1.0f,1.0f);

    _vRotationQuatLimitStart = Vector(1,0,0,0);
    _fQuatLimitMaxAngle = PI;
    _fQuatMaxAngleVelocity = 1.0;
    _fQuatAngleResolution = 0.01f;
    _fQuatAngleWeight = 0.4f;
}

RobotBase::~RobotBase()
{
    Destroy();
}

void RobotBase::Destroy()
{
    _pManipActive.reset();
    _vecManipulators.clear();
    _vecAttachedSensors.clear();
    _vecConnectedBodies.clear();
    _nActiveDOF = 0;
    _vActiveDOFIndices.clear();
    _vAllDOFIndices.resize(0);
    SetController(ControllerBasePtr(),std::vector<int>(),0);

    KinBody::Destroy();
}

bool RobotBase::Init(const std::vector<KinBody::LinkInfoConstPtr>& linkinfos, const std::vector<KinBody::JointInfoConstPtr>& jointinfos, const std::vector<RobotBase::ManipulatorInfoConstPtr>& manipinfos, const std::vector<RobotBase::AttachedSensorInfoConstPtr>& attachedsensorinfos, const std::string& uri)
{
    if( !KinBody::Init(linkinfos, jointinfos, uri) ) {
        return false;
    }

    _vecManipulators.clear();
    _vecManipulators.reserve(manipinfos.size());
    FOREACHC(itmanipinfo, manipinfos) {
        ManipulatorPtr newmanip(new Manipulator(shared_robot(),**itmanipinfo));
        _vecManipulators.push_back(newmanip);
        __hashrobotstructure.resize(0);
    }
    _vecAttachedSensors.clear();
    _vecAttachedSensors.reserve(attachedsensorinfos.size());
    FOREACHC(itattachedsensorinfo, attachedsensorinfos) {
        AttachedSensorPtr newattachedsensor(new AttachedSensor(shared_robot(),**itattachedsensorinfo));
        _vecAttachedSensors.push_back(newattachedsensor);
        newattachedsensor->UpdateInfo(); // just in case
        __hashrobotstructure.resize(0);
    }
    _vecConnectedBodies.clear();

    return true;
}

bool RobotBase::InitFromRobotInfo(const RobotBaseInfo& info)
{
    std::vector<KinBody::LinkInfoConstPtr> vLinkInfosConst(info._vLinkInfos.begin(), info._vLinkInfos.end());
    std::vector<KinBody::JointInfoConstPtr> vJointInfosConst(info._vJointInfos.begin(), info._vJointInfos.end());

    std::vector<RobotBase::ManipulatorInfoConstPtr> vManipulatorInfosConst(info._vManipulatorInfos.begin(), info._vManipulatorInfos.end());
    std::vector<RobotBase::AttachedSensorInfoConstPtr> vAttachedSensorInfosConst(info._vAttachedSensorInfos.begin(), info._vAttachedSensorInfos.end());

    if( !RobotBase::Init(vLinkInfosConst, vJointInfosConst, vManipulatorInfosConst, vAttachedSensorInfosConst, info._uri) ) {
        return false;
    }

    _vecConnectedBodies.clear();
    FOREACHC(itconnectedbodyinfo, info._vConnectedBodyInfos) {
        ConnectedBodyPtr newconnectedbody(new ConnectedBody(shared_robot(), **itconnectedbodyinfo));
        _vecConnectedBodies.push_back(newconnectedbody);
    }

    _vecGripperInfos.clear();
    FOREACH(itgripperinfo, info._vGripperInfos) {
        GripperInfoPtr newGripperInfo(new GripperInfo(**itgripperinfo));
        _vecGripperInfos.push_back(newGripperInfo);
    }

    _id = info._id;
    _name = info._name;
    _referenceUri = info._referenceUri;

    FOREACH(it, info._mReadableInterfaces) {
        SetReadableInterface(it->first, it->second);
    }
    return true;
}

bool RobotBase::SetController(ControllerBasePtr controller, const std::vector<int>& jointindices, int nControlTransformation)
{
    RAVELOG_DEBUG_FORMAT("env=%d, default robot doesn't not support setting controllers (try GenericRobot)", GetEnv()->GetId());
    return false;
}

void RobotBase::SetName(const std::string& newname)
{
    if( _name != newname ) {
        // have to replace the 2nd word of all the groups with the robot name
        FOREACH(itgroup, _activespec._vgroups) {
            stringstream ss(itgroup->name);
            string grouptype, oldname;
            ss >> grouptype >> oldname;
            stringbuf buf;
            ss.get(buf,0);
            itgroup->name = str(boost::format("%s %s %s")%grouptype%newname%buf.str());
        }

        // have to rename any attached sensors with robotname:attachedname!!
        FOREACH(itattached, _vecAttachedSensors) {
            AttachedSensorPtr pattached = *itattached;
            if( !!pattached->_psensor ) {
                pattached->_psensor->SetName(str(boost::format("%s:%s")%newname%pattached->_info._name)); // need a unique targettable name
            }
            else {

            }
        }

        KinBody::SetName(newname);
    }
}

void RobotBase::SetDOFValues(const std::vector<dReal>& vJointValues, uint32_t bCheckLimits, const std::vector<int>& dofindices)
{
    KinBody::SetDOFValues(vJointValues, bCheckLimits,dofindices);
    _UpdateAttachedSensors();
}

void RobotBase::SetDOFValues(const std::vector<dReal>& vJointValues, const Transform& transbase, uint32_t bCheckLimits)
{
    KinBody::SetDOFValues(vJointValues, transbase, bCheckLimits); // should call RobotBase::SetDOFValues, so no need to upgrade grabbed bodies, attached sensors
}

void RobotBase::SetLinkTransformations(const std::vector<Transform>& transforms)
{
    KinBody::SetLinkTransformations(transforms);
    _UpdateAttachedSensors();
}

void RobotBase::SetLinkTransformations(const std::vector<Transform>& transforms, const std::vector<dReal>& doflastsetvalues)
{
    KinBody::SetLinkTransformations(transforms,doflastsetvalues);
    _UpdateAttachedSensors();
}

void RobotBase::SetTransform(const Transform& trans)
{
    KinBody::SetTransform(trans);
    _UpdateAttachedSensors();
}

bool RobotBase::SetVelocity(const Vector& linearvel, const Vector& angularvel)
{
    if( !KinBody::SetVelocity(linearvel,angularvel) ) {
        return false;
    }
    return true;
}

void RobotBase::SetDOFVelocities(const std::vector<dReal>& dofvelocities, const Vector& linearvel, const Vector& angularvel,uint32_t checklimits)
{
    KinBody::SetDOFVelocities(dofvelocities,linearvel,angularvel,checklimits);
    // do sensors need to have their velocities updated?
}

void RobotBase::SetDOFVelocities(const std::vector<dReal>& dofvelocities, uint32_t checklimits, const std::vector<int>& dofindices)
{
    KinBody::SetDOFVelocities(dofvelocities,checklimits, dofindices); // RobotBase::SetDOFVelocities should be called internally
}

void RobotBase::_UpdateAttachedSensors()
{
    FOREACH(itsensor, _vecAttachedSensors) {
        if( !!(*itsensor)->GetSensor() && !(*itsensor)->pattachedlink.expired() ) {
            (*itsensor)->GetSensor()->SetTransform(LinkPtr((*itsensor)->pattachedlink)->GetTransform()*(*itsensor)->GetRelativeTransform());
        }
    }
}

void RobotBase::_ResolveInfoIds()
{
    KinBody::_ResolveInfoIds();

    char sTempIndexConversion[9]; // temp memory space for converting indices to hex strings, enough space to convert uint32_t
    uint32_t nTempIndexConversion = 0; // length of sTempIndexConversion

    static const char pManipulatorIdPrefix[] = "tool";
    int nManipulatorId = 0;
    int nummanipulators = (int)_vecManipulators.size();
    for(int imanipulator = 0; imanipulator < nummanipulators; ++imanipulator) {
        RobotBase::ManipulatorInfo& manipulatorinfo = _vecManipulators[imanipulator]->_info;
        bool bGenerateNewId = manipulatorinfo._id.empty();
        if( !bGenerateNewId ) {
            for(int itestmanipulator = 0; itestmanipulator < imanipulator; ++itestmanipulator) {
                if( _vecManipulators[itestmanipulator]->_info._id == manipulatorinfo._id ) {
                    bGenerateNewId = true;
                    break;
                }
            }
        }

        if( bGenerateNewId ) {
            while(1) {
                nTempIndexConversion = ConvertUIntToHex(nManipulatorId, sTempIndexConversion);
                bool bHasSame = false;
                for(int itestmanipulator = 0; itestmanipulator < nummanipulators; ++itestmanipulator) {
                    const std::string& testid = _vecManipulators[itestmanipulator]->_info._id;
                    if( testid.size() == sizeof(pManipulatorIdPrefix)-1+nTempIndexConversion ) {
                        if( strncmp(testid.c_str() + (sizeof(pManipulatorIdPrefix)-1), sTempIndexConversion, nTempIndexConversion) == 0 ) {
                            // matches
                            bHasSame = true;
                            break;
                        }
                    }
                }

                if( bHasSame ) {
                    nManipulatorId++;
                    continue;
                }

                break;
            }

            manipulatorinfo._id = pManipulatorIdPrefix;
            manipulatorinfo._id += sTempIndexConversion;
            nManipulatorId++;
        }
    }

    static const char pAttachedSensorIdPrefix[] = "attachedSensor";
    int nAttachedSensorId = 0;
    int numattachedSensors = (int)_vecAttachedSensors.size();
    for(int iattachedSensor = 0; iattachedSensor < numattachedSensors; ++iattachedSensor) {
        RobotBase::AttachedSensorInfo& attachedSensorinfo = _vecAttachedSensors[iattachedSensor]->_info;
        bool bGenerateNewId = attachedSensorinfo._id.empty();
        if( !bGenerateNewId ) {
            for(int itestattachedSensor = 0; itestattachedSensor < iattachedSensor; ++itestattachedSensor) {
                if( _vecAttachedSensors[itestattachedSensor]->_info._id == attachedSensorinfo._id ) {
                    bGenerateNewId = true;
                    break;
                }
            }
        }

        if( bGenerateNewId ) {
            while(1) {
                nTempIndexConversion = ConvertUIntToHex(nAttachedSensorId, sTempIndexConversion);
                bool bHasSame = false;
                for(int itestattachedSensor = 0; itestattachedSensor < numattachedSensors; ++itestattachedSensor) {
                    const std::string& testid = _vecAttachedSensors[itestattachedSensor]->_info._id;
                    if( testid.size() == sizeof(pAttachedSensorIdPrefix)-1+nTempIndexConversion ) {
                        if( strncmp(testid.c_str() + (sizeof(pAttachedSensorIdPrefix)-1), sTempIndexConversion, nTempIndexConversion) == 0 ) {
                            // matches
                            bHasSame = true;
                            break;
                        }
                    }
                }

                if( bHasSame ) {
                    nAttachedSensorId++;
                    continue;
                }

                break;
            }

            attachedSensorinfo._id = pAttachedSensorIdPrefix;
            attachedSensorinfo._id += sTempIndexConversion;
            nAttachedSensorId++;
        }
    }

    static const char pConnectedBodyIdPrefix[] = "connectedBody";
    int nConnectedBodyId = 0;
    int numconnectedBodies = (int)_vecConnectedBodies.size();
    for(int iconnectedBody = 0; iconnectedBody < numconnectedBodies; ++iconnectedBody) {
        RobotBase::ConnectedBodyInfo& connectedBodyinfo = _vecConnectedBodies[iconnectedBody]->_info;
        bool bGenerateNewId = connectedBodyinfo._id.empty();
        if( !bGenerateNewId ) {
            for(int itestconnectedBody = 0; itestconnectedBody < iconnectedBody; ++itestconnectedBody) {
                if( _vecConnectedBodies[itestconnectedBody]->_info._id == connectedBodyinfo._id ) {
                    bGenerateNewId = true;
                    break;
                }
            }
        }

        if( bGenerateNewId ) {
            while(1) {
                nTempIndexConversion = ConvertUIntToHex(nConnectedBodyId, sTempIndexConversion);
                bool bHasSame = false;
                for(int itestconnectedBody = 0; itestconnectedBody < numconnectedBodies; ++itestconnectedBody) {
                    const std::string& testid = _vecConnectedBodies[itestconnectedBody]->_info._id;
                    if( testid.size() == sizeof(pConnectedBodyIdPrefix)-1+nTempIndexConversion ) {
                        if( strncmp(testid.c_str() + (sizeof(pConnectedBodyIdPrefix)-1), sTempIndexConversion, nTempIndexConversion) == 0 ) {
                            // matches
                            bHasSame = true;
                            break;
                        }
                    }
                }

                if( bHasSame ) {
                    nConnectedBodyId++;
                    continue;
                }

                break;
            }

            connectedBodyinfo._id = pConnectedBodyIdPrefix;
            connectedBodyinfo._id += sTempIndexConversion;
            nConnectedBodyId++;
        }
    }

    static const char pGripperInfoIdPrefix[] = "gripperInfo";
    int nGripperInfoId = 0;
    int numgripperInfos = (int)_vecGripperInfos.size();
    for(int igripperInfo = 0; igripperInfo < numgripperInfos; ++igripperInfo) {
        RobotBase::GripperInfo& gripperInfo = *_vecGripperInfos[igripperInfo];
        bool bGenerateNewId = gripperInfo._id.empty();
        if( !bGenerateNewId ) {
            for(int itestgripperInfo = 0; itestgripperInfo < igripperInfo; ++itestgripperInfo) {
                if( _vecGripperInfos[itestgripperInfo]->_id == gripperInfo._id ) {
                    bGenerateNewId = true;
                    break;
                }
            }
        }

        if( bGenerateNewId ) {
            while(1) {
                nTempIndexConversion = ConvertUIntToHex(nGripperInfoId, sTempIndexConversion);
                bool bHasSame = false;
                for(int itestgripperInfo = 0; itestgripperInfo < numgripperInfos; ++itestgripperInfo) {
                    const std::string& testid = _vecGripperInfos[itestgripperInfo]->_id;
                    if( testid.size() == sizeof(pGripperInfoIdPrefix)-1+nTempIndexConversion ) {
                        if( strncmp(testid.c_str() + (sizeof(pGripperInfoIdPrefix)-1), sTempIndexConversion, nTempIndexConversion) == 0 ) {
                            // matches
                            bHasSame = true;
                            break;
                        }
                    }
                }

                if( bHasSame ) {
                    nGripperInfoId++;
                    continue;
                }

                break;
            }

            gripperInfo._id = pGripperInfoIdPrefix;
            gripperInfo._id += sTempIndexConversion;
            nGripperInfoId++;
        }
    }
}

void RobotBase::SetAffineTranslationLimits(const Vector& lower, const Vector& upper)
{
    _vTranslationLowerLimits = lower;
    _vTranslationUpperLimits = upper;
}

void RobotBase::SetAffineRotationAxisLimits(const Vector& lower, const Vector& upper)
{
    _vRotationAxisLowerLimits = lower;
    _vRotationAxisUpperLimits = upper;
}

void RobotBase::SetAffineRotation3DLimits(const Vector& lower, const Vector& upper)
{
    _vRotation3DLowerLimits = lower;
    _vRotation3DUpperLimits = upper;
}

void RobotBase::SetAffineRotationQuatLimits(const Vector& quatangle)
{
    _fQuatLimitMaxAngle = RaveSqrt(quatangle.lengthsqr4());
    if( _fQuatLimitMaxAngle > 0 ) {
        _vRotationQuatLimitStart = quatangle * (1/_fQuatLimitMaxAngle);
    }
    else {
        _vRotationQuatLimitStart = GetTransform().rot;
    }
}

void RobotBase::SetAffineTranslationMaxVels(const Vector& vels)
{
    _vTranslationMaxVels = vels;
}

void RobotBase::SetAffineRotationAxisMaxVels(const Vector& vels)
{
    _vRotationAxisMaxVels = vels;
}

void RobotBase::SetAffineRotation3DMaxVels(const Vector& vels)
{
    _vRotation3DMaxVels = vels;
}

void RobotBase::SetAffineRotationQuatMaxVels(dReal anglevelocity)
{
    _fQuatMaxAngleVelocity = anglevelocity;
}

void RobotBase::SetAffineTranslationResolution(const Vector& resolution)
{
    _vTranslationResolutions = resolution;
}

void RobotBase::SetAffineRotationAxisResolution(const Vector& resolution)
{
    _vRotationAxisResolutions = resolution;
}

void RobotBase::SetAffineRotation3DResolution(const Vector& resolution)
{
    _vRotation3DResolutions = resolution;
}

void RobotBase::SetAffineRotationQuatResolution(dReal angleresolution)
{
    _fQuatAngleResolution = angleresolution;
}

void RobotBase::SetAffineTranslationWeights(const Vector& weights)
{
    _vTranslationWeights = weights;
}

void RobotBase::SetAffineRotationAxisWeights(const Vector& weights)
{
    _vRotationAxisWeights = weights;
}

void RobotBase::SetAffineRotation3DWeights(const Vector& weights)
{
    _vRotation3DWeights = weights;
}

void RobotBase::SetAffineRotationQuatWeights(dReal angleweight)
{
    _fQuatAngleWeight = angleweight;
}

void RobotBase::GetAffineTranslationLimits(Vector& lower, Vector& upper) const
{
    lower = _vTranslationLowerLimits;
    upper = _vTranslationUpperLimits;
}

void RobotBase::GetAffineRotationAxisLimits(Vector& lower, Vector& upper) const
{
    lower = _vRotationAxisLowerLimits;
    upper = _vRotationAxisUpperLimits;
}

void RobotBase::GetAffineRotation3DLimits(Vector& lower, Vector& upper) const
{
    lower = _vRotation3DLowerLimits;
    upper = _vRotation3DUpperLimits;
}

void RobotBase::SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOFBitmask, const Vector& vRotationAxis)
{
    vActvAffineRotationAxis = vRotationAxis;
    SetActiveDOFs(vJointIndices,nAffineDOFBitmask);
}

void RobotBase::SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOFBitmask)
{
    FOREACHC(itj, vJointIndices) {
        OPENRAVE_ASSERT_FORMAT(*itj>=0 && *itj<GetDOF(), "env=%d, robot '%s' bad index %d (dof=%d)",GetEnv()->GetId()%GetName()%(*itj)%GetDOF(),ORE_InvalidArguments);
    }
    // only reset the cache if the dof values are different
    if( _vActiveDOFIndices.size() != vJointIndices.size() ) {
        _nNonAdjacentLinkCache &= ~AO_ActiveDOFs;
    }
    else {
        for(size_t i = 0; i < vJointIndices.size(); ++i) {
            if( _vActiveDOFIndices[i] != vJointIndices[i] ) {
                _nNonAdjacentLinkCache &= ~AO_ActiveDOFs;
                break;
            }
        }
    }

    bool bactivedofchanged = false;
    if( _vActiveDOFIndices.size() != vJointIndices.size() ) {
        bactivedofchanged = true;
    }
    else {
        // same size, check to see if the values and order is the same
        for(size_t i = 0; i < _vActiveDOFIndices.size(); ++i) {
            if( _vActiveDOFIndices[i] != vJointIndices[i] ) {
                bactivedofchanged = true;
                break;
            }
        }
    }
    if( bactivedofchanged ) {
        _vActiveDOFIndices = vJointIndices;
    }

    if( _nAffineDOFs != nAffineDOFBitmask ) {
        bactivedofchanged = true;
        _nAffineDOFs = nAffineDOFBitmask;
    }

    _nActiveDOF = vJointIndices.size() + RaveGetAffineDOF(_nAffineDOFs);

    if( bactivedofchanged ) {
        // do not initialize interpolation, since it implies a motion sampling strategy
        int offset = 0;
        _activespec._vgroups.resize(0);
        if( GetActiveDOFIndices().size() > 0 ) {
            ConfigurationSpecification::Group group;
            stringstream ss;
            ss << "joint_values " << GetName();
            FOREACHC(it,GetActiveDOFIndices()) {
                ss << " " << *it;
            }
            group.name = ss.str();
            group.dof = (int)GetActiveDOFIndices().size();
            group.offset = offset;
            offset += group.dof;
            _activespec._vgroups.push_back(group);
        }
        if( GetAffineDOF() > 0 ) {
            ConfigurationSpecification::Group group;
            group.name = str(boost::format("affine_transform %s %d")%GetName()%GetAffineDOF());
            group.offset = offset;
            group.dof = RaveGetAffineDOF(GetAffineDOF());
            _activespec._vgroups.push_back(group);
        }

        _PostprocessChangedParameters(Prop_RobotActiveDOFs);
    }
}

void RobotBase::SetActiveDOFValues(const std::vector<dReal>& values, uint32_t bCheckLimits)
{
    if(_nActiveDOF < 0) {
        SetDOFValues(values,bCheckLimits);
        return;
    }
    OPENRAVE_ASSERT_OP_FORMAT((int)values.size(),>=,GetActiveDOF(), "not enough values %d<%d",values.size()%GetActiveDOF(),ORE_InvalidArguments);

    Transform t;
    if( (int)_vActiveDOFIndices.size() < _nActiveDOF ) {
        t = GetTransform();
        RaveGetTransformFromAffineDOFValues(t, values.begin()+_vActiveDOFIndices.size(),_nAffineDOFs,vActvAffineRotationAxis);
        if( _nAffineDOFs & DOF_RotationQuat ) {
            t.rot = quatMultiply(_vRotationQuatLimitStart, t.rot);
        }
        if( _vActiveDOFIndices.size() == 0 ) {
            SetTransform(t);
        }
    }

    if( _vActiveDOFIndices.size() > 0 ) {
        GetDOFValues(_vTempRobotJoints);
        for(size_t i = 0; i < _vActiveDOFIndices.size(); ++i) {
            _vTempRobotJoints[_vActiveDOFIndices[i]] = values[i];
        }
        if( (int)_vActiveDOFIndices.size() < _nActiveDOF ) {
            SetDOFValues(_vTempRobotJoints, t, bCheckLimits);
        }
        else {
            SetDOFValues(_vTempRobotJoints, bCheckLimits);
        }
    }
}

void RobotBase::GetActiveDOFValues(std::vector<dReal>& values) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFValues(values);
        return;
    }

    values.resize(GetActiveDOF());
    if( values.size() == 0 ) {
        return;
    }
    vector<dReal>::iterator itvalues = values.begin();
    if( _vActiveDOFIndices.size() != 0 ) {
        GetDOFValues(_vTempRobotJoints);
        FOREACHC(it, _vActiveDOFIndices) {
            *itvalues++ = _vTempRobotJoints[*it];
        }
    }

    if( _nAffineDOFs == DOF_NoTransform ) {
        return;
    }
    Transform t = GetTransform();
    if( _nAffineDOFs & DOF_RotationQuat ) {
        t.rot = quatMultiply(quatInverse(_vRotationQuatLimitStart), t.rot);
    }
    RaveGetAffineDOFValuesFromTransform(itvalues,t,_nAffineDOFs,vActvAffineRotationAxis);
}

void RobotBase::SetActiveDOFVelocities(const std::vector<dReal>& velocities, uint32_t bCheckLimits)
{
    if(_nActiveDOF < 0) {
        SetDOFVelocities(velocities,true);
        return;
    }
    OPENRAVE_ASSERT_OP_FORMAT((int)velocities.size(),>=,GetActiveDOF(), "not enough values %d<%d",velocities.size()%GetActiveDOF(),ORE_InvalidArguments);

    Vector linearvel, angularvel;
    if( (int)_vActiveDOFIndices.size() < _nActiveDOF ) {
        // first set the affine transformation of the first link before setting joints
        const dReal* pAffineValues = &velocities[_vActiveDOFIndices.size()];

        _veclinks.at(0)->GetVelocity(linearvel, angularvel);

        if( _nAffineDOFs & DOF_X ) linearvel.x = *pAffineValues++;
        if( _nAffineDOFs & DOF_Y ) linearvel.y = *pAffineValues++;
        if( _nAffineDOFs & DOF_Z ) linearvel.z = *pAffineValues++;
        if( _nAffineDOFs & DOF_RotationAxis ) {
            angularvel = vActvAffineRotationAxis * *pAffineValues++;
        }
        else if( _nAffineDOFs & DOF_Rotation3D ) {
            angularvel.x = *pAffineValues++;
            angularvel.y = *pAffineValues++;
            angularvel.z = *pAffineValues++;
        }
        else if( _nAffineDOFs & DOF_RotationQuat ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("quaternions not supported"),ORE_InvalidArguments);
        }

        if( _vActiveDOFIndices.size() == 0 ) {
            SetVelocity(linearvel, angularvel);
        }
    }

    if( _vActiveDOFIndices.size() > 0 ) {
        GetDOFVelocities(_vTempRobotJoints);
        std::vector<dReal>::const_iterator itvel = velocities.begin();
        FOREACHC(it, _vActiveDOFIndices) {
            _vTempRobotJoints[*it] = *itvel++;
        }
        if( (int)_vActiveDOFIndices.size() < _nActiveDOF ) {
            SetDOFVelocities(_vTempRobotJoints,linearvel,angularvel,bCheckLimits);
        }
        else {
            SetDOFVelocities(_vTempRobotJoints,bCheckLimits);
        }
    }
}

void RobotBase::GetActiveDOFVelocities(std::vector<dReal>& velocities) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFVelocities(velocities);
        return;
    }

    velocities.resize(GetActiveDOF());
    if( velocities.size() == 0 )
        return;
    dReal* pVelocities = &velocities[0];
    if( _vActiveDOFIndices.size() != 0 ) {
        GetDOFVelocities(_vTempRobotJoints);
        FOREACHC(it, _vActiveDOFIndices) {
            *pVelocities++ = _vTempRobotJoints[*it];
        }
    }

    if( _nAffineDOFs == DOF_NoTransform ) {
        return;
    }
    Vector linearvel, angularvel;
    _veclinks.at(0)->GetVelocity(linearvel, angularvel);

    if( _nAffineDOFs & DOF_X ) *pVelocities++ = linearvel.x;
    if( _nAffineDOFs & DOF_Y ) *pVelocities++ = linearvel.y;
    if( _nAffineDOFs & DOF_Z ) *pVelocities++ = linearvel.z;
    if( _nAffineDOFs & DOF_RotationAxis ) {

        *pVelocities++ = vActvAffineRotationAxis.dot3(angularvel);
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pVelocities++ = angularvel.x;
        *pVelocities++ = angularvel.y;
        *pVelocities++ = angularvel.z;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("quaternions not supported"),ORE_InvalidArguments);
    }
}

void RobotBase::GetActiveDOFLimits(std::vector<dReal>& lower, std::vector<dReal>& upper) const
{
    lower.resize(GetActiveDOF());
    upper.resize(GetActiveDOF());
    if( GetActiveDOF() == 0 ) {
        return;
    }
    dReal* pLowerLimit = &lower[0];
    dReal* pUpperLimit = &upper[0];
    vector<dReal> alllower,allupper;

    if( _nAffineDOFs == 0 ) {
        if( _nActiveDOF < 0 ) {
            GetDOFLimits(lower,upper);
            return;
        }
        else {
            GetDOFLimits(alllower,allupper);
            FOREACHC(it, _vActiveDOFIndices) {
                *pLowerLimit++ = alllower.at(*it);
                *pUpperLimit++ = allupper.at(*it);
            }
        }
    }
    else {
        if( _vActiveDOFIndices.size() > 0 ) {
            GetDOFLimits(alllower,allupper);
            FOREACHC(it, _vActiveDOFIndices) {
                *pLowerLimit++ = alllower.at(*it);
                *pUpperLimit++ = allupper.at(*it);
            }
        }

        if( _nAffineDOFs & DOF_X ) {
            *pLowerLimit++ = _vTranslationLowerLimits.x;
            *pUpperLimit++ = _vTranslationUpperLimits.x;
        }
        if( _nAffineDOFs & DOF_Y ) {
            *pLowerLimit++ = _vTranslationLowerLimits.y;
            *pUpperLimit++ = _vTranslationUpperLimits.y;
        }
        if( _nAffineDOFs & DOF_Z ) {
            *pLowerLimit++ = _vTranslationLowerLimits.z;
            *pUpperLimit++ = _vTranslationUpperLimits.z;
        }

        if( _nAffineDOFs & DOF_RotationAxis ) {
            *pLowerLimit++ = _vRotationAxisLowerLimits.x;
            *pUpperLimit++ = _vRotationAxisUpperLimits.x;
        }
        else if( _nAffineDOFs & DOF_Rotation3D ) {
            *pLowerLimit++ = _vRotation3DLowerLimits.x;
            *pLowerLimit++ = _vRotation3DLowerLimits.y;
            *pLowerLimit++ = _vRotation3DLowerLimits.z;
            *pUpperLimit++ = _vRotation3DUpperLimits.x;
            *pUpperLimit++ = _vRotation3DUpperLimits.y;
            *pUpperLimit++ = _vRotation3DUpperLimits.z;
        }
        else if( _nAffineDOFs & DOF_RotationQuat ) {
            // this is actually difficult to do correctly...
            dReal fsin = RaveSin(_fQuatLimitMaxAngle);
            *pLowerLimit++ = RaveCos(_fQuatLimitMaxAngle);
            *pLowerLimit++ = -fsin;
            *pLowerLimit++ = -fsin;
            *pLowerLimit++ = -fsin;
            *pUpperLimit++ = 1;
            *pUpperLimit++ = fsin;
            *pUpperLimit++ = fsin;
            *pUpperLimit++ = fsin;
        }
    }
}

void RobotBase::GetActiveDOFResolutions(std::vector<dReal>& resolution) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFResolutions(resolution);
        return;
    }

    resolution.resize(GetActiveDOF());
    if( resolution.size() == 0 ) {
        return;
    }
    dReal* pResolution = &resolution[0];

    GetDOFResolutions(_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pResolution++ = _vTempRobotJoints[*it];
    }
    // set some default limits
    if( _nAffineDOFs & DOF_X ) {
        *pResolution++ = _vTranslationResolutions.x;
    }
    if( _nAffineDOFs & DOF_Y ) {
        *pResolution++ = _vTranslationResolutions.y;
    }
    if( _nAffineDOFs & DOF_Z ) { *pResolution++ = _vTranslationResolutions.z; }

    if( _nAffineDOFs & DOF_RotationAxis ) {
        *pResolution++ = _vRotationAxisResolutions.x;
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pResolution++ = _vRotation3DResolutions.x;
        *pResolution++ = _vRotation3DResolutions.y;
        *pResolution++ = _vRotation3DResolutions.z;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        *pResolution++ = _fQuatLimitMaxAngle;
        *pResolution++ = _fQuatLimitMaxAngle;
        *pResolution++ = _fQuatLimitMaxAngle;
        *pResolution++ = _fQuatLimitMaxAngle;
    }
}

void RobotBase::GetActiveDOFWeights(std::vector<dReal>& weights) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFWeights(weights);
        return;
    }

    weights.resize(GetActiveDOF());
    if( weights.size() == 0 ) {
        return;
    }
    dReal* pweight = &weights[0];

    GetDOFWeights(_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pweight++ = _vTempRobotJoints[*it];
    }
    // set some default limits
    if( _nAffineDOFs & DOF_X ) { *pweight++ = _vTranslationWeights.x; }
    if( _nAffineDOFs & DOF_Y ) { *pweight++ = _vTranslationWeights.y; }
    if( _nAffineDOFs & DOF_Z ) { *pweight++ = _vTranslationWeights.z; }

    if( _nAffineDOFs & DOF_RotationAxis ) { *pweight++ = _vRotationAxisWeights.x; }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pweight++ = _vRotation3DWeights.x;
        *pweight++ = _vRotation3DWeights.y;
        *pweight++ = _vRotation3DWeights.z;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        *pweight++ = _fQuatAngleWeight;
        *pweight++ = _fQuatAngleWeight;
        *pweight++ = _fQuatAngleWeight;
        *pweight++ = _fQuatAngleWeight;
    }
}

void RobotBase::GetActiveDOFVelocityLimits(std::vector<dReal>& maxvel) const
{
    std::vector<dReal> dummy;
    if( _nActiveDOF < 0 ) {
        GetDOFVelocityLimits(dummy,maxvel);
        return;
    }
    maxvel.resize(GetActiveDOF());
    if( maxvel.size() == 0 ) {
        return;
    }
    dReal* pMaxVel = &maxvel[0];

    GetDOFVelocityLimits(dummy,_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pMaxVel++ = _vTempRobotJoints[*it];
    }
    if( _nAffineDOFs & DOF_X ) { *pMaxVel++ = _vTranslationMaxVels.x; }
    if( _nAffineDOFs & DOF_Y ) { *pMaxVel++ = _vTranslationMaxVels.y; }
    if( _nAffineDOFs & DOF_Z ) { *pMaxVel++ = _vTranslationMaxVels.z; }

    if( _nAffineDOFs & DOF_RotationAxis ) { *pMaxVel++ = _vRotationAxisMaxVels.x; }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pMaxVel++ = _vRotation3DMaxVels.x;
        *pMaxVel++ = _vRotation3DMaxVels.y;
        *pMaxVel++ = _vRotation3DMaxVels.z;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        *pMaxVel++ = _fQuatMaxAngleVelocity;
        *pMaxVel++ = _fQuatMaxAngleVelocity;
        *pMaxVel++ = _fQuatMaxAngleVelocity;
        *pMaxVel++ = _fQuatMaxAngleVelocity;
    }
}

void RobotBase::GetActiveDOFAccelerationLimits(std::vector<dReal>& maxaccel) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFAccelerationLimits(maxaccel);
        return;
    }
    maxaccel.resize(GetActiveDOF());
    if( maxaccel.size() == 0 ) {
        return;
    }
    dReal* pMaxAccel = &maxaccel[0];

    GetDOFAccelerationLimits(_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pMaxAccel++ = _vTempRobotJoints[*it];
    }
    if( _nAffineDOFs & DOF_X ) { *pMaxAccel++ = _vTranslationMaxVels.x; } // wrong
    if( _nAffineDOFs & DOF_Y ) { *pMaxAccel++ = _vTranslationMaxVels.y; } // wrong
    if( _nAffineDOFs & DOF_Z ) { *pMaxAccel++ = _vTranslationMaxVels.z; } // wrong

    if( _nAffineDOFs & DOF_RotationAxis ) { *pMaxAccel++ = _vRotationAxisMaxVels.x; } // wrong
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        *pMaxAccel++ = _vRotation3DMaxVels.x; // wrong
        *pMaxAccel++ = _vRotation3DMaxVels.y; // wrong
        *pMaxAccel++ = _vRotation3DMaxVels.z; // wrong
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
        *pMaxAccel++ = _fQuatMaxAngleVelocity; // wrong
    }
}

void RobotBase::GetActiveDOFJerkLimits(std::vector<dReal>& maxjerk) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFJerkLimits(maxjerk);
        return;
    }
    maxjerk.resize(GetActiveDOF());
    if( maxjerk.size() == 0 ) {
        return;
    }
    dReal* pMaxJerk = &maxjerk[0];

    GetDOFJerkLimits(_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pMaxJerk++ = _vTempRobotJoints[*it];
    }
}

void RobotBase::GetActiveDOFHardVelocityLimits(std::vector<dReal>& maxvel) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFHardVelocityLimits(maxvel);
        return;
    }
    maxvel.resize(GetActiveDOF());
    if( maxvel.size() == 0 ) {
        return;
    }
    dReal* pMaxVel = &maxvel[0];

    GetDOFHardVelocityLimits(_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pMaxVel++ = _vTempRobotJoints[*it];
    }
}

void RobotBase::GetActiveDOFHardAccelerationLimits(std::vector<dReal>& maxaccel) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFHardAccelerationLimits(maxaccel);
        return;
    }
    maxaccel.resize(GetActiveDOF());
    if( maxaccel.size() == 0 ) {
        return;
    }
    dReal* pMaxAccel = &maxaccel[0];

    GetDOFHardAccelerationLimits(_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pMaxAccel++ = _vTempRobotJoints[*it];
    }
}

void RobotBase::GetActiveDOFHardJerkLimits(std::vector<dReal>& maxjerk) const
{
    if( _nActiveDOF < 0 ) {
        GetDOFHardJerkLimits(maxjerk);
        return;
    }
    maxjerk.resize(GetActiveDOF());
    if( maxjerk.size() == 0 ) {
        return;
    }
    dReal* pMaxJerk = &maxjerk[0];

    GetDOFHardJerkLimits(_vTempRobotJoints);
    FOREACHC(it, _vActiveDOFIndices) {
        *pMaxJerk++ = _vTempRobotJoints[*it];
    }
}

void RobotBase::SubtractActiveDOFValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const
{
    if( _nActiveDOF < 0 ) {
        SubtractDOFValues(q1,q2);
        return;
    }

    OPENRAVE_ASSERT_OP(q1.size(),==,q2.size());
    OPENRAVE_ASSERT_OP(q1.size(), >=, _vActiveDOFIndices.size());
    size_t index = 0;
    if (_bAreAllJoints1DOFAndNonCircular) {
        for (size_t i = 0; i < _vActiveDOFIndices.size(); ++i) {
            q1[i] -= q2[i];
        }
        index = _vActiveDOFIndices.size();
    }
    else {
        // go through all active joints
        for(; index < _vActiveDOFIndices.size(); ++index) {
            // We already did range check above
            JointConstPtr pjoint = GetJointFromDOFIndex(_vActiveDOFIndices[index]);
            q1[index] = pjoint->SubtractValue(q1[index],q2[index],_vActiveDOFIndices[index]-pjoint->GetDOFIndex());
        }
    }

    if( _nAffineDOFs == DOF_NoTransform ) {
        return;
    }

    if( _nAffineDOFs & DOF_X ) {
        q1.at(index) -= q2.at(index);
        index++;
    }
    if( _nAffineDOFs & DOF_Y ) {
        q1.at(index) -= q2.at(index);
        index++;
    }
    if( _nAffineDOFs & DOF_Z ) {
        q1.at(index) -= q2.at(index);
        index++;
    }

    if( _nAffineDOFs & DOF_RotationAxis ) {
        q1.at(index) = utils::SubtractCircularAngle(q1.at(index),q2.at(index));
        index++;
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        // would like to do q2^-1 q1, but that might break rest of planners...?
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
        q1.at(index) -= q2.at(index); index++;
    }
}

const std::vector<int>& RobotBase::GetActiveDOFIndices() const
{
    return _nActiveDOF < 0 ? _vAllDOFIndices : _vActiveDOFIndices;
}

ConfigurationSpecification RobotBase::GetActiveConfigurationSpecification(const std::string& interpolation) const
{
    if( interpolation.size() == 0 ) {
        return _activespec;
    }
    ConfigurationSpecification spec = _activespec;
    FOREACH(itgroup,spec._vgroups) {
        itgroup->interpolation=interpolation;
    }
    return spec;
}

void RobotBase::CalculateActiveJacobian(int index, const Vector& offset, vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateJacobian(index, offset, vjacobian);
        return;
    }

    int dofstride = GetActiveDOF();
    vjacobian.resize(3*dofstride);
    if( _vActiveDOFIndices.size() != 0 ) {
        if( _nAffineDOFs == DOF_NoTransform ) {
            ComputeJacobianTranslation(index, offset, vjacobian, _vActiveDOFIndices);
            return;
        }
        // have to copy
        std::vector<dReal> vjacobianjoints;
        ComputeJacobianTranslation(index, offset, vjacobianjoints, _vActiveDOFIndices);
        for(size_t i = 0; i < 3; ++i) {
            std::copy(vjacobianjoints.begin()+i*_vActiveDOFIndices.size(),vjacobianjoints.begin()+(i+1)*_vActiveDOFIndices.size(),vjacobian.begin()+i*dofstride);
        }
    }

    if( _nAffineDOFs == DOF_NoTransform ) {
        return;
    }
    size_t ind = _vActiveDOFIndices.size();
    if( _nAffineDOFs & DOF_X ) {
        vjacobian[ind] = 1;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Y ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 1;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Z ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 1;
        ind++;
    }
    if( _nAffineDOFs & DOF_RotationAxis ) {
        Vector vj = vActvAffineRotationAxis.cross(offset-GetTransform().trans);
        vjacobian[ind] = vj.x;
        vjacobian[dofstride+ind] = vj.y;
        vjacobian[2*dofstride+ind] = vj.z;
        ind++;
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        // have to take the partial derivative dT/dA of the axis*angle representation with respect to the transformation it induces
        // can introduce converting to quaternions in the middle, then by chain rule,  dT/dA = dT/tQ * dQ/dA
        // for questions on derivation email rdiankov@cs.cmu.edu
        Transform t = GetTransform();
        dReal Qx = t.rot.x, Qy = t.rot.y, Qz = t.rot.z, Qw = t.rot.w;
        dReal Tx = offset.x-t.trans.x, Ty = offset.y-t.trans.y, Tz = offset.z-t.trans.z;

        // after some math, the dT/dQ looks like:
        dReal dRQ[12] = { 2*Qy*Ty+2*Qz*Tz,         -4*Qy*Tx+2*Qx*Ty+2*Qw*Tz,   -4*Qz*Tx-2*Qw*Ty+2*Qx*Tz,   -2*Qz*Ty+2*Qy*Tz,
                          2*Qy*Tx-4*Qx*Ty-2*Qw*Tz, 2*Qx*Tx+2*Qz*Tz,            2*Qw*Tx-4*Qz*Ty+2*Qy*Tz,    2*Qz*Tx-2*Qx*Tz,
                          2*Qz*Tx+2*Qw*Ty-4*Qx*Tz, -2*Qw*Tx+2*Qz*Ty-4*Qy*Tz,   2*Qx*Tx+2*Qy*Ty,            -2*Qy*Tx+2*Qx*Ty };

        // calc dQ/dA
        dReal fsin = sqrt(t.rot.y * t.rot.y + t.rot.z * t.rot.z + t.rot.w * t.rot.w);
        dReal fcos = t.rot.x;
        dReal fangle = 2 * atan2(fsin, fcos);
        dReal normalizer = fangle / fsin;
        dReal Ax = normalizer * t.rot.y;
        dReal Ay = normalizer * t.rot.z;
        dReal Az = normalizer * t.rot.w;

        if( RaveFabs(fangle) < 1e-8f )
            fangle = 1e-8f;

        dReal fangle2 = fangle*fangle;
        dReal fiangle2 = 1/fangle2;
        dReal inormalizer = normalizer > 0 ? 1/normalizer : 0;
        dReal fconst = inormalizer*fiangle2;
        dReal fconst2 = fcos*fiangle2;
        dReal dQA[12] = { -0.5f*Ax*inormalizer,                     -0.5f*Ay*inormalizer,                       -0.5f*Az*inormalizer,
                          inormalizer+0.5f*Ax*Ax*(fconst2-fconst),  0.5f*Ax*fconst2*Ay-Ax*fconst*Ay,            0.5f*Ax*fconst2*Az-Ax*fconst*Az,
                          0.5f*Ax*fconst2*Ay-Ax*fconst*Ay,          inormalizer+0.5f*Ay*Ay*(fconst2-fconst),    0.5f*Ay*fconst2*Az-Ay*fconst*Az,
                          0.5f*Ax*fconst2*Az-Ax*fconst*Az,          0.5f*Ay*fconst2*Az-Ay*fconst*Az,            inormalizer+0.5f*Az*Az*(fconst2-fconst)};

        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                vjacobian[i*dofstride+ind+j] = dRQ[4*i+0]*dQA[3*0+j] + dRQ[4*i+1]*dQA[3*1+j] + dRQ[4*i+2]*dQA[3*2+j] + dRQ[4*i+3]*dQA[3*3+j];
            }
        }
        ind += 3;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        Transform t; t.identity(); t.rot = quatInverse(_vRotationQuatLimitStart);
        t = t * GetTransform();
        // note: qw, qx, qy, qz here follow the standard quaternion convention, not the openrave one
        dReal qw = t.rot[0], qx = t.rot[1], qy = t.rot[2], qz = t.rot[3];
        Vector offset_local = t.inverse() * offset;
        dReal x = offset_local.x, y = offset_local.y, z = offset_local.z;

        dReal dRQ[12] = {-2*qz*y + 2*qw*x + 2*qy*z,2*qx*x + 2*qy*y + 2*qz*z,-2*qy*x + 2*qw*z + 2*qx*y,-2*qw*y - 2*qz*x + 2*qx*z,-2*qx*z + 2*qw*y + 2*qz*x,-2*qw*z - 2*qx*y + 2*qy*x,2*qx*x + 2*qy*y + 2*qz*z,-2*qz*y + 2*qw*x + 2*qy*z,-2*qy*x + 2*qw*z + 2*qx*y,-2*qx*z + 2*qw*y + 2*qz*x,-2*qw*x - 2*qy*z + 2*qz*y,2*qx*x + 2*qy*y + 2*qz*z};
        for (int i=0; i < 3; ++i) {
            double qdotrow = dRQ[4*i]*qw + dRQ[4*i+1]*qx + dRQ[4*i+2]*qy + dRQ[4*i+3]*qz;
            dRQ[4*i] -= qdotrow*qw;
            dRQ[4*i+1] -= qdotrow*qx;
            dRQ[4*i+2] -= qdotrow*qy;
            dRQ[4*i+3] -= qdotrow*qz;
        }

        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 4; ++j) {
                vjacobian[i*dofstride+ind + j] = dRQ[4*i+j];
            }
        }
        ind += 4;
    }
}

void RobotBase::CalculateActiveJacobian(int linkindex, const Vector& offset, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateJacobian(linkindex, offset, mjacobian);
        return;
    }
    std::vector<dReal> vjacobian;
    RobotBase::CalculateActiveJacobian(linkindex,offset,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*GetActiveDOF());
    mjacobian.resize(boost::extents[3][GetActiveDOF()]);
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetActiveDOF(),itdst->begin());
        itsrc += GetActiveDOF();
    }
}

void RobotBase::CalculateActiveRotationJacobian(int index, const Vector& q, std::vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateRotationJacobian(index, q, vjacobian);
        return;
    }
    int dofstride = GetActiveDOF();
    vjacobian.resize(4*dofstride);
    if( _vActiveDOFIndices.size() != 0 ) {
        std::vector<dReal> vjacobianjoints;
        CalculateRotationJacobian(index, q, vjacobianjoints);
        for(size_t i = 0; i < _vActiveDOFIndices.size(); ++i) {
            vjacobian[i] = vjacobianjoints[_vActiveDOFIndices[i]];
            vjacobian[dofstride+i] = vjacobianjoints[GetDOF()+_vActiveDOFIndices[i]];
            vjacobian[2*dofstride+i] = vjacobianjoints[2*GetDOF()+_vActiveDOFIndices[i]];
            vjacobian[3*dofstride+i] = vjacobianjoints[3*GetDOF()+_vActiveDOFIndices[i]];
        }
    }

    if( _nAffineDOFs == DOF_NoTransform ) {
        return;
    }

    size_t ind = _vActiveDOFIndices.size();
    if( _nAffineDOFs & DOF_X ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        vjacobian[3*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Y ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        vjacobian[3*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Z ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        vjacobian[3*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        vjacobian[ind] = dReal(0.5)*(-q.y*v.x - q.z*v.y - q.w*v.z);
        vjacobian[dofstride+ind] = dReal(0.5)*(q.x*v.x - q.z*v.z + q.w*v.y);
        vjacobian[2*dofstride+ind] = dReal(0.5)*(q.x*v.y + q.y*v.z - q.w*v.x);
        vjacobian[3*dofstride+ind] = dReal(0.5)*(q.x*v.z - q.y*v.y + q.z*v.x);
        ind++;
    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("robot %s rotation 3d not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);
        ind += 3;
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("robot %s quaternion not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);
        ind += 4;
    }
}

void RobotBase::CalculateActiveRotationJacobian(int linkindex, const Vector& q, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateRotationJacobian(linkindex, q, mjacobian);
        return;
    }
    std::vector<dReal> vjacobian;
    RobotBase::CalculateActiveRotationJacobian(linkindex,q,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,4*GetActiveDOF());
    mjacobian.resize(boost::extents[4][GetActiveDOF()]);
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetActiveDOF(),itdst->begin());
        itsrc += GetActiveDOF();
    }
}

void RobotBase::CalculateActiveAngularVelocityJacobian(int index, std::vector<dReal>& vjacobian) const
{
    if( _nActiveDOF < 0 ) {
        ComputeJacobianAxisAngle(index, vjacobian);
        return;
    }

    int dofstride = GetActiveDOF();
    vjacobian.resize(3*dofstride);
    if( _vActiveDOFIndices.size() != 0 ) {
        if( _nAffineDOFs == DOF_NoTransform ) {
            ComputeJacobianAxisAngle(index, vjacobian, _vActiveDOFIndices);
            return;
        }
        // have to copy
        std::vector<dReal> vjacobianjoints;
        ComputeJacobianAxisAngle(index, vjacobianjoints, _vActiveDOFIndices);
        for(size_t i = 0; i < 3; ++i) {
            std::copy(vjacobianjoints.begin()+i*_vActiveDOFIndices.size(),vjacobianjoints.begin()+(i+1)*_vActiveDOFIndices.size(),vjacobian.begin()+i*dofstride);
        }
    }

    if( _nAffineDOFs == DOF_NoTransform ) {
        return;
    }
    size_t ind = _vActiveDOFIndices.size();
    if( _nAffineDOFs & DOF_X ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Y ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_Z ) {
        vjacobian[ind] = 0;
        vjacobian[dofstride+ind] = 0;
        vjacobian[2*dofstride+ind] = 0;
        ind++;
    }
    if( _nAffineDOFs & DOF_RotationAxis ) {
        const Vector& v = vActvAffineRotationAxis;
        vjacobian[ind] = v.x;
        vjacobian[dofstride+ind] = v.y;
        vjacobian[2*dofstride+ind] = v.z;

    }
    else if( _nAffineDOFs & DOF_Rotation3D ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("robot %s rotation 3d not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);
    }
    else if( _nAffineDOFs & DOF_RotationQuat ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("robot %s quaternion not supported, affine=%d"),GetName()%_nAffineDOFs,ORE_NotImplemented);

        // most likely wrong
        Transform t; t.rot = quatInverse(_vRotationQuatLimitStart);
        t = t * GetTransform();
        dReal fnorm = t.rot.y*t.rot.y+t.rot.z*t.rot.z+t.rot.w*t.rot.w;
        if( fnorm > 0 ) {
            fnorm = dReal(1)/RaveSqrt(fnorm);
            vjacobian[ind] = t.rot.y*fnorm;
            vjacobian[dofstride+ind] = t.rot.z*fnorm;
            vjacobian[2*dofstride+ind] = t.rot.w*fnorm;
        }
        else {
            vjacobian[ind] = 0;
            vjacobian[dofstride+ind] = 0;
            vjacobian[2*dofstride+ind] = 0;
        }

        ++ind;
    }
}

void RobotBase::CalculateActiveAngularVelocityJacobian(int linkindex, boost::multi_array<dReal,2>& mjacobian) const
{
    if( _nActiveDOF < 0 ) {
        CalculateAngularVelocityJacobian(linkindex, mjacobian);
        return;
    }
    std::vector<dReal> vjacobian;
    CalculateActiveAngularVelocityJacobian(linkindex,vjacobian);
    OPENRAVE_ASSERT_OP((int)vjacobian.size(),==,3*GetActiveDOF());
    mjacobian.resize(boost::extents[3][GetActiveDOF()]);
    vector<dReal>::const_iterator itsrc = vjacobian.begin();
    FOREACH(itdst,mjacobian) {
        std::copy(itsrc,itsrc+GetActiveDOF(),itdst->begin());
        itsrc += GetActiveDOF();
    }
}

bool CompareNonAdjacentFarthest(int pair0, int pair1); // defined in kinbody.cpp

const std::vector<int>& RobotBase::GetNonAdjacentLinks(int adjacentoptions) const
{
    KinBody::GetNonAdjacentLinks(0); // need to call to set the cache
    if( (_nNonAdjacentLinkCache&adjacentoptions) != adjacentoptions ) {
        int requestedoptions = (~_nNonAdjacentLinkCache)&adjacentoptions;
        // find out what needs to computed
        boost::array<uint8_t,4> compute={ { 0,0,0,0}};
        if( requestedoptions & AO_Enabled ) {
            for(size_t i = 0; i < compute.size(); ++i) {
                if( i & AO_Enabled ) {
                    compute[i] = 1;
                }
            }
        }
        if( requestedoptions & AO_ActiveDOFs ) {
            for(size_t i = 0; i < compute.size(); ++i) {
                if( i & AO_ActiveDOFs ) {
                    compute[i] = 1;
                }
            }
        }
        if( requestedoptions & ~(AO_Enabled|AO_ActiveDOFs) ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("does not support adjacentoptions %d"),adjacentoptions,ORE_InvalidArguments);
        }

        // compute it
        if( compute.at(AO_Enabled) ) {
            _vNonAdjacentLinks.at(AO_Enabled).resize(0);
            FOREACHC(itset, _vNonAdjacentLinks[0]) {
                KinBody::LinkConstPtr plink1(_veclinks.at(*itset&0xffff)), plink2(_veclinks.at(*itset>>16));
                if( plink1->IsEnabled() && plink2->IsEnabled() ) {
                    _vNonAdjacentLinks[AO_Enabled].push_back(*itset);
                }
            }
            std::sort(_vNonAdjacentLinks[AO_Enabled].begin(), _vNonAdjacentLinks[AO_Enabled].end(), CompareNonAdjacentFarthest);
        }
        if( compute.at(AO_ActiveDOFs) ) {
            _vNonAdjacentLinks.at(AO_ActiveDOFs).resize(0);
            FOREACHC(itset, _vNonAdjacentLinks[0]) {
                FOREACHC(it, GetActiveDOFIndices()) {
                    if( IsDOFInChain(*itset&0xffff,*itset>>16,*it) ) {
                        _vNonAdjacentLinks[AO_ActiveDOFs].push_back(*itset);
                        break;
                    }
                }
            }
            std::sort(_vNonAdjacentLinks[AO_ActiveDOFs].begin(), _vNonAdjacentLinks[AO_ActiveDOFs].end(), CompareNonAdjacentFarthest);
        }
        if( compute.at(AO_Enabled|AO_ActiveDOFs) ) {
            _vNonAdjacentLinks.at(AO_Enabled|AO_ActiveDOFs).resize(0);
            FOREACHC(itset, _vNonAdjacentLinks[AO_ActiveDOFs]) {
                KinBody::LinkConstPtr plink1(_veclinks.at(*itset&0xffff)), plink2(_veclinks.at(*itset>>16));
                if( plink1->IsEnabled() && plink2->IsEnabled() ) {
                    _vNonAdjacentLinks[AO_Enabled|AO_ActiveDOFs].push_back(*itset);
                }
            }
            std::sort(_vNonAdjacentLinks[AO_Enabled|AO_ActiveDOFs].begin(), _vNonAdjacentLinks[AO_Enabled|AO_ActiveDOFs].end(), CompareNonAdjacentFarthest);
        }
        _nNonAdjacentLinkCache |= requestedoptions;
    }
    return _vNonAdjacentLinks.at(adjacentoptions);
}

void RobotBase::SetNonCollidingConfiguration()
{
    KinBody::SetNonCollidingConfiguration();
    RegrabAll();
}

bool RobotBase::Grab(KinBodyPtr pbody)
{
    ManipulatorPtr pmanip = GetActiveManipulator();
    if( !pmanip ) {
        return false;
    }
    return Grab(pbody, pmanip->GetEndEffector());
}

bool RobotBase::Grab(KinBodyPtr pbody, const std::set<int>& setRobotLinksToIgnore)
{
    ManipulatorPtr pmanip = GetActiveManipulator();
    if( !pmanip ) {
        return false;
    }
    return Grab(pbody, pmanip->GetEndEffector(), setRobotLinksToIgnore);
}

bool RobotBase::Grab(KinBodyPtr body, LinkPtr pRobotLinkToGrabWith)
{
    return KinBody::Grab(body, pRobotLinkToGrabWith);
}

bool RobotBase::Grab(KinBodyPtr body, LinkPtr pRobotLinkToGrabWith, const std::set<int>& setRobotLinksToIgnore)
{
    return KinBody::Grab(body, pRobotLinkToGrabWith, setRobotLinksToIgnore);
}

void RobotBase::SetActiveManipulator(ManipulatorConstPtr pmanip)
{
    if( !pmanip ) {
        _pManipActive.reset();
    }
    else {
        FOREACH(itmanip,_vecManipulators) {
            if( *itmanip == pmanip ) {
                _pManipActive = *itmanip;
                return;
            }
        }
        // manipulator might have been recoreded, search for the same name
        FOREACH(itmanip,_vecManipulators) {
            if( (*itmanip)->GetName() == pmanip->GetName() ) {
                _pManipActive = *itmanip;
                return;
            }
        }

        _pManipActive.reset();
        RAVELOG_WARN_FORMAT("failed to find manipulator with name %s, most likely removed", pmanip->GetName());
    }
}

RobotBase::ManipulatorPtr RobotBase::SetActiveManipulator(const std::string& manipname)
{
    if( manipname.size() > 0 ) {
        FOREACH(itmanip,_vecManipulators) {
            if( (*itmanip)->GetName() == manipname ) {
                _pManipActive = *itmanip;
                return _pManipActive;
            }
        }
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to find manipulator with name: %s"), manipname, ORE_InvalidArguments);
    }
    _pManipActive.reset();
    return _pManipActive;
}

RobotBase::ManipulatorPtr RobotBase::GetActiveManipulator()
{
    return _pManipActive;
}

RobotBase::ManipulatorConstPtr RobotBase::GetActiveManipulator() const
{
    return _pManipActive;
}

RobotBase::ManipulatorPtr RobotBase::AddManipulator(const RobotBase::ManipulatorInfo& manipinfo, bool removeduplicate)
{
    OPENRAVE_ASSERT_OP(manipinfo._name.size(),>,0);
    int iremoveindex = -1;
    for(int imanip = 0; imanip < (int)_vecManipulators.size(); ++imanip) {
        if( _vecManipulators[imanip]->GetName() == manipinfo._name ) {
            if( removeduplicate ) {
                iremoveindex = imanip;
                break;
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("manipulator with name %s already exists"),manipinfo._name,ORE_InvalidArguments);
            }
        }
    }
    ManipulatorPtr newmanip(new Manipulator(shared_robot(),manipinfo));
    newmanip->_ComputeInternalInformation();
    if( iremoveindex >= 0 ) {
        // replace the old one
        _vecManipulators[iremoveindex] = newmanip;
    }
    else {
        _vecManipulators.push_back(newmanip);
    }
    __hashrobotstructure.resize(0);
    return newmanip;
}

bool RobotBase::RemoveManipulator(ManipulatorPtr manip)
{
    if( _pManipActive == manip ) {
        _pManipActive.reset();
    }
    FOREACH(itmanip,_vecManipulators) {
        if( *itmanip == manip ) {
            _vecManipulators.erase(itmanip);
            __hashrobotstructure.resize(0);
            return true;
        }
    }
    return false;
}

RobotBase::AttachedSensorPtr RobotBase::AddAttachedSensor(const RobotBase::AttachedSensorInfo& attachedsensorinfo, bool removeduplicate)
{
    OPENRAVE_ASSERT_OP(attachedsensorinfo._name.size(),>,0);
    int iremoveindex = -1;
    for(int iasensor = 0; iasensor < (int)_vecAttachedSensors.size(); ++iasensor) {
        if( _vecAttachedSensors[iasensor]->GetName() == attachedsensorinfo._name ) {
            if( removeduplicate ) {
                iremoveindex = iasensor;
                break;
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("attached sensor with name %s already exists"),attachedsensorinfo._name,ORE_InvalidArguments);
            }
        }
    }
    AttachedSensorPtr newattachedsensor(new AttachedSensor(shared_robot(),attachedsensorinfo));
//    if( _nHierarchyComputed ) {
//        newattachedsensor->_ComputeInternalInformation();
//    }
    if( iremoveindex >= 0 ) {
        // replace the old one
        _vecAttachedSensors[iremoveindex] = newattachedsensor;
    }
    else {
        _vecAttachedSensors.push_back(newattachedsensor);
    }
    newattachedsensor->UpdateInfo(); // just in case
    __hashrobotstructure.resize(0);
    return newattachedsensor;
}

RobotBase::AttachedSensorPtr RobotBase::GetAttachedSensor(const std::string& name) const
{
    FOREACHC(itsensor, _vecAttachedSensors) {
        if( (*itsensor)->GetName() == name ) {
            return *itsensor;
        }
    }
    return RobotBase::AttachedSensorPtr();
}

bool RobotBase::RemoveAttachedSensor(RobotBase::AttachedSensor &attsensor)
{
    FOREACH(itattsensor,_vecAttachedSensors) {
        if( itattsensor->get() == &attsensor ) {
            _vecAttachedSensors.erase(itattsensor);
            __hashrobotstructure.resize(0);
            return true;
        }
    }
    return false;
}

bool RobotBase::AddGripperInfo(GripperInfoPtr gripperInfo, bool removeduplicate)
{
    if( !gripperInfo ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("Cannot add invalid gripperInfo to robot %s."),GetName(),ORE_InvalidArguments);
    }
    if( gripperInfo->name.size() == 0 ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("Cannot add gripperInfo to robot %s since its name is empty."),GetName(),ORE_InvalidArguments);
    }

    for(int igripper = 0; igripper < (int)_vecGripperInfos.size(); ++igripper) {
        if( _vecGripperInfos[igripper]->name == gripperInfo->name ) {
            if( removeduplicate ) {
                _vecGripperInfos[igripper] = gripperInfo;
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("gripper with name %s already exists"),gripperInfo->name,ORE_InvalidArguments);
            }
        }
    }

    _vecGripperInfos.push_back(gripperInfo);
    return true;
}

bool RobotBase::RemoveGripperInfo(const std::string& name)
{
    for(int igripper = 0; igripper < (int)_vecGripperInfos.size(); ++igripper) {
        if( _vecGripperInfos[igripper]->name == name ) {
            _vecGripperInfos.erase(_vecGripperInfos.begin()+igripper);
            return true;
        }
    }
    return false;
}

RobotBase::GripperInfoPtr RobotBase::GetGripperInfo(const std::string& name) const
{
    FOREACHC(itGripperInfo, _vecGripperInfos) {
        if( (*itGripperInfo)->name == name ) {
            return *itGripperInfo;
        }
    }
    return RobotBase::GripperInfoPtr();
}

void RobotBase::SimulationStep(dReal fElapsedTime)
{
    KinBody::SimulationStep(fElapsedTime);
    _UpdateAttachedSensors();
}

void RobotBase::_ComputeInternalInformation()
{
    _ComputeConnectedBodiesInformation(); // should process the connected bodies in order to get the real resolved links, joints, etc

    KinBody::_ComputeInternalInformation();
    _vAllDOFIndices.resize(GetDOF());
    for(int i = 0; i < GetDOF(); ++i) {
        _vAllDOFIndices[i] = i;
    }

    // make sure the active dof indices in _vActiveDOFIndices are all within the new DOFs
    {
        int iwriteindex = 0;
        int realdof = GetDOF();
        for(int index = 0; index < (int)_vActiveDOFIndices.size(); ++index) {
            if( _vActiveDOFIndices[index] < realdof ) {
                _vActiveDOFIndices[iwriteindex++] = _vActiveDOFIndices[index];
            }
            else {
                RAVELOG_INFO_FORMAT("env=%d, robot '%s' had active dof index %d outside of dof range (%d)", GetEnv()->GetId()%GetName()%_vActiveDOFIndices[index]%realdof);
            }
        }
        _vActiveDOFIndices.resize(iwriteindex);
    }

    _activespec._vgroups.reserve(2);
    _activespec._vgroups.resize(0);
    if( _vAllDOFIndices.size() > 0 ) {
        ConfigurationSpecification::Group group;
        stringstream ss;
        ss << "joint_values " << GetName();
        if( _nActiveDOF >= 0 ) {
            // use _vActiveDOFIndices
            FOREACHC(it,_vActiveDOFIndices) {
                ss << " " << *it;
            }
            group.dof = (int)_vActiveDOFIndices.size();
        }
        else {
            FOREACHC(it,_vAllDOFIndices) {
                ss << " " << *it;
            }
            group.dof = (int)_vAllDOFIndices.size();
        }
        group.name = ss.str();
        group.offset = 0;
        // do not initialize interpolation, since it implies a motion sampling strategy
        _activespec._vgroups.push_back(group);
    }

    int manipindex=0;
    FOREACH(itmanip,_vecManipulators) {
        if( (*itmanip)->_info._name.size() == 0 ) {
            stringstream ss;
            ss << "manip" << manipindex;
            RAVELOG_WARN(str(boost::format("robot %s has a manipulator with no name, setting to %s\n")%GetName()%ss.str()));
            (*itmanip)->_info._name = ss.str();
        }
        (*itmanip)->_ComputeInternalInformation();
        vector<ManipulatorPtr>::iterator itmanip2 = itmanip; ++itmanip2;
        for(; itmanip2 != _vecManipulators.end(); ++itmanip2) {
            if( (*itmanip)->GetName() == (*itmanip2)->GetName() ) {
                RAVELOG_WARN(str(boost::format("robot %s has two manipulators with the same name: %s!\n")%GetName()%(*itmanip)->GetName()));
            }
        }
        manipindex++;
    }
    // set active manipulator to first manipulator
    if( _vecManipulators.size() > 0 ) {
        // preserve active manip when robot is removed and added back to the env
        if (!!_pManipActive) {
            bool bmanipfound = false;
            FOREACHC(itmanip, _vecManipulators) {
                if (*itmanip == _pManipActive) {
                    bmanipfound = true;
                    break;
                }
            }
            if (!bmanipfound) {
                _pManipActive.reset();
            }
        }
        if (!_pManipActive) {
            _pManipActive = _vecManipulators.at(0);
        }
    }
    else {
        _pManipActive.reset();
    }

    int sensorindex=0;
    FOREACH(itsensor,_vecAttachedSensors) {
        if( (*itsensor)->GetName().size() == 0 ) {
            stringstream ss;
            ss << "sensor" << sensorindex;
            RAVELOG_WARN(str(boost::format("robot %s has a sensor with no name, setting to %s\n")%GetName()%ss.str()));
            (*itsensor)->_info._name = ss.str();
        }
        else if( !utils::IsValidName((*itsensor)->GetName()) ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("sensor name \"%s\" is not valid"), (*itsensor)->GetName(), ORE_Failed);
        }
        //(*itsensor)->_ComputeInternalInformation();
        sensorindex++;
    }

    {
        __hashrobotstructure.resize(0);
        FOREACH(itsensor,_vecAttachedSensors) {
            (*itsensor)->__hashstructure.resize(0);
        }
    }

    if( ComputeAABB().extents.lengthsqr3() > 900.0f ) {
        RAVELOG_WARN(str(boost::format("Robot %s span is greater than 30 meaning that it is most likely defined in a unit other than meters. It is highly encouraged to define all OpenRAVE robots in meters since many metrics, database models, and solvers have been specifically optimized for this unit\n")%GetName()));
    }

    if( !GetController() ) {
        RAVELOG_VERBOSE(str(boost::format("no default controller set on robot %s\n")%GetName()));
        std::vector<int> dofindices;
        for(int i = 0; i < GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        SetController(RaveCreateController(GetEnv(), "IdealController"),dofindices,1);
    }

    // reset the power on the sensors
    FOREACH(itsensor,_vecAttachedSensors) {
        SensorBasePtr psensor = (*itsensor)->GetSensor();
        if( !!psensor ) {
            int ispower = psensor->Configure(SensorBase::CC_PowerCheck);
            psensor->Configure(ispower ? SensorBase::CC_PowerOn : SensorBase::CC_PowerOff);
        }
    }
}

void RobotBase::_DeinitializeInternalInformation()
{
    KinBody::_DeinitializeInternalInformation();
    _DeinitializeConnectedBodiesInformation();
}

void RobotBase::_PostprocessChangedParameters(uint32_t parameters)
{
    if( parameters & (Prop_Sensors|Prop_SensorPlacement) ) {
        FOREACH(itsensor,_vecAttachedSensors) {
            (*itsensor)->__hashstructure.resize(0);
        }
    }
    if( parameters & Prop_RobotManipulatorTool ) {
        FOREACH(itmanip,_vecManipulators) {
            (*itmanip)->__hashstructure.resize(0);
            (*itmanip)->__hashkinematicsstructure.resize(0);
        }
    }
    KinBody::_PostprocessChangedParameters(parameters);
}

const std::vector<RobotBase::ManipulatorPtr>& RobotBase::GetManipulators() const
{
    return _vecManipulators;
}

RobotBase::ManipulatorPtr RobotBase::GetManipulator(const std::string& name) const
{
    FOREACHC(itmanip, _vecManipulators) {
        if( (*itmanip)->GetName() == name ) {
            return *itmanip;
        }
    }
    return RobotBase::ManipulatorPtr();
}

void RobotBase::Clone(InterfaceBaseConstPtr preference, int cloningoptions)
{
    KinBody::Clone(preference,cloningoptions);
    RobotBaseConstPtr r = RaveInterfaceConstCast<RobotBase>(preference);
    __hashrobotstructure = r->__hashrobotstructure;
    _vecManipulators.clear();
    _pManipActive.reset();
    FOREACHC(itmanip, r->_vecManipulators) {
        ManipulatorPtr pmanip(new Manipulator(shared_robot(),*itmanip));
        _vecManipulators.push_back(pmanip);
        if( !!r->GetActiveManipulator() && r->GetActiveManipulator()->GetName() == (*itmanip)->GetName() ) {
            _pManipActive = pmanip;
        }
    }

    _vecConnectedBodies.clear();
    FOREACHC(itConnectedBody, r->_vecConnectedBodies) {
        ConnectedBodyPtr pConnectedBody(new ConnectedBody(shared_robot(),**itConnectedBody,cloningoptions));
        _vecConnectedBodies.push_back(pConnectedBody);
    }

    _vecAttachedSensors.clear();
    FOREACHC(itsensor, r->_vecAttachedSensors) {
        _vecAttachedSensors.push_back(AttachedSensorPtr(new AttachedSensor(shared_robot(),**itsensor,cloningoptions)));
    }
    _UpdateAttachedSensors();

    _vecGripperInfos.clear();
    FOREACH(itGripperInfo, r->_vecGripperInfos) {
        _vecGripperInfos.push_back(GripperInfoPtr(new GripperInfo(**itGripperInfo))); // deep copy
    }

    _vActiveDOFIndices = r->_vActiveDOFIndices;
    _activespec = r->_activespec;
    _vAllDOFIndices = r->_vAllDOFIndices;
    vActvAffineRotationAxis = r->vActvAffineRotationAxis;
    _nActiveDOF = r->_nActiveDOF;
    _nAffineDOFs = r->_nAffineDOFs;

    _vTranslationLowerLimits = r->_vTranslationLowerLimits;
    _vTranslationUpperLimits = r->_vTranslationUpperLimits;
    _vTranslationMaxVels = r->_vTranslationMaxVels;
    _vTranslationResolutions = r->_vTranslationResolutions;
    _vRotationAxisLowerLimits = r->_vRotationAxisLowerLimits;
    _vRotationAxisUpperLimits = r->_vRotationAxisUpperLimits;
    _vRotationAxisMaxVels = r->_vRotationAxisMaxVels;
    _vRotationAxisResolutions = r->_vRotationAxisResolutions;
    _vRotation3DLowerLimits = r->_vRotation3DLowerLimits;
    _vRotation3DUpperLimits = r->_vRotation3DUpperLimits;
    _vRotation3DMaxVels = r->_vRotation3DMaxVels;
    _vRotation3DResolutions = r->_vRotation3DResolutions;
    _vRotationQuatLimitStart = r->_vRotationQuatLimitStart;
    _fQuatLimitMaxAngle = r->_fQuatLimitMaxAngle;
    _fQuatMaxAngleVelocity = r->_fQuatMaxAngleVelocity;
    _fQuatAngleResolution = r->_fQuatAngleResolution;
    _fQuatAngleWeight = r->_fQuatAngleWeight;

    // clone the controller
    if( (cloningoptions&Clone_RealControllers) && !!r->GetController() ) {
        if( !SetController(RaveCreateController(GetEnv(), r->GetController()->GetXMLId()),r->GetController()->GetControlDOFIndices(),r->GetController()->IsControlTransformation()) ) {
            RAVELOG_WARN(str(boost::format("failed to set %s controller for robot %s\n")%r->GetController()->GetXMLId()%GetName()));
        }
    }

    if( !GetController() ) {
        std::vector<int> dofindices;
        for(int i = 0; i < GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        if( !SetController(RaveCreateController(GetEnv(), "IdealController"),dofindices, 1) ) {
            RAVELOG_WARN("failed to set IdealController\n");
        }
    }
}

void RobotBase::serialize(std::ostream& o, int options) const
{
    KinBody::serialize(o,options);
    if( options & SO_RobotManipulators ) {
        FOREACHC(itmanip,_vecManipulators) {
            (*itmanip)->serialize(o,options);
        }
    }
    if( options & SO_RobotSensors ) {
        FOREACHC(itsensor,_vecAttachedSensors) {
            (*itsensor)->serialize(o,options);
        }
    }
}

const std::string& RobotBase::GetRobotStructureHash() const
{
    CHECK_INTERNAL_COMPUTATION;
    if( __hashrobotstructure.size() == 0 ) {
        ostringstream ss;
        ss << std::fixed << std::setprecision(SERIALIZATION_PRECISION);
        serialize(ss,SO_Kinematics|SO_Geometry|SO_RobotManipulators|SO_RobotSensors);
        __hashrobotstructure = utils::GetMD5HashString(ss.str());
    }
    return __hashrobotstructure;
}

void RobotBase::ExtractInfo(RobotBaseInfo& info)
{
    KinBody::ExtractInfo(info);

    info._isRobot = true;
    // need to avoid extracting info from connectedbodies
    std::vector<bool> isConnectedManipulator(_vecManipulators.size(), false);
    std::vector<bool> isConnectedAttachedSensor(_vecAttachedSensors.size(), false);
    std::vector<bool> isConnectedGripperInfo(_vecConnectedBodies.size(), false);

    FOREACHC(itConnectedBody, _vecConnectedBodies) {
        std::vector<RobotBase::ManipulatorPtr> resolvedManipulators;
        std::vector<RobotBase::AttachedSensorPtr> resolvedAttachedSensors;
        std::vector<RobotBase::GripperInfoPtr> resolvedGripperInfos;
        (*itConnectedBody)->GetResolvedManipulators(resolvedManipulators);
        (*itConnectedBody)->GetResolvedAttachedSensors(resolvedAttachedSensors);
        (*itConnectedBody)->GetResolvedGripperInfos(resolvedGripperInfos);
        FOREACHC(itManipulator, _vecManipulators) {
            if (std::find(resolvedManipulators.begin(), resolvedManipulators.end(), *itManipulator) != resolvedManipulators.end()) {
                isConnectedManipulator[itManipulator - _vecManipulators.begin()] = true;
            }
        }
        FOREACHC(itAttachedSensor, _vecAttachedSensors) {
            if (std::find(resolvedAttachedSensors.begin(), resolvedAttachedSensors.end(), *itAttachedSensor) != resolvedAttachedSensors.end()) {
                isConnectedAttachedSensor[itAttachedSensor - _vecAttachedSensors.begin()] = true;
            }
        }
        FOREACHC(itGripperInfo, _vecGripperInfos) {
            if (std::find(resolvedGripperInfos.begin(), resolvedGripperInfos.end(), *itGripperInfo) != resolvedGripperInfos.end()) {
                isConnectedGripperInfo[itGripperInfo - _vecGripperInfos.begin()] = true;
            }
        }
    }

    info._vManipulatorInfos.reserve(_vecManipulators.size());
    for(size_t iManipulator = 0; iManipulator < _vecManipulators.size(); ++iManipulator) {
        if (isConnectedManipulator[iManipulator]) {
            continue;
        }
        RobotBase::ManipulatorInfoPtr pManip(new RobotBase::ManipulatorInfo());
        info._vManipulatorInfos.push_back(pManip);
        _vecManipulators[iManipulator]->ExtractInfo(*(info._vManipulatorInfos.back()));
    }

    info._vAttachedSensorInfos.reserve(_vecAttachedSensors.size());
    for(size_t iAttachedSensor = 0; iAttachedSensor < _vecAttachedSensors.size(); ++iAttachedSensor) {
        if (isConnectedAttachedSensor[iAttachedSensor]) {
            continue;
        }
        RobotBase::AttachedSensorInfoPtr pAttachedSensor(new RobotBase::AttachedSensorInfo());
        info._vAttachedSensorInfos.push_back(pAttachedSensor);
        _vecAttachedSensors[iAttachedSensor]->ExtractInfo(*(info._vAttachedSensorInfos.back()));
    }

    info._vConnectedBodyInfos.resize(_vecConnectedBodies.size());
    for(size_t i = 0; i < _vecConnectedBodies.size(); ++i) {
        info._vConnectedBodyInfos[i].reset(new RobotBase::ConnectedBodyInfo());
        _vecConnectedBodies[i]->ExtractInfo(*info._vConnectedBodyInfos[i]);
    }

    info._vGripperInfos.reserve(_vecGripperInfos.size());
    for(size_t i = 0; i < _vecGripperInfos.size(); ++i) {
        if (isConnectedGripperInfo[i]) {
            continue;
        }
        RobotBase::GripperInfoPtr pGripperInfo(new RobotBase::GripperInfo(*_vecGripperInfos[i]));
        info._vGripperInfos.push_back(pGripperInfo);
    }
}

UpdateFromInfoResult RobotBase::UpdateFromRobotInfo(const RobotBaseInfo& info)
{
    UpdateFromInfoResult updateFromInfoResult = UpdateFromKinBodyInfo(info);
    if (updateFromInfoResult != UFIR_Success) {
        return updateFromInfoResult;
    }

    // manipulators
    FOREACHC(itManipulatorInfo, info._vManipulatorInfos) {
        // find existing manipulator in robot
        std::vector<RobotBase::ManipulatorPtr>::iterator itExistingManipulator = _vecManipulators.end();
        FOREACHC(itManipulator, _vecManipulators) {
            if ((*itManipulator)->_info._id == (*itManipulatorInfo)->_id) {
                itExistingManipulator = itManipulator;
                break;
            }
        }
        RobotBase::ManipulatorInfoPtr pManipulatorInfo = *itManipulatorInfo;
        if (itExistingManipulator != _vecManipulators.end()) {
            // update existing manipulator
            UpdateFromInfoResult updateFromManipulatorInfoResult = UFIR_Success;
            RobotBase::ManipulatorPtr pManipulator = *itExistingManipulator;
            updateFromManipulatorInfoResult = pManipulator->UpdateFromInfo(*pManipulatorInfo);

            if (updateFromManipulatorInfoResult == UFIR_Success) {
                continue;
            }
            // manipulator update failed;
            return updateFromManipulatorInfoResult;
        }
        return UFIR_RequireRemoveFromEnvironment;
    }

    // attachedsensors
    FOREACHC(itAttachedSensorInfo, info._vAttachedSensorInfos) {
        // find existing attachedsensor in robot
        std::vector<RobotBase::AttachedSensorPtr>::iterator itExistingAttachedSensor = _vecAttachedSensors.end();
        FOREACHC(itAttachedSensor, _vecAttachedSensors) {
            if ((*itAttachedSensor)->_info._id == (*itAttachedSensorInfo)->_id) {
                itExistingAttachedSensor = itAttachedSensor;
                break;
            }
        }
        RobotBase::AttachedSensorInfoPtr pAttachedSensorInfo = *itAttachedSensorInfo;
        if (itExistingAttachedSensor != _vecAttachedSensors.end()) {
            // update existing attachedsensor
            UpdateFromInfoResult updateFromAttachedSensorInfoResult = UFIR_Success;
            RobotBase::AttachedSensorPtr pAttachedSensor = *itExistingAttachedSensor;
            updateFromAttachedSensorInfoResult = pAttachedSensor->UpdateFromInfo(*pAttachedSensorInfo);
            if (updateFromAttachedSensorInfoResult == UFIR_Success) {
                continue;
            }
            // attachedsensor update failed;
            return updateFromAttachedSensorInfoResult;
        }
        return UFIR_RequireRemoveFromEnvironment;
    }

    // connectedbodies
    FOREACHC(itConnectedBodyInfo, info._vConnectedBodyInfos) {
        // find existing connectedbody in robot
        std::vector<RobotBase::ConnectedBodyPtr>::iterator itExistingConnectedBody = _vecConnectedBodies.end();
        FOREACHC(itConnectedBody, _vecConnectedBodies) {
            if ((*itConnectedBody)->_info._id == (*itConnectedBodyInfo)->_id) {
                itExistingConnectedBody = itConnectedBody;
                break;
            }
        }
        RobotBase::ConnectedBodyInfoPtr pConnectedBodyInfo = *itConnectedBodyInfo;
        if (itExistingConnectedBody != _vecConnectedBodies.end()) {
            // update existing connectedbody
            UpdateFromInfoResult updateFromConnectedBodyInfoResult = UFIR_Success;
            RobotBase::ConnectedBodyPtr pConnectedBody = *itExistingConnectedBody;
            updateFromConnectedBodyInfoResult = pConnectedBody->UpdateFromInfo(*pConnectedBodyInfo);
            if (updateFromConnectedBodyInfoResult == UFIR_Success) {
                continue;
            }
            // connectedbody update failed;
            return updateFromConnectedBodyInfoResult;
        }
        return UFIR_RequireRemoveFromEnvironment;
    }


    // gripperinfos
    FOREACHC(itGripperInfo, info._vGripperInfos) {
        // find exisiting gripperinfo in robot
        typename std::vector<RobotBase::GripperInfoPtr>::iterator itExistingGripperInfo;
        for(itExistingGripperInfo = _vecGripperInfos.begin(); itExistingGripperInfo != _vecGripperInfos.end(); itExistingGripperInfo++) {
            if ((*itExistingGripperInfo)->_id == (*itGripperInfo)->_id) {
                // find existing gripperinfo
                if ((*itExistingGripperInfo) != (*itGripperInfo)) {
                    return UFIR_RequireReinitialize;
                }
                break;
            }
        }
        if (itExistingGripperInfo == _vecGripperInfos.end()) {
            // new gripper info
            return UFIR_RequireReinitialize;
        }
    }

    return UFIR_Success;
}

} // end namespace OpenRAVE
