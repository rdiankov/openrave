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
#include <algorithm>
#include <boost/algorithm/string.hpp> // boost::trim
#include <boost/lexical_cast.hpp>

#include "fparsermulti.h"


namespace OpenRAVE {

int KinBody::JointInfo::GetDOF() const
{
    if(_type & KinBody::JointSpecialBit) {
        switch(_type) {
        case KinBody::JointHinge2:
        case KinBody::JointUniversal: return 2;
        case KinBody::JointSpherical: return 3;
        case KinBody::JointTrajectory: return 1;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("invalid joint type 0x%x"), _type, ORE_Failed);
        }
    }
    return int(_type & 0xf);
}

void KinBody::JointInfo::Reset()
{
    _type = JointNone;
    _id.clear();
    _name.clear();
    _linkname0.clear();
    _linkname1.clear();
    _vanchor = Vector();
    _vaxes = {Vector(0,0,1), Vector(0,0,1), Vector(0,0,1)};
    _vcurrentvalues.clear();
    _vresolution = {0.02, 0.02, 0.02};
    _vmaxvel = {10, 10, 10};
    _vhardmaxvel = {0, 0, 0};
    _vmaxaccel = {50, 50, 50};
    _vhardmaxaccel = {0, 0, 0};
    _vmaxjerk = {50*1000, 50*1000, 50*1000};
    _vhardmaxjerk = {0, 0, 0};
    _vmaxtorque = {0, 0, 0};
    _vmaxinertia = {0, 0, 0};
    _vweights = {1, 1, 1};
    _voffsets = {0, 0, 0};
    _vlowerlimit = {0, 0, 0};
    _vupperlimit = {0, 0, 0};
    _trajfollow.reset();
    FOREACH(itmimic, _vmimic) {
        itmimic->reset();
    }

    _mapFloatParameters.clear();
    _mapIntParameters.clear();
    _mapStringParameters.clear();
    _infoElectricMotor.reset();
    _bIsCircular = {0, 0, 0};
    _bIsActive = true;
    _controlMode = JCM_None;
    _jci_robotcontroller.reset();
    _jci_io.reset();
    _jci_externaldevice.reset();
}

void KinBody::JointInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    int dof = GetDOF();

    switch (_type) {
    case JointRevolute: // jointHinge and jointRevolute have same value
        orjson::SetJsonValueByKey(value, "type", "revolute", allocator);
        break;
    case JointPrismatic: // jointSlider and jointPrismatic have same value
        orjson::SetJsonValueByKey(value, "type", "prismatic", allocator);
        break;
    case JointRR:
        orjson::SetJsonValueByKey(value, "type", "rr", allocator);
        break;
    case JointRP:
        orjson::SetJsonValueByKey(value, "type", "rp", allocator);
        break;
    case JointPR:
        orjson::SetJsonValueByKey(value, "type", "pr", allocator);
        break;
    case JointPP:
        orjson::SetJsonValueByKey(value, "type", "pp", allocator);
        break;
    case JointSpecialBit:
        orjson::SetJsonValueByKey(value, "type", "specialbit", allocator);
        break;
    case JointUniversal:
        orjson::SetJsonValueByKey(value, "type", "universal", allocator);
        break;
    case JointHinge2:
        orjson::SetJsonValueByKey(value, "type", "hinge2", allocator);
        break;
    case JointSpherical:
        orjson::SetJsonValueByKey(value, "type", "spherical", allocator);
        break;
    case JointTrajectory:
        orjson::SetJsonValueByKey(value, "type", "trajectory", allocator);
        break;
    case JointNone:
        break;
    default:
        RAVELOG_WARN(str(boost::format("Unknow JointType %d")%static_cast<int>(_type)));
        orjson::SetJsonValueByKey(value, "type", static_cast<int>(_type), allocator);  // TODO: should we raise error here ?
        break;
    }

    dReal fjointmult = fUnitScale;
    if(_type == JointRevolute)
    {
        fjointmult = 1;
    }
    else if(_type == JointPrismatic)
    {
        fjointmult = fUnitScale;
    }

    orjson::SetJsonValueByKey(value, "id", _id, allocator);
    orjson::SetJsonValueByKey(value, "name", _name, allocator);
    orjson::SetJsonValueByKey(value, "anchors", _vanchor, allocator);
    orjson::SetJsonValueByKey(value, "parentLinkName", _linkname0, allocator);
    orjson::SetJsonValueByKey(value, "childLinkName", _linkname1, allocator);
    orjson::SetJsonValueByKey(value, "axes", _vaxes, allocator);
    orjson::SetJsonValueByKey(value, "currentValues", _vcurrentvalues, allocator);
    orjson::SetJsonValueByKey(value, "resolutions", _vresolution, allocator, dof);

    boost::array<dReal, 3> newvmaxvel = _vmaxvel;
    boost::array<dReal, 3> newvmaxaccel = _vmaxaccel;
    boost::array<dReal, 3> newvlowerlimit = _vlowerlimit;
    boost::array<dReal, 3> newvupperlimit = _vupperlimit;
    for(size_t i = 0; i < 3; i++) {
        newvmaxvel[i] *= fjointmult;
        newvmaxaccel[i] *= fjointmult;
        newvlowerlimit[i] *= fjointmult;
        newvupperlimit[i] *= fjointmult;
    }
    orjson::SetJsonValueByKey(value, "maxVel", newvmaxvel, allocator, dof);
    orjson::SetJsonValueByKey(value, "hardMaxVel", _vhardmaxvel, allocator, dof);
    orjson::SetJsonValueByKey(value, "maxAccel", newvmaxaccel, allocator, dof);
    orjson::SetJsonValueByKey(value, "hardMaxAccel", _vhardmaxaccel, allocator, dof);
    orjson::SetJsonValueByKey(value, "maxJerk", _vmaxjerk, allocator, dof);
    orjson::SetJsonValueByKey(value, "hardMaxJerk", _vhardmaxjerk, allocator, dof);
    orjson::SetJsonValueByKey(value, "maxTorque", _vmaxtorque, allocator, dof);
    orjson::SetJsonValueByKey(value, "maxInertia", _vmaxinertia, allocator, dof);
    orjson::SetJsonValueByKey(value, "weights", _vweights, allocator, dof);
    orjson::SetJsonValueByKey(value, "offsets", _voffsets, allocator, dof);
    orjson::SetJsonValueByKey(value, "lowerLimit", newvlowerlimit, allocator, dof);
    orjson::SetJsonValueByKey(value, "upperLimit", newvupperlimit, allocator, dof);
    // TODO: orjson::SetJsonValueByKey(value, allocator, "trajfollow", _trajfollow);

    if (_vmimic.size() > 0) {
        bool bfound = false;
        for (size_t i = 0; i < _vmimic.size() && i < (size_t)dof; ++i) {
            if (!!_vmimic[i]) {
                bfound = true;
                break;
            }
        }
        if (bfound) {
            rapidjson::Value mimics;
            mimics.SetArray();
            for (size_t i = 0; i < _vmimic.size() && i < (size_t)dof; ++i) {
                rapidjson::Value mimicValue;
                _vmimic[i]->SerializeJSON(mimicValue, allocator, fUnitScale, options);
                mimics.PushBack(mimicValue, allocator);
            }
            value.AddMember("mimics", mimics, allocator);
        }
    }

    if(_mapFloatParameters.size() > 0)
    {
        rapidjson::Value parameters;
        parameters.SetArray();
        FOREACHC(it, _mapFloatParameters) {
            rapidjson::Value parameter;
            parameter.SetObject();
            orjson::SetJsonValueByKey(parameter, "id", it->first, allocator);
            orjson::SetJsonValueByKey(parameter, "values", it->second, allocator);
            parameters.PushBack(parameter, allocator);
        }
        value.AddMember("floatParameters", parameters, allocator);
    }
    if(_mapIntParameters.size() > 0)
    {
        rapidjson::Value parameters;
        parameters.SetArray();
        FOREACHC(it, _mapIntParameters) {
            rapidjson::Value parameter;
            parameter.SetObject();
            orjson::SetJsonValueByKey(parameter, "id", it->first, allocator);
            orjson::SetJsonValueByKey(parameter, "values", it->second, allocator);
            parameters.PushBack(parameter, allocator);
        }
        value.AddMember("intParameters", parameters, allocator);
    }
    if(_mapStringParameters.size() > 0)
    {
        rapidjson::Value parameters;
        parameters.SetArray();
        FOREACHC(it, _mapStringParameters) {
            rapidjson::Value parameter;
            parameter.SetObject();
            orjson::SetJsonValueByKey(parameter, "id", it->first, allocator);
            orjson::SetJsonValueByKey(parameter, "value", it->second, allocator);
            parameters.PushBack(parameter, allocator);
        }
        value.AddMember("stringParameters", parameters, allocator);
    }

    if (!!_infoElectricMotor) {
        rapidjson::Value electricMotorInfoValue;
        electricMotorInfoValue.SetObject();
        _infoElectricMotor->SerializeJSON(electricMotorInfoValue, allocator, fUnitScale, options);
        value.AddMember("electricMotorActuator", electricMotorInfoValue, allocator);
    }

    orjson::SetJsonValueByKey(value, "isCircular", _bIsCircular, allocator, dof);
    orjson::SetJsonValueByKey(value, "isActive", _bIsActive, allocator);

}

void KinBody::JointInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    // two cases to update type:
    // 1. empty joininfo with type equals to JointNone
    // 2. non-empty jointInfo with non-empty typestr partial update
    if (value.HasMember("type") || _type == JointType::JointNone) {
        std::string typestr;
        orjson::LoadJsonValueByKey(value, "type", typestr);
        std::map<std::string, KinBody::JointType> jointTypeMapping = {
            {"revolute", JointType::JointRevolute},
            {"prismatic", JointType::JointPrismatic},
            {"rr", JointType::JointRR},
            {"rp", JointType::JointRP},
            {"pr", JointType::JointPR},
            {"pp", JointType::JointPP},
            {"specialbit", JointType::JointSpecialBit},
            {"universal", JointType::JointUniversal},
            {"hinge2", JointType::JointHinge2},
            {"spherical", JointType::JointSpherical},
            {"trajectory", JointType::JointTrajectory},
            // {"", JointType::JointNone}  //  TODO: do we allow empty?
        };

        if (jointTypeMapping.find(typestr) == jointTypeMapping.end()) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed to deserialize json, unsupported joint type \"%s\"", typestr, ORE_InvalidArguments);
        }
        _type = jointTypeMapping[typestr];
    }

    // deserializing partial json only multply fUnitScale if value exists
    // multiply fUnitScale on maxVel, maxAccel, lowerLimit, upperLimit
    dReal fjointmult = fUnitScale;
    if(_type == JointRevolute) {
        fjointmult = 1;
    }
    else if(_type == JointPrismatic) {
        fjointmult = fUnitScale;
    }

    orjson::LoadJsonValueByKey(value, "name", _name);
    orjson::LoadJsonValueByKey(value, "id", _id);
    if( _id.empty() ) {
        _id = _name;
    }

    orjson::LoadJsonValueByKey(value, "parentLinkName", _linkname0);
    orjson::LoadJsonValueByKey(value, "anchors", _vanchor);
    orjson::LoadJsonValueByKey(value, "childLinkName", _linkname1);
    orjson::LoadJsonValueByKey(value, "axes", _vaxes);
    orjson::LoadJsonValueByKey(value, "currentValues", _vcurrentvalues);
    orjson::LoadJsonValueByKey(value, "resolutions", _vresolution);

    if (value.HasMember("maxVel")) {
        orjson::LoadJsonValueByKey(value, "maxVel", _vmaxvel);
        for(size_t ic = 0; ic < _vaxes.size(); ic++) {
            _vmaxvel[ic] *= fjointmult;
        }
    }
    orjson::LoadJsonValueByKey(value, "hardMaxVel", _vhardmaxvel);
    if (value.HasMember("maxAccel")) {
        orjson::LoadJsonValueByKey(value, "maxAccel", _vmaxaccel);
        for(size_t ic = 0; ic < _vaxes.size(); ic++) {
            _vmaxaccel[ic] *= fjointmult;
        }
    }
    orjson::LoadJsonValueByKey(value, "hardMaxAccel", _vhardmaxaccel);
    orjson::LoadJsonValueByKey(value, "maxJerk", _vmaxjerk);
    orjson::LoadJsonValueByKey(value, "hardMaxJerk", _vhardmaxjerk);
    orjson::LoadJsonValueByKey(value, "maxTorque", _vmaxtorque);
    orjson::LoadJsonValueByKey(value, "maxInertia", _vmaxinertia);
    orjson::LoadJsonValueByKey(value, "weights", _vweights);
    orjson::LoadJsonValueByKey(value, "offsets", _voffsets);

    if (value.HasMember("lowerLimit")) {
        orjson::LoadJsonValueByKey(value, "lowerLimit", _vlowerlimit);
        for(size_t ic = 0; ic < _vaxes.size(); ic++) {
            _vlowerlimit[ic] *= fjointmult;
        }
    }
    if (value.HasMember("upperLimit")) {
        orjson::LoadJsonValueByKey(value, "upperLimit", _vupperlimit);
        for(size_t ic = 0; ic < _vaxes.size(); ic++) {
            _vupperlimit[ic] *= fjointmult;
        }
    }
    orjson::LoadJsonValueByKey(value, "isCircular", _bIsCircular);
    orjson::LoadJsonValueByKey(value, "isActive", _bIsActive);


    if (value.HasMember("mimics") && value["mimics"].IsArray())
    {
        boost::array<MimicInfoPtr, 3> newmimic;
        for (rapidjson::SizeType i = 0; i < value["mimics"].Size(); ++i) {
            MimicInfoPtr mimicinfo(new MimicInfo());
            mimicinfo->DeserializeJSON(value["mimics"][i], fUnitScale, options);
            newmimic[i] = mimicinfo;
        }
        _vmimic = newmimic;
    }

    if (value.HasMember("floatParameters") && value["floatParameters"].IsArray()) {
        _mapFloatParameters.clear();
        for (rapidjson::Value::ConstValueIterator it = value["floatParameters"].Begin(); it != value["floatParameters"].End(); ++it) {
            std::string key;
            if( it->HasMember("id") ) {
                orjson::LoadJsonValueByKey(*it, "id", key);
            }
            else if( it->HasMember("key") ) {
                // backward compatibility
                orjson::LoadJsonValueByKey(*it, "key", key);
            }
            if (key.empty()) {
                RAVELOG_WARN_FORMAT("ignored an entry in floatParameters in joint %s due to missing or empty id", _id);
                continue;
            }
            orjson::LoadJsonValueByKey(*it, "values", _mapFloatParameters[key]);
        }
    }
    if (value.HasMember("intParameters") && value["intParameters"].IsArray()) {
        _mapIntParameters.clear();
        for (rapidjson::Value::ConstValueIterator it = value["intParameters"].Begin(); it != value["intParameters"].End(); ++it) {
            std::string key;
            if( it->HasMember("id") ) {
                orjson::LoadJsonValueByKey(*it, "id", key);
            }
            else if( it->HasMember("key") ) {
                // backward compatibility
                orjson::LoadJsonValueByKey(*it, "key", key);
            }
            if (key.empty()) {
                RAVELOG_WARN_FORMAT("ignored an entry in intParameters in joint %s due to missing or empty id", _id);
                continue;
            }
            orjson::LoadJsonValueByKey(*it, "values", _mapIntParameters[key]);
        }
    }
    if (value.HasMember("stringParameters") && value["stringParameters"].IsArray()) {
        _mapStringParameters.clear();
        for (rapidjson::Value::ConstValueIterator it = value["stringParameters"].Begin(); it != value["stringParameters"].End(); ++it) {
            std::string key;
            if( it->HasMember("id") ) {
                orjson::LoadJsonValueByKey(*it, "id", key);
            }
            else if( it->HasMember("key") ) {
                // backward compatibility
                orjson::LoadJsonValueByKey(*it, "key", key);
            }
            if (key.empty()) {
                RAVELOG_WARN_FORMAT("ignored an entry in stringParameters in joint %s due to missing or empty id", _id);
                continue;
            }
            orjson::LoadJsonValueByKey(*it, "value", _mapStringParameters[key]);
        }
    }

    if (value.HasMember("electricMotorActuator")) {
        if (!_infoElectricMotor) {
            _infoElectricMotor.reset(new ElectricMotorActuatorInfo());
        }
        _infoElectricMotor->DeserializeJSON(value["electricMotorActuator"], fUnitScale, options);
    }
}

bool KinBody::JointInfo::operator==(const KinBody::JointInfo& other) const
{
    return _type == other._type
           && _id == other._id
           && _name == other._name
           && _linkname0 == other._linkname0
           && _linkname1 == other._linkname1
           && _vanchor == other._vanchor
           && _vaxes == other._vaxes
           && _vcurrentvalues == other._vcurrentvalues
           && _vresolution == other._vresolution
           && _vmaxvel == other._vmaxvel
           && _vhardmaxvel == other._vhardmaxvel
           && _vmaxaccel == other._vmaxaccel
           && _vhardmaxaccel == other._vhardmaxaccel
           && _vmaxjerk == other._vmaxjerk
           && _vhardmaxjerk == other._vhardmaxjerk
           && _vmaxtorque == other._vmaxtorque
           && _vmaxinertia == other._vmaxinertia
           && _vweights == other._vweights
           && _voffsets == other._voffsets
           && _vlowerlimit == other._vlowerlimit
           && _vupperlimit == other._vupperlimit
           && _trajfollow == other._trajfollow
           && _vmimic == other._vmimic
           && _mapFloatParameters == other._mapFloatParameters
           && _mapIntParameters == other._mapIntParameters
           && _mapStringParameters == other._mapStringParameters
           && _infoElectricMotor == other._infoElectricMotor
           && _bIsCircular == other._bIsCircular
           && _bIsActive == other._bIsActive
           && _controlMode == other._controlMode
           && _jci_robotcontroller == other._jci_robotcontroller
           && _jci_io == other._jci_io
           && _jci_externaldevice == other._jci_externaldevice;
}

KinBody::JointInfo& KinBody::JointInfo::operator=(const KinBody::JointInfo& other)
{
    _id = other._id;
    _type = other._type;
    _name = other._name;
    _linkname0 = other._linkname0;
    _linkname1 = other._linkname1;
    _vanchor = other._vanchor;
    _vaxes = other._vaxes;
    _vcurrentvalues = other._vcurrentvalues;
    _vresolution = other._vresolution;
    _vmaxvel = other._vmaxvel;
    _vhardmaxvel = other._vhardmaxvel;
    _vmaxaccel = other._vmaxaccel;
    _vhardmaxaccel = other._vhardmaxaccel;
    _vmaxjerk = other._vmaxjerk;
    _vhardmaxjerk = other._vhardmaxjerk;
    _vmaxtorque = other._vmaxtorque;
    _vmaxinertia = other._vmaxinertia;
    _vweights = other._vweights;
    _voffsets = other._voffsets;
    _vlowerlimit = other._vlowerlimit;
    _vupperlimit = other._vupperlimit;

    if( !other._trajfollow ) {
        _trajfollow.reset();
    }
    else {
        _trajfollow = RaveClone<TrajectoryBase>(other._trajfollow, Clone_All);
    }

    for( size_t i = 0; i < _vmimic.size(); ++i ) {
        if( !other._vmimic[i] ) {
            _vmimic[i].reset();
        }
        else {
            _vmimic[i].reset(new MimicInfo(*(other._vmimic[i])));
        }
    }

    _mapFloatParameters = other._mapFloatParameters;
    _mapIntParameters = other._mapIntParameters;
    _mapStringParameters = other._mapStringParameters;

    if( !other._infoElectricMotor ) {
        _infoElectricMotor.reset();
    }
    else {
        _infoElectricMotor.reset(new ElectricMotorActuatorInfo(*other._infoElectricMotor));
    }

    _bIsCircular = other._bIsCircular;
    _bIsActive = other._bIsActive;

    _controlMode = other._controlMode;
    _jci_robotcontroller.reset();
    _jci_io.reset();
    _jci_externaldevice.reset();
    if( _controlMode == KinBody::JCM_RobotController ) {
        if( !other._jci_robotcontroller ) {
            _jci_robotcontroller.reset();
        }
        else {
            _jci_robotcontroller.reset(new JointControlInfo_RobotController(*other._jci_robotcontroller));
        }
    }
    else if( _controlMode == KinBody::JCM_IO ) {
        if( !other._jci_io ) {
            _jci_io.reset();
        }
        else {
            _jci_io.reset(new JointControlInfo_IO(*other._jci_io));
        }
    }
    else if( _controlMode == KinBody::JCM_ExternalDevice ) {
        if( !other._jci_externaldevice ) {
            _jci_externaldevice.reset();
        }
        else {
            _jci_externaldevice.reset(new JointControlInfo_ExternalDevice(*other._jci_externaldevice));
        }
    }
    return *this;
}

static void fparser_polyroots2(vector<dReal>& rawroots, const vector<dReal>& rawcoeffs)
{
    BOOST_ASSERT(rawcoeffs.size()==3);
    int numroots=0;
    rawroots.resize(2);
    polyroots2<dReal>(&rawcoeffs[0],&rawroots[0],numroots);
    rawroots.resize(numroots);
}

template <int D>
static void fparser_polyroots(vector<dReal>& rawroots, const vector<dReal>& rawcoeffs)
{
    BOOST_ASSERT(rawcoeffs.size()==D+1);
    int numroots=0;
    rawroots.resize(D);
    polyroots<dReal,D>(&rawcoeffs[0],&rawroots[0],numroots);
    rawroots.resize(numroots);
}

// take triangle 3 sides and compute the angle opposite the first side
static void fparser_sssa(std::vector<dReal>& res, const vector<dReal>& coeffs)
{
    dReal a = coeffs.at(0), b = coeffs.at(1), c = coeffs.at(2);
    dReal f = (a*a+b*b-c*c)/(2*b);
    res.resize(1);
    res[0] = RaveAtan2(RaveSqrt(a*a-f*f),b-f);
}

/// take triangle 2 sides and an angle and compute the missing angle
static void fparser_sasa(std::vector<dReal>& res, const vector<dReal>& coeffs)
{
    dReal a = coeffs[0], gamma = coeffs[1], b = coeffs[2];
    res.resize(1);
    res[0] = RaveAtan2(a*RaveSin(gamma),b-a*RaveCos(gamma));
}

/// take triangle 2 sides and an angle and compute the missing side
static void fparser_sass(std::vector<dReal>& res, const vector<dReal>& coeffs)
{
    dReal a = coeffs[0], gamma = coeffs[1], b = coeffs[2];
    res.resize(1);
    res[0] = RaveSqrt(a*a+b*b-2*a*b*RaveCos(gamma));
}

OpenRAVEFunctionParserRealPtr CreateJointFunctionParser()
{
    OpenRAVEFunctionParserRealPtr parser(new OpenRAVEFunctionParserReal());
#ifdef OPENRAVE_FPARSER_SETEPSILON
    parser->setEpsilon(g_fEpsilonLinear);
#endif
    // register commonly used functions
    parser->AddBoostFunction("polyroots2",fparser_polyroots2,3);
    parser->AddBoostFunction("polyroots3",fparser_polyroots<3>,4);
    parser->AddBoostFunction("polyroots4",fparser_polyroots<4>,5);
    parser->AddBoostFunction("polyroots5",fparser_polyroots<5>,6);
    parser->AddBoostFunction("polyroots6",fparser_polyroots<6>,7);
    parser->AddBoostFunction("polyroots7",fparser_polyroots<7>,8);
    parser->AddBoostFunction("polyroots8",fparser_polyroots<8>,9);
    parser->AddBoostFunction("SSSA",fparser_sssa,3);
    parser->AddBoostFunction("SASA",fparser_sasa,3);
    parser->AddBoostFunction("SASS",fparser_sass,3);
    return parser;
}

KinBody::Joint::Joint(KinBodyPtr parent, KinBody::JointType type)
{
    _parent = parent;
    FOREACH(it,_doflastsetvalues) {
        *it = 0;
    }
    for(size_t i = 0; i < _vaxes.size(); ++i) {
        _vaxes[i] = Vector(0,0,1);
    }
    jointindex=-1;
    dofindex = -1; // invalid index
    _bInitialized = false;
    _nIsStatic = -1;
    _info._type = type;
    _info._controlMode = JCM_None;
}

KinBody::Joint::~Joint()
{
}

int KinBody::Joint::GetDOF() const
{
    return _info.GetDOF();
}

bool KinBody::Joint::IsCircular() const
{
    return _info._bIsCircular[0] || _info._bIsCircular[1] || _info._bIsCircular[2];
}

bool KinBody::Joint::IsCircular(int iaxis) const
{
    return static_cast<bool>(_info._bIsCircular.at(iaxis));
}

bool KinBody::Joint::IsActive() const
{
    return _info._bIsActive;
}

bool KinBody::Joint::IsRevolute(int iaxis) const
{
    if( _info._type & KinBody::JointSpecialBit ) {
        return _info._type == KinBody::JointHinge2 || _info._type == KinBody::JointUniversal;
    }
    return !(_info._type&(1<<(4+iaxis)));
}

bool KinBody::Joint::IsPrismatic(int iaxis) const
{
    if( _info._type & KinBody::JointSpecialBit ) {
        return false;
    }
    return !!(_info._type&(1<<(4+iaxis)));
}

bool KinBody::Joint::IsStatic() const
{
    if(_nIsStatic != -1) {
        return _nIsStatic==1;
    }

    if( IsMimic() ) {
        bool bstatic = _bInitialized && _nIsStatic != -1; // if not _bInitialized, then most likely do not know if dependent joints are static, so always return false
        KinBodyConstPtr parent(_parent);
        for(int i = 0; i < GetDOF(); ++i) {
            if( !!_vmimic.at(i) ) {
                FOREACHC(it, _vmimic.at(i)->_vmimicdofs) {
                    if( !parent->GetJointFromDOFIndex(it->dofindex)->IsStatic() ) {
                        bstatic = false;
                        break;
                    }
                }
                if( !bstatic ) {
                    break;
                }
            }
        }
        if( bstatic ) {
            return true;
        }
    }
    for(int i = 0; i < GetDOF(); ++i) {
        if( IsCircular(i) ) {
            return false;
        }
        if( _info._vlowerlimit.at(i) < _info._vupperlimit.at(i) ) {
            return false;
        }
    }
    return true;
}

void KinBody::Joint::GetValues(std::vector<dReal>& pValues, bool bAppend) const
{
    OPENRAVE_ASSERT_FORMAT0(_bInitialized, "joint not initialized",ORE_NotInitialized);
    if( !bAppend ) {
        pValues.clear();
    }
    if( GetDOF() == 1 ) {
        pValues.push_back(GetValue(0));
        return;
    }
    dReal f;
    Transform tjoint = _tinvLeft * _attachedbodies[0]->GetTransform().inverse() * _attachedbodies[1]->GetTransform() * _tinvRight;
    if( _info._type & KinBody::JointSpecialBit ) {
        switch(_info._type) {
        case KinBody::JointHinge2: {
            Vector axis1cur = tjoint.rotate(_vaxes[0]), axis2cur = tjoint.rotate(_vaxes[1]);
            Vector vec1, vec2, vec3;
            vec1 = (_vaxes[1] - _vaxes[0].dot3(_vaxes[1])*_vaxes[0]).normalize();
            vec2 = (axis2cur - _vaxes[0].dot3(axis2cur)*_vaxes[0]).normalize();
            vec3 = _vaxes[0].cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot3(vec2), vec1.dot3(vec2));
            pValues.push_back(GetClosestValueAlongCircle(_info._voffsets[0]+f, _doflastsetvalues[0]));
            vec1 = (_vaxes[0] - axis2cur.dot(_vaxes[0])*axis2cur).normalize();
            vec2 = (axis1cur - axis2cur.dot(axis1cur)*axis2cur).normalize();
            vec3 = axis2cur.cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot(vec2), vec1.dot(vec2));
            if( f < -PI ) {
                f += 2*PI;
            }
            else if( f > PI ) {
                f -= 2*PI;
            }
            pValues.push_back(GetClosestValueAlongCircle(_info._voffsets[1]+f, _doflastsetvalues[1]));
            break;
        }
        case KinBody::JointSpherical: {
            dReal fsinang2 = tjoint.rot.y*tjoint.rot.y+tjoint.rot.z*tjoint.rot.z+tjoint.rot.w*tjoint.rot.w;
            if( fsinang2 > 1e-10f ) {
                dReal fsinang = RaveSqrt(fsinang2);
                dReal fmult = 2*RaveAtan2(fsinang,tjoint.rot.x)/fsinang;
                pValues.push_back(tjoint.rot.y*fmult);
                pValues.push_back(tjoint.rot.z*fmult);
                pValues.push_back(tjoint.rot.w*fmult);
            }
            else {
                pValues.push_back(0);
                pValues.push_back(0);
                pValues.push_back(0);
            }
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("unknown joint type 0x%x"), _info._type, ORE_Failed);
        }
    }
    else {
        // chain of revolute and prismatic joints
        for(int i = 0; i < GetDOF(); ++i) {
            Vector vaxis = _vaxes.at(i);
            if( IsRevolute(i) ) {
                if( i+1 < GetDOF() ) {
                    std::pair<dReal, Vector > res = normalizeAxisRotation(vaxis,tjoint.rot);
                    tjoint.rot = res.second;
                    if( res.first != 0 ) {
                        // could speed up by checking if trans is ever needed after this
                        tjoint.trans = quatRotate(quatFromAxisAngle(vaxis,res.first),tjoint.trans);
                    }
                    f = -res.first;
                }
                else {
                    f = 2.0f*RaveAtan2(tjoint.rot.y*vaxis.x+tjoint.rot.z*vaxis.y+tjoint.rot.w*vaxis.z, tjoint.rot.x);
                }
                // expect values to be within -PI to PI range
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                pValues.push_back(GetClosestValueAlongCircle(_info._voffsets[i]+f, _doflastsetvalues[i]));
            }
            else { // prismatic
                f = tjoint.trans.x*vaxis.x+tjoint.trans.y*vaxis.y+tjoint.trans.z*vaxis.z;
                pValues.push_back(_info._voffsets[i]+f);
                if( i+1 < GetDOF() ) {
                    tjoint.trans -= vaxis*f;
                }
            }
        }
    }
}

void KinBody::Joint::GetValues(boost::array<dReal, 3>& pValues) const
{
    OPENRAVE_ASSERT_FORMAT0(_bInitialized, "joint not initialized", ORE_NotInitialized);
    if( this->GetDOF() == 1 ) {
        pValues[0] = this->GetValue(0);
        return;
    }

    dReal f;
    Transform tjoint = _tinvLeft * _attachedbodies[0]->GetTransform().inverse() * _attachedbodies[1]->GetTransform() * _tinvRight;
    if( _info._type & KinBody::JointSpecialBit ) {
        switch(_info._type) {
        case KinBody::JointHinge2: {
            Vector axis1cur = tjoint.rotate(_vaxes[0]), axis2cur = tjoint.rotate(_vaxes[1]);
            Vector vec1, vec2, vec3;
            vec1 = (_vaxes[1] - _vaxes[0].dot3(_vaxes[1])*_vaxes[0]).normalize();
            vec2 = (axis2cur - _vaxes[0].dot3(axis2cur)*_vaxes[0]).normalize();
            vec3 = _vaxes[0].cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot3(vec2), vec1.dot3(vec2));
            pValues[0] = GetClosestValueAlongCircle(_info._voffsets[0]+f, _doflastsetvalues[0]);
            vec1 = (_vaxes[0] - axis2cur.dot(_vaxes[0])*axis2cur).normalize();
            vec2 = (axis1cur - axis2cur.dot(axis1cur)*axis2cur).normalize();
            vec3 = axis2cur.cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot(vec2), vec1.dot(vec2));
            if( f < -PI ) {
                f += 2*PI;
            }
            else if( f > PI ) {
                f -= 2*PI;
            }
            pValues[1] = GetClosestValueAlongCircle(_info._voffsets[1]+f, _doflastsetvalues[1]);
            break;
        }
        case KinBody::JointSpherical: {
            dReal fsinang2 = tjoint.rot.y*tjoint.rot.y+tjoint.rot.z*tjoint.rot.z+tjoint.rot.w*tjoint.rot.w;
            if( fsinang2 > 1e-10f ) {
                dReal fsinang = RaveSqrt(fsinang2);
                dReal fmult = 2*RaveAtan2(fsinang,tjoint.rot.x)/fsinang;
                pValues = {tjoint.rot.y*fmult, tjoint.rot.z*fmult, tjoint.rot.w*fmult};
            }
            else {
                pValues = {0, 0, 0};
            }
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("unknown joint type 0x%x"), _info._type, ORE_Failed);
        }
    }
    else {
        // chain of revolute and prismatic joints
        for(int i = 0; i < GetDOF(); ++i) {
            Vector vaxis = _vaxes.at(i);
            if( IsRevolute(i) ) {
                if( i+1 < GetDOF() ) {
                    std::pair<dReal, Vector > res = normalizeAxisRotation(vaxis,tjoint.rot);
                    tjoint.rot = res.second;
                    if( res.first != 0 ) {
                        // could speed up by checking if trans is ever needed after this
                        tjoint.trans = quatRotate(quatFromAxisAngle(vaxis,res.first),tjoint.trans);
                    }
                    f = -res.first;
                }
                else {
                    f = 2.0f*RaveAtan2(tjoint.rot.y*vaxis.x+tjoint.rot.z*vaxis.y+tjoint.rot.w*vaxis.z, tjoint.rot.x);
                }
                // expect values to be within -PI to PI range
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                pValues[0] = GetClosestValueAlongCircle(_info._voffsets[i]+f, _doflastsetvalues[i]);
            }
            else { // prismatic
                f = tjoint.trans.x*vaxis.x+tjoint.trans.y*vaxis.y+tjoint.trans.z*vaxis.z;
                pValues[0] = _info._voffsets[i]+f;
                if( i+1 < GetDOF() ) {
                    tjoint.trans -= vaxis*f;
                }
            }
        }
    }
}

dReal KinBody::Joint::GetValue(int iaxis) const
{
    OPENRAVE_ASSERT_FORMAT0(_bInitialized, "joint not initialized",ORE_NotInitialized);
    if(this->IsStatic()) {
        return _info._vlowerlimit.at(iaxis);
    }
    dReal f;
    Transform tjoint = _tinvLeft * _attachedbodies[0]->GetTransform().inverse() * _attachedbodies[1]->GetTransform() * _tinvRight;
    if( _info._type & KinBody::JointSpecialBit ) {
        switch(_info._type) {
        case KinBody::JointHinge2: {
            Vector axis1cur = tjoint.rotate(_vaxes[0]), axis2cur = tjoint.rotate(_vaxes[1]);
            Vector vec1, vec2, vec3;
            if( iaxis == 0 ) {
                vec1 = (_vaxes[1] - _vaxes[0].dot3(_vaxes[1])*_vaxes[0]).normalize();
                vec2 = (axis2cur - _vaxes[0].dot3(axis2cur)*_vaxes[0]).normalize();
                vec3 = _vaxes[0].cross(vec1);
                f = 2.0*RaveAtan2(vec3.dot3(vec2), vec1.dot3(vec2));
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                return GetClosestValueAlongCircle(_info._voffsets[0]+f, _doflastsetvalues[0]);
            }
            else if( iaxis == 1 ) {
                vec1 = (_vaxes[0] - axis2cur.dot(_vaxes[0])*axis2cur).normalize();
                vec2 = (axis1cur - axis2cur.dot(axis1cur)*axis2cur).normalize();
                vec3 = axis2cur.cross(vec1);
                f = 2.0*RaveAtan2(vec3.dot(vec2), vec1.dot(vec2));
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                return GetClosestValueAlongCircle(_info._voffsets[1]+f, _doflastsetvalues[1]);
            }
            break;
        }
        case KinBody::JointSpherical: {
            dReal fsinang2 = tjoint.rot.y*tjoint.rot.y+tjoint.rot.z*tjoint.rot.z+tjoint.rot.w*tjoint.rot.w;
            if( fsinang2 > 1e-10f ) {
                dReal fsinang = RaveSqrt(fsinang2);
                dReal fmult = 2*RaveAtan2(fsinang,tjoint.rot.x)/fsinang;
                if( iaxis == 0 ) {
                    return tjoint.rot.y*fmult;
                }
                else if( iaxis == 1 ) {
                    return tjoint.rot.z*fmult;
                }
                else if( iaxis == 2 ) {
                    return tjoint.rot.w*fmult;
                }
            }
            else {
                if((iaxis >= 0)&&(iaxis < 3)) {
                    return 0;
                }
            }
            break;
        }
        case KinBody::JointTrajectory: {
            //uint64_t starttime = utils::GetMicroTime();
            vector<dReal> vsampledata;
            dReal splitpercentage = 0.01;
            dReal precision(1e-6);
            dReal timemin = 0, timemax = _info._trajfollow->GetDuration();
            Transform tbest, ttest;
            int totalcalls = 0;
            while(timemin+precision < timemax) {
                dReal timestep = (timemax-timemin)*splitpercentage;
                dReal timeclosest = timemin;
                dReal bestdist = 1e30, besttime=0;
                for(; timeclosest < timemax; timeclosest += timestep ) {
                    if( timeclosest > timemax ) {
                        timeclosest = timemax;
                    }
                    totalcalls += 1;
                    _info._trajfollow->Sample(vsampledata,timeclosest);
                    if( _info._trajfollow->GetConfigurationSpecification().ExtractTransform(ttest,vsampledata.begin(),KinBodyConstPtr()) ) {
                        dReal fdist = TransformDistanceFast(ttest,tjoint,0.3);
                        if( bestdist > fdist ) {
                            besttime = timeclosest;
                            bestdist = fdist;
                            tbest = ttest;
                        }
                    }
                }
                OPENRAVE_ASSERT_OP_FORMAT(bestdist, <, 1e30, "failed to compute trajectory value for joint %s\n",GetName(),ORE_Assert);
                timemin = max(timemin,besttime-timestep);
                timemax = min(timemax, besttime+timestep);
                splitpercentage = 0.1f;
                //RAVELOG_INFO(str(boost::format("calls: %d time: %f")%totalcalls%((utils::GetMicroTime()-starttime)*1e-6)));
            }
            return 0.5*(timemin+timemax);
        }
        default:
            break;
        }
    }
    else {
        if( _info._type == KinBody::JointPrismatic ) {
            return _info._voffsets[0]+(tjoint.trans.x*_vaxes[0].x+tjoint.trans.y*_vaxes[0].y+tjoint.trans.z*_vaxes[0].z);
        }
        else if( _info._type == KinBody::JointRevolute ) {
            f = 2.0f*RaveAtan2(tjoint.rot.y*_vaxes[0].x+tjoint.rot.z*_vaxes[0].y+tjoint.rot.w*_vaxes[0].z, tjoint.rot.x);
            // expect values to be within -PI to PI range
            if( f < -PI ) {
                f += 2*PI;
            }
            else if( f > PI ) {
                f -= 2*PI;
            }
            return GetClosestValueAlongCircle(_info._voffsets[0]+f, _doflastsetvalues[0]);
        }

        // chain of revolute and prismatic joints
        for(int i = 0; i < GetDOF(); ++i) {
            Vector vaxis = _vaxes.at(i);
            if( IsRevolute(i) ) {
                if( i+1 < GetDOF() ) {
                    std::pair<dReal, Vector > res = normalizeAxisRotation(vaxis,tjoint.rot);
                    tjoint.rot = res.second;
                    if( res.first != 0 ) {
                        // could speed up by checking if trans is ever needed after this
                        tjoint.trans = quatRotate(quatFromAxisAngle(vaxis,res.first),tjoint.trans);
                    }
                    f = -res.first;
                }
                else {
                    f = 2.0f*RaveAtan2(tjoint.rot.y*vaxis.x+tjoint.rot.z*vaxis.y+tjoint.rot.w*vaxis.z, tjoint.rot.x);
                }
                // expect values to be within -PI to PI range
                if( f < -PI ) {
                    f += 2*PI;
                }
                else if( f > PI ) {
                    f -= 2*PI;
                }
                if( i == iaxis ) {
                    return GetClosestValueAlongCircle(_info._voffsets[i]+f, _doflastsetvalues[i]);
                }
            }
            else { // prismatic
                f = tjoint.trans.x*vaxis.x+tjoint.trans.y*vaxis.y+tjoint.trans.z*vaxis.z;
                if( i == iaxis ) {
                    return _info._voffsets[i]+f;
                }
                if( i+1 < GetDOF() ) {
                    tjoint.trans -= vaxis*f;
                }
            }
        }
    }
    throw OPENRAVE_EXCEPTION_FORMAT(_("unknown joint type 0x%x axis %d\n"), _info._type%iaxis, ORE_Failed);
}

void KinBody::Joint::GetVelocities(std::vector<dReal>& pVelocities, bool bAppend) const
{
    OPENRAVE_ASSERT_FORMAT0(_bInitialized, "joint not initialized",ORE_NotInitialized);
    if( !bAppend ) {
        pVelocities.resize(0);
    }
    if( GetDOF() == 1 ) {
        pVelocities.push_back(GetVelocity(0));
        return;
    }
    _GetVelocities(pVelocities,bAppend,_attachedbodies[0]->GetVelocity(), _attachedbodies[1]->GetVelocity());
};

dReal KinBody::Joint::GetVelocity(int axis) const
{
    OPENRAVE_ASSERT_FORMAT0(_bInitialized, "joint not initialized",ORE_NotInitialized);
    return _GetVelocity(axis,_attachedbodies[0]->GetVelocity(), _attachedbodies[1]->GetVelocity());
}

void KinBody::Joint::_GetVelocities(std::vector<dReal>& pVelocities, bool bAppend, const std::pair<Vector,Vector>& linkparentvelocity, const std::pair<Vector,Vector>& linkchildvelocity) const
{
    if( !bAppend ) {
        pVelocities.resize(0);
    }
    if( GetDOF() == 1 ) {
        pVelocities.push_back(_GetVelocity(0, linkparentvelocity, linkchildvelocity));
        return;
    }
    const Transform& linkparenttransform = _attachedbodies[0]->_info._t;
    const Transform& linkchildtransform = _attachedbodies[1]->_info._t;
    Vector quatdelta = quatMultiply(linkparenttransform.rot,_tLeft.rot);
    Vector quatdeltainv = quatInverse(quatdelta);
    if( _info._type & KinBody::JointSpecialBit ) {
        switch(_info._type) {
        case KinBody::JointSpherical: {
            Vector v = quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second);
            pVelocities.push_back(v.x);
            pVelocities.push_back(v.y);
            pVelocities.push_back(v.z);
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("unknown joint type 0x%x"), _info._type, ORE_InvalidArguments);
        }
    }
    else {
        // chain of revolute and prismatic joints
        Vector angvelocitycovered, linvelocitycovered;
        for(int i = 0; i < GetDOF(); ++i) {
            if( IsRevolute(i) ) {
                pVelocities.push_back(_vaxes[i].dot3(quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second-angvelocitycovered)));
                angvelocitycovered += quatRotate(quatdelta,_vaxes[i]*pVelocities.back());
            }
            else { // prismatic
                pVelocities.push_back(_vaxes[i].dot3(quatRotate(quatdeltainv,linkchildvelocity.first-linkparentvelocity.first-(linkparentvelocity.second-angvelocitycovered).cross(linkchildtransform.trans-linkparenttransform.trans)-linvelocitycovered)));
                linvelocitycovered += quatRotate(quatdelta,_vaxes[i]*pVelocities.back());
            }
        }
    }
}

dReal KinBody::Joint::_GetVelocity(int axis, const std::pair<Vector,Vector>&linkparentvelocity, const std::pair<Vector,Vector>&linkchildvelocity) const
{
    const Transform& linkparenttransform = _attachedbodies[0]->_info._t;
    const Transform& linkchildtransform = _attachedbodies[1]->_info._t;
    Vector quatdelta = quatMultiply(linkparenttransform.rot,_tLeft.rot);
    Vector quatdeltainv = quatInverse(quatdelta);
    if( _info._type & KinBody::JointSpecialBit ) {
        switch(_info._type) {
        case KinBody::JointSpherical: {
            Vector v = quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second);
            return v[axis];
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("unknown joint type 0x%x"), _info._type, ORE_InvalidArguments);
        }
    }
    else {
        if( _info._type == KinBody::JointPrismatic ) {
            return _vaxes[0].dot3(quatRotate(quatdeltainv,linkchildvelocity.first-linkparentvelocity.first-linkparentvelocity.second.cross(linkchildtransform.trans-linkparenttransform.trans)));
        }
        else if( _info._type == KinBody::JointRevolute ) {
            return _vaxes[0].dot3(quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second));
        }
        else {
            // chain of revolute and prismatic joints
            Vector angvelocitycovered, linvelocitycovered;
            for(int i = 0; i < GetDOF(); ++i) {
                if( IsRevolute(i) ) {
                    dReal fvelocity = _vaxes[i].dot3(quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second-angvelocitycovered));
                    if( i == axis ) {
                        return fvelocity;
                    }
                    angvelocitycovered += quatRotate(quatdelta,_vaxes[i]*fvelocity);
                }
                else { // prismatic
                    dReal fvelocity = _vaxes[i].dot3(quatRotate(quatdeltainv,linkchildvelocity.first-linkparentvelocity.first-(linkparentvelocity.second-angvelocitycovered).cross(linkparenttransform.trans-linkchildtransform.trans)-linvelocitycovered));
                    if( i == axis ) {
                        return fvelocity;
                    }
                    linvelocitycovered += quatRotate(quatdelta,_vaxes[i]*fvelocity);
                }
            }
        }
    }
    throw OPENRAVE_EXCEPTION_FORMAT(_("unsupported joint type 0x%x"), _info._type, ORE_InvalidArguments);
}

Vector KinBody::Joint::GetAnchor() const
{
    return _attachedbodies[0]->GetTransform() * _tLeft.trans;
}

Vector KinBody::Joint::GetAxis(int iaxis) const
{
    return _attachedbodies[0]->GetTransform().rotate(_tLeft.rotate(_vaxes.at(iaxis)));
}

void KinBody::Joint::_ComputeJointInternalInformation(LinkPtr plink0, LinkPtr plink1, const Vector& vanchorraw, const std::vector<Vector>& vaxes, const std::vector<dReal>& vcurrentvalues)
{
    OPENRAVE_ASSERT_OP_FORMAT(!!plink0,&&,!!plink1, "one or more attached _attachedbodies are invalid for joint %s", GetName(),ORE_InvalidArguments);
    for(int i = 0; i < GetDOF(); ++i) {
        OPENRAVE_ASSERT_OP_FORMAT(_info._vmaxvel[i], >=, 0, "joint %s[%d] max velocity is invalid",_info._name%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(_info._vmaxaccel[i], >=, 0, "joint %s[%d] max acceleration is invalid",_info._name%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(_info._vmaxjerk[i], >=, 0, "joint %s[%d] max jerk is invalid",_info._name%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(_info._vmaxtorque[i], >=, 0, "joint %s[%d] max torque is invalid",_info._name%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(_info._vmaxinertia[i], >=, 0, "joint %s[%d] max inertia is invalid",_info._name%i, ORE_InvalidArguments);
    }

    KinBodyPtr parent(_parent);
    _bInitialized = false;
    _attachedbodies[0] = plink0;
    _attachedbodies[1] = plink1;
    Transform trel, tbody0, tbody1;
    Vector vanchor=vanchorraw;
    for(size_t i = 0; i < vaxes.size(); ++i) {
        _vaxes[i] = vaxes[i];
    }
    // make sure first body is always closer to the root, unless the second body is static and the first body is not the root link
    if( _attachedbodies[1]->IsStatic() && _attachedbodies[0]->GetIndex() > 0) {
        if( !_attachedbodies[0]->IsStatic() ) {
            Transform tswap = plink1->GetTransform().inverse() * plink0->GetTransform();
            for(int i = 0; i < GetDOF(); ++i) {
                _vaxes[i] = -tswap.rotate(_vaxes[i]);
            }
            vanchor = tswap*vanchor;
            swap(_attachedbodies[0],_attachedbodies[1]);
        }
    }

    // update _info
    for(size_t i = 0; i < vaxes.size(); ++i) {
        _info._vaxes[i] = _vaxes[i];
    }
    _info._vanchor = vanchor;

    tbody0 = _attachedbodies[0]->GetTransform();
    tbody1 = _attachedbodies[1]->GetTransform();
    trel = tbody0.inverse() * tbody1;
    _tLeft = Transform();
    _tLeftNoOffset = Transform();
    _tRight = Transform();
    _tRightNoOffset = Transform();

    if( _info._type & KinBody::JointSpecialBit ) {
        switch(_info._type) {
        case KinBody::JointUniversal:
            _tLeft.trans = vanchor;
            _tRight.trans = -vanchor;
            _tRight = _tRight * trel;
            OPENRAVE_ASSERT_OP((int)vaxes.size(),==,2);
            break;
        case KinBody::JointHinge2:
            _tLeft.trans = vanchor;
            _tRight.trans = -vanchor;
            _tRight = _tRight * trel;
            OPENRAVE_ASSERT_OP((int)vaxes.size(),==,2);
            break;
        case KinBody::JointSpherical:
            _tLeft.trans = vanchor;
            _tRight.trans = -vanchor;
            _tRight = _tRight * trel;
            break;
        case KinBody::JointTrajectory:
            if( !_info._trajfollow ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("trajectory joint requires Joint::_trajfollow to be initialized"),ORE_InvalidState);
            }
            _tRight = _tRight * trel;
            break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("unrecognized joint type 0x%x"), _info._type, ORE_InvalidArguments);
        }
        _tLeftNoOffset = _tLeft;
        _tRightNoOffset = _tRight;
    }
    else {
        OPENRAVE_ASSERT_OP((int)vaxes.size(),==,GetDOF());
        _tLeftNoOffset.trans = vanchor;
        _tRightNoOffset.trans = -vanchor;
        _tRightNoOffset = _tRightNoOffset * trel;
        if( GetDOF() == 1 && IsRevolute(0) ) {
            // in the case of one axis, create a new coordinate system such that the axis rotates about (0,0,1)
            // this is necessary in order to simplify the rotation matrices (for future symbolic computation)
            // and suppress any floating-point error. The data structures are only setup for this to work in 1 DOF.
            Transform trot; trot.rot = quatRotateDirection(_vaxes[0],Vector(0,0,1));
            _tLeftNoOffset = _tLeftNoOffset * trot.inverse();
            _tRightNoOffset = trot*_tRightNoOffset;
            _vaxes[0] = Vector(0,0,1);
        }

        Transform toffset;
        if( IsRevolute(0) ) {
            toffset.rot = quatFromAxisAngle(_vaxes[0], _info._voffsets[0]); // rotate about (0,0,1) by offset angle
        }
        else {
            toffset.trans = _vaxes[0]*_info._voffsets[0];
        }
        _tLeft = _tLeftNoOffset * toffset;
        _tRight = _tRightNoOffset;
        if( GetDOF() > 1 ) {
            // right multiply by the offset of the last axis, might be buggy?
            if( IsRevolute(GetDOF()-1) ) {
                _tRight = matrixFromAxisAngle(_vaxes[GetDOF()-1], _info._voffsets[GetDOF()-1]) * _tRight;
            }
            else {
                _tRight.trans += _vaxes[GetDOF()-1]*_info._voffsets[GetDOF()-1];
            }
        }
    }

    if( vcurrentvalues.size() > 0 ) {
        // see if any joints have offsets
        Transform toffset;
        if( _info._type == KinBody::JointTrajectory ) {
            vector<dReal> vsampledata;
            Transform t0, t1;
            _info._trajfollow->Sample(vsampledata,0);
            if( !_info._trajfollow->GetConfigurationSpecification().ExtractTransform(t0,vsampledata.begin(),KinBodyConstPtr()) ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("failed to sample trajectory for joint %s"),GetName(),ORE_Assert);
            }
            _info._trajfollow->Sample(vsampledata,vcurrentvalues.at(0));
            if( !_info._trajfollow->GetConfigurationSpecification().ExtractTransform(t1,vsampledata.begin(),KinBodyConstPtr()) ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("failed to sample trajectory for joint %s"),GetName(),ORE_Assert);
            }
            toffset = t0*t1.inverse();
        }
        else if( !(_info._type&KinBody::JointSpecialBit) || _info._type == KinBody::JointUniversal || _info._type == KinBody::JointHinge2 ) {
            if( IsRevolute(0) ) {
                toffset.rot = quatFromAxisAngle(_vaxes[0], -vcurrentvalues[0]);
            }
            else {
                toffset.trans = -_vaxes[0]*vcurrentvalues[0];
            }
        }
        _tLeftNoOffset *= toffset;
        _tLeft *= toffset;
        if( vcurrentvalues.size() > 1 ) {
            if( IsRevolute(GetDOF()-1) ) {
                toffset.rot = quatFromAxisAngle(_vaxes[GetDOF()-1], -vcurrentvalues.at(GetDOF()-1));
            }
            else {
                toffset.trans = -_vaxes[GetDOF()-1]*vcurrentvalues.at(GetDOF()-1);
            }
            _tRightNoOffset = toffset * _tRightNoOffset;
            _tRight = toffset * _tRight;
        }
    }
    _tinvRight = _tRight.inverse();
    _tinvLeft = _tLeft.inverse();

    _vcircularlowerlimit = _info._vlowerlimit;
    _vcircularupperlimit = _info._vupperlimit;
    for(int i = 0; i < GetDOF(); ++i) {
        if( IsCircular(i) ) {
            // can rotate forever, so don't limit it. Unfortunately if numbers are too big precision will start getting lost
            _info._vlowerlimit.at(i) = -1e4;
            _info._vupperlimit.at(i) = 1e4;
        }
    }

    if( !!_attachedbodies[0] ) {
        _info._linkname0 = _attachedbodies[0]->GetName();
    }
    else {
        _info._linkname0.clear();
    }
    if( !!_attachedbodies[1] ) {
        _info._linkname1 = _attachedbodies[1]->GetName();
    }
    else {
        _info._linkname1.clear();
    }
    _info._vcurrentvalues = vcurrentvalues;

    if( _attachedbodies[1]->IsStatic() && !IsStatic() ) {
        RAVELOG_WARN(str(boost::format("joint %s: all attached links are static, but joint is not!\n")%GetName()));
    }

    // set _bInitialized at the end
    _bInitialized = true;
}

void KinBody::Joint::_ComputeInternalStaticInformation()
{
    if(this->IsStatic()) {
        for(int idof = 0; idof < GetDOF(); ++idof) {
            if( _info._vlowerlimit[idof] != 0 ) {
                if( RaveFabs(_info._vlowerlimit[idof]) > g_fEpsilon ) {
                    RAVELOG_WARN_FORMAT("static joint %s has non-zero lower limit %e, setting to 0", _info._name%_info._vlowerlimit[idof]);
                }
                _info._vlowerlimit[idof] = 0;
            }
            if( _info._vupperlimit[idof] != 0 ) {
                if( RaveFabs(_info._vupperlimit[idof]) > g_fEpsilon ) {
                    RAVELOG_WARN_FORMAT("static joint %s has non-zero upper limit %e, setting to 0", _info._name%_info._vupperlimit[idof]);
                }
                _info._vupperlimit[idof] = 0;
            }
        }
        _nIsStatic = 1;
        _tLeftNoOffset *= _tRightNoOffset;
        _tRightNoOffset = Transform();
        _tLeft *= _tRight;
        _tRight = Transform();
        _tinvLeft = _tLeft.inverse();
        _tinvRight = Transform();
    }
    else {
        _nIsStatic = 0;
    }
}

KinBody::LinkPtr KinBody::Joint::GetHierarchyParentLink() const
{
    return _attachedbodies[0];
}

KinBody::LinkPtr KinBody::Joint::GetHierarchyChildLink() const
{
    return _attachedbodies[1];
}

const Vector& KinBody::Joint::GetInternalHierarchyAxis(int iaxis) const
{
    return _vaxes.at(iaxis);
}

const Transform& KinBody::Joint::GetInternalHierarchyLeftTransform() const
{
    OPENRAVE_ASSERT_FORMAT0(_bInitialized, "joint not initialized",ORE_NotInitialized);
    return _tLeftNoOffset;
}

const Transform& KinBody::Joint::GetInternalHierarchyRightTransform() const
{
    OPENRAVE_ASSERT_FORMAT0(_bInitialized, "joint not initialized",ORE_NotInitialized);
    return _tRightNoOffset;
}

void KinBody::Joint::GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit, bool bAppend) const
{
    if( !bAppend ) {
        vLowerLimit.resize(0);
        vUpperLimit.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vLowerLimit.push_back(_info._vlowerlimit[i]);
        vUpperLimit.push_back(_info._vupperlimit[i]);
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetLimit(int iaxis) const
{
    return make_pair(_info._vlowerlimit.at(iaxis),_info._vupperlimit.at(iaxis));
}

void KinBody::Joint::SetLimits(const std::vector<dReal>& vLowerLimit, const std::vector<dReal>& vUpperLimit)
{
    bool bChanged = false;
    for(int i = 0; i < GetDOF(); ++i) {
        if( _info._vlowerlimit[i] != vLowerLimit.at(i) || _info._vupperlimit[i] != vUpperLimit.at(i) ) {
            bChanged = true;
            _info._vlowerlimit[i] = vLowerLimit.at(i);
            _info._vupperlimit[i] = vUpperLimit.at(i);
            if( IsRevolute(i) && !IsCircular(i) ) {
                // TODO, necessary to set wrap?
                if( _info._vlowerlimit[i] < -PI || _info._vupperlimit[i] > PI) {
                    SetWrapOffset(0.5f * (_info._vlowerlimit.at(i) + _info._vupperlimit.at(i)),i);
                }
                else {
                    SetWrapOffset(0,i);
                }
            }
        }
    }
    if( bChanged ) {
        GetParent()->_PostprocessChangedParameters(Prop_JointLimits);
    }
}

void KinBody::Joint::GetVelocityLimits(std::vector<dReal>& vlower, std::vector<dReal>& vupper, bool bAppend) const
{
    if( !bAppend ) {
        vlower.resize(0);
        vupper.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vlower.push_back(-_info._vmaxvel[i]);
        vupper.push_back(_info._vmaxvel[i]);
    }
}

void KinBody::Joint::GetVelocityLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info._vmaxvel[i]);
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetVelocityLimit(int iaxis) const
{
    return make_pair(-_info._vmaxvel.at(iaxis), _info._vmaxvel.at(iaxis));
}

void KinBody::Joint::SetVelocityLimits(const std::vector<dReal>& vmaxvel)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info._vmaxvel[i] = vmaxvel.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetAccelerationLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info._vmaxaccel[i]);
    }
}

dReal KinBody::Joint::GetAccelerationLimit(int iaxis) const
{
    return _info._vmaxaccel.at(iaxis);
}

void KinBody::Joint::SetAccelerationLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info._vmaxaccel[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetJerkLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info._vmaxjerk[i]);
    }
}

dReal KinBody::Joint::GetJerkLimit(int iaxis) const
{
    return _info._vmaxjerk.at(iaxis);
}

void KinBody::Joint::SetJerkLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info._vmaxjerk[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetHardVelocityLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info._vhardmaxvel[i]);
    }
}

dReal KinBody::Joint::GetHardVelocityLimit(int iaxis) const
{
    return _info._vhardmaxvel.at(iaxis);
}

void KinBody::Joint::SetHardVelocityLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info._vhardmaxvel[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetHardAccelerationLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info._vhardmaxaccel[i]);
    }
}

dReal KinBody::Joint::GetHardAccelerationLimit(int iaxis) const
{
    return _info._vhardmaxaccel.at(iaxis);
}

void KinBody::Joint::SetHardAccelerationLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info._vhardmaxaccel[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetHardJerkLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info._vhardmaxjerk[i]);
    }
}

dReal KinBody::Joint::GetHardJerkLimit(int iaxis) const
{
    return _info._vhardmaxjerk.at(iaxis);
}

void KinBody::Joint::SetHardJerkLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info._vhardmaxjerk[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetTorqueLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info._vmaxtorque[i]);
    }
}

void KinBody::Joint::SetTorqueLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info._vmaxtorque[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetInertiaLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info._vmaxinertia[i]);
    }
}

void KinBody::Joint::SetInertiaLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info._vmaxinertia[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::SetWrapOffset(dReal newoffset, int iaxis)
{
    if( _info._voffsets.at(iaxis) != newoffset ) {
        _info._voffsets.at(iaxis) = newoffset;
        if( iaxis == 0 ) {
            Transform toffset;
            if( IsRevolute(0) ) {
                toffset.rot = quatFromAxisAngle(_vaxes[0], newoffset);
            }
            else {
                toffset.trans = _vaxes[0]*newoffset;
            }
            _tLeft = _tLeftNoOffset * toffset;
            _tinvLeft = _tLeft.inverse();
        }
        if(GetDOF() > 1 && iaxis==GetDOF()-1 ) {
            _tRight = _tRightNoOffset;
            // right multiply by the offset of the last axis, might be buggy?
            if( IsRevolute(GetDOF()-1) ) {
                _tRight = matrixFromAxisAngle(_vaxes[GetDOF()-1], newoffset) * _tRight;
            }
            else {
                _tRight.trans += _vaxes[GetDOF()-1]*newoffset;
            }
            _tinvRight = _tRight.inverse();
        }
        GetParent()->_PostprocessChangedParameters(Prop_JointOffset);
    }
}

void KinBody::Joint::GetResolutions(std::vector<dReal>& resolutions, bool bAppend) const
{
    if( !bAppend ) {
        resolutions.resize(GetDOF());
    }
    for(int i = 0; i < GetDOF(); ++i) {
        resolutions.push_back(_info._vresolution[i]);
    }
}

dReal KinBody::Joint::GetResolution(int iaxis) const
{
    return _info._vresolution.at(iaxis);
}

void KinBody::Joint::SetResolution(dReal resolution, int iaxis)
{
    _info._vresolution.at(iaxis) = resolution;
    GetParent()->_PostprocessChangedParameters(Prop_JointProperties);
}

void KinBody::Joint::GetWeights(std::vector<dReal>& weights, bool bAppend) const
{
    if( !bAppend ) {
        weights.resize(GetDOF());
    }
    for(int i = 0; i < GetDOF(); ++i) {
        weights.push_back(_info._vweights[i]);
    }
}

dReal KinBody::Joint::GetWeight(int iaxis) const
{
    return _info._vweights.at(iaxis);
}

void KinBody::Joint::SetWeights(const std::vector<dReal>& vweights)
{
    for(int i = 0; i < GetDOF(); ++i) {
        OPENRAVE_ASSERT_OP(vweights.at(i),>,0);
        _info._vweights[i] = vweights.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointProperties);
}

void KinBody::Joint::SubtractValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const
{
    for(int i = 0; i < GetDOF(); ++i) {
        if( IsCircular(i) ) {
            q1.at(i) = utils::NormalizeCircularAngle(q1.at(i)-q2.at(i),_vcircularlowerlimit.at(i),_vcircularupperlimit.at(i));
        }
        else {
            q1.at(i) -= q2.at(i);
        }
    }
}

dReal KinBody::Joint::SubtractValue(dReal value1, dReal value2, int iaxis) const
{
    if( IsCircular(iaxis) ) {
        return utils::NormalizeCircularAngle(value1-value2,_vcircularlowerlimit.at(iaxis),_vcircularupperlimit.at(iaxis));
    }
    else {
        return value1-value2;
    }
}

void KinBody::Joint::AddTorque(const std::vector<dReal>& pTorques)
{
    GetParent()->GetEnv()->GetPhysicsEngine()->AddJointTorque(shared_from_this(), pTorques);
}

dReal KinBody::Joint::GetMaxTorque(int iaxis) const
{
    if( !_info._infoElectricMotor ) {
        return _info._vmaxtorque.at(iaxis);
    }
    else {
        if( _info._infoElectricMotor->max_speed_torque_points.size() > 0 ) {
            if( _info._infoElectricMotor->max_speed_torque_points.size() == 1 ) {
                // doesn't matter what the velocity is
                return _info._infoElectricMotor->max_speed_torque_points.at(0).second*_info._infoElectricMotor->gear_ratio;
            }

            dReal velocity = RaveFabs(GetVelocity(iaxis));
            dReal revolutionsPerSecond = _info._infoElectricMotor->gear_ratio * velocity;
            if( IsRevolute(iaxis) ) {
                revolutionsPerSecond /= 2*M_PI;
            }

            if( revolutionsPerSecond <= _info._infoElectricMotor->max_speed_torque_points.at(0).first ) {
                return _info._infoElectricMotor->max_speed_torque_points.at(0).second*_info._infoElectricMotor->gear_ratio;
            }

            for(size_t i = 1; i < _info._infoElectricMotor->max_speed_torque_points.size(); ++i) {
                if( revolutionsPerSecond <= _info._infoElectricMotor->max_speed_torque_points.at(i).first ) {
                    // linearly interpolate to get the desired torque
                    dReal rps0 = _info._infoElectricMotor->max_speed_torque_points.at(i-1).first;
                    dReal torque0 = _info._infoElectricMotor->max_speed_torque_points.at(i-1).second;
                    dReal rps1 = _info._infoElectricMotor->max_speed_torque_points.at(i).first;
                    dReal torque1 = _info._infoElectricMotor->max_speed_torque_points.at(i).second;
                    if( rps1 - rps0 <= g_fEpsilonLinear ) {
                        return torque1*_info._infoElectricMotor->gear_ratio;
                    }

                    return ((revolutionsPerSecond - rps0)/(rps1 - rps0)*(torque1-torque0) + torque0)*_info._infoElectricMotor->gear_ratio;
                }
            }

            // revolutionsPerSecond is huge, return the last point
            return _info._infoElectricMotor->max_speed_torque_points.back().second*_info._infoElectricMotor->gear_ratio;
        }
        else {
            return _info._infoElectricMotor->max_instantaneous_torque*_info._infoElectricMotor->gear_ratio;
        }
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetInstantaneousTorqueLimits(int iaxis) const
{
    if( !_info._infoElectricMotor ) {
        return std::make_pair(-_info._vmaxtorque.at(iaxis), _info._vmaxtorque.at(iaxis));
    }
    else {
        if( _info._infoElectricMotor->max_speed_torque_points.size() > 0 ) {
            dReal fMaxTorqueAtZeroSpeed = _info._infoElectricMotor->max_speed_torque_points.at(0).second*_info._infoElectricMotor->gear_ratio;
            if( _info._infoElectricMotor->max_speed_torque_points.size() == 1 ) {
                // doesn't matter what the velocity is
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            dReal rawvelocity = GetVelocity(iaxis);
            dReal velocity = RaveFabs(rawvelocity);
            dReal revolutionsPerSecond = _info._infoElectricMotor->gear_ratio * velocity;
            if( IsRevolute(iaxis) ) {
                revolutionsPerSecond /= 2*M_PI;
            }

            if( revolutionsPerSecond <= _info._infoElectricMotor->max_speed_torque_points.at(0).first ) {
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            for(size_t i = 1; i < _info._infoElectricMotor->max_speed_torque_points.size(); ++i) {
                if( revolutionsPerSecond <= _info._infoElectricMotor->max_speed_torque_points.at(i).first ) {
                    // linearly interpolate to get the desired torque
                    dReal rps0 = _info._infoElectricMotor->max_speed_torque_points.at(i-1).first;
                    dReal torque0 = _info._infoElectricMotor->max_speed_torque_points.at(i-1).second;
                    dReal rps1 = _info._infoElectricMotor->max_speed_torque_points.at(i).first;
                    dReal torque1 = _info._infoElectricMotor->max_speed_torque_points.at(i).second;

                    dReal finterpolatedtorque;
                    if( rps1 - rps0 <= g_fEpsilonLinear ) {
                        finterpolatedtorque = torque1*_info._infoElectricMotor->gear_ratio;
                    }
                    else {
                        finterpolatedtorque = ((revolutionsPerSecond - rps0)/(rps1 - rps0)*(torque1-torque0) + torque0)*_info._infoElectricMotor->gear_ratio;
                    }

                    // due to back emf, the deceleration magnitude is less than acceleration?
                    if (abs(rawvelocity) < 1.0/360) {
                        return std::make_pair(-finterpolatedtorque, finterpolatedtorque);
                    }
                    else if( rawvelocity > 0 ) {
                        return std::make_pair(-0.9*finterpolatedtorque, finterpolatedtorque);
                    }
                    else {
                        return std::make_pair(-finterpolatedtorque, 0.9*finterpolatedtorque);
                    }
                }
            }

            // due to back emf, the deceleration magnitude is less than acceleration?
            // revolutionsPerSecond is huge, return the last point
            dReal f = _info._infoElectricMotor->max_speed_torque_points.back().second*_info._infoElectricMotor->gear_ratio;
            if (abs(rawvelocity) < 1.0/360) {
                return std::make_pair(-f, f);
            }
            else if( rawvelocity > 0 ) {
                return std::make_pair(-0.9*f, f);
            }
            else {
                return std::make_pair(-f, 0.9*f);
            }
        }
        else {
            dReal f = _info._infoElectricMotor->max_instantaneous_torque*_info._infoElectricMotor->gear_ratio;
            return std::make_pair(-f, f);
        }
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetNominalTorqueLimits(int iaxis) const
{
    if( !_info._infoElectricMotor ) {
        return std::make_pair(-_info._vmaxtorque.at(iaxis), _info._vmaxtorque.at(iaxis));
    }
    else {
        if( _info._infoElectricMotor->nominal_speed_torque_points.size() > 0 ) {
            dReal fMaxTorqueAtZeroSpeed = _info._infoElectricMotor->nominal_speed_torque_points.at(0).second*_info._infoElectricMotor->gear_ratio;
            if( _info._infoElectricMotor->nominal_speed_torque_points.size() == 1 ) {
                // doesn't matter what the velocity is
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            dReal rawvelocity = GetVelocity(iaxis);
            dReal velocity = RaveFabs(rawvelocity);
            dReal revolutionsPerSecond = _info._infoElectricMotor->gear_ratio * velocity;
            if( IsRevolute(iaxis) ) {
                revolutionsPerSecond /= 2*M_PI;
            }

            if( revolutionsPerSecond <= _info._infoElectricMotor->nominal_speed_torque_points.at(0).first ) {
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            for(size_t i = 1; i < _info._infoElectricMotor->nominal_speed_torque_points.size(); ++i) {
                if( revolutionsPerSecond <= _info._infoElectricMotor->nominal_speed_torque_points.at(i).first ) {
                    // linearly interpolate to get the desired torque
                    dReal rps0 = _info._infoElectricMotor->nominal_speed_torque_points.at(i-1).first;
                    dReal torque0 = _info._infoElectricMotor->nominal_speed_torque_points.at(i-1).second;
                    dReal rps1 = _info._infoElectricMotor->nominal_speed_torque_points.at(i).first;
                    dReal torque1 = _info._infoElectricMotor->nominal_speed_torque_points.at(i).second;

                    dReal finterpolatedtorque;
                    if( rps1 - rps0 <= g_fEpsilonLinear ) {
                        finterpolatedtorque = torque1*_info._infoElectricMotor->gear_ratio;
                    }
                    else {
                        finterpolatedtorque = ((revolutionsPerSecond - rps0)/(rps1 - rps0)*(torque1-torque0) + torque0)*_info._infoElectricMotor->gear_ratio;
                    }

                    // due to back emf, the deceleration magnitude is less than acceleration?
                    if (abs(rawvelocity) < 1.0/360) {
                        return std::make_pair(-finterpolatedtorque, finterpolatedtorque);
                    }
                    else if( rawvelocity > 0 ) {
                        return std::make_pair(-0.9*finterpolatedtorque, finterpolatedtorque);
                    }
                    else {
                        return std::make_pair(-finterpolatedtorque, 0.9*finterpolatedtorque);
                    }
                }
            }

            // due to back emf, the deceleration magnitude is less than acceleration?
            // revolutionsPerSecond is huge, return the last point
            dReal f = _info._infoElectricMotor->nominal_speed_torque_points.back().second*_info._infoElectricMotor->gear_ratio;
            if (abs(rawvelocity) < 1.0/360) {
                return std::make_pair(-f, f);
            }
            else if( rawvelocity > 0 ) {
                return std::make_pair(-0.9*f, f);
            }
            else {
                return std::make_pair(-f, 0.9*f);
            }
        }
        else {
            dReal f = _info._infoElectricMotor->nominal_torque*_info._infoElectricMotor->gear_ratio;
            return std::make_pair(-f, f);
        }
    }
}

int KinBody::Joint::GetMimicJointIndex() const
{
    for(int i = 0; i < GetDOF(); ++i) {
        if( !!_vmimic.at(i) &&(_vmimic.at(i)->_vmimicdofs.size() > 0)) {
            return GetParent()->GetJointFromDOFIndex(_vmimic.at(i)->_vmimicdofs.front().dofindex)->GetJointIndex();
        }
    }
    return -1;
}

const std::vector<dReal> KinBody::Joint::GetMimicCoeffs() const
{
    RAVELOG_WARN("deprecated KinBody::Joint::GetMimicCoeffs(): could not deduce coeffs\n");
    std::vector<dReal> coeffs(2); coeffs[0] = 1; coeffs[1] = 0;
    return coeffs;
}

bool KinBody::Joint::IsMimic(int iaxis) const
{
    if( iaxis >= 0 ) {
        return !!_vmimic.at(iaxis);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        if( !!_vmimic.at(i) ) {
            return true;
        }
    }
    return false;
}

std::string KinBody::Joint::GetMimicEquation(int iaxis, int itype, const std::string& format) const
{
    if( !_vmimic.at(iaxis) ) {
        return "";
    }
    if((format.size() == 0)||(format == "fparser")) {
        return _vmimic.at(iaxis)->_equations.at(itype);
    }
    else if( format == "mathml" ) {
        boost::format mathfmt("<math xmlns=\"http://www.w3.org/1998/Math/MathML\">\n%s</math>\n");
        std::vector<std::string> Vars;
        std::string sout;
        KinBodyConstPtr parent(_parent);
        FOREACHC(itdofformat, _vmimic.at(iaxis)->_vdofformat) {
            JointConstPtr pjoint = itdofformat->GetJoint(*parent);
            if( pjoint->GetDOF() > 1 ) {
                Vars.push_back(str(boost::format("<csymbol>%s_%d</csymbol>")%pjoint->GetName()%(int)itdofformat->axis));
            }
            else {
                Vars.push_back(str(boost::format("<csymbol>%s</csymbol>")%pjoint->GetName()));
            }
        }
        if( itype == 0 ) {
            _vmimic.at(iaxis)->_posfn->toMathML(sout,Vars);
            if((sout.size() > 9)&&(sout.substr(0,9) == "<csymbol>")) {
                // due to a bug in ROS robot_model, have to return with <apply> (remove this in 2012).
                sout = boost::str(boost::format("<apply>\n  <plus/><cn type=\"real\">0</cn>\n  %s\n  </apply>")%sout);
            }
            sout = str(mathfmt%sout);
        }
        else if( itype == 1 ) {
            std::string stemp;
            FOREACHC(itfn, _vmimic.at(iaxis)->_velfns) {
                (*itfn)->toMathML(stemp,Vars);
                sout += str(mathfmt%stemp);
            }
        }
        else if( itype == 2 ) {
            std::string stemp;
            FOREACHC(itfn, _vmimic.at(iaxis)->_accelfns) {
                (*itfn)->toMathML(stemp,Vars);
                sout += str(mathfmt%stemp);
            }
        }
        return sout;
    }
    throw OPENRAVE_EXCEPTION_FORMAT(_("unsupported math format %s"), format, ORE_InvalidArguments);
}

void KinBody::Joint::GetMimicDOFIndices(std::vector<int>& vmimicdofs, int iaxis) const
{
    OPENRAVE_ASSERT_FORMAT(!!_vmimic.at(iaxis), "joint %s axis %d is not mimic", GetName()%iaxis,ORE_InvalidArguments);
    vmimicdofs.resize(0);
    FOREACHC(it, _vmimic.at(iaxis)->_vmimicdofs) {
        std::vector<int>::iterator itinsert = std::lower_bound(vmimicdofs.begin(),vmimicdofs.end(), it->dofindex);
        if((itinsert == vmimicdofs.end())||(*itinsert != it->dofindex)) {
            vmimicdofs.insert(itinsert,it->dofindex);
        }
    }
}

void KinBody::Joint::SetMimicEquations(int iaxis, const std::string& poseq, const std::string& veleq, const std::string& acceleq)
{
    _vmimic.at(iaxis).reset();
    if( poseq.size() == 0 ) {
        return;
    }
    KinBodyPtr parent(_parent);
    std::vector<std::string> resultVars;
    MimicPtr mimic(new Mimic());
    mimic->_equations.at(0) = poseq;
    mimic->_equations.at(1) = veleq;
    mimic->_equations.at(2) = acceleq;

    // copy equations into the info
    if( !_info._vmimic.at(iaxis) ) {
        _info._vmimic.at(iaxis).reset(new MimicInfo());
    }
    _info._vmimic.at(iaxis)->_equations = mimic->_equations;

    OpenRAVEFunctionParserRealPtr posfn = CreateJointFunctionParser();
    mimic->_posfn = posfn;
    // because openrave joint names can hold symbols like '-' and '.' can affect the equation, so first do a search and replace
    std::vector< std::pair<std::string, std::string> > jointnamepairs; jointnamepairs.reserve(parent->GetJoints().size());
    FOREACHC(itjoint,parent->GetJoints()) {
        if( (*itjoint)->GetName().size() > 0 ) {
            std::string newname = str(boost::format("joint%d")%(*itjoint)->GetJointIndex());
            jointnamepairs.emplace_back((*itjoint)->GetName(), newname);
        }
    }
    size_t index = parent->GetJoints().size();
    FOREACHC(itjoint,parent->GetPassiveJoints()) {
        if( (*itjoint)->GetName().size() > 0 ) {
            std::string newname = str(boost::format("joint%d")%index);
            jointnamepairs.emplace_back((*itjoint)->GetName(), newname);
        }
        ++index;
    }

    std::map<std::string,std::string> mapinvnames;
    FOREACH(itpair,jointnamepairs) {
        mapinvnames[itpair->second] = itpair->first;
    }

    std::string eq;
    int ret = posfn->ParseAndDeduceVariables(utils::SearchAndReplace(eq,mimic->_equations[0],jointnamepairs),resultVars);
    if( ret >= 0 ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to set equation '%s' on %s:%s, at %d. Error is %s\n"), mimic->_equations[0]%parent->GetName()%GetName()%ret%posfn->ErrorMsg(),ORE_InvalidArguments);
    }
    // process the variables
    FOREACH(itvar,resultVars) {
        OPENRAVE_ASSERT_FORMAT(itvar->find("joint") == 0, "equation '%s' uses unknown variable", mimic->_equations[0], ORE_InvalidArguments);
        MIMIC::DOFFormat dofformat;
        size_t axisindex = itvar->find('_');
        if( axisindex != std::string::npos ) {
            dofformat.jointindex = boost::lexical_cast<uint16_t>(itvar->substr(5,axisindex-5));
            dofformat.axis = boost::lexical_cast<uint8_t>(itvar->substr(axisindex+1));
        }
        else {
            dofformat.jointindex = boost::lexical_cast<uint16_t>(itvar->substr(5));
            dofformat.axis = 0;
        }
        dofformat.dofindex = -1;
        JointPtr pjoint = dofformat.GetJoint(*parent);
        if((pjoint->GetDOFIndex() >= 0)&& !pjoint->IsMimic(dofformat.axis) ) {
            dofformat.dofindex = pjoint->GetDOFIndex()+dofformat.axis;
            MIMIC::DOFHierarchy h;
            h.dofindex = dofformat.dofindex;
            h.dofformatindex = mimic->_vdofformat.size();
            mimic->_vmimicdofs.push_back(h);
        }
        mimic->_vdofformat.push_back(dofformat);
    }

    // need to set sVars to resultVars since that's what the user will be feeding with the input
    stringstream sVars;
    if( !resultVars.empty() ) {
        sVars << resultVars.at(0);
        for(size_t i = 1; i < resultVars.size(); ++i) {
            sVars << "," << resultVars[i];
        }
    }

    for(int itype = 1; itype < 3; ++itype) {
        if((itype == 2)&&(mimic->_equations[itype].size() == 0)) {
            continue;
        }

        std::vector<OpenRAVEFunctionParserRealPtr> vfns(resultVars.size());
        // extract the equations
        utils::SearchAndReplace(eq,mimic->_equations[itype],jointnamepairs);
        size_t index = eq.find('|');
        while(index != std::string::npos) {
            size_t startindex = index+1;
            index = eq.find('|',startindex);
            string sequation;
            if( index != std::string::npos) {
                sequation = eq.substr(startindex,index-startindex);
            }
            else {
                sequation = eq.substr(startindex);
            }
            boost::trim(sequation);
            size_t nameendindex = sequation.find(' ');
            string varname;
            if( nameendindex == std::string::npos ) {
                RAVELOG_WARN(str(boost::format("invalid equation syntax '%s' for joint %s")%sequation%_info._name));
                varname = sequation;
                sequation = "0";
            }
            else {
                varname = sequation.substr(0,nameendindex);
                sequation = sequation.substr(nameendindex);
            }
            vector<string>::iterator itnameindex = find(resultVars.begin(),resultVars.end(),varname);
            OPENRAVE_ASSERT_FORMAT(itnameindex != resultVars.end(), "variable %s from velocity equation is not referenced in the position, skipping...", mapinvnames[varname],ORE_InvalidArguments);

            OpenRAVEFunctionParserRealPtr fn = CreateJointFunctionParser();
            ret = fn->Parse(sequation,sVars.str());
            if( ret >= 0 ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("failed to set equation '%s' on %s:%s, at %d. Error is %s"), sequation%parent->GetName()%GetName()%ret%fn->ErrorMsg(),ORE_InvalidArguments);
            }
            vfns.at(itnameindex-resultVars.begin()) = fn;
        }
        // check if anything is missing
        for(size_t j = 0; j < resultVars.size(); ++j) {
            if( !vfns[j] ) {
                // print a message instead of throwing an exception since it might be common for only position equations to be specified
                RAVELOG_WARN(str(boost::format("SetMimicEquations: missing variable %s from partial derivatives of joint %s!")%mapinvnames[resultVars[j]]%_info._name));
                vfns[j] = CreateJointFunctionParser();
                vfns[j]->Parse("0","");
            }
        }

        if( itype == 1 ) {
            mimic->_velfns.swap(vfns);
        }
        else {
            mimic->_accelfns.swap(vfns);
        }
    }
    _vmimic.at(iaxis) = mimic;
    parent->_PostprocessChangedParameters(Prop_JointMimic);
}

void KinBody::Joint::_ComputePartialVelocities(std::vector<std::pair<int,dReal> >& vpartials, int iaxis, std::map< std::pair<MIMIC::DOFFormat, int>, dReal >& mapcachedpartials) const
{
    vpartials.resize(0);
    if( dofindex >= 0 ) {
        vpartials.emplace_back(dofindex+iaxis, 1.0);
        return;
    }
    OPENRAVE_ASSERT_FORMAT(!!_vmimic.at(iaxis), "cannot compute partial velocities of joint %s", _info._name, ORE_Failed);
    KinBodyConstPtr parent(_parent);
    MIMIC::DOFFormat thisdofformat;
    thisdofformat.dofindex = -1; // always -1 since it is mimiced
    thisdofformat.axis = iaxis;
    thisdofformat.jointindex = jointindex;
    if( jointindex < 0 ) {
        // this is a weird computation... have to figure out the passive joint index given where it is in parent->GetPassiveJoints()
        thisdofformat.jointindex = parent->GetJoints().size() + (find(parent->GetPassiveJoints().begin(),parent->GetPassiveJoints().end(),shared_from_this()) - parent->GetPassiveJoints().begin());
    }
    std::vector<std::pair<int,dReal> > vtemppartials;
    vector<dReal> vtempvalues;
    FOREACHC(itmimicdof, _vmimic[iaxis]->_vmimicdofs) {
        std::pair<MIMIC::DOFFormat, int> key = make_pair(thisdofformat,itmimicdof->dofindex);
        std::map< std::pair<MIMIC::DOFFormat, int>, dReal >::iterator it = mapcachedpartials.find(key);
        if( it == mapcachedpartials.end() ) {
            // not in the cache so compute using the chain rule
            if( vtempvalues.empty() ) {
                FOREACHC(itdofformat, _vmimic[iaxis]->_vdofformat) {
                    vtempvalues.push_back(itdofformat->GetJoint(*parent)->GetValue(itdofformat->axis));
                }
            }
            dReal fvel = _vmimic[iaxis]->_velfns.at(itmimicdof->dofformatindex)->Eval(vtempvalues.empty() ? NULL : &vtempvalues[0]);
            const MIMIC::DOFFormat& dofformat = _vmimic[iaxis]->_vdofformat.at(itmimicdof->dofformatindex);
            if( dofformat.GetJoint(*parent)->IsMimic(dofformat.axis) ) {
                dofformat.GetJoint(*parent)->_ComputePartialVelocities(vtemppartials,dofformat.axis,mapcachedpartials);
                dReal fpartial = 0;
                FOREACHC(itpartial,vtemppartials) {
                    if( itpartial->first == itmimicdof->dofindex ) {
                        fpartial += itpartial->second;
                    }
                }
                fvel *= fpartial;
            }
            // before pushing back, check for repetition
            bool badd = true;
            FOREACH(itpartial,vpartials) {
                if( itpartial->first == itmimicdof->dofindex ) {
                    itpartial->second += fvel;
                    badd = false;
                    break;
                }
            }
            if( badd ) {
                vpartials.emplace_back(itmimicdof->dofindex,  fvel);
            }
        }
        else {
            bool badd = true;
            FOREACH(itpartial,vpartials) {
                if( itpartial->first == itmimicdof->dofindex ) {
                    badd = false;
                    break;
                }
            }
            if( badd ) {
                vpartials.emplace_back(itmimicdof->dofindex,  it->second);
            }
        }
    }
}

int KinBody::Joint::_Eval(int axis, uint32_t timederiv, const std::vector<dReal>& vdependentvalues, std::vector<dReal>& voutput) const
{
    if( timederiv == 0 ) {
        _vmimic.at(axis)->_posfn->EvalMulti(voutput, vdependentvalues.empty() ? NULL : &vdependentvalues[0]);
        return _vmimic.at(axis)->_posfn->EvalError();
    }
    else if( timederiv == 1 ) {
        voutput.resize(_vmimic.at(axis)->_velfns.size());
        for(size_t i = 0; i < voutput.size(); ++i) {
            voutput[i] = _vmimic.at(axis)->_velfns.at(i)->Eval(vdependentvalues.empty() ? NULL : &vdependentvalues[0]);
            int err = _vmimic.at(axis)->_velfns.at(i)->EvalError();
            if( err ) {
                return err;
            }
        }
    }
    else if( timederiv == 2 ) {
        voutput.resize(_vmimic.at(axis)->_accelfns.size());
        for(size_t i = 0; i < voutput.size(); ++i) {
            voutput[i] = _vmimic.at(axis)->_accelfns.at(i)->Eval(vdependentvalues.empty() ? NULL : &vdependentvalues[0]);
            int err = _vmimic.at(axis)->_accelfns.at(i)->EvalError();
            if( err ) {
                return err;
            }
        }
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT(_("timederiv %d not supported"),timederiv,ORE_InvalidArguments);
    }
    return 0;
}

bool KinBody::Joint::MIMIC::DOFFormat::operator <(const KinBody::Joint::MIMIC::DOFFormat& r) const
{
    return jointindex < r.jointindex || (jointindex == r.jointindex && (dofindex < r.dofindex || (dofindex == r.dofindex && axis < r.axis)));
}

bool KinBody::Joint::MIMIC::DOFFormat::operator ==(const KinBody::Joint::MIMIC::DOFFormat& r) const
{
    return jointindex == r.jointindex && dofindex == r.dofindex && axis == r.axis;
}

KinBody::JointPtr KinBody::Joint::MIMIC::DOFFormat::GetJoint(KinBody &parent) const
{
    int numjoints = (int)parent.GetJoints().size();
    return jointindex < numjoints ? parent.GetJoints().at(jointindex) : parent.GetPassiveJoints().at(jointindex-numjoints);
}

KinBody::JointConstPtr KinBody::Joint::MIMIC::DOFFormat::GetJoint(const KinBody &parent) const
{
    int numjoints = (int)parent.GetJoints().size();
    return jointindex < numjoints ? parent.GetJoints().at(jointindex) : parent.GetPassiveJoints().at(jointindex-numjoints);
}

void KinBody::Joint::SetFloatParameters(const std::string& key, const std::vector<dReal>& parameters)
{
    if( parameters.size() > 0 ) {
        _info._mapFloatParameters[key] = parameters;
    }
    else {
        _info._mapFloatParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointCustomParameters);
}

void KinBody::Joint::SetIntParameters(const std::string& key, const std::vector<int>& parameters)
{
    if( parameters.size() > 0 ) {
        _info._mapIntParameters[key] = parameters;
    }
    else {
        _info._mapIntParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointCustomParameters);
}

void KinBody::Joint::SetStringParameters(const std::string& key, const std::string& value)
{
    if( value.size() > 0 ) {
        _info._mapStringParameters[key] = value;
    }
    else {
        _info._mapStringParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointCustomParameters);
}

void KinBody::Joint::UpdateInfo()
{
    _info._vcurrentvalues.resize(0);
    GetValues(_info._vcurrentvalues);
}

void KinBody::Joint::ExtractInfo(KinBody::JointInfo& info) const
{
    info = _info;
    info._vcurrentvalues.resize(0);
    GetValues(info._vcurrentvalues);
}

UpdateFromInfoResult KinBody::Joint::UpdateFromInfo(const KinBody::JointInfo& info)
{
    BOOST_ASSERT(info._id == _info._id);
    bool isDiff = false;
    // _name
    if (GetName() != info._name) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _type
    if (GetType() != info._type) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _linkname0
    // _linkname1
    if (GetFirstAttached()->GetName() != info._linkname0 || GetSecondAttached()->GetName() != info._linkname1) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // TODO: maybe only need to call _ComputeInternalInformation?
    // _vanchor
    if (_info._vanchor != info._vanchor) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _vaxes
    if (_info._vaxes != info._vaxes) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _vcurrentvalues(not needed)

    // _vresolution
    for (int iaxis = 0; iaxis < GetDOF(); iaxis++) {
        if (GetResolution(iaxis) != info._vresolution[iaxis]) {
            return UFIR_RequireRemoveFromEnvironment;
        }
    }

    // _vmaxvel
    if (_info._vmaxvel != info._vmaxvel) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _vhardmaxvel
    std::vector<dReal> vHardVelocityLimits;
    GetHardVelocityLimits(vHardVelocityLimits);
    for (int iaxis = 0; iaxis < GetDOF(); iaxis++) {
        if (vHardVelocityLimits[iaxis] != info._vhardmaxvel[iaxis]) {
            vHardVelocityLimits[iaxis] = info._vhardmaxvel[iaxis];
            isDiff = true;
        }
    }
    if (isDiff) {
        SetHardVelocityLimits(vHardVelocityLimits);
        isDiff = false;
    }
    // _vmaxaccel
    if (_info._vmaxaccel != info._vmaxaccel) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _vhardmaxaccel
    std::vector<dReal> vHardAccelerationLimits;
    GetHardAccelerationLimits(vHardAccelerationLimits);
    for (int iaxis = 0; iaxis < GetDOF(); iaxis++) {
        if (vHardAccelerationLimits[iaxis] != info._vhardmaxaccel[iaxis]) {
            vHardAccelerationLimits[iaxis] = info._vhardmaxaccel[iaxis];
            isDiff = true;
        }
    }
    if (isDiff) {
        SetHardAccelerationLimits(vHardAccelerationLimits);
        isDiff = false;
    }

    // _vmaxjerk
    if (_info._vmaxjerk != info._vmaxjerk) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _vhardmaxjerk
    std::vector<dReal> vHardJerkLimits;
    GetHardJerkLimits(vHardJerkLimits);
    for (int iaxis = 0; iaxis < GetDOF(); iaxis++) {
        if (vHardJerkLimits[iaxis] != info._vhardmaxjerk[iaxis]) {
            vHardJerkLimits[iaxis] = info._vhardmaxjerk[iaxis];
            isDiff = true;
        }
    }
    if (isDiff) {
        SetHardJerkLimits(vHardJerkLimits);
        isDiff = false;
    }

    // _vmaxtorque
    std::vector<dReal> vMaxTorque;

    GetTorqueLimits(vMaxTorque);
    for (int iaxis = 0; iaxis < GetDOF(); iaxis++) {
        if (vMaxTorque[iaxis] != info._vmaxtorque[iaxis]) {
            vMaxTorque[iaxis] = info._vmaxtorque[iaxis];
            isDiff = true;
        }
    }
    if (isDiff) {
        SetTorqueLimits(vMaxTorque);
        isDiff = false;
    }

    // _vmaxinertia
    std::vector<dReal> vMaxInertialLimits;
    GetInertiaLimits(vMaxInertialLimits);
    for (int iaxis = 0; iaxis < GetDOF(); iaxis++) {
        if (vMaxInertialLimits[iaxis] != info._vmaxinertia[iaxis]) {
            vMaxInertialLimits[iaxis] = info._vmaxinertia[iaxis];
            isDiff = true;
        }
    }
    if (isDiff) {
        SetInertiaLimits(vMaxInertialLimits);
        isDiff = false;
    }

    // _vweights
    std::vector<dReal> vWeights;
    GetWeights(vWeights);

    for (int iaxis = 0; iaxis < GetDOF(); iaxis++) {
        if (vWeights[iaxis] != info._vweights[iaxis]) {
            vWeights[iaxis] = info._vweights[iaxis];
            isDiff = true;
        }
    }
    if (isDiff) {
        SetWeights(vWeights);
        isDiff = false;
    }

    // _voffsets
    if (_info._voffsets != info._voffsets) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _vlowerlimit
    // _vupperlimit
    std::vector<dReal> vUpperLimit;
    std::vector<dReal> vLowerLimit;
    GetLimits(vLowerLimit, vUpperLimit, false);
    for (int iaxis = 0; iaxis < GetDOF(); iaxis++) {
        if (vUpperLimit[iaxis] != info._vupperlimit[iaxis] || vLowerLimit[iaxis] != info._vlowerlimit[iaxis]) {
            SetLimits(vLowerLimit, vUpperLimit);
            break;
        }
    }

    for(int iaxis = 0; iaxis < GetDOF(); iaxis++) {
        if (info._vupperlimit[iaxis] != vUpperLimit[iaxis] || info._vlowerlimit[iaxis] != vLowerLimit[iaxis]) {
            SetLimits(vLowerLimit, vUpperLimit);
            break;
        }
    }

    // TODO: _trajfollow (not needed?)

    // _vmimic
    if (_info._vmimic != info._vmimic) {
        return UFIR_RequireRemoveFromEnvironment;
    }

    // _mapFloatParameters
    const std::map<std::string, std::vector<dReal> > floatParameters = GetFloatParameters();
    if (floatParameters != info._mapFloatParameters) {
        FOREACH(itParam, floatParameters) {
            SetFloatParameters(itParam->first, {}); // erase current parameters
        }

        FOREACH(itParam, info._mapFloatParameters) {
            SetFloatParameters(itParam->first, itParam->second);  // update with new info
        }
    }

    // _mapIntParameters
    const std::map<std::string, std::vector<int> > intParameters = GetIntParameters();
    if (intParameters != info._mapIntParameters) {
        FOREACH(itParam, intParameters) {
            SetIntParameters(itParam->first, {});
        }
        FOREACH(itParam, info._mapIntParameters) {
            SetIntParameters(itParam->first, itParam->second);
        }
    }

    // _mapStringParameters
    const std::map<std::string, std::string> stringParameters = GetStringParameters();
    if (stringParameters != info._mapStringParameters) {
        FOREACH(itParam, stringParameters) {
            SetStringParameters(itParam->first, {});
        }
        FOREACH(itParam, info._mapStringParameters) {
            SetStringParameters(itParam->first, itParam->second);
        }
    }

    // _infoElectricMotor
    if (!!_info._infoElectricMotor) {
        if (!!info._infoElectricMotor) {
            // both are not empty, compare the content
            if (*(_info._infoElectricMotor) != *(info._infoElectricMotor)) {
                *_info._infoElectricMotor = *info._infoElectricMotor;
            }
        }
        else {
            _info._infoElectricMotor.reset();
        }
    }
    else if (!!info._infoElectricMotor) {
        _info._infoElectricMotor.reset(new ElectricMotorActuatorInfo(*info._infoElectricMotor));
    }

    // _bIsCircular
    if (_info._bIsCircular != info._bIsCircular) {
        return UFIR_RequireReinitialize;
    }

    // _bIsActive
    if (_info._bIsActive != info._bIsActive) {
        return UFIR_RequireReinitialize;
    }

    // _controlMode, it will reset _jci_robotcontroller, _jci_io, _jci_externaldevice
    if (GetControlMode() != _info._controlMode) {
        return UFIR_RequireRemoveFromEnvironment;
    }
    return UFIR_Success;
}

void KinBody::Joint::serialize(std::ostream& o, int options) const
{
    if( options & SO_Kinematics ) {
        o << dofindex << " " << jointindex << " " << _info._type << " ";
        SerializeRound(o,_tRightNoOffset);
        SerializeRound(o,_tLeftNoOffset);
        for(int i = 0; i < GetDOF(); ++i) {
            SerializeRound3(o,_vaxes[i]);
            if( !!_vmimic.at(i) ) {
                FOREACHC(iteq,_vmimic.at(i)->_equations) {
                    o << *iteq << " ";
                }
            }
        }
        o << (!_attachedbodies[0] ? -1 : _attachedbodies[0]->GetIndex()) << " " << (_attachedbodies[1]->GetIndex()) << " ";
    }
    // in the past was including saving limits as part of SO_Dynamics, but given that limits change a lot when planning, should *not* include them as part of dynamics.
    if( options & SO_JointLimits ) {
        for(int i = 0; i < GetDOF(); ++i) {
            SerializeRound(o,_info._vmaxvel[i]);
            SerializeRound(o,_info._vmaxaccel[i]);
            SerializeRound(o,_info._vmaxjerk[i]);
            SerializeRound(o,_info._vmaxtorque[i]);
            SerializeRound(o,_info._vmaxinertia[i]);
            SerializeRound(o,_info._vlowerlimit[i]);
            SerializeRound(o,_info._vupperlimit[i]);
        }
    }
}

void KinBody::MimicInfo::Reset()
{
    FOREACH(iteq, _equations) {
        iteq->clear();
    }
}

void KinBody::MimicInfo::SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const
{
    orjson::SetJsonValueByKey(value, "equations", _equations, allocator);
}

void KinBody::MimicInfo::DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options)
{
    orjson::LoadJsonValueByKey(value, "equations", _equations);
}


}
