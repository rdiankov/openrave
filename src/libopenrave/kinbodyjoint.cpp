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

// the following constructor handles mapping from deprecated reference to the actual
// member, so need to disable deprecation warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
KinBody::JointInfo::JointInfo() :
    XMLReadable("joint"),
    _type(type),
    _name(name),
    _linkname0(parentLinkName),
    _linkname1(childLinkName),
    _vanchor(anchor),
    _vaxes(axes),
    _vcurrentvalues(currentValues),
    _vresolution(resolutions),
    _vmaxvel(maxVel),
    _vhardmaxvel(hardMaxVel),
    _vmaxaccel(maxAccel),
    _vmaxtorque(maxTorque),
    _vmaxinertia(maxInertia),
    _vweights(weights),
    _voffsets(offsets),
    _vlowerlimit(lowerLimit),
    _vupperlimit(upperLimit),
    _vmimic(mimic),
    _mapFloatParameters(floatParameters),
    _mapIntParameters(intParameters),
    _mapStringParameters(stringParameters),
    _infoElectricMotor(electricMotorActuator),
    _bIsCircular(isCircular),
    _bIsActive(isActive)
{
    type = JointNone;
    isActive = true;
    for(size_t i = 0; i < axes.size(); ++i) {
        axes[i] = Vector(0,0,1);
    }
    std::fill(resolutions.begin(), resolutions.end(), 0.02);
    std::fill(maxVel.begin(), maxVel.end(), 10);
    std::fill(hardMaxVel.begin(), hardMaxVel.end(), 10);
    std::fill(maxAccel.begin(), maxAccel.end(), 50);
    std::fill(maxTorque.begin(), maxTorque.end(), 0); // set max torque to 0 to notify the system that dynamics parameters might not be valid.
    std::fill(maxInertia.begin(), maxInertia.end(), 0);
    std::fill(weights.begin(), weights.end(), 1);
    std::fill(offsets.begin(), offsets.end(), 0);
    std::fill(lowerLimit.begin(), lowerLimit.end(), 0);
    std::fill(upperLimit.begin(), upperLimit.end(), 0);
    std::fill(isCircular.begin(), isCircular.end(), 0);
}
#pragma GCC diagnostic pop

KinBody::JointInfo::JointInfo(const KinBody::JointInfo& other) : KinBody::JointInfo()
{
    *this = other;
}

KinBody::JointInfo::~JointInfo()
{
}

KinBody::JointInfo& KinBody::JointInfo::operator=(const KinBody::JointInfo& other)
{
    sid = other.sid;
    type = other.type;
    name = other.name;
    parentLinkName = other.parentLinkName;
    childLinkName = other.childLinkName;
    anchor = other.anchor;
    axes = other.axes;
    currentValues = other.currentValues;
    resolutions = other.resolutions;
    maxVel = other.maxVel;
    hardMaxVel = other.hardMaxVel;
    maxAccel = other.maxAccel;
    maxTorque = other.maxTorque;
    maxInertia = other.maxInertia;
    weights = other.weights;
    offsets = other.offsets;
    lowerLimit = other.lowerLimit;
    upperLimit = other.upperLimit;
    mimic = other.mimic;
    floatParameters = other.floatParameters;
    intParameters = other.intParameters;
    stringParameters = other.stringParameters;
    electricMotorActuator = other.electricMotorActuator;
    isCircular = other.isCircular;
    isActive = other.isActive;

    return *this;
}

int KinBody::JointInfo::GetDOF() const
{
    if( type & KinBody::JointSpecialBit ) {
        switch(type) {
        case KinBody::JointHinge2:
        case KinBody::JointUniversal: return 2;
        case KinBody::JointSpherical: return 3;
        case KinBody::JointTrajectory: return 1;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("invalid joint type 0x%x"), type, ORE_Failed);
        }
    }
    return int(type & 0xf);
}

void KinBody::JointInfo::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, int options)
{
    int dof = GetDOF();

    RAVE_SERIALIZEJSON_ENSURE_OBJECT(value);

    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "sid", sid);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "name", name);
    switch (type) {
    case JointRevolute:
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "type", "revolute");
        break;
    case JointPrismatic:
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "type", "prismatic");
        break;
    case JointNone:
        break;
    default:
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "type", static_cast<int>(type));
        break;
    }

    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "parentLinkName", parentLinkName);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "childLinkName", childLinkName);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "anchor", anchor);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "axes", axes, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "currentValues", currentValues);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "resolutions", resolutions, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "maxVel", maxVel, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "hardMaxVel", hardMaxVel, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "maxAccel", maxAccel, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "maxTorque", maxTorque, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "maxInertia", maxInertia, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "weights", weights, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "offsets", offsets, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "lowerLimit", lowerLimit, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "upperLimit", upperLimit, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "isCircular", isCircular, dof);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "isActive", isActive);

    // TODO(jsonserialization)
    // _trajfollow

    if (mimic.size() > 0) {
        bool bfound = false;
        for (size_t i = 0; i < mimic.size() && i < (size_t)dof; ++i) {
            if (!!mimic[i]) {
                bfound = true;
                break;
            }
        }
        if (bfound) {
            rapidjson::Value mimics;
            RAVE_SERIALIZEJSON_CLEAR_ARRAY(mimics);
            for (size_t i = 0; i < mimic.size() && i < (size_t)dof; ++i) {
                rapidjson::Value mimicValue;
                mimic[i]->SerializeJSON(mimicValue, allocator, options);
                mimics.PushBack(mimicValue, allocator);
            }
            value.AddMember("mimic", mimics, allocator);
        }
    }

    if (floatParameters.size() > 0) {
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "floatParameters", floatParameters);
    }

    if (intParameters.size() > 0) {
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "intParameters", intParameters);
    }

    if (stringParameters.size() > 0) {
        RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "stringParameters", stringParameters);
    }

    if (!!electricMotorActuator) {
        rapidjson::Value electricMotorActuatorValue;
        electricMotorActuator->SerializeJSON(electricMotorActuatorValue, allocator, options);
        value.AddMember("electricMotorActuator", electricMotorActuatorValue, allocator);
    }
}

void KinBody::JointInfo::DeserializeJSON(const rapidjson::Value &value, const dReal fUnitScale)
{
    RAVE_DESERIALIZEJSON_ENSURE_OBJECT(value);

    RAVE_DESERIALIZEJSON_REQUIRED(value, "sid", sid);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "name", name);

    std::string typestr;
    RAVE_DESERIALIZEJSON_REQUIRED(value, "type", typestr);

    if (typestr == "revolute")
    {
        type = JointRevolute;
    }
    else if (typestr == "prismatic")
    {
        type = JointPrismatic;
    }
    else
    {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to deserialize json, unsupported joint type \"%s\"", typestr, ORE_InvalidArguments);
    }

    RAVE_DESERIALIZEJSON_REQUIRED(value, "parentLinkName", parentLinkName);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "childLinkName", childLinkName);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "anchor", anchor);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "axes", axes);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "currentValues", currentValues);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "resolutions", resolutions);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "maxVel", maxVel);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "hardMaxVel", hardMaxVel);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "maxAccel", maxAccel);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "maxTorque", maxTorque);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "maxInertia", maxInertia);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "weights", weights);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "offsets", offsets);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "lowerLimit", lowerLimit);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "upperLimit", upperLimit);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "isCircular", isCircular);
    RAVE_DESERIALIZEJSON_REQUIRED(value, "isActive", isActive);

    // multiply fUnitScale on maxVel, maxAccel, lowerLimit, upperLimit

    dReal fjointmult = fUnitScale;
    if( type == JointRevolute ) 
    {
        fjointmult = 1;
    }
    else if (type == JointPrismatic) 
    {
        fjointmult = fUnitScale;
    }
    for(size_t ic = 0; ic < axes.size(); ic++)
    {
        
        maxVel[ic] *= fjointmult;
        maxAccel[ic] *= fjointmult;
        lowerLimit[ic] *= fjointmult;
        upperLimit[ic] *= fjointmult;
    }
    

    boost::array<MimicInfoPtr, 3> newmimic;
    if (value.HasMember("mimic"))
    {
        RAVE_DESERIALIZEJSON_ENSURE_ARRAY(value["mimic"]);
        for (rapidjson::SizeType i = 0; i < value["mimic"].Size(); ++i) {
            MimicInfoPtr mimicinfo(new MimicInfo());
            mimicinfo->DeserializeJSON(value["mimic"][i]);
            newmimic[i] = mimicinfo;
        }
    }
    mimic = newmimic;

    RAVE_DESERIALIZEJSON_OPTIONAL(value, "floatParameters", floatParameters);
    RAVE_DESERIALIZEJSON_OPTIONAL(value, "intParameters", intParameters);
    RAVE_DESERIALIZEJSON_OPTIONAL(value, "stringParameters", stringParameters);

    if (value.HasMember("electricMotorActuator")) {
        ElectricMotorActuatorInfoPtr info(new ElectricMotorActuatorInfo());
        info->DeserializeJSON(value["electricMotorActuator"]);
        electricMotorActuator = info;
    }
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
    _info.type = type;
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
    return static_cast<bool>(_info.isCircular.at(iaxis));
}

bool KinBody::Joint::IsRevolute(int iaxis) const
{
    if( _info.type & KinBody::JointSpecialBit ) {
        return _info.type == KinBody::JointHinge2 || _info.type == KinBody::JointUniversal;
    }
    return !(_info.type&(1<<(4+iaxis)));
}

bool KinBody::Joint::IsPrismatic(int iaxis) const
{
    if( _info.type & KinBody::JointSpecialBit ) {
        return false;
    }
    return !!(_info.type&(1<<(4+iaxis)));
}

bool KinBody::Joint::IsStatic() const
{
    if( IsMimic() ) {
        bool bstatic = true;
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
        if( _info.lowerLimit.at(i) < _info.upperLimit.at(i) ) {
            return false;
        }
    }
    return true;
}

void KinBody::Joint::GetValues(vector<dReal>& pValues, bool bAppend) const
{
    OPENRAVE_ASSERT_FORMAT0(_bInitialized, "joint not initialized",ORE_NotInitialized);
    if( !bAppend ) {
        pValues.resize(0);
    }
    if( GetDOF() == 1 ) {
        pValues.push_back(GetValue(0));
        return;
    }
    dReal f;
    Transform tjoint = _tinvLeft * _attachedbodies[0]->GetTransform().inverse() * _attachedbodies[1]->GetTransform() * _tinvRight;
    if( _info.type & KinBody::JointSpecialBit ) {
        switch(_info.type) {
        case KinBody::JointHinge2: {
            Vector axis1cur = tjoint.rotate(_vaxes[0]), axis2cur = tjoint.rotate(_vaxes[1]);
            Vector vec1, vec2, vec3;
            vec1 = (_vaxes[1] - _vaxes[0].dot3(_vaxes[1])*_vaxes[0]).normalize();
            vec2 = (axis2cur - _vaxes[0].dot3(axis2cur)*_vaxes[0]).normalize();
            vec3 = _vaxes[0].cross(vec1);
            f = 2.0*RaveAtan2(vec3.dot3(vec2), vec1.dot3(vec2));
            pValues.push_back(GetClosestValueAlongCircle(_info.offsets[0]+f, _doflastsetvalues[0]));
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
            pValues.push_back(GetClosestValueAlongCircle(_info.offsets[1]+f, _doflastsetvalues[1]));
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
            throw OPENRAVE_EXCEPTION_FORMAT(_("unknown joint type 0x%x"), _info.type, ORE_Failed);
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
                pValues.push_back(GetClosestValueAlongCircle(_info.offsets[i]+f, _doflastsetvalues[i]));
            }
            else { // prismatic
                f = tjoint.trans.x*vaxis.x+tjoint.trans.y*vaxis.y+tjoint.trans.z*vaxis.z;
                pValues.push_back(_info.offsets[i]+f);
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
    dReal f;
    Transform tjoint = _tinvLeft * _attachedbodies[0]->GetTransform().inverse() * _attachedbodies[1]->GetTransform() * _tinvRight;
    if( _info.type & KinBody::JointSpecialBit ) {
        switch(_info.type) {
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
                return GetClosestValueAlongCircle(_info.offsets[0]+f, _doflastsetvalues[0]);
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
                return GetClosestValueAlongCircle(_info.offsets[1]+f, _doflastsetvalues[1]);
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
        if( _info.type == KinBody::JointPrismatic ) {
            return _info.offsets[0]+(tjoint.trans.x*_vaxes[0].x+tjoint.trans.y*_vaxes[0].y+tjoint.trans.z*_vaxes[0].z);
        }
        else if( _info.type == KinBody::JointRevolute ) {
            f = 2.0f*RaveAtan2(tjoint.rot.y*_vaxes[0].x+tjoint.rot.z*_vaxes[0].y+tjoint.rot.w*_vaxes[0].z, tjoint.rot.x);
            // expect values to be within -PI to PI range
            if( f < -PI ) {
                f += 2*PI;
            }
            else if( f > PI ) {
                f -= 2*PI;
            }
            return GetClosestValueAlongCircle(_info.offsets[0]+f, _doflastsetvalues[0]);
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
                    return GetClosestValueAlongCircle(_info.offsets[i]+f, _doflastsetvalues[i]);
                }
            }
            else { // prismatic
                f = tjoint.trans.x*vaxis.x+tjoint.trans.y*vaxis.y+tjoint.trans.z*vaxis.z;
                if( i == iaxis ) {
                    return _info.offsets[i]+f;
                }
                if( i+1 < GetDOF() ) {
                    tjoint.trans -= vaxis*f;
                }
            }
        }
    }
    throw OPENRAVE_EXCEPTION_FORMAT(_("unknown joint type 0x%x axis %d\n"), _info.type%iaxis, ORE_Failed);
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
        pVelocities.push_back(GetVelocity(0));
        return;
    }
    const Transform& linkparenttransform = _attachedbodies[0]->_info.transform;
    const Transform& linkchildtransform = _attachedbodies[1]->_info.transform;
    Vector quatdelta = quatMultiply(linkparenttransform.rot,_tLeft.rot);
    Vector quatdeltainv = quatInverse(quatdelta);
    if( _info.type & KinBody::JointSpecialBit ) {
        switch(_info.type) {
        case KinBody::JointSpherical: {
            Vector v = quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second);
            pVelocities.push_back(v.x);
            pVelocities.push_back(v.y);
            pVelocities.push_back(v.z);
            break;
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("unknown joint type 0x%x"), _info.type, ORE_InvalidArguments);
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
    const Transform& linkparenttransform = _attachedbodies[0]->_info.transform;
    const Transform& linkchildtransform = _attachedbodies[1]->_info.transform;
    Vector quatdelta = quatMultiply(linkparenttransform.rot,_tLeft.rot);
    Vector quatdeltainv = quatInverse(quatdelta);
    if( _info.type & KinBody::JointSpecialBit ) {
        switch(_info.type) {
        case KinBody::JointSpherical: {
            Vector v = quatRotate(quatdeltainv,linkchildvelocity.second-linkparentvelocity.second);
            return v[axis];
        }
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("unknown joint type 0x%x"), _info.type, ORE_InvalidArguments);
        }
    }
    else {
        if( _info.type == KinBody::JointPrismatic ) {
            return _vaxes[0].dot3(quatRotate(quatdeltainv,linkchildvelocity.first-linkparentvelocity.first-linkparentvelocity.second.cross(linkchildtransform.trans-linkparenttransform.trans)));
        }
        else if( _info.type == KinBody::JointRevolute ) {
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
    throw OPENRAVE_EXCEPTION_FORMAT(_("unsupported joint type 0x%x"), _info.type, ORE_InvalidArguments);
}

Vector KinBody::Joint::GetAnchor() const
{
    return _attachedbodies[0]->GetTransform() * _tLeft.trans;
}

Vector KinBody::Joint::GetAxis(int iaxis) const
{
    return _attachedbodies[0]->GetTransform().rotate(_tLeft.rotate(_vaxes.at(iaxis)));
}

void KinBody::Joint::_ComputeInternalInformation(LinkPtr plink0, LinkPtr plink1, const Vector& vanchorraw, const std::vector<Vector>& vaxes, const std::vector<dReal>& vcurrentvalues)
{
    OPENRAVE_ASSERT_OP_FORMAT(!!plink0,&&,!!plink1, "one or more attached _attachedbodies are invalid for joint %s", GetName(),ORE_InvalidArguments);
    for(int i = 0; i < GetDOF(); ++i) {
        OPENRAVE_ASSERT_OP_FORMAT(_info.maxVel[i], >=, 0, "joint %s[%d] max velocity is invalid",_info.name%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(_info.maxAccel[i], >=, 0, "joint %s[%d] max acceleration is invalid",_info.name%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(_info.maxTorque[i], >=, 0, "joint %s[%d] max torque is invalid",_info.name%i, ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT(_info.maxInertia[i], >=, 0, "joint %s[%d] max inertia is invalid",_info.name%i, ORE_InvalidArguments);
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
        _info.axes[i] = _vaxes[i];
    }
    _info.anchor = vanchor;

    tbody0 = _attachedbodies[0]->GetTransform();
    tbody1 = _attachedbodies[1]->GetTransform();
    trel = tbody0.inverse() * tbody1;
    _tLeft = Transform();
    _tLeftNoOffset = Transform();
    _tRight = Transform();
    _tRightNoOffset = Transform();

    if( _info.type & KinBody::JointSpecialBit ) {
        switch(_info.type) {
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
            throw OPENRAVE_EXCEPTION_FORMAT(_("unrecognized joint type 0x%x"), _info.type, ORE_InvalidArguments);
        }
        _tLeftNoOffset = _tLeft;
        _tRightNoOffset = _tRight;
    }
    else {
        OPENRAVE_ASSERT_OP((int)vaxes.size(),==,GetDOF());
        _tLeftNoOffset.trans = vanchor;
        _tRightNoOffset.trans = -vanchor;
        _tRightNoOffset = _tRightNoOffset * trel;
        if( GetDOF() == 1 ) {
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
            toffset.rot = quatFromAxisAngle(_vaxes[0], _info.offsets[0]);
        }
        else {
            toffset.trans = _vaxes[0]*_info.offsets[0];
        }
        _tLeft = _tLeftNoOffset * toffset;
        _tRight = _tRightNoOffset;
        if( GetDOF() > 1 ) {
            // right multiply by the offset of the last axis, might be buggy?
            if( IsRevolute(GetDOF()-1) ) {
                _tRight = matrixFromAxisAngle(_vaxes[GetDOF()-1], _info.offsets[GetDOF()-1]) * _tRight;
            }
            else {
                _tRight.trans += _vaxes[GetDOF()-1]*_info.offsets[GetDOF()-1];
            }
        }
    }

    if( vcurrentvalues.size() > 0 ) {
        // see if any joints have offsets
        Transform toffset;
        if( _info.type == KinBody::JointTrajectory ) {
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
        else if( !(_info.type&KinBody::JointSpecialBit) || _info.type == KinBody::JointUniversal || _info.type == KinBody::JointHinge2 ) {
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

    _vcircularlowerlimit = _info.lowerLimit;
    _vcircularupperlimit = _info.upperLimit;
    for(int i = 0; i < GetDOF(); ++i) {
        if( IsCircular(i) ) {
            // can rotate forever, so don't limit it. Unfortunately if numbers are too big precision will start getting lost
            _info.lowerLimit.at(i) = -1e4;
            _info.upperLimit.at(i) = 1e4;
        }
    }

    if( !!_attachedbodies[0] ) {
        _info.parentLinkName = _attachedbodies[0]->GetName();
    }
    else {
        _info.parentLinkName.clear();
    }
    if( !!_attachedbodies[1] ) {
        _info.childLinkName = _attachedbodies[1]->GetName();
    }
    else {
        _info.childLinkName.clear();
    }
    _info.currentValues = vcurrentvalues;

    _bInitialized = true;

    if( _attachedbodies[1]->IsStatic() && !IsStatic() ) {
        RAVELOG_WARN(str(boost::format("joint %s: all attached links are static, but joint is not!\n")%GetName()));
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

Vector KinBody::Joint::GetInternalHierarchyAxis(int iaxis) const
{
    return _vaxes.at(iaxis);
}

Transform KinBody::Joint::GetInternalHierarchyLeftTransform() const
{
    OPENRAVE_ASSERT_FORMAT0(_bInitialized, "joint not initialized",ORE_NotInitialized);
    return _tLeftNoOffset;
}

Transform KinBody::Joint::GetInternalHierarchyRightTransform() const
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
        vLowerLimit.push_back(_info.lowerLimit[i]);
        vUpperLimit.push_back(_info.upperLimit[i]);
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetLimit(int iaxis) const
{
    return make_pair(_info.lowerLimit.at(iaxis),_info.upperLimit.at(iaxis));
}

void KinBody::Joint::SetLimits(const std::vector<dReal>& vLowerLimit, const std::vector<dReal>& vUpperLimit)
{
    bool bChanged = false;
    for(int i = 0; i < GetDOF(); ++i) {
        if( _info.lowerLimit[i] != vLowerLimit.at(i) || _info.upperLimit[i] != vUpperLimit.at(i) ) {
            bChanged = true;
            _info.lowerLimit[i] = vLowerLimit.at(i);
            _info.upperLimit[i] = vUpperLimit.at(i);
            if( IsRevolute(i) && !IsCircular(i) ) {
                // TODO, necessary to set wrap?
                if( _info.lowerLimit[i] < -PI || _info.upperLimit[i] > PI) {
                    SetWrapOffset(0.5f * (_info.lowerLimit.at(i) + _info.upperLimit.at(i)),i);
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
        vlower.push_back(-_info.maxVel[i]);
        vupper.push_back(_info.maxVel[i]);
    }
}

void KinBody::Joint::GetVelocityLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info.maxVel[i]);
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetVelocityLimit(int iaxis) const
{
    return make_pair(-_info.maxVel.at(iaxis), _info.maxVel.at(iaxis));
}

void KinBody::Joint::SetVelocityLimits(const std::vector<dReal>& vmaxvel)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info.maxVel[i] = vmaxvel.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetAccelerationLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info.maxAccel[i]);
    }
}

dReal KinBody::Joint::GetAccelerationLimit(int iaxis) const
{
    return _info.maxAccel.at(iaxis);
}

void KinBody::Joint::SetAccelerationLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info.maxAccel[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetTorqueLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info.maxTorque[i]);
    }
}

void KinBody::Joint::SetTorqueLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info.maxTorque[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::GetInertiaLimits(std::vector<dReal>& vmax, bool bAppend) const
{
    if( !bAppend ) {
        vmax.resize(0);
    }
    for(int i = 0; i < GetDOF(); ++i) {
        vmax.push_back(_info.maxInertia[i]);
    }
}

void KinBody::Joint::SetInertiaLimits(const std::vector<dReal>& vmax)
{
    for(int i = 0; i < GetDOF(); ++i) {
        _info.maxInertia[i] = vmax.at(i);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointAccelerationVelocityTorqueLimits);
}

void KinBody::Joint::SetWrapOffset(dReal newoffset, int iaxis)
{
    if( _info.offsets.at(iaxis) != newoffset ) {
        _info.offsets.at(iaxis) = newoffset;
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
        resolutions.push_back(_info.resolutions[i]);
    }
}

dReal KinBody::Joint::GetResolution(int iaxis) const
{
    return _info.resolutions.at(iaxis);
}

void KinBody::Joint::SetResolution(dReal resolution, int iaxis)
{
    _info.resolutions.at(iaxis) = resolution;
    GetParent()->_PostprocessChangedParameters(Prop_JointProperties);
}

void KinBody::Joint::GetWeights(std::vector<dReal>& weights, bool bAppend) const
{
    if( !bAppend ) {
        weights.resize(GetDOF());
    }
    for(int i = 0; i < GetDOF(); ++i) {
        weights.push_back(_info.weights[i]);
    }
}

dReal KinBody::Joint::GetWeight(int iaxis) const
{
    return _info.weights.at(iaxis);
}

void KinBody::Joint::SetWeights(const std::vector<dReal>& vweights)
{
    for(int i = 0; i < GetDOF(); ++i) {
        OPENRAVE_ASSERT_OP(vweights.at(i),>,0);
        _info.weights[i] = vweights.at(i);
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
    if( !_info.electricMotorActuator ) {
        return _info.maxTorque.at(iaxis);
    }
    else {
        if( _info.electricMotorActuator->maxSpeedTorquePoints.size() > 0 ) {
            if( _info.electricMotorActuator->maxSpeedTorquePoints.size() == 1 ) {
                // doesn't matter what the velocity is
                return _info.electricMotorActuator->maxSpeedTorquePoints.at(0).second*_info.electricMotorActuator->gearRatio;
            }

            dReal velocity = RaveFabs(GetVelocity(iaxis));
            dReal revolutionsPerSecond = _info.electricMotorActuator->gearRatio * velocity;
            if( IsRevolute(iaxis) ) {
                revolutionsPerSecond /= 2*M_PI;
            }

            if( revolutionsPerSecond <= _info.electricMotorActuator->maxSpeedTorquePoints.at(0).first ) {
                return _info.electricMotorActuator->maxSpeedTorquePoints.at(0).second*_info.electricMotorActuator->gearRatio;
            }

            for(size_t i = 1; i < _info.electricMotorActuator->maxSpeedTorquePoints.size(); ++i) {
                if( revolutionsPerSecond <= _info.electricMotorActuator->maxSpeedTorquePoints.at(i).first ) {
                    // linearly interpolate to get the desired torque
                    dReal rps0 = _info.electricMotorActuator->maxSpeedTorquePoints.at(i-1).first;
                    dReal torque0 = _info.electricMotorActuator->maxSpeedTorquePoints.at(i-1).second;
                    dReal rps1 = _info.electricMotorActuator->maxSpeedTorquePoints.at(i).first;
                    dReal torque1 = _info.electricMotorActuator->maxSpeedTorquePoints.at(i).second;
                    if( rps1 - rps0 <= g_fEpsilonLinear ) {
                        return torque1*_info.electricMotorActuator->gearRatio;
                    }

                    return ((revolutionsPerSecond - rps0)/(rps1 - rps0)*(torque1-torque0) + torque0)*_info.electricMotorActuator->gearRatio;
                }
            }

            // revolutionsPerSecond is huge, return the last point
            return _info.electricMotorActuator->maxSpeedTorquePoints.back().second*_info.electricMotorActuator->gearRatio;
        }
        else {
            return _info.electricMotorActuator->maxInstantaneousTorque*_info.electricMotorActuator->gearRatio;
        }
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetInstantaneousTorqueLimits(int iaxis) const
{
    if( !_info.electricMotorActuator ) {
        return std::make_pair(-_info.maxTorque.at(iaxis), _info.maxTorque.at(iaxis));
    }
    else {
        if( _info.electricMotorActuator->maxSpeedTorquePoints.size() > 0 ) {
            dReal fMaxTorqueAtZeroSpeed = _info.electricMotorActuator->maxSpeedTorquePoints.at(0).second*_info.electricMotorActuator->gearRatio;
            if( _info.electricMotorActuator->maxSpeedTorquePoints.size() == 1 ) {
                // doesn't matter what the velocity is
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            dReal rawvelocity = GetVelocity(iaxis);
            dReal velocity = RaveFabs(rawvelocity);
            dReal revolutionsPerSecond = _info.electricMotorActuator->gearRatio * velocity;
            if( IsRevolute(iaxis) ) {
                revolutionsPerSecond /= 2*M_PI;
            }

            if( revolutionsPerSecond <= _info.electricMotorActuator->maxSpeedTorquePoints.at(0).first ) {
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            for(size_t i = 1; i < _info.electricMotorActuator->maxSpeedTorquePoints.size(); ++i) {
                if( revolutionsPerSecond <= _info.electricMotorActuator->maxSpeedTorquePoints.at(i).first ) {
                    // linearly interpolate to get the desired torque
                    dReal rps0 = _info.electricMotorActuator->maxSpeedTorquePoints.at(i-1).first;
                    dReal torque0 = _info.electricMotorActuator->maxSpeedTorquePoints.at(i-1).second;
                    dReal rps1 = _info.electricMotorActuator->maxSpeedTorquePoints.at(i).first;
                    dReal torque1 = _info.electricMotorActuator->maxSpeedTorquePoints.at(i).second;

                    dReal finterpolatedtorque;
                    if( rps1 - rps0 <= g_fEpsilonLinear ) {
                        finterpolatedtorque = torque1*_info.electricMotorActuator->gearRatio;
                    }
                    else {
                        finterpolatedtorque = ((revolutionsPerSecond - rps0)/(rps1 - rps0)*(torque1-torque0) + torque0)*_info.electricMotorActuator->gearRatio;
                    }

                    // due to back emf, the deceleration magnitude is less than acceleration?
                    if( rawvelocity > 0 ) {
                        return std::make_pair(-0.9*finterpolatedtorque, finterpolatedtorque);
                    }
                    else {
                        return std::make_pair(-finterpolatedtorque, 0.9*finterpolatedtorque);
                    }
                }
            }

            // due to back emf, the deceleration magnitude is less than acceleration?
            // revolutionsPerSecond is huge, return the last point
            dReal f = _info.electricMotorActuator->maxSpeedTorquePoints.back().second*_info.electricMotorActuator->gearRatio;
            if( rawvelocity > 0 ) {
                return std::make_pair(-0.9*f, f);
            }
            else {
                return std::make_pair(-f, 0.9*f);
            }
        }
        else {
            dReal f = _info.electricMotorActuator->maxInstantaneousTorque*_info.electricMotorActuator->gearRatio;
            return std::make_pair(-f, f);
        }
    }
}

std::pair<dReal, dReal> KinBody::Joint::GetNominalTorqueLimits(int iaxis) const
{
    if( !_info.electricMotorActuator ) {
        return std::make_pair(-_info.maxTorque.at(iaxis), _info.maxTorque.at(iaxis));
    }
    else {
        if( _info.electricMotorActuator->nominalSpeedTorquePoints.size() > 0 ) {
            dReal fMaxTorqueAtZeroSpeed = _info.electricMotorActuator->nominalSpeedTorquePoints.at(0).second*_info.electricMotorActuator->gearRatio;
            if( _info.electricMotorActuator->nominalSpeedTorquePoints.size() == 1 ) {
                // doesn't matter what the velocity is
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            dReal rawvelocity = GetVelocity(iaxis);
            dReal velocity = RaveFabs(rawvelocity);
            dReal revolutionsPerSecond = _info.electricMotorActuator->gearRatio * velocity;
            if( IsRevolute(iaxis) ) {
                revolutionsPerSecond /= 2*M_PI;
            }

            if( revolutionsPerSecond <= _info.electricMotorActuator->nominalSpeedTorquePoints.at(0).first ) {
                return std::make_pair(-fMaxTorqueAtZeroSpeed, fMaxTorqueAtZeroSpeed);
            }

            for(size_t i = 1; i < _info.electricMotorActuator->nominalSpeedTorquePoints.size(); ++i) {
                if( revolutionsPerSecond <= _info.electricMotorActuator->nominalSpeedTorquePoints.at(i).first ) {
                    // linearly interpolate to get the desired torque
                    dReal rps0 = _info.electricMotorActuator->nominalSpeedTorquePoints.at(i-1).first;
                    dReal torque0 = _info.electricMotorActuator->nominalSpeedTorquePoints.at(i-1).second;
                    dReal rps1 = _info.electricMotorActuator->nominalSpeedTorquePoints.at(i).first;
                    dReal torque1 = _info.electricMotorActuator->nominalSpeedTorquePoints.at(i).second;

                    dReal finterpolatedtorque;
                    if( rps1 - rps0 <= g_fEpsilonLinear ) {
                        finterpolatedtorque = torque1*_info.electricMotorActuator->gearRatio;
                    }
                    else {
                        finterpolatedtorque = ((revolutionsPerSecond - rps0)/(rps1 - rps0)*(torque1-torque0) + torque0)*_info.electricMotorActuator->gearRatio;
                    }

                    // due to back emf, the deceleration magnitude is less than acceleration?
                    if( rawvelocity > 0 ) {
                        return std::make_pair(-0.9*finterpolatedtorque, finterpolatedtorque);
                    }
                    else {
                        return std::make_pair(-finterpolatedtorque, 0.9*finterpolatedtorque);
                    }
                }
            }

            // due to back emf, the deceleration magnitude is less than acceleration?
            // revolutionsPerSecond is huge, return the last point
            dReal f = _info.electricMotorActuator->nominalSpeedTorquePoints.back().second*_info.electricMotorActuator->gearRatio;
            if( rawvelocity > 0 ) {
                return std::make_pair(-0.9*f, f);
            }
            else {
                return std::make_pair(-f, 0.9*f);
            }
        }
        else {
            dReal f = _info.electricMotorActuator->nominalTorque*_info.electricMotorActuator->gearRatio;
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
            JointConstPtr pjoint = itdofformat->GetJoint(parent);
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

void KinBody::Joint::GetMimicDOFIndices(std::vector<int>& v_vmimicdofs, int iaxis) const
{
    OPENRAVE_ASSERT_FORMAT(!!_vmimic.at(iaxis), "joint %s axis %d is not mimic", GetName()%iaxis,ORE_InvalidArguments);
    v_vmimicdofs.resize(0);
    FOREACHC(it, _vmimic.at(iaxis)->_vmimicdofs) {
        std::vector<int>::iterator itinsert = std::lower_bound(v_vmimicdofs.begin(),v_vmimicdofs.end(), it->dofindex);
        if((itinsert == v_vmimicdofs.end())||(*itinsert != it->dofindex)) {
            v_vmimicdofs.insert(itinsert,it->dofindex);
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
    if( !_info.mimic.at(iaxis) ) {
        _info.mimic.at(iaxis).reset(new MimicInfo());
    }
    _info.mimic.at(iaxis)->_equations = mimic->_equations;

    OpenRAVEFunctionParserRealPtr posfn = CreateJointFunctionParser();
    mimic->_posfn = posfn;
    // because openrave joint names can hold symbols like '-' and '.' can affect the equation, so first do a search and replace
    std::vector< std::pair<std::string, std::string> > jointnamepairs; jointnamepairs.reserve(parent->GetJoints().size());
    FOREACHC(itjoint,parent->GetJoints()) {
        if( (*itjoint)->GetName().size() > 0 ) {
            std::string newname = str(boost::format("joint%d")%(*itjoint)->GetJointIndex());
            jointnamepairs.push_back(make_pair((*itjoint)->GetName(),newname));
        }
    }
    size_t index = parent->GetJoints().size();
    FOREACHC(itjoint,parent->GetPassiveJoints()) {
        if( (*itjoint)->GetName().size() > 0 ) {
            std::string newname = str(boost::format("joint%d")%index);
            jointnamepairs.push_back(make_pair((*itjoint)->GetName(),newname));
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
        JointPtr pjoint = dofformat.GetJoint(parent);
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
                RAVELOG_WARN(str(boost::format("invalid equation syntax '%s' for joint %s")%sequation%_info.name));
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
                RAVELOG_WARN(str(boost::format("SetMimicEquations: missing variable %s from partial derivatives of joint %s!")%mapinvnames[resultVars[j]]%_info.name));
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
        vpartials.push_back(make_pair(dofindex+iaxis,1.0));
        return;
    }
    OPENRAVE_ASSERT_FORMAT(!!_vmimic.at(iaxis), "cannot compute partial velocities of joint %s", _info.name, ORE_Failed);
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
                    vtempvalues.push_back(itdofformat->GetJoint(parent)->GetValue(itdofformat->axis));
                }
            }
            dReal fvel = _vmimic[iaxis]->_velfns.at(itmimicdof->dofformatindex)->Eval(vtempvalues.empty() ? NULL : &vtempvalues[0]);
            const MIMIC::DOFFormat& dofformat = _vmimic[iaxis]->_vdofformat.at(itmimicdof->dofformatindex);
            if( dofformat.GetJoint(parent)->IsMimic(dofformat.axis) ) {
                dofformat.GetJoint(parent)->_ComputePartialVelocities(vtemppartials,dofformat.axis,mapcachedpartials);
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
                vpartials.push_back(make_pair(itmimicdof->dofindex, fvel));
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
                vpartials.push_back(make_pair(itmimicdof->dofindex, it->second));
            }
        }
    }
}

int KinBody::Joint::_Eval(int axis, uint32_t timederiv, const std::vector<dReal>& vdependentvalues, std::vector<dReal>& voutput)
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

KinBody::JointPtr KinBody::Joint::MIMIC::DOFFormat::GetJoint(KinBodyPtr parent) const
{
    int numjoints = (int)parent->GetJoints().size();
    return jointindex < numjoints ? parent->GetJoints().at(jointindex) : parent->GetPassiveJoints().at(jointindex-numjoints);
}

KinBody::JointConstPtr KinBody::Joint::MIMIC::DOFFormat::GetJoint(KinBodyConstPtr parent) const
{
    int numjoints = (int)parent->GetJoints().size();
    return jointindex < numjoints ? parent->GetJoints().at(jointindex) : parent->GetPassiveJoints().at(jointindex-numjoints);
}

void KinBody::Joint::SetFloatParameters(const std::string& key, const std::vector<dReal>& parameters)
{
    if( parameters.size() > 0 ) {
        _info.floatParameters[key] = parameters;
    }
    else {
        _info.floatParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointCustomParameters);
}

void KinBody::Joint::SetIntParameters(const std::string& key, const std::vector<int>& parameters)
{
    if( parameters.size() > 0 ) {
        _info.intParameters[key] = parameters;
    }
    else {
        _info.intParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointCustomParameters);
}

void KinBody::Joint::SetStringParameters(const std::string& key, const std::string& value)
{
    if( value.size() > 0 ) {
        _info.stringParameters[key] = value;
    }
    else {
        _info.stringParameters.erase(key);
    }
    GetParent()->_PostprocessChangedParameters(Prop_JointCustomParameters);
}

void KinBody::Joint::UpdateInfo()
{
    _info.currentValues.resize(0);
    GetValues(_info.currentValues);
}

void KinBody::Joint::serialize(std::ostream& o, int options) const
{
    if( options & SO_Kinematics ) {
        o << dofindex << " " << jointindex << " " << _info.type << " ";
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
    if( options & SO_Dynamics ) {
        for(int i = 0; i < GetDOF(); ++i) {
            SerializeRound(o,_info.maxVel[i]);
            SerializeRound(o,_info.maxAccel[i]);
            SerializeRound(o,_info.maxTorque[i]);
            SerializeRound(o,_info.maxInertia[i]);
            SerializeRound(o,_info.lowerLimit[i]);
            SerializeRound(o,_info.upperLimit[i]);
        }
    }
}

void KinBody::Joint::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, int options)
{
    UpdateInfo();
    _info.SerializeJSON(value, allocator, options);
}

void KinBody::Joint::DeserializeJSON(const rapidjson::Value &value)
{
    _info.DeserializeJSON(value);
}

void KinBody::MimicInfo::SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, int options)
{
    RAVE_SERIALIZEJSON_ENSURE_OBJECT(value);
    RAVE_SERIALIZEJSON_ADDMEMBER(value, allocator, "equations", _equations);
}

void KinBody::MimicInfo::DeserializeJSON(const rapidjson::Value &value)
{
    RAVE_DESERIALIZEJSON_ENSURE_OBJECT(value);

    RAVE_DESERIALIZEJSON_REQUIRED(value, "equations", _equations);
}

}
