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
#include "openravepy_kinbody.h"

namespace openravepy {

using py::object;
using py::extract;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::no_init;
using py::bases;
using py::init;
using py::scope;
using py::args;
using py::return_value_policy;
using py::copy_const_reference;
using py::docstring_options;
using py::pickle_suite;
using py::manage_new_object;
using py::def;
namespace numeric = py::numeric;

template <typename T>
object GetCustomParameters(const std::map<std::string, std::vector<T> >& parameters, object oname=object(), int index=-1)
{
    if( IS_PYTHONOBJECT_NONE(oname) ) {
        py::dict oparameters;
        FOREACHC(it, parameters) {
            oparameters[it->first] = toPyArray(it->second);
        }
        return oparameters;
    }
    std::string name = py::extract<std::string>(oname);
    typename std::map<std::string, std::vector<T> >::const_iterator it = parameters.find(name);
    if( it != parameters.end() ) {
        if( index >= 0 ) {
            if( (size_t)index < it->second.size() ) {
                return object(it->second.at(index));
            }
            else {
                return object();
            }
        }
        return toPyArray(it->second);
    }
    return object();
}

class PySideWall
{
public:
    PySideWall() {
        transf = ReturnTransform(Transform());
        vExtents = toPyVector3(Vector());
        type = 0;
    }
    PySideWall(const KinBody::GeometryInfo::SideWall& sidewall) {
        transf = ReturnTransform(sidewall.transf);
        vExtents = toPyVector3(sidewall.vExtents);
        type = sidewall.type;
    }
    void Get(KinBody::GeometryInfo::SideWall& sidewall) {
        sidewall.transf = ExtractTransform(transf);
        sidewall.vExtents = ExtractVector<dReal>(vExtents);
        sidewall.type = static_cast<KinBody::GeometryInfo::SideWallType>(type);
    }

    object transf, vExtents;
    int type;
};

class PyGeometryInfo
{
public:
    PyGeometryInfo() {
        _t = ReturnTransform(Transform());
        _vGeomData = toPyVector4(Vector());
        _vGeomData2 = toPyVector4(Vector());
        _vGeomData3 = toPyVector4(Vector());
        _vGeomData4 = toPyVector4(Vector());
        _vDiffuseColor = toPyVector3(Vector(1,1,1));
        _vAmbientColor = toPyVector3(Vector(0,0,0));
        _type = GT_None;
        _fTransparency = 0;
        _vRenderScale = toPyVector3(Vector(1,1,1));
        _vCollisionScale = toPyVector3(Vector(1,1,1));
        _bVisible = true;
        _bModifiable = true;
        _vSideWalls = py::list();
    }
    PyGeometryInfo(const KinBody::GeometryInfo& info) {
        Init(info);
    }

    void Init(const KinBody::GeometryInfo& info) {
        _t = ReturnTransform(info._t);
        _vGeomData = toPyVector4(info._vGeomData);
        _vGeomData2 = toPyVector4(info._vGeomData2);
        _vGeomData3 = toPyVector4(info._vGeomData3);
        _vGeomData4 = toPyVector4(info._vGeomData4);

        _vSideWalls = py::list();
        for (size_t i = 0; i < info._vSideWalls.size(); ++i) {
            _vSideWalls.append(PySideWall(info._vSideWalls[i]));
        }

        _vDiffuseColor = toPyVector3(info._vDiffuseColor);
        _vAmbientColor = toPyVector3(info._vAmbientColor);
        _meshcollision = toPyTriMesh(info._meshcollision);
        _type = info._type;
        _name = ConvertStringToUnicode(info._name);
        _filenamerender = ConvertStringToUnicode(info._filenamerender);
        _filenamecollision = ConvertStringToUnicode(info._filenamecollision);
        _vRenderScale = toPyVector3(info._vRenderScale);
        _vCollisionScale = toPyVector3(info._vCollisionScale);
        _fTransparency = info._fTransparency;
        _bVisible = info._bVisible;
        _bModifiable = info._bModifiable;
        //TODO
        //_mapExtraGeometries = info. _mapExtraGeometries;
    }

    object ComputeInnerEmptyVolume()
    {
        Transform tInnerEmptyVolume;
        Vector abInnerEmptyExtents;
        KinBody::GeometryInfoPtr pgeominfo = GetGeometryInfo();
        if( pgeominfo->ComputeInnerEmptyVolume(tInnerEmptyVolume, abInnerEmptyExtents) ) {
            return py::make_tuple(ReturnTransform(tInnerEmptyVolume), toPyVector3(abInnerEmptyExtents));
        }
        return py::make_tuple(object(), object());
    }

    object ComputeAABB(object otransform) {
        KinBody::GeometryInfoPtr pgeominfo = GetGeometryInfo();
        return toPyAABB(pgeominfo->ComputeAABB(ExtractTransform(otransform)));
    }

    void DeserializeJSON(object obj, const dReal fUnitScale=1.0)
    {
        rapidjson::Document doc;
        toRapidJSONValue(obj, doc, doc.GetAllocator());
        KinBody::GeometryInfoPtr pgeominfo = GetGeometryInfo();
        pgeominfo->DeserializeJSON(doc, fUnitScale);
        Init(*pgeominfo);
    }

    object SerializeJSON(const dReal fUnitScale=1.0, object options=object())
    {
        rapidjson::Document doc;
        KinBody::GeometryInfoPtr pgeominfo = GetGeometryInfo();
        pgeominfo->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(options,0));
        return toPyObject(doc);
    }

    KinBody::GeometryInfoPtr GetGeometryInfo() {
        KinBody::GeometryInfoPtr pinfo(new KinBody::GeometryInfo());
        KinBody::GeometryInfo& info = *pinfo;
        info._t = ExtractTransform(_t);
        info._vGeomData = ExtractVector<dReal>(_vGeomData);
        info._vGeomData2 = ExtractVector<dReal>(_vGeomData2);
        info._vGeomData3 = ExtractVector<dReal>(_vGeomData3);
        info._vGeomData4 = ExtractVector<dReal>(_vGeomData4);

        info._vSideWalls.clear();
        for (size_t i = 0; i < info._vSideWalls.size(); ++i) {
            info._vSideWalls.push_back({});
            boost::shared_ptr<PySideWall> pysidewall = py::extract<boost::shared_ptr<PySideWall> >(_vSideWalls[i]);
            pysidewall->Get(info._vSideWalls[i]);
        }

        info._vDiffuseColor = ExtractVector34<dReal>(_vDiffuseColor,0);
        info._vAmbientColor = ExtractVector34<dReal>(_vAmbientColor,0);
        if( !IS_PYTHONOBJECT_NONE(_meshcollision) ) {
            ExtractTriMesh(_meshcollision,info._meshcollision);
        }
        info._type = _type;
        if( !IS_PYTHONOBJECT_NONE(_name) ) {
            info._name = py::extract<std::string>(_name);
        }
        if( !IS_PYTHONOBJECT_NONE(_filenamerender) ) {
            info._filenamerender = py::extract<std::string>(_filenamerender);
        }
        if( !IS_PYTHONOBJECT_NONE(_filenamecollision) ) {
            info._filenamecollision = py::extract<std::string>(_filenamecollision);
        }
        info._vRenderScale = ExtractVector3(_vRenderScale);
        info._vCollisionScale = ExtractVector3(_vCollisionScale);
        info._fTransparency = _fTransparency;
        info._bVisible = _bVisible;
        info._bModifiable = _bModifiable;
        //TODO
        //info._mapExtraGeometries =  _mapExtraGeometries;
        return pinfo;
    }

    object _t, _vGeomData, _vGeomData2, _vGeomData3, _vGeomData4, _vDiffuseColor, _vAmbientColor, _meshcollision;
    py::list _vSideWalls;
    float _containerBaseHeight;
    GeometryType _type;
    object _name;
    object _filenamerender, _filenamecollision;
    object _vRenderScale, _vCollisionScale;
    py::dict _mapExtraGeometries;
    float _fTransparency;
    bool _bVisible, _bModifiable;
};
typedef boost::shared_ptr<PyGeometryInfo> PyGeometryInfoPtr;

class PyLinkInfo
{
public:
    PyLinkInfo() {
        _t = ReturnTransform(Transform());
        _tMassFrame = ReturnTransform(Transform());
        _mass = 0;
        _vinertiamoments = toPyVector3(Vector(1,1,1));
        _bStatic = false;
        _bIsEnabled = true;
        _vForcedAdjacentLinks = py::list();
    }
    PyLinkInfo(const KinBody::LinkInfo& info) {
        _Update(info);
    }

    KinBody::LinkInfoPtr GetLinkInfo() {
        KinBody::LinkInfoPtr pinfo(new KinBody::LinkInfo());
        KinBody::LinkInfo& info = *pinfo;
        info._vgeometryinfos.resize(len(_vgeometryinfos));
        for(size_t i = 0; i < info._vgeometryinfos.size(); ++i) {
            PyGeometryInfoPtr pygeom = py::extract<PyGeometryInfoPtr>(_vgeometryinfos[i]);
            info._vgeometryinfos[i] = pygeom->GetGeometryInfo();
        }
        if( !IS_PYTHONOBJECT_NONE(_name) ) {
            info._name = py::extract<std::string>(_name);
        }
        info._t = ExtractTransform(_t);
        info._tMassFrame = ExtractTransform(_tMassFrame);
        info._mass = _mass;
        info._vinertiamoments = ExtractVector3(_vinertiamoments);
        size_t num = len(_mapFloatParameters);
        object okeyvalueiter = _mapFloatParameters.iteritems();
        info._mapFloatParameters.clear();
        for(size_t i = 0; i < num; ++i) {
            object okeyvalue = okeyvalueiter.attr("next") ();
            std::string name = extract<std::string>(okeyvalue[0]);
            info._mapFloatParameters[name] = ExtractArray<dReal>(okeyvalue[1]);
        }
        okeyvalueiter = _mapIntParameters.iteritems();
        num = len(_mapIntParameters);
        info._mapIntParameters.clear();
        for(size_t i = 0; i < num; ++i) {
            object okeyvalue = okeyvalueiter.attr("next") ();
            std::string name = extract<std::string>(okeyvalue[0]);
            info._mapIntParameters[name] = ExtractArray<int>(okeyvalue[1]);
        }
        okeyvalueiter = _mapStringParameters.iteritems();
        num = len(_mapStringParameters);
        info._mapStringParameters.clear();
        for(size_t i = 0; i < num; ++i) {
            object okeyvalue = okeyvalueiter.attr("next") ();
            std::string name = extract<std::string>(okeyvalue[0]);
            info._mapStringParameters[name] = (std::string)extract<std::string>(okeyvalue[1]);
        }
        info._vForcedAdjacentLinks = ExtractArray<std::string>(_vForcedAdjacentLinks);
        info._bStatic = _bStatic;
        info._bIsEnabled = _bIsEnabled;
        return pinfo;
    }

    object SerializeJSON(const dReal fUnitScale=1.0, object options=object())
    {
        rapidjson::Document doc;
        KinBody::LinkInfoPtr pInfo = GetLinkInfo();
        pInfo->SerializeJSON(doc, doc.GetAllocator(), pyGetIntFromPy(options, 0));
        return toPyObject(doc);
    }

    void DeserializeJSON(object obj, const dReal fUnitScale=1.0)
    {
        rapidjson::Document doc;
        toRapidJSONValue(obj, doc, doc.GetAllocator());
        KinBody::LinkInfo info;
        info.DeserializeJSON(doc, fUnitScale);
        _Update(info);
    }

    py::list _vgeometryinfos;
    object _name;
    object _t, _tMassFrame;
    dReal _mass;
    object _vinertiamoments;
    py::dict _mapFloatParameters, _mapIntParameters, _mapStringParameters;
    object _vForcedAdjacentLinks;
    bool _bStatic;
    bool _bIsEnabled;

private:
    void _Update(const KinBody::LinkInfo& info)
    {
        FOREACHC(itgeominfo, info._vgeometryinfos) {
            _vgeometryinfos.append(PyGeometryInfoPtr(new PyGeometryInfo(**itgeominfo)));
        }
        _name = ConvertStringToUnicode(info._name);
        _t = ReturnTransform(info._t);
        _tMassFrame = ReturnTransform(info._tMassFrame);
        _mass = info._mass;
        _vinertiamoments = toPyVector3(info._vinertiamoments);
        FOREACHC(it, info._mapFloatParameters) {
            _mapFloatParameters[it->first] = toPyArray(it->second);
        }
        FOREACHC(it, info._mapIntParameters) {
            _mapIntParameters[it->first] = toPyArray(it->second);
        }
        FOREACHC(it, info._mapStringParameters) {
            _mapStringParameters[it->first] = ConvertStringToUnicode(it->second);
        }
        py::list vForcedAdjacentLinks;
        FOREACHC(it, info._vForcedAdjacentLinks) {
            vForcedAdjacentLinks.append(ConvertStringToUnicode(*it));
        }
        _vForcedAdjacentLinks = vForcedAdjacentLinks;
        _bStatic = info._bStatic;
        _bIsEnabled = info._bIsEnabled;
    }
};

PyLinkInfoPtr toPyLinkInfo(const KinBody::LinkInfo& linkinfo)
{
    return PyLinkInfoPtr(new PyLinkInfo(linkinfo));
}

class PyElectricMotorActuatorInfo
{
public:
    PyElectricMotorActuatorInfo() {
        gear_ratio = 0;
        assigned_power_rating = 0;
        max_speed = 0;
        no_load_speed = 0;
        stall_torque = 0;
        max_instantaneous_torque = 0;
        nominal_torque = 0;
        rotor_inertia = 0;
        torque_constant = 0;
        nominal_voltage = 0;
        speed_constant = 0;
        starting_current = 0;
        terminal_resistance = 0;
        coloumb_friction = 0;
        viscous_friction = 0;
    }
    PyElectricMotorActuatorInfo(const ElectricMotorActuatorInfo& info) {
        _Update(info);
    }

    ElectricMotorActuatorInfoPtr GetElectricMotorActuatorInfo() {
        ElectricMotorActuatorInfoPtr pinfo(new ElectricMotorActuatorInfo());
        ElectricMotorActuatorInfo& info = *pinfo;
        info.model_type = model_type;
        info.gear_ratio = gear_ratio;
        info.assigned_power_rating = assigned_power_rating;
        info.max_speed = max_speed;
        info.no_load_speed = no_load_speed;
        info.stall_torque = stall_torque;
        info.max_instantaneous_torque = max_instantaneous_torque;
        if( !IS_PYTHONOBJECT_NONE(nominal_speed_torque_points) ) {
            size_t num = len(nominal_speed_torque_points);
            for(size_t i = 0; i < num; ++i) {
                info.nominal_speed_torque_points.emplace_back((dReal) py::extract<dReal>(nominal_speed_torque_points[i][0]),  (dReal) py::extract<dReal>(nominal_speed_torque_points[i][1]));
            }
        }
        if( !IS_PYTHONOBJECT_NONE(max_speed_torque_points) ) {
            size_t num = len(max_speed_torque_points);
            for(size_t i = 0; i < num; ++i) {
                info.max_speed_torque_points.emplace_back((dReal) py::extract<dReal>(max_speed_torque_points[i][0]),  (dReal) py::extract<dReal>(max_speed_torque_points[i][1]));
            }
        }
        info.nominal_torque = nominal_torque;
        info.rotor_inertia = rotor_inertia;
        info.torque_constant = torque_constant;
        info.nominal_voltage = nominal_voltage;
        info.speed_constant = speed_constant;
        info.starting_current = starting_current;
        info.terminal_resistance = terminal_resistance;
        info.coloumb_friction = coloumb_friction;
        info.viscous_friction = viscous_friction;
        return pinfo;
    }

    object SerializeJSON(object options=object())
    {
        rapidjson::Document doc;
        ElectricMotorActuatorInfoPtr pInfo = GetElectricMotorActuatorInfo();
        pInfo->SerializeJSON(doc, doc.GetAllocator(), pyGetIntFromPy(options, 0));
        return toPyObject(doc);
    }
    void DeserializeJSON(object obj, PyEnvironmentBasePtr penv)
    {
        rapidjson::Document doc;
        toRapidJSONValue(obj, doc, doc.GetAllocator());
        ElectricMotorActuatorInfo info;
        info.DeserializeJSON(doc, GetEnvironment(penv));
        _Update(info);
        return;
    }

    std::string model_type;
    dReal gear_ratio;
    dReal assigned_power_rating;
    dReal max_speed;
    dReal no_load_speed;
    dReal stall_torque;
    dReal max_instantaneous_torque;
    py::list nominal_speed_torque_points, max_speed_torque_points;
    dReal nominal_torque;
    dReal rotor_inertia;
    dReal torque_constant;
    dReal nominal_voltage;
    dReal speed_constant;
    dReal starting_current;
    dReal terminal_resistance;
    dReal coloumb_friction;
    dReal viscous_friction;

private:
    void _Update(const ElectricMotorActuatorInfo& info)
    {
        model_type = info.model_type;
        gear_ratio = info.gear_ratio;
        assigned_power_rating = info.assigned_power_rating;
        max_speed = info.max_speed;
        no_load_speed = info.no_load_speed;
        stall_torque = info.stall_torque;
        max_instantaneous_torque = info.max_instantaneous_torque;
        FOREACH(itpoint, info.nominal_speed_torque_points) {
            nominal_speed_torque_points.append(py::make_tuple(itpoint->first, itpoint->second));
        }
        FOREACH(itpoint, info.max_speed_torque_points) {
            max_speed_torque_points.append(py::make_tuple(itpoint->first, itpoint->second));
        }
        nominal_torque = info.nominal_torque;
        rotor_inertia = info.rotor_inertia;
        torque_constant = info.torque_constant;
        nominal_voltage = info.nominal_voltage;
        speed_constant = info.speed_constant;
        starting_current = info.starting_current;
        terminal_resistance = info.terminal_resistance;
        coloumb_friction = info.coloumb_friction;
        viscous_friction = info.viscous_friction;
    }
};
typedef boost::shared_ptr<PyElectricMotorActuatorInfo> PyElectricMotorActuatorInfoPtr;

class PyJointControlInfo_RobotController
{
public:
    PyJointControlInfo_RobotController()
    {
        robotId = -1;
        robotControllerDOFIndex = toPyVector3(Vector(-1, -1, -1));
    }
    PyJointControlInfo_RobotController(const KinBody::JointInfo::JointControlInfo_RobotController& jci)
    {
        robotId = jci.robotId;

        py::list _robotControllerDOFIndex;
        FOREACHC(itdofindex, jci.robotControllerDOFIndex) {
            _robotControllerDOFIndex.append(*itdofindex);
        }
        robotControllerDOFIndex = _robotControllerDOFIndex;
    }
    KinBody::JointInfo::JointControlInfo_RobotControllerPtr GetJointControlInfo()
    {
        KinBody::JointInfo::JointControlInfo_RobotControllerPtr pinfo(new KinBody::JointInfo::JointControlInfo_RobotController());
        KinBody::JointInfo::JointControlInfo_RobotController& info = *pinfo;
        info.robotId = robotId;
        if( !IS_PYTHONOBJECT_NONE(robotControllerDOFIndex) ) {
            size_t num = len(robotControllerDOFIndex);
            OPENRAVE_EXCEPTION_FORMAT0(num == info.robotControllerDOFIndex.size(), ORE_InvalidState);
            for( size_t i = 0; i < num; ++i ) {
                info.robotControllerDOFIndex[i] = py::extract<int>(robotControllerDOFIndex[i]);
            }
        }
        return pinfo;
    }

    int robotId;
    object robotControllerDOFIndex;
};
typedef boost::shared_ptr<PyJointControlInfo_RobotController> PyJointControlInfo_RobotControllerPtr;

class PyJointControlInfo_IO
{
public:
    PyJointControlInfo_IO()
    {
        deviceId = -1;
        vMoveIONames = py::list();
        vUpperLimitIONames = py::list();
        vUpperLimitSensorIsOn = py::list();
        vLowerLimitIONames = py::list();
        vLowerLimitSensorIsOn = py::list();
    }
    PyJointControlInfo_IO(const KinBody::JointInfo::JointControlInfo_IO& jci)
    {
        deviceId = jci.deviceId;

        py::list _vMoveIONames;
        FOREACHC(itmoveionamelist, jci.vMoveIONames) {
            if( itmoveionamelist->size() == 0 ) {
                _vMoveIONames.append(py::list());
            }
            else {
                py::list ionames;
                FOREACHC(itioname, *itmoveionamelist) {
                    ionames.append(ConvertStringToUnicode(*itioname));
                }
                _vMoveIONames.append(ionames);
            }
        }
        vMoveIONames = _vMoveIONames;

        py::list _vUpperLimitIONames;
        FOREACHC(itupperlimitionamelist, jci.vUpperLimitIONames) {
            if( itupperlimitionamelist->size() == 0 ) {
                _vUpperLimitIONames.append(py::list());
            }
            else {
                py::list ionames;
                FOREACHC(itioname, *itupperlimitionamelist) {
                    ionames.append(ConvertStringToUnicode(*itioname));
                }
                _vUpperLimitIONames.append(ionames);
            }
        }
        vUpperLimitIONames = _vUpperLimitIONames;

        py::list _vUpperLimitSensorIsOn;
        FOREACHC(itiovaluelist, jci.vUpperLimitSensorIsOn) {
            if( itiovaluelist->size() == 0 ) {
                _vUpperLimitSensorIsOn.append(py::list());
            }
            else {
                py::list iovalues;
                FOREACHC(itiovalue, *itiovaluelist) {
                    iovalues.append(*itiovalue);
                }
                _vUpperLimitSensorIsOn.append(iovalues);
            }
        }
        vUpperLimitSensorIsOn = _vUpperLimitSensorIsOn;

        py::list _vLowerLimitIONames;
        FOREACHC(itlowerlimitionamelist, jci.vLowerLimitIONames) {
            if( itlowerlimitionamelist->size() == 0 ) {
                _vLowerLimitIONames.append(py::list());
            }
            else {
                py::list ionames;
                FOREACHC(itioname, *itlowerlimitionamelist) {
                    ionames.append(ConvertStringToUnicode(*itioname));
                }
                _vLowerLimitIONames.append(ionames);
            }
        }
        vLowerLimitIONames = _vLowerLimitIONames;

        py::list _vLowerLimitSensorIsOn;
        FOREACHC(itiovaluelist, jci.vLowerLimitSensorIsOn) {
            if( itiovaluelist->size() == 0 ) {
                _vLowerLimitSensorIsOn.append(py::list());
            }
            else {
                py::list iovalues;
                FOREACHC(itiovalue, *itiovaluelist) {
                    iovalues.append(*itiovalue);
                }
                _vLowerLimitSensorIsOn.append(iovalues);
            }
        }
        vLowerLimitSensorIsOn = _vLowerLimitSensorIsOn;
    }

    KinBody::JointInfo::JointControlInfo_IOPtr GetJointControlInfo()
    {
        KinBody::JointInfo::JointControlInfo_IOPtr pinfo(new KinBody::JointInfo::JointControlInfo_IO());
        KinBody::JointInfo::JointControlInfo_IO& info = *pinfo;
        info.deviceId = deviceId;

        size_t num1, num2;
        if( !IS_PYTHONOBJECT_NONE(vMoveIONames) ) {
            num1 = len(vMoveIONames);
            OPENRAVE_EXCEPTION_FORMAT0(num1 == info.vMoveIONames.size(), ORE_InvalidState);
            for( size_t i1 = 0; i1 < num1; ++i1 ) {
                num2 = len(vMoveIONames[i1]);
                info.vMoveIONames[i1].resize(num2);
                for( size_t i2 = 0; i2 < num2; ++i2 ) {
                    info.vMoveIONames[i1].at(i2) = py::extract<std::string>(vMoveIONames[i1][i2]);
                }
            }
        }

        if( !IS_PYTHONOBJECT_NONE(vUpperLimitIONames) ) {
            num1 = len(vUpperLimitIONames);
            OPENRAVE_EXCEPTION_FORMAT0(num1 == info.vUpperLimitIONames.size(), ORE_InvalidState);
            for( size_t i1 = 0; i1 < num1; ++i1 ) {
                num2 = len(vUpperLimitIONames[i1]);
                info.vUpperLimitIONames[i1].resize(num2);
                for( size_t i2 = 0; i2 < num2; ++i2 ) {
                    info.vUpperLimitIONames[i1].at(i2) = py::extract<std::string>(vUpperLimitIONames[i1][i2]);
                }
            }
        }

        if( !IS_PYTHONOBJECT_NONE(vUpperLimitSensorIsOn) ) {
            num1 = len(vUpperLimitSensorIsOn);
            OPENRAVE_EXCEPTION_FORMAT0(num1 == info.vUpperLimitSensorIsOn.size(), ORE_InvalidState);
            for( size_t i1 = 0; i1 < num1; ++i1 ) {
                num2 = len(vUpperLimitSensorIsOn[i1]);
                info.vUpperLimitSensorIsOn[i1].resize(num2);
                for( size_t i2 = 0; i2 < num2; ++i2 ) {
                    info.vUpperLimitSensorIsOn[i1].at(i2) = py::extract<uint8_t>(vUpperLimitSensorIsOn[i1][i2]);
                }
            }
        }

        if( !IS_PYTHONOBJECT_NONE(vLowerLimitIONames) ) {
            num1 = len(vLowerLimitIONames);
            OPENRAVE_EXCEPTION_FORMAT0(num1 == info.vLowerLimitIONames.size(), ORE_InvalidState);
            for( size_t i1 = 0; i1 < num1; ++i1 ) {
                num2 = len(vLowerLimitIONames[i1]);
                info.vLowerLimitIONames[i1].resize(num2);
                for( size_t i2 = 0; i2 < num2; ++i2 ) {
                    info.vLowerLimitIONames[i1].at(i2) = py::extract<std::string>(vLowerLimitIONames[i1][i2]);
                }
            }
        }

        if( !IS_PYTHONOBJECT_NONE(vLowerLimitSensorIsOn) ) {
            num1 = len(vLowerLimitSensorIsOn);
            OPENRAVE_EXCEPTION_FORMAT0(num1 == info.vLowerLimitSensorIsOn.size(), ORE_InvalidState);
            for( size_t i1 = 0; i1 < num1; ++i1 ) {
                num2 = len(vLowerLimitSensorIsOn[i1]);
                info.vLowerLimitSensorIsOn[i1].resize(num2);
                for( size_t i2 = 0; i2 < num2; ++i2 ) {
                    info.vLowerLimitSensorIsOn[i1].at(i2) = py::extract<uint8_t>(vLowerLimitSensorIsOn[i1][i2]);
                }
            }
        }
        return pinfo;
    }

    int deviceId;
    object vMoveIONames;
    object vUpperLimitIONames;
    object vUpperLimitSensorIsOn;
    object vLowerLimitIONames;
    object vLowerLimitSensorIsOn;
};
typedef boost::shared_ptr<PyJointControlInfo_IO> PyJointControlInfo_IOPtr;

class PyJointControlInfo_ExternalDevice
{
public:
    PyJointControlInfo_ExternalDevice()
    {
    }
    PyJointControlInfo_ExternalDevice(const KinBody::JointInfo::JointControlInfo_ExternalDevice &jci)
    {
        externalDeviceId = jci.externalDeviceId;
    }
    KinBody::JointInfo::JointControlInfo_ExternalDevicePtr GetJointControlInfo()
    {
        KinBody::JointInfo::JointControlInfo_ExternalDevicePtr pinfo(new KinBody::JointInfo::JointControlInfo_ExternalDevice());
        KinBody::JointInfo::JointControlInfo_ExternalDevice& info = *pinfo;
        info.externalDeviceId = externalDeviceId;
        return pinfo;
    }

    std::string externalDeviceId;
};
typedef boost::shared_ptr<PyJointControlInfo_ExternalDevice> PyJointControlInfo_ExternalDevicePtr;

class PyJointInfo
{
public:
    PyJointInfo() {
        _type = KinBody::JointNone;
        _vanchor = toPyVector3(Vector());
        _vaxes = py::list();
        _vresolution = toPyVector3(Vector(0.02,0.02,0.02));
        // As for initial values for vel, accel, and jerk, please see kinbodyjoint.cpp's definition.
        _vmaxvel = toPyVector3(Vector(10,10,10));
        _vhardmaxvel = toPyVector3(Vector(0,0,0));
        _vmaxaccel = toPyVector3(Vector(50,50,50));
        _vhardmaxaccel = toPyVector3(Vector(0,0,0));
        _vmaxjerk = toPyVector3(Vector(2e6,2e6,2e6));
        _vhardmaxjerk = toPyVector3(Vector(0, 0, 0));
        _vmaxtorque = toPyVector3(Vector(1e5,1e5,1e5));
        _vmaxinertia = toPyVector3(Vector(1e5,1e5,1e5));
        _vweights = toPyVector3(Vector(1,1,1));
        _voffsets = toPyVector3(Vector(0,0,0));
        _vlowerlimit = toPyVector3(Vector(0,0,0));
        _vupperlimit = toPyVector3(Vector(0,0,0));
        _bIsCircular = py::list();
        _bIsActive = true;
        _controlMode = KinBody::JCM_None;
    }

    PyJointInfo(const KinBody::JointInfo& info, PyEnvironmentBasePtr pyenv) {
        _Update(info, pyenv);
    }

    KinBody::JointInfoPtr GetJointInfo() {
        KinBody::JointInfoPtr pinfo(new KinBody::JointInfo());
        KinBody::JointInfo& info = *pinfo;
        info._type = _type;
        if( !IS_PYTHONOBJECT_NONE(_name) ) {
            info._name = py::extract<std::string>(_name);
        }
        if( !IS_PYTHONOBJECT_NONE(_linkname0) ) {
            info._linkname0 = py::extract<std::string>(_linkname0);
        }
        if( !IS_PYTHONOBJECT_NONE(_linkname1) ) {
            info._linkname1 = py::extract<std::string>(_linkname1);
        }
        info._vanchor = ExtractVector3(_vanchor);

        // We might be able to replace these exceptions with static_assert in C++11
        size_t num = len(_vaxes);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vaxes.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vaxes[i] = ExtractVector3(_vaxes[i]);
        }

        if( !IS_PYTHONOBJECT_NONE(_vcurrentvalues) ) {
            info._vcurrentvalues = ExtractArray<dReal>(_vcurrentvalues);
        }

        num = len(_vresolution);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vresolution.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vresolution[i] = py::extract<dReal>(_vresolution[i]);
        }

        num = len(_vmaxvel);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vmaxvel.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vmaxvel[i] = py::extract<dReal>(_vmaxvel[i]);
        }

        num = len(_vhardmaxvel);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vhardmaxvel.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vhardmaxvel[i] = py::extract<dReal>(_vhardmaxvel[i]);
        }

        num = len(_vmaxaccel);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vmaxaccel.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vmaxaccel[i] = py::extract<dReal>(_vmaxaccel[i]);
        }

        num = len(_vhardmaxaccel);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vhardmaxaccel.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vhardmaxaccel[i] = py::extract<dReal>(_vhardmaxaccel[i]);
        }

        num = len(_vmaxjerk);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vmaxjerk.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vmaxjerk[i] = py::extract<dReal>(_vmaxjerk[i]);
        }

        num = len(_vhardmaxjerk);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vhardmaxjerk.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vhardmaxjerk[i] = py::extract<dReal>(_vhardmaxjerk[i]);
        }

        num = len(_vmaxtorque);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vmaxtorque.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vmaxtorque[i] = py::extract<dReal>(_vmaxtorque[i]);
        }

        num = len(_vmaxinertia);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vmaxinertia.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vmaxinertia[i] = py::extract<dReal>(_vmaxinertia[i]);
        }

        num = len(_vweights);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vweights.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vweights[i] = py::extract<dReal>(_vweights[i]);
        }

        num = len(_voffsets);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._voffsets.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._voffsets[i] = py::extract<dReal>(_voffsets[i]);
        }

        num = len(_vlowerlimit);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vlowerlimit.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vlowerlimit[i] = py::extract<dReal>(_vlowerlimit[i]);
        }

        num = len(_vupperlimit);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vupperlimit.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vupperlimit[i] = py::extract<dReal>(_vupperlimit[i]);
        }

        if( !IS_PYTHONOBJECT_NONE(_trajfollow) ) {
            info._trajfollow = GetTrajectory(_trajfollow);
        }
        if( !IS_PYTHONOBJECT_NONE(_vmimic) ) {
            num = len(_vmimic);
            for(size_t i = 0; i < num; ++i) {
                object omimic = _vmimic[i];
                if( !IS_PYTHONOBJECT_NONE(omimic) ) {
                    info._vmimic[i].reset(new KinBody::MimicInfo());
                    for(size_t j = 0; j < 3; ++j) {
                        info._vmimic[i]->_equations.at(j) = py::extract<std::string>(omimic[j]);
                    }
                }
            }
        }
        num = len(_mapFloatParameters);
        object okeyvalueiter = _mapFloatParameters.iteritems();
        info._mapFloatParameters.clear();
        for(size_t i = 0; i < num; ++i) {
            object okeyvalue = okeyvalueiter.attr("next") ();
            std::string name = extract<std::string>(okeyvalue[0]);
            info._mapFloatParameters[name] = ExtractArray<dReal>(okeyvalue[1]);
        }
        okeyvalueiter = _mapIntParameters.iteritems();
        num = len(_mapIntParameters);
        info._mapIntParameters.clear();
        for(size_t i = 0; i < num; ++i) {
            object okeyvalue = okeyvalueiter.attr("next") ();
            std::string name = extract<std::string>(okeyvalue[0]);
            info._mapIntParameters[name] = ExtractArray<int>(okeyvalue[1]);
        }
        okeyvalueiter = _mapStringParameters.iteritems();
        num = len(_mapStringParameters);
        info._mapStringParameters.clear();
        for(size_t i = 0; i < num; ++i) {
            object okeyvalue = okeyvalueiter.attr("next") ();
            std::string name = extract<std::string>(okeyvalue[0]);
            info._mapStringParameters[name] = (std::string)extract<std::string>(okeyvalue[1]);
        }
        num = len(_bIsCircular);
        for(size_t i = 0; i < num; ++i) {
            info._bIsCircular.at(i) = py::extract<int>(_bIsCircular[i])!=0;
        }
        info._bIsActive = _bIsActive;
        if( !!_infoElectricMotor ) {
            //PyElectricMotorActuatorInfoPtr pinfo = py::extract<PyElectricMotorActuatorInfoPtr>(_infoElectricMotor);
            //if( !!pinfo ) {
            info._infoElectricMotor = _infoElectricMotor->GetElectricMotorActuatorInfo();
            //}
        }

        // joint control
        info._controlMode = _controlMode;
        if( _controlMode == KinBody::JCM_RobotController ) {
            info._jci_robotcontroller = _jci_robotcontroller->GetJointControlInfo();
        }
        else if( _controlMode == KinBody::JCM_IO ) {
            info._jci_io = _jci_io->GetJointControlInfo();
        }
        else if( _controlMode == KinBody::JCM_ExternalDevice ) {
            info._jci_externaldevice = _jci_externaldevice->GetJointControlInfo();
        }

        return pinfo;
    }

    object SerializeJSON(object options=object())
    {
    	rapidjson::Document doc;
    	KinBody::JointInfoPtr pInfo = GetJointInfo();
    	pInfo->SerializeJSON(doc, doc.GetAllocator(), pyGetIntFromPy(options, 0));
    	return toPyObject(doc);
    }

    void DeserializeJSON(object obj, PyEnvironmentBasePtr penv, const dReal fUnitScale=1.0)
    {
        rapidjson::Document doc;
        toRapidJSONValue(obj, doc, doc.GetAllocator());
        KinBody::JointInfo info;
        info.DeserializeJSON(doc, GetEnvironment(penv), fUnitScale);
        _Update(info, penv);
        return;
    }

    KinBody::JointType _type;
    object _name;
    object _linkname0, _linkname1;
    object _vanchor, _vaxes, _vcurrentvalues, _vresolution, _vmaxvel, _vhardmaxvel, _vmaxaccel, _vhardmaxaccel, _vmaxjerk, _vhardmaxjerk, _vmaxtorque, _vmaxinertia, _vweights, _voffsets, _vlowerlimit, _vupperlimit;
    object _trajfollow;
    PyElectricMotorActuatorInfoPtr _infoElectricMotor;
    py::list _vmimic;
    py::dict _mapFloatParameters, _mapIntParameters, _mapStringParameters;
    object _bIsCircular;
    bool _bIsActive;
    KinBody::JointControlMode _controlMode;

    PyJointControlInfo_RobotControllerPtr _jci_robotcontroller;
    PyJointControlInfo_IOPtr _jci_io;
    PyJointControlInfo_ExternalDevicePtr _jci_externaldevice;

private:
    void _Update(const KinBody::JointInfo& info, PyEnvironmentBasePtr pyenv)
    {
        _type = info._type;
        _name = ConvertStringToUnicode(info._name);
        _linkname0 = ConvertStringToUnicode(info._linkname0);
        _linkname1 = ConvertStringToUnicode(info._linkname1);
        _vanchor = toPyVector3(info._vanchor);
        py::list vaxes;
        for(size_t i = 0; i < info._vaxes.size(); ++i) {
            vaxes.append(toPyVector3(info._vaxes[i]));
        }
        _vaxes = vaxes;
        _vcurrentvalues = toPyArray(info._vcurrentvalues);
        _vresolution = toPyArray<dReal,3>(info._vresolution);
        _vmaxvel = toPyArray<dReal,3>(info._vmaxvel);
        _vhardmaxvel = toPyArray<dReal,3>(info._vhardmaxvel);
        _vmaxaccel = toPyArray<dReal,3>(info._vmaxaccel);
        _vhardmaxaccel = toPyArray<dReal,3>(info._vhardmaxaccel);
        _vmaxjerk = toPyArray<dReal,3>(info._vmaxjerk);
        _vhardmaxjerk = toPyArray<dReal,3>(info._vhardmaxjerk);
        _vmaxtorque = toPyArray<dReal,3>(info._vmaxtorque);
        _vmaxinertia = toPyArray<dReal,3>(info._vmaxinertia);
        _vweights = toPyArray<dReal,3>(info._vweights);
        _voffsets = toPyArray<dReal,3>(info._voffsets);
        _vlowerlimit = toPyArray<dReal,3>(info._vlowerlimit);
        _vupperlimit = toPyArray<dReal,3>(info._vupperlimit);
        _trajfollow = object(toPyTrajectory(info._trajfollow, pyenv));
        FOREACHC(itmimic, info._vmimic) {
            if( !*itmimic ) {
                _vmimic.append(object());
            }
            else {
                py::list oequations;
                FOREACHC(itequation, (*itmimic)->_equations) {
                    oequations.append(*itequation);
                }
                _vmimic.append(oequations);
            }
        }
        FOREACHC(it, info._mapFloatParameters) {
            _mapFloatParameters[it->first] = toPyArray(it->second);
        }
        FOREACHC(it, info._mapIntParameters) {
            _mapIntParameters[it->first] = toPyArray(it->second);
        }
        FOREACHC(it, info._mapStringParameters) {
            _mapStringParameters[it->first] = ConvertStringToUnicode(it->second);
        }
        py::list bIsCircular;
        FOREACHC(it, info._bIsCircular) {
            bIsCircular.append(*it);
        }
        _bIsCircular = bIsCircular;
        _bIsActive = info._bIsActive;
        if( !!info._infoElectricMotor ) {
            _infoElectricMotor = PyElectricMotorActuatorInfoPtr(new PyElectricMotorActuatorInfo(*info._infoElectricMotor));
        }

        // joint control
        _controlMode = info._controlMode;
        if( _controlMode == KinBody::JCM_RobotController ) {
            if( !!info._jci_robotcontroller ) {
                _jci_robotcontroller = PyJointControlInfo_RobotControllerPtr(new PyJointControlInfo_RobotController(*info._jci_robotcontroller));
            }
        }
        else if( _controlMode == KinBody::JCM_IO ) {
            if( !!info._jci_io ) {
                _jci_io = PyJointControlInfo_IOPtr(new PyJointControlInfo_IO(*info._jci_io));
            }
        }
        else if( _controlMode == KinBody::JCM_ExternalDevice ) {
            if( !!info._jci_externaldevice ) {
                _jci_externaldevice = PyJointControlInfo_ExternalDevicePtr(new PyJointControlInfo_ExternalDevice(*info._jci_externaldevice));
            }
        }
    }
};

PyJointInfoPtr toPyJointInfo(const KinBody::JointInfo& jointinfo, PyEnvironmentBasePtr pyenv)
{
    return PyJointInfoPtr(new PyJointInfo(jointinfo, pyenv));
}

class PyLink
{
    KinBody::LinkPtr _plink;
    PyEnvironmentBasePtr _pyenv;
public:
    class PyGeometry
    {
        KinBody::Link::GeometryPtr _pgeometry;
public:
        PyGeometry(KinBody::Link::GeometryPtr pgeometry) : _pgeometry(pgeometry) {
        }

        virtual void SetCollisionMesh(object pytrimesh) {
            TriMesh mesh;
            if( ExtractTriMesh(pytrimesh,mesh) ) {
                _pgeometry->SetCollisionMesh(mesh);
            }
            else {
                throw openrave_exception(_("bad trimesh"));
            }
        }

        bool InitCollisionMesh(float fTessellation=1) {
            return _pgeometry->InitCollisionMesh(fTessellation);
        }
        uint8_t GetSideWallExists() const {
            return _pgeometry->GetSideWallExists();
        }

        object GetCollisionMesh() {
            return toPyTriMesh(_pgeometry->GetCollisionMesh());
        }
        object ComputeAABB(object otransform) const {
            return toPyAABB(_pgeometry->ComputeAABB(ExtractTransform(otransform)));
        }
        void SetDraw(bool bDraw) {
            _pgeometry->SetVisible(bDraw);
        }
        bool SetVisible(bool visible) {
            return _pgeometry->SetVisible(visible);
        }
        void SetTransparency(float f) {
            _pgeometry->SetTransparency(f);
        }
        void SetAmbientColor(object ocolor) {
            _pgeometry->SetAmbientColor(ExtractVector3(ocolor));
        }
        void SetDiffuseColor(object ocolor) {
            _pgeometry->SetDiffuseColor(ExtractVector3(ocolor));
        }
        void SetRenderFilename(const string& filename) {
            _pgeometry->SetRenderFilename(filename);
        }
        void SetName(const std::string& name) {
            _pgeometry->SetName(name);
        }
        bool IsDraw() {
            RAVELOG_WARN("IsDraw deprecated, use Geometry.IsVisible\n");
            return _pgeometry->IsVisible();
        }
        bool IsVisible() {
            return _pgeometry->IsVisible();
        }
        bool IsModifiable() {
            return _pgeometry->IsModifiable();
        }
        GeometryType GetType() {
            return _pgeometry->GetType();
        }
        object GetTransform() {
            return ReturnTransform(_pgeometry->GetTransform());
        }
        object GetTransformPose() {
            return toPyArray(_pgeometry->GetTransform());
        }
        dReal GetSphereRadius() const {
            return _pgeometry->GetSphereRadius();
        }
        dReal GetCylinderRadius() const {
            return _pgeometry->GetCylinderRadius();
        }
        dReal GetCylinderHeight() const {
            return _pgeometry->GetCylinderHeight();
        }
        object GetBoxExtents() const {
            return toPyVector3(_pgeometry->GetBoxExtents());
        }
        object GetContainerOuterExtents() const {
            return toPyVector3(_pgeometry->GetContainerOuterExtents());
        }
        object GetContainerInnerExtents() const {
            return toPyVector3(_pgeometry->GetContainerInnerExtents());
        }
        object GetContainerBottomCross() const {
            return toPyVector3(_pgeometry->GetContainerBottomCross());
        }
        object GetContainerBottom() const {
            return toPyVector3(_pgeometry->GetContainerBottom());
        }
        object GetRenderScale() const {
            return toPyVector3(_pgeometry->GetRenderScale());
        }
        object GetRenderFilename() const {
            return ConvertStringToUnicode(_pgeometry->GetRenderFilename());
        }
        object GetName() const {
            return ConvertStringToUnicode(_pgeometry->GetName());
        }
        float GetTransparency() const {
            return _pgeometry->GetTransparency();
        }
        object GetDiffuseColor() const {
            return toPyVector3(_pgeometry->GetDiffuseColor());
        }
        object GetAmbientColor() const {
            return toPyVector3(_pgeometry->GetAmbientColor());
        }
        object GetInfo() {
            return object(PyGeometryInfoPtr(new PyGeometryInfo(_pgeometry->GetInfo())));
        }
        object ComputeInnerEmptyVolume() const
        {
            Transform tInnerEmptyVolume;
            Vector abInnerEmptyExtents;
            if( _pgeometry->ComputeInnerEmptyVolume(tInnerEmptyVolume, abInnerEmptyExtents) ) {
                return py::make_tuple(ReturnTransform(tInnerEmptyVolume), toPyVector3(abInnerEmptyExtents));
            }
            return py::make_tuple(object(), object());
        }
        bool __eq__(boost::shared_ptr<PyGeometry> p) {
            return !!p && _pgeometry == p->_pgeometry;
        }
        bool __ne__(boost::shared_ptr<PyGeometry> p) {
            return !p || _pgeometry != p->_pgeometry;
        }
        int __hash__() {
            return static_cast<int>(uintptr_t(_pgeometry.get()));
        }
    };

    PyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv) : _plink(plink), _pyenv(pyenv) {
    }
    virtual ~PyLink() {
    }

    KinBody::LinkPtr GetLink() {
        return _plink;
    }

    object GetName() {
        return ConvertStringToUnicode(_plink->GetName());
    }
    int GetIndex() {
        return _plink->GetIndex();
    }
    bool IsEnabled() const {
        return _plink->IsEnabled();
    }
    bool SetVisible(bool visible) {
        return _plink->SetVisible(visible);
    }
    bool IsVisible() const {
        return _plink->IsVisible();
    }
    bool IsStatic() const {
        return _plink->IsStatic();
    }
    void Enable(bool bEnable) {
        _plink->Enable(bEnable);
    }

    object GetParent() const
    {
        KinBodyPtr parent = _plink->GetParent();
        if( parent->IsRobot() ) {
            return object(toPyRobot(RaveInterfaceCast<RobotBase>(_plink->GetParent()),_pyenv));
        }
        else {
            return object(PyKinBodyPtr(new PyKinBody(_plink->GetParent(),_pyenv)));
        }
    }

    object GetParentLinks() const
    {
        std::vector<KinBody::LinkPtr> vParentLinks;
        _plink->GetParentLinks(vParentLinks);
        py::list links;
        FOREACHC(itlink, vParentLinks) {
            links.append(PyLinkPtr(new PyLink(*itlink, _pyenv)));
        }
        return links;
    }

    bool IsParentLink(boost::shared_ptr<PyLink> pylink) const {
        return _plink->IsParentLink(*pylink->GetLink());
    }

    object GetCollisionData() {
        return toPyTriMesh(_plink->GetCollisionData());
    }
    object ComputeLocalAABB() const { // TODO object otransform=object()
        //if( IS_PYTHONOBJECT_NONE(otransform) ) {
        return toPyAABB(_plink->ComputeLocalAABB());
    }

    object ComputeAABB() const {
        return toPyAABB(_plink->ComputeAABB());
    }

    object ComputeAABBFromTransform(object otransform) const {
        return toPyAABB(_plink->ComputeAABBFromTransform(ExtractTransform(otransform)));
    }

    object GetTransform() const {
        return ReturnTransform(_plink->GetTransform());
    }
    object GetTransformPose() const {
        return toPyArray(_plink->GetTransform());
    }

    object GetCOMOffset() const {
        return toPyVector3(_plink->GetCOMOffset());
    }
    object GetLocalCOM() const {
        return toPyVector3(_plink->GetLocalCOM());
    }
    object GetGlobalCOM() const {
        return toPyVector3(_plink->GetGlobalCOM());
    }

    object GetLocalInertia() const {
        TransformMatrix t = _plink->GetLocalInertia();
        npy_intp dims[] = { 3, 3};
        PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
        pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2];
        pdata[3] = t.m[4]; pdata[4] = t.m[5]; pdata[5] = t.m[6];
        pdata[6] = t.m[8]; pdata[7] = t.m[9]; pdata[8] = t.m[10];
        return static_cast<numeric::array>(handle<>(pyvalues));
    }
    object GetGlobalInertia() const {
        TransformMatrix t = _plink->GetGlobalInertia();
        npy_intp dims[] = { 3, 3};
        PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
        pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2];
        pdata[3] = t.m[4]; pdata[4] = t.m[5]; pdata[5] = t.m[6];
        pdata[6] = t.m[8]; pdata[7] = t.m[9]; pdata[8] = t.m[10];
        return static_cast<numeric::array>(handle<>(pyvalues));
    }
    dReal GetMass() const {
        return _plink->GetMass();
    }
    object GetPrincipalMomentsOfInertia() const {
        return toPyVector3(_plink->GetPrincipalMomentsOfInertia());
    }
    object GetLocalMassFrame() const {
        return ReturnTransform(_plink->GetLocalMassFrame());
    }
    object GetGlobalMassFrame() const {
        return ReturnTransform(_plink->GetGlobalMassFrame());
    }
    void SetLocalMassFrame(object omassframe) {
        _plink->SetLocalMassFrame(ExtractTransform(omassframe));
    }
    void SetPrincipalMomentsOfInertia(object oinertiamoments) {
        _plink->SetPrincipalMomentsOfInertia(ExtractVector3(oinertiamoments));
    }
    void SetMass(dReal mass) {
        _plink->SetMass(mass);
    }

    void SetStatic(bool bStatic) {
        _plink->SetStatic(bStatic);
    }
    void SetTransform(object otrans) {
        _plink->SetTransform(ExtractTransform(otrans));
    }
    void SetForce(object oforce, object opos, bool bAdd) {
        return _plink->SetForce(ExtractVector3(oforce),ExtractVector3(opos),bAdd);
    }
    void SetTorque(object otorque, bool bAdd) {
        return _plink->SetTorque(ExtractVector3(otorque),bAdd);
    }

    object GetGeometries() {
        py::list geoms;
        size_t N = _plink->GetGeometries().size();
        for(size_t i = 0; i < N; ++i) {
            geoms.append(boost::shared_ptr<PyGeometry>(new PyGeometry(_plink->GetGeometry(i))));
        }
        return geoms;
    }

    void InitGeometries(object ogeometryinfos)
    {
        std::vector<KinBody::GeometryInfoConstPtr> geometries(len(ogeometryinfos));
        for(size_t i = 0; i < geometries.size(); ++i) {
            PyGeometryInfoPtr pygeom = py::extract<PyGeometryInfoPtr>(ogeometryinfos[i]);
            if( !pygeom ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.GeometryInfo"),ORE_InvalidArguments);
            }
            geometries[i] = pygeom->GetGeometryInfo();
        }
        return _plink->InitGeometries(geometries);
    }

    void AddGeometry(object ogeometryinfo, bool addToGroups)
    {
        PyGeometryInfoPtr pygeom = py::extract<PyGeometryInfoPtr>(ogeometryinfo);
        if( !pygeom ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.GeometryInfo"),ORE_InvalidArguments);
        }
        _plink->AddGeometry(pygeom->GetGeometryInfo(), addToGroups);
    }

    void RemoveGeometryByName(const std::string& geometryname, bool removeFromAllGroups)
    {
        _plink->RemoveGeometryByName(geometryname, removeFromAllGroups);
    }

    void SetGeometriesFromGroup(const std::string& name)
    {
        _plink->SetGeometriesFromGroup(name);
    }

    object GetGeometriesFromGroup(const std::string& name)
    {
        py::list ogeometryinfos;
        FOREACHC(itinfo, _plink->GetGeometriesFromGroup(name)) {
            ogeometryinfos.append(PyGeometryInfoPtr(new PyGeometryInfo(**itinfo)));
        }
        return ogeometryinfos;
    }

    void SetGroupGeometries(const std::string& name, object ogeometryinfos)
    {
        std::vector<KinBody::GeometryInfoPtr> geometries(len(ogeometryinfos));
        for(size_t i = 0; i < geometries.size(); ++i) {
            PyGeometryInfoPtr pygeom = py::extract<PyGeometryInfoPtr>(ogeometryinfos[i]);
            if( !pygeom ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.GeometryInfo"),ORE_InvalidArguments);
            }
            geometries[i] = pygeom->GetGeometryInfo();
        }
        _plink->SetGroupGeometries(name, geometries);
    }

    int GetGroupNumGeometries(const std::string& geomname)
    {
        return _plink->GetGroupNumGeometries(geomname);
    }

    object GetRigidlyAttachedLinks() const {
        std::vector<KinBody::LinkPtr> vattachedlinks;
        _plink->GetRigidlyAttachedLinks(vattachedlinks);
        py::list links;
        FOREACHC(itlink, vattachedlinks) {
            links.append(PyLinkPtr(new PyLink(*itlink, _pyenv)));
        }
        return links;
    }

    bool IsRigidlyAttached(boost::shared_ptr<PyLink> plink) {
        CHECK_POINTER(plink);
        return _plink->IsRigidlyAttached(*plink->GetLink());
    }

    void SetVelocity(object olinear, object oangular) {
        _plink->SetVelocity(ExtractVector3(olinear),ExtractVector3(oangular));
    }

    object GetVelocity() const {
        std::pair<Vector,Vector> velocity;
        velocity = _plink->GetVelocity();
        boost::array<dReal,6> v = {{ velocity.first.x, velocity.first.y, velocity.first.z, velocity.second.x, velocity.second.y, velocity.second.z}};
        return toPyArray<dReal,6>(v);
    }

    object GetFloatParameters(object oname=object(), int index=-1) const {
        return GetCustomParameters(_plink->GetFloatParameters(), oname, index);
    }

    void SetFloatParameters(const std::string& key, object oparameters)
    {
        _plink->SetFloatParameters(key,ExtractArray<dReal>(oparameters));
    }

    object GetIntParameters(object oname=object(), int index=-1) const {
        return GetCustomParameters(_plink->GetIntParameters(), oname, index);
    }

    void SetIntParameters(const std::string& key, object oparameters)
    {
        _plink->SetIntParameters(key,ExtractArray<int>(oparameters));
    }

    object GetStringParameters(object oname=object()) const
    {
        if( IS_PYTHONOBJECT_NONE(oname) ) {
            py::dict oparameters;
            FOREACHC(it, _plink->GetStringParameters()) {
                oparameters[it->first] = ConvertStringToUnicode(it->second);
            }
            return oparameters;
        }
        std::string name = py::extract<std::string>(oname);
        std::map<std::string, std::string >::const_iterator it = _plink->GetStringParameters().find(name);
        if( it != _plink->GetStringParameters().end() ) {
            return ConvertStringToUnicode(it->second);
        }
        return object();
    }

    void SetStringParameters(const std::string& key, object ovalue)
    {
        _plink->SetStringParameters(key,extract<std::string>(ovalue));
    }

    void UpdateInfo() {
        _plink->UpdateInfo();
    }
    object GetInfo() {
        return object(PyLinkInfoPtr(new PyLinkInfo(_plink->GetInfo())));
    }
    object UpdateAndGetInfo() {
        return object(PyLinkInfoPtr(new PyLinkInfo(_plink->UpdateAndGetInfo())));
    }


    std::string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s').GetLink('%s')")%RaveGetEnvironmentId(_plink->GetParent()->GetEnv())%_plink->GetParent()->GetName()%_plink->GetName());
    }
    std::string __str__() {
        return boost::str(boost::format("<link:%s (%d), parent=%s>")%_plink->GetName()%_plink->GetIndex()%_plink->GetParent()->GetName());
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
    bool __eq__(boost::shared_ptr<PyLink> p) {
        return !!p && _plink == p->_plink;
    }
    bool __ne__(boost::shared_ptr<PyLink> p) {
        return !p || _plink != p->_plink;
    }
    int __hash__() {
        return static_cast<int>(uintptr_t(_plink.get()));
    }
};

PyLinkPtr toPyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv)
{
    return PyLinkPtr(new PyLink(plink, pyenv));
}

class PyJoint
{
    KinBody::JointPtr _pjoint;
    PyEnvironmentBasePtr _pyenv;
public:
    PyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv) : _pjoint(pjoint), _pyenv(pyenv) {
    }
    virtual ~PyJoint() {
    }

    KinBody::JointPtr GetJoint() {
        return _pjoint;
    }

    object GetName() {
        return ConvertStringToUnicode(_pjoint->GetName());
    }
    bool IsMimic(int iaxis=-1) {
        return _pjoint->IsMimic(iaxis);
    }
    string GetMimicEquation(int iaxis=0, int itype=0, const std::string& format="") {
        return _pjoint->GetMimicEquation(iaxis,itype,format);
    }
    object GetMimicDOFIndices(int iaxis=0) {
        std::vector<int> vmimicdofs;
        _pjoint->GetMimicDOFIndices(vmimicdofs,iaxis);
        return toPyArray(vmimicdofs);
    }
    void SetMimicEquations(int iaxis, const std::string& poseq, const std::string& veleq, const std::string& acceleq) {
        _pjoint->SetMimicEquations(iaxis,poseq,veleq,acceleq);
    }

    dReal GetMaxVel(int iaxis=0) const {
        return _pjoint->GetMaxVel(iaxis);
    }
    dReal GetMaxAccel(int iaxis=0) const {
        return _pjoint->GetMaxAccel(iaxis);
    }
    dReal GetMaxJerk(int iaxis=0) const {
        return _pjoint->GetMaxJerk(iaxis);
    }
    dReal GetMaxTorque(int iaxis=0) const {
        return _pjoint->GetMaxTorque(iaxis);
    }
    object GetInstantaneousTorqueLimits(int iaxis=0) const {
        std::pair<dReal, dReal> values = _pjoint->GetInstantaneousTorqueLimits(iaxis);
        return py::make_tuple(values.first, values.second);
    }
    object GetNominalTorqueLimits(int iaxis=0) const {
        std::pair<dReal, dReal> values = _pjoint->GetNominalTorqueLimits(iaxis);
        return py::make_tuple(values.first, values.second);
    }

    dReal GetMaxInertia(int iaxis=0) const {
        return _pjoint->GetMaxInertia(iaxis);
    }

    int GetDOFIndex() const {
        return _pjoint->GetDOFIndex();
    }
    int GetJointIndex() const {
        return _pjoint->GetJointIndex();
    }

    PyKinBodyPtr GetParent() const {
        return PyKinBodyPtr(new PyKinBody(_pjoint->GetParent(),_pyenv));
    }

    PyLinkPtr GetFirstAttached() const {
        return !_pjoint->GetFirstAttached() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetFirstAttached(), _pyenv));
    }
    PyLinkPtr GetSecondAttached() const {
        return !_pjoint->GetSecondAttached() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetSecondAttached(), _pyenv));
    }

    KinBody::JointType GetType() const {
        return _pjoint->GetType();
    }
    bool IsCircular(int iaxis) const {
        return _pjoint->IsCircular(iaxis);
    }
    bool IsRevolute(int iaxis) const {
        return _pjoint->IsRevolute(iaxis);
    }
    bool IsPrismatic(int iaxis) const {
        return _pjoint->IsPrismatic(iaxis);
    }
    bool IsStatic() const {
        return _pjoint->IsStatic();
    }

    int GetDOF() const {
        return _pjoint->GetDOF();
    }
    object GetValues() const {
        vector<dReal> values;
        _pjoint->GetValues(values);
        return toPyArray(values);
    }
    dReal GetValue(int iaxis) const {
        return _pjoint->GetValue(iaxis);
    }
    object GetVelocities() const {
        vector<dReal> values;
        _pjoint->GetVelocities(values);
        return toPyArray(values);
    }

    object GetAnchor() const {
        return toPyVector3(_pjoint->GetAnchor());
    }
    object GetAxis(int iaxis=0) {
        return toPyVector3(_pjoint->GetAxis(iaxis));
    }
    PyLinkPtr GetHierarchyParentLink() const {
        return !_pjoint->GetHierarchyParentLink() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetHierarchyParentLink(),_pyenv));
    }
    PyLinkPtr GetHierarchyChildLink() const {
        return !_pjoint->GetHierarchyChildLink() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetHierarchyChildLink(),_pyenv));
    }
    object GetInternalHierarchyAxis(int iaxis) {
        return toPyVector3(_pjoint->GetInternalHierarchyAxis(iaxis));
    }
    object GetInternalHierarchyLeftTransform() {
        return ReturnTransform(_pjoint->GetInternalHierarchyLeftTransform());
    }
    object GetInternalHierarchyLeftTransformPose() {
        return toPyArray(_pjoint->GetInternalHierarchyLeftTransform());
    }
    object GetInternalHierarchyRightTransform() {
        return ReturnTransform(_pjoint->GetInternalHierarchyRightTransform());
    }
    object GetInternalHierarchyRightTransformPose() {
        return toPyArray(_pjoint->GetInternalHierarchyRightTransform());
    }

    object GetLimits() const {
        vector<dReal> lower, upper;
        _pjoint->GetLimits(lower,upper);
        return py::make_tuple(toPyArray(lower),toPyArray(upper));
    }
    object GetVelocityLimits() const {
        vector<dReal> vlower,vupper;
        _pjoint->GetVelocityLimits(vlower,vupper);
        return py::make_tuple(toPyArray(vlower),toPyArray(vupper));
    }
    object GetAccelerationLimits() const {
        vector<dReal> v;
        _pjoint->GetAccelerationLimits(v);
        return toPyArray(v);
    }
    object GetJerkLimits() const {
        vector<dReal> v;
        _pjoint->GetJerkLimits(v);
        return toPyArray(v);
    }
    object GetHardVelocityLimits() const {
        vector<dReal> v;
        _pjoint->GetHardVelocityLimits(v);
        return toPyArray(v);
    }
    object GetHardAccelerationLimits() const {
        vector<dReal> v;
        _pjoint->GetHardAccelerationLimits(v);
        return toPyArray(v);
    }
    object GetHardJerkLimits() const {
        vector<dReal> v;
        _pjoint->GetHardJerkLimits(v);
        return toPyArray(v);
    }
    object GetTorqueLimits() const {
        vector<dReal> v;
        _pjoint->GetTorqueLimits(v);
        return toPyArray(v);
    }

    dReal GetWrapOffset(int iaxis=0) {
        return _pjoint->GetWrapOffset(iaxis);
    }
    void SetWrapOffset(dReal offset, int iaxis=0) {
        _pjoint->SetWrapOffset(offset,iaxis);
    }
    void SetLimits(object olower, object oupper) {
        vector<dReal> vlower = ExtractArray<dReal>(olower);
        vector<dReal> vupper = ExtractArray<dReal>(oupper);
        if(( vlower.size() != vupper.size()) ||( (int)vlower.size() != _pjoint->GetDOF()) ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetLimits(vlower,vupper);
    }
    void SetVelocityLimits(object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetVelocityLimits(vmaxlimits);
    }
    void SetAccelerationLimits(object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetAccelerationLimits(vmaxlimits);
    }
    void SetJerkLimits(object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetJerkLimits(vmaxlimits);
    }
    void SetHardVelocityLimits(object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetHardVelocityLimits(vmaxlimits);
    }
    void SetHardAccelerationLimits(object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetHardAccelerationLimits(vmaxlimits);
    }
    void SetHardJerkLimits(object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetHardJerkLimits(vmaxlimits);
    }
    void SetTorqueLimits(object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetTorqueLimits(vmaxlimits);
    }

    object GetResolutions() const {
        vector<dReal> resolutions;
        _pjoint->GetResolutions(resolutions);
        return toPyArray(resolutions);
    }
    dReal GetResolution(int iaxis) {
        return _pjoint->GetResolution(iaxis);
    }
    void SetResolution(dReal resolution) {
        _pjoint->SetResolution(resolution);
    }

    object GetWeights() const {
        vector<dReal> weights;
        _pjoint->GetWeights(weights);
        return toPyArray(weights);
    }
    dReal GetWeight(int iaxis) {
        return _pjoint->GetWeight(iaxis);
    }
    void SetWeights(object o) {
        _pjoint->SetWeights(ExtractArray<dReal>(o));
    }

    object SubtractValues(object ovalues0, object ovalues1) {
        vector<dReal> values0 = ExtractArray<dReal>(ovalues0);
        vector<dReal> values1 = ExtractArray<dReal>(ovalues1);
        BOOST_ASSERT((int)values0.size() == GetDOF() );
        BOOST_ASSERT((int)values1.size() == GetDOF() );
        _pjoint->SubtractValues(values0,values1);
        return toPyArray(values0);
    }

    dReal SubtractValue(dReal value0, dReal value1, int iaxis) {
        return _pjoint->SubtractValue(value0,value1,iaxis);
    }

    void AddTorque(object otorques) {
        vector<dReal> vtorques = ExtractArray<dReal>(otorques);
        return _pjoint->AddTorque(vtorques);
    }

    object GetFloatParameters(object oname=object(), int index=-1) const {
        return GetCustomParameters(_pjoint->GetFloatParameters(), oname, index);
    }

    void SetFloatParameters(const std::string& key, object oparameters)
    {
        _pjoint->SetFloatParameters(key,ExtractArray<dReal>(oparameters));
    }

    object GetIntParameters(object oname=object(), int index=-1) const {
        return GetCustomParameters(_pjoint->GetIntParameters(), oname, index);
    }

    void SetIntParameters(const std::string& key, object oparameters)
    {
        _pjoint->SetIntParameters(key,ExtractArray<int>(oparameters));
    }

    object GetStringParameters(object oname=object()) const {
        if( IS_PYTHONOBJECT_NONE(oname) ) {
            py::dict oparameters;
            FOREACHC(it, _pjoint->GetStringParameters()) {
                oparameters[it->first] = ConvertStringToUnicode(it->second);
            }
            return oparameters;
        }
        std::string name = py::extract<std::string>(oname);
        std::map<std::string, std::string >::const_iterator it = _pjoint->GetStringParameters().find(name);
        if( it != _pjoint->GetStringParameters().end() ) {
            return ConvertStringToUnicode(it->second);
        }
        return object();
    }

    void SetStringParameters(const std::string& key, object ovalue)
    {
        _pjoint->SetStringParameters(key,extract<std::string>(ovalue));
    }

    KinBody::JointControlMode GetControlMode() const {
        return _pjoint->GetControlMode();
    }

    void UpdateInfo() {
        _pjoint->UpdateInfo();
    }
    object GetInfo() {
        return object(PyJointInfoPtr(new PyJointInfo(_pjoint->GetInfo(), _pyenv)));
    }
    object UpdateAndGetInfo() {
        return object(PyJointInfoPtr(new PyJointInfo(_pjoint->UpdateAndGetInfo(), _pyenv)));
    }

    string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s').GetJoint('%s')")%RaveGetEnvironmentId(_pjoint->GetParent()->GetEnv())%_pjoint->GetParent()->GetName()%_pjoint->GetName());
    }
    string __str__() {
        return boost::str(boost::format("<joint:%s (%d), dof=%d, parent=%s>")%_pjoint->GetName()%_pjoint->GetJointIndex()%_pjoint->GetDOFIndex()%_pjoint->GetParent()->GetName());
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
    bool __eq__(boost::shared_ptr<PyJoint> p) {
        return !!p && _pjoint==p->_pjoint;
    }
    bool __ne__(boost::shared_ptr<PyJoint> p) {
        return !p || _pjoint!=p->_pjoint;
    }
    int __hash__() {
        return static_cast<int>(uintptr_t(_pjoint.get()));
    }
};

PyJointPtr toPyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv)
{
    if( !!pjoint ) {
        return PyJointPtr(new PyJoint(pjoint, pyenv));
    }
    else {
        return PyJointPtr();
    }
}

class PyKinBodyStateSaver
{
    PyEnvironmentBasePtr _pyenv;
    KinBody::KinBodyStateSaver _state;
public:
    PyKinBodyStateSaver(PyKinBodyPtr pybody) : _pyenv(pybody->GetEnv()), _state(pybody->GetBody()) {
        // python should not support restoring on destruction since there's garbage collection
        _state.SetRestoreOnDestructor(false);
    }
    PyKinBodyStateSaver(PyKinBodyPtr pybody, object options) : _pyenv(pybody->GetEnv()), _state(pybody->GetBody(),pyGetIntFromPy(options,0)) {
        // python should not support restoring on destruction since there's garbage collection
        _state.SetRestoreOnDestructor(false);
    }
    PyKinBodyStateSaver(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv) : _pyenv(pyenv), _state(pbody) {
        // python should not support restoring on destruction since there's garbage collection
        _state.SetRestoreOnDestructor(false);
    }
    PyKinBodyStateSaver(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv, object options) : _pyenv(pyenv), _state(pbody,pyGetIntFromPy(options, 0)) {
        // python should not support restoring on destruction since there's garbage collection
        _state.SetRestoreOnDestructor(false);
    }
    virtual ~PyKinBodyStateSaver() {
        _state.Release();
    }

    object GetBody() const {
        KinBodyPtr pbody = _state.GetBody();
        if( !pbody ) {
            return object();
        }
        if( pbody->IsRobot() ) {
            return object(openravepy::toPyRobot(RaveInterfaceCast<RobotBase>(pbody),_pyenv));
        }
        else {
            return object(openravepy::toPyKinBody(pbody,_pyenv));
        }
    }

    void Restore(PyKinBodyPtr pybody=PyKinBodyPtr()) {
        _state.Restore(!pybody ? KinBodyPtr() : pybody->GetBody());
    }

    void Release() {
        _state.Release();
    }

    std::string __str__() {
        KinBodyPtr pbody = _state.GetBody();
        if( !pbody ) {
            return "state empty";
        }
        return boost::str(boost::format("state for %s")%pbody->GetName());
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
};
typedef boost::shared_ptr<PyKinBodyStateSaver> PyKinBodyStateSaverPtr;

class PyManageData
{
    KinBody::ManageDataPtr _pdata;
    PyEnvironmentBasePtr _pyenv;
public:
    PyManageData(KinBody::ManageDataPtr pdata, PyEnvironmentBasePtr pyenv) : _pdata(pdata), _pyenv(pyenv) {
    }
    virtual ~PyManageData() {
    }

    KinBody::ManageDataPtr GetManageData() {
        return _pdata;
    }

    object GetSystem() {
        return object(openravepy::toPySensorSystem(_pdata->GetSystem(),_pyenv));
    }

    PyVoidHandleConst GetData() const {
        return PyVoidHandleConst(_pdata->GetData());
    }
    PyLinkPtr GetOffsetLink() const {
        KinBody::LinkPtr plink = _pdata->GetOffsetLink();
        return !plink ? PyLinkPtr() : PyLinkPtr(new PyLink(plink,_pyenv));
    }
    bool IsPresent() {
        return _pdata->IsPresent();
    }
    bool IsEnabled() {
        return _pdata->IsEnabled();
    }
    bool IsLocked() {
        return _pdata->IsLocked();
    }
    bool Lock(bool bDoLock) {
        return _pdata->Lock(bDoLock);
    }

    string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s').GetManageData()")%RaveGetEnvironmentId(_pdata->GetOffsetLink()->GetParent()->GetEnv())%_pdata->GetOffsetLink()->GetParent()->GetName());
    }
    string __str__() {
        KinBody::LinkPtr plink = _pdata->GetOffsetLink();
        SensorSystemBasePtr psystem = _pdata->GetSystem();
        string systemname = !psystem ? "(NONE)" : psystem->GetXMLId();
        return boost::str(boost::format("<managedata:%s, parent=%s:%s>")%systemname%plink->GetParent()->GetName()%plink->GetName());
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
    bool __eq__(boost::shared_ptr<PyManageData> p) {
        return !!p && _pdata==p->_pdata;
    }
    bool __ne__(boost::shared_ptr<PyManageData> p) {
        return !p || _pdata!=p->_pdata;
    }
};
typedef boost::shared_ptr<PyManageData> PyManageDataPtr;
typedef boost::shared_ptr<PyManageData const> PyManageDataConstPtr;

PyKinBody::PyKinBody(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pbody,pyenv), _pbody(pbody)
{
}

PyKinBody::PyKinBody(const PyKinBody& r) : PyInterfaceBase(r._pbody,r._pyenv)
{
    _pbody = r._pbody;
}

PyKinBody::~PyKinBody()
{
}

KinBodyPtr PyKinBody::GetBody()
{
    return _pbody;
}

void PyKinBody::Destroy()
{
    _pbody->Destroy();
}

bool PyKinBody::InitFromBoxes(const boost::multi_array<dReal,2>& vboxes, bool bDraw, const std::string& uri)
{
    if( vboxes.shape()[1] != 6 ) {
        throw openrave_exception(_("boxes needs to be a Nx6 vector\n"));
    }
    std::vector<AABB> vaabbs(vboxes.shape()[0]);
    for(size_t i = 0; i < vaabbs.size(); ++i) {
        vaabbs[i].pos = Vector(vboxes[i][0],vboxes[i][1],vboxes[i][2]);
        vaabbs[i].extents = Vector(vboxes[i][3],vboxes[i][4],vboxes[i][5]);
    }
    return _pbody->InitFromBoxes(vaabbs,bDraw,uri);
}
bool PyKinBody::InitFromSpheres(const boost::multi_array<dReal,2>& vspheres, bool bDraw, const std::string& uri)
{
    if( vspheres.shape()[1] != 4 ) {
        throw openrave_exception(_("spheres needs to be a Nx4 vector\n"));
    }
    std::vector<Vector> vvspheres(vspheres.shape()[0]);
    for(size_t i = 0; i < vvspheres.size(); ++i) {
        vvspheres[i] = Vector(vspheres[i][0],vspheres[i][1],vspheres[i][2],vspheres[i][3]);
    }
    return _pbody->InitFromSpheres(vvspheres,bDraw,uri);
}

bool PyKinBody::InitFromTrimesh(object pytrimesh, bool bDraw, const std::string& uri)
{
    TriMesh mesh;
    if( ExtractTriMesh(pytrimesh,mesh) ) {
        return _pbody->InitFromTrimesh(mesh,bDraw,uri);
    }
    else {
        throw openrave_exception(_("bad trimesh"));
    }
}

bool PyKinBody::InitFromGeometries(object ogeometries, const std::string& uri)
{
    std::vector<KinBody::GeometryInfoConstPtr> geometries(len(ogeometries));
    for(size_t i = 0; i < geometries.size(); ++i) {
        PyGeometryInfoPtr pygeom = py::extract<PyGeometryInfoPtr>(ogeometries[i]);
        if( !pygeom ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.GeometryInfo"),ORE_InvalidArguments);
        }
        geometries[i] = pygeom->GetGeometryInfo();
    }
    return _pbody->InitFromGeometries(geometries, uri);
}

bool PyKinBody::Init(object olinkinfos, object ojointinfos, const std::string& uri)
{
    std::vector<KinBody::LinkInfoConstPtr> vlinkinfos;
    _ParseLinkInfos(olinkinfos, vlinkinfos);
    std::vector<KinBody::JointInfoConstPtr> vjointinfos;
    _ParseJointInfos(ojointinfos, vjointinfos);
    return _pbody->Init(vlinkinfos, vjointinfos, uri);
}

void PyKinBody::SetLinkGeometriesFromGroup(const std::string& geomname)
{
    _pbody->SetLinkGeometriesFromGroup(geomname);
}

void PyKinBody::SetLinkGroupGeometries(const std::string& geomname, object olinkgeometryinfos)
{
    std::vector< std::vector<KinBody::GeometryInfoPtr> > linkgeometries(len(olinkgeometryinfos));
    for(size_t i = 0; i < linkgeometries.size(); ++i) {
        std::vector<KinBody::GeometryInfoPtr>& geometries = linkgeometries[i];
        geometries.resize(len(olinkgeometryinfos[i]));
        for(size_t j = 0; j < geometries.size(); ++j) {
            PyGeometryInfoPtr pygeom = py::extract<PyGeometryInfoPtr>(olinkgeometryinfos[i][j]);
            if( !pygeom ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.GeometryInfo"),ORE_InvalidArguments);
            }
            geometries[j] = pygeom->GetGeometryInfo();
        }
    }
    _pbody->SetLinkGroupGeometries(geomname, linkgeometries);
}

void PyKinBody::_ParseLinkInfos(object olinkinfos, std::vector<KinBody::LinkInfoConstPtr>& vlinkinfos)
{
    vlinkinfos.resize(len(olinkinfos));
    for(size_t i = 0; i < vlinkinfos.size(); ++i) {
        PyLinkInfoPtr pylink = py::extract<PyLinkInfoPtr>(olinkinfos[i]);
        if( !pylink ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.LinkInfo"),ORE_InvalidArguments);
        }
        vlinkinfos[i] = pylink->GetLinkInfo();
    }
}

void PyKinBody::_ParseJointInfos(object ojointinfos, std::vector<KinBody::JointInfoConstPtr>& vjointinfos)
{
    vjointinfos.resize(len(ojointinfos));
    for(size_t i = 0; i < vjointinfos.size(); ++i) {
        PyJointInfoPtr pyjoint = py::extract<PyJointInfoPtr>(ojointinfos[i]);
        if( !pyjoint ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.JointInfo"),ORE_InvalidArguments);
        }
        vjointinfos[i] = pyjoint->GetJointInfo();
    }
}

void PyKinBody::SetName(const std::string& name)
{
    _pbody->SetName(name);
}
object PyKinBody::GetName() const
{
    return ConvertStringToUnicode(_pbody->GetName());
}
int PyKinBody::GetDOF() const
{
    return _pbody->GetDOF();
}

object PyKinBody::GetDOFValues() const
{
    vector<dReal> values;
    _pbody->GetDOFValues(values);
    return toPyArray(values);
}
object PyKinBody::GetDOFValues(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> values;
    _pbody->GetDOFValues(values,vindices);
    return toPyArray(values);
}

object PyKinBody::GetDOFVelocities() const
{
    vector<dReal> values;
    _pbody->GetDOFVelocities(values);
    return toPyArray(values);
}

object PyKinBody::GetDOFVelocities(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> values;
    _pbody->GetDOFVelocities(values,vindices);
    return toPyArray(values);
}

object PyKinBody::GetDOFLimits() const
{
    vector<dReal> vlower, vupper;
    _pbody->GetDOFLimits(vlower,vupper);
    return py::make_tuple(toPyArray(vlower),toPyArray(vupper));
}

object PyKinBody::GetDOFVelocityLimits() const
{
    vector<dReal> vmax;
    _pbody->GetDOFVelocityLimits(vmax);
    return toPyArray(vmax);
}

object PyKinBody::GetDOFAccelerationLimits() const
{
    vector<dReal> vmax;
    _pbody->GetDOFAccelerationLimits(vmax);
    return toPyArray(vmax);
}

object PyKinBody::GetDOFJerkLimits() const
{
    vector<dReal> vmax;
    _pbody->GetDOFJerkLimits(vmax);
    return toPyArray(vmax);
}

object PyKinBody::GetDOFHardVelocityLimits() const
{
    vector<dReal> vmax;
    _pbody->GetDOFHardVelocityLimits(vmax);
    return toPyArray(vmax);
}

object PyKinBody::GetDOFHardAccelerationLimits() const
{
    vector<dReal> vmax;
    _pbody->GetDOFHardAccelerationLimits(vmax);
    return toPyArray(vmax);
}

object PyKinBody::GetDOFHardJerkLimits() const
{
    vector<dReal> vmax;
    _pbody->GetDOFHardJerkLimits(vmax);
    return toPyArray(vmax);
}

object PyKinBody::GetDOFTorqueLimits() const
{
    vector<dReal> vmax;
    _pbody->GetDOFTorqueLimits(vmax);
    return toPyArray(vmax);
}

object PyKinBody::GetDOFLimits(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return py::make_tuple(numeric::array(py::list()), numeric::array(py::list())); // always need 2 since users can do lower, upper = GetDOFLimits()
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return py::make_tuple(numeric::array(py::list()), numeric::array(py::list())); // always need 2 since users can do lower, upper = GetDOFLimits()
    }
    vector<dReal> vlower, vupper, vtemplower, vtempupper;
    vlower.reserve(vindices.size());
    vupper.reserve(vindices.size());
    FOREACHC(it, vindices) {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
        pjoint->GetLimits(vtemplower,vtempupper,false);
        vlower.push_back(vtemplower.at(*it-pjoint->GetDOFIndex()));
        vupper.push_back(vtempupper.at(*it-pjoint->GetDOFIndex()));
    }
    return py::make_tuple(toPyArray(vlower),toPyArray(vupper));
}

object PyKinBody::GetDOFVelocityLimits(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> vmax, vtempmax;
    vmax.reserve(vindices.size());
    FOREACHC(it, vindices) {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
        pjoint->GetVelocityLimits(vtempmax,false);
        vmax.push_back(vtempmax.at(*it-pjoint->GetDOFIndex()));
    }
    return toPyArray(vmax);
}

object PyKinBody::GetDOFAccelerationLimits(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> vmax, vtempmax;
    vmax.reserve(vindices.size());
    FOREACHC(it, vindices) {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
        pjoint->GetAccelerationLimits(vtempmax,false);
        vmax.push_back(vtempmax.at(*it-pjoint->GetDOFIndex()));
    }
    return toPyArray(vmax);
}

object PyKinBody::GetDOFJerkLimits(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> vmax, vtempmax;
    vmax.reserve(vindices.size());
    FOREACHC(it, vindices) {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
        pjoint->GetJerkLimits(vtempmax,false);
        vmax.push_back(vtempmax.at(*it-pjoint->GetDOFIndex()));
    }
    return toPyArray(vmax);
}

object PyKinBody::GetDOFHardVelocityLimits(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> vmax, vtempmax;
    vmax.reserve(vindices.size());
    FOREACHC(it, vindices) {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
        pjoint->GetHardVelocityLimits(vtempmax,false);
        vmax.push_back(vtempmax.at(*it-pjoint->GetDOFIndex()));
    }
    return toPyArray(vmax);
}

object PyKinBody::GetDOFHardAccelerationLimits(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> vmax, vtempmax;
    vmax.reserve(vindices.size());
    FOREACHC(it, vindices) {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
        pjoint->GetHardAccelerationLimits(vtempmax,false);
        vmax.push_back(vtempmax.at(*it-pjoint->GetDOFIndex()));
    }
    return toPyArray(vmax);
}

object PyKinBody::GetDOFHardJerkLimits(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> vmax, vtempmax;
    vmax.reserve(vindices.size());
    FOREACHC(it, vindices) {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
        pjoint->GetHardJerkLimits(vtempmax,false);
        vmax.push_back(vtempmax.at(*it-pjoint->GetDOFIndex()));
    }
    return toPyArray(vmax);
}

object PyKinBody::GetDOFTorqueLimits(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> vmax, vtempmax;
    vmax.reserve(vindices.size());
    FOREACHC(it, vindices) {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
        pjoint->GetTorqueLimits(vtempmax,false);
        vmax.push_back(vtempmax.at(*it-pjoint->GetDOFIndex()));
    }
    return toPyArray(vmax);
}

object PyKinBody::GetDOFMaxVel() const
{
    RAVELOG_WARN("KinBody.GetDOFMaxVel() is deprecated, use GetDOFVelocityLimits\n");
    vector<dReal> values;
    _pbody->GetDOFVelocityLimits(values);
    return toPyArray(values);
}
object PyKinBody::GetDOFMaxTorque() const
{
    vector<dReal> values;
    _pbody->GetDOFMaxTorque(values);
    return toPyArray(values);
}
object PyKinBody::GetDOFMaxAccel() const
{
    RAVELOG_WARN("KinBody.GetDOFMaxAccel() is deprecated, use GetDOFAccelerationLimits\n");
    vector<dReal> values;
    _pbody->GetDOFAccelerationLimits(values);
    return toPyArray(values);
}

object PyKinBody::GetDOFWeights() const
{
    vector<dReal> values;
    _pbody->GetDOFWeights(values);
    return toPyArray(values);
}

object PyKinBody::GetDOFWeights(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> values, v;
    values.reserve(vindices.size());
    FOREACHC(it, vindices) {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
        values.push_back(pjoint->GetWeight(*it-pjoint->GetDOFIndex()));
    }
    return toPyArray(values);
}

object PyKinBody::GetDOFResolutions() const
{
    vector<dReal> values;
    _pbody->GetDOFResolutions(values);
    return toPyArray(values);
}

object PyKinBody::GetDOFResolutions(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return numeric::array(py::list());
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    if( vindices.size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> values, v;
    values.reserve(vindices.size());
    FOREACHC(it, vindices) {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
        values.push_back(pjoint->GetResolution());
    }
    return toPyArray(values);
}

object PyKinBody::GetLinks() const
{
    py::list links;
    FOREACHC(itlink, _pbody->GetLinks()) {
        links.append(PyLinkPtr(new PyLink(*itlink, GetEnv())));
    }
    return links;
}

object PyKinBody::GetLinks(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return GetLinks();
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    py::list links;
    FOREACHC(it, vindices) {
        links.append(PyLinkPtr(new PyLink(_pbody->GetLinks().at(*it),GetEnv())));
    }
    return links;
}

object PyKinBody::GetLink(const std::string& linkname) const
{
    KinBody::LinkPtr plink = _pbody->GetLink(linkname);
    return !plink ? object() : object(PyLinkPtr(new PyLink(plink,GetEnv())));
}

object PyKinBody::GetJoints() const
{
    py::list joints;
    FOREACHC(itjoint, _pbody->GetJoints()) {
        joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
    }
    return joints;
}

object PyKinBody::GetJoints(object oindices) const
{
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        return GetJoints();
    }
    vector<int> vindices = ExtractArray<int>(oindices);
    py::list joints;
    FOREACHC(it, vindices) {
        joints.append(PyJointPtr(new PyJoint(_pbody->GetJoints().at(*it),GetEnv())));
    }
    return joints;
}

object PyKinBody::GetPassiveJoints()
{
    py::list joints;
    FOREACHC(itjoint, _pbody->GetPassiveJoints()) {
        joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
    }
    return joints;
}

object PyKinBody::GetDependencyOrderedJoints()
{
    py::list joints;
    FOREACHC(itjoint, _pbody->GetDependencyOrderedJoints()) {
        joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
    }
    return joints;
}

object PyKinBody::GetClosedLoops()
{
    py::list loops;
    FOREACHC(itloop, _pbody->GetClosedLoops()) {
        py::list loop;
        FOREACHC(itpair,*itloop) {
            loop.append(py::make_tuple(PyLinkPtr(new PyLink(itpair->first,GetEnv())),PyJointPtr(new PyJoint(itpair->second,GetEnv()))));
        }
        loops.append(loop);
    }
    return loops;
}

object PyKinBody::GetRigidlyAttachedLinks(int linkindex) const
{
    RAVELOG_WARN("KinBody.GetRigidlyAttachedLinks is deprecated, use KinBody.Link.GetRigidlyAttachedLinks\n");
    std::vector<KinBody::LinkPtr> vattachedlinks;
    _pbody->GetLinks().at(linkindex)->GetRigidlyAttachedLinks(vattachedlinks);
    py::list links;
    FOREACHC(itlink, vattachedlinks) {
        links.append(PyLinkPtr(new PyLink(*itlink, GetEnv())));
    }
    return links;
}

object PyKinBody::GetChain(int linkindex1, int linkindex2,bool returnjoints) const
{
    py::list chain;
    if( returnjoints ) {
        std::vector<KinBody::JointPtr> vjoints;
        _pbody->GetChain(linkindex1,linkindex2,vjoints);
        FOREACHC(itjoint, vjoints) {
            chain.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
        }
    }
    else {
        std::vector<KinBody::LinkPtr> vlinks;
        _pbody->GetChain(linkindex1,linkindex2,vlinks);
        FOREACHC(itlink, vlinks) {
            chain.append(PyLinkPtr(new PyLink(*itlink, GetEnv())));
        }
    }
    return chain;
}

bool PyKinBody::IsDOFInChain(int linkindex1, int linkindex2, int dofindex) const
{
    return _pbody->IsDOFInChain(linkindex1,linkindex2,dofindex);
}

int PyKinBody::GetJointIndex(const std::string& jointname) const
{
    return _pbody->GetJointIndex(jointname);
}

object PyKinBody::GetJoint(const std::string& jointname) const
{
    KinBody::JointPtr pjoint = _pbody->GetJoint(jointname);
    return !pjoint ? object() : object(PyJointPtr(new PyJoint(pjoint,GetEnv())));
}

object PyKinBody::GetJointFromDOFIndex(int dofindex) const
{
    KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(dofindex);
    return !pjoint ? object() : object(PyJointPtr(new PyJoint(pjoint,GetEnv())));
}

object PyKinBody::GetTransform() const {
    return ReturnTransform(_pbody->GetTransform());
}

object PyKinBody::GetTransformPose() const {
    return toPyArray(_pbody->GetTransform());
}

object PyKinBody::GetLinkTransformations(bool returndoflastvlaues) const
{
    py::list otransforms;
    vector<Transform> vtransforms;
    std::vector<dReal> vdoflastsetvalues;
    _pbody->GetLinkTransformations(vtransforms, vdoflastsetvalues);
    FOREACHC(it, vtransforms) {
        otransforms.append(ReturnTransform(*it));
    }
    if( returndoflastvlaues ) {
        return py::make_tuple(otransforms, toPyArray(vdoflastsetvalues));
    }
    return otransforms;
}

void PyKinBody::SetLinkTransformations(object transforms, object odoflastvalues)
{
    size_t numtransforms = len(transforms);
    if( numtransforms != _pbody->GetLinks().size() ) {
        throw openrave_exception(_("number of input transforms not equal to links"));
    }
    std::vector<Transform> vtransforms(numtransforms);
    for(size_t i = 0; i < numtransforms; ++i) {
        vtransforms[i] = ExtractTransform(transforms[i]);
    }
    if( IS_PYTHONOBJECT_NONE(odoflastvalues) ) {
        _pbody->SetLinkTransformations(vtransforms);
    }
    else {
        _pbody->SetLinkTransformations(vtransforms, ExtractArray<dReal>(odoflastvalues));
    }
}

void PyKinBody::SetLinkVelocities(object ovelocities)
{
    std::vector<std::pair<Vector,Vector> > velocities;
    velocities.resize(len(ovelocities));
    for(size_t i = 0; i < velocities.size(); ++i) {
        vector<dReal> v = ExtractArray<dReal>(ovelocities[i]);
        BOOST_ASSERT(v.size()==6);
        velocities[i].first.x = v[0];
        velocities[i].first.y = v[1];
        velocities[i].first.z = v[2];
        velocities[i].second.x = v[3];
        velocities[i].second.y = v[4];
        velocities[i].second.z = v[5];
    }
    return _pbody->SetLinkVelocities(velocities);
}

object PyKinBody::GetLinkEnableStates() const
{
    std::vector<uint8_t> enablestates;
    _pbody->GetLinkEnableStates(enablestates);
    return toPyArray(enablestates);
}

void PyKinBody::SetLinkEnableStates(object oenablestates)
{
    std::vector<uint8_t> enablestates = ExtractArray<uint8_t>(oenablestates);
    _pbody->SetLinkEnableStates(enablestates);
}

bool PyKinBody::SetVelocity(object olinearvel, object oangularvel)
{
    return _pbody->SetVelocity(ExtractVector3(olinearvel),ExtractVector3(oangularvel));
}

void PyKinBody::SetDOFVelocities(object odofvelocities, object olinearvel, object oangularvel, uint32_t checklimits)
{
    _pbody->SetDOFVelocities(ExtractArray<dReal>(odofvelocities),ExtractVector3(olinearvel),ExtractVector3(oangularvel),checklimits);
}

void PyKinBody::SetDOFVelocities(object odofvelocities, object olinearvel, object oangularvel)
{
    _pbody->SetDOFVelocities(ExtractArray<dReal>(odofvelocities),ExtractVector3(olinearvel),ExtractVector3(oangularvel));
}

void PyKinBody::SetDOFVelocities(object odofvelocities)
{
    _pbody->SetDOFVelocities(ExtractArray<dReal>(odofvelocities));
}

void PyKinBody::SetDOFVelocities(object odofvelocities, uint32_t checklimits, object oindices)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> vsetvalues = ExtractArray<dReal>(odofvelocities);
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        _pbody->SetDOFVelocities(vsetvalues,checklimits);
    }
    else {
        if( len(oindices) == 0 ) {
            return;
        }
        vector<int> vindices = ExtractArray<int>(oindices);
        _pbody->SetDOFVelocities(vsetvalues,checklimits, vindices);
    }
}

object PyKinBody::GetLinkVelocities() const
{
    if( _pbody->GetLinks().size() == 0 ) {
        return numeric::array(py::list());
    }
    std::vector<std::pair<Vector,Vector> > velocities;
    _pbody->GetLinkVelocities(velocities);

    npy_intp dims[] = {npy_intp(velocities.size()),npy_intp(6)};
    PyObject *pyvel = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pfvel = (dReal*)PyArray_DATA(pyvel);
    for(size_t i = 0; i < velocities.size(); ++i) {
        pfvel[6*i+0] = velocities[i].first.x;
        pfvel[6*i+1] = velocities[i].first.y;
        pfvel[6*i+2] = velocities[i].first.z;
        pfvel[6*i+3] = velocities[i].second.x;
        pfvel[6*i+4] = velocities[i].second.y;
        pfvel[6*i+5] = velocities[i].second.z;
    }
    return static_cast<numeric::array>(handle<>(pyvel));
}

object PyKinBody::GetLinkAccelerations(object odofaccelerations, object oexternalaccelerations=object()) const
{
    if( _pbody->GetLinks().size() == 0 ) {
        return numeric::array(py::list());
    }
    vector<dReal> vDOFAccelerations = ExtractArray<dReal>(odofaccelerations);
    KinBody::AccelerationMapPtr pmapExternalAccelerations;
    if( !IS_PYTHONOBJECT_NONE(oexternalaccelerations) ) {
        //externalaccelerations
        pmapExternalAccelerations.reset(new KinBody::AccelerationMap());
        py::dict odict = (py::dict)oexternalaccelerations;
        py::list iterkeys = (py::list)odict.iterkeys();
        vector<dReal> v;
        for (int i = 0; i < py::len(iterkeys); i++) {
            int linkindex = py::extract<int>(iterkeys[i]);
            object olinkaccelerations = odict[iterkeys[i]];
            OPENRAVE_ASSERT_OP(len(olinkaccelerations),==,6);
            (*pmapExternalAccelerations)[linkindex] = make_pair(Vector(py::extract<dReal>(olinkaccelerations[0]),py::extract<dReal>(olinkaccelerations[1]),py::extract<dReal>(olinkaccelerations[2])),Vector(py::extract<dReal>(olinkaccelerations[3]),py::extract<dReal>(olinkaccelerations[4]),py::extract<dReal>(olinkaccelerations[5])));
        }
    }
    std::vector<std::pair<Vector,Vector> > vLinkAccelerations;
    _pbody->GetLinkAccelerations(vDOFAccelerations, vLinkAccelerations, pmapExternalAccelerations);

    npy_intp dims[] = {npy_intp(vLinkAccelerations.size()),npy_intp(6)};
    PyObject *pyaccel = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pf = (dReal*)PyArray_DATA(pyaccel);
    for(size_t i = 0; i < vLinkAccelerations.size(); ++i) {
        pf[6*i+0] = vLinkAccelerations[i].first.x;
        pf[6*i+1] = vLinkAccelerations[i].first.y;
        pf[6*i+2] = vLinkAccelerations[i].first.z;
        pf[6*i+3] = vLinkAccelerations[i].second.x;
        pf[6*i+4] = vLinkAccelerations[i].second.y;
        pf[6*i+5] = vLinkAccelerations[i].second.z;
    }
    return static_cast<numeric::array>(handle<>(pyaccel));
}

object PyKinBody::ComputeAABB(bool bEnabledOnlyLinks)
{
    return toPyAABB(_pbody->ComputeAABB(bEnabledOnlyLinks));
}

object PyKinBody::ComputeAABBFromTransform(object otransform, bool bEnabledOnlyLinks)
{
    return toPyAABB(_pbody->ComputeAABBFromTransform(ExtractTransform(otransform), bEnabledOnlyLinks));
}

object PyKinBody::ComputeLocalAABB(bool bEnabledOnlyLinks)
{
    return toPyAABB(_pbody->ComputeLocalAABB(bEnabledOnlyLinks));
}

object PyKinBody::GetCenterOfMass() const
{
    return toPyVector3(_pbody->GetCenterOfMass());
}

void PyKinBody::Enable(bool bEnable)
{
    _pbody->Enable(bEnable);
}
bool PyKinBody::IsEnabled() const
{
    return _pbody->IsEnabled();
}
bool PyKinBody::SetVisible(bool visible)
{
    return _pbody->SetVisible(visible);
}
bool PyKinBody::IsVisible() const
{
    return _pbody->IsVisible();
}

bool PyKinBody::IsDOFRevolute(int dofindex) const
{
    return _pbody->IsDOFRevolute(dofindex);
}

bool PyKinBody::IsDOFPrismatic(int dofindex) const
{
    return _pbody->IsDOFPrismatic(dofindex);
}

void PyKinBody::SetTransform(object transform)
{
    _pbody->SetTransform(ExtractTransform(transform));
}

void PyKinBody::SetDOFWeights(object o)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(o);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFWeights(values);
}

void PyKinBody::SetDOFResolutions(object o)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(o);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFResolutions(values);
}

void PyKinBody::SetDOFLimits(object olower, object oupper, object oindices)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> vlower = ExtractArray<dReal>(olower), vupper = ExtractArray<dReal>(oupper);
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        if( (int)vlower.size() != GetDOF() || (int)vupper.size() != GetDOF() ) {
            throw openrave_exception(_("values do not equal to body degrees of freedom"));
        }
        _pbody->SetDOFLimits(vlower,vupper);
    }
    else {
        if( len(oindices) == 0 ) {
            return;
        }
        vector<int> vindices = ExtractArray<int>(oindices);
        _pbody->SetDOFLimits(vlower, vupper, vindices);
    }
}

void PyKinBody::SetDOFVelocityLimits(object o)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(o);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFVelocityLimits(values);
}

void PyKinBody::SetDOFAccelerationLimits(object o)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(o);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFAccelerationLimits(values);
}

void PyKinBody::SetDOFJerkLimits(object o)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(o);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFJerkLimits(values);
}

void PyKinBody::SetDOFHardVelocityLimits(object o)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(o);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFHardVelocityLimits(values);
}

void PyKinBody::SetDOFHardAccelerationLimits(object o)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(o);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFHardAccelerationLimits(values);
}

void PyKinBody::SetDOFHardJerkLimits(object o)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(o);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFHardJerkLimits(values);
}

void PyKinBody::SetDOFTorqueLimits(object o)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(o);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFTorqueLimits(values);
}

void PyKinBody::SetDOFValues(object o)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(o);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFValues(values,KinBody::CLA_CheckLimits);
}
void PyKinBody::SetTransformWithDOFValues(object otrans,object ojoints)
{
    if( _pbody->GetDOF() == 0 ) {
        _pbody->SetTransform(ExtractTransform(otrans));
        return;
    }
    vector<dReal> values = ExtractArray<dReal>(ojoints);
    if( (int)values.size() != GetDOF() ) {
        throw openrave_exception(_("values do not equal to body degrees of freedom"));
    }
    _pbody->SetDOFValues(values,ExtractTransform(otrans),KinBody::CLA_CheckLimits);
}

void PyKinBody::SetDOFValues(object o, object indices, uint32_t checklimits)
{
    if( _pbody->GetDOF() == 0 ) {
        return;
    }
    vector<dReal> vsetvalues = ExtractArray<dReal>(o);
    if( IS_PYTHONOBJECT_NONE(indices) ) {
        _pbody->SetDOFValues(vsetvalues,checklimits);
    }
    else {
        if( len(indices) == 0 ) {
            return;
        }
        vector<int> vindices = ExtractArray<int>(indices);
        _pbody->SetDOFValues(vsetvalues,checklimits, vindices);
    }
}

void PyKinBody::SetDOFValues(object o, object indices)
{
    SetDOFValues(o,indices,KinBody::CLA_CheckLimits);
}

object PyKinBody::SubtractDOFValues(object ovalues0, object ovalues1, object oindices)
{
    vector<dReal> values0 = ExtractArray<dReal>(ovalues0);
    vector<dReal> values1 = ExtractArray<dReal>(ovalues1);
    vector<int> vindices;
    if( IS_PYTHONOBJECT_NONE(oindices) ) {
        OPENRAVE_ASSERT_OP((int)values0.size(), ==, GetDOF());
        OPENRAVE_ASSERT_OP((int)values1.size(), ==, GetDOF());
        _pbody->SubtractDOFValues(values0,values1);
    }
    else {
        vindices = ExtractArray<int>(oindices);
        OPENRAVE_ASSERT_OP(values0.size(), ==, vindices.size());
        OPENRAVE_ASSERT_OP(values1.size(), ==, vindices.size());
        _pbody->SubtractDOFValues(values0,values1,vindices);
    }
    return toPyArray(values0);
}

void PyKinBody::SetDOFTorques(object otorques, bool bAdd)
{
    vector<dReal> vtorques = ExtractArray<dReal>(otorques);
    BOOST_ASSERT((int)vtorques.size() == GetDOF() );
    _pbody->SetDOFTorques(vtorques,bAdd);
}

object PyKinBody::ComputeJacobianTranslation(int index, object oposition, object oindices)
{
    vector<int> vindices;
    if( !IS_PYTHONOBJECT_NONE(oindices) ) {
        vindices = ExtractArray<int>(oindices);
    }
    std::vector<dReal> vjacobian;
    _pbody->ComputeJacobianTranslation(index,ExtractVector3(oposition),vjacobian,vindices);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = vjacobian.size()/3;
    return toPyArray(vjacobian,dims);
}

object PyKinBody::ComputeJacobianAxisAngle(int index, object oindices)
{
    vector<int> vindices;
    if( !IS_PYTHONOBJECT_NONE(oindices) ) {
        vindices = ExtractArray<int>(oindices);
    }
    std::vector<dReal> vjacobian;
    _pbody->ComputeJacobianAxisAngle(index,vjacobian,vindices);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = vjacobian.size()/3;
    return toPyArray(vjacobian,dims);
}

object PyKinBody::CalculateJacobian(int index, object oposition)
{
    std::vector<dReal> vjacobian;
    _pbody->CalculateJacobian(index,ExtractVector3(oposition),vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = vjacobian.size()/3;
    return toPyArray(vjacobian,dims);
}

object PyKinBody::CalculateRotationJacobian(int index, object q) const
{
    std::vector<dReal> vjacobian;
    _pbody->CalculateRotationJacobian(index,ExtractVector4(q),vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 4; dims[1] = _pbody->GetDOF();
    return toPyArray(vjacobian,dims);
}

object PyKinBody::CalculateAngularVelocityJacobian(int index) const
{
    std::vector<dReal> vjacobian;
    _pbody->ComputeJacobianAxisAngle(index,vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _pbody->GetDOF();
    return toPyArray(vjacobian,dims);
}

object PyKinBody::ComputeHessianTranslation(int index, object oposition, object oindices)
{
    vector<int> vindices;
    if( !IS_PYTHONOBJECT_NONE(oindices) ) {
        vindices = ExtractArray<int>(oindices);
    }
    size_t dof = vindices.size() == 0 ? (size_t)_pbody->GetDOF() : vindices.size();
    std::vector<dReal> vhessian;
    _pbody->ComputeHessianTranslation(index,ExtractVector3(oposition),vhessian,vindices);
    std::vector<npy_intp> dims(3); dims[0] = dof; dims[1] = 3; dims[2] = dof;
    return toPyArray(vhessian,dims);
}

object PyKinBody::ComputeHessianAxisAngle(int index, object oindices)
{
    vector<int> vindices;
    if( !IS_PYTHONOBJECT_NONE(oindices) ) {
        vindices = ExtractArray<int>(oindices);
    }
    size_t dof = vindices.size() == 0 ? (size_t)_pbody->GetDOF() : vindices.size();
    std::vector<dReal> vhessian;
    _pbody->ComputeHessianAxisAngle(index,vhessian,vindices);
    std::vector<npy_intp> dims(3); dims[0] = dof; dims[1] = 3; dims[2] = dof;
    return toPyArray(vhessian,dims);
}

object PyKinBody::ComputeInverseDynamics(object odofaccelerations, object oexternalforcetorque, bool returncomponents)
{
    vector<dReal> vDOFAccelerations;
    if( !IS_PYTHONOBJECT_NONE(odofaccelerations) ) {
        vDOFAccelerations = ExtractArray<dReal>(odofaccelerations);
    }
    KinBody::ForceTorqueMap mapExternalForceTorque;
    if( !IS_PYTHONOBJECT_NONE(oexternalforcetorque) ) {
        py::dict odict = (py::dict)oexternalforcetorque;
        py::list iterkeys = (py::list)odict.iterkeys();
        vector<dReal> v;
        for (int i = 0; i < py::len(iterkeys); i++) {
            int linkindex = py::extract<int>(iterkeys[i]);
            object oforcetorque = odict[iterkeys[i]];
            OPENRAVE_ASSERT_OP(len(oforcetorque),==,6);
            mapExternalForceTorque[linkindex] = make_pair(Vector(py::extract<dReal>(oforcetorque[0]),py::extract<dReal>(oforcetorque[1]),py::extract<dReal>(oforcetorque[2])),Vector(py::extract<dReal>(oforcetorque[3]),py::extract<dReal>(oforcetorque[4]),py::extract<dReal>(oforcetorque[5])));
        }
    }
    if( returncomponents ) {
        boost::array< vector<dReal>, 3> vDOFTorqueComponents;
        _pbody->ComputeInverseDynamics(vDOFTorqueComponents,vDOFAccelerations,mapExternalForceTorque);
        return py::make_tuple(toPyArray(vDOFTorqueComponents[0]), toPyArray(vDOFTorqueComponents[1]), toPyArray(vDOFTorqueComponents[2]));
    }
    else {
        vector<dReal> vDOFTorques;
        _pbody->ComputeInverseDynamics(vDOFTorques,vDOFAccelerations,mapExternalForceTorque);
        return toPyArray(vDOFTorques);
    }
}

void PyKinBody::SetSelfCollisionChecker(PyCollisionCheckerBasePtr pycollisionchecker)
{
    _pbody->SetSelfCollisionChecker(openravepy::GetCollisionChecker(pycollisionchecker));
}

PyInterfaceBasePtr PyKinBody::GetSelfCollisionChecker()
{
    return openravepy::toPyCollisionChecker(_pbody->GetSelfCollisionChecker(), _pyenv);
}

bool PyKinBody::CheckSelfCollision(PyCollisionReportPtr pReport, PyCollisionCheckerBasePtr pycollisionchecker)
{
    bool bCollision = _pbody->CheckSelfCollision(openravepy::GetCollisionReport(pReport), openravepy::GetCollisionChecker(pycollisionchecker));
    openravepy::UpdateCollisionReport(pReport,GetEnv());
    return bCollision;
}

bool PyKinBody::IsAttached(PyKinBodyPtr pattachbody)
{
    CHECK_POINTER(pattachbody);
    return _pbody->IsAttached(*pattachbody->GetBody());
}
object PyKinBody::GetAttached() const
{
    py::list attached;
    std::set<KinBodyPtr> vattached;
    _pbody->GetAttached(vattached);
    FOREACHC(it,vattached)
    attached.append(PyKinBodyPtr(new PyKinBody(*it,_pyenv)));
    return attached;
}

void PyKinBody::SetZeroConfiguration()
{
    _pbody->SetZeroConfiguration();
}
void PyKinBody::SetNonCollidingConfiguration()
{
    _pbody->SetNonCollidingConfiguration();
}

object PyKinBody::GetConfigurationSpecification(const std::string& interpolation) const
{
    return object(openravepy::toPyConfigurationSpecification(_pbody->GetConfigurationSpecification(interpolation)));
}

object PyKinBody::GetConfigurationSpecificationIndices(object oindices,const std::string& interpolation) const
{
    vector<int> vindices = ExtractArray<int>(oindices);
    return object(openravepy::toPyConfigurationSpecification(_pbody->GetConfigurationSpecificationIndices(vindices,interpolation)));
}

void PyKinBody::SetConfigurationValues(object ovalues, uint32_t checklimits)
{
    vector<dReal> vvalues = ExtractArray<dReal>(ovalues);
    BOOST_ASSERT((int)vvalues.size()==_pbody->GetDOF()+7);
    _pbody->SetConfigurationValues(vvalues.begin(),checklimits);
}

object PyKinBody::GetConfigurationValues() const
{
    vector<dReal> values;
    _pbody->GetConfigurationValues(values);
    return toPyArray(values);
}


bool PyKinBody::Grab(PyKinBodyPtr pbody, object pylink, object linkstoignore)
{
    CHECK_POINTER(pbody);
    CHECK_POINTER(pylink);
    std::set<int> setlinkstoignore = ExtractSet<int>(linkstoignore);
    return _pbody->Grab(pbody->GetBody(), GetKinBodyLink(pylink), setlinkstoignore);
}

bool PyKinBody::Grab(PyKinBodyPtr pbody, object pylink)
{
    CHECK_POINTER(pbody);
    CHECK_POINTER(pylink);
    KinBody::LinkPtr plink = GetKinBodyLink(pylink);
    return _pbody->Grab(pbody->GetBody(), plink);
}

void PyKinBody::Release(PyKinBodyPtr pbody)
{
    CHECK_POINTER(pbody); _pbody->Release(*pbody->GetBody());
}
void PyKinBody::ReleaseAllGrabbed() {
    _pbody->ReleaseAllGrabbed();
}
void PyKinBody::ReleaseAllGrabbedWithLink(object pylink) {
    CHECK_POINTER(pylink);
    KinBody::LinkPtr plink = GetKinBodyLink(pylink);
    _pbody->ReleaseAllGrabbedWithLink(*plink);
}
void PyKinBody::RegrabAll()
{
    _pbody->RegrabAll();
}
object PyKinBody::IsGrabbing(PyKinBodyPtr pbody) const
{
    CHECK_POINTER(pbody);
    KinBody::LinkPtr plink = _pbody->IsGrabbing(*pbody->GetBody());
    return toPyKinBodyLink(plink,_pyenv);
}

object PyKinBody::GetGrabbed() const
{
    py::list bodies;
    std::vector<KinBodyPtr> vbodies;
    _pbody->GetGrabbed(vbodies);
    FOREACH(itbody, vbodies) {
        bodies.append(PyKinBodyPtr(new PyKinBody(*itbody,_pyenv)));
    }
    return bodies;
}

object PyKinBody::GetGrabbedInfo() const
{
    py::list ograbbed;
    std::vector<RobotBase::GrabbedInfoPtr> vgrabbedinfo;
    _pbody->GetGrabbedInfo(vgrabbedinfo);
    FOREACH(itgrabbed, vgrabbedinfo) {
        ograbbed.append(PyGrabbedInfoPtr(new PyGrabbedInfo(**itgrabbed)));
    }
    return ograbbed;
}

void PyKinBody::ResetGrabbed(object ograbbedinfos)
{
    std::vector<RobotBase::GrabbedInfoConstPtr> vgrabbedinfos(len(ograbbedinfos));
    for(size_t i = 0; i < vgrabbedinfos.size(); ++i) {
        PyGrabbedInfoPtr pygrabbed = py::extract<PyGrabbedInfoPtr>(ograbbedinfos[i]);
        if( !pygrabbed ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to Robot.GrabbedInfo"),ORE_InvalidArguments);
        }
        vgrabbedinfos[i] = pygrabbed->GetGrabbedInfo();
    }
    _pbody->ResetGrabbed(vgrabbedinfos);
}

bool PyKinBody::IsRobot() const
{
    return _pbody->IsRobot();
}
int PyKinBody::GetEnvironmentId() const
{
    return _pbody->GetEnvironmentId();
}

int PyKinBody::DoesAffect(int jointindex, int linkindex ) const
{
    return _pbody->DoesAffect(jointindex,linkindex);
}

int PyKinBody::DoesDOFAffectLink(int dofindex, int linkindex ) const
{
    return _pbody->DoesDOFAffectLink(dofindex,linkindex);
}

object PyKinBody::GetURI() const
{
    return ConvertStringToUnicode(_pbody->GetURI());
}

object PyKinBody::GetNonAdjacentLinks() const
{
    py::list ononadjacent;
    const std::vector<int>& nonadjacent = _pbody->GetNonAdjacentLinks();
    FOREACHC(it,nonadjacent) {
        ononadjacent.append(py::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
    }
    return ononadjacent;
}
object PyKinBody::GetNonAdjacentLinks(int adjacentoptions) const
{
    py::list ononadjacent;
    const std::vector<int>& nonadjacent = _pbody->GetNonAdjacentLinks(adjacentoptions);
    FOREACHC(it,nonadjacent) {
        ononadjacent.append(py::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
    }
    return ononadjacent;
}

void PyKinBody::SetAdjacentLinks(int linkindex0, int linkindex1)
{
    _pbody->SetAdjacentLinks(linkindex0, linkindex1);
}

object PyKinBody::GetAdjacentLinks() const
{
    py::list adjacent;
    FOREACHC(it,_pbody->GetAdjacentLinks())
    adjacent.append(py::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
    return adjacent;
}

object PyKinBody::GetManageData() const
{
    KinBody::ManageDataPtr pdata = _pbody->GetManageData();
    return !pdata ? object() : object(PyManageDataPtr(new PyManageData(pdata,_pyenv)));
}
int PyKinBody::GetUpdateStamp() const
{
    return _pbody->GetUpdateStamp();
}

string PyKinBody::serialize(int options) const
{
    stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
    _pbody->serialize(ss,options);
    return ss.str();
}

string PyKinBody::GetKinematicsGeometryHash() const
{
    return _pbody->GetKinematicsGeometryHash();
}

PyStateRestoreContextBase* PyKinBody::CreateKinBodyStateSaver(object options)
{
    return CreateStateSaver(options);
}

PyStateRestoreContextBase* PyKinBody::CreateStateSaver(object options)
{
    PyKinBodyStateSaverPtr saver;
    if( IS_PYTHONOBJECT_NONE(options) ) {
        saver.reset(new PyKinBodyStateSaver(_pbody,_pyenv));
    }
    else {
        saver.reset(new PyKinBodyStateSaver(_pbody,_pyenv,options));
    }
    return new PyStateRestoreContext<PyKinBodyStateSaverPtr, PyKinBodyPtr>(saver);
}

string PyKinBody::__repr__()
{
    return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s')")%RaveGetEnvironmentId(_pbody->GetEnv())%_pbody->GetName());
}
string PyKinBody::__str__()
{
    return boost::str(boost::format("<%s:%s - %s (%s)>")%RaveGetInterfaceName(_pbody->GetInterfaceType())%_pbody->GetXMLId()%_pbody->GetName()%_pbody->GetKinematicsGeometryHash());
}

object PyKinBody::__unicode__()
{
    return ConvertStringToUnicode(__str__());
}

void PyKinBody::__enter__()
{
    // necessary to lock physics to prevent multiple threads from interfering
    if( _listStateSavers.size() == 0 ) {
        openravepy::LockEnvironment(_pyenv);
    }
    _listStateSavers.push_back(boost::shared_ptr<void>(new KinBody::KinBodyStateSaver(_pbody)));
}

void PyKinBody::__exit__(object type, object value, object traceback)
{
    BOOST_ASSERT(_listStateSavers.size()>0);
    _listStateSavers.pop_back();
    if( _listStateSavers.size() == 0 ) {
        openravepy::UnlockEnvironment(_pyenv);
    }
}

object toPyKinBodyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv)
{
    if( !plink ) {
        return object();
    }
    return object(PyLinkPtr(new PyLink(plink,pyenv)));
}

object toPyKinBodyLink(KinBody::LinkPtr plink, object opyenv)
{
    extract<PyEnvironmentBasePtr> pyenv(opyenv);
    if( pyenv.check() ) {
        return object(toPyKinBodyLink(plink,(PyEnvironmentBasePtr)pyenv));
    }
    return object();
}

object toPyKinBodyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv)
{
    if( !pjoint ) {
        return object();
    }
    return object(PyJointPtr(new PyJoint(pjoint,pyenv)));
}

KinBody::LinkPtr GetKinBodyLink(object o)
{
    extract<PyLinkPtr> pylink(o);
    if( pylink.check() ) {
        return ((PyLinkPtr)pylink)->GetLink();
    }
    return KinBody::LinkPtr();
}

KinBody::LinkConstPtr GetKinBodyLinkConst(object o)
{
    extract<PyLinkPtr> pylink(o);
    if( pylink.check() ) {
        return ((PyLinkPtr)pylink)->GetLink();
    }
    return KinBody::LinkConstPtr();
}

KinBody::JointPtr GetKinBodyJoint(object o)
{
    extract<PyJointPtr> pyjoint(o);
    if( pyjoint.check() ) {
        return ((PyJointPtr)pyjoint)->GetJoint();
    }
    return KinBody::JointPtr();
}

std::string reprPyKinBodyJoint(object o)
{
    extract<PyJointPtr> pyjoint(o);
    if( pyjoint.check() ) {
        return ((PyJointPtr)pyjoint)->__repr__();
    }
    return std::string();
}

std::string strPyKinBodyJoint(object o)
{
    extract<PyJointPtr> pyjoint(o);
    if( pyjoint.check() ) {
        return ((PyJointPtr)pyjoint)->__str__();
    }
    return std::string();
}

KinBodyPtr GetKinBody(object o)
{
    extract<PyKinBodyPtr> pykinbody(o);
    if( pykinbody.check() ) {
        return ((PyKinBodyPtr)pykinbody)->GetBody();
    }
    return KinBodyPtr();
}

KinBodyPtr GetKinBody(PyKinBodyPtr pykinbody)
{
    return !pykinbody ? KinBodyPtr() : pykinbody->GetBody();
}

PyEnvironmentBasePtr GetPyEnvFromPyKinBody(object o)
{
    extract<PyKinBodyPtr> pykinbody(o);
    if( pykinbody.check() ) {
        return ((PyKinBodyPtr)pykinbody)->GetEnv();
    }
    return PyEnvironmentBasePtr();
}

PyEnvironmentBasePtr toPyEnvironment(PyKinBodyPtr pykinbody)
{
    return pykinbody->GetEnv();
}

PyInterfaceBasePtr toPyKinBody(KinBodyPtr pkinbody, PyEnvironmentBasePtr pyenv)
{
    if( !pkinbody ) {
        return PyInterfaceBasePtr();
    }
    if( pkinbody->IsRobot() ) {
        return toPyRobot(RaveInterfaceCast<RobotBase>(pkinbody), pyenv);
    }
    return PyInterfaceBasePtr(new PyKinBody(pkinbody,pyenv));
}

object toPyKinBody(KinBodyPtr pkinbody, object opyenv)
{
    extract<PyEnvironmentBasePtr> pyenv(opyenv);
    if( pyenv.check() ) {
        return object(toPyKinBody(pkinbody,(PyEnvironmentBasePtr)pyenv));
    }
    return object();
}

PyKinBodyPtr RaveCreateKinBody(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    KinBodyPtr p = OpenRAVE::RaveCreateKinBody(openravepy::GetEnvironment(pyenv), name);
    if( !p ) {
        return PyKinBodyPtr();
    }
    return PyKinBodyPtr(new PyKinBody(p,pyenv));
}

class GeometryInfo_pickle_suite : public pickle_suite
{
public:
    static py::tuple getstate(const PyGeometryInfo& r)
    {
        return py::make_tuple(r._t, py::make_tuple(r._vGeomData, r._vGeomData2, r._vGeomData3, r._vGeomData4), r._vDiffuseColor, r._vAmbientColor, r._meshcollision, (int)r._type, py::make_tuple(r._name, r._filenamerender, r._filenamecollision), r._vRenderScale, r._vCollisionScale, r._fTransparency, r._bVisible, r._bModifiable, r._mapExtraGeometries);
    }
    static void setstate(PyGeometryInfo& r, py::tuple state) {
        //int num = len(state);
        r._t = state[0];
        r._vGeomData = state[1][0];
        r._vGeomData2 = state[1][1];
        r._vGeomData3 = state[1][2];
        if( py::len(state[1]) >= 4 ) { // for backward compatibility
            r._vGeomData4 = state[1][3];
        }
        r._vDiffuseColor = state[2];
        r._vAmbientColor = state[3];
        r._meshcollision = state[4];
        r._type = (GeometryType)(int)py::extract<int>(state[5]);

        py::extract<std::string> pyoldfilenamerender(state[6]);
        if( pyoldfilenamerender.check() ) {
            // old format
            r._filenamerender = state[6];
            r._filenamecollision = state[7];
            r._name = object();
            r._vRenderScale = state[8];
            r._vCollisionScale = state[9];
            r._fTransparency = py::extract<float>(state[10]);
            r._bVisible = py::extract<bool>(state[11]);
            r._bModifiable = py::extract<bool>(state[12]);
            r._mapExtraGeometries = dict(state[13]);
        }
        else {
            // new format
            r._name = state[6][0];
            r._filenamerender = state[6][1];
            r._filenamecollision = state[6][2];
            r._vRenderScale = state[7];
            r._vCollisionScale = state[8];
            r._fTransparency = py::extract<float>(state[9]);
            r._bVisible = py::extract<bool>(state[10]);
            r._bModifiable = py::extract<bool>(state[11]);
            r._mapExtraGeometries = dict(state[12]);
        }
    }
};

class LinkInfo_pickle_suite : public pickle_suite
{
public:
    static py::tuple getstate(const PyLinkInfo& r)
    {
        return py::make_tuple(r._vgeometryinfos, r._name, r._t, r._tMassFrame, r._mass, r._vinertiamoments, r._mapFloatParameters, r._mapIntParameters, r._vForcedAdjacentLinks, r._bStatic, r._bIsEnabled, r._mapStringParameters);
    }
    static void setstate(PyLinkInfo& r, py::tuple state) {
        int num = len(state);
        r._vgeometryinfos = py::list(state[0]);
        r._name = state[1];
        r._t = state[2];
        r._tMassFrame = state[3];
        r._mass = py::extract<dReal>(state[4]);
        r._vinertiamoments = state[5];
        r._mapFloatParameters = dict(state[6]);
        r._mapIntParameters = dict(state[7]);
        r._vForcedAdjacentLinks = dict(state[8]);
        r._bStatic = py::extract<bool>(state[9]);
        r._bIsEnabled = py::extract<bool>(state[10]);
        if( num > 11 ) {
            r._mapStringParameters = dict(state[11]);
        }
    }
};

class ElectricMotorActuatorInfo_pickle_suite : public pickle_suite
{
public:
    static py::tuple getstate(const PyElectricMotorActuatorInfo& r)
    {
        return py::make_tuple(r.gear_ratio, r.assigned_power_rating, r.max_speed, r.no_load_speed, py::make_tuple(r.stall_torque, r.max_instantaneous_torque), py::make_tuple(r.nominal_speed_torque_points, r.max_speed_torque_points), r.nominal_torque, r.rotor_inertia, r.torque_constant, r.nominal_voltage, r.speed_constant, r.starting_current, r.terminal_resistance, py::make_tuple(r.coloumb_friction, r.viscous_friction));
    }
    static void setstate(PyElectricMotorActuatorInfo& r, py::tuple state) {
        r.gear_ratio = py::extract<dReal>(state[0]);
        r.assigned_power_rating = py::extract<dReal>(state[1]);
        r.max_speed = py::extract<dReal>(state[2]);
        r.no_load_speed = py::extract<dReal>(state[3]);
        r.stall_torque = py::extract<dReal>(state[4][0]);
        r.max_instantaneous_torque = py::extract<dReal>(state[4][1]);
        r.nominal_speed_torque_points = py::list(state[5][0]);
        r.max_speed_torque_points = py::list(state[5][1]);
        r.nominal_torque = py::extract<dReal>(state[6]);
        r.rotor_inertia = py::extract<dReal>(state[7]);
        r.torque_constant = py::extract<dReal>(state[8]);
        r.nominal_voltage = py::extract<dReal>(state[9]);
        r.speed_constant = py::extract<dReal>(state[10]);
        r.starting_current = py::extract<dReal>(state[11]);
        r.terminal_resistance = py::extract<dReal>(state[12]);
        r.coloumb_friction = py::extract<dReal>(state[13][0]);
        r.viscous_friction = py::extract<dReal>(state[13][1]);
    }
};

class JointControlInfo_RobotController_pickle_suite : public pickle_suite
{
public:
    static py::tuple getstate(const PyJointControlInfo_RobotController& r)
    {
        return py::make_tuple(r.robotId, r.robotControllerDOFIndex);
    }
    static void setstate(PyJointControlInfo_RobotController& r, py::tuple state) {
        r.robotId = py::extract<int>(state[0]);
        r.robotControllerDOFIndex = state[1];
    }
};

class JointControlInfo_IO_pickle_suite : public pickle_suite
{
public:
    static py::tuple getstate(const PyJointControlInfo_IO& r)
    {
        return py::make_tuple(r.deviceId, r.vMoveIONames, r.vUpperLimitIONames, r.vUpperLimitSensorIsOn, r.vLowerLimitIONames, r.vLowerLimitSensorIsOn);
    }
    static void setstate(PyJointControlInfo_IO& r, py::tuple state) {
        r.deviceId = py::extract<int>(state[0]);
        r.vMoveIONames = state[1];
        r.vUpperLimitIONames = state[2];
        r.vUpperLimitSensorIsOn = state[3];
        r.vLowerLimitIONames = state[4];
        r.vLowerLimitSensorIsOn = state[5];
    }
};

class JointControlInfo_ExternalDevice_pickle_suite : public pickle_suite
{
public:
    static py::tuple getstate(const PyJointControlInfo_ExternalDevice& r)
    {
        return py::make_tuple(r.externalDeviceId);
    }
    static void setstate(PyJointControlInfo_ExternalDevice& r, py::tuple state) {
        r.externalDeviceId = py::extract<int>(state[0]);
    }
};

class JointInfo_pickle_suite : public pickle_suite
{
public:
    static py::tuple getstate(const PyJointInfo& r)
    {
        return py::make_tuple(py::make_tuple((int)r._type, r._name, r._linkname0, r._linkname1, r._vanchor, r._vaxes, r._vcurrentvalues), py::make_tuple(r._vresolution, r._vmaxvel, r._vhardmaxvel, r._vmaxaccel, r._vmaxtorque, r._vweights, r._voffsets, r._vlowerlimit, r._vupperlimit), py::make_tuple(r._trajfollow, r._vmimic, r._mapFloatParameters, r._mapIntParameters, r._bIsCircular, r._bIsActive, r._mapStringParameters, r._infoElectricMotor, r._vmaxinertia, r._vmaxjerk, r._vhardmaxaccel, r._vhardmaxjerk), py::make_tuple((int)r._controlMode, r._jci_robotcontroller, r._jci_io, r._jci_externaldevice));
    }
    static void setstate(PyJointInfo& r, py::tuple state) {
        r._type = (KinBody::JointType)(int)py::extract<int>(state[0][0]);
        r._name = state[0][1];
        r._linkname0 = state[0][2];
        r._linkname1 = state[0][3];
        r._vanchor = state[0][4];
        r._vaxes = state[0][5];
        r._vcurrentvalues = state[0][6];
        r._vresolution = state[1][0];
        r._vmaxvel = state[1][1];
        r._vhardmaxvel = state[1][2];
        r._vmaxaccel = state[1][3];
        r._vmaxtorque = state[1][4];
        r._vweights = state[1][5];
        r._voffsets = state[1][6];
        r._vlowerlimit = state[1][7];
        r._vupperlimit = state[1][8];
        r._trajfollow = state[2][0];
        int num2 = len(state[2]);
        r._vmimic = py::list(state[2][1]);
        r._mapFloatParameters = py::dict(state[2][2]);
        r._mapIntParameters = py::dict(state[2][3]);
        r._bIsCircular = state[2][4];
        r._bIsActive = py::extract<bool>(state[2][5]);
        if( num2 > 6 ) {
            r._mapStringParameters = py::dict(state[2][6]);
            if( num2 > 7 ) {
                r._infoElectricMotor = py::extract<PyElectricMotorActuatorInfoPtr>(state[2][7]);
                if( num2 > 8 ) {
                    r._vmaxinertia = state[2][8];
                    if( num2 > 9 ) {
                        r._vmaxjerk = state[2][9];
                        if( num2 > 10 ) {
                            r._vhardmaxaccel = state[2][10];
                            if( num2 > 11 ) {
                                r._vhardmaxjerk = state[2][11];
                            }
                        }
                    }
                }
            }
        }
        if( len(state) > 3 ) {
            r._controlMode = (KinBody::JointControlMode)(int)py::extract<int>(state[3][0]);
            r._jci_robotcontroller = py::extract<PyJointControlInfo_RobotControllerPtr>(state[3][1]);
            r._jci_io = py::extract<PyJointControlInfo_IOPtr>(state[3][2]);
            r._jci_externaldevice = py::extract<PyJointControlInfo_ExternalDevicePtr>(state[3][3]);
        }
    }
};

class GrabbedInfo_pickle_suite : public pickle_suite
{
public:
    static py::tuple getstate(const PyKinBody::PyGrabbedInfo& r)
    {
        return py::make_tuple(r._grabbedname, r._robotlinkname, r._trelative, r._setRobotLinksToIgnore);
    }
    static void setstate(PyKinBody::PyGrabbedInfo& r, py::tuple state) {
        r._grabbedname = state[0];
        r._robotlinkname = state[1];
        r._trelative = state[2];
        r._setRobotLinksToIgnore = state[3];
    }
};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(IsMimic_overloads, IsMimic, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMimicEquation_overloads, GetMimicEquation, 0, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMimicDOFIndices_overloads, GetMimicDOFIndices, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetChain_overloads, GetChain, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetConfigurationSpecification_overloads, GetConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetConfigurationSpecificationIndices_overloads, GetConfigurationSpecificationIndices, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetAxis_overloads, GetAxis, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetWrapOffset_overloads, GetWrapOffset, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetWrapOffset_overloads, SetWrapOffset, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMaxVel_overloads, GetMaxVel, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMaxAccel_overloads, GetMaxAccel, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMaxJerk_overloads, GetMaxJerk, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMaxTorque_overloads, GetMaxTorque, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetInstantaneousTorqueLimits_overloads, GetInstantaneousTorqueLimits, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetNominalTorqueLimits_overloads, GetNominalTorqueLimits, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMaxInertia_overloads, GetMaxInertia, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetLinkTransformations_overloads, GetLinkTransformations, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetLinkTransformations_overloads, SetLinkTransformations, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetDOFLimits_overloads, SetDOFLimits, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SubtractDOFValues_overloads, SubtractDOFValues, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeJacobianTranslation_overloads, ComputeJacobianTranslation, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeJacobianAxisAngle_overloads, ComputeJacobianAxisAngle, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeHessianTranslation_overloads, ComputeHessianTranslation, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeHessianAxisAngle_overloads, ComputeHessianAxisAngle, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeInverseDynamics_overloads, ComputeInverseDynamics, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Restore_overloads, Restore, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateKinBodyStateSaver_overloads, CreateKinBodyStateSaver, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetConfigurationValues_overloads, SetConfigurationValues, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetFloatParameters_overloads, GetFloatParameters, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIntParameters_overloads, GetIntParameters, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetStringParameters_overloads, GetStringParameters, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckSelfCollision_overloads, CheckSelfCollision, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetLinkAccelerations_overloads, GetLinkAccelerations, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(InitCollisionMesh_overloads, InitCollisionMesh, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(InitFromBoxes_overloads, InitFromBoxes, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(InitFromSpheres_overloads, InitFromSpheres, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(InitFromTrimesh_overloads, InitFromTrimesh, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(InitFromGeometries_overloads, InitFromGeometries, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Init_overloads, Init, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SerializeJSON_overloads, SerializeJSON, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeAABB_overloads, ComputeAABB, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeAABBFromTransform_overloads, ComputeAABBFromTransform, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeLocalAABB_overloads, ComputeLocalAABB, 0, 1)

void init_openravepy_kinbody()
{
    class_<PyStateRestoreContextBase, boost::noncopyable>("StateRestoreContext",no_init)
    .def("__enter__",&PyStateRestoreContextBase::__enter__,"returns the object storing the state")
    .def("__exit__",&PyStateRestoreContextBase::__exit__,"restores the state held in the object")
    .def("GetBody",&PyStateRestoreContextBase::GetBody,DOXY_FN(KinBody::KinBodyStateSaver, GetBody))
    .def("Restore",&PyStateRestoreContextBase::Restore,Restore_overloads(args("body"), DOXY_FN(KinBody::KinBodyStateSaver, Restore)))
    .def("Release",&PyStateRestoreContextBase::Release,DOXY_FN(KinBody::KinBodyStateSaver, Release))
    .def("Close",&PyStateRestoreContextBase::Close,DOXY_FN(KinBody::KinBodyStateSaver, Close))
    .def("__str__",&PyStateRestoreContextBase::__str__)
    .def("__unicode__",&PyStateRestoreContextBase::__unicode__)
    ;
    object geometrytype = enum_<GeometryType>("GeometryType" DOXY_ENUM(GeometryType))
                          .value("None",GT_None)
                          .value("Box",GT_Box)
                          .value("Sphere",GT_Sphere)
                          .value("Cylinder",GT_Cylinder)
                          .value("Trimesh",GT_TriMesh)
                          .value("Container",GT_Container)
                          .value("Cage",GT_Cage)
    ;
    object sidewalltype = enum_<KinBody::GeometryInfo::SideWallType>("SideWallType" DOXY_ENUM(KinBody::GeometryInfo::SideWallType))
                          .value("SWT_NX",KinBody::GeometryInfo::SideWallType::SWT_NX)
                          .value("SWT_PX",KinBody::GeometryInfo::SideWallType::SWT_PX)
                          .value("SWT_NY",KinBody::GeometryInfo::SideWallType::SWT_NY)
                          .value("SWT_PY",KinBody::GeometryInfo::SideWallType::SWT_PY)
    ;
    object electricmotoractuatorinfo = class_<PyElectricMotorActuatorInfo, boost::shared_ptr<PyElectricMotorActuatorInfo> >("ElectricMotorActuatorInfo", DOXY_CLASS(KinBody::ElectricMotorActuatorInfo))
                                       .def_readwrite("model_type",&PyElectricMotorActuatorInfo::model_type)
                                       .def_readwrite("assigned_power_rating",&PyElectricMotorActuatorInfo::assigned_power_rating)
                                       .def_readwrite("max_speed",&PyElectricMotorActuatorInfo::max_speed)
                                       .def_readwrite("no_load_speed",&PyElectricMotorActuatorInfo::no_load_speed)
                                       .def_readwrite("stall_torque",&PyElectricMotorActuatorInfo::stall_torque)
                                       .def_readwrite("max_instantaneous_torque",&PyElectricMotorActuatorInfo::max_instantaneous_torque)
                                       .def_readwrite("nominal_speed_torque_points",&PyElectricMotorActuatorInfo::nominal_speed_torque_points)
                                       .def_readwrite("max_speed_torque_points",&PyElectricMotorActuatorInfo::max_speed_torque_points)
                                       .def_readwrite("nominal_torque",&PyElectricMotorActuatorInfo::nominal_torque)
                                       .def_readwrite("rotor_inertia",&PyElectricMotorActuatorInfo::rotor_inertia)
                                       .def_readwrite("torque_constant",&PyElectricMotorActuatorInfo::torque_constant)
                                       .def_readwrite("nominal_voltage",&PyElectricMotorActuatorInfo::nominal_voltage)
                                       .def_readwrite("speed_constant",&PyElectricMotorActuatorInfo::speed_constant)
                                       .def_readwrite("starting_current",&PyElectricMotorActuatorInfo::starting_current)
                                       .def_readwrite("terminal_resistance",&PyElectricMotorActuatorInfo::terminal_resistance)
                                       .def_readwrite("gear_ratio",&PyElectricMotorActuatorInfo::gear_ratio)
                                       .def_readwrite("coloumb_friction",&PyElectricMotorActuatorInfo::coloumb_friction)
                                       .def_readwrite("viscous_friction",&PyElectricMotorActuatorInfo::viscous_friction)
                                       .def_pickle(ElectricMotorActuatorInfo_pickle_suite())
                                       .def("SerializeJSON", &PyElectricMotorActuatorInfo::SerializeJSON, SerializeJSON_overloads(args("options"), DOXY_FN(ElectricMotorActuatorInfo, SerializeJSON)))
                                       .def("DeserializeJSON", &PyElectricMotorActuatorInfo::DeserializeJSON, args("obj", "penv"), DOXY_FN(ElectricMotorActuatorInfo, DeserializeJSON))
    ;

    object jointtype = enum_<KinBody::JointType>("JointType" DOXY_ENUM(JointType))
                       .value("None",KinBody::JointNone)
                       .value("Hinge",KinBody::JointHinge)
                       .value("Revolute",KinBody::JointRevolute)
                       .value("Slider",KinBody::JointSlider)
                       .value("Prismatic",KinBody::JointPrismatic)
                       .value("RR",KinBody::JointRR)
                       .value("RP",KinBody::JointRP)
                       .value("PR",KinBody::JointPR)
                       .value("PP",KinBody::JointPP)
                       .value("Universal",KinBody::JointUniversal)
                       .value("Hinge2",KinBody::JointHinge2)
                       .value("Spherical",KinBody::JointSpherical)
                       .value("Trajectory",KinBody::JointTrajectory)
    ;

    object jointcontrolmode = enum_<KinBody::JointControlMode>("JointControlMode" DOXY_ENUM(JointControlMode))
                              .value("JCM_None",KinBody::JCM_None)
                              .value("JCM_RobotController",KinBody::JCM_RobotController)
                              .value("JCM_IO",KinBody::JCM_IO)
                              .value("JCM_ExternalDevice",KinBody::JCM_ExternalDevice)
    ;

    object geometryinfo = class_<PyGeometryInfo, boost::shared_ptr<PyGeometryInfo> >("GeometryInfo", DOXY_CLASS(KinBody::GeometryInfo))
                          .def_readwrite("_t",&PyGeometryInfo::_t)
                          .def_readwrite("_vGeomData",&PyGeometryInfo::_vGeomData)
                          .def_readwrite("_vGeomData2",&PyGeometryInfo::_vGeomData2)
                          .def_readwrite("_vGeomData3",&PyGeometryInfo::_vGeomData3)
                          .def_readwrite("_vGeomData4",&PyGeometryInfo::_vGeomData4)
                          .def_readwrite("_vDiffuseColor",&PyGeometryInfo::_vDiffuseColor)
                          .def_readwrite("_vAmbientColor",&PyGeometryInfo::_vAmbientColor)
                          .def_readwrite("_meshcollision",&PyGeometryInfo::_meshcollision)
                          .def_readwrite("_type",&PyGeometryInfo::_type)
                          .def_readwrite("_name",&PyGeometryInfo::_name)
                          .def_readwrite("_filenamerender",&PyGeometryInfo::_filenamerender)
                          .def_readwrite("_filenamecollision",&PyGeometryInfo::_filenamecollision)
                          .def_readwrite("_vRenderScale",&PyGeometryInfo::_vRenderScale)
                          .def_readwrite("_vCollisionScale",&PyGeometryInfo::_vCollisionScale)
                          .def_readwrite("_fTransparency",&PyGeometryInfo::_fTransparency)
                          .def_readwrite("_bVisible",&PyGeometryInfo::_bVisible)
                          .def_readwrite("_bModifiable",&PyGeometryInfo::_bModifiable)
                          .def_readwrite("_mapExtraGeometries",&PyGeometryInfo::_mapExtraGeometries)
                          .def_readwrite("_vSideWalls", &PyGeometryInfo::_vSideWalls)
                          .def("ComputeInnerEmptyVolume",&PyGeometryInfo::ComputeInnerEmptyVolume, DOXY_FN(GeomeryInfo,ComputeInnerEmptyVolume))
                          .def("ComputeAABB",&PyGeometryInfo::ComputeAABB, args("transform"), DOXY_FN(GeomeryInfo,ComputeAABB))
                          .def("DeserializeJSON", &PyGeometryInfo::DeserializeJSON, args("obj", "unitScale"), DOXY_FN(GeometryInfo, DeserializeJSON))
                          .def("SerializeJSON", &PyGeometryInfo::SerializeJSON, SerializeJSON_overloads(args("unitScale", "options"), DOXY_FN(GeometryInfo, SerializeJSON)))
                          .def_pickle(GeometryInfo_pickle_suite())
    ;

    object sidewall = class_<PySideWall, boost::shared_ptr<PySideWall> >("SideWall", DOXY_CLASS(KinBody::GeometryInfo::SideWall))
                      .def_readwrite("transf",&PySideWall::transf)
                      .def_readwrite("vExtents",&PySideWall::vExtents)
                      .def_readwrite("type",&PySideWall::type)
    ;

    object linkinfo = class_<PyLinkInfo, boost::shared_ptr<PyLinkInfo> >("LinkInfo", DOXY_CLASS(KinBody::LinkInfo))
                      .def_readwrite("_vgeometryinfos",&PyLinkInfo::_vgeometryinfos)
                      .def_readwrite("_name",&PyLinkInfo::_name)
                      .def_readwrite("_t",&PyLinkInfo::_t)
                      .def_readwrite("_tMassFrame",&PyLinkInfo::_tMassFrame)
                      .def_readwrite("_mass",&PyLinkInfo::_mass)
                      .def_readwrite("_vinertiamoments",&PyLinkInfo::_vinertiamoments)
                      .def_readwrite("_mapFloatParameters",&PyLinkInfo::_mapFloatParameters)
                      .def_readwrite("_mapIntParameters",&PyLinkInfo::_mapIntParameters)
                      .def_readwrite("_mapStringParameters",&PyLinkInfo::_mapStringParameters)
                      .def_readwrite("_vForcedAdjacentLinks",&PyLinkInfo::_vForcedAdjacentLinks)
                      .def_readwrite("_bStatic",&PyLinkInfo::_bStatic)
                      .def_readwrite("_bIsEnabled",&PyLinkInfo::_bIsEnabled)
                      .def("DeserializeJSON", &PyLinkInfo::DeserializeJSON, args("obj", "unitScale"), DOXY_FN(LinkInfo, DeserializeJSON))
                      .def("SerializeJSON", &PyLinkInfo::SerializeJSON,SerializeJSON_overloads(args("unitScale", "options"), DOXY_FN(LinkInfo, SerializeJSON)))
                      .def_pickle(LinkInfo_pickle_suite())
    ;

    object jointcontrolinfo_robotcontroller =
        class_<PyJointControlInfo_RobotController, boost::shared_ptr<PyJointControlInfo_RobotController> >("JointControlInfo_RobotController", DOXY_CLASS(KinBody::JointInfo::JointControlInfo_RobotController))
        .def_readwrite("robotId", &PyJointControlInfo_RobotController::robotId)
        .def_readwrite("robotControllerDOFIndex", &PyJointControlInfo_RobotController::robotControllerDOFIndex)
        .def_pickle(JointControlInfo_RobotController_pickle_suite())
    ;

    object jointcontrolinfo_io =
        class_<PyJointControlInfo_IO, boost::shared_ptr<PyJointControlInfo_IO> >("JointControlInfo_IO", DOXY_CLASS(KinBody::JointInfo::JointControlInfo_IO))
        .def_readwrite("deviceId", &PyJointControlInfo_IO::deviceId)
        .def_readwrite("vMoveIONames", &PyJointControlInfo_IO::vMoveIONames)
        .def_readwrite("vUpperLimitIONames", &PyJointControlInfo_IO::vUpperLimitIONames)
        .def_readwrite("vUpperLimitSensorIsOn", &PyJointControlInfo_IO::vUpperLimitSensorIsOn)
        .def_readwrite("vLowerLimitIONames", &PyJointControlInfo_IO::vLowerLimitIONames)
        .def_readwrite("vLowerLimitSensorIsOn", &PyJointControlInfo_IO::vLowerLimitSensorIsOn)
        .def_pickle(JointControlInfo_IO_pickle_suite())
    ;

    object jointcontrolinfo_externaldevice =
        class_<PyJointControlInfo_ExternalDevice, boost::shared_ptr<PyJointControlInfo_ExternalDevice> >("JointControlInfo_ExternalDevice", DOXY_CLASS(KinBody::JointInfo::JointControlInfo_ExternalDevice))
        .def_readwrite("externalDeviceId", &PyJointControlInfo_ExternalDevice::externalDeviceId)
        .def_pickle(JointControlInfo_ExternalDevice_pickle_suite())
    ;

    object jointinfo = class_<PyJointInfo, boost::shared_ptr<PyJointInfo> >("JointInfo", DOXY_CLASS(KinBody::JointInfo))
                       .def_readwrite("_type",&PyJointInfo::_type)
                       .def_readwrite("_name",&PyJointInfo::_name)
                       .def_readwrite("_linkname0",&PyJointInfo::_linkname0)
                       .def_readwrite("_linkname1",&PyJointInfo::_linkname1)
                       .def_readwrite("_vanchor",&PyJointInfo::_vanchor)
                       .def_readwrite("_vaxes",&PyJointInfo::_vaxes)
                       .def_readwrite("_vcurrentvalues",&PyJointInfo::_vcurrentvalues)
                       .def_readwrite("_vresolution",&PyJointInfo::_vresolution)
                       .def_readwrite("_vmaxvel",&PyJointInfo::_vmaxvel)
                       .def_readwrite("_vhardmaxvel",&PyJointInfo::_vhardmaxvel)
                       .def_readwrite("_vmaxaccel",&PyJointInfo::_vmaxaccel)
                       .def_readwrite("_vhardmaxaccel",&PyJointInfo::_vhardmaxaccel)
                       .def_readwrite("_vmaxjerk",&PyJointInfo::_vmaxjerk)
                       .def_readwrite("_vhardmaxjerk",&PyJointInfo::_vhardmaxjerk)
                       .def_readwrite("_vmaxtorque",&PyJointInfo::_vmaxtorque)
                       .def_readwrite("_vmaxinertia",&PyJointInfo::_vmaxinertia)
                       .def_readwrite("_vweights",&PyJointInfo::_vweights)
                       .def_readwrite("_voffsets",&PyJointInfo::_voffsets)
                       .def_readwrite("_vlowerlimit",&PyJointInfo::_vlowerlimit)
                       .def_readwrite("_vupperlimit",&PyJointInfo::_vupperlimit)
                       .def_readwrite("_trajfollow",&PyJointInfo::_trajfollow)
                       .def_readwrite("_vmimic",&PyJointInfo::_vmimic)
                       .def_readwrite("_mapFloatParameters",&PyJointInfo::_mapFloatParameters)
                       .def_readwrite("_mapIntParameters",&PyJointInfo::_mapIntParameters)
                       .def_readwrite("_mapStringParameters",&PyJointInfo::_mapStringParameters)
                       .def_readwrite("_bIsCircular",&PyJointInfo::_bIsCircular)
                       .def_readwrite("_bIsActive",&PyJointInfo::_bIsActive)
                       .def_readwrite("_infoElectricMotor", &PyJointInfo::_infoElectricMotor)
                       .def_readwrite("_controlMode", &PyJointInfo::_controlMode)
                       .def_readwrite("_jci_robotcontroller", &PyJointInfo::_jci_robotcontroller)
                       .def_readwrite("_jci_io", &PyJointInfo::_jci_io)
                       .def_readwrite("_jci_externaldevice", &PyJointInfo::_jci_externaldevice)
                       .def("SerializeJSON", &PyJointInfo::SerializeJSON, SerializeJSON_overloads(args("options"), DOXY_FN(KinBody::JointInfo, SerializeJSON)))
                       .def("DeserializeJSON", &PyJointInfo::DeserializeJSON, args("obj", "penv", "unitScale"), DOXY_FN(KinBody::JointInfo, DeserializeJSON))
                       .def_pickle(JointInfo_pickle_suite())
    ;

    object grabbedinfo = class_<PyKinBody::PyGrabbedInfo, boost::shared_ptr<PyKinBody::PyGrabbedInfo> >("GrabbedInfo", DOXY_CLASS(KinBody::GrabbedInfo))
                         .def_readwrite("_grabbedname",&PyKinBody::PyGrabbedInfo::_grabbedname)
                         .def_readwrite("_robotlinkname",&PyKinBody::PyGrabbedInfo::_robotlinkname)
                         .def_readwrite("_trelative",&PyKinBody::PyGrabbedInfo::_trelative)
                         .def_readwrite("_setRobotLinksToIgnore",&PyKinBody::PyGrabbedInfo::_setRobotLinksToIgnore)
                         .def("SerializeJSON", &PyKinBody::PyGrabbedInfo::SerializeJSON, SerializeJSON_overloads(args("options"), DOXY_FN(KinBody::GrabbedInfo, SerializeJSON)))
                         .def("DeserializeJSON", &PyKinBody::PyGrabbedInfo::DeserializeJSON, args("obj", "penv"), DOXY_FN(KinBody::GrabbedInfo, DeserializeJSON))
                         .def("__str__",&PyKinBody::PyGrabbedInfo::__str__)
                         .def("__unicode__",&PyKinBody::PyGrabbedInfo::__unicode__)
                         .def_pickle(GrabbedInfo_pickle_suite())
    ;


    {
        void (PyKinBody::*psetdofvalues1)(object) = &PyKinBody::SetDOFValues;
        void (PyKinBody::*psetdofvalues2)(object,object) = &PyKinBody::SetDOFValues;
        void (PyKinBody::*psetdofvalues3)(object,object,uint32_t) = &PyKinBody::SetDOFValues;
        object (PyKinBody::*getdofvalues1)() const = &PyKinBody::GetDOFValues;
        object (PyKinBody::*getdofvalues2)(object) const = &PyKinBody::GetDOFValues;
        object (PyKinBody::*getdofvelocities1)() const = &PyKinBody::GetDOFVelocities;
        object (PyKinBody::*getdofvelocities2)(object) const = &PyKinBody::GetDOFVelocities;
        object (PyKinBody::*getdoflimits1)() const = &PyKinBody::GetDOFLimits;
        object (PyKinBody::*getdoflimits2)(object) const = &PyKinBody::GetDOFLimits;
        object (PyKinBody::*getdofweights1)() const = &PyKinBody::GetDOFWeights;
        object (PyKinBody::*getdofweights2)(object) const = &PyKinBody::GetDOFWeights;
        object (PyKinBody::*getdofresolutions1)() const = &PyKinBody::GetDOFResolutions;
        object (PyKinBody::*getdofresolutions2)(object) const = &PyKinBody::GetDOFResolutions;
        object (PyKinBody::*getdofvelocitylimits1)() const = &PyKinBody::GetDOFVelocityLimits;
        object (PyKinBody::*getdofvelocitylimits2)(object) const = &PyKinBody::GetDOFVelocityLimits;
        object (PyKinBody::*getdofaccelerationlimits1)() const = &PyKinBody::GetDOFAccelerationLimits;
        object (PyKinBody::*getdofaccelerationlimits2)(object) const = &PyKinBody::GetDOFAccelerationLimits;
        object (PyKinBody::*getdofjerklimits1)() const = &PyKinBody::GetDOFJerkLimits;
        object (PyKinBody::*getdofjerklimits2)(object) const = &PyKinBody::GetDOFJerkLimits;
        object (PyKinBody::*getdofhardvelocitylimits1)() const = &PyKinBody::GetDOFHardVelocityLimits;
        object (PyKinBody::*getdofhardvelocitylimits2)(object) const = &PyKinBody::GetDOFHardVelocityLimits;
        object (PyKinBody::*getdofhardaccelerationlimits1)() const = &PyKinBody::GetDOFHardAccelerationLimits;
        object (PyKinBody::*getdofhardaccelerationlimits2)(object) const = &PyKinBody::GetDOFHardAccelerationLimits;
        object (PyKinBody::*getdofhardjerklimits1)() const = &PyKinBody::GetDOFHardJerkLimits;
        object (PyKinBody::*getdofhardjerklimits2)(object) const = &PyKinBody::GetDOFHardJerkLimits;
        object (PyKinBody::*getdoftorquelimits1)() const = &PyKinBody::GetDOFTorqueLimits;
        object (PyKinBody::*getdoftorquelimits2)(object) const = &PyKinBody::GetDOFTorqueLimits;
        object (PyKinBody::*getlinks1)() const = &PyKinBody::GetLinks;
        object (PyKinBody::*getlinks2)(object) const = &PyKinBody::GetLinks;
        object (PyKinBody::*getjoints1)() const = &PyKinBody::GetJoints;
        object (PyKinBody::*getjoints2)(object) const = &PyKinBody::GetJoints;
        void (PyKinBody::*setdofvelocities1)(object) = &PyKinBody::SetDOFVelocities;
        void (PyKinBody::*setdofvelocities2)(object,object,object) = &PyKinBody::SetDOFVelocities;
        void (PyKinBody::*setdofvelocities3)(object,uint32_t,object) = &PyKinBody::SetDOFVelocities;
        void (PyKinBody::*setdofvelocities4)(object,object,object,uint32_t) = &PyKinBody::SetDOFVelocities;
        bool (PyKinBody::*pgrab2)(PyKinBodyPtr,object) = &PyKinBody::Grab;
        bool (PyKinBody::*pgrab4)(PyKinBodyPtr,object,object) = &PyKinBody::Grab;
        object (PyKinBody::*GetNonAdjacentLinks1)() const = &PyKinBody::GetNonAdjacentLinks;
        object (PyKinBody::*GetNonAdjacentLinks2)(int) const = &PyKinBody::GetNonAdjacentLinks;
        std::string sInitFromBoxesDoc = std::string(DOXY_FN(KinBody,InitFromBoxes "const std::vector< AABB; bool")) + std::string("\nboxes is a Nx6 array, first 3 columsn are position, last 3 are extents");
        std::string sGetChainDoc = std::string(DOXY_FN(KinBody,GetChain)) + std::string("If returnjoints is false will return a list of links, otherwise will return a list of links (default is true)");
        std::string sComputeInverseDynamicsDoc = std::string(":param returncomponents: If True will return three N-element arrays that represents the torque contributions to M, C, and G.\n\n:param externalforcetorque: A dictionary of link indices and a 6-element array of forces/torques in that order.\n\n") + std::string(DOXY_FN(KinBody, ComputeInverseDynamics));
        scope kinbody = class_<PyKinBody, boost::shared_ptr<PyKinBody>, bases<PyInterfaceBase> >("KinBody", DOXY_CLASS(KinBody), no_init)
                        .def("Destroy",&PyKinBody::Destroy, DOXY_FN(KinBody,Destroy))
                        .def("InitFromBoxes",&PyKinBody::InitFromBoxes,InitFromBoxes_overloads(args("boxes","draw","uri"), sInitFromBoxesDoc.c_str()))
                        .def("InitFromSpheres",&PyKinBody::InitFromSpheres,InitFromSpheres_overloads(args("spherex","draw","uri"), DOXY_FN(KinBody,InitFromSpheres)))
                        .def("InitFromTrimesh",&PyKinBody::InitFromTrimesh,InitFromTrimesh_overloads(args("trimesh","draw","uri"), DOXY_FN(KinBody,InitFromTrimesh)))
                        .def("InitFromGeometries",&PyKinBody::InitFromGeometries,InitFromGeometries_overloads(args("geometries", "uri"), DOXY_FN(KinBody,InitFromGeometries)))
                        .def("Init",&PyKinBody::Init,Init_overloads(args("linkinfos","jointinfos","uri"), DOXY_FN(KinBody,Init)))
                        .def("SetLinkGeometriesFromGroup",&PyKinBody::SetLinkGeometriesFromGroup, args("name"), DOXY_FN(KinBody,SetLinkGeometriesFromGroup))
                        .def("SetLinkGroupGeometries", &PyKinBody::SetLinkGroupGeometries, args("name", "linkgeometries"), DOXY_FN(KinBody, SetLinkGroupGeometries))
                        .def("SetName", &PyKinBody::SetName,args("name"),DOXY_FN(KinBody,SetName))
                        .def("GetName",&PyKinBody::GetName,DOXY_FN(KinBody,GetName))
                        .def("GetDOF",&PyKinBody::GetDOF,DOXY_FN(KinBody,GetDOF))
                        .def("GetDOFValues",getdofvalues1,DOXY_FN(KinBody,GetDOFValues))
                        .def("GetDOFValues",getdofvalues2,args("indices"),DOXY_FN(KinBody,GetDOFValues))
                        .def("GetDOFVelocities",getdofvelocities1, DOXY_FN(KinBody,GetDOFVelocities))
                        .def("GetDOFVelocities",getdofvelocities2, args("indices"), DOXY_FN(KinBody,GetDOFVelocities))
                        .def("GetDOFLimits",getdoflimits1, DOXY_FN(KinBody,GetDOFLimits))
                        .def("GetDOFLimits",getdoflimits2, args("indices"),DOXY_FN(KinBody,GetDOFLimits))
                        .def("GetDOFVelocityLimits",getdofvelocitylimits1, DOXY_FN(KinBody,GetDOFVelocityLimits))
                        .def("GetDOFVelocityLimits",getdofvelocitylimits2, args("indices"),DOXY_FN(KinBody,GetDOFVelocityLimits))
                        .def("GetDOFAccelerationLimits",getdofaccelerationlimits1, DOXY_FN(KinBody,GetDOFAccelerationLimits))
                        .def("GetDOFAccelerationLimits",getdofaccelerationlimits2, args("indices"),DOXY_FN(KinBody,GetDOFAccelerationLimits))
                        .def("GetDOFJerkLimits",getdofjerklimits1, DOXY_FN(KinBody,GetDOFJerkLimits1))
                        .def("GetDOFJerkLimits",getdofjerklimits2, args("indices"),DOXY_FN(KinBody,GetDOFJerkLimits2))
                        .def("GetDOFHardVelocityLimits",getdofhardvelocitylimits1, DOXY_FN(KinBody,GetDOFHardVelocityLimits1))
                        .def("GetDOFHardVelocityLimits",getdofhardvelocitylimits2, args("indices"),DOXY_FN(KinBody,GetDOFHardVelocityLimits2))
                        .def("GetDOFHardAccelerationLimits",getdofhardaccelerationlimits1, DOXY_FN(KinBody,GetDOFHardAccelerationLimits1))
                        .def("GetDOFHardAccelerationLimits",getdofhardaccelerationlimits2, args("indices"),DOXY_FN(KinBody,GetDOFHardAccelerationLimits2))
                        .def("GetDOFHardJerkLimits",getdofhardjerklimits1, DOXY_FN(KinBody,GetDOFHardJerkLimits1))
                        .def("GetDOFHardJerkLimits",getdofhardjerklimits2, args("indices"),DOXY_FN(KinBody,GetDOFHardJerkLimits2))
                        .def("GetDOFTorqueLimits",getdoftorquelimits1, DOXY_FN(KinBody,GetDOFTorqueLimits))
                        .def("GetDOFTorqueLimits",getdoftorquelimits2, args("indices"),DOXY_FN(KinBody,GetDOFTorqueLimits))
                        .def("GetDOFMaxVel",&PyKinBody::GetDOFMaxVel, DOXY_FN(KinBody,GetDOFMaxVel))
                        .def("GetDOFMaxTorque",&PyKinBody::GetDOFMaxTorque, DOXY_FN(KinBody,GetDOFMaxTorque))
                        .def("GetDOFMaxAccel",&PyKinBody::GetDOFMaxAccel, DOXY_FN(KinBody,GetDOFMaxAccel))
                        .def("GetDOFWeights",getdofweights1, DOXY_FN(KinBody,GetDOFWeights))
                        .def("GetDOFWeights",getdofweights2, DOXY_FN(KinBody,GetDOFWeights))
                        .def("SetDOFWeights",&PyKinBody::SetDOFWeights, args("weights"), DOXY_FN(KinBody,SetDOFWeights))
                        .def("SetDOFResolutions",&PyKinBody::SetDOFResolutions, args("resolutions"), DOXY_FN(KinBody,SetDOFResolutions))
                        .def("SetDOFLimits",&PyKinBody::SetDOFLimits, SetDOFLimits_overloads(args("lower","upper","indices"), DOXY_FN(KinBody,SetDOFLimits)))
                        .def("SetDOFVelocityLimits",&PyKinBody::SetDOFVelocityLimits, args("limits"), DOXY_FN(KinBody,SetDOFVelocityLimits))
                        .def("SetDOFAccelerationLimits",&PyKinBody::SetDOFAccelerationLimits, args("limits"), DOXY_FN(KinBody,SetDOFAccelerationLimits))
                        .def("SetDOFJerkLimits",&PyKinBody::SetDOFJerkLimits, args("limits"), DOXY_FN(KinBody,SetDOFJerkLimits))
                        .def("SetDOFHardVelocityLimits",&PyKinBody::SetDOFHardVelocityLimits, args("limits"), DOXY_FN(KinBody,SetDOFHardVelocityLimits))
                        .def("SetDOFHardAccelerationLimits",&PyKinBody::SetDOFHardAccelerationLimits, args("limits"), DOXY_FN(KinBody,SetDOFHardAccelerationLimits))
                        .def("SetDOFHardJerkLimits",&PyKinBody::SetDOFHardJerkLimits, args("limits"), DOXY_FN(KinBody,SetDOFHardJerkLimits))
                        .def("SetDOFTorqueLimits",&PyKinBody::SetDOFTorqueLimits, args("limits"), DOXY_FN(KinBody,SetDOFTorqueLimits))
                        .def("GetDOFResolutions",getdofresolutions1, DOXY_FN(KinBody,GetDOFResolutions))
                        .def("GetDOFResolutions",getdofresolutions2, DOXY_FN(KinBody,GetDOFResolutions))
                        .def("GetLinks",getlinks1, DOXY_FN(KinBody,GetLinks))
                        .def("GetLinks",getlinks2, args("indices"), DOXY_FN(KinBody,GetLinks))
                        .def("GetLink",&PyKinBody::GetLink,args("name"), DOXY_FN(KinBody,GetLink))
                        .def("GetJoints",getjoints1, DOXY_FN(KinBody,GetJoints))
                        .def("GetJoints",getjoints2, args("indices"), DOXY_FN(KinBody,GetJoints))
                        .def("GetPassiveJoints",&PyKinBody::GetPassiveJoints, DOXY_FN(KinBody,GetPassiveJoints))
                        .def("GetDependencyOrderedJoints",&PyKinBody::GetDependencyOrderedJoints, DOXY_FN(KinBody,GetDependencyOrderedJoints))
                        .def("GetClosedLoops",&PyKinBody::GetClosedLoops,DOXY_FN(KinBody,GetClosedLoops))
                        .def("GetRigidlyAttachedLinks",&PyKinBody::GetRigidlyAttachedLinks,args("linkindex"), DOXY_FN(KinBody,GetRigidlyAttachedLinks))
                        .def("GetChain",&PyKinBody::GetChain,GetChain_overloads(args("linkindex1","linkindex2","returnjoints"), sGetChainDoc.c_str()))
                        .def("IsDOFInChain",&PyKinBody::IsDOFInChain,args("linkindex1","linkindex2","dofindex"), DOXY_FN(KinBody,IsDOFInChain))
                        .def("GetJointIndex",&PyKinBody::GetJointIndex,args("name"), DOXY_FN(KinBody,GetJointIndex))
                        .def("GetJoint",&PyKinBody::GetJoint,args("name"), DOXY_FN(KinBody,GetJoint))
                        .def("GetJointFromDOFIndex",&PyKinBody::GetJointFromDOFIndex,args("dofindex"), DOXY_FN(KinBody,GetJointFromDOFIndex))
                        .def("GetTransform",&PyKinBody::GetTransform, DOXY_FN(KinBody,GetTransform))
                        .def("GetTransformPose",&PyKinBody::GetTransformPose, DOXY_FN(KinBody,GetTransform))
                        .def("GetLinkTransformations",&PyKinBody::GetLinkTransformations, GetLinkTransformations_overloads(args("returndoflastvlaues"), DOXY_FN(KinBody,GetLinkTransformations)))
                        .def("GetBodyTransformations",&PyKinBody::GetLinkTransformations, DOXY_FN(KinBody,GetLinkTransformations))
                        .def("SetLinkTransformations",&PyKinBody::SetLinkTransformations,SetLinkTransformations_overloads(args("transforms","doflastsetvalues"), DOXY_FN(KinBody,SetLinkTransformations)))
                        .def("SetBodyTransformations",&PyKinBody::SetLinkTransformations,args("transforms"), DOXY_FN(KinBody,SetLinkTransformations))
                        .def("SetLinkVelocities",&PyKinBody::SetLinkVelocities,args("velocities"), DOXY_FN(KinBody,SetLinkVelocities))
                        .def("SetVelocity",&PyKinBody::SetVelocity, args("linear","angular"), DOXY_FN(KinBody,SetVelocity "const Vector; const Vector"))
                        .def("SetDOFVelocities",setdofvelocities1, args("dofvelocities"), DOXY_FN(KinBody,SetDOFVelocities "const std::vector; uint32_t"))
                        .def("SetDOFVelocities",setdofvelocities2, args("dofvelocities","linear","angular"), DOXY_FN(KinBody,SetDOFVelocities "const std::vector; const Vector; const Vector; uint32_t"))
                        .def("SetDOFVelocities",setdofvelocities3, args("dofvelocities","checklimits","indices"), DOXY_FN(KinBody,SetDOFVelocities "const std::vector; uint32_t; const std::vector"))
                        .def("SetDOFVelocities",setdofvelocities4, args("dofvelocities","linear","angular","checklimits"), DOXY_FN(KinBody,SetDOFVelocities "const std::vector; const Vector; const Vector; uint32_t"))
                        .def("GetLinkVelocities",&PyKinBody::GetLinkVelocities, DOXY_FN(KinBody,GetLinkVelocities))
                        .def("GetLinkAccelerations",&PyKinBody::GetLinkAccelerations, GetLinkAccelerations_overloads(args("dofaccelerations", "externalaccelerations"), DOXY_FN(KinBody,GetLinkAccelerations)))
                        .def("GetLinkEnableStates",&PyKinBody::GetLinkEnableStates, DOXY_FN(KinBody,GetLinkEnableStates))
                        .def("SetLinkEnableStates",&PyKinBody::SetLinkEnableStates, DOXY_FN(KinBody,SetLinkEnableStates))
                        .def("ComputeAABB",&PyKinBody::ComputeAABB, ComputeAABB_overloads(args("enabledOnlyLinks"), DOXY_FN(KinBody,ComputeAABB)))
                        .def("ComputeAABBFromTransform",&PyKinBody::ComputeAABBFromTransform, ComputeAABBFromTransform_overloads(args("transform", "enabledOnlyLinks"), DOXY_FN(KinBody,ComputeAABBFromTransform)))
                        .def("ComputeLocalAABB",&PyKinBody::ComputeLocalAABB, ComputeLocalAABB_overloads(args("enabledOnlyLinks"), DOXY_FN(KinBody,ComputeLocalAABB)))
                        .def("GetCenterOfMass", &PyKinBody::GetCenterOfMass, DOXY_FN(KinBody,GetCenterOfMass))
                        .def("Enable",&PyKinBody::Enable,args("enable"), DOXY_FN(KinBody,Enable))
                        .def("IsEnabled",&PyKinBody::IsEnabled, DOXY_FN(KinBody,IsEnabled))
                        .def("SetVisible",&PyKinBody::SetVisible,args("visible"), DOXY_FN(KinBody,SetVisible))
                        .def("IsVisible",&PyKinBody::IsVisible, DOXY_FN(KinBody,IsVisible))
                        .def("IsDOFRevolute",&PyKinBody::IsDOFRevolute, args("dofindex"), DOXY_FN(KinBody,IsDOFRevolute))
                        .def("IsDOFPrismatic",&PyKinBody::IsDOFPrismatic, args("dofindex"), DOXY_FN(KinBody,IsDOFPrismatic))
                        .def("SetTransform",&PyKinBody::SetTransform,args("transform"), DOXY_FN(KinBody,SetTransform))
                        .def("SetJointValues",psetdofvalues1,args("values"), DOXY_FN(KinBody,SetDOFValues "const std::vector; uint32_t; const std::vector"))
                        .def("SetJointValues",psetdofvalues2,args("values","dofindices"), DOXY_FN(KinBody,SetDOFValues "const std::vector; uint32_t; const std::vector"))
                        .def("SetDOFValues",psetdofvalues1,args("values"), DOXY_FN(KinBody,SetDOFValues "const std::vector; uint32_t; const std::vector"))
                        .def("SetDOFValues",psetdofvalues2,args("values","dofindices"), DOXY_FN(KinBody,SetDOFValues "const std::vector; uint32_t; const std::vector"))
                        .def("SetDOFValues",psetdofvalues3,args("values","dofindices","checklimits"), DOXY_FN(KinBody,SetDOFValues "const std::vector; uint32_t; const std::vector"))
                        .def("SubtractDOFValues",&PyKinBody::SubtractDOFValues,SubtractDOFValues_overloads(args("values0","values1"), DOXY_FN(KinBody,SubtractDOFValues)))
                        .def("SetDOFTorques",&PyKinBody::SetDOFTorques,args("torques","add"), DOXY_FN(KinBody,SetDOFTorques))
                        .def("SetJointTorques",&PyKinBody::SetDOFTorques,args("torques","add"), DOXY_FN(KinBody,SetDOFTorques))
                        .def("SetTransformWithJointValues",&PyKinBody::SetTransformWithDOFValues,args("transform","values"), DOXY_FN(KinBody,SetDOFValues "const std::vector; const Transform; uint32_t"))
                        .def("SetTransformWithDOFValues",&PyKinBody::SetTransformWithDOFValues,args("transform","values"), DOXY_FN(KinBody,SetDOFValues "const std::vector; const Transform; uint32_t"))
                        .def("ComputeJacobianTranslation",&PyKinBody::ComputeJacobianTranslation,ComputeJacobianTranslation_overloads(args("linkindex","position","indices"), DOXY_FN(KinBody,ComputeJacobianTranslation)))
                        .def("ComputeJacobianAxisAngle",&PyKinBody::ComputeJacobianAxisAngle,ComputeJacobianAxisAngle_overloads(args("linkindex","indices"), DOXY_FN(KinBody,ComputeJacobianAxisAngle)))
                        .def("CalculateJacobian",&PyKinBody::CalculateJacobian,args("linkindex","position"), DOXY_FN(KinBody,CalculateJacobian "int; const Vector; std::vector"))
                        .def("CalculateRotationJacobian",&PyKinBody::CalculateRotationJacobian,args("linkindex","quat"), DOXY_FN(KinBody,CalculateRotationJacobian "int; const Vector; std::vector"))
                        .def("CalculateAngularVelocityJacobian",&PyKinBody::CalculateAngularVelocityJacobian,args("linkindex"), DOXY_FN(KinBody,CalculateAngularVelocityJacobian "int; std::vector"))
                        .def("Grab",pgrab2,args("body","grablink"), DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr"))
                        .def("Grab",pgrab4,args("body","grablink","linkstoignore"), DOXY_FN(KinBody,Grab "KinBodyPtr; LinkPtr; const std::set"))
                        .def("Release",&PyKinBody::Release,args("body"), DOXY_FN(KinBody,Release))
                        .def("ReleaseAllGrabbed",&PyKinBody::ReleaseAllGrabbed, DOXY_FN(KinBody,ReleaseAllGrabbed))
                        .def("ReleaseAllGrabbedWithLink",&PyKinBody::ReleaseAllGrabbedWithLink, args("grablink"), DOXY_FN(KinBody,ReleaseAllGrabbedWithLink))
                        .def("RegrabAll",&PyKinBody::RegrabAll, DOXY_FN(KinBody,RegrabAll))
                        .def("IsGrabbing",&PyKinBody::IsGrabbing,args("body"), DOXY_FN(KinBody,IsGrabbing))
                        .def("GetGrabbed",&PyKinBody::GetGrabbed, DOXY_FN(KinBody,GetGrabbed))
                        .def("GetGrabbedInfo",&PyKinBody::GetGrabbedInfo, DOXY_FN(KinBody,GetGrabbedInfo))
                        .def("ResetGrabbed",&PyKinBody::ResetGrabbed, args("grabbedinfos"), DOXY_FN(KinBody,ResetGrabbed))
                        .def("ComputeHessianTranslation",&PyKinBody::ComputeHessianTranslation,ComputeHessianTranslation_overloads(args("linkindex","position","indices"), DOXY_FN(KinBody,ComputeHessianTranslation)))
                        .def("ComputeHessianAxisAngle",&PyKinBody::ComputeHessianAxisAngle,ComputeHessianAxisAngle_overloads(args("linkindex","indices"), DOXY_FN(KinBody,ComputeHessianAxisAngle)))
                        .def("ComputeInverseDynamics",&PyKinBody::ComputeInverseDynamics, ComputeInverseDynamics_overloads(args("dofaccelerations","externalforcetorque","returncomponents"), sComputeInverseDynamicsDoc.c_str()))
                        .def("SetSelfCollisionChecker",&PyKinBody::SetSelfCollisionChecker,args("collisionchecker"), DOXY_FN(KinBody,SetSelfCollisionChecker))
                        .def("GetSelfCollisionChecker",&PyKinBody::GetSelfCollisionChecker,args("collisionchecker"), DOXY_FN(KinBody,GetSelfCollisionChecker))
                        .def("CheckSelfCollision",&PyKinBody::CheckSelfCollision, CheckSelfCollision_overloads(args("report","collisionchecker"), DOXY_FN(KinBody,CheckSelfCollision)))
                        .def("IsAttached",&PyKinBody::IsAttached,args("body"), DOXY_FN(KinBody,IsAttached))
                        .def("GetAttached",&PyKinBody::GetAttached, DOXY_FN(KinBody,GetAttached))
                        .def("SetZeroConfiguration",&PyKinBody::SetZeroConfiguration, DOXY_FN(KinBody,SetZeroConfiguration))
                        .def("SetNonCollidingConfiguration",&PyKinBody::SetNonCollidingConfiguration, DOXY_FN(KinBody,SetNonCollidingConfiguration))
                        .def("GetConfigurationSpecification",&PyKinBody::GetConfigurationSpecification, GetConfigurationSpecification_overloads(args("interpolation"), DOXY_FN(KinBody,GetConfigurationSpecification)))
                        .def("GetConfigurationSpecificationIndices",&PyKinBody::GetConfigurationSpecificationIndices, GetConfigurationSpecificationIndices_overloads(args("indices","interpolation"), DOXY_FN(KinBody,GetConfigurationSpecificationIndices)))
                        .def("SetConfigurationValues",&PyKinBody::SetConfigurationValues, SetConfigurationValues_overloads(args("values","checklimits"), DOXY_FN(KinBody,SetConfigurationValues)))
                        .def("GetConfigurationValues",&PyKinBody::GetConfigurationValues, DOXY_FN(KinBody,GetConfigurationValues))
                        .def("IsRobot",&PyKinBody::IsRobot, DOXY_FN(KinBody,IsRobot))
                        .def("GetEnvironmentId",&PyKinBody::GetEnvironmentId, DOXY_FN(KinBody,GetEnvironmentId))
                        .def("DoesAffect",&PyKinBody::DoesAffect,args("jointindex","linkindex"), DOXY_FN(KinBody,DoesAffect))
                        .def("DoesDOFAffectLink",&PyKinBody::DoesDOFAffectLink,args("dofindex","linkindex"), DOXY_FN(KinBody,DoesDOFAffectLink))
                        .def("GetURI",&PyKinBody::GetURI, DOXY_FN(InterfaceBase,GetURI))
                        .def("GetXMLFilename",&PyKinBody::GetURI, DOXY_FN(InterfaceBase,GetURI))
                        .def("GetNonAdjacentLinks",GetNonAdjacentLinks1, DOXY_FN(KinBody,GetNonAdjacentLinks))
                        .def("GetNonAdjacentLinks",GetNonAdjacentLinks2, args("adjacentoptions"), DOXY_FN(KinBody,GetNonAdjacentLinks))
                        .def("SetAdjacentLinks",&PyKinBody::SetAdjacentLinks, args("linkindex0", "linkindex1"), DOXY_FN(KinBody,SetAdjacentLinks))
                        .def("GetAdjacentLinks",&PyKinBody::GetAdjacentLinks, DOXY_FN(KinBody,GetAdjacentLinks))
                        .def("GetManageData",&PyKinBody::GetManageData, DOXY_FN(KinBody,GetManageData))
                        .def("GetUpdateStamp",&PyKinBody::GetUpdateStamp, DOXY_FN(KinBody,GetUpdateStamp))
                        .def("serialize",&PyKinBody::serialize,args("options"), DOXY_FN(KinBody,serialize))
                        .def("GetKinematicsGeometryHash",&PyKinBody::GetKinematicsGeometryHash, DOXY_FN(KinBody,GetKinematicsGeometryHash))
                        .def("CreateKinBodyStateSaver",&PyKinBody::CreateKinBodyStateSaver, CreateKinBodyStateSaver_overloads(args("options"), "Creates an object that can be entered using 'with' and returns a KinBodyStateSaver")[return_value_policy<manage_new_object>()])
                        .def("__enter__",&PyKinBody::__enter__)
                        .def("__exit__",&PyKinBody::__exit__)
                        .def("__repr__",&PyKinBody::__repr__)
                        .def("__str__",&PyKinBody::__str__)
                        .def("__unicode__",&PyKinBody::__unicode__)
        ;

        enum_<KinBody::SaveParameters>("SaveParameters" DOXY_ENUM(SaveParameters))
        .value("LinkTransformation",KinBody::Save_LinkTransformation)
        .value("LinkEnable",KinBody::Save_LinkEnable)
        .value("LinkVelocities", KinBody::Save_LinkVelocities)
        .value("JointMaxVelocityAndAcceleration",KinBody::Save_JointMaxVelocityAndAcceleration)
        .value("JointWeights", KinBody::Save_JointWeights)
        .value("JointLimits", KinBody::Save_JointLimits)
        .value("ActiveDOF",KinBody::Save_ActiveDOF)
        .value("ActiveManipulator",KinBody::Save_ActiveManipulator)
        .value("GrabbedBodies",KinBody::Save_GrabbedBodies)
        .value("ActiveManipulatorToolTransform",KinBody::Save_ActiveManipulatorToolTransform)
        ;
        enum_<KinBody::CheckLimitsAction>("CheckLimitsAction" DOXY_ENUM(CheckLimitsAction))
        .value("Nothing",KinBody::CLA_Nothing)
        .value("CheckLimits",KinBody::CLA_CheckLimits)
        .value("CheckLimitsSilent",KinBody::CLA_CheckLimitsSilent)
        .value("CheckLimitsThrow",KinBody::CLA_CheckLimitsThrow)
        ;
        enum_<KinBody::AdjacentOptions>("AdjacentOptions" DOXY_ENUM(AdjacentOptions))
        .value("Enabled",KinBody::AO_Enabled)
        .value("ActiveDOFs",KinBody::AO_ActiveDOFs)
        ;
        kinbody.attr("JointType") = jointtype;
        kinbody.attr("LinkInfo") = linkinfo;
        kinbody.attr("GeometryInfo") = geometryinfo;
        kinbody.attr("JointInfo") = jointinfo;
        kinbody.attr("GrabbedInfo") = grabbedinfo;
        {
            scope link = class_<PyLink, boost::shared_ptr<PyLink> >("Link", DOXY_CLASS(KinBody::Link), no_init)
                         .def("GetName",&PyLink::GetName, DOXY_FN(KinBody::Link,GetName))
                         .def("GetIndex",&PyLink::GetIndex, DOXY_FN(KinBody::Link,GetIndex))
                         .def("Enable",&PyLink::Enable,args("enable"), DOXY_FN(KinBody::Link,Enable))
                         .def("IsEnabled",&PyLink::IsEnabled, DOXY_FN(KinBody::Link,IsEnabled))
                         .def("IsStatic",&PyLink::IsStatic, DOXY_FN(KinBody::Link,IsStatic))
                         .def("SetVisible",&PyLink::SetVisible,args("visible"), DOXY_FN(KinBody::Link,SetVisible))
                         .def("IsVisible",&PyLink::IsVisible, DOXY_FN(KinBody::Link,IsVisible))
                         .def("GetParent",&PyLink::GetParent, DOXY_FN(KinBody::Link,GetParent))
                         .def("GetParentLinks",&PyLink::GetParentLinks, DOXY_FN(KinBody::Link,GetParentLinks))
                         .def("IsParentLink",&PyLink::IsParentLink, DOXY_FN(KinBody::Link,IsParentLink))
                         .def("GetCollisionData",&PyLink::GetCollisionData, DOXY_FN(KinBody::Link,GetCollisionData))
                         .def("ComputeAABB",&PyLink::ComputeAABB, DOXY_FN(KinBody::Link,ComputeAABB))
                         .def("ComputeAABBFromTransform",&PyLink::ComputeAABBFromTransform, args("transform"), DOXY_FN(KinBody::Link,ComputeAABB))
                         .def("ComputeLocalAABB",&PyLink::ComputeLocalAABB, DOXY_FN(KinBody::Link,ComputeLocalAABB))
                         .def("GetTransform",&PyLink::GetTransform, DOXY_FN(KinBody::Link,GetTransform))
                         .def("GetTransformPose",&PyLink::GetTransformPose, DOXY_FN(KinBody::Link,GetTransform))
                         .def("GetCOMOffset",&PyLink::GetCOMOffset, DOXY_FN(KinBody::Link,GetCOMOffset))
                         .def("GetLocalCOM",&PyLink::GetLocalCOM, DOXY_FN(KinBody::Link,GetLocalCOM))
                         .def("GetGlobalCOM",&PyLink::GetGlobalCOM, DOXY_FN(KinBody::Link,GetGlobalCOM))
                         .def("GetLocalInertia",&PyLink::GetLocalInertia, DOXY_FN(KinBody::Link,GetLocalInertia))
                         .def("GetGlobalInertia",&PyLink::GetGlobalInertia, DOXY_FN(KinBody::Link,GetGlobalInertia))
                         .def("GetPrincipalMomentsOfInertia",&PyLink::GetPrincipalMomentsOfInertia, DOXY_FN(KinBody::Link,GetPrincipalMomentsOfInertia))
                         .def("GetLocalMassFrame",&PyLink::GetLocalMassFrame, DOXY_FN(KinBody::Link,GetLocalMassFrame))
                         .def("GetGlobalMassFrame",&PyLink::GetGlobalMassFrame, DOXY_FN(KinBody::Link,GetGlobalMassFrame))
                         .def("GetMass",&PyLink::GetMass, DOXY_FN(KinBody::Link,GetMass))
                         .def("SetLocalMassFrame",&PyLink::SetLocalMassFrame, args("massframe"), DOXY_FN(KinBody::Link,SetLocalMassFrame))
                         .def("SetPrincipalMomentsOfInertia",&PyLink::SetPrincipalMomentsOfInertia, args("inertiamoments"), DOXY_FN(KinBody::Link,SetPrincipalMomentsOfInertia))
                         .def("SetMass",&PyLink::SetMass, args("mass"), DOXY_FN(KinBody::Link,SetMass))
                         .def("SetStatic",&PyLink::SetStatic,args("static"), DOXY_FN(KinBody::Link,SetStatic))
                         .def("SetTransform",&PyLink::SetTransform,args("transform"), DOXY_FN(KinBody::Link,SetTransform))
                         .def("SetForce",&PyLink::SetForce,args("force","pos","add"), DOXY_FN(KinBody::Link,SetForce))
                         .def("SetTorque",&PyLink::SetTorque,args("torque","add"), DOXY_FN(KinBody::Link,SetTorque))
                         .def("GetGeometries",&PyLink::GetGeometries, DOXY_FN(KinBody::Link,GetGeometries))
                         .def("InitGeometries",&PyLink::InitGeometries, args("geometries"), DOXY_FN(KinBody::Link,InitGeometries))
                         .def("AddGeometry", &PyLink::AddGeometry, args("geometryinfo", "addToGroups"), DOXY_FN(KinBody::Link,AddGeometry))
                         .def("RemoveGeometryByName", &PyLink::RemoveGeometryByName, args("geometryname", "removeFromAllGroups"), DOXY_FN(KinBody::Link,RemoveGeometryByName))
                         .def("SetGeometriesFromGroup",&PyLink::SetGeometriesFromGroup, args("name"), DOXY_FN(KinBody::Link,SetGeometriesFromGroup))
                         .def("GetGeometriesFromGroup",&PyLink::GetGeometriesFromGroup, args("name"), DOXY_FN(KinBody::Link,GetGeometriesFromGroup))
                         .def("SetGroupGeometries",&PyLink::SetGroupGeometries, args("geometries"), DOXY_FN(KinBody::Link,SetGroupGeometries))
                         .def("GetGroupNumGeometries",&PyLink::GetGroupNumGeometries, args("geometries"), DOXY_FN(KinBody::Link,GetGroupNumGeometries))
                         .def("GetRigidlyAttachedLinks",&PyLink::GetRigidlyAttachedLinks, DOXY_FN(KinBody::Link,GetRigidlyAttachedLinks))
                         .def("IsRigidlyAttached",&PyLink::IsRigidlyAttached, DOXY_FN(KinBody::Link,IsRigidlyAttached))
                         .def("GetVelocity",&PyLink::GetVelocity,DOXY_FN(KinBody::Link,GetVelocity))
                         .def("SetVelocity",&PyLink::SetVelocity,DOXY_FN(KinBody::Link,SetVelocity))
                         .def("GetFloatParameters",&PyLink::GetFloatParameters,GetFloatParameters_overloads(args("name","index"), DOXY_FN(KinBody::Link,GetFloatParameters)))
                         .def("SetFloatParameters",&PyLink::SetFloatParameters,DOXY_FN(KinBody::Link,SetFloatParameters))
                         .def("GetIntParameters",&PyLink::GetIntParameters,GetIntParameters_overloads(args("name", "index"), DOXY_FN(KinBody::Link,GetIntParameters)))
                         .def("SetIntParameters",&PyLink::SetIntParameters,DOXY_FN(KinBody::Link,SetIntParameters))
                         .def("GetStringParameters",&PyLink::GetStringParameters,GetStringParameters_overloads(args("name"), DOXY_FN(KinBody::Link,GetStringParameters)))
                         .def("SetStringParameters",&PyLink::SetStringParameters,DOXY_FN(KinBody::Link,SetStringParameters))
                         .def("UpdateInfo",&PyLink::UpdateInfo,DOXY_FN(KinBody::Link,UpdateInfo))
                         .def("GetInfo",&PyLink::GetInfo,DOXY_FN(KinBody::Link,GetInfo))
                         .def("UpdateAndGetInfo",&PyLink::UpdateAndGetInfo,DOXY_FN(KinBody::Link,UpdateAndGetInfo))
                         .def("SetIntParameters",&PyLink::SetIntParameters,DOXY_FN(KinBody::Link,SetIntParameters))
                         .def("__repr__", &PyLink::__repr__)
                         .def("__str__", &PyLink::__str__)
                         .def("__unicode__", &PyLink::__unicode__)
                         .def("__eq__",&PyLink::__eq__)
                         .def("__ne__",&PyLink::__ne__)
                         .def("__hash__",&PyLink::__hash__)
            ;
            // \deprecated (12/10/18)
            link.attr("GeomType") = geometrytype;
            link.attr("GeometryInfo") = geometryinfo;
            {
                scope geometry = class_<PyLink::PyGeometry, boost::shared_ptr<PyLink::PyGeometry> >("Geometry", DOXY_CLASS(KinBody::Link::Geometry),no_init)
                                 .def("SetCollisionMesh",&PyLink::PyGeometry::SetCollisionMesh,args("trimesh"), DOXY_FN(KinBody::Link::Geometry,SetCollisionMesh))
                                 .def("GetCollisionMesh",&PyLink::PyGeometry::GetCollisionMesh, DOXY_FN(KinBody::Link::Geometry,GetCollisionMesh))
                                 .def("InitCollisionMesh",&PyLink::PyGeometry::InitCollisionMesh, InitCollisionMesh_overloads(args("tesselation"), DOXY_FN(KinBody::Link::Geometry,GetCollisionMesh)))
                                 .def("ComputeAABB",&PyLink::PyGeometry::ComputeAABB, args("transform"), DOXY_FN(KinBody::Link::Geometry,ComputeAABB))
                                 .def("GetSideWallExists",&PyLink::PyGeometry::GetSideWallExists, DOXY_FN(KinBody::Link::Geometry,GetSideWallExists))
                                 .def("SetDraw",&PyLink::PyGeometry::SetDraw,args("draw"), DOXY_FN(KinBody::Link::Geometry,SetDraw))
                                 .def("SetTransparency",&PyLink::PyGeometry::SetTransparency,args("transparency"), DOXY_FN(KinBody::Link::Geometry,SetTransparency))
                                 .def("SetDiffuseColor",&PyLink::PyGeometry::SetDiffuseColor,args("color"), DOXY_FN(KinBody::Link::Geometry,SetDiffuseColor))
                                 .def("SetAmbientColor",&PyLink::PyGeometry::SetAmbientColor,args("color"), DOXY_FN(KinBody::Link::Geometry,SetAmbientColor))
                                 .def("SetRenderFilename",&PyLink::PyGeometry::SetRenderFilename,args("color"), DOXY_FN(KinBody::Link::Geometry,SetRenderFilename))
                                 .def("SetName",&PyLink::PyGeometry::SetName,args("name"), DOXY_FN(KinBody::Link::Geometry,setName))
                                 .def("SetVisible",&PyLink::PyGeometry::SetVisible,args("visible"), DOXY_FN(KinBody::Link::Geometry,SetVisible))
                                 .def("IsDraw",&PyLink::PyGeometry::IsDraw, DOXY_FN(KinBody::Link::Geometry,IsDraw))
                                 .def("IsVisible",&PyLink::PyGeometry::IsVisible, DOXY_FN(KinBody::Link::Geometry,IsVisible))
                                 .def("IsModifiable",&PyLink::PyGeometry::IsModifiable, DOXY_FN(KinBody::Link::Geometry,IsModifiable))
                                 .def("GetType",&PyLink::PyGeometry::GetType, DOXY_FN(KinBody::Link::Geometry,GetType))
                                 .def("GetTransform",&PyLink::PyGeometry::GetTransform, DOXY_FN(KinBody::Link::Geometry,GetTransform))
                                 .def("GetTransformPose",&PyLink::PyGeometry::GetTransformPose, DOXY_FN(KinBody::Link::Geometry,GetTransform))
                                 .def("GetSphereRadius",&PyLink::PyGeometry::GetSphereRadius, DOXY_FN(KinBody::Link::Geometry,GetSphereRadius))
                                 .def("GetCylinderRadius",&PyLink::PyGeometry::GetCylinderRadius, DOXY_FN(KinBody::Link::Geometry,GetCylinderRadius))
                                 .def("GetCylinderHeight",&PyLink::PyGeometry::GetCylinderHeight, DOXY_FN(KinBody::Link::Geometry,GetCylinderHeight))
                                 .def("GetBoxExtents",&PyLink::PyGeometry::GetBoxExtents, DOXY_FN(KinBody::Link::Geometry,GetBoxExtents))
                                 .def("GetContainerOuterExtents",&PyLink::PyGeometry::GetContainerOuterExtents, DOXY_FN(KinBody::Link::Geometry,GetContainerOuterExtents))
                                 .def("GetContainerInnerExtents",&PyLink::PyGeometry::GetContainerInnerExtents, DOXY_FN(KinBody::Link::Geometry,GetContainerInnerExtents))
                                 .def("GetContainerBottomCross",&PyLink::PyGeometry::GetContainerBottomCross, DOXY_FN(KinBody::Link::Geometry,GetContainerBottomCross))
                                 .def("GetContainerBottom",&PyLink::PyGeometry::GetContainerBottom, DOXY_FN(KinBody::Link::Geometry,GetContainerBottom))
                                 .def("GetRenderScale",&PyLink::PyGeometry::GetRenderScale, DOXY_FN(KinBody::Link::Geometry,GetRenderScale))
                                 .def("GetRenderFilename",&PyLink::PyGeometry::GetRenderFilename, DOXY_FN(KinBody::Link::Geometry,GetRenderFilename))
                                 .def("GetName",&PyLink::PyGeometry::GetName, DOXY_FN(KinBody::Link::Geometry,GetName))
                                 .def("GetTransparency",&PyLink::PyGeometry::GetTransparency,DOXY_FN(KinBody::Link::Geometry,GetTransparency))
                                 .def("GetDiffuseColor",&PyLink::PyGeometry::GetDiffuseColor,DOXY_FN(KinBody::Link::Geometry,GetDiffuseColor))
                                 .def("GetAmbientColor",&PyLink::PyGeometry::GetAmbientColor,DOXY_FN(KinBody::Link::Geometry,GetAmbientColor))
                                 .def("ComputeInnerEmptyVolume",&PyLink::PyGeometry::ComputeInnerEmptyVolume,DOXY_FN(KinBody::Link::Geometry,ComputeInnerEmptyVolume))
                                 .def("GetInfo",&PyLink::PyGeometry::GetInfo,DOXY_FN(KinBody::Link::Geometry,GetInfo))
                                 .def("__eq__",&PyLink::PyGeometry::__eq__)
                                 .def("__ne__",&PyLink::PyGeometry::__ne__)
                                 .def("__hash__",&PyLink::PyGeometry::__hash__)
                ;
                // \deprecated (12/07/16)
                geometry.attr("Type") = geometrytype;
            }
            // \deprecated (12/07/16)
            link.attr("GeomProperties") = link.attr("Geometry");
        }
        {
            scope joint = class_<PyJoint, boost::shared_ptr<PyJoint> >("Joint", DOXY_CLASS(KinBody::Joint),no_init)
                          .def("GetName", &PyJoint::GetName, DOXY_FN(KinBody::Joint,GetName))
                          .def("IsMimic",&PyJoint::IsMimic,IsMimic_overloads(args("axis"), DOXY_FN(KinBody::Joint,IsMimic)))
                          .def("GetMimicEquation",&PyJoint::GetMimicEquation,GetMimicEquation_overloads(args("axis","type","format"), DOXY_FN(KinBody::Joint,GetMimicEquation)))
                          .def("GetMimicDOFIndices",&PyJoint::GetMimicDOFIndices,GetMimicDOFIndices_overloads(args("axis"), DOXY_FN(KinBody::Joint,GetMimicDOFIndices)))
                          .def("SetMimicEquations", &PyJoint::SetMimicEquations, args("axis","poseq","veleq","acceleq"), DOXY_FN(KinBody::Joint,SetMimicEquations))
                          .def("GetMaxVel", &PyJoint::GetMaxVel, GetMaxVel_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxVel)))
                          .def("GetMaxAccel", &PyJoint::GetMaxAccel, GetMaxAccel_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxAccel)))
                          .def("GetMaxJerk", &PyJoint::GetMaxJerk, GetMaxJerk_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxJerk)))
                          .def("GetMaxTorque", &PyJoint::GetMaxTorque, GetMaxTorque_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxTorque)))
                          .def("GetInstantaneousTorqueLimits", &PyJoint::GetInstantaneousTorqueLimits, GetInstantaneousTorqueLimits_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetInstantaneousTorqueLimits)))
                          .def("GetNominalTorqueLimits", &PyJoint::GetNominalTorqueLimits, GetNominalTorqueLimits_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetNominalTorqueLimits)))
                          .def("GetMaxInertia", &PyJoint::GetMaxInertia, GetMaxInertia_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxInertia)))
                          .def("GetDOFIndex", &PyJoint::GetDOFIndex, DOXY_FN(KinBody::Joint,GetDOFIndex))
                          .def("GetJointIndex", &PyJoint::GetJointIndex, DOXY_FN(KinBody::Joint,GetJointIndex))
                          .def("GetParent", &PyJoint::GetParent, DOXY_FN(KinBody::Joint,GetParent))
                          .def("GetFirstAttached", &PyJoint::GetFirstAttached, DOXY_FN(KinBody::Joint,GetFirstAttached))
                          .def("GetSecondAttached", &PyJoint::GetSecondAttached, DOXY_FN(KinBody::Joint,GetSecondAttached))
                          .def("IsStatic",&PyJoint::IsStatic, DOXY_FN(KinBody::Joint,IsStatic))
                          .def("IsCircular",&PyJoint::IsCircular, DOXY_FN(KinBody::Joint,IsCircular))
                          .def("IsRevolute",&PyJoint::IsRevolute, DOXY_FN(KinBody::Joint,IsRevolute))
                          .def("IsPrismatic",&PyJoint::IsPrismatic, DOXY_FN(KinBody::Joint,IsPrismatic))
                          .def("GetType", &PyJoint::GetType, DOXY_FN(KinBody::Joint,GetType))
                          .def("GetDOF", &PyJoint::GetDOF, DOXY_FN(KinBody::Joint,GetDOF))
                          .def("GetValues", &PyJoint::GetValues, DOXY_FN(KinBody::Joint,GetValues))
                          .def("GetValue", &PyJoint::GetValue, DOXY_FN(KinBody::Joint,GetValue))
                          .def("GetVelocities", &PyJoint::GetVelocities, DOXY_FN(KinBody::Joint,GetVelocities))
                          .def("GetAnchor", &PyJoint::GetAnchor, DOXY_FN(KinBody::Joint,GetAnchor))
                          .def("GetAxis", &PyJoint::GetAxis,GetAxis_overloads(args("axis"), DOXY_FN(KinBody::Joint,GetAxis)))
                          .def("GetHierarchyParentLink", &PyJoint::GetHierarchyParentLink, DOXY_FN(KinBody::Joint,GetHierarchyParentLink))
                          .def("GetHierarchyChildLink", &PyJoint::GetHierarchyChildLink, DOXY_FN(KinBody::Joint,GetHierarchyChildLink))
                          .def("GetInternalHierarchyAxis", &PyJoint::GetInternalHierarchyAxis,args("axis"), DOXY_FN(KinBody::Joint,GetInternalHierarchyAxis))
                          .def("GetInternalHierarchyLeftTransform",&PyJoint::GetInternalHierarchyLeftTransform, DOXY_FN(KinBody::Joint,GetInternalHierarchyLeftTransform))
                          .def("GetInternalHierarchyLeftTransformPose",&PyJoint::GetInternalHierarchyLeftTransformPose, DOXY_FN(KinBody::Joint,GetInternalHierarchyLeftTransform))
                          .def("GetInternalHierarchyRightTransform",&PyJoint::GetInternalHierarchyRightTransform, DOXY_FN(KinBody::Joint,GetInternalHierarchyRightTransform))
                          .def("GetInternalHierarchyRightTransformPose",&PyJoint::GetInternalHierarchyRightTransformPose, DOXY_FN(KinBody::Joint,GetInternalHierarchyRightTransform))
                          .def("GetLimits", &PyJoint::GetLimits, DOXY_FN(KinBody::Joint,GetLimits))
                          .def("GetVelocityLimits", &PyJoint::GetVelocityLimits, DOXY_FN(KinBody::Joint,GetVelocityLimits))
                          .def("GetAccelerationLimits", &PyJoint::GetAccelerationLimits, DOXY_FN(KinBody::Joint,GetAccelerationLimits))
                          .def("GetJerkLimits", &PyJoint::GetJerkLimits, DOXY_FN(KinBody::Joint,GetJerkLimits))
                          .def("GetHardVelocityLimits", &PyJoint::GetHardVelocityLimits, DOXY_FN(KinBody::Joint,GetHardVelocityLimits))
                          .def("GetHardAccelerationLimits", &PyJoint::GetHardAccelerationLimits, DOXY_FN(KinBody::Joint,GetHardAccelerationLimits))
                          .def("GetHardJerkLimits", &PyJoint::GetHardJerkLimits, DOXY_FN(KinBody::Joint,GetHardJerkLimits))
                          .def("GetTorqueLimits", &PyJoint::GetTorqueLimits, DOXY_FN(KinBody::Joint,GetTorqueLimits))
                          .def("SetWrapOffset",&PyJoint::SetWrapOffset,SetWrapOffset_overloads(args("offset","axis"), DOXY_FN(KinBody::Joint,SetWrapOffset)))
                          .def("GetWrapOffset",&PyJoint::GetWrapOffset,GetWrapOffset_overloads(args("axis"), DOXY_FN(KinBody::Joint,GetWrapOffset)))
                          .def("SetLimits",&PyJoint::SetLimits,args("lower","upper"), DOXY_FN(KinBody::Joint,SetLimits))
                          .def("SetVelocityLimits",&PyJoint::SetVelocityLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetVelocityLimits))
                          .def("SetAccelerationLimits",&PyJoint::SetAccelerationLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetAccelerationLimits))
                          .def("SetJerkLimits",&PyJoint::SetJerkLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetJerkLimits))
                          .def("SetHardVelocityLimits",&PyJoint::SetHardVelocityLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetHardVelocityLimits))
                          .def("SetHardAccelerationLimits",&PyJoint::SetHardAccelerationLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetHardAccelerationLimits))
                          .def("SetHardJerkLimits",&PyJoint::SetHardJerkLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetHardJerkLimits))
                          .def("SetTorqueLimits",&PyJoint::SetTorqueLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetTorqueLimits))
                          .def("GetResolution",&PyJoint::GetResolution,args("axis"), DOXY_FN(KinBody::Joint,GetResolution))
                          .def("GetResolutions",&PyJoint::GetResolutions,DOXY_FN(KinBody::Joint,GetResolutions))
                          .def("SetResolution",&PyJoint::SetResolution,args("resolution"), DOXY_FN(KinBody::Joint,SetResolution))
                          .def("GetWeight",&PyJoint::GetWeight,args("axis"), DOXY_FN(KinBody::Joint,GetWeight))
                          .def("GetWeights",&PyJoint::GetWeights,DOXY_FN(KinBody::Joint,GetWeights))
                          .def("SetWeights",&PyJoint::SetWeights,args("weights"), DOXY_FN(KinBody::Joint,SetWeights))
                          .def("SubtractValues",&PyJoint::SubtractValues,args("values0","values1"), DOXY_FN(KinBody::Joint,SubtractValues))
                          .def("SubtractValue",&PyJoint::SubtractValue,args("value0","value1","axis"), DOXY_FN(KinBody::Joint,SubtractValue))

                          .def("AddTorque",&PyJoint::AddTorque,args("torques"), DOXY_FN(KinBody::Joint,AddTorque))
                          .def("GetFloatParameters",&PyJoint::GetFloatParameters,GetFloatParameters_overloads(args("name", "index"), DOXY_FN(KinBody::Joint,GetFloatParameters)))
                          .def("SetFloatParameters",&PyJoint::SetFloatParameters,DOXY_FN(KinBody::Joint,SetFloatParameters))
                          .def("GetIntParameters",&PyJoint::GetIntParameters,GetIntParameters_overloads(args("name", "index"), DOXY_FN(KinBody::Joint,GetIntParameters)))
                          .def("SetIntParameters",&PyJoint::SetIntParameters,DOXY_FN(KinBody::Joint,SetIntParameters))
                          .def("GetStringParameters",&PyJoint::GetStringParameters,GetStringParameters_overloads(args("name", "index"), DOXY_FN(KinBody::Joint,GetStringParameters)))
                          .def("SetStringParameters",&PyJoint::SetStringParameters,DOXY_FN(KinBody::Joint,SetStringParameters))
                          .def("GetControlMode",&PyJoint::GetControlMode,DOXY_FN(KinBody::Joint,GetControlMode))
                          .def("UpdateInfo",&PyJoint::UpdateInfo,DOXY_FN(KinBody::Joint,UpdateInfo))
                          .def("GetInfo",&PyJoint::GetInfo,DOXY_FN(KinBody::Joint,GetInfo))
                          .def("UpdateAndGetInfo",&PyJoint::UpdateAndGetInfo,DOXY_FN(KinBody::Joint,UpdateAndGetInfo))
                          .def("__repr__", &PyJoint::__repr__)
                          .def("__str__", &PyJoint::__str__)
                          .def("__unicode__", &PyJoint::__unicode__)
                          .def("__eq__",&PyJoint::__eq__)
                          .def("__ne__",&PyJoint::__ne__)
                          .def("__hash__",&PyJoint::__hash__)
            ;
            joint.attr("Type") = jointtype;
        }

        {
            scope statesaver = class_<PyKinBodyStateSaver, boost::shared_ptr<PyKinBodyStateSaver> >("KinBodyStateSaver", DOXY_CLASS(KinBody::KinBodyStateSaver), no_init)
                               .def(init<PyKinBodyPtr>(args("body")))
                               .def(init<PyKinBodyPtr,object>(args("body","options")))
                               .def("GetBody",&PyKinBodyStateSaver::GetBody,DOXY_FN(KinBody::KinBodyStateSaver, GetBody))
                               .def("Restore",&PyKinBodyStateSaver::Restore,Restore_overloads(args("body"), DOXY_FN(KinBody::KinBodyStateSaver, Restore)))
                               .def("Release",&PyKinBodyStateSaver::Release,DOXY_FN(KinBody::KinBodyStateSaver, Release))
                               .def("__str__",&PyKinBodyStateSaver::__str__)
                               .def("__unicode__",&PyKinBodyStateSaver::__unicode__)
            ;
        }

        {
            scope managedata = class_<PyManageData, boost::shared_ptr<PyManageData> >("ManageData", DOXY_CLASS(KinBody::ManageData),no_init)
                               .def("GetSystem", &PyManageData::GetSystem, DOXY_FN(KinBody::ManageData,GetSystem))
                               .def("GetData", &PyManageData::GetData, DOXY_FN(KinBody::ManageData,GetData))
                               .def("GetOffsetLink", &PyManageData::GetOffsetLink, DOXY_FN(KinBody::ManageData,GetOffsetLink))
                               .def("IsPresent", &PyManageData::IsPresent, DOXY_FN(KinBody::ManageData,IsPresent))
                               .def("IsEnabled", &PyManageData::IsEnabled, DOXY_FN(KinBody::ManageData,IsEnabled))
                               .def("IsLocked", &PyManageData::IsLocked, DOXY_FN(KinBody::ManageData,IsLocked))
                               .def("Lock", &PyManageData::Lock,args("dolock"), DOXY_FN(KinBody::ManageData,Lock))
                               .def("__repr__", &PyManageData::__repr__)
                               .def("__str__", &PyManageData::__str__)
                               .def("__unicode__", &PyManageData::__unicode__)
                               .def("__eq__",&PyManageData::__eq__)
                               .def("__ne__",&PyManageData::__ne__)
            ;
        }
    }


    def("RaveCreateKinBody",openravepy::RaveCreateKinBody,args("env","name"),DOXY_FN1(RaveCreateKinBody));
}

}
