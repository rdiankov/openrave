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
#ifndef OPENRAVEPY_INTERNAL_JOINTINFO_H
#define OPENRAVEPY_INTERNAL_JOINTINFO_H

#define NO_IMPORT_ARRAY
#include "../openravepy_int.h"
#include "../openravepy_kinbody.h"

namespace openravepy {
using py::object;

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

    object SerializeJSON(const dReal fUnitScale=1.0, object ooptions=object())
    {
        rapidjson::Document doc;
        KinBody::GeometryInfoPtr pgeominfo = GetGeometryInfo();
        pgeominfo->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(ooptions,0));
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
        for (size_t i = 0; i < len(_vSideWalls); ++i) {
            info._vSideWalls.push_back({});
            OPENRAVE_SHARED_PTR<PySideWall> pysidewall = py::extract<OPENRAVE_SHARED_PTR<PySideWall> >(_vSideWalls[i]);
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
typedef OPENRAVE_SHARED_PTR<PyGeometryInfo> PyGeometryInfoPtr;

class PyLinkInfo
{
public:
    PyLinkInfo();
    PyLinkInfo(const KinBody::LinkInfo& info);

    KinBody::LinkInfoPtr GetLinkInfo();

    py::list _vgeometryinfos;
    object _name;
    object _t, _tMassFrame;
    dReal _mass;
    object _vinertiamoments;
    py::dict _mapFloatParameters, _mapIntParameters, _mapStringParameters;
    object _vForcedAdjacentLinks;
    bool _bStatic;
    bool _bIsEnabled;
};

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
};
typedef OPENRAVE_SHARED_PTR<PyElectricMotorActuatorInfo> PyElectricMotorActuatorInfoPtr;

class PyJointInfo
{
public:
    PyJointInfo();

    PyJointInfo(const KinBody::JointInfo& info, PyEnvironmentBasePtr pyenv);
    KinBody::JointInfoPtr GetJointInfo();
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
};

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
            return py::to_object(PyGeometryInfoPtr(new PyGeometryInfo(_pgeometry->GetInfo())));
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
        bool __eq__(OPENRAVE_SHARED_PTR<PyGeometry> p) {
            return !!p && _pgeometry == p->_pgeometry;
        }
        bool __ne__(OPENRAVE_SHARED_PTR<PyGeometry> p) {
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
            return py::to_object(toPyRobot(RaveInterfaceCast<RobotBase>(_plink->GetParent()),_pyenv));
        }
        else {
            return py::to_object(PyKinBodyPtr(new PyKinBody(_plink->GetParent(),_pyenv)));
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

    bool IsParentLink(OPENRAVE_SHARED_PTR<PyLink> pylink) const {
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
        return py::to_array(pyvalues);
    }
    object GetGlobalInertia() const {
        TransformMatrix t = _plink->GetGlobalInertia();
        npy_intp dims[] = { 3, 3};
        PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
        pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2];
        pdata[3] = t.m[4]; pdata[4] = t.m[5]; pdata[5] = t.m[6];
        pdata[6] = t.m[8]; pdata[7] = t.m[9]; pdata[8] = t.m[10];
        return py::to_array(pyvalues);
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
            geoms.append(OPENRAVE_SHARED_PTR<PyGeometry>(new PyGeometry(_plink->GetGeometry(i))));
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

    bool IsRigidlyAttached(OPENRAVE_SHARED_PTR<PyLink> plink) {
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

    object GetStringParameters(object oname=object()) const;

    void SetStringParameters(const std::string& key, object ovalue)
    {
        _plink->SetStringParameters(key, py::extract<std::string>(ovalue));
    }

    void UpdateInfo() {
        _plink->UpdateInfo();
    }
    object GetInfo() {
        return py::to_object(PyLinkInfoPtr(new PyLinkInfo(_plink->GetInfo())));
    }
    object UpdateAndGetInfo() {
        return py::to_object(PyLinkInfoPtr(new PyLinkInfo(_plink->UpdateAndGetInfo())));
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
    bool __eq__(OPENRAVE_SHARED_PTR<PyLink> p) {
        return !!p && _plink == p->_plink;
    }
    bool __ne__(OPENRAVE_SHARED_PTR<PyLink> p) {
        return !p || _plink != p->_plink;
    }
    int __hash__() {
        return static_cast<int>(uintptr_t(_plink.get()));
    }
};

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

    object GetStringParameters(object oname=object()) const;

    void SetStringParameters(const std::string& key, object ovalue)
    {
        _pjoint->SetStringParameters(key,py::extract<std::string>(ovalue));
    }

    void UpdateInfo() {
        _pjoint->UpdateInfo();
    }
    object GetInfo() {
        return py::to_object(PyJointInfoPtr(new PyJointInfo(_pjoint->GetInfo(), _pyenv)));
    }
    object UpdateAndGetInfo() {
        return py::to_object(PyJointInfoPtr(new PyJointInfo(_pjoint->UpdateAndGetInfo(), _pyenv)));
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
    bool __eq__(OPENRAVE_SHARED_PTR<PyJoint> p) {
        return !!p && _pjoint==p->_pjoint;
    }
    bool __ne__(OPENRAVE_SHARED_PTR<PyJoint> p) {
        return !p || _pjoint!=p->_pjoint;
    }
    int __hash__() {
        return static_cast<int>(uintptr_t(_pjoint.get()));
    }
};

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
            return py::object();
        }
        if( pbody->IsRobot() ) {
            return py::to_object(openravepy::toPyRobot(RaveInterfaceCast<RobotBase>(pbody),_pyenv));
        }
        else {
            return py::to_object(openravepy::toPyKinBody(pbody,_pyenv));
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
typedef OPENRAVE_SHARED_PTR<PyKinBodyStateSaver> PyKinBodyStateSaverPtr;

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
        return py::to_object(openravepy::toPySensorSystem(_pdata->GetSystem(),_pyenv));
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
    bool __eq__(OPENRAVE_SHARED_PTR<PyManageData> p) {
        return !!p && _pdata==p->_pdata;
    }
    bool __ne__(OPENRAVE_SHARED_PTR<PyManageData> p) {
        return !p || _pdata!=p->_pdata;
    }
};
typedef OPENRAVE_SHARED_PTR<PyManageData> PyManageDataPtr;
typedef OPENRAVE_SHARED_PTR<PyManageData const> PyManageDataConstPtr;

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_JOINTINFO_H
