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
#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_kinbody.h>

namespace openravepy {
using py::object;

class PySideWall
{
public:
    PySideWall();
    PySideWall(const KinBody::GeometryInfo::SideWall& sidewall);
    void Get(KinBody::GeometryInfo::SideWall& sidewall);

    object transf = ReturnTransform(Transform());
    object vExtents = toPyVector3(Vector());
    int type = 0;
};


class PyGeometryInfo
{
public:
    PyGeometryInfo();
    //PyGeometryInfo(const PyGeometryInfo& pyinfo);
    PyGeometryInfo(const KinBody::GeometryInfo& info);
    void Init(const KinBody::GeometryInfo& info);

    object ComputeInnerEmptyVolume();
    object ComputeAABB(object otransform);

    object SerializeJSON(dReal fUnitScale=1.0, object options=py::none_());
    void DeserializeJSON(object obj, dReal fUnitScale=1.0);
    KinBody::GeometryInfoPtr GetGeometryInfo();

    object _t;
    object _vGeomData;
    object _vGeomData2;
    object _vGeomData3;
    object _vGeomData4;
    object _vDiffuseColor;
    object _vAmbientColor;
    object _meshcollision;
    py::list _vSideWalls;
    float _containerBaseHeight;
    GeometryType _type;
    // best to initialize these as None's
    object _name;
    object _filenamerender;
    object _filenamecollision;
    object _vRenderScale;
    object _vCollisionScale;
    float _fTransparency;
    bool _bVisible;
    bool _bModifiable;
};

typedef OPENRAVE_SHARED_PTR<PyGeometryInfo> PyGeometryInfoPtr;

class PyLinkInfo
{
public:
    PyLinkInfo();
    PyLinkInfo(const KinBody::LinkInfo& info);
    KinBody::LinkInfoPtr GetLinkInfo();

    object SerializeJSON(dReal fUnitScale=1.0, object options=py::none_());
    void DeserializeJSON(object obj, dReal fUnitScale=1.0);

    py::list _vgeometryinfos;
    object _name = py::none_();
    object _t = ReturnTransform(Transform());
    object _tMassFrame = ReturnTransform(Transform());
    dReal _mass = 0.0;
    object _vinertiamoments = toPyVector3(Vector(1,1,1));
    py::dict _mapFloatParameters, _mapIntParameters, _mapStringParameters, _mapExtraGeometries;
    object _vForcedAdjacentLinks = py::list();
    bool _bStatic = false;
    bool _bIsEnabled = true;

private:
    void _Update(const KinBody::LinkInfo& info);
};

class PyElectricMotorActuatorInfo
{
public:
    PyElectricMotorActuatorInfo();
    PyElectricMotorActuatorInfo(const ElectricMotorActuatorInfo& info);
    ElectricMotorActuatorInfoPtr GetElectricMotorActuatorInfo();
    object SerializeJSON(dReal fUnitScale=1.0, object options=py::none_());
    void DeserializeJSON(object obj, dReal fUnitScale=1.0);

    std::string model_type;
    dReal gear_ratio = 0.0;
    dReal assigned_power_rating = 0.0;
    dReal max_speed = 0.0;
    dReal no_load_speed = 0.0;
    dReal stall_torque = 0.0;
    dReal max_instantaneous_torque = 0.0;
    py::list nominal_speed_torque_points, max_speed_torque_points;
    dReal nominal_torque = 0.0;
    dReal rotor_inertia = 0.0;
    dReal torque_constant = 0.0;
    dReal nominal_voltage = 0.0;
    dReal speed_constant = 0.0;
    dReal starting_current = 0.0;
    dReal terminal_resistance = 0.0;
    dReal coloumb_friction = 0.0;
    dReal viscous_friction = 0.0;
private:
    void _Update(const ElectricMotorActuatorInfo& info);
};
typedef OPENRAVE_SHARED_PTR<PyElectricMotorActuatorInfo> PyElectricMotorActuatorInfoPtr;

class PyJointControlInfo_RobotController
{
public:
    PyJointControlInfo_RobotController();
    PyJointControlInfo_RobotController(const KinBody::JointInfo::JointControlInfo_RobotController& jci);
    KinBody::JointInfo::JointControlInfo_RobotControllerPtr GetJointControlInfo();

    int robotId = -1;
    object robotControllerDOFIndex = toPyVector3(Vector(-1, -1, -1));
};
typedef OPENRAVE_SHARED_PTR<PyJointControlInfo_RobotController> PyJointControlInfo_RobotControllerPtr;

class PyJointControlInfo_IO
{
public:
    PyJointControlInfo_IO();
    PyJointControlInfo_IO(const KinBody::JointInfo::JointControlInfo_IO& jci);
    KinBody::JointInfo::JointControlInfo_IOPtr GetJointControlInfo();

    int deviceId = -1;
    object vMoveIONames = py::list();
    object vUpperLimitIONames = py::list();
    object vUpperLimitSensorIsOn = py::list();
    object vLowerLimitIONames = py::list();
    object vLowerLimitSensorIsOn = py::list();
};
typedef OPENRAVE_SHARED_PTR<PyJointControlInfo_IO> PyJointControlInfo_IOPtr;

class PyJointControlInfo_ExternalDevice
{
public:
    PyJointControlInfo_ExternalDevice();
    PyJointControlInfo_ExternalDevice(const KinBody::JointInfo::JointControlInfo_ExternalDevice &jci);
    KinBody::JointInfo::JointControlInfo_ExternalDevicePtr GetJointControlInfo();
    std::string externalDeviceId;
};
typedef OPENRAVE_SHARED_PTR<PyJointControlInfo_ExternalDevice> PyJointControlInfo_ExternalDevicePtr;

class PyJointInfo
{
public:
    PyJointInfo();
    PyJointInfo(const KinBody::JointInfo& info);
    KinBody::JointInfoPtr GetJointInfo();
    object GetDOF();
    object SerializeJSON(dReal fUnitScale=1.0, object options=py::none_());
    void DeserializeJSON(object obj, dReal fUnitScale=1.0);

    KinBody::JointType _type = KinBody::JointNone;
    object _name = py::none_();
    object _linkname0 = py::none_(), _linkname1 = py::none_();
    object _vanchor = toPyVector3(Vector());
    object _vaxes = py::list();
    object _vcurrentvalues = py::none_();
    object _vresolution = toPyVector3(Vector(0.02,0.02,0.02));
    object _vmaxvel = toPyVector3(Vector(10,10,10));
    object _vhardmaxvel = toPyVector3(Vector(0,0,0));
    object _vmaxaccel = toPyVector3(Vector(50,50,50));
    object _vhardmaxaccel = toPyVector3(Vector(0,0,0));
    object _vmaxjerk = toPyVector3(Vector(2e6,2e6,2e6));
    object _vhardmaxjerk= toPyVector3(Vector(0, 0, 0));
    object _vmaxtorque = toPyVector3(Vector(1e5,1e5,1e5));
    object _vmaxinertia = toPyVector3(Vector(1e5,1e5,1e5));
    object _vweights = toPyVector3(Vector(1,1,1));
    object _voffsets = toPyVector3(Vector(0,0,0));
    object _vlowerlimit = toPyVector3(Vector(0,0,0));
    object _vupperlimit = toPyVector3(Vector(0,0,0));
    object _trajfollow = py::none_();
    PyElectricMotorActuatorInfoPtr _infoElectricMotor;
    py::list _vmimic;
    py::dict _mapFloatParameters, _mapIntParameters, _mapStringParameters;
    object _bIsCircular = py::list();
    bool _bIsActive = true;
    KinBody::JointControlMode _controlMode = KinBody::JointControlMode::JCM_None;
    PyJointControlInfo_RobotControllerPtr _jci_robotcontroller;
    PyJointControlInfo_IOPtr _jci_io;
    PyJointControlInfo_ExternalDevicePtr _jci_externaldevice;

private:
    void _Update(const KinBody::JointInfo& info);
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
        PyGeometry(KinBody::Link::GeometryPtr pgeometry);

        virtual void SetCollisionMesh(object pytrimesh);

        bool InitCollisionMesh(float fTessellation=1.0);
        uint8_t GetSideWallExists() const;

        object GetCollisionMesh();
        object ComputeAABB(object otransform) const;
        void SetDraw(bool bDraw);
        bool SetVisible(bool visible);
        void SetTransparency(float f);
        void SetAmbientColor(object ocolor);
        void SetDiffuseColor(object ocolor);
        void SetRenderFilename(const string& filename);
        void SetName(const std::string& name);
        bool IsDraw();
        bool IsVisible();
        bool IsModifiable();
        GeometryType GetType();
        object GetTransform();
        object GetTransformPose();
        dReal GetSphereRadius() const;
        dReal GetCylinderRadius() const ;
        dReal GetCylinderHeight() const;
        object GetBoxExtents() const;
        object GetContainerOuterExtents() const;
        object GetContainerInnerExtents() const;
        object GetContainerBottomCross() const;
        object GetContainerBottom() const;
        object GetRenderScale() const;
        object GetRenderFilename() const;
        object GetName() const;
        float GetTransparency() const;
        object GetDiffuseColor() const;
        object GetAmbientColor() const;
        object GetInfo();
        object ComputeInnerEmptyVolume() const;
        bool __eq__(OPENRAVE_SHARED_PTR<PyGeometry> p);
        bool __ne__(OPENRAVE_SHARED_PTR<PyGeometry> p);
        int __hash__();
    };

    PyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv);
    virtual ~PyLink();

    KinBody::LinkPtr GetLink();

    object GetName();
    int GetIndex();
    bool IsEnabled() const;
    bool SetVisible(bool visible);
    bool IsVisible() const;
    bool IsStatic() const;
    void Enable(bool bEnable);

    object GetParent() const;

    object GetParentLinks() const;

    bool IsParentLink(OPENRAVE_SHARED_PTR<PyLink> pylink) const;

    object GetCollisionData();
    object ComputeLocalAABB() const;

    object ComputeAABB() const;
    object ComputeAABBFromTransform(object otransform) const;

    object GetTransform() const;
    object GetTransformPose() const;

    object GetCOMOffset() const;
    object GetLocalCOM() const;
    object GetGlobalCOM() const;

    object GetLocalInertia() const;
    object GetGlobalInertia() const;
    dReal GetMass() const;
    object GetPrincipalMomentsOfInertia() const;
    object GetLocalMassFrame() const;
    object GetGlobalMassFrame() const;
    void SetLocalMassFrame(object omassframe);
    void SetPrincipalMomentsOfInertia(object oinertiamoments);
    void SetMass(dReal mass);

    void SetStatic(bool bStatic);
    void SetTransform(object otrans);
    void SetForce(object oforce, object opos, bool bAdd);
    void SetTorque(object otorque, bool bAdd);

    object GetGeometries();

    void InitGeometries(object ogeometryinfos);

    void AddGeometry(object ogeometryinfo, bool addToGroups);

    void RemoveGeometryByName(const std::string& geometryname, bool removeFromAllGroups);
    void SetGeometriesFromGroup(const std::string& name);

    object GetGeometriesFromGroup(const std::string& name);

    void SetGroupGeometries(const std::string& name, object ogeometryinfos);

    int GetGroupNumGeometries(const std::string& geomname);

    object GetRigidlyAttachedLinks() const;

    bool IsRigidlyAttached(OPENRAVE_SHARED_PTR<PyLink> plink);

    void SetVelocity(object olinear, object oangular);

    object GetVelocity() const;

    object GetFloatParameters(object oname=py::none_(), int index=-1) const;

    void SetFloatParameters(const std::string& key, object oparameters);

    object GetIntParameters(object oname=py::none_(), int index=-1) const;

    void SetIntParameters(const std::string& key, object oparameters);

    object GetStringParameters(object oname=py::none_()) const;

    void SetStringParameters(const std::string& key, object ovalue);

    void UpdateInfo();
    object GetInfo();
    object UpdateAndGetInfo();

    std::string __repr__();
    std::string __str__();
    object __unicode__();
    bool __eq__(OPENRAVE_SHARED_PTR<PyLink> p);
    bool __ne__(OPENRAVE_SHARED_PTR<PyLink> p);
    int __hash__();
};

class PyJoint
{
    KinBody::JointPtr _pjoint;
    PyEnvironmentBasePtr _pyenv;
public:
    PyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv);
    virtual ~PyJoint();

    KinBody::JointPtr GetJoint();

    object GetName();
    bool IsMimic(int iaxis=-1);
    string GetMimicEquation(int iaxis=0, int itype=0, const std::string& format="");
    object GetMimicDOFIndices(int iaxis=0);
    void SetMimicEquations(int iaxis, const std::string& poseq, const std::string& veleq, const std::string& acceleq);

    dReal GetMaxVel(int iaxis=0) const;
    dReal GetMaxAccel(int iaxis=0) const;
    dReal GetMaxJerk(int iaxis=0) const;
    dReal GetMaxTorque(int iaxis=0) const;
    object GetInstantaneousTorqueLimits(int iaxis=0) const;
    object GetNominalTorqueLimits(int iaxis=0) const;

    dReal GetMaxInertia(int iaxis=0) const;

    int GetDOFIndex() const;
    int GetJointIndex() const;

    PyKinBodyPtr GetParent() const;

    PyLinkPtr GetFirstAttached() const;
    PyLinkPtr GetSecondAttached() const;

    KinBody::JointType GetType() const;
    bool IsCircular(int iaxis) const;
    bool IsRevolute(int iaxis) const;
    bool IsPrismatic(int iaxis) const;
    bool IsStatic() const;

    int GetDOF() const;
    object GetValues() const;
    dReal GetValue(int iaxis) const;
    object GetVelocities() const;

    object GetAnchor() const;
    object GetAxis(int iaxis=0);
    PyLinkPtr GetHierarchyParentLink() const;
    PyLinkPtr GetHierarchyChildLink() const;
    object GetInternalHierarchyAxis(int iaxis);
    object GetInternalHierarchyLeftTransform();
    object GetInternalHierarchyLeftTransformPose();
    object GetInternalHierarchyRightTransform();
    object GetInternalHierarchyRightTransformPose();

    object GetLimits() const;
    object GetVelocityLimits() const;
    object GetAccelerationLimits() const;
    object GetJerkLimits() const;
    object GetHardVelocityLimits() const;
    object GetHardAccelerationLimits() const;
    object GetHardJerkLimits() const;
    object GetTorqueLimits() const;

    dReal GetWrapOffset(int iaxis=0);
    void SetWrapOffset(dReal offset, int iaxis=0);
    void SetLimits(object olower, object oupper);
    void SetVelocityLimits(object omaxlimits);
    void SetAccelerationLimits(object omaxlimits);
    void SetJerkLimits(object omaxlimits);
    void SetHardVelocityLimits(object omaxlimits);
    void SetHardAccelerationLimits(object omaxlimits);
    void SetHardJerkLimits(object omaxlimits);
    void SetTorqueLimits(object omaxlimits);

    object GetResolutions() const;
    dReal GetResolution(int iaxis);
    void SetResolution(dReal resolution);

    object GetWeights() const;
    dReal GetWeight(int iaxis);
    void SetWeights(object o);

    object SubtractValues(object ovalues0, object ovalues1);

    dReal SubtractValue(dReal value0, dReal value1, int iaxis);

    void AddTorque(object otorques);

    object GetFloatParameters(object oname=py::none_(), int index=-1) const;

    void SetFloatParameters(const std::string& key, object oparameters);

    object GetIntParameters(object oname=py::none_(), int index=-1) const;

    void SetIntParameters(const std::string& key, object oparameters);

    object GetStringParameters(object oname=py::none_()) const;

    void SetStringParameters(const std::string& key, object ovalue);

    KinBody::JointControlMode GetControlMode() const;
    void UpdateInfo();
    object GetInfo();
    object UpdateAndGetInfo();

    std::string __repr__();
    std::string __str__();
    object __unicode__();
    bool __eq__(OPENRAVE_SHARED_PTR<PyJoint> p);
    bool __ne__(OPENRAVE_SHARED_PTR<PyJoint> p);
    int __hash__();
};

class PyKinBodyStateSaver
{
    PyEnvironmentBasePtr _pyenv;
    KinBody::KinBodyStateSaver _state;
public:
    PyKinBodyStateSaver(PyKinBodyPtr pybody);
    PyKinBodyStateSaver(PyKinBodyPtr pybody, object options);
    PyKinBodyStateSaver(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv);
    PyKinBodyStateSaver(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv, object options);
    virtual ~PyKinBodyStateSaver();

    object GetBody() const;

    void Restore(PyKinBodyPtr pybody=PyKinBodyPtr());

    void Release();

    std::string __str__();
    object __unicode__();
};
typedef OPENRAVE_SHARED_PTR<PyKinBodyStateSaver> PyKinBodyStateSaverPtr;

class PyManageData
{
    KinBody::ManageDataPtr _pdata;
    PyEnvironmentBasePtr _pyenv;
public:
    PyManageData(KinBody::ManageDataPtr pdata, PyEnvironmentBasePtr pyenv);
    virtual ~PyManageData();

    KinBody::ManageDataPtr GetManageData();

    object GetSystem();

    PyVoidHandleConst GetData() const;
    PyLinkPtr GetOffsetLink() const;
    bool IsPresent();
    bool IsEnabled();
    bool IsLocked();
    bool Lock(bool bDoLock);

    string __repr__();
    string __str__();
    object __unicode__();
    bool __eq__(OPENRAVE_SHARED_PTR<PyManageData> p);
    bool __ne__(OPENRAVE_SHARED_PTR<PyManageData> p);
};
typedef OPENRAVE_SHARED_PTR<PyManageData> PyManageDataPtr;
typedef OPENRAVE_SHARED_PTR<PyManageData const> PyManageDataConstPtr;

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_JOINTINFO_H
