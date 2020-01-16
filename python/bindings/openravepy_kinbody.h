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
#ifndef OPENRAVEPY_INTERNAL_KINBODY_H
#define OPENRAVEPY_INTERNAL_KINBODY_H

#include "openravepy_int.h"
#include <openrave/utils.h>

namespace openravepy {

class PyStateRestoreContextBase
{
public:
    virtual ~PyStateRestoreContextBase() {
    }
    virtual py::object __enter__() = 0;
    virtual void __exit__(py::object type, py::object value, py::object traceback) = 0;
    virtual py::object GetBody() const = 0;
    virtual void Restore(py::object p=py::object())  = 0;
    virtual void Release() = 0;
    virtual void Close() = 0;
    virtual std::string __str__() = 0;
    virtual py::object __unicode__() = 0;
};

/// \brief simple wrapper around a save state that manages  enter/exit scope
template <typename T, typename U>
class PyStateRestoreContext : public PyStateRestoreContextBase
{
    T _state;
public:
    PyStateRestoreContext(T state) : _state(state) {
    }
    virtual ~PyStateRestoreContext() {
    }
    py::object __enter__() {
        return py::object(_state);
    }
    void __exit__(py::object type, py::object value, py::object traceback) {
        _state->Restore();
    }

    py::object GetBody() const {
        return _state->GetBody();
    }

    void Restore(py::object p=py::object()) {
        if( IS_PYTHONOBJECT_NONE(p) ) {
            _state->Restore();
        }
        else {
            U pytarget = py::extract<U>(p);
            _state->Restore(pytarget);
        }
    }

    void Release() {
        _state->Release();
    }

    void Close() {
        _state.reset();
    }

    std::string __str__() {
        return _state->__str__();
    }
    py::object __unicode__() {
        return _state->__unicode__();
    }
};

class PyKinBody : public PyInterfaceBase
{
 public:
    class PyGrabbedInfo
{
public:
    PyGrabbedInfo() {
        _trelative = ReturnTransform(Transform());
    }
    PyGrabbedInfo(const RobotBase::GrabbedInfo& info) {
        _Update(info);
    }

    RobotBase::GrabbedInfoPtr GetGrabbedInfo() const
    {
        RobotBase::GrabbedInfoPtr pinfo(new RobotBase::GrabbedInfo());
        pinfo->_grabbedname = py::extract<std::string>(_grabbedname);
        pinfo->_robotlinkname = py::extract<std::string>(_robotlinkname);
        pinfo->_trelative = ExtractTransform(_trelative);
        std::vector<int> v = ExtractArray<int>(_setRobotLinksToIgnore);
        pinfo->_setRobotLinksToIgnore.clear();
        FOREACHC(it,v) {
            pinfo->_setRobotLinksToIgnore.insert(*it);
        }
        return pinfo;
    }

    void DeserializeJSON(object obj, PyEnvironmentBasePtr penv)
    {
        rapidjson::Document doc;
        toRapidJSONValue(obj, doc, doc.GetAllocator());
        KinBody::GrabbedInfo info;
        info.DeserializeJSON(doc, GetEnvironment(penv));
        _Update(info);
    }

    object SerializeJSON(object ooptions=object())
    {
        rapidjson::Document doc;
        KinBody::GrabbedInfoPtr pInfo = GetGrabbedInfo();
        pInfo->SerializeJSON(doc, doc.GetAllocator(), pyGetIntFromPy(ooptions,0));
        return toPyObject(doc);
    }

    std::string __str__() {
        std::string robotlinkname = py::extract<std::string>(_robotlinkname);
        std::string grabbedname = py::extract<std::string>(_grabbedname);
        return boost::str(boost::format("<grabbedinfo:%s -> %s>")%robotlinkname%grabbedname);
    }
    py::object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    py::object _grabbedname, _robotlinkname;
    py::object _trelative;
    py::object _setRobotLinksToIgnore;


private:
    void _Update(const RobotBase::GrabbedInfo& info)
    {
        _grabbedname = ConvertStringToUnicode(info._grabbedname);
        _robotlinkname = ConvertStringToUnicode(info._robotlinkname);
        _trelative = ReturnTransform(info._trelative);
        py::list setRobotLinksToIgnore;
        FOREACHC(itindex, info._setRobotLinksToIgnore) {
            setRobotLinksToIgnore.append(*itindex);
        }
        _setRobotLinksToIgnore = setRobotLinksToIgnore;
    }
};
typedef boost::shared_ptr<PyGrabbedInfo> PyGrabbedInfoPtr;

protected:
    KinBodyPtr _pbody;
    std::list<boost::shared_ptr<void> > _listStateSavers;

public:
    PyKinBody(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv);
    PyKinBody(const PyKinBody& r);
    virtual ~PyKinBody();
    void Destroy();
    KinBodyPtr GetBody();
    bool InitFromBoxes(const boost::multi_array<dReal,2>& vboxes, bool bDraw=true, const std::string& uri=std::string());
    bool InitFromSpheres(const boost::multi_array<dReal,2>& vspheres, bool bDraw=true, const std::string& uri=std::string());
    bool InitFromTrimesh(py::object pytrimesh, bool bDraw=true, const std::string& uri=std::string());
    bool InitFromGeometries(py::object ogeometries, const std::string& uri=std::string());
    bool Init(py::object olinkinfos, py::object ojointinfos, const std::string& uri=std::string());
    void SetLinkGeometriesFromGroup(const std::string& geomname);
    void SetLinkGroupGeometries(const std::string& geomname, py::object olinkgeometryinfos);
    void SetName(const std::string& name);
    py::object GetName() const;
    int GetDOF() const;
    py::object GetDOFValues() const;
    py::object GetDOFValues(py::object oindices) const;
    py::object GetDOFVelocities() const;
    py::object GetDOFVelocities(py::object oindices) const;
    py::object GetDOFLimits() const;
    py::object GetDOFVelocityLimits() const;
    py::object GetDOFAccelerationLimits() const;
    py::object GetDOFJerkLimits() const;
    py::object GetDOFHardVelocityLimits() const;
    py::object GetDOFHardAccelerationLimits() const;
    py::object GetDOFHardJerkLimits() const;
    py::object GetDOFTorqueLimits() const;
    py::object GetDOFLimits(py::object oindices) const;
    py::object GetDOFVelocityLimits(py::object oindices) const;
    py::object GetDOFAccelerationLimits(py::object oindices) const;
    py::object GetDOFJerkLimits(py::object oindices) const;
    py::object GetDOFHardVelocityLimits(py::object oindices) const;
    py::object GetDOFHardAccelerationLimits(py::object oindices) const;
    py::object GetDOFHardJerkLimits(py::object oindices) const;
    py::object GetDOFTorqueLimits(py::object oindices) const;
    py::object GetDOFMaxVel() const;
    py::object GetDOFMaxTorque() const;
    py::object GetDOFMaxAccel() const;
    py::object GetDOFWeights() const;
    py::object GetDOFWeights(py::object oindices) const;
    py::object GetDOFResolutions() const;
    py::object GetDOFResolutions(py::object oindices) const;
    py::object GetLinks() const;
    py::object GetLinks(py::object oindices) const;
    py::object GetLink(const std::string& linkname) const;
    py::object GetJoints() const;
    py::object GetJoints(py::object oindices) const;
    py::object GetPassiveJoints();
    py::object GetDependencyOrderedJoints();
    py::object GetClosedLoops();
    py::object GetRigidlyAttachedLinks(int linkindex) const;
    py::object GetChain(int linkindex1, int linkindex2,bool returnjoints = true) const;
    bool IsDOFInChain(int linkindex1, int linkindex2, int dofindex) const;
    int GetJointIndex(const std::string& jointname) const;
    py::object GetJoint(const std::string& jointname) const;
    py::object GetJointFromDOFIndex(int dofindex) const;
    py::object GetTransform() const;
    py::object GetTransformPose() const;
    py::object GetLinkTransformations(bool returndoflastvlaues=false) const;
    void SetLinkTransformations(py::object transforms, py::object odoflastvalues=py::object());
    void SetLinkVelocities(py::object ovelocities);
    py::object GetLinkEnableStates() const;
    void SetLinkEnableStates(py::object oenablestates);
    bool SetVelocity(py::object olinearvel, py::object oangularvel);
    void SetDOFVelocities(py::object odofvelocities, py::object olinearvel, py::object oangularvel, uint32_t checklimits);
    void SetDOFVelocities(py::object odofvelocities, py::object olinearvel, py::object oangularvel);
    void SetDOFVelocities(py::object odofvelocities);
    void SetDOFVelocities(py::object odofvelocities, uint32_t checklimits=KinBody::CLA_CheckLimits, py::object oindices = py::object());
    py::object GetLinkVelocities() const;
    py::object GetLinkAccelerations(py::object odofaccelerations, py::object oexternalaccelerations) const;
    py::object ComputeAABB(bool bEnabledOnlyLinks=false);
    py::object ComputeAABBFromTransform(py::object otransform, bool bEnabledOnlyLinks=false);
    py::object ComputeLocalAABB(bool bEnabledOnlyLinks=false);
    py::object GetCenterOfMass() const;
    void Enable(bool bEnable);
    bool IsEnabled() const;
    bool SetVisible(bool visible);
    bool IsVisible() const;
    bool IsDOFRevolute(int dofindex) const;
    bool IsDOFPrismatic(int dofindex) const;
    void SetTransform(py::object otransform);
    void SetDOFWeights(py::object o);
    void SetDOFResolutions(py::object o);
    void SetDOFLimits(py::object olower, py::object oupper, py::object oindices=py::object());
    void SetDOFVelocityLimits(py::object o);
    void SetDOFAccelerationLimits(py::object o);
    void SetDOFJerkLimits(py::object o);
    void SetDOFHardVelocityLimits(py::object o);
    void SetDOFHardAccelerationLimits(py::object o);
    void SetDOFHardJerkLimits(py::object o);
    void SetDOFTorqueLimits(py::object o);
    void SetDOFValues(py::object o);
    void SetTransformWithDOFValues(py::object otrans,py::object ojoints);
    void SetDOFValues(py::object o, py::object indices, uint32_t checklimits);
    void SetDOFValues(py::object o, py::object indices);
    py::object SubtractDOFValues(py::object ovalues0, py::object ovalues1, py::object oindices=py::object());
    void SetDOFTorques(py::object otorques, bool bAdd);
    py::object ComputeJacobianTranslation(int index, py::object oposition, py::object oindices=py::object());
    py::object ComputeJacobianAxisAngle(int index, py::object oindices=py::object());
    py::object CalculateJacobian(int index, py::object oposition);
    py::object CalculateRotationJacobian(int index, py::object q) const;
    py::object CalculateAngularVelocityJacobian(int index) const;
    py::object ComputeHessianTranslation(int index, py::object oposition, py::object oindices=py::object());
    py::object ComputeHessianAxisAngle(int index, py::object oindices=py::object());
    py::object ComputeInverseDynamics(py::object odofaccelerations, py::object oexternalforcetorque=py::object(), bool returncomponents=false);
    void SetSelfCollisionChecker(PyCollisionCheckerBasePtr pycollisionchecker);
    PyInterfaceBasePtr GetSelfCollisionChecker();
    bool CheckSelfCollision(PyCollisionReportPtr pReport=PyCollisionReportPtr(), PyCollisionCheckerBasePtr pycollisionchecker=PyCollisionCheckerBasePtr());
    bool IsAttached(PyKinBodyPtr pattachbody);
    py::object GetAttached() const;
    void SetZeroConfiguration();
    void SetNonCollidingConfiguration();
    py::object GetConfigurationSpecification(const std::string& interpolation="") const;
    py::object GetConfigurationSpecificationIndices(py::object oindices,const std::string& interpolation="") const;
    void SetConfigurationValues(py::object ovalues, uint32_t checklimits=KinBody::CLA_CheckLimits);
    py::object GetConfigurationValues() const;
    bool Grab(PyKinBodyPtr pbody, py::object pylink_or_linkstoignore);
    bool Grab(PyKinBodyPtr pbody, py::object pylink, py::object linkstoignore);
    void Release(PyKinBodyPtr pbody);
    void ReleaseAllGrabbed();
    void ReleaseAllGrabbedWithLink(py::object pylink);
    void RegrabAll();
    py::object IsGrabbing(PyKinBodyPtr pbody) const;
    py::object GetGrabbed() const;
    py::object GetGrabbedInfo() const;
    void ResetGrabbed(py::object ograbbedinfos);
    bool IsRobot() const;
    int GetEnvironmentId() const;
    int DoesAffect(int jointindex, int linkindex ) const;
    int DoesDOFAffectLink(int dofindex, int linkindex ) const;
    py::object GetURI() const;
    py::object GetNonAdjacentLinks() const;
    py::object GetNonAdjacentLinks(int adjacentoptions) const;
    void SetAdjacentLinks(int linkindex0, int linkindex1);
    py::object GetAdjacentLinks() const;
    py::object GetManageData() const;
    int GetUpdateStamp() const;
    string serialize(int options) const;
    string GetKinematicsGeometryHash() const;
    PyStateRestoreContextBase* CreateKinBodyStateSaver(py::object options=py::object());
    virtual PyStateRestoreContextBase* CreateStateSaver(py::object options);
    virtual string __repr__();
    virtual string __str__();
    virtual py::object __unicode__();
    virtual void __enter__();
    virtual void __exit__(py::object type, py::object value, py::object traceback);

protected:
    /// \brief parse list of PyLinkInfoPtr into LinkInfoPtr
    void _ParseLinkInfos(py::object olinkinfos, std::vector<KinBody::LinkInfoConstPtr>& vlinkinfos);
    /// \brief parse list of PyJointInfoPtr into JointInfoPtr
    void _ParseJointInfos(py::object ojointinfos, std::vector<KinBody::JointInfoConstPtr>& vjointinfos);
};

}

#endif
