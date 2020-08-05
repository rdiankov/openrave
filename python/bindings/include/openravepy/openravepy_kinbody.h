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

#include <openravepy/openravepy_int.h>
#include <openrave/utils.h>

namespace openravepy {
class PyStateRestoreContextBase
{
public:
    PyStateRestoreContextBase() {}
    virtual ~PyStateRestoreContextBase() {
    }
    virtual py::object __enter__() = 0;
    virtual void __exit__(py::object type, py::object value, py::object traceback) = 0;
    virtual py::object GetBody() const = 0;
    virtual void Restore(py::object p=py::none_())  = 0;
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
        return py::to_object(_state);
    }
    void __exit__(py::object type, py::object value, py::object traceback) {
        _state->Restore();
    }

    py::object GetBody() const {
        return _state->GetBody();
    }

    void Restore(py::object p=py::none_()) {
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
    PyGrabbedInfo();
    PyGrabbedInfo(const RobotBase::GrabbedInfo& info);

    RobotBase::GrabbedInfoPtr GetGrabbedInfo() const;

    py::object SerializeJSON(dReal fUnitScale=1.0, py::object ooptions=py::none_());

    void DeserializeJSON(py::object obj, dReal fUnitScale=1.0, py::object options=py::none_());

    std::string __str__();
    py::object __unicode__();

private:
    void _Update(const RobotBase::GrabbedInfo& info);

public:

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    std::string _id;
    std::string _grabbedname;
    std::string _robotlinkname;
#else
    py::object _id = py::none_();
    py::object _grabbedname = py::none_();
    py::object _robotlinkname = py::none_();
#endif
    py::object _trelative = ReturnTransform(Transform());
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    std::vector<int> _setRobotLinksToIgnore;
#else
    py::object _setRobotLinksToIgnore = py::none_();
#endif
}; // class PyGrabbedInfo
typedef OPENRAVE_SHARED_PTR<PyGrabbedInfo> PyGrabbedInfoPtr;

public:
    class PyKinBodyInfo
{
public:
    PyKinBodyInfo();
    PyKinBodyInfo(const KinBody::KinBodyInfo& info);
    py::object SerializeJSON(dReal fUnitScale=1.0, py::object options=py::none_());
    void DeserializeJSON(py::object obj, dReal fUnitScale=1.0, py::object options=py::none_());
    KinBody::KinBodyInfoPtr GetKinBodyInfo() const;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    std::vector<KinBody::LinkInfoPtr> _vLinkInfos;
    std::vector<KinBody::JointInfoPtr> _vJointInfos;
    std::vector<KinBody::GrabbedInfoPtr> _vGrabbedInfos;
    std::string _uri;
    std::string _id;
    std::string _name;
    std::string _referenceUri;
#else
    py::object _vLinkInfos = py::none_();
    py::object _vJointInfos = py::none_();
    py::object _vGrabbedInfos = py::none_();
    py::object _uri = py::none_();
    py::object _referenceUri = py::none_();
    py::object _id = py::none_();
    py::object _name = py::none_();
#endif
    py::object _transform = ReturnTransform(Transform());
    bool _isRobot = false;
    py::object _dofValues = py::none_();
    py::object _readableInterfaces = py::none_();
    virtual std::string __str__();
    virtual py::object __unicode__();

protected:
    void _Update(const KinBody::KinBodyInfo& info);
}; // class PyKinBodyInfo


protected:
    KinBodyPtr _pbody;
    std::list<OPENRAVE_SHARED_PTR<void> > _listStateSavers;

public:
    PyKinBody(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv);
    PyKinBody(const PyKinBody& r);
    virtual ~PyKinBody();
    void Destroy();
    KinBodyPtr GetBody();

    bool InitFromKinBodyInfo(const py::object pyKinBodyInfo);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    bool InitFromBoxes(const std::vector<std::vector<dReal> >& vboxes, const bool bDraw = true, const std::string& uri = "");
    bool InitFromSpheres(const std::vector<std::vector<dReal> >& vspheres, const bool bDraw = true, const std::string& uri = "");
#else
    bool InitFromBoxes(const boost::multi_array<dReal,2>& vboxes, bool bDraw=true, const std::string& uri=std::string());
    bool InitFromSpheres(const boost::multi_array<dReal,2>& vspheres, bool bDraw=true, const std::string& uri=std::string());
#endif
    bool InitFromTrimesh(py::object pytrimesh, bool bDraw=true, const std::string& uri=std::string());
    bool InitFromGeometries(py::object ogeometries, const std::string& uri=std::string());
    void InitFromLinkInfos(py::object olinkinfos, const std::string& uri=std::string());
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
    void SetLinkTransformations(py::object transforms, py::object odoflastvalues=py::none_());
    void SetLinkVelocities(py::object ovelocities);
    py::object GetLinkEnableStates() const;
    void SetLinkEnableStates(py::object oenablestates);
    bool SetVelocity(py::object olinearvel, py::object oangularvel);
    void SetDOFVelocities(py::object odofvelocities, py::object olinearvel, py::object oangularvel, uint32_t checklimits);
    void SetDOFVelocities(py::object odofvelocities, py::object olinearvel, py::object oangularvel);
    void SetDOFVelocities(py::object odofvelocities);
    void SetDOFVelocities(py::object odofvelocities, uint32_t checklimits=KinBody::CLA_CheckLimits, py::object oindices = py::none_());
    py::object GetLinkVelocities() const;
    py::object GetLinkAccelerations(py::object odofaccelerations, py::object oexternalaccelerations=py::none_()) const;
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
    void SetDOFLimits(py::object olower, py::object oupper, py::object oindices=py::none_());
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
    py::object SubtractDOFValues(py::object ovalues0, py::object ovalues1, py::object oindices=py::none_());
    void SetDOFTorques(py::object otorques, bool bAdd);
    py::object ComputeJacobianTranslation(int index, py::object oposition, py::object oindices=py::none_());
    py::object ComputeJacobianAxisAngle(int index, py::object oindices=py::none_());
    py::object CalculateJacobian(int index, py::object oposition);
    py::object CalculateRotationJacobian(int index, py::object q) const;
    py::object CalculateAngularVelocityJacobian(int index) const;
    py::object ComputeHessianTranslation(int index, py::object oposition, py::object oindices=py::none_());
    py::object ComputeHessianAxisAngle(int index, py::object oindices=py::none_());
    py::object ComputeInverseDynamics(py::object odofaccelerations, py::object oexternalforcetorque=py::none_(), bool returncomponents=false);
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
    py::object GetGrabbedInfo(py::object ograbbedname=py::none_()) const;
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
    std::string serialize(int options) const;
    std::string GetKinematicsGeometryHash() const;
    PyStateRestoreContextBase* CreateKinBodyStateSaver(py::object options=py::none_());

    py::object ExtractInfo() const;

    virtual PyStateRestoreContextBase* CreateStateSaver(py::object options);
    virtual std::string __repr__();
    virtual std::string __str__();
    virtual py::object __unicode__();
    virtual void __enter__();
    virtual void __exit__(py::object type, py::object value, py::object traceback);

protected:
    /// \brief parse list of PyLinkInfoPtr into LinkInfoPtr
    void _ParseLinkInfos(py::object olinkinfos, std::vector<KinBody::LinkInfoConstPtr>& vlinkinfos);
    /// \brief parse list of PyJointInfoPtr into JointInfoPtr
    void _ParseJointInfos(py::object ojointinfos, std::vector<KinBody::JointInfoConstPtr>& vjointinfos);
};

template <typename T>
py::object GetCustomParameters(const std::map<std::string, std::vector<T> >& parameters, py::object oname = py::none_(), int index = -1);

} // namespace openravepy

#endif
