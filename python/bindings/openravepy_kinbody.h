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
    virtual object __enter__() = 0;
    virtual void __exit__(object type, object value, object traceback) = 0;
    virtual object GetBody() const = 0;
    virtual void Restore(object p=object())  = 0;
    virtual void Release() = 0;
    virtual void Close() = 0;
    virtual std::string __str__() = 0;
    virtual object __unicode__() = 0;
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
    object __enter__() {
        return object(_state);
    }
    void __exit__(object type, object value, object traceback) {
        _state->Restore();
    }

    object GetBody() const {
        return _state->GetBody();
    }

    void Restore(object p=object()) {
        if( p == object() ) {
            _state->Restore();
        }
        else {
            U pytarget = boost::python::extract<U>(p);
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
    object __unicode__() {
        return _state->__unicode__();
    }
};

class PyKinBody : public PyInterfaceBase
{
protected:
    KinBodyPtr _pbody;
    std::list<boost::shared_ptr<void> > _listStateSavers;

public:
    PyKinBody(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv);
    PyKinBody(const PyKinBody& r);
    virtual ~PyKinBody();
    KinBodyPtr GetBody();
    bool InitFromBoxes(const boost::multi_array<dReal,2>& vboxes, bool bDraw);
    bool InitFromSpheres(const boost::multi_array<dReal,2>& vspheres, bool bDraw);
    bool InitFromTrimesh(object pytrimesh, bool bDraw);
    bool InitFromGeometries(object ogeometries);
    bool Init(object olinkinfos, object ojointinfos);
    void SetName(const std::string& name);
    object GetName() const;
    int GetDOF() const;
    object GetDOFValues() const;
    object GetDOFValues(object oindices) const;
    object GetDOFVelocities() const;
    object GetDOFVelocities(object oindices) const;
    object GetDOFLimits() const;
    object GetDOFVelocityLimits() const;
    object GetDOFAccelerationLimits() const;
    object GetDOFTorqueLimits() const;
    object GetDOFLimits(object oindices) const;
    object GetDOFVelocityLimits(object oindices) const;
    object GetDOFAccelerationLimits(object oindices) const;
    object GetDOFTorqueLimits(object oindices) const;
    object GetDOFMaxVel() const;
    object GetDOFMaxTorque() const;
    object GetDOFMaxAccel() const;
    object GetDOFWeights() const;
    object GetDOFWeights(object oindices) const;
    object GetDOFResolutions() const;
    object GetDOFResolutions(object oindices) const;
    object GetLinks() const;
    object GetLinks(object oindices) const;
    object GetLink(const std::string& linkname) const;
    object GetJoints() const;
    object GetJoints(object oindices) const;
    object GetPassiveJoints();
    object GetDependencyOrderedJoints();
    object GetClosedLoops();
    object GetRigidlyAttachedLinks(int linkindex) const;
    object GetChain(int linkindex1, int linkindex2,bool returnjoints = true) const;
    bool IsDOFInChain(int linkindex1, int linkindex2, int dofindex) const;
    int GetJointIndex(const std::string& jointname) const;
    object GetJoint(const std::string& jointname) const;
    object GetJointFromDOFIndex(int dofindex) const;
    object GetTransform() const;
    object GetLinkTransformations(bool returndofbranches=false) const;
    void SetLinkTransformations(object transforms, object odofbranches=object());
    void SetLinkVelocities(object ovelocities);
    bool SetVelocity(object olinearvel, object oangularvel);
    void SetDOFVelocities(object odofvelocities, object olinearvel, object oangularvel, uint32_t checklimits);
    void SetDOFVelocities(object odofvelocities, object olinearvel, object oangularvel);
    void SetDOFVelocities(object odofvelocities);
    void SetDOFVelocities(object odofvelocities, uint32_t checklimits=KinBody::CLA_CheckLimits, object oindices = object());
    object GetLinkVelocities() const;
    object GetLinkAccelerations(object odofaccelerations) const;
    object ComputeAABB();
    void Enable(bool bEnable);
    bool IsEnabled() const;
    bool SetVisible(bool visible);
    bool IsVisible() const;
    void SetTransform(object transform);
    void SetDOFWeights(object o);
    void SetDOFLimits(object olower, object oupper);
    void SetDOFVelocityLimits(object o);
    void SetDOFAccelerationLimits(object o);
    void SetDOFTorqueLimits(object o);
    void SetDOFValues(object o);
    void SetTransformWithDOFValues(object otrans,object ojoints);
    void SetDOFValues(object o, object indices, uint32_t checklimits);
    void SetDOFValues(object o, object indices);
    object SubtractDOFValues(object ovalues0, object ovalues1);
    void SetDOFTorques(object otorques, bool bAdd);
    object ComputeJacobianTranslation(int index, object oposition, object oindices=object());
    object ComputeJacobianAxisAngle(int index, object oindices=object());
    object CalculateJacobian(int index, object oposition);
    object CalculateRotationJacobian(int index, object q) const;
    object CalculateAngularVelocityJacobian(int index) const;
    object ComputeHessianTranslation(int index, object oposition, object oindices=object());
    object ComputeHessianAxisAngle(int index, object oindices=object());
    object ComputeInverseDynamics(object odofaccelerations, object oexternalforcetorque=object(), bool returncomponents=false);
    bool CheckSelfCollision();
    bool CheckSelfCollision(PyCollisionReportPtr pReport);
    bool IsAttached(PyKinBodyPtr pattachbody);
    object GetAttached() const;
    void SetZeroConfiguration();
    void SetNonCollidingConfiguration();
    object GetConfigurationSpecification(const std::string& interpolation="") const;
    object GetConfigurationSpecificationIndices(object oindices,const std::string& interpolation="") const;
    void SetConfigurationValues(object ovalues, uint32_t checklimits=KinBody::CLA_CheckLimits);
    object GetConfigurationValues() const;
    bool IsRobot() const;
    int GetEnvironmentId() const;
    int DoesAffect(int jointindex, int linkindex ) const;
    object GetViewerData() const;
    object GetURI() const;
    object GetNonAdjacentLinks() const;
    object GetNonAdjacentLinks(int adjacentoptions) const;
    object GetAdjacentLinks() const;
    object GetPhysicsData() const;
    object GetCollisionData() const;
    object GetManageData() const;
    int GetUpdateStamp() const;
    string serialize(int options) const;
    string GetKinematicsGeometryHash() const;
    PyStateRestoreContextBase* CreateKinBodyStateSaver(object options=object());
    virtual string __repr__();
    virtual string __str__();
    virtual object __unicode__();
    virtual void __enter__();
    virtual void __exit__(object type, object value, object traceback);

protected:
    /// \brief parse list of PyLinkInfoPtr into LinkInfoPtr
    void _ParseLinkInfos(object olinkinfos, std::vector<KinBody::LinkInfoConstPtr>& vlinkinfos);
    /// \brief parse list of PyJointInfoPtr into JointInfoPtr
    void _ParseJointInfos(object ojointinfos, std::vector<KinBody::JointInfoConstPtr>& vjointinfos);
};

}

#endif
