// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
/** \file physicsengine.h
    \brief Physics engine related definitions.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_PHYSICSENGINE_H
#define OPENRAVE_PHYSICSENGINE_H

namespace OpenRAVE {

/// basic options for physics engine
enum PhysicsEngineOptions
{
    PEO_SelfCollisions = 1, ///< if set, physics engine will use contact forces from self-collisions
};

/** \brief <b>[interface]</b> The physics engine interfaces supporting simulations and dynamics. See \ref arch_physicsengine.
    \ingroup interfaces
 */
class OPENRAVE_API PhysicsEngineBase : public InterfaceBase
{
public:
    PhysicsEngineBase(EnvironmentBasePtr penv) : InterfaceBase(PT_PhysicsEngine, penv) {
    }
    virtual ~PhysicsEngineBase() {
    }

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_PhysicsEngine;
    }

    /// Set basic physics engine using the PhysicsEngineOptions enum
    virtual bool SetPhysicsOptions(int physicsoptions) = 0;
    virtual int GetPhysicsOptions() const = 0;

    /// \deprecated (10/11/18) use SendCommand instead
    virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput) RAVE_DEPRECATED = 0;

    /// called when environment sets this physics engine, engine assumes responsibility for KinBody::_pPhysicsData
    virtual bool InitEnvironment() = 0;

    /// called when environment switches to a different physics engine
    /// has to clear/deallocate any memory associated with KinBody::_pPhysicsData
    virtual void DestroyEnvironment() = 0;

    /// \brief notified when a new body has been initialized in the environment. Return
    virtual bool InitKinBody(KinBodyPtr body) = 0;

    /// \brief notified when a body has been removed from the environment.
    virtual void RemoveKinBody(KinBodyPtr body) = 0;

    /// \brief Force the body velocity of a link, velocities correspond to the link's coordinate system origin.
    ///
    /// \param[in] link link to set velocities.
    /// \param[in] linearvel linear velocity of base link
    /// \param[in] angularvel angular velocity rotation_axis*theta_dot
    virtual bool SetLinkVelocity(KinBody::LinkPtr link, const Vector& linearvel, const Vector& angularvel) = 0;

    /// \brief Sets the velocities for each link, velocities correspond to the link's coordinate system origin.
    ///
    /// \param[in] body the body to query velocities from.
    /// \param[in] velocities sets the linear and angular (axis * angular_speed) velocities for each link
    virtual bool SetLinkVelocities(KinBodyPtr body, const std::vector<std::pair<Vector,Vector> >& velocities) = 0;

    /// \brief Gets the velocity of a link, velocities correspond to the link's coordinate system origin.
    /// \param[out] linearvel - linear velocity of base link
    /// \param[out] angularvel - angular velocity rotation_axis*theta_dot
    virtual bool GetLinkVelocity(KinBody::LinkConstPtr link, Vector& linearvel, Vector& angularvel) = 0;

    /// \brief Sets the velocities for each link, velocities correspond to the link's coordinate system origin.
    /// \param[out] velocities the linear and angular (axis * angular_speed) velocities for each link.
    virtual bool GetLinkVelocities(KinBodyConstPtr body, std::vector<std::pair<Vector,Vector> >& velocities) = 0;

    /// add a force at a particular position in a link
    /// \param force the direction and magnitude of the force
    /// \param position in the world where the force is getting applied
    /// \param bAdd if true, force is added to previous forces, otherwise it is set
    virtual bool SetBodyForce(KinBody::LinkPtr link, const Vector& force, const Vector& position, bool bAdd) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// adds torque to a body (absolute coords)
    /// \param link the link to add a torque to
    /// \param torque torque vector
    /// \param bAdd if true, torque is added to previous torques, otherwise it is set
    virtual bool SetBodyTorque(KinBody::LinkPtr link, const Vector& torque, bool bAdd) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// adds torque to a joint
    /// \param pjoint - the joint the torque is added to
    /// \param pTorques - the torques added to the joint. Pointer because the joint dof can be greater than 1.
    virtual bool AddJointTorque(KinBody::JointPtr pjoint, const std::vector<dReal>& pTorques) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \deprecated (13/03/25)
    virtual bool GetLinkForceTorque(KinBody::LinkConstPtr link, Vector& force, Vector& torque);

    /// Return forces and torques exerted by a joint wrt the joint anchor frame.
    /// \param[in] joint a constant pointer to a joint
    /// \param[out] force current overall force exerted by the joint
    /// \param[out] torque current overall torque exerted by the joint
    virtual bool GetJointForceTorque(KinBody::JointConstPtr joint, Vector& force, Vector& torque) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// set the gravity direction
    virtual void SetGravity(const Vector& gravity) OPENRAVE_DUMMY_IMPLEMENTATION;
    virtual const Vector& GetGravity() OPENRAVE_DUMMY_IMPLEMENTATION;

    /// dynamically simulate system for fTimeElapsed seconds
    /// add torques to the joints of the body. Torques disappear after one timestep of simulation
    virtual void SimulateStep(dReal fTimeElapsed)=0;

    /// \deprecated (10/11/18)
    virtual bool GetBodyVelocity(KinBodyConstPtr body, std::vector<Vector>& vLinearVelocities, std::vector<Vector>& vAngularVelocities) RAVE_DEPRECATED {
        std::vector<std::pair<Vector,Vector> > velocities;
        if( !GetLinkVelocities(body, velocities) ) {
            return false;
        }
        vLinearVelocities.resize(velocities.size());
        vAngularVelocities.resize(velocities.size());
        for(size_t i = 0; i < velocities.size(); ++i) {
            vLinearVelocities[i] = velocities[i].first;
            vAngularVelocities[i] = velocities[i].second;
        }
        return true;
    }

    virtual bool SetBodyVelocity(KinBodyPtr body, const std::vector<Vector>& vLinearVelocities, const std::vector<Vector>& vAngularVelocities) RAVE_DEPRECATED {
        BOOST_ASSERT(vLinearVelocities.size()==vAngularVelocities.size());
        std::vector<std::pair<Vector,Vector> > velocities(vLinearVelocities.size());
        for(size_t i = 0; i < velocities.size(); ++i) {
            velocities[i].first = vLinearVelocities[i];
            velocities[i].second = vAngularVelocities[i];
        }
        return SetLinkVelocities(body,velocities);
    }

protected:
    /// \deprecated (12/12/11)
    virtual void SetPhysicsData(KinBodyPtr body, UserDataPtr data) RAVE_DEPRECATED {
        body->SetUserData(GetXMLId(), data);
    }
};

} // end namespace OpenRAVE

#endif
