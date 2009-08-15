// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_PHYSICSENGINE_H
#define OPENRAVE_PHYSICSENGINE_H

namespace OpenRAVE {

/// basic options for physics engine
enum PhysicsEngineOptions
{
    PEO_SelfCollisions = 1, ///< if set, physics engine will use contact forces from self-collisions
};

/// A physics engine supports simulating the dynamics of every object in the environment
class PhysicsEngineBase : public InterfaceBase
{
public:
    PhysicsEngineBase(EnvironmentBase* penv) : InterfaceBase(penv) {}
    virtual ~PhysicsEngineBase() {}

    /// Set basic physics engine using the PhysicsEngineOptions enum
    virtual bool SetPhysicsOptions(int physicsoptions) = 0;
    virtual int GetPhysicsOptions() const = 0;

    /// set and get various physics engine options
    /// \return true if command succeeded
    virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput) = 0;

    /// called when environment sets this physics engine, engine assumes responsibility for KinBody::_pPhysicsData
    virtual bool InitEnvironment() = 0;

    /// called when environment switches to a different physics engine
    /// has to clear/deallocate any memory associated with KinBody::_pPhysicsData
    virtual void DestroyEnvironment() = 0;

    /// notified when a new body has been initialized in the environment
    virtual bool InitKinBody(KinBody* pbody) = 0;

    /// notified when a body is about to be destroyed
    virtual bool DestroyKinBody(KinBody* pbody) = 0;

    /// sets the body velocity
    /// \param linearvel linear velocity of base link
    /// \param angularvel angular velocity rotation_axis*theta_dot
    /// \param pJointVelocity (optional) - the joint velocities of the robot
    virtual bool SetBodyVelocity(KinBody* pbody, const Vector& linearvel, const Vector& angularvel, const dReal* pJointVelocity) = 0;

    /// sets the velocities for each link
    /// \param pLinearVelocities the linear velocities for each link
    /// \param pAngularVelocities the angular velocities for each link (axis * angular_speed)
    virtual bool SetBodyVelocity(KinBody* pbody, const Vector* pLinearVelocities, const Vector* pAngularVelocities) = 0;

    /// gets the velocity
    /// \param linearvel - linear velocity of base link
    /// \param angularvel - angular velocity rotation_axis*theta_dot
    virtual bool GetBodyVelocity(const KinBody* pbody, Vector& linearvel, Vector& angularvel, dReal* pJointVelocity) = 0;

    /// sets the velocities for each link
    /// \param pLinearVelocities the linear velocities for each link, has to be a valid pointer
    /// \param pAngularVelocities the angular velocities for each link (axis * angular_speed), has to be a valid pointer
    virtual bool GetBodyVelocity(KinBody* pbody, Vector* pLinearVelocities, Vector* pAngularVelocities) = 0;

    /// sets the body joint
    virtual bool SetJointVelocity(KinBody::Joint* pjoint, const dReal* pJointVelocity) = 0;

    /// gets the joint velocity
    virtual bool GetJointVelocity(const KinBody::Joint* pjoint, dReal* pJointVelocity) = 0;

    /// add a force at a particular position in a link
    /// \param force the direction and magnitude of the force
    /// \param pos in the world where the force is getting applied
    /// \param bAdd if true, force is added to previous forces, otherwise it is set
    virtual bool SetBodyForce(KinBody::Link* plink, const Vector& force, const Vector& position, bool bAdd) = 0;

    /// adds torque to a body (absolute coords)
    /// \param plink the link to add a torque to
    /// \param torque torque vector
    /// \param bAdd if true, torque is added to previous torques, otherwise it is set
    virtual bool SetBodyTorque(KinBody::Link* plink, const Vector& torque, bool bAdd) = 0;

    /// adds torque to a joint
    /// \param pjoint - the joint the torque is added to
    /// \param pTorques - the torques added to the joint. Pointer because the joint dof can be greater than 1.
    virtual bool AddJointTorque(KinBody::Joint* pjoint, const dReal* pTorques) = 0;

    /// set the gravity direction
    virtual void SetGravity(const Vector& gravity) = 0;
    virtual Vector GetGravity() = 0;

    /// dynamically simulate system for fTimeElapsed seconds
    /// add torques to the joints of the body. Torques disappear after one timestep of simulation
    virtual void SimulateStep(dReal fTimeElapsed)=0;
    
protected:
    virtual const char* GetHash() const { return OPENRAVE_PHYSICSENGINE_HASH; }

	virtual void SetPhysicsData(KinBody* pbody, void* data) {
		if( pbody != NULL )
			pbody->SetPhysicsData(data);
	}
};

} // end namespace OpenRAVE

#endif
