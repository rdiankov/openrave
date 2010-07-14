// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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

/** \brief The physics engine interfaces supporting simulations and dynamics.
    
    \ingroup interfaces
*/
class RAVE_API PhysicsEngineBase : public InterfaceBase
{
public:
    PhysicsEngineBase(EnvironmentBasePtr penv) : InterfaceBase(PT_PhysicsEngine, penv) {}
    virtual ~PhysicsEngineBase() {}

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() { return PT_PhysicsEngine; }
    
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
    virtual bool InitKinBody(KinBodyPtr pbody) = 0;

    /// force the body velocity of a link
    /// \param[in] linearvel linear velocity of base link
    /// \param[in] angularvel angular velocity rotation_axis*theta_dot
    virtual bool SetLinkVelocity(KinBody::LinkPtr plink, const Vector& linearvel, const Vector& angularvel) = 0;

    /// sets the body velocity
    /// \param[in] linearvel linear velocity of base link
    /// \param[in] angularvel angular velocity rotation_axis*theta_dot
    /// \param[in] pJointVelocity - the joint velocities of the robot
    virtual bool SetBodyVelocity(KinBodyPtr pbody, const Vector& linearvel, const Vector& angularvel, const std::vector<dReal>& pJointVelocity) = 0;

    /// sets the velocities for each link
    /// \param[out] pLinearVelocities the linear velocities for each link
    /// \param[out] pAngularVelocities the angular velocities for each link (axis * angular_speed)
    virtual bool SetBodyVelocity(KinBodyPtr pbody, const std::vector<Vector>& pLinearVelocities, const std::vector<Vector>& pAngularVelocities) = 0;

    /// gets the velocity of a link
    /// \param[out] linearvel - linear velocity of base link
    /// \param[out] angularvel - angular velocity rotation_axis*theta_dot
    virtual bool GetLinkVelocity(KinBody::LinkConstPtr plink, Vector& linearvel, Vector& angularvel) = 0;

    /// gets the velocity
    /// \param[out] linearvel - linear velocity of base link
    /// \param[out] angularvel - angular velocity rotation_axis*theta_dot
    virtual bool GetBodyVelocity(KinBodyConstPtr pbody, Vector& linearvel, Vector& angularvel, std::vector<dReal>& pJointVelocity) = 0;

    /// sets the velocities for each link
    /// \param[out] pLinearVelocities the linear velocities for each link, has to be a valid pointer
    /// \param[out] pAngularVelocities the angular velocities for each link (axis * angular_speed), has to be a valid pointer
    virtual bool GetBodyVelocity(KinBodyConstPtr pbody, std::vector<Vector>& pLinearVelocities, std::vector<Vector>& pAngularVelocities) = 0;

    /// sets the joint velocity
    /// \param[in] pjoint the joint
    /// \param[in] vJointVelocity the new joint velocity
    virtual bool SetJointVelocity(KinBody::JointPtr pjoint, const std::vector<dReal>& vJointVelocity) = 0;

    /// gets the joint velocity
    /// \param[out] vJointVelocity the new joint velocity
    virtual bool GetJointVelocity(KinBody::JointConstPtr pjoint, std::vector<dReal>& vJointVelocity) = 0;

    /// add a force at a particular position in a link
    /// \param force the direction and magnitude of the force
    /// \param position in the world where the force is getting applied
    /// \param bAdd if true, force is added to previous forces, otherwise it is set
    virtual bool SetBodyForce(KinBody::LinkPtr plink, const Vector& force, const Vector& position, bool bAdd) = 0;

    /// adds torque to a body (absolute coords)
    /// \param plink the link to add a torque to
    /// \param torque torque vector
    /// \param bAdd if true, torque is added to previous torques, otherwise it is set
    virtual bool SetBodyTorque(KinBody::LinkPtr plink, const Vector& torque, bool bAdd) = 0;

    /// adds torque to a joint
    /// \param pjoint - the joint the torque is added to
    /// \param pTorques - the torques added to the joint. Pointer because the joint dof can be greater than 1.
    virtual bool AddJointTorque(KinBody::JointPtr pjoint, const std::vector<dReal>& pTorques) = 0;

    /// \param[in] plink the link
    /// \param[out] force current accumulated force on the COM of the link
    /// \param[out] torque current accumulated torque on the COM of the link
    virtual bool GetLinkForceTorque(KinBody::LinkConstPtr plink, Vector& force, Vector& torque) = 0;
    
    /// set the gravity direction
    virtual void SetGravity(const Vector& gravity) = 0;
    virtual Vector GetGravity() = 0;

    /// dynamically simulate system for fTimeElapsed seconds
    /// add torques to the joints of the body. Torques disappear after one timestep of simulation
    virtual void SimulateStep(dReal fTimeElapsed)=0;
    
protected:
	virtual void SetPhysicsData(KinBodyPtr pbody, boost::shared_ptr<void> data) { pbody->SetPhysicsData(data); }

private:
    virtual const char* GetHash() const { return OPENRAVE_PHYSICSENGINE_HASH; }
};

} // end namespace OpenRAVE

#endif
