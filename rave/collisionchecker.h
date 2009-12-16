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
#ifndef OPENRAVE_COLLISIONCHECKER_H
#define OPENRAVE_COLLISIONCHECKER_H

namespace OpenRAVE {

/// options for collision checker
enum CollisionOptions
{
    CO_Distance = 1,
    CO_UseTolerance = 2,
    CO_Contacts = 4,    ///< Collision returns contact points
    CO_RayAnyHit = 8, ///< when performing collision with rays, if this is set, algorithm just returns any hit instead of the closest (can be faster)
};

/// derive from this class for every different collision checker library that OpenRAVE should support
class RAVE_API CollisionCheckerBase : public InterfaceBase
{
public:
    CollisionCheckerBase(EnvironmentBasePtr penv) : InterfaceBase(PT_CollisionChecker, penv) {}
    virtual ~CollisionCheckerBase() {}

    /// called when environment sets this collision checker, checker assumes responsibility for KinBody::_pCollisionData
    /// checker should also gather all current bodies in the environment and put them in its collision space
    virtual bool InitEnvironment() = 0;

    /// called when environment switches to a different collision checker engine
    /// has to clear/deallocate any memory associated with KinBody::_pCollisionData
    virtual void DestroyEnvironment() = 0;

    /// notified when a new body has been initialized in the environment
    virtual bool InitKinBody(KinBodyPtr pbody) = 0;

    /// enables or disables a kinematic body from being considered in collisions
    /// \return true if operation succeeded
    virtual bool Enable(KinBodyConstPtr pbody, bool bEnable) = 0;
    
    /// enables or disables a link from being considered in collisions
    /// \return true if operation succeeded
    virtual bool EnableLink(KinBody::LinkConstPtr pbody, bool bEnable) = 0;

    /// Set basic collision options using the CollisionOptions enum
    virtual bool SetCollisionOptions(int collisionoptions) = 0;
    virtual int GetCollisionOptions() const = 0;

    /// set and get various collision checker options
    /// \return true if command succeeded
    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) = 0;

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr())=0;
    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr())=0;
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr())=0;
    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr())=0;
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())=0;
    
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())=0;
    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr())=0;
    
    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) = 0;
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr()) = 0;
    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) = 0;

    virtual void SetTolerance(dReal tolerance) = 0;

protected:
	virtual void SetCollisionData(KinBodyPtr pbody, boost::shared_ptr<void> data) { pbody->SetCollisionData(data); }

private:
    virtual const char* GetHash() const { return OPENRAVE_COLLISIONCHECKER_HASH; }
};

} // end namespace OpenRAVE

#endif
