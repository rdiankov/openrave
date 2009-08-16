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
class CollisionCheckerBase : public InterfaceBase
{
public:
    CollisionCheckerBase(EnvironmentBase* penv) : InterfaceBase(PT_CollisionChecker, penv) {}
    virtual ~CollisionCheckerBase() {}

    /// called when environment sets this collision checker, checker assumes responsibility for KinBody::_pCollisionData
    /// checker should also gather all current bodies in the environment and put them in its collision space
    virtual bool InitEnvironment() = 0;

    /// called when environment switches to a different collision checker engine
    /// has to clear/deallocate any memory associated with KinBody::_pCollisionData
    virtual void DestroyEnvironment() = 0;

    /// notified when a new body has been initialized in the environment
    virtual bool InitKinBody(KinBody* pbody) = 0;

    /// notified when a body is about to be destroyed
    virtual bool DestroyKinBody(KinBody* pbody) = 0;

    /// enables or disables a kinematic body from being considered in collisions
    /// \return true if operation succeeded
    virtual bool Enable(const KinBody* pbody, bool bEnable) = 0;
    
    /// enables or disables a link from being considered in collisions
    /// \return true if operation succeeded
    virtual bool EnableLink(const KinBody::Link* pbody, bool bEnable) = 0;

    /// Set basic collision options using the CollisionOptions enum
    virtual bool SetCollisionOptions(int collisionoptions) = 0;
    virtual int GetCollisionOptions() const = 0;

    /// set and get various collision checker options
    /// \return true if command succeeded
    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) = 0;

    virtual bool CheckCollision(const KinBody* pbody1, COLLISIONREPORT*)=0;
    virtual bool CheckCollision(const KinBody* pbody1, const KinBody* pbody2, COLLISIONREPORT*)=0;
    virtual bool CheckCollision(const KinBody::Link* plink, COLLISIONREPORT*)=0;
    virtual bool CheckCollision(const KinBody::Link* plink1, const KinBody::Link* plink2, COLLISIONREPORT*)=0;
    virtual bool CheckCollision(const KinBody::Link* plink, const KinBody* pbody, COLLISIONREPORT*)=0;
    
    virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT*)=0;
    virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT*)=0;
    
    virtual bool CheckCollision(const RAY& ray, const KinBody::Link* plink, COLLISIONREPORT* pReport) = 0;
    virtual bool CheckCollision(const RAY& ray, const KinBody* pbody, COLLISIONREPORT* pReport) = 0;
    virtual bool CheckCollision(const RAY& ray, COLLISIONREPORT* pReport) = 0;

    //tolerance check
    virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, dReal tolerance)= 0;
    virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, dReal tolerance)= 0;

    virtual bool CheckSelfCollision(const KinBody* pbody, COLLISIONREPORT* pReport) = 0;

protected:
	virtual void SetCollisionData(KinBody* pbody, void* data) {
		if( pbody != NULL )
			pbody->SetCollisionData(data);
	}

private:
    virtual const char* GetHash() const { return OPENRAVE_COLLISIONCHECKER_HASH; }
};

} // end namespace OpenRAVE

#endif
