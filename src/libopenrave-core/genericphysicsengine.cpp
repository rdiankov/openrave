// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "ravep.h"

namespace OpenRAVE {

class GenericPhysicsEngine : public PhysicsEngineBase
{
    class PhysicsData : public UserData
    {
public:
        PhysicsData(KinBodyPtr pbody) {
            linkvelocities.resize(pbody->GetLinks().size());
        }
        virtual ~PhysicsData() {
        }
        std::vector< std::pair<Vector, Vector> > linkvelocities;
    };

    inline boost::shared_ptr<PhysicsData> _GetData(const KinBodyConstPtr& pbody) {
        boost::shared_ptr<PhysicsData> pdata = boost::dynamic_pointer_cast<PhysicsData>(pbody->GetUserData("_genericphysics_"));
        if( !pdata ) {
            // isn't initialized for some reason, this can happen during environment cloning
            InitKinBody(boost::const_pointer_cast<KinBody>(pbody)); // fixme
            pdata = boost::dynamic_pointer_cast<PhysicsData>(pbody->GetUserData("_genericphysics_"));
        }
        return pdata;
    }

public:
    GenericPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput) : PhysicsEngineBase(penv) {
    }
    virtual bool SetPhysicsOptions(int physicsoptions) {
        return true;
    }
    virtual int GetPhysicsOptions() const {
        return 0;
    }

    virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput) {
        return true;
    }

    virtual bool InitEnvironment() {
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACH(itbody, vbodies) {
            InitKinBody(*itbody);
        }
        return true;
    }
    virtual void DestroyEnvironment()
    {
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACH(itbody, vbodies) {
            RemoveKinBody(*itbody);
        }
    }

    virtual bool InitKinBody(KinBodyPtr pbody) {
        pbody->SetUserData("_genericphysics_", UserDataPtr(new PhysicsData(pbody)));
        return true;
    }

    virtual void RemoveKinBody(KinBodyPtr pbody) {
        if( !!pbody ) {
            pbody->RemoveUserData("_genericphysics_");

            const int bodyid = pbody->GetEnvironmentBodyIndex();
            if (bodyid < (int)_pysicsDataCache.size() && !!_pysicsDataCache.at(bodyid)) {
                _pysicsDataCache.at(bodyid).reset();
            }
            else {
                RAVELOG_VERBOSE_FORMAT("bodyid=%d(name=%s) is already invalidated (either never initialized or invalidated already)", bodyid%(pbody->GetName()));
            }
        }
    }

    inline const boost::shared_ptr<PhysicsData>& _EnsureData(const KinBodyConstPtr& pbody)
    {
        // cannot access GetEnv()->GetMaxEnvironmentBodyIndex() because this function can be called while _mutexInterfaces is already locked, and _mutexInterfaces is not recursive mutex
        // so just resize to bodyIndex + 1 for now
        const int bodyIndex = pbody->GetEnvironmentBodyIndex();
        if (bodyIndex >= (int)_pysicsDataCache.size()) {
            //RAVELOG_INFO_FORMAT("extend _pysicsDataCache of size %d to %d from bodyIndex=%d (name=%s)", (_pysicsDataCache.size())%(bodyIndex + 1)%bodyIndex%(pbody->GetName()));
            _pysicsDataCache.resize(bodyIndex + 1, boost::shared_ptr<PhysicsData>());
        }
        if (!_pysicsDataCache.at(bodyIndex)) {
            _pysicsDataCache.at(bodyIndex) = _GetData(pbody);
        }
        return _pysicsDataCache.at(bodyIndex);
    }

    virtual bool GetLinkVelocity(KinBody::LinkConstPtr plink, Vector& linearvel, Vector& angularvel) {
        std::pair<Vector, Vector> vel = _EnsureData(plink->GetParent())->linkvelocities.at(plink->GetIndex());
        linearvel = vel.first;
        angularvel = vel.second;
        return true;
    }
    bool GetLinkVelocities(KinBodyConstPtr body, std::vector<std::pair<Vector,Vector> >& velocities) {
        velocities = _EnsureData(body)->linkvelocities;
        return true;
    }

    virtual bool SetLinkVelocity(KinBody::LinkPtr plink, const Vector& linearvel, const Vector& angularvel)
    {
        _EnsureData(plink->GetParent())->linkvelocities.at(plink->GetIndex()) = make_pair(linearvel,angularvel);
        return true;
    }
    bool SetLinkVelocities(KinBodyPtr body, const std::vector<std::pair<Vector,Vector> >& velocities)
    {
        _EnsureData(body)->linkvelocities = velocities;
        return true;
    }

    virtual bool SetBodyForce(KinBody::LinkPtr plink, const Vector& force, const Vector& position, bool bAdd) {
        return true;
    }
    virtual bool SetBodyTorque(KinBody::LinkPtr plink, const Vector& torque, bool bAdd) {
        return true;
    }
    virtual bool AddJointTorque(KinBody::JointPtr pjoint, const std::vector<dReal>& pTorques) {
        return true;
    }

    virtual void SetGravity(const Vector& gravity) {
        _vgravity = gravity;
    }
    virtual const Vector& GetGravity() {
        return _vgravity;
    }

    virtual bool GetJointForceTorque(KinBody::JointConstPtr pjoint, Vector& force, Vector& torque) {
        force = Vector(0,0,0);
        torque = Vector(0,0,0);
        return true;
    }

    virtual void SimulateStep(dReal fTimeElapsed) {
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        PhysicsEngineBase::Clone(preference,cloningoptions);
        boost::shared_ptr<GenericPhysicsEngine const> r = boost::dynamic_pointer_cast<GenericPhysicsEngine const>(preference);
        _vgravity = r->_vgravity;
    }

private:
    Vector _vgravity;
    std::vector<boost::shared_ptr<PhysicsData> > _pysicsDataCache; // cache of physics data to avoid slow call to dynamic_pointer_cast and GetUserData("_genericphysics_"). Index of the vector is the environment id (id of the body in the env, not __nUniqueId of env) of the kinbody who holds  at that index. It is assumed that environment id of kin body does not grow to infinity over time.
};

PhysicsEngineBasePtr CreateGenericPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PhysicsEngineBasePtr(new GenericPhysicsEngine(penv,sinput));
}

}
