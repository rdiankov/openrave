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

    std::shared_ptr<PhysicsData> _GetData(KinBodyConstPtr pbody) {
        std::shared_ptr<PhysicsData> pdata = std::dynamic_pointer_cast<PhysicsData>(pbody->GetUserData("_genericphysics_"));
        if( !pdata ) {
            // isn't initialized for some reason, this can happen during environment cloning
            InitKinBody(std::const_pointer_cast<KinBody>(pbody)); // fixme
            pdata = std::dynamic_pointer_cast<PhysicsData>(pbody->GetUserData("_genericphysics_"));
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
        }
    }

    virtual bool GetLinkVelocity(KinBody::LinkConstPtr plink, Vector& linearvel, Vector& angularvel) {
        std::pair<Vector, Vector> vel = _GetData(plink->GetParent())->linkvelocities.at(plink->GetIndex());
        linearvel = vel.first;
        angularvel = vel.second;
        return true;
    }
    bool GetLinkVelocities(KinBodyConstPtr body, std::vector<std::pair<Vector,Vector> >& velocities) {
        velocities = _GetData(body)->linkvelocities;
        return true;
    }

    virtual bool SetLinkVelocity(KinBody::LinkPtr plink, const Vector& linearvel, const Vector& angularvel)
    {
        _GetData(plink->GetParent())->linkvelocities.at(plink->GetIndex()) = make_pair(linearvel,angularvel);
        return true;
    }
    bool SetLinkVelocities(KinBodyPtr body, const std::vector<std::pair<Vector,Vector> >& velocities)
    {
        _GetData(body)->linkvelocities = velocities;
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
    virtual Vector GetGravity() {
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
        std::shared_ptr<GenericPhysicsEngine const> r = std::dynamic_pointer_cast<GenericPhysicsEngine const>(preference);
        _vgravity = r->_vgravity;
    }

private:
    Vector _vgravity;
};

PhysicsEngineBasePtr CreateGenericPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PhysicsEngineBasePtr(new GenericPhysicsEngine(penv,sinput));
}

}
