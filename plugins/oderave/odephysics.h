// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
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
#ifndef RAVE_PHYSICSENGINE_ODE
#define RAVE_PHYSICSENGINE_ODE

#include "odespace.h"

class ODEPhysicsEngine : public OpenRAVE::PhysicsEngineBase
{
public:
    ODEPhysicsEngine(OpenRAVE::EnvironmentBase* penv);

    virtual bool InitEnvironment();
    virtual void DestroyEnvironment();
    
    virtual bool InitKinBody(KinBody* pbody);
    virtual bool DestroyKinBody(KinBody* pbody);
    
    virtual bool SetPhysicsOptions(int physicsoptions);
    virtual int GetPhysicsOptions() const;

    virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput);

    virtual bool SetBodyVelocity(KinBody* pbody, const Vector& linearvel, const Vector& angularvel, const OpenRAVE::dReal* pJointVelocity);
    virtual bool SetBodyVelocity(KinBody* pbody, const Vector* pLinearVelocities, const Vector* pAngularVelocities);

    virtual bool GetBodyVelocity(const KinBody* pbody, Vector& linearvel, Vector& angularvel, OpenRAVE::dReal* pJointVelocity);
    virtual bool GetBodyVelocity(KinBody* pbody, Vector* pLinearVelocities, Vector* pAngularVelocities);

    virtual bool SetJointVelocity(KinBody::Joint* pjoint, const OpenRAVE::dReal* pJointVelocity);
    virtual bool GetJointVelocity(const KinBody::Joint* pjoint, OpenRAVE::dReal* pJointVelocity);

    virtual bool SetBodyForce(KinBody::Link* plink, const Vector& force, const Vector& position, bool bAdd);
    virtual bool SetBodyTorque(KinBody::Link* plink, const Vector& torque, bool bAdd);
    virtual bool AddJointTorque(KinBody::Joint* pjoint, const OpenRAVE::dReal* pTorques);
    
    virtual void SetGravity(const Vector& gravity);
    virtual Vector GetGravity();
    
    virtual void SimulateStep(OpenRAVE::dReal fTimeElapsed);
    
 private:
    static void* GetPhysicsInfo(const KinBody* pbody) { return pbody->GetPhysicsData(); }
    
    static void nearCallback(void *data, dGeomID o1, dGeomID o2);
    void _nearCallback(dGeomID o1, dGeomID o2);

    static void SyncCallback(void* userdata, ODESpace::KINBODYINFO* pinfo);
    void _SyncCallback(ODESpace::KINBODYINFO* pinfo);

    ODESpace odespace;
    Vector _gravity;
    int _options;
    
    typedef void (*JointSetFn)(dJointID, int param, dReal val);
    typedef dReal (*JointGetFn)(dJointID);
    typedef dReal (*JointGetParamFn)(dJointID, int param);
    typedef void (*JointAddForceFn)(dJointID, const dReal* vals);

    JointSetFn _jointset[12];
    JointGetParamFn _jointgetparam[12];
    JointAddForceFn _jointadd[12];
    vector<JointGetFn> _jointgetpos[12], _jointgetvel[12];
};

#endif
