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
    // ODE joint helper fns
    static void DummySetParam(dJointID id, int param, dReal)
    {
        cerr << "failed to set param to dummy " << dJointGetType(id) << endl;
    }

    static dReal DummyGetParam(dJointID id, int param)
    {
        return 0;
    }

    static void DummyAddForce(dJointID id, const dReal* vals)
    {
        cerr << "failed to add force to dummy " << dJointGetType(id) << endl;
    }

    static void dJointAddHingeTorque_(dJointID id, const dReal* vals) { dJointAddHingeTorque(id, vals[0]); }
    static void dJointAddSliderForce_(dJointID id, const dReal* vals) { dJointAddSliderForce(id, vals[0]); }
    static void dJointAddUniversalTorques_(dJointID id, const dReal* vals) { dJointAddUniversalTorques(id, vals[0], vals[1]); }
    static void dJointAddHinge2Torques_(dJointID id, const dReal* vals) { dJointAddHinge2Torques(id, vals[0], vals[1]); }

    static dReal dJointGetHinge2Angle2_(dJointID id) { return 0; }

    static dReal JointGetBallVelocityX(dJointID) { return 0; }
    static dReal JointGetBallVelocityY(dJointID) { return 0; }
    static dReal JointGetBallVelocityZ(dJointID) { return 0; }

    inline boost::shared_ptr<ODEPhysicsEngine> shared_physics() { return boost::static_pointer_cast<ODEPhysicsEngine>(shared_from_this()); }
    inline boost::shared_ptr<ODEPhysicsEngine const> shared_physics_const() const { return boost::static_pointer_cast<ODEPhysicsEngine const>(shared_from_this()); }

 public:
 ODEPhysicsEngine(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::PhysicsEngineBase(penv), odespace(new ODESpace(penv, GetPhysicsInfo, true)) {
        memset(_jointset, 0, sizeof(_jointset));
        _jointset[dJointTypeBall] = DummySetParam;
        _jointset[dJointTypeHinge] = dJointSetHingeParam;
        _jointset[dJointTypeSlider] = dJointSetSliderParam;
        _jointset[dJointTypeUniversal] = dJointSetUniversalParam;
        _jointset[dJointTypeHinge2] = dJointSetHinge2Param;

        memset(_jointadd, 0, sizeof(_jointadd));
        _jointadd[dJointTypeBall] = DummyAddForce;
        _jointadd[dJointTypeHinge] = dJointAddHingeTorque_;
        _jointadd[dJointTypeSlider] = dJointAddSliderForce_;
        _jointadd[dJointTypeUniversal] = dJointAddUniversalTorques_;
        _jointadd[dJointTypeHinge2] = dJointAddHinge2Torques_;
    
        _jointgetvel[dJointTypeBall].push_back(JointGetBallVelocityX);
        _jointgetvel[dJointTypeBall].push_back(JointGetBallVelocityY);
        _jointgetvel[dJointTypeBall].push_back(JointGetBallVelocityZ);
        _jointgetvel[dJointTypeHinge].push_back(dJointGetHingeAngleRate);
        _jointgetvel[dJointTypeSlider].push_back(dJointGetSliderPositionRate);
        _jointgetvel[dJointTypeUniversal].push_back(dJointGetUniversalAngle1Rate);
        _jointgetvel[dJointTypeUniversal].push_back(dJointGetUniversalAngle2Rate);
        _jointgetvel[dJointTypeHinge2].push_back(dJointGetHinge2Angle1Rate);
        _jointgetvel[dJointTypeHinge2].push_back(dJointGetHinge2Angle2Rate);
    }

    virtual bool InitEnvironment()
    {
        odespace->SetSynchornizationCallback(boost::bind(&ODEPhysicsEngine::_SyncCallback, shared_physics(),_1));
        if( !odespace->InitEnvironment() )
            return false;

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies)
            InitKinBody(*itbody);

        SetGravity(_gravity);
        return true;
    }


    virtual void DestroyEnvironment()
    {
        odespace->DestroyEnvironment();
    }
    
    virtual bool InitKinBody(KinBodyPtr pbody)
    {
        boost::shared_ptr<void> pinfo = odespace->InitKinBody(pbody);
        SetPhysicsData(pbody, pinfo);
        return !!pinfo;
    }

    virtual bool SetPhysicsOptions(int physicsoptions)
    {
        _options = physicsoptions;
        return true;
    }

    virtual int GetPhysicsOptions() const
    {
        return _options;
    }

    virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput)
    {
        return false;
    }

    virtual bool SetBodyVelocity(KinBodyPtr pbody, const Vector& linearvel, const Vector& angularvel)
    {
        RAVELOG_ERRORA("setting ode body velocities not supported!\n");
        return false;
    }
    virtual bool SetBodyVelocity(KinBodyPtr pbody, const Vector& linearvel, const Vector& angularvel, const std::vector<OpenRAVE::dReal>& pJointVelocity)
    {
        RAVELOG_ERRORA("setting ode body velocities not supported!\n");
        return false;
    }
    virtual bool SetBodyVelocity(KinBodyPtr pbody, const std::vector<Vector>& pLinearVelocities, const std::vector<Vector>& pAngularVelocities)
    {
        RAVELOG_ERRORA("setting ode body velocities not supported!\n");
        return false;
    }

    virtual bool GetBodyVelocity(KinBodyConstPtr pbody, Vector& linearvel, Vector& angularvel)
    {
        odespace->Synchronize(pbody);
        dBodyID body = odespace->GetLinkBody(pbody->GetLinks().front());
        if( body ) {
            const dReal* p = dBodyGetLinearVel(body);
            linearvel = Vector(p[0], p[1], p[2]);
            p = dBodyGetAngularVel(body);
            angularvel = Vector(p[0], p[1], p[2]);
        }
        else
            angularvel = linearvel = Vector(0,0,0);
        return true;
    }

    virtual bool GetBodyVelocity(KinBodyConstPtr pbody, Vector& linearvel, Vector& angularvel, std::vector<OpenRAVE::dReal>& pJointVelocity)
    {
        GetBodyVelocity(pbody,linearvel,angularvel);

        pJointVelocity.resize(pbody->GetDOF());
        vector<JointGetFn>::iterator itfn;
        FOREACHC(it, pbody->GetJoints()) {
            dJointID joint = odespace->GetJoint(*it);
            vector<OpenRAVE::dReal>::iterator itvel = pJointVelocity.begin()+(*it)->GetDOFIndex();
            FORIT(itfn, _jointgetvel[dJointGetType(joint)])
                *itvel++ = (*itfn)(joint);
        }

        return true;
    }

    virtual bool GetBodyVelocity(KinBodyConstPtr pbody, std::vector<Vector>& pLinearVelocities, std::vector<Vector>& pAngularVelocities)
    {
        odespace->Synchronize(pbody);
        pLinearVelocities.resize(pbody->GetLinks().size());
        pAngularVelocities.resize(pbody->GetLinks().size());
        std::vector<Vector>::iterator itlin = pLinearVelocities.begin(), itang = pAngularVelocities.begin();

        FOREACHC(itlink, pbody->GetLinks()) {
            dBodyID body = odespace->GetLinkBody(*itlink);
            if( body ) {
                const dReal* pf = dBodyGetLinearVel(body);
                *itlin++ = Vector(pf[0], pf[1], pf[2]);
                pf = dBodyGetAngularVel(body);
                *itang++ = Vector(pf[0], pf[1], pf[2]);
            }
            else {
                *itlin++ = Vector(0,0,0);
                *itang = Vector(0,0,0);
            }
        }

        return true;
    }

    virtual bool SetJointVelocity(KinBody::JointPtr pjoint, const std::vector<OpenRAVE::dReal>& pJointVelocity)
    {
        dJointID joint = odespace->GetJoint(pjoint);
        BOOST_ASSERT( joint != NULL );

        std::vector<OpenRAVE::dReal>::const_iterator itvel = pJointVelocity.begin();
        odespace->Synchronize(KinBodyConstPtr(pjoint->GetParent()));
        for(int i = 0; i < pjoint->GetDOF(); ++i)
            _jointset[dJointGetType(joint)](joint, dParamVel + dParamGroup * i, *itvel++);
        return true;
    }

    virtual bool GetJointVelocity(KinBody::JointConstPtr pjoint, std::vector<OpenRAVE::dReal>& pJointVelocity)
    {
        dJointID joint = odespace->GetJoint(pjoint);
        BOOST_ASSERT( joint != NULL );

        odespace->Synchronize(KinBodyConstPtr(pjoint->GetParent()));
        vector<JointGetFn>::iterator itfn;
        pJointVelocity.resize(pjoint->GetDOF());
        vector<OpenRAVE::dReal>::iterator itvel = pJointVelocity.begin();
        FORIT(itfn, _jointgetvel[dJointGetType(joint)])
            *itvel++ = (*itfn)(joint);
        return true;
    }

    virtual bool SetBodyForce(KinBody::LinkPtr plink, const Vector& force, const Vector& position, bool bAdd)
    {
        dBodyID body = odespace->GetLinkBody(plink);
        if( body == NULL )
            return false;

        odespace->Synchronize(KinBodyConstPtr(plink->GetParent()));
    
        if( !bAdd )
            dBodySetForce(body, 0, 0, 0);
        dBodyAddForceAtPos(body, force.x, force.y, force.z, position.x, position.y, position.z);
        return true;
    }

    virtual bool SetBodyTorque(KinBody::LinkPtr plink, const Vector& torque, bool bAdd)
    {
        dBodyID body = odespace->GetLinkBody(plink);
        if( body == NULL )
            return false;

        odespace->Synchronize(KinBodyConstPtr(plink->GetParent()));

        if( !bAdd )
            dBodySetTorque(body, torque.x, torque.y, torque.z);
        else
            dBodyAddTorque(body, torque.x, torque.y, torque.z);
        return true;
    }

    virtual bool AddJointTorque(KinBody::JointPtr pjoint, const std::vector<OpenRAVE::dReal>& pTorques)
    {
        dJointID joint = odespace->GetJoint(pjoint);
        BOOST_ASSERT( joint != NULL );

        odespace->Synchronize(KinBodyConstPtr(pjoint->GetParent()));
        std::vector<dReal> vtorques(pTorques.size());
        std::copy(pTorques.begin(),pTorques.end(),vtorques.begin());
        _jointadd[dJointGetType(joint)](joint, &vtorques[0]);
        return true;
    }

    virtual void SetGravity(const Vector& gravity)
    {
        _gravity = gravity;
        dWorldSetGravity(odespace->GetWorld(),_gravity.x, _gravity.y, _gravity.z);
    }

    virtual Vector GetGravity()
    {
        return _gravity;
    }
    
    virtual void SimulateStep(OpenRAVE::dReal fTimeElapsed)
    {
        odespace->Synchronize();
        dSpaceCollide (odespace->GetSpace(),this,nearCallback);
    
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);

        if( _options & OpenRAVE::PEO_SelfCollisions ) {
            FOREACHC(itbody, vbodies) {
                if( (*itbody)->GetLinks().size() > 1 ) {
                    // more than one link, check collision
                    dSpaceCollide(odespace->GetBodySpace(*itbody), this, nearCallback);
                }
            }
        }

        dWorldQuickStep(odespace->GetWorld(), fTimeElapsed);
        dJointGroupEmpty (odespace->GetContactGroup());

        // synchronize all the objects from the ODE world to the OpenRAVE world
        Transform t;
        FOREACHC(itbody, vbodies) {
            ODESpace::KinBodyInfoPtr pinfo = boost::static_pointer_cast<ODESpace::KinBodyInfo>((*itbody)->GetPhysicsData());
            BOOST_ASSERT( pinfo->vlinks.size() == (*itbody)->GetLinks().size());
            for(size_t i = 0; i < pinfo->vlinks.size(); ++i) {
                t.rot = *(RaveVector<dReal>*)dBodyGetQuaternion(pinfo->vlinks[i]->body);
                t.trans = *(RaveVector<dReal>*)dBodyGetPosition(pinfo->vlinks[i]->body);
                (*itbody)->GetLinks()[i]->SetTransform(t);
            }

            pinfo->nLastStamp = (*itbody)->GetUpdateStamp();
        }
    }

    
 private:
    static boost::shared_ptr<void> GetPhysicsInfo(KinBodyConstPtr pbody) { return pbody->GetPhysicsData(); }
    
    static void nearCallback(void *data, dGeomID o1, dGeomID o2)
    {
        ((ODEPhysicsEngine*)data)->_nearCallback(o1,o2);
    }

    void _nearCallback(dGeomID o1, dGeomID o2)
    {
        if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) )
            return;

        if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
            // colliding a space with something
            dSpaceCollide2(o1,o2,this, nearCallback);
            return;
        }

        dBodyID b1,b2;
        b1 = dGeomGetBody(o1);
        b2 = dGeomGetBody(o2);
        if (b1 && b2 && dAreConnected (b1,b2)) {
            return;
        }

        // ignore static, static collisions
        if( (b1 == NULL || !dBodyIsEnabled(b1)) && (b2 == NULL || !dBodyIsEnabled(b2)) )
            return;

        KinBody::LinkPtr pkb1,pkb2;
        if(!!b1 && dBodyGetData(b1))
            pkb1 = *(KinBody::LinkPtr*)dBodyGetData(b1);
        if(!!b2 && dBodyGetData(b1))
            pkb2 = *(KinBody::LinkPtr*)dBodyGetData(b2);

        if( !!pkb1 && !pkb1->IsEnabled() )
            return;
        if( !!pkb2 && !pkb2->IsEnabled() )
            return;

        if( pkb1->GetParent() == pkb2->GetParent() ) {
            // check if links are adjacent
            int minindex = min(pkb1->GetIndex(), pkb2->GetIndex());
            int maxindex = max(pkb1->GetIndex(), pkb2->GetIndex());

            if( pkb1->GetParent()->GetAdjacentLinks().find(minindex|(maxindex<<16)) != pkb1->GetParent()->GetAdjacentLinks().end() )
                return;
        }

        const int N = 16;
        dContact contact[N];
        int n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
        for (int i=0; i<n; i++) 
            {
                contact[i].surface.mode = 0;
                contact[i].surface.mu = (dReal)100;
                contact[i].surface.mu2 = 100;
                //        contact[i].surface.slip1 = 0.7;
                //        contact[i].surface.slip2 = 0.7;
                //        contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
                //        contact[i].surface.mu = 50.0; // was: dInfinity
                //        contact[i].surface.soft_erp = 0.96;
                //        contact[i].surface.soft_cfm = 0.04;
                dJointID c = dJointCreateContact (odespace->GetWorld(),odespace->GetContactGroup(),contact+i);

                // make sure that static objects are not enabled by adding a joint attaching them
                if( b1 ) b1 = dBodyIsEnabled(b1)?b1:0;
                if( b2 ) b2 = dBodyIsEnabled(b2)?b2:0;
                dJointAttach (c, b1, b2);

                //wprintf(L"intersection %s %s\n", ((KinBody::Link*)dBodyGetData(b1))->GetName(), ((KinBody::Link*)dBodyGetData(b2))->GetName());

                //        contact[i].surface.slip1 = 0.7;
                //        contact[i].surface.slip2 = 0.7;
                //        contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
                //        contact[i].surface.mu = 50.0; // was: dInfinity
                //        contact[i].surface.soft_erp = 0.96;
                //        contact[i].surface.soft_cfm = 0.04;
                //        dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
                //        dJointAttach (c,
                //            dGeomGetBody(contact[i].geom.g1),
                //            dGeomGetBody(contact[i].geom.g2));
            }
        //
        //        dJointID c = dJointCreateContact (GetEnv()->world,GetEnv()->contactgroup,&contact);
        //        dJointAttach (c,b1,b2);
    }

    void _SyncCallback(ODESpace::KinBodyInfoConstPtr pinfo)
    {
        // reset dynamics
        FOREACH(itlink, pinfo->vlinks) {
            if( (*itlink)->body != NULL ) {
                dBodySetAngularVel((*itlink)->body, 0, 0, 0);
                dBodySetLinearVel((*itlink)->body, 0, 0, 0);
            }
        }
    }

    boost::shared_ptr<ODESpace> odespace;
    Vector _gravity;
    int _options;
    
    typedef void (*JointSetFn)(dJointID, int param, dReal val);
    typedef dReal (*JointGetFn)(dJointID);
    typedef void (*JointAddForceFn)(dJointID, const dReal* vals);

    JointSetFn _jointset[12];
    JointAddForceFn _jointadd[12];
    vector<JointGetFn> _jointgetvel[12];
};

#endif
