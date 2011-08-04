// -*- coding: utf-8 -*-
// Copyright (c) 2011 Max Argus, Nick Hillier, Katrina Monkley, Rosen Diankov
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
#include "bulletspace.h"

class BulletPhysicsEngine : public PhysicsEngineBase
{
    class PhysicsFilterCallback : public OpenRAVEFilterCallback
    {
public:
        PhysicsFilterCallback() : OpenRAVEFilterCallback() {
        }
        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
            KinBodyPtr pbody0 = plink0->GetParent();
            KinBodyPtr pbody1 = plink1->GetParent();
            if( pbody0->IsAttached(pbody1) ) {
                return false;
            }
            if( pbody0 != pbody1 ) {
                return true;
            }

            // check if links are in adjacency list
            int index0 = min(plink0->GetIndex(), plink1->GetIndex());
            int index1 = max(plink0->GetIndex(), plink1->GetIndex());
            return pbody0->GetNonAdjacentLinks(KinBody::AO_Enabled).count(index0|(index1<<16))>0;
        }
    };


    inline boost::shared_ptr<BulletPhysicsEngine> shared_physics() {
        return boost::static_pointer_cast<BulletPhysicsEngine>(shared_from_this());
    }
    inline boost::shared_ptr<BulletPhysicsEngine const> shared_physics_const() const {
        return boost::static_pointer_cast<BulletPhysicsEngine const>(shared_from_this());
    }

public:

    BulletPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput) : PhysicsEngineBase(penv), _space(new BulletSpace(penv, GetPhysicsInfo, true))
    {
        __description = ":Interface Authors: Max Argus, Nick Hillier, Katrina Monkley, Rosen Diankov\n\nInterface to `Bullet Physics Engine <http://bulletphysics.org/>`_\n";
        _solver_iterations = 10;
        _margin_depth = 0.001;
        _linear_damping = 0.2;
        _rotation_damping = 0.9;
        _global_contact_force_mixing = 0;
        _global_friction = 0.4;
        _global_restitution = 0.2;
    }

    virtual bool InitEnvironment()
    {
        RAVELOG_VERBOSE("init bullet physics environment\n");
        _space->SetSynchornizationCallback(boost::bind(&BulletPhysicsEngine::_SyncCallback, shared_physics(),_1));

        _broadphase.reset(new btDbvtBroadphase());

        // allowes configuration of collision detection
        _collisionConfiguration.reset(new btDefaultCollisionConfiguration());

        // handels conves and concave collisions
        //_dispatcher = new btOpenraveDispatcher::btOpenraveDispatcher(_collisionConfiguration);
        _dispatcher.reset(new btCollisionDispatcher(_collisionConfiguration.get()));
        _solver.reset(new btSequentialImpulseConstraintSolver());

        // btContinuousDynamicsWorld gives a segfault for some reason
        _dynamicsWorld.reset(new btDiscreteDynamicsWorld(_dispatcher.get(),_broadphase.get(),_solver.get(),_collisionConfiguration.get()));
        _filterCallback.reset(new PhysicsFilterCallback());
        _dynamicsWorld->getPairCache()->setOverlapFilterCallback(_filterCallback.get());

        btContactSolverInfo& solverInfo = _dynamicsWorld->getSolverInfo();
        RAVELOG_DEBUG(str(boost::format("bullet dynamics: m_numIterations=%d, m_globalCfm=%f")%_solver_iterations%_global_contact_force_mixing));
        solverInfo.m_numIterations = _solver_iterations;
        solverInfo.m_globalCfm = _global_contact_force_mixing;

        if(!_space->InitEnvironment(_dynamicsWorld)) {
            return false;
        }

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            InitKinBody(*itbody);
        }
        SetGravity(_gravity);
        return true;
    }

    virtual void DestroyEnvironment()
    {
        RAVELOG_VERBOSE("destroy bullet physics environment\n");
        _space->DestroyEnvironment();
        _dynamicsWorld.reset();
        _collisionConfiguration.reset();
        _broadphase.reset();
        _solver.reset();
        _dispatcher.reset();
        _report.reset();
        _filterCallback.reset();
        _listcallbacks.clear();
    }

    virtual bool InitKinBody(KinBodyPtr pbody)
    {
        if( !_dynamicsWorld ) {
            return false;
        }
        BulletSpace::KinBodyInfoPtr pinfo = _space->InitKinBody(pbody);
        SetPhysicsData(pbody, pinfo);
        if( !!pinfo ) {
            FOREACH(itlink,pinfo->vlinks) {
                (*itlink)->_rigidbody->setFriction(_global_friction);
                (*itlink)->_rigidbody->setRestitution(_global_restitution);
            }
        }
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

    virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput) {
        return false;
    }

    virtual bool SetLinkVelocity(KinBody::LinkPtr plink, const Vector& linearvel, const Vector& angularvel)
    {
        RAVELOG_DEBUG("SetLinkVelocity not supported\n");
        return false;
    }
    virtual bool SetLinkVelocities(KinBodyPtr pbody, const std::vector<std::pair<Vector,Vector> >& velocities)
    {
        RAVELOG_DEBUG("SetLinkVelocities not supported\n");
        return false;
    }

    virtual bool GetLinkVelocity(KinBody::LinkConstPtr plink, Vector& linearvel, Vector& angularvel)
    {
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        boost::shared_ptr<btRigidBody> rigidbody = boost::dynamic_pointer_cast<btRigidBody>(_space->GetLinkBody(plink));
        if (!!rigidbody) {
            btVector3 pf = rigidbody->getLinearVelocity();
            linearvel = Vector(pf[0],pf[1],pf[2]);
            pf = rigidbody->getAngularVelocity();
            angularvel = Vector(pf[0],pf[1],pf[2]);
        }
        else {
            linearvel = angularvel = Vector(0,0,0);
        }
        return true;
    }

    virtual bool GetLinkVelocities(KinBodyConstPtr pbody, std::vector<std::pair<Vector,Vector> >& velocities)
    {
        _space->Synchronize(pbody);
        velocities.resize(0);
        velocities.resize(pbody->GetLinks().size());

        FOREACHC(itlink, pbody->GetLinks()) {
            boost::shared_ptr<btRigidBody> rigidbody = boost::dynamic_pointer_cast<btRigidBody>(_space->GetLinkBody(*itlink));
            if(!!rigidbody) {
                btVector3 pf = rigidbody->getLinearVelocity();
                Vector angularvel(pf[0], pf[1], pf[2]);
                velocities.at((*itlink)->GetIndex()).second = angularvel;
                pf = rigidbody->getAngularVelocity();
                velocities.at((*itlink)->GetIndex()).first = Vector(pf[0], pf[1], pf[2]) - angularvel.cross((*itlink)->GetTransform()*(*itlink)->GetCOMOffset() - (*itlink)->GetTransform().trans);
            }
        }

        return true;
    }

    virtual bool SetJointVelocity(KinBody::JointPtr pjoint, const std::vector<dReal>& pJointVelocity)
    {
        boost::shared_ptr<btTypedConstraint> joint = _space->GetJoint(pjoint);
        _space->Synchronize(KinBodyConstPtr(pjoint->GetParent()));
        std::vector<btScalar> vVelocity(pJointVelocity.size());
        std::copy(pJointVelocity.begin(),pJointVelocity.end(),vVelocity.begin());
        RAVELOG_ERROR("SetJointVelocity not implemented\n");
        switch(joint->getConstraintType()) {
        case HINGE_CONSTRAINT_TYPE:
            break;
        case SLIDER_CONSTRAINT_TYPE:
            break;
        default:
            RAVELOG_ERROR(str(boost::format("SetJointVelocity joint type 0x%x not supported\n")%joint->getConstraintType()));
        }
        return true;
    }

    virtual bool GetJointVelocity(KinBody::JointConstPtr pjoint, std::vector<dReal>& pJointVelocity)
    {
        boost::shared_ptr<btTypedConstraint> joint = _space->GetJoint(pjoint);
        _space->Synchronize(KinBodyConstPtr(pjoint->GetParent()));
        pJointVelocity.resize(pjoint->GetDOF());
        switch(joint->getConstraintType()) {
        case HINGE_CONSTRAINT_TYPE:
            break;
        case SLIDER_CONSTRAINT_TYPE:
            break;
        default:
            RAVELOG_ERROR(str(boost::format("GetJointVelocity joint type 0x%x not supported\n")%joint->getConstraintType()));
        }
        return true;
    }

    virtual bool AddJointTorque(KinBody::JointPtr pjoint, const std::vector<dReal>& pTorques)
    {
        boost::shared_ptr<btTypedConstraint> joint = _space->GetJoint(pjoint);
        _space->Synchronize(KinBodyConstPtr(pjoint->GetParent()));
        std::vector<btScalar> vtorques(pTorques.size());
        std::copy(pTorques.begin(),pTorques.end(),vtorques.begin());
        btRigidBody& bodyA = joint->getRigidBodyA();
        btRigidBody& bodyB = joint->getRigidBodyB();
        switch(joint->getConstraintType()) {
        case HINGE_CONSTRAINT_TYPE: {
            boost::shared_ptr<btHingeConstraint> hingejoint = boost::dynamic_pointer_cast<btHingeConstraint>(joint);
            btTransform transA = bodyA.getCenterOfMassTransform();
            btTransform transB = bodyB.getCenterOfMassTransform();
            btTransform jointTransA = hingejoint->getAFrame();
            btTransform jointTransB = hingejoint->getBFrame();
            btVector3 torqueA = jointTransA.getBasis().transpose().getRow(2);
            btVector3 torqueB = jointTransB.getBasis().transpose().getRow(2);
            torqueA = vtorques.at(0)*torqueA;
            torqueB = -vtorques.at(0)*torqueB;
            bodyA.applyTorque(torqueA);
            bodyB.applyTorque(torqueB);
            break;
        }
        case SLIDER_CONSTRAINT_TYPE: {
            boost::shared_ptr<btSliderConstraint> slidejoint = boost::dynamic_pointer_cast<btSliderConstraint>(joint);
            btTransform frameInA = slidejoint->getFrameOffsetA();
            btTransform frameInB = slidejoint->getFrameOffsetB();
            btVector3 locA = bodyA.getCenterOfMassTransform().getOrigin();
            btVector3 locB = bodyB.getCenterOfMassTransform().getOrigin();
            btTransform transA = bodyA.getCenterOfMassTransform()*frameInA;
            btTransform transB = bodyB.getCenterOfMassTransform()*frameInB;
            btVector3 forceA = vtorques.at(0) * transA.getBasis().getColumn(0);
            btVector3 forceB = -vtorques.at(0) * transB.getBasis().getColumn(0);
            bodyA.applyForce(forceA, locA);
            bodyB.applyForce(forceB, locB);
            break;
        }
        default:
            RAVELOG_ERROR(str(boost::format("AddJointTorque joint type 0x%x not supported\n")%joint->getConstraintType()));
        }
        return true;
    }

    virtual bool SetBodyForce(KinBody::LinkPtr plink, const Vector& force, const Vector& position, bool bAdd)
    {
        boost::shared_ptr<btRigidBody> rigidbody = boost::dynamic_pointer_cast<btRigidBody>(_space->GetLinkBody(plink));
        btVector3 _Force(force[0], force[1], force[2]);
        btVector3 _Position(position[0], position[1], position[2]);
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        if( !bAdd ) {
            rigidbody->clearForces();
        }
        rigidbody->applyForce(_Force,_Position);
        return true;
    }
    virtual bool SetBodyTorque(KinBody::LinkPtr plink, const Vector& torque, bool bAdd)
    {
        btVector3 _Torque(torque[0], torque[1], torque[2]);
        boost::shared_ptr<btRigidBody> rigidbody = boost::dynamic_pointer_cast<btRigidBody>(_space->GetLinkBody(plink));
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        if( !bAdd ) {
            rigidbody->clearForces();
        }
        rigidbody->applyTorque(_Torque);
        return true;
    }

    virtual void SetGravity(const Vector& gravity)
    {
        _gravity = gravity;
        _dynamicsWorld->setGravity(btVector3(_gravity.x,_gravity.y,_gravity.z));
    }

    virtual Vector GetGravity()
    {
        return _gravity;
    }

    virtual void SimulateStep(dReal fTimeElapsed)
    {
        _space->Synchronize();
        int maxSubSteps = 0;
        _dynamicsWorld->applyGravity();
        _dynamicsWorld->stepSimulation(fTimeElapsed,maxSubSteps);

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            BulletSpace::KinBodyInfoPtr pinfo = GetPhysicsInfo(*itbody);
            FOREACH(itlink, pinfo->vlinks) {
                Transform t;
                btQuaternion btquat = (*itlink)->_rigidbody->getCenterOfMassTransform().getRotation();
                t.rot.x = btquat.w();
                t.rot.y = btquat.x();
                t.rot.z = btquat.y();
                t.rot.w = btquat.z();
                btVector3 bttrans = (*itlink)->_rigidbody->getCenterOfMassTransform().getOrigin();
                t.trans.x = bttrans.x();
                t.trans.y = bttrans.y();
                t.trans.z = bttrans.z();
                (*itlink)->plink->SetTransform(t*(*itlink)->tlocal.inverse());
            }
            pinfo->nLastStamp = (*itbody)->GetUpdateStamp();
        }
        _dynamicsWorld->clearForces();
    }

private:
    static BulletSpace::KinBodyInfoPtr GetPhysicsInfo(KinBodyConstPtr pbody)
    {
        return boost::dynamic_pointer_cast<BulletSpace::KinBodyInfo>(pbody->GetPhysicsData());
    }

    void _SyncCallback(BulletSpace::KinBodyInfoConstPtr pinfo)
    {
        // reset dynamics
        FOREACH(itlink, pinfo->vlinks) {
            if( !!(*itlink)->_rigidbody ) {
                (*itlink)->_rigidbody->setLinearVelocity(btVector3(0,0,0));
                (*itlink)->_rigidbody->setAngularVelocity(btVector3(0,0,0));
            }
        }
    }

    int _options;
    Vector _gravity;
    btScalar _global_friction;
    int _solver_iterations;
    btScalar _margin_depth;
    btScalar _linear_damping, _rotation_damping;
    btScalar _global_contact_force_mixing;
    btScalar _global_restitution;

    boost::shared_ptr<BulletSpace> _space;
    boost::shared_ptr<btDiscreteDynamicsWorld> _dynamicsWorld;
    boost::shared_ptr<btDefaultCollisionConfiguration> _collisionConfiguration;
    boost::shared_ptr<btBroadphaseInterface> _broadphase;
    boost::shared_ptr<btCollisionDispatcher> _dispatcher;
    boost::shared_ptr<btConstraintSolver> _solver;
    boost::shared_ptr<btOverlapFilterCallback> _filterCallback;

    std::list<EnvironmentBase::CollisionCallbackFn> _listcallbacks;
    CollisionReportPtr _report;
};


PhysicsEngineBasePtr CreateBulletPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PhysicsEngineBasePtr(new BulletPhysicsEngine(penv,sinput));
}
