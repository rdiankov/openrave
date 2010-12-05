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
#ifndef OPENRAVE_BULLET_SPACE
#define OPENRAVE_BULLET_SPACE

#include "plugindefs.h"

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
//#include <BulletCollision/Gimpact/btConcaveConcaveCollisionAlgorithm.h>

// groups bits for bullet
#define ENABLED_GROUP 1 // mask ENABLED_GROUP
#define DISABLED_GROUP 256 // mask 0

// manages a space of bullet objects
class BulletSpace : public boost::enable_shared_from_this<BulletSpace>
{
    inline boost::weak_ptr<BulletSpace> weak_space() { return shared_from_this(); }

public:
    // information about the kinematics of the body
    struct KinBodyInfo : public OpenRAVE::UserData
    {
        struct LINK
        {
            virtual ~LINK()
            {
                motionstate.reset();
                obj.reset();
                shape.reset();
                listchildren.clear();
                listmeshes.clear();
            }

            boost::shared_ptr<btDefaultMotionState> motionstate;
            boost::shared_ptr<btCollisionObject> obj;
            boost::shared_ptr<btCollisionShape> shape;
            list<boost::shared_ptr<btCollisionShape> > listchildren;
            list<boost::shared_ptr<btStridingMeshInterface> > listmeshes;

            KinBody::LinkPtr plink;            
            Transform tlocal; /// local offset transform to account for inertias not aligned to axes
        };
        
    KinBodyInfo(boost::shared_ptr<btCollisionWorld> world, bool bPhysics) : _world(world), _bPhysics(bPhysics) { nLastStamp = 0; }
        virtual ~KinBodyInfo() { Reset(); }
        
        void Reset()
        {
            FOREACH(itlink, vlinks) {
                if( _bPhysics )
                    ((btDynamicsWorld*)_world.get())->removeRigidBody((btRigidBody*)(*itlink)->obj.get());
                else
                    _world->removeCollisionObject((*itlink)->obj.get());
            }
            vlinks.resize(0);
            vjoints.resize(0);
            _geometrycallback.reset();
        }

        KinBodyPtr pbody; ///< body associated with this structure
        int nLastStamp;
        
        vector<boost::shared_ptr<LINK> > vlinks; ///< if body is disabled, then geom is static (it can't be connected to a joint!)
                                ///< the pointer to this Link is the userdata
        vector< boost::shared_ptr<btTypedConstraint> > vjoints;
        boost::shared_ptr<void> _geometrycallback;
        boost::weak_ptr<BulletSpace> _bulletspace;

    private:
        boost::shared_ptr<btCollisionWorld> _world;
        bool _bPhysics;
    };

    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;
    typedef boost::function<UserDataPtr(KinBodyConstPtr)> GetInfoFn;
    typedef boost::function<void(KinBodyInfoPtr)> SynchornizeCallbackFn;

 BulletSpace(EnvironmentBasePtr penv, const GetInfoFn& infofn, bool bPhysics) : _penv(penv), GetInfo(infofn), _bPhysics(bPhysics) {}
    virtual ~BulletSpace() {}
    
    bool InitEnvironment(boost::shared_ptr<btCollisionWorld>& world)
    {
        RAVELOG_VERBOSE("init bullet collision environment\n");

        _world = world;
        btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher*)_world->getDispatcher());
        //btConcaveConcaveCollisionAlgorithm::registerAlgorithm(_world->getDispatcher());

        return true;
    }

    void DestroyEnvironment()
    {
        RAVELOG_VERBOSE("destroying bullet collision environment\n");
        _world.reset();
    }
    
    UserDataPtr InitKinBody(KinBodyPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr()) {
        // create all ode bodies and joints
        if( !pinfo )
            pinfo.reset(new KinBodyInfo(_world,_bPhysics));
        pinfo->Reset();
        pinfo->pbody = pbody;
        pinfo->_bulletspace = weak_space();
        pinfo->vlinks.reserve(pbody->GetLinks().size());
        pinfo->vjoints.reserve(pbody->GetJoints().size());
    
        pinfo->vlinks.reserve(pbody->GetLinks().size());
        float fmargin=0.0004f;
        FOREACHC(itlink, pbody->GetLinks()) {
            boost::shared_ptr<KinBodyInfo::LINK> link(new KinBodyInfo::LINK());

            btCompoundShape* pshapeparent = new btCompoundShape();
            link->shape.reset(pshapeparent);
            pshapeparent->setMargin(fmargin); // need to set margin very small (we're not simulating anyway)
        
            // add all the correct geometry objects
            FOREACHC(itgeom, (*itlink)->GetGeometries()) {
                boost::shared_ptr<btCollisionShape> child;
                switch(itgeom->GetType()) {
                case KinBody::Link::GEOMPROPERTIES::GeomBox:
                    child.reset(new btBoxShape(GetBtVector(itgeom->GetBoxExtents())));
                    break;
                case KinBody::Link::GEOMPROPERTIES::GeomSphere:
                    child.reset(new btSphereShape(itgeom->GetSphereRadius()));
                    break;
                case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
                    // cylinder axis aligned to Y
                    child.reset(new btCylinderShapeZ(btVector3(itgeom->GetCylinderRadius(),itgeom->GetCylinderRadius(),itgeom->GetCylinderHeight()*0.5f)));
                    break;
                case KinBody::Link::GEOMPROPERTIES::GeomTrimesh: {
                    if( itgeom->GetCollisionMesh().indices.size() >= 3 ) {
                        btTriangleMesh* ptrimesh = new btTriangleMesh();

                        // for some reason adding indices makes everything crash
                        for(size_t i = 0; i < itgeom->GetCollisionMesh().indices.size(); i += 3)
                            ptrimesh->addTriangle(GetBtVector(itgeom->GetCollisionMesh().vertices[i]),
                                                  GetBtVector(itgeom->GetCollisionMesh().vertices[i+1]),
                                                  GetBtVector(itgeom->GetCollisionMesh().vertices[i+2]));

                        //child.reset(new btBvhTriangleMeshShape(ptrimesh, true, true)); // doesn't do tri-tri collisions!
                        btGImpactMeshShape* pgimpact = new btGImpactMeshShape(ptrimesh);
                        pgimpact->setMargin(fmargin); // need to set margin very small (we're not simulating anyway)
                        pgimpact->updateBound();
                        child.reset(pgimpact);
                        link->listmeshes.push_back(boost::shared_ptr<btStridingMeshInterface>(ptrimesh));
                    }
                    break;
                }
                default:
                    break;
                }

                if( !child ) {
                    RAVELOG_WARN("did not create geom type %d\n", itgeom->GetType());
                    continue;
                }

                link->listchildren.push_back(child);
                child->setMargin(fmargin); // need to set margin very small (we're not simulating anyway)
                pshapeparent->addChildShape(GetBtTransform(itgeom->GetTransform()), child.get());
            }

            if( _bPhysics ) {

                // set the mass and inertia and extract the eigenvectors of the tensor
                btTransform startTransform;
                startTransform.setOrigin(GetBtVector((*itlink)->GetInertia().trans));
                btVector3 localInertia;
                link->tlocal = Transform();
                BOOST_ASSERT(0); // need more work with setting inertia transform
        
                link->motionstate.reset(new btDefaultMotionState(startTransform));
                btRigidBody::btRigidBodyConstructionInfo rbInfo((*itlink)->GetMass(),link->motionstate.get(),pshapeparent,localInertia);

                link->obj.reset(new btRigidBody(rbInfo));
                // setCenterOfMassTransform - updates inertia
            }
            else {
                link->obj.reset(new btCollisionObject());
                link->obj->setCollisionShape(pshapeparent);
            }

            link->plink = *itlink;
            link->obj->setWorldTransform(GetBtTransform((*itlink)->GetTransform()*link->tlocal));
            link->obj->setUserPointer(&link->plink);
            link->obj->setCollisionFlags((*itlink)->IsStatic() ? btCollisionObject::CF_KINEMATIC_OBJECT : 0);
        
            if( _bPhysics && !(*itlink)->IsStatic() )
                ((btDynamicsWorld*)_world.get())->addRigidBody((btRigidBody*)link->obj.get());
            else
                _world->addCollisionObject(link->obj.get());

            link->obj->activate(true);
            pinfo->vlinks.push_back(link);
        }

        if( _bPhysics ) {
            pinfo->vjoints.reserve(pbody->GetJoints().size());

            FOREACHC(itjoint, pbody->GetJoints()) {
                RaveVector<dReal> anchor = (*itjoint)->GetAnchor();
                RaveVector<dReal> axis0 = (*itjoint)->GetAxis(0);
                RaveVector<dReal> axis1 = (*itjoint)->GetAxis(1);

                btRigidBody* body0 = NULL;
                if( ((*itjoint)->GetFirstAttached() != NULL && !(*itjoint)->GetFirstAttached()->IsStatic()) )
                    body0 = (btRigidBody*)pinfo->vlinks[(*itjoint)->GetFirstAttached()->GetIndex()]->obj.get();
            
                btRigidBody* body1 = NULL;
                if( ((*itjoint)->GetSecondAttached() != NULL && !(*itjoint)->GetSecondAttached()->IsStatic()) )
                    body1 = (btRigidBody*)pinfo->vlinks[(*itjoint)->GetSecondAttached()->GetIndex()]->obj.get();
            
                if( body0 == NULL || body1 == NULL ) {
                    RAVELOG_ERROR(str(boost::format("joint %s needs to be attached to two bodies!\n")%(*itjoint)->GetName()));
                    continue;
                }

                Transform t0inv = (*itjoint)->GetFirstAttached()->GetTransform().inverse();
                Transform t1inv = (*itjoint)->GetSecondAttached()->GetTransform().inverse();
                boost::shared_ptr<btTypedConstraint> joint;

                switch((*itjoint)->GetType()) {
                case KinBody::Joint::JointHinge: {
                    btVector3 pivotInA = GetBtVector(t0inv * anchor);
                    btVector3 pivotInB = GetBtVector(t1inv * anchor);
                    btVector3 axisInA = GetBtVector(t0inv.rotate(axis0));
                    btVector3 axisInB = GetBtVector(t1inv.rotate(axis1));
                    joint.reset(new btHingeConstraint(*body0, *body1, pivotInA, pivotInB, axisInA, axisInB));
                    break;
                }
                case KinBody::Joint::JointSlider: {
                    TransformMatrix tslider;
                    Vector vup(1,1,1);
                    vup -= axis0*dot3(axis0,vup);
                    if( vup.lengthsqr3() < 0.05 ) {
                        vup = Vector(0,1,0);
                        vup -= axis0*dot3(axis0,vup);
                    }
                    vup.normalize3();
                    Vector vdir = axis0.cross(vup);
                
                    tslider.m[0] = axis0.x; tslider.m[1] = vup.x; tslider.m[2] = vdir.x;
                    tslider.m[4] = axis0.y; tslider.m[5] = vup.y; tslider.m[6] = vdir.y;
                    tslider.m[8] = axis0.z; tslider.m[9] = vup.z; tslider.m[10] = vdir.z;
                
                    btTransform frameInA = GetBtTransform(tslider*t0inv);
                    btTransform frameInB = GetBtTransform(tslider*t1inv);
                    joint.reset(new btSliderConstraint(*body0, *body1, frameInA, frameInB, true));
                    break;
                }
                case KinBody::Joint::JointUniversal:
                    RAVELOG_ERROR("universal joint not supported by bullet\n");
                    break;
                case KinBody::Joint::JointHinge2:
                    RAVELOG_ERROR("hinge2 joint not supported by bullet\n");
                    break;
                default:
                    RAVELOG_ERROR("unknown joint type %d\n", (*itjoint)->GetType());
                    break;
                }
            
                if( !!joint ) {
                    ((btDynamicsWorld*)_world.get())->addConstraint(joint.get(), true);
                    pinfo->vjoints.push_back(joint);
                }
            }
        }

        pinfo->_geometrycallback = pbody->RegisterChangeCallback(KinBody::Prop_LinkGeometry, boost::bind(&BulletSpace::GeometryChangedCallback,boost::bind(&sptr_from<BulletSpace>, weak_space()),KinBodyWeakPtr(pbody)));

        Synchronize(pinfo);
        return pinfo;
    }

    bool Enable(KinBodyConstPtr pbody, bool bEnable) { return true; }
    bool EnableLink(KinBody::LinkConstPtr pbody, bool bEnable) { return true; }

    void Synchronize()
    {
        vector<KinBodyPtr> vbodies;
        _penv->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<KinBodyInfo>(GetInfo(*itbody));
            BOOST_ASSERT( pinfo->pbody == *itbody );
            if( pinfo->nLastStamp != (*itbody)->GetUpdateStamp() )
                Synchronize(pinfo);
        }
    }

    void Synchronize(KinBodyConstPtr pbody)
    {
        KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<KinBodyInfo>(GetInfo(pbody));
        BOOST_ASSERT( pinfo->pbody == pbody );
        if( pinfo->nLastStamp != pbody->GetUpdateStamp() )
            Synchronize(pinfo);
    }

    boost::shared_ptr<btCollisionObject> GetLinkObj(KinBody::LinkConstPtr plink)
    {
        KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<KinBodyInfo>(GetInfo(plink->GetParent()));
        BOOST_ASSERT(pinfo->pbody == plink->GetParent() );
        BOOST_ASSERT( plink->GetIndex() >= 0 && plink->GetIndex() < (int)pinfo->vlinks.size());
        return pinfo->vlinks[plink->GetIndex()]->obj;
    }

    boost::shared_ptr<btTypedConstraint> GetJoint(KinBody::JointConstPtr pjoint)
    {
        KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<KinBodyInfo>(GetInfo(pjoint->GetParent()));
        BOOST_ASSERT(pinfo->pbody == pjoint->GetParent() );
        BOOST_ASSERT( pjoint->GetParent()->GetJointFromDOFIndex(pjoint->GetDOFIndex()) == pjoint );
        BOOST_ASSERT( pjoint->GetJointIndex() >= 0 && pjoint->GetJointIndex() < (int)pinfo->vjoints.size());
        return pinfo->vjoints[pjoint->GetJointIndex()];
    }

    void SetSynchornizationCallback(const SynchornizeCallbackFn& synccallback) { _synccallback = synccallback; }

    static inline btTransform GetBtTransform(const Transform& t)
    {
        return btTransform(btQuaternion(t.rot.y,t.rot.z,t.rot.w,t.rot.x),GetBtVector(t.trans));
    }

    static inline btVector3 GetBtVector(const Vector& v)
    {
        return btVector3(v.x,v.y,v.z);
    }

private:
    
    void Synchronize(KinBodyInfoPtr pinfo)
    {
        vector<Transform> vtrans;
        pinfo->pbody->GetBodyTransformations(vtrans);
        pinfo->nLastStamp = pinfo->pbody->GetUpdateStamp();
        BOOST_ASSERT( vtrans.size() == pinfo->vlinks.size() );
        for(size_t i = 0; i < vtrans.size(); ++i)
            pinfo->vlinks[i]->obj->getWorldTransform() = GetBtTransform(vtrans[i]*pinfo->vlinks[i]->tlocal);

        if( !!_synccallback )
            _synccallback(pinfo);
    }

    virtual void GeometryChangedCallback(KinBodyWeakPtr _pbody)
    {
        EnvironmentMutex::scoped_lock lock(_penv->GetMutex());
        KinBodyPtr pbody(_pbody);
        KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<KinBodyInfo>(GetInfo(pbody));
        if( !pinfo ) {
            return;   
        }     
        BOOST_ASSERT(boost::shared_ptr<BulletSpace>(pinfo->_bulletspace) == shared_from_this());
        BOOST_ASSERT(pinfo->pbody==pbody);
        InitKinBody(pbody,pinfo);
    }

    EnvironmentBasePtr _penv;
    GetInfoFn GetInfo;
    boost::shared_ptr<btCollisionWorld> _world;
    SynchornizeCallbackFn _synccallback;
    bool _bPhysics;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(BulletSpace)
BOOST_TYPEOF_REGISTER_TYPE(BulletSpace::KinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(BulletSpace::KinBodyInfo::LINK)
BOOST_TYPEOF_REGISTER_TYPE(dJointID)
#endif

#endif
