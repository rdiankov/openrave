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
#include "plugindefs.h"

#include "bulletspace.h"
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
//#include <BulletCollision/Gimpact/btConcaveConcaveCollisionAlgorithm.h>

BulletSpace::KINBODYINFO::KINBODYINFO()
{
    nLastStamp = 0;
}

BulletSpace::KINBODYINFO::~KINBODYINFO()
{
}

BulletSpace::BulletSpace(EnvironmentBase* penv, GetInfoFn infofn, bool bPhysics) : _penv(penv), _bPhysics(bPhysics)
{
    _synccallback = NULL;
    _userdata = NULL;
    GetInfo = infofn;
}

BulletSpace::~BulletSpace()
{
}

bool BulletSpace::InitEnvironment(boost::shared_ptr<btCollisionWorld>& world)
{
    RAVELOG_DEBUGA("creating bullet collision\n");

    _world = world;
    btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher*)_world->getDispatcher());
    //btConcaveConcaveCollisionAlgorithm::registerAlgorithm(_world->getDispatcher());

    return true;
}

void BulletSpace::DestroyEnvironment()
{
    RAVELOG_DEBUGA("destroying bullet collision\n");
    _world.reset();
}

void* BulletSpace::InitKinBody(KinBody* pbody)
{
    // create all ode bodies and joints
    KINBODYINFO* pinfo = new KINBODYINFO();
    pinfo->pbody = pbody;
    pinfo->vlinks.reserve(pbody->GetLinks().size());
    pinfo->vjoints.reserve(pbody->GetJoints().size());
    
    pinfo->vlinks.reserve(pbody->GetLinks().size());

    FOREACHC(itlink, pbody->GetLinks()) {

        KINBODYINFO::LINK link;

        btCompoundShape* pshapeparent = new btCompoundShape();
        link.shape.reset(pshapeparent);
        pshapeparent->setMargin(0.0004f); // need to set margin very small (we're not simulating anyway)
        
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
                    pgimpact->setMargin(0.0004f); // need to set margin very small (we're not simulating anyway)
                    pgimpact->updateBound();
                    child.reset(pgimpact);
                    link.listmeshes.push_back(boost::shared_ptr<btStridingMeshInterface>(ptrimesh));
                }
                break;
            }
            default:
                break;
            }

            if( !child ) {
                RAVELOG_WARNA("did not create geom type %d\n", itgeom->GetType());
                continue;
            }

            link.listchildren.push_back(child);
            child->setMargin(0.0004f); // need to set margin very small (we're not simulating anyway)
            pshapeparent->addChildShape(GetBtTransform(itgeom->GetTransform()), child.get());
        }

        if( _bPhysics ) {

            // set the mass and inertia and extract the eigenvectors of the tensor
            btTransform startTransform;
            startTransform.setOrigin(GetBtVector((*itlink)->GetInertia().trans));
            btVector3 localInertia;
            link.tlocal = Transform();
            assert(0); // need more work with setting inertia transform
        
            link.motionstate.reset(new btDefaultMotionState(startTransform));
            btRigidBody::btRigidBodyConstructionInfo rbInfo((*itlink)->GetMass(),link.motionstate.get(),pshapeparent,localInertia);

            link.obj.reset(new btRigidBody(rbInfo));
            // setCenterOfMassTransform - updates inertia
        }
        else {
            link.obj.reset(new btCollisionObject());
            link.obj->setCollisionShape(pshapeparent);
        }

        link.obj->setWorldTransform(GetBtTransform((*itlink)->GetTransform()*link.tlocal));
        link.obj->setUserPointer(*itlink);
        link.obj->setCollisionFlags((*itlink)->IsStatic() ? btCollisionObject::CF_KINEMATIC_OBJECT : 0);
        
        if( _bPhysics && !(*itlink)->IsStatic() )
            ((btDynamicsWorld*)_world.get())->addRigidBody((btRigidBody*)link.obj.get());
        else
            _world->addCollisionObject(link.obj.get());

        link.obj->activate(true);
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
                body0 = (btRigidBody*)pinfo->vlinks[(*itjoint)->GetFirstAttached()->GetIndex()].obj.get();
            
            btRigidBody* body1 = NULL;
            if( ((*itjoint)->GetSecondAttached() != NULL && !(*itjoint)->GetSecondAttached()->IsStatic()) )
                body1 = (btRigidBody*)pinfo->vlinks[(*itjoint)->GetSecondAttached()->GetIndex()].obj.get();
            
            if( body0 == NULL || body1 == NULL ) {
                RAVELOG_ERRORA("joint %S needs to be attached to two bodies!\n", (*itjoint)->GetName());
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
                Vector vdir; vdir.Cross(axis0,vup);
                
                tslider.m[0] = axis0.x; tslider.m[1] = vup.x; tslider.m[2] = vdir.x;
                tslider.m[4] = axis0.y; tslider.m[5] = vup.y; tslider.m[6] = vdir.y;
                tslider.m[8] = axis0.z; tslider.m[9] = vup.z; tslider.m[10] = vdir.z;
                
                btTransform frameInA = GetBtTransform(tslider*t0inv);
                btTransform frameInB = GetBtTransform(tslider*t1inv);
                joint.reset(new btSliderConstraint(*body0, *body1, frameInA, frameInB, true));
                break;
            }
            case KinBody::Joint::JointUniversal:
                RAVELOG_ERRORA("universal joint not supported by bullet\n");
                break;
            case KinBody::Joint::JointHinge2:
                RAVELOG_ERRORA("hinge2 joint not supported by bullet\n");
                break;
            default:
                RAVELOG_ERRORA("unknown joint type %d\n", (*itjoint)->GetType());
                break;
            }
            
            if( !!joint ) {
                ((btDynamicsWorld*)_world.get())->addConstraint(joint.get(), true);
                pinfo->vjoints.push_back(joint);
            }
        }
    }

    Synchronize(pinfo);
    return pinfo;
}

bool BulletSpace::DestroyKinBody(KinBody* pbody)
{
    assert( pbody != NULL );
    KINBODYINFO* pinfo = (KINBODYINFO*)GetInfo(pbody);
    if( pinfo == NULL )
        return true;
    assert( pinfo != NULL && pinfo->pbody == pbody );

    FOREACH(itlink, pinfo->vlinks) {
        if( _bPhysics )
            ((btDynamicsWorld*)_world.get())->removeRigidBody((btRigidBody*)itlink->obj.get());
        else
            _world->removeCollisionObject(itlink->obj.get());
    }

    delete pinfo;
    return true;
}

bool BulletSpace::Enable(const KinBody* pbody, bool bEnable)
{
//    assert( pbody != NULL );
//    KINBODYINFO* pinfo = (KINBODYINFO*)GetInfo(pbody);
//    assert( pinfo != NULL && pinfo->pbody == pbody );
//    FOREACH(it, pinfo->vlinks)
//        it->Enable(bEnable);
    return true;
}

bool BulletSpace::EnableLink(const KinBody::Link* plink, bool bEnable)
{
//    assert( plink != NULL && plink->GetParent() != NULL );
//    KINBODYINFO* pinfo = (KINBODYINFO*)GetInfo(plink->GetParent());
//    assert( pinfo != NULL && pinfo->pbody == plink->GetParent() );
//    assert( plink->GetIndex() >= 0 && plink->GetIndex() < (int)pinfo->vlinks.size());
//    pinfo->vlinks[plink->GetIndex()].Enable(bEnable);
    return true;
}

void BulletSpace::SetSynchornizationCallback(SynchornizeCallbackFn synccallback, void* userdata)
{
    _synccallback = synccallback;
    _userdata = userdata;
}

void BulletSpace::Synchronize()
{
    FOREACHC(itbody, _penv->GetBodies()) {
        KINBODYINFO* pinfo = (KINBODYINFO*)GetInfo(*itbody);
        assert( pinfo != NULL );
        assert( pinfo->pbody == *itbody );
        if( pinfo->nLastStamp != (*itbody)->GetUpdateStamp() )
            Synchronize(pinfo);
    }
}

void BulletSpace::Synchronize(const KinBody* pbody)
{
    KINBODYINFO* pinfo = (KINBODYINFO*)GetInfo(pbody);
    assert( pinfo != NULL );
    assert( pinfo->pbody == pbody );
    if( pinfo->nLastStamp != pbody->GetUpdateStamp() )
        Synchronize(pinfo);
}

void BulletSpace::Synchronize(KINBODYINFO* pinfo)
{
    vector<Transform> vtrans;
    pinfo->pbody->GetBodyTransformations(vtrans);
    pinfo->nLastStamp = pinfo->pbody->GetUpdateStamp();
    assert( vtrans.size() == pinfo->vlinks.size() );
    for(size_t i = 0; i < vtrans.size(); ++i)
        pinfo->vlinks[i].obj->getWorldTransform() = GetBtTransform(vtrans[i]*pinfo->vlinks[i].tlocal);

    if( _synccallback != NULL )
        _synccallback(_userdata, pinfo);
}

btCollisionObject* BulletSpace::GetLinkObj(const KinBody::Link* plink)
{
    assert( plink != NULL && plink->GetParent() != NULL );
    KINBODYINFO* pinfo = (KINBODYINFO*)GetInfo(plink->GetParent());
    assert( pinfo != NULL && pinfo->pbody == plink->GetParent() );
    assert( plink->GetIndex() >= 0 && plink->GetIndex() < (int)pinfo->vlinks.size());
    return pinfo->vlinks[plink->GetIndex()].obj.get();
}

btTypedConstraint* BulletSpace::GetJoint(const KinBody::Joint* pjoint)
{
    assert( pjoint != NULL && pjoint->GetParent() != NULL );
    KINBODYINFO* pinfo = (KINBODYINFO*)GetInfo(pjoint->GetParent());
    assert( pinfo != NULL && pinfo->pbody == pjoint->GetParent() );
    assert( pjoint->GetParent()->GetJointFromDOFIndex(pjoint->GetDOFIndex()) == pjoint );
    assert( pjoint->GetJointIndex() >= 0 && pjoint->GetJointIndex() < (int)pinfo->vjoints.size());
    return pinfo->vjoints[pjoint->GetJointIndex()].get();
}
