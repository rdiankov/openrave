// -*- coding: utf-8 -*-
// Copyright (C) 2015 James Taylor <jrt@gwu.edu>, Rosen Diankov <rosen.diankov@gmail.com>
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

#ifndef OPENRAVE_MOBY_SPACE
#define OPENRAVE_MOBY_SPACE

#include "plugindefs.h"

#include <Moby/TimeSteppingSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/RigidBody.h>
#include <Moby/Joint.h>
#include <Moby/TriangleMeshPrimitive.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/Pose3d.h>

// manages a space of Moby objects
class MobySpace : public boost::enable_shared_from_this<MobySpace>
{
    inline boost::weak_ptr<MobySpace> weak_space() {
        return shared_from_this();
    }

public:

    // information about the kinematics of the body
    class KinBodyInfo : public UserData
    {
public:
        class LINK : public Moby::RigidBody
        {
public:
            virtual ~LINK() {
            }

            virtual void GetWorldTransform(Ravelin::Transform3d& comWorldTrans) const
            {
                comWorldTrans = GetRavelinTransform( plink->GetTransform()*tlocal );
            }

            virtual void SetWorldTransform(const Ravelin::Transform3d& comWorldTrans)
            {
                plink->SetTransform(GetTransform(comWorldTrans)*tlocal.inverse());
            }

            Moby::RigidBodyPtr _body;
            KinBody::LinkPtr plink;
            Transform tlocal;     /// local offset transform to account for inertias not aligned to axes
        };

        KinBodyInfo(boost::shared_ptr<Moby::Simulator> world, bool bPhysics) : _world(world), _bPhysics(bPhysics) {
            nLastStamp = 0;
        }


        virtual ~KinBodyInfo() {
            Reset();
        }

        void Reset()
        {
            FOREACH(itlink, vlinks) {
                _world->remove_dynamic_body((*itlink)->_body);
            }
        }

        KinBodyPtr pbody;     ///< body associated with this structure
        int nLastStamp;
        std::vector<boost::shared_ptr<LINK> > vlinks;     
        UserDataPtr _geometrycallback;
        boost::weak_ptr<MobySpace> _mobyspace;

private:
        boost::shared_ptr<Moby::Simulator> _world;
        bool _bPhysics;
    };

    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;
    typedef boost::function<KinBodyInfoPtr(KinBodyConstPtr)> GetInfoFn;
    typedef boost::function<void (KinBodyInfoPtr)> SynchronizeCallbackFn;

    MobySpace(EnvironmentBasePtr penv, const GetInfoFn& infofn, bool bPhysics) : _penv(penv), GetInfo(infofn), _bPhysics(bPhysics) {
    }
    virtual ~MobySpace() {
    }

    bool InitEnvironment(boost::shared_ptr<Moby::TimeSteppingSimulator> world)
    {
 
        _world = world;

        return true;
    }

    void DestroyEnvironment()
    {
        _world.reset();
    }

    KinBodyInfoPtr InitKinBody(KinBodyPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr(), double fmargin=0.0005) 
    {
	KinBody::KinBodyStateSaver saver(pbody);
	pbody->SetTransform(Transform());
	std::vector<dReal> vzeros(pbody->GetDOF(), 0);
	pbody->SetDOFValues(vzeros);
	
        if( !pinfo ) {
            pinfo.reset(new KinBodyInfo(_world,_bPhysics));
        }
        pinfo->Reset();
        pinfo->pbody = pbody;
        pinfo->_mobyspace = weak_space();
        pinfo->vlinks.reserve(pbody->GetLinks().size());

        //Create an articulated body
        Moby::RCArticulatedBodyPtr mobody( new Moby::RCArticulatedBody() );

        FOREACHC(itlink, pbody->GetLinks()) {
            boost::shared_ptr<KinBodyInfo::LINK> link(new KinBodyInfo::LINK());

            // create a Moby::RigidBody
            // NOTE: the mass reference frame may not be centered.  
            //   Do the Moby primitives generally assume a centered frame?
            AABB bb = (*itlink)->ComputeLocalAABB();
            Moby::PrimitivePtr prim(new Moby::BoxPrimitive(bb.extents.x*2,bb.extents.y*2,bb.extents.z*2));
            Ravelin::Vector3d com(bb.pos.x,bb.pos.y,bb.pos.z);

            Moby::RigidBodyPtr child(new Moby::RigidBody());
            prim->set_mass((*itlink)->GetMass());
            
            child->set_visualization_data(prim->create_visualization());
            child->set_inertia(prim->get_inertia());
            child->set_enabled(true);

            

            // TODO: set contact parameters?
            
            // NOTE: mass is a link component and numerous geometries may
            //   be attached to the link.  Per Rosen, the link geometry
            //   can instead be approximated with a box instead rather than
            //   trying to match the whole set of geometric primitives 
            //   so the below should be reconsidered 

            /*
            FOREACHC(itgeom, (*itlink)->GetGeometries()) {
                KinBody::Link::GeometryPtr geom = *itgeom;
                Moby::RigidBodyPtr child(new Moby::RigidBody());
                Moby::PrimitivePtr prim;
                //KinBody::GeometryInfo info = 
                switch(geom->GetType()) {
                case GT_None:
                    break;
                case GT_Box: {
                    //child.reset(new btBoxShape(GetBtVector(geom->GetBoxExtents())));
                    Ravelin::Vector3d len = GetRavelinVector(geom->GetBoxExtents())*2;
                    prim = Moby::PrimitivePtr(new Moby::BoxPrimitive(len.x(),len.y(),len.z()));
                    prim->set_mass(geom->GetMass());

                    child->set_visualization_data(prim->create_visualization());
                    child->set_inertia(prim->get_inertia());
                    child->set_enabled(true);
                    // TODO: set contact parameters?
                    }
                    break;
                case GT_Sphere:
                    //child.reset(new btSphereShape(geom->GetSphereRadius()));
                    prim = Moby::PrimitivePtr(new Moby::SpherePrimitive(geom->GetSphereRadius()));
                    prim->set_mass(geom->GetMass());

                    child->set_visualization_data(prim->create_visualization());
                    child->set_inertia(prim->get_inertia());
                    child->set_enabled(true);
                    // TODO: set contact parameters?
                    break;
                case GT_Cylinder:
                    // cylinder axis aligned to Y
                    //child.reset(new btCylinderShapeZ(btVector3(geom->GetCylinderRadius(),geom->GetCylinderRadius(),geom->GetCylinderHeight()*0.5f)));
                    prim = Moby::PrimitivePtr(new Moby::CylinderPrimitive(geom->GetCylinderRadius(),geom->GetCylinderHeight()));
                    prim->set_mass(geom->GetMass());

                    child->set_visualization_data(prim->create_visualization());
                    child->set_inertia(prim->get_inertia());
                    child->set_enabled(true);
                    // TODO: set contact parameters?
                    break;
                case GT_TriMesh: {
                    prim = Moby::PrimitivePtr(new Moby::TriangleMeshPrimitive());
                    // TODO: build primitive
                    // TODO: add the primitive to the object
                    }
                    break;
                default:
                    break;
                }
            }
            */
        }
    }

    void Synchronize()
    {
        vector<KinBodyPtr> vbodies;
        _penv->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            KinBodyInfoPtr pinfo = GetInfo(*itbody);
            BOOST_ASSERT( pinfo->pbody == *itbody );
            if( pinfo->nLastStamp != (*itbody)->GetUpdateStamp() ) {
                _Synchronize(pinfo);
            }
        }
    }

    void Synchronize(KinBodyConstPtr pbody)
    {
        KinBodyInfoPtr pinfo = GetInfo(pbody);
        BOOST_ASSERT( pinfo->pbody == pbody );
        if( pinfo->nLastStamp != pbody->GetUpdateStamp() ) {
            _Synchronize(pinfo);
        }
    }

    Moby::RigidBodyPtr GetLinkBody(KinBody::LinkConstPtr plink)
    {
        KinBodyInfoPtr pinfo = GetInfo(plink->GetParent());
        BOOST_ASSERT(pinfo->pbody == plink->GetParent() );
        return pinfo->vlinks.at(plink->GetIndex());
    }

    Moby::JointPtr GetJoint(KinBody::JointConstPtr pjoint)
    {
        KinBodyInfoPtr pinfo = GetInfo(pjoint->GetParent());
        BOOST_ASSERT(pinfo->pbody == pjoint->GetParent() );

        // TODO: How are we mapping joints?

        // return joint;
    }

    void SetSynchronizationCallback(const SynchronizeCallbackFn &synccallback) {
        _synccallback = synccallback;
    }

    static inline Transform GetTransform(const Ravelin::Transform3d &t)
    {
        return Transform(Vector(t.q.w, t.q.x, t.q.y, t.q.z), Vector(t.x.x(), t.x.y(), t.x.z()));
    }

    static inline Ravelin::Transform3d GetRavelinTransform(const Transform &t)
    {
        // TODO assertion sanity check
        OPENRAVE_ASSERT_OP(RaveFabs(t.rot.lengthsqr4()-1),<=,0.01);

        // TODO: check with Rosen on the Quaternion parameter ordering
        //   as modeled from bulletspace.h.  Reason for this mapping 
        //   unclear.

        return(Ravelin::Transform3d(Ravelin::Quatd(t.rot.y, t.rot.z, t.rot.w, t.rot.x), GetRavelinOrigin(t.trans)));
    }

    static inline Ravelin::Origin3d GetRavelinOrigin(const Vector &v)
    {
        return Ravelin::Origin3d(v.x, v.y, v.z);
    }

    bool IsInitialized() {
        return !!_world;
    }


private:

    void _Synchronize(KinBodyInfoPtr pinfo)
    {
/*
        vector<Transform> vtrans;
        std::vector<int> dofbranches;
        pinfo->pbody->GetLinkTransformations(vtrans,dofbranches);
        pinfo->nLastStamp = pinfo->pbody->GetUpdateStamp();
        BOOST_ASSERT( vtrans.size() == pinfo->vlinks.size() );
        for(size_t i = 0; i < vtrans.size(); ++i) {
            pinfo->vlinks[i]->obj->getWorldTransform() = GetBtTransform(vtrans[i]*pinfo->vlinks[i]->tlocal);
        }
        if( !!_synccallback ) {
            _synccallback(pinfo);
        }
*/
    }

/*
    virtual void GeometryChangedCallback(KinBodyWeakPtr _pbody)
    {
        EnvironmentMutex::scoped_lock lock(_penv->GetMutex());
        KinBodyPtr pbody(_pbody);
        KinBodyInfoPtr pinfo = GetInfo(pbody);
        if( !pinfo ) {
            return;
        }
        BOOST_ASSERT(boost::shared_ptr<MobySpace>(pinfo->_mobyspace) == shared_from_this());
        BOOST_ASSERT(pinfo->pbody==pbody);
        InitKinBody(pbody,pinfo);
    }

    EnvironmentBasePtr _penv;
    GetInfoFn GetInfo;
    boost::shared_ptr<btCollisionWorld> _world;
    boost::shared_ptr<btDiscreteDynamicsWorld> _worlddynamics;
    SynchronizeCallbackFn _synccallback;
    bool _bPhysics;
*/

private:
    EnvironmentBasePtr _penv;
    GetInfoFn GetInfo;
    boost::shared_ptr<Moby::TimeSteppingSimulator> _world;
    SynchronizeCallbackFn _synccallback;
    bool _bPhysics;

};
/*
static KinBody::LinkPtr GetLinkFromCollision(const btCollisionObject* co) {
    BOOST_ASSERT(co != NULL);
    return static_cast<MobySpace::KinBodyInfo::LINK*>(co->getUserPointer())->plink;
}

static KinBody::LinkPtr GetLinkFromProxy(btBroadphaseProxy* proxy) {
    return GetLinkFromCollision(static_cast<btCollisionObject*>(proxy->m_clientObject));
}
*/
/*
class OpenRAVEFilterCallback : public btOverlapFilterCallback
{
public:
    virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const = 0;
    virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
    {
        BOOST_ASSERT( static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL );
        BOOST_ASSERT( static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL );
        KinBody::LinkPtr plink0 = GetLinkFromProxy(proxy0);
        KinBody::LinkPtr plink1 = GetLinkFromProxy(proxy1);
        if( !plink0->IsEnabled() || !plink1->IsEnabled() ) {
            return false;
        }
        return CheckLinks(plink0,plink1);
    }
};
*/
#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(MobySpace)
BOOST_TYPEOF_REGISTER_TYPE(MobySpace::KinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(MobySpace::KinBodyInfo::LINK)
BOOST_TYPEOF_REGISTER_TYPE(dJointID)
#endif

#endif
