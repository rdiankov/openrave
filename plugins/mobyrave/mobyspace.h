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
#include <Ravelin/Pose3d.h>

#include <Moby/RevoluteJoint.h>
#include <Moby/PrismaticJoint.h>
#include <Moby/UniversalJoint.h>
#include <Moby/SphericalJoint.h>
#include <Moby/GravityForce.h>

typedef void (*ControllerCallbackFn)(boost::shared_ptr<Moby::DynamicBody>,double,void*);

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
            virtual ~LINK() 
            {

            }

            virtual void GetWorldPose(Ravelin::Pose3d& comWorldTrans) const
            {
                comWorldTrans = GetRavelinPose( plink->GetTransform()*tlocal );
            }

            virtual void SetWorldPose(const Ravelin::Pose3d& comWorldTrans)
            {
                plink->SetTransform(GetTransform(comWorldTrans)*tlocal.inverse());
            }

            Moby::PrimitivePtr _primitive;  // for modeling the inertia of the link
            KinBody::LinkPtr plink;         // link back to openrave correspondent
            Transform tlocal;               // center of mass frame
        };

        KinBodyInfo(boost::shared_ptr<Moby::Simulator> world, bool bPhysics) : _world(world), _bPhysics(bPhysics) 
        {
            nLastStamp = 0;
        }

        virtual ~KinBodyInfo() 
        {
            Reset();
        }

        void Reset()
        {
            FOREACH(itlink, vlinks) {
                _world->remove_dynamic_body((*itlink));
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

    MobySpace(EnvironmentBasePtr penv, const GetInfoFn& infofn, bool bPhysics) : _penv(penv), GetInfo(infofn), _bPhysics(bPhysics) 
    {

    }

    virtual ~MobySpace() 
    {

    }

    bool InitEnvironment(boost::shared_ptr<Moby::Simulator> world)
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

        if( !pinfo ) 
        {
            pinfo.reset(new KinBodyInfo(_world,_bPhysics));
        }
        pinfo->Reset();
        pinfo->pbody = pbody;
        pinfo->_mobyspace = weak_space();
        pinfo->vlinks.reserve(pbody->GetLinks().size());

        if(pbody->GetLinks().size() == 1) 
        {
            // Note: this branch implies that there is only one link
            FOREACHC(itlink, pbody->GetLinks()) 
            {
                boost::shared_ptr<KinBodyInfo::LINK> link(new KinBodyInfo::LINK());

                // compute a box to approximate link inertial properties
                AABB bb = (*itlink)->ComputeLocalAABB();
                link->_primitive = Moby::PrimitivePtr(new Moby::BoxPrimitive(bb.extents.x*2,bb.extents.y*2,bb.extents.z*2));
                link->_primitive->set_mass((*itlink)->GetMass());

                // assign link parameters
                link->id = (*itlink)->GetName();                      // identity  
                link->set_inertia(link->_primitive->get_inertia());   // inertia
                link->set_enabled(true);                              // enable physics  
                link->get_recurrent_forces().push_back(gravity);      // s.t. gravity

                // assign transforms (Note: maintain the order of this section)
                link->tlocal = (*itlink)->GetLocalMassFrame();        // com frame transform
                link->set_pose(GetRavelinPose((*itlink)->GetTransform()*link->tlocal)); // pose

                // check for a static link
                if( (*itlink)->IsStatic() ) 
                {
                    link->set_enabled(false);          // disable to fix to the world         
                }

                // TODO: set contact parameters

                // add the link reference to all relevant containers               
                link->plink = *itlink;             // ref back to openrave link
                pinfo->vlinks.push_back(link);     // ref for link indexing 
   
                _world->add_dynamic_body(link);    // add the body to the world
            }
        }
        else if(pbody->GetLinks().size() > 1)
        {
            // RCArticulated body at the root of the hierarchy
            Moby::RCArticulatedBodyPtr morcab(new Moby::RCArticulatedBody());

            // initialize the articulated body parameters 
            morcab->id = pbody->GetName();
            morcab->algorithm_type = Moby::RCArticulatedBody::eCRB;
            morcab->set_floating_base(true);   // assume floating base until static link found
    
            // make allocations for links and joints
            std::vector<Moby::RigidBodyPtr> molinks;   // for adding the links to the articulated body
            molinks.reserve(pbody->GetLinks().size());
    
            // ?: Do PassiveJoints -> fixed joints/welds
            vector<KinBody::JointPtr> vbodyjoints;
            vbodyjoints.reserve(pbody->GetJoints().size()+pbody->GetPassiveJoints().size());
            vbodyjoints.insert(vbodyjoints.end(),pbody->GetJoints().begin(),pbody->GetJoints().end());
            vbodyjoints.insert(vbodyjoints.end(),pbody->GetPassiveJoints().begin(),pbody->GetPassiveJoints().end());
            std::vector<Moby::JointPtr> mojoints;      // for adding the joints to the articulated body
            mojoints.reserve(pbody->GetJoints().size()+pbody->GetPassiveJoints().size());

            // iterate over the set of links in the OpenRave environment
            // create a link for each and insert them into the Moby articulated body
            FOREACHC(itlink, pbody->GetLinks()) 
            {
                boost::shared_ptr<KinBodyInfo::LINK> link(new KinBodyInfo::LINK());

                // compute a box to approximate link inertial properties
                AABB bb = (*itlink)->ComputeLocalAABB();
                link->_primitive = Moby::PrimitivePtr(new Moby::BoxPrimitive(bb.extents.x*2,bb.extents.y*2,bb.extents.z*2));
                link->_primitive->set_mass((*itlink)->GetMass());

                // assign link parameters
                link->id = (*itlink)->GetName();                      // identity  
                link->set_inertia(link->_primitive->get_inertia());   // inertia
                link->set_enabled(true);                              // physics enabled  
                link->get_recurrent_forces().push_back(gravity);      // s.t. gravity

                // assign transforms (Note: maintain the order of this section)
                link->tlocal = (*itlink)->GetLocalMassFrame();        // com frame transform
                link->set_pose(GetRavelinPose((*itlink)->GetTransform()*link->tlocal)); // pose

                // check for a static link
                if( (*itlink)->IsStatic() ) {
                    link->set_enabled(false);          // disable to fix to the world         
                    morcab->set_floating_base(false);  // opt for a fixed base for kinematic chain
                }

                // TODO: set contact parameters

                // add the link reference to all relevant containers               
                link->plink = *itlink;             // ref back to openrave link
                pinfo->vlinks.push_back(link);     // ref for link indexing    
                molinks.push_back(link);           // ref for Moby articulated body definition
            }
    
            // iterate over the set of joints in the OpenRave environment
            // create a joint for each and insert them into the Moby articulated body
            FOREACH(itjoint, vbodyjoints) 
            {
    
                Moby::RigidBodyPtr inboard, outboard; // inboard=parent, outboard=child
                Moby::JointPtr joint;
   
                // validate that there are two link references for the joint   
                if( !!(*itjoint)->GetFirstAttached() ) {
                    inboard = pinfo->vlinks.at((*itjoint)->GetFirstAttached()->GetIndex());
                }
                if( !!(*itjoint)->GetSecondAttached() ) {
                    outboard = pinfo->vlinks.at((*itjoint)->GetSecondAttached()->GetIndex());
                }
                if( !inboard || !outboard ) {
                    RAVELOG_ERROR(str(boost::format("joint %s needs to be attached to two bodies!\n")%(*itjoint)->GetName()));
                    continue;
                }
                  
                switch((*itjoint)->GetType()) 
                {
                    case KinBody::JointHinge: 
                    {
                        // map a Moby revolute joint
                        boost::shared_ptr<Moby::RevoluteJoint> rjoint(new Moby::RevoluteJoint);
                        rjoint->id = (*itjoint)->GetName();
        
                        // set the location of the joint with respect to body0 and body1
                        Vector anchor = (*itjoint)->GetAnchor();
                        rjoint->set_location(Ravelin::Vector3d(anchor[0], anchor[1], anchor[2], Moby::GLOBAL), inboard, outboard);
    
                        // set the joint axis w.r.t. the global frame
                        Vector axis = (*itjoint)->GetAxis(0);
                        rjoint->set_axis(Ravelin::Vector3d(axis[0],axis[1],axis[2],Moby::GLOBAL));    
    
                        // get the joint limits
                        vector<dReal> vupper,vlower;
                        (*itjoint)->GetLimits(vlower,vupper);
    
                        // set joint limits
                        if( vlower.size() )
                        {
                            rjoint->lolimit = vlower.at(0);
                        }
                        if( vupper.size() )
                        {
                            rjoint->hilimit = vupper.at(0);
                        }

                        // convert the revolute reference to a generalized joint
                        joint = rjoint;
    
                        break;
                    }
                    case KinBody::JointSlider: 
                    {
                        // map a Moby prismatic joint
                        boost::shared_ptr<Moby::PrismaticJoint> pjoint(new Moby::PrismaticJoint);
                        pjoint->id = (*itjoint)->GetName();
        
                        // set the location of the joint with respect to body0 and body1
                        Vector anchor = (*itjoint)->GetAnchor();
                        pjoint->set_location(Ravelin::Vector3d(anchor[0], anchor[1], anchor[2], Moby::GLOBAL), inboard, outboard);
    
                        // set the joint axis w.r.t. the global frame
                        Vector axis = (*itjoint)->GetAxis(0);
                        pjoint->set_axis(Ravelin::Vector3d(axis[0],axis[1],axis[2],Moby::GLOBAL));    
    
                        // get the joint limits
                        vector<dReal> vupper,vlower;
                        (*itjoint)->GetLimits(vlower,vupper);
    
                        // set joint limits
                        if( vlower.size() )
                        {
                            pjoint->lolimit = vlower[0];
                        }
                        if( vupper.size() )
                        {
                            pjoint->hilimit = vupper[0];
                        }
 
                        // convert the prismatic reference to a generalized joint
                        joint = pjoint;

                        break;
                    }
                    case KinBody::JointSpherical: 
                    {
                        boost::shared_ptr<Moby::SphericalJoint> sjoint(new Moby::SphericalJoint);
                        joint = sjoint;
        /*
                        btVector3 pivotInA = GetBtVector(t0inv * (*itjoint)->GetAnchor());
                        btVector3 pivotInB = GetBtVector(t1inv * (*itjoint)->GetAnchor());
                        boost::shared_ptr<btPoint2PointConstraint> spherical(new btPoint2PointConstraint(*body0, *body1, pivotInA, pivotInB));
                        joint = spherical;
        */
                        break;
                    }
                    case KinBody::JointUniversal: 
                    {
                        boost::shared_ptr<Moby::UniversalJoint> sjoint(new Moby::UniversalJoint);
                        joint = sjoint;
                        
                        break;
                    }
                    case KinBody::JointHinge2:
                    {
                        RAVELOG_ERROR("hinge2 joint not supported by Moby\n");
                        break;
                    }
                    default:
                    {
                        RAVELOG_ERROR("unknown joint type 0x%8.8x\n", (*itjoint)->GetType());
                        break;
                    }
                }
    
                if( !!joint ) 
                { 
                    // add the joint to the set of joints
                    mojoints.push_back(joint);
                    _mapJoints.insert(std::pair<KinBody::JointPtr,Moby::JointPtr>(*itjoint,joint));
                }
            }
     
            // add the links and joints to the articulated body
            morcab->set_links_and_joints(molinks,mojoints);
            // add gravity to the articulated body
            morcab->get_recurrent_forces().push_back(gravity);

/*
            //morcab->controller = boost::bind(&MobySpace::_Controller,shared_from_this(),_3);
            //morcab->controller = &MobySpace::_Controller;
            void (*f)(Moby::DynamicBodyPtr, double, void*);
            f = boost::bind(&MobySpace::_Controller, shared_from_this(), _1, _2, _3);
            //f = boost::bind(&MobySpace::_Controller, this, _1, _2, _3);
            //morcab->controller = (ControllerCallbackFn)boost::bind(&MobySpace::_Controller, shared_from_this(), _3);
*/
            // add the articulated body to the world
            _world->add_dynamic_body(morcab);
        }
        else
        {
            RAVELOG_INFO("Request to map a KinBody with no links\n");
        }

        saver.Restore();

        return pinfo;
    }

    void Synchronize()
    {
        vector<KinBodyPtr> vbodies;
        _penv->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) 
        {
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
        if( pinfo->nLastStamp != pbody->GetUpdateStamp() ) 
        {
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
        std::map<KinBody::JointConstPtr, Moby::JointPtr>::iterator it;
        Moby::JointPtr joint;

        it = _mapJoints.find(pjoint);
        if(it != _mapJoints.end()) 
        {
            joint = it->second;
        }

        return joint;
    }

    void SetSynchronizationCallback(const SynchronizeCallbackFn &synccallback) 
    {
        _synccallback = synccallback;
    }

    void ClearBuffers(void) 
    {
        //
    }

    static inline Transform GetTransform(const Ravelin::Pose3d &p)
    {
        Ravelin::Pose3d t( p );
        t.update_relative_pose(Moby::GLOBAL);
        return Transform(Vector(t.q.w, t.q.x, t.q.y, t.q.z), Vector(t.x.x(), t.x.y(), t.x.z()));
    }

    static inline Ravelin::Pose3d GetRavelinPose(const Transform &t)
    {
        // TODO assertion sanity check
        OPENRAVE_ASSERT_OP(RaveFabs(t.rot.lengthsqr4()-1),<=,0.01);

        // TODO: check with Rosen on the Quaternion parameter ordering
        //   as modeled from bulletspace.h.  Reason for this mapping 
        //   unclear.

        return(Ravelin::Pose3d(Ravelin::Quatd(t.rot.y, t.rot.z, t.rot.w, t.rot.x), GetRavelinOrigin(t.trans), Moby::GLOBAL));
    }

    static inline Ravelin::Origin3d GetRavelinOrigin(const Vector &v)
    {
        return Ravelin::Origin3d(v.x, v.y, v.z);
    }

    static inline Ravelin::VectorNd GetRavelinVectorN(const std::vector<dReal>& vector)
    {
        Ravelin::VectorNd result( vector.size() );
        for(unsigned i=0; i < vector.size(); i++) {
            result[i] = vector[i];
        }
        return result;
    }

    bool IsInitialized() 
    {
        return !!_world;
    }

    boost::shared_ptr<Moby::GravityForce> gravity;

private:

    void _Synchronize(KinBodyInfoPtr pinfo)
    {
        vector<Transform> vtrans;
        std::vector<int> dofbranches;
        pinfo->pbody->GetLinkTransformations(vtrans,dofbranches);
        pinfo->nLastStamp = pinfo->pbody->GetUpdateStamp();
        BOOST_ASSERT( vtrans.size() == pinfo->vlinks.size() );
        for(size_t i = 0; i < vtrans.size(); ++i) 
        {
            pinfo->vlinks[i]->set_pose(GetRavelinPose(vtrans[i]*pinfo->vlinks[i]->tlocal));
        }
        if( !!_synccallback ) 
        {
            _synccallback(pinfo);
        }
    }

    virtual void GeometryChangedCallback(KinBodyWeakPtr _pbody)
    {
        EnvironmentMutex::scoped_lock lock(_penv->GetMutex());
        KinBodyPtr pbody(_pbody);
        KinBodyInfoPtr pinfo = GetInfo(pbody);
        if( !pinfo ) 
        {
            return;
        }
        BOOST_ASSERT(boost::shared_ptr<MobySpace>(pinfo->_mobyspace) == shared_from_this());
        BOOST_ASSERT(pinfo->pbody==pbody);
        InitKinBody(pbody,pinfo);
    }


/*
    void SetControllerCallback(const  &controllercallback) 
    {
        _controllercallback = controllercallback;
    }
*/
    void _Controller( Moby::DynamicBodyPtr db, double t, void* ) {
        
    }

private:
    EnvironmentBasePtr _penv;
    GetInfoFn GetInfo;
    boost::shared_ptr<Moby::Simulator> _world;
    SynchronizeCallbackFn _synccallback;
    bool _bPhysics;
    //ControllerCallbackFn _controllercallback;

    std::map<KinBody::JointConstPtr, Moby::JointPtr> _mapJoints;
    

};
/*
static KinBody::LinkPtr GetLinkFromCollision(const btCollisionObject* co) 
{
    BOOST_ASSERT(co != NULL);
    return static_cast<MobySpace::KinBodyInfo::LINK*>(co->getUserPointer())->plink;
}

static KinBody::LinkPtr GetLinkFromProxy(btBroadphaseProxy* proxy) 
{
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
        if( !plink0->IsEnabled() || !plink1->IsEnabled() ) 
        {
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
