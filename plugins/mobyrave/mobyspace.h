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
        // Note: may need to inherit from DynamicBody instead
        class LINK : public Moby::RigidBody
        {
public:
            virtual ~LINK() {
            }

            virtual void GetWorldPose(Ravelin::Pose3d& comWorldTrans) const
            {
                comWorldTrans = GetRavelinPose( plink->GetTransform()*tlocal );
            }

            virtual void SetWorldPose(const Ravelin::Pose3d& comWorldTrans)
            {
                plink->SetTransform(GetTransform(comWorldTrans)*tlocal.inverse());
            }

            //Moby::RigidBodyPtr _body;
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

        // ?: if pbody is robot then rcarticulatedbody otherwise rigidbody
	// does IsRobot imply that the KinBody is hierarchical?
        if( pbody->IsRobot() ){

            // TODO: this is only the case for the construction of an articulated robot
            //   Rigid bodies should be created as RigidBody   
            // RCArticulated body at the root of the hierarchy
            Moby::RCArticulatedBodyPtr morcab(new Moby::RCArticulatedBody());
    
            std::vector<Moby::RigidBodyPtr> molinks;
            std::vector<Moby::JointPtr> mojoints;
    
            //std::map<std::string, Moby::RigidBodyPtr> link_map;
    
            FOREACHC(itlink, pbody->GetLinks()) {
                boost::shared_ptr<KinBodyInfo::LINK> link(new KinBodyInfo::LINK());
                link->id = (*itlink)->GetName();
    
                // create a Moby::RigidBody
                // NOTE: the mass reference frame may not be centered.  
                //   Do the Moby primitives generally assume a centered frame?
                AABB bb = (*itlink)->ComputeLocalAABB();
                Moby::PrimitivePtr prim(new Moby::BoxPrimitive(bb.extents.x*2,bb.extents.y*2,bb.extents.z*2));
                Ravelin::Vector3d com(bb.pos.x,bb.pos.y,bb.pos.z);
    
                //Moby::RigidBodyPtr child(new Moby::RigidBody());
                prim->set_mass((*itlink)->GetMass());
                
                link->set_visualization_data(prim->create_visualization());
                link->set_inertia(prim->get_inertia());
                link->set_enabled(true);
    
                // TODO: validate inertial setup for primitive
    
                // TODO: set contact parameters?
               
                pinfo->vlinks.push_back(link);
                molinks.push_back(link);
                //link_map.insert(std::pair<std::string,Moby::RigidBodyPtr>(link->id,link));
            }
    
            vector<KinBody::JointPtr> vbodyjoints; vbodyjoints.reserve(pbody->GetJoints().size()+pbody->GetPassiveJoints().size());
            vbodyjoints.insert(vbodyjoints.end(),pbody->GetJoints().begin(),pbody->GetJoints().end());
            vbodyjoints.insert(vbodyjoints.end(),pbody->GetPassiveJoints().begin(),pbody->GetPassiveJoints().end());
            // ?: Do PassiveJoints -> fixed joints/welds
    
            FOREACH(itjoint, vbodyjoints) {
                // create a moby joint
    
                Moby::RigidBodyPtr body0, body1;
                Moby::JointPtr joint;
    
                if( !!(*itjoint)->GetFirstAttached() ) {
                    body0 = pinfo->vlinks.at((*itjoint)->GetFirstAttached()->GetIndex());
                }
                if( !!(*itjoint)->GetSecondAttached() ) {
                    body1 = pinfo->vlinks.at((*itjoint)->GetSecondAttached()->GetIndex());
                }
                if( !body0 || !body1 ) {
                    RAVELOG_ERROR(str(boost::format("joint %s needs to be attached to two bodies!\n")%(*itjoint)->GetName()));
                    continue;
                }
                
                //Transform t0inv = GetTransform(body0->get_pose()).inverse();
                //Transform t1inv = GetTransform(body1->get_pose()).inverse();
                
                switch((*itjoint)->GetType()) {
                case KinBody::JointHinge: {
                    boost::shared_ptr<Moby::RevoluteJoint> rjoint(new Moby::RevoluteJoint);
                    joint = rjoint;
    
                    // ?: what reference frame is the KinBody Joint information in?
                    // Answer will affect the construction of the joint location below
    
                    Ravelin::Pose3d pose;
    
                    pose.rpose = body1->get_pose();
                    pose.update_relative_pose(Moby::GLOBAL);
                    Ravelin::Vector3d loc(pose.x[0], pose.x[1], pose.x[2], Moby::GLOBAL);
                    joint->set_location(loc, body0, body1);
    
                    // set axis
                    //(*itjoint)->GetAnchor();
                    //(*itjoint)->GetAxis(0);
    /*
                    btVector3 pivotInA = GetBtVector(t0inv * (*itjoint)->GetAnchor());
                    btVector3 pivotInB = GetBtVector(t1inv * (*itjoint)->GetAnchor());
                    btVector3 axisInA = GetBtVector(t0inv.rotate((*itjoint)->GetAxis(0)));
                    btVector3 axisInB = GetBtVector(t1inv.rotate((*itjoint)->GetAxis(0)));
                    boost::shared_ptr<btHingeConstraint> hinge(new btHingeConstraint(*body0, *body1, pivotInA, pivotInB, axisInA, axisInB));
                    //hinge->setParam(BT_CONSTRAINT_STOP_ERP,0.8);
                    //hinge->setParam(BT_CONSTRAINT_STOP_CFM,0);
                    //hinge->setParam(BT_CONSTRAINT_CFM,0);
                    vector<dReal> vupper,vlower;
                    (*itjoint)->GetLimits(vlower,vupper);
                    hinge->setLimit(vlower.at(0),vupper.at(0),0.9f,0.9f,1.0f);
                    if( !(*itjoint)->IsCircular(0) ) {
                        vector<dReal> vlower, vupper;
                        (*itjoint)->GetLimits(vlower,vupper);
                        btScalar orInitialAngle = (*itjoint)->GetValue(0);
                        btScalar btInitialAngle = hinge->getHingeAngle();
                        btScalar lower_adj, upper_adj;
                        btScalar diff = (btInitialAngle + orInitialAngle);
                        lower_adj = diff - vupper.at(0);
                        upper_adj = diff - vlower.at(0);
                        hinge->setLimit(lower_adj,upper_adj);
                    }
                    joint = hinge;
    */
                    break;
                }
                case KinBody::JointSlider: {
                    boost::shared_ptr<Moby::PrismaticJoint> pjoint(new Moby::PrismaticJoint);
                    joint = pjoint;
    /*
                    Transform tslider; tslider.rot = quatRotateDirection(Vector(1,0,0),(*itjoint)->GetAxis(0));
                    btTransform frameInA = GetBtTransform(t0inv*tslider);
                    btTransform frameInB = GetBtTransform(t1inv*tslider);
                    joint.reset(new btSliderConstraint(*body0, *body1, frameInA, frameInB, true));
    */
                    break;
                }
                case KinBody::JointSpherical: {
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
                case KinBody::JointUniversal: {
                    boost::shared_ptr<Moby::UniversalJoint> sjoint(new Moby::UniversalJoint);
                    joint = sjoint;
    
                    
                    break;
                }
                case KinBody::JointHinge2:
                    RAVELOG_ERROR("hinge2 joint not supported by Moby\n");
                    break;
                default:
                    RAVELOG_ERROR("unknown joint type 0x%8.8x\n", (*itjoint)->GetType());
                    break;
                }
    
                if( !!joint ) {
    
    
                    // add the joint to the set of joints
                    mojoints.push_back(joint);
                }
            }
    
            // set fixed base
            // TODO: determine whether object has fixed base and branch for the following if so
            morcab->set_floating_base(false);
    
            // add the links and joints to the articulated body
            morcab->set_links_and_joints(molinks,mojoints);
        }
        else
        {
            //rigid body
        }
        return pinfo;
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

    static inline Transform GetTransform(const Ravelin::Pose3d &p)
    {
        return Transform(Vector(p.q.w, p.q.x, p.q.y, p.q.z), Vector(p.x.x(), p.x.y(), p.x.z()));
    }

    static inline Ravelin::Pose3d GetRavelinPose(const Transform &t)
    {
        // TODO assertion sanity check
        OPENRAVE_ASSERT_OP(RaveFabs(t.rot.lengthsqr4()-1),<=,0.01);

        // TODO: check with Rosen on the Quaternion parameter ordering
        //   as modeled from bulletspace.h.  Reason for this mapping 
        //   unclear.

        return(Ravelin::Pose3d(Ravelin::Quatd(t.rot.y, t.rot.z, t.rot.w, t.rot.x), GetRavelinOrigin(t.trans)));
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
        vector<Transform> vtrans;
        std::vector<int> dofbranches;
        pinfo->pbody->GetLinkTransformations(vtrans,dofbranches);
        pinfo->nLastStamp = pinfo->pbody->GetUpdateStamp();
        BOOST_ASSERT( vtrans.size() == pinfo->vlinks.size() );
        for(size_t i = 0; i < vtrans.size(); ++i) {
            pinfo->vlinks[i]->set_pose(GetRavelinPose(vtrans[i]*pinfo->vlinks[i]->tlocal));
        }
        if( !!_synccallback ) {
            _synccallback(pinfo);
        }
    }

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
