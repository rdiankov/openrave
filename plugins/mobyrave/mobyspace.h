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

            virtual void SetWorldTransform(const Transform& comWorldTrans)
            {
                plink->SetTransform(comWorldTrans*tlocal.inverse());
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
        vector<boost::shared_ptr<LINK> > vlinks;     
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

    inline boost::shared_ptr<KinBodyInfo::LINK> DeriveLink(boost::shared_ptr<KinBody::Link> plink) {

        boost::shared_ptr<KinBodyInfo::LINK> link;
        link = boost::shared_ptr<KinBodyInfo::LINK>(new KinBodyInfo::LINK());
        std::cout << "processing link[" << plink->GetName() << "], mass[" << plink->GetMass() << "]"<< std::endl; 
        if(plink->GetMass() < 1e-12)
        {
            std::cout << "link " << plink->GetName() << " has no effective mass. mass=" << plink->GetMass() << std::endl; 
            //return link;
            link->set_kinematic(true);
        } 

        // add the link reference to all relevant containers               
        link->plink = plink;               // ref back to OpenRave link

        // compute a box to approximate link inertial properties
        AABB bb = plink->ComputeLocalAABB();
        link->_primitive = Moby::PrimitivePtr(new Moby::BoxPrimitive(bb.extents.x*2,bb.extents.y*2,bb.extents.z*2));
        link->_primitive->set_mass(plink->GetMass());

        // assign link parameters
        link->id = plink->GetName();                          // identity  
        link->set_inertia(link->_primitive->get_inertia());   // inertia
        link->set_enabled(true);                              // enable physics
        link->get_recurrent_forces().push_back(_gravity);     // gravity

        // assign transforms (Note: maintain the order of this section)
        link->tlocal = plink->GetLocalMassFrame();            // com frame transform
        // initialize the moby based pose for the link
        Ravelin::Pose3d pose = GetRavelinPose(plink->GetTransform()*link->tlocal);
        pose.update_relative_pose(link->get_pose()->rpose);
        link->set_pose(pose);

        // check for a static link
        if( plink->IsStatic() ) {
            link->set_enabled(false);    // disable physics to fix to the world 
        }

        // allocate _mapImpulses for the link
        pair<map<Moby::RigidBodyPtr, vector<Ravelin::SForced> >::iterator, bool> it;
        it = _mapImpulses.insert(pair<Moby::RigidBodyPtr, vector<Ravelin::SForced> >(link, vector<Ravelin::SForced>()));
        it.first->second.reserve(1);

        // TODO: set contact parameters

        return link;
    }

    inline Moby::JointPtr DeriveJoint(KinBodyInfoPtr pinfo, boost::shared_ptr<KinBody::Joint> pjoint) {
        Moby::RigidBodyPtr inboard, outboard; // inboard=parent, outboard=child
        Moby::JointPtr joint;
   
        // validate that there are two link references for the joint   
        if( !!pjoint->GetFirstAttached() ) {
            inboard = pinfo->vlinks.at(pjoint->GetFirstAttached()->GetIndex());
        }
        if( !!pjoint->GetSecondAttached() ) {
            outboard = pinfo->vlinks.at(pjoint->GetSecondAttached()->GetIndex());
        }
        if( !inboard || !outboard ) {
            RAVELOG_ERROR(str(boost::format("joint %s needs to be attached to two bodies!\n")%pjoint->GetName()));
            return joint;  // null
        }
          
        switch(pjoint->GetType()) 
        {
            case KinBody::JointHinge: 
            {
                // map a Moby revolute joint
                boost::shared_ptr<Moby::RevoluteJoint> rjoint(new Moby::RevoluteJoint);
                rjoint->id = pjoint->GetName();
        
                // set the location of the joint with respect to body0 and body1
                Vector anchor = pjoint->GetAnchor();
                rjoint->set_location(Ravelin::Vector3d(anchor[0], anchor[1], anchor[2], Moby::GLOBAL), inboard, outboard);
    
                // set the joint axis w.r.t. the global frame
                Vector axis = pjoint->GetAxis(0);
                rjoint->set_axis(Ravelin::Vector3d(axis[0],axis[1],axis[2],Moby::GLOBAL));    
    
                // get the joint limits
                vector<dReal> vupper,vlower;
                pjoint->GetLimits(vlower,vupper);
    
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
/*
                // map a Moby prismatic joint
                boost::shared_ptr<Moby::PrismaticJoint> sjoint(new Moby::PrismaticJoint);
                sjoint->id = pjoint->GetName();
        
                // set the location of the joint with respect to body0 and body1
                Vector anchor = pjoint->GetAnchor();
                sjoint->set_location(Ravelin::Vector3d(anchor[0], anchor[1], anchor[2], Moby::GLOBAL), inboard, outboard);
    
                // set the joint axis w.r.t. the global frame
                Vector axis = pjoint->GetAxis(0);
                sjoint->set_axis(Ravelin::Vector3d(axis[0],axis[1],axis[2],Moby::GLOBAL));    
    
                // get the joint limits
                vector<dReal> vupper,vlower;
                pjoint->GetLimits(vlower,vupper);
    
                // set joint limits
                if( vlower.size() )
                {
                    sjoint->lolimit = vlower[0];
                }
                if( vupper.size() )
                {
                    sjoint->hilimit = vupper[0];
                }
 
                // convert the prismatic reference to a generalized joint
                joint = sjoint;
*/
                break;
            }
            case KinBody::JointSpherical: 
            {
                boost::shared_ptr<Moby::SphericalJoint> sjoint(new Moby::SphericalJoint);
                joint = sjoint;
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
                RAVELOG_ERROR("unknown joint type 0x%8.8x\n", pjoint->GetType());
                break;
            }
        }

        if(!joint) 
        {
            return joint;  // null
        }

        // allocate _mapControls for the joint
        pair<map<Moby::JointPtr, vector<Ravelin::VectorNd> >::iterator, bool> it;
        it = _mapControls.insert(pair<Moby::JointPtr, vector<Ravelin::VectorNd> >(joint, vector<Ravelin::VectorNd>() ));
        it.first->second.reserve(1);

        return joint;
    }

    KinBodyInfoPtr InitKinBody(KinBodyPtr pbody, KinBodyInfoPtr pinfo = KinBodyInfoPtr(), double fmargin=0.0005)
    {
	vector<dReal> vzeros(pbody->GetDOF(), 0);
	pbody->SetDOFValues(vzeros);

        if( !pinfo ) 
        {
            pinfo.reset(new KinBodyInfo(_world,_bPhysics));
        }
        pinfo->Reset();
        pinfo->pbody = pbody;
        pinfo->_mobyspace = weak_space();

        // Note: need to deal with purely collision hull bodies

        if(pbody->GetLinks().size() == 1) 
        {
            // branch already asserted size
            boost::shared_ptr<KinBody::Link> plink = pbody->GetLinks()[0];

            // initialize the Moby compatible link by deriving from the OpenRave link
            boost::shared_ptr<KinBodyInfo::LINK> link = DeriveLink(plink);

            // if the link failed to be derived (most likely due to being just a collision hull) don't add it
            if(!link)
            {
                std::cout << "failed to process link[" << plink->GetName() << "]"<< std::endl; 
                return pinfo;
            }
                            
            pinfo->vlinks.reserve(pbody->GetLinks().size());
            pinfo->vlinks.push_back(link);     // ref for link indexing 

            // register the controller callback; necessary for writing state and controls
            link->register_controller_callback( boost::bind( &MobySpace::_Controller,shared_from_this(),_1,_2,_3) );

            // add the body to the world
            _world->add_dynamic_body(link);
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
            vector<Moby::RigidBodyPtr> molinks;   // for adding the links to the articulated body
            molinks.reserve(pbody->GetLinks().size());
    
            // ?: Do PassiveJoints -> fixed joints/welds
            vector<KinBody::JointPtr> vbodyjoints;
            vbodyjoints.reserve(pbody->GetJoints().size()+pbody->GetPassiveJoints().size());
            vbodyjoints.insert(vbodyjoints.end(),pbody->GetJoints().begin(),pbody->GetJoints().end());
            vbodyjoints.insert(vbodyjoints.end(),pbody->GetPassiveJoints().begin(),pbody->GetPassiveJoints().end());
            vector<Moby::JointPtr> mojoints;      // for adding the joints to the articulated body
            vector<boost::shared_ptr<KinBodyInfo::LINK> > veclinks;
            mojoints.reserve(pbody->GetJoints().size()+pbody->GetPassiveJoints().size());

            // iterate over the set of links in the OpenRave environment
            // create a link for each and insert them into the Moby articulated body
            FOREACHC(itlink, pbody->GetLinks()) 
            {
                // initialize the Moby compatible link by deriving from the OpenRave link
                boost::shared_ptr<KinBodyInfo::LINK> link = DeriveLink(*itlink);

                // if the link failed to be derived (most likely due to being just a collision hull) don't add it
                if(!link)
                {
                    std::cout << "failed to process link[" << (*itlink)->GetName() << "]"<< std::endl; 
                    continue;
                }

                // if a static link opt for a fixed based for the whole kinematic chain
                if( (*itlink)->IsStatic() ) {
                    morcab->set_floating_base(false);
                }

                // add the link to the set used to initialize the Moby articulated body
                molinks.push_back(link);
                veclinks.push_back(link);
            }
    
            pinfo->vlinks.reserve(veclinks.size());

            FOREACH(itlink, veclinks) 
            { 
                pinfo->vlinks.push_back(*itlink);     // ref for link indexing 
            }

            // iterate over the set of joints in the OpenRave environment
            // create a joint for each and insert them into the Moby articulated body
            FOREACH(itjoint, vbodyjoints) 
            {
                // initialize the Moby joint by deriving from the OpenRave joint
                Moby::JointPtr joint = DeriveJoint(pinfo, *itjoint);

                // if the initialization failed, move on to next joint 
                if( !joint ) 
                {
                    std::cout << "failed to process joint[" << (*itjoint)->GetName() << "]"<< std::endl; 
                    continue;
                } 

                // add the joint to the set used to initialize the Moby articulated body
                mojoints.push_back(joint);

                // add the joint to the OpenRave joint to Moby joint map
                _mapJointId.insert(pair<string,KinBody::JointConstPtr>((*itjoint)->GetName(), *itjoint));
                _mapJoint.insert(pair<KinBody::JointConstPtr,Moby::JointPtr>(*itjoint,joint));
            }
     
            // add the links and joints to the articulated body
            morcab->set_links_and_joints(molinks,mojoints);
            // add gravity to the articulated body
            morcab->get_recurrent_forces().push_back(_gravity);
            // register the controller callback; necessary for writing state and controls
            morcab->register_controller_callback( boost::bind( &MobySpace::_Controller,shared_from_this(),_1,_2,_3) );
            // add the articulated body to the world
            _world->add_dynamic_body(morcab);
        }
        else
        {
            RAVELOG_INFO("Request to map a KinBody with no links\n");
        }

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

    Moby::JointPtr GetJoint(string id)
    {
        map<string, KinBody::JointConstPtr>::iterator it;
        it = _mapJointId.find(id);
        if( it == _mapJointId.end() ) 
        {
            return Moby::JointPtr();  //null
        } 
        return GetJoint(it->second);
    }

    Moby::JointPtr GetJoint(KinBody::JointConstPtr pjoint)
    {
        map<KinBody::JointConstPtr, Moby::JointPtr>::iterator it;
        it = _mapJoint.find(pjoint);
        if(it == _mapJoint.end()) 
        {
            return Moby::JointPtr();  // null
        }
        return it->second;
    }

/*
    dReal GetPosition(Moby::JointPtr joint, unsigned axis)
    {
        if(!joint || axis >= joint->num_dof())
        {
            return 0; // should throw instead
        }
        return joint->q[axis];
    }
*/
    void SetPosition(Moby::JointPtr joint, unsigned axis, dReal value)
    {
        if(!joint || axis >= joint->num_dof())
        {
            return;
        }
        joint->q[axis] = value;
    }

    void AddControl(Moby::JointPtr joint, Ravelin::VectorNd u)
    {
        if(!joint) 
        {
            return;
        }

        map<Moby::JointPtr, vector<Ravelin::VectorNd> >::iterator it;
        it = _mapControls.find(joint);
        if(it != _mapControls.end()) 
        {
            it->second.push_back(u);
        }
    }

    void AddImpulse(Moby::RigidBodyPtr body, Ravelin::SForced force)
    {
        if(!body) 
        {
            return;
        }

        map<Moby::RigidBodyPtr, vector<Ravelin::SForced> >::iterator it;
        it = _mapImpulses.find(body);
        if( it != _mapImpulses.end() ) 
        {
            it->second.push_back(force);
        }
    }

    // not validated and probably not valid
    void SetVelocity(Moby::RigidBodyPtr body, Ravelin::SVelocityd velocity)
    {
        if(!body) 
        {
            return;
        }

        map<Moby::RigidBodyPtr, Ravelin::SVelocityd>::iterator mit = _mapVelocity.find(body);
        if( mit == _mapVelocity.end() ) 
        {
            _mapVelocity.insert(pair<Moby::RigidBodyPtr, Ravelin::SVelocityd>(body, velocity));
        }
        else
        {
            (*mit).second = velocity;
        }
    }

    void SetSynchronizationCallback(const SynchronizeCallbackFn &synccallback) 
    {
        _synccallback = synccallback;
    }

    static inline Transform GetTransform(const Ravelin::Pose3d &p)
    {
        Ravelin::Pose3d t( p );
        t.update_relative_pose(Moby::GLOBAL);
        return Transform(Vector(t.q.w, t.q.x, t.q.y, t.q.z), Vector(t.x.x(), t.x.y(), t.x.z()));
    }

    static inline Ravelin::Pose3d GetRavelinPose(const Transform &t)
    {
        OPENRAVE_ASSERT_OP(RaveFabs(t.rot.lengthsqr4()-1),<=,0.01);

        Ravelin::Quatd q(t.rot.y, t.rot.z, t.rot.w, t.rot.x);
        q.normalize();
        Ravelin::Origin3d x = GetRavelinOrigin(t.trans);
        return Ravelin::Pose3d(q, x, Moby::GLOBAL);
    }

    static inline Ravelin::Origin3d GetRavelinOrigin(const Vector &v)
    {
        return Ravelin::Origin3d(v.x, v.y, v.z);
    }

    static inline Ravelin::SForced GetRavelinSForce(const Vector& force, const Vector& torque, boost::shared_ptr<const Ravelin::Pose3d> pose)
    {
        return Ravelin::SForced(Ravelin::Vector3d(force[0],force[1],force[2],Moby::GLOBAL), Ravelin::Vector3d(torque[0],torque[1],torque[2]),pose);
    }

    static inline Ravelin::VectorNd GetRavelinVectorN(const vector<dReal>& vector)
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

private:

    void _Synchronize(KinBodyInfoPtr pinfo)
    {
        vector<Transform> vtrans;
        vector<int> dofbranches;
        pinfo->pbody->GetLinkTransformations(vtrans,dofbranches);
        pinfo->nLastStamp = pinfo->pbody->GetUpdateStamp();
        BOOST_ASSERT( vtrans.size() == pinfo->vlinks.size() );
        for(size_t i = 0; i < vtrans.size(); ++i) 
        {
            pinfo->vlinks[i]->SetWorldTransform(vtrans[i]);
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

    // the callback controller function.  
    // Moby requires that controls be applied inside a callback
    // this function ensures openrave is able to update controls
    void _Controller( Moby::DynamicBodyPtr db, const double& t, void* ) {

        // input buffer for controls and state
        // output buffer for state?

        // try to process as an articulated body
        Moby::ArticulatedBodyPtr ab = boost::dynamic_pointer_cast<Moby::ArticulatedBody>(db);
        if(!!ab) 
        {
            vector<Moby::JointPtr> joints = ab->get_joints();
            vector<Moby::RigidBodyPtr> links = ab->get_links();

            // process joint controls
            for(vector<Moby::JointPtr>::iterator jit = joints.begin(); jit != joints.end(); jit++)
            {

                map<Moby::JointPtr, vector<Ravelin::VectorNd> >::iterator mit = _mapControls.find(*jit);
                if( mit == _mapControls.end() ) 
                {
                    continue;
                }

                for(vector<Ravelin::VectorNd>::iterator vit = mit->second.begin(); vit != mit->second.end(); vit++ )
                {
                    (*jit)->add_force((*vit));
                }
                mit->second.clear();
            }

            // process link velocities
            for(vector<Moby::RigidBodyPtr>::iterator vit = links.begin(); vit != links.end(); vit++)
            {
                map<Moby::RigidBodyPtr, Ravelin::SVelocityd>::iterator mit = _mapVelocity.find(*vit);
                if( mit == _mapVelocity.end() ) 
                {
                    continue;
                }
                (*vit)->set_velocity((*mit).second);
            }

            // process link impulses
            for(vector<Moby::RigidBodyPtr>::iterator lit = links.begin(); lit != links.end(); lit++)
            {
                map<Moby::RigidBodyPtr, vector<Ravelin::SForced> >::iterator mit = _mapImpulses.find(*lit);
                if( mit == _mapImpulses.end() ) 
                {
                    continue;
                }

                for(vector<Ravelin::SForced>::iterator vit = mit->second.begin(); vit != mit->second.end(); vit++ )
                {
                    (*lit)->add_force((*vit));
                }
                mit->second.clear();
            }

            // done with processing for this dynamic body so exit here
            return; 
        }

        // otherwise try to process as a rigid body
        Moby::RigidBodyPtr rb = boost::dynamic_pointer_cast<Moby::RigidBody>(db);
        if(!!rb) 
        {

            // process body velocity
            {
                map<Moby::RigidBodyPtr, Ravelin::SVelocityd>::iterator mit = _mapVelocity.find(rb);
                if( mit != _mapVelocity.end() ) 
                {
                    rb->set_velocity((*mit).second);
                }
            }

            // process body impulses
            {
                map<Moby::RigidBodyPtr, vector<Ravelin::SForced> >::iterator mit = _mapImpulses.find(rb);
                if( mit != _mapImpulses.end() ) 
                {
                    for(vector<Ravelin::SForced>::iterator vit = mit->second.begin(); vit != mit->second.end(); vit++ )
                    {
                        rb->add_force((*vit));
                    }
                    mit->second.clear();
                }
            }

            // done with processing for this dynamic body so exit here
            return; 
        }
    }

private:
    EnvironmentBasePtr _penv;
    GetInfoFn GetInfo;
    boost::shared_ptr<Moby::Simulator> _world;
    SynchronizeCallbackFn _synccallback;
    bool _bPhysics;

    // templated comparator for comparing the value of two shared pointers
    // used predominantly to ensure maps keyed on shared pointers are hashed properly
    template<class T>
    class _CompareSharedPtrs {
    public:
       bool operator()(boost::shared_ptr<T> a, boost::shared_ptr<T> b) const {
          return a.get() < b.get();
       }
    };

    map<string, KinBody::JointConstPtr> _mapJointId;
    map<KinBody::JointConstPtr, Moby::JointPtr, _CompareSharedPtrs<KinBody::Joint const> > _mapJoint;

    map<Moby::JointPtr, vector<Ravelin::VectorNd>, _CompareSharedPtrs<Moby::Joint> > _mapControls;
    map<Moby::RigidBodyPtr, vector<Ravelin::SForced>, _CompareSharedPtrs<Moby::RigidBody> > _mapImpulses;
    map<Moby::RigidBodyPtr, Ravelin::SVelocityd, _CompareSharedPtrs<Moby::RigidBody> > _mapVelocity;

public:
    boost::shared_ptr<Moby::GravityForce> _gravity;

};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(MobySpace)
BOOST_TYPEOF_REGISTER_TYPE(MobySpace::KinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(MobySpace::KinBodyInfo::LINK)
BOOST_TYPEOF_REGISTER_TYPE(dJointID)
#endif

#endif
