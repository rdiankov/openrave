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

#include "btBulletCollisionCommon.h"
#include "btBulletDynamicsCommon.h"

// groups bits for bullet
#define ENABLED_GROUP 1 // mask ENABLED_GROUP
#define DISABLED_GROUP 256 // mask 0

// manages a space of ODE objects
class BulletSpace
{
public:
    // information about the kinematics of the body
    struct KINBODYINFO
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
            
            Transform tlocal; /// local offset transform to account for inertias not aligned to axes
        };

        KINBODYINFO();
        virtual ~KINBODYINFO();

        KinBody* pbody; ///< body associated with this structure
        int nLastStamp;
        
        vector<LINK> vlinks; ///< if body is disabled, then geom is static (it can't be connected to a joint!)
                                ///< the pointer to this Link is the userdata
        vector< boost::shared_ptr<btTypedConstraint> > vjoints;
    };

    typedef void* (*GetInfoFn)(const KinBody* pbody);
    typedef void (*SynchornizeCallbackFn)(void* userdata, KINBODYINFO* pinfo);

    BulletSpace(EnvironmentBase* penv, GetInfoFn infofn, bool bPhysics);
    ~BulletSpace();

    bool InitEnvironment(boost::shared_ptr<btCollisionWorld>& world);
    void DestroyEnvironment();
    void* InitKinBody(KinBody* pbody); /// returns data to set
    bool DestroyKinBody(KinBody* pbody);

    bool Enable(const KinBody* pbody, bool bEnable);
    bool EnableLink(const KinBody::Link* pbody, bool bEnable);

    void Synchronize();
    void Synchronize(const KinBody* pbody);

    btCollisionObject* GetLinkObj(const KinBody::Link* plink);
    btTypedConstraint* GetJoint(const KinBody::Joint*);

    void SetSynchornizationCallback(SynchornizeCallbackFn synccallback, void* userdata);

    static inline btTransform GetBtTransform(const Transform& t)
    {
        return btTransform(btQuaternion(t.rot.y,t.rot.z,t.rot.w,t.rot.x),GetBtVector(t.trans));
    }

    static inline btVector3 GetBtVector(const Vector& v)
    {
        return btVector3(v.x,v.y,v.z);
    }

private:
    
    void Synchronize(KINBODYINFO* pinfo);
    void RestoreJoints(KinBody* pbody);

    EnvironmentBase* _penv;
    GetInfoFn GetInfo;
    boost::shared_ptr<btCollisionWorld> _world;

    SynchornizeCallbackFn _synccallback;
    void* _userdata;

    bool _bPhysics;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(BulletSpace)
BOOST_TYPEOF_REGISTER_TYPE(BulletSpace::KINBODYINFO)
BOOST_TYPEOF_REGISTER_TYPE(BulletSpace::KINBODYINFO::LINK)
BOOST_TYPEOF_REGISTER_TYPE(dJointID)
#endif

#endif
