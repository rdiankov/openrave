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
#ifndef OPENRAVE_ODE_SPACE
#define OPENRAVE_ODE_SPACE

// manages a space of ODE objects
class ODESpace
{
public:
    // information about the kinematics of the body
    struct KINBODYINFO
    {
        struct LINK
        {
            LINK() : body(NULL), geom(NULL) {}

            dBodyID body;
            dGeomID geom;

            void Enable(bool bEnable);

            list<dTriIndex*> listtrimeshinds;
            list<dReal*> listvertices;
        };

        KINBODYINFO(dSpaceID odespace);
        virtual ~KINBODYINFO();

        KinBody* pbody; ///< body associated with this structure
        int nLastStamp;
        
        vector<LINK> vlinks; ///< if body is disabled, then geom is static (it can't be connected to a joint!)
                                ///< the pointer to this Link is the userdata
        vector<dJointID> vjoints;

        dSpaceID space;                     ///< space that contanis all the collision objects of this chain
        dJointGroupID jointgroup;
    };

    typedef void* (*GetInfoFn)(const KinBody* pbody);
    typedef void (*SynchornizeCallbackFn)(void* userdata, KINBODYINFO* pinfo);

    ODESpace(EnvironmentBase* penv, GetInfoFn infofn, bool bCreateJoints);
    ~ODESpace();

    bool InitEnvironment();
    void DestroyEnvironment();
    void* InitKinBody(KinBody* pbody); /// returns data to set
    bool DestroyKinBody(KinBody* pbody);

    bool Enable(const KinBody* pbody, bool bEnable);
    bool EnableLink(const KinBody::Link* pbody, bool bEnable);

    void Synchronize();
    void Synchronize(const KinBody* pbody);

    dSpaceID GetBodySpace(const KinBody* pbody);
    dBodyID GetLinkBody(const KinBody::Link* plink);
    dGeomID GetLinkGeom(const KinBody::Link* plink);
    dJointID GetJoint(const KinBody::Joint*);

    dWorldID GetWorld() const { return world; }
    dSpaceID GetSpace() const { return space; }
    dJointGroupID GetContactGroup() const { return contactgroup; }

    void SetSynchornizationCallback(SynchornizeCallbackFn synccallback, void* userdata);

private:
    void Synchronize(KINBODYINFO* pinfo);
    void RestoreJoints(KinBody* pbody);

    EnvironmentBase* _penv;
    dWorldID world;  ///< the dynamics world
    dSpaceID space;  ///< the collision world
    dJointGroupID contactgroup;
    GetInfoFn GetInfo;

    SynchornizeCallbackFn _synccallback;
    void* _userdata;

    bool _bCreateJoints;

    static bool s_bIsODEInitialized;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(ODESpace)
BOOST_TYPEOF_REGISTER_TYPE(ODESpace::KINBODYINFO)
BOOST_TYPEOF_REGISTER_TYPE(ODESpace::KINBODYINFO::LINK)
BOOST_TYPEOF_REGISTER_TYPE(dJointID)
#endif

#endif
