// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "bulletspace.h"

class BulletCollisionChecker : public CollisionCheckerBase
{
private:
    class CollisionFilterCallback : public OpenRAVEFilterCallback
    {
public:
        CollisionFilterCallback(CollisionCheckerBasePtr pchecker, KinBodyConstPtr pbody) : _pchecker(pchecker), _pbody(pbody)
        {
            _bActiveDOFs = !!(pchecker->GetCollisionOptions() & OpenRAVE::CO_ActiveDOFs);
        }

        bool IsActiveLink(KinBodyConstPtr pbody, int linkindex) const
        {
            if( !_bActiveDOFs || !_pbody || !_pbody->IsRobot()) {
                return true;
            }
            RobotBaseConstPtr probot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(_pbody);
            if( pbody != _pbody ) {
                // pbody could be attached to a robot's link that is not active!
                KinBody::LinkConstPtr pgrabbinglink = probot->IsGrabbing(pbody);
                if( !pgrabbinglink ) {
                    return true;
                }
                linkindex = pgrabbinglink->GetIndex();
            }
            if( _vactivelinks.size() == 0 ) {
                if( probot->GetAffineDOF() ) {
                    // enable everything
                    _vactivelinks.resize(probot->GetLinks().size(),1);
                }
                else {
                    _vactivelinks.resize(probot->GetLinks().size(),0);
                    // only check links that can potentially move with respect to each other
                    _vactivelinks.resize(probot->GetLinks().size(),0);
                    for(size_t i = 0; i < probot->GetLinks().size(); ++i) {
                        FOREACHC(itindex, probot->GetActiveDOFIndices()) {
                            if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),i) ) {
                                _vactivelinks[i] = 1;
                                break;
                            }
                        }
                    }
                }
            }
            return _vactivelinks.at(linkindex)>0;
        }

        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
            KinBodyPtr pbody0 = plink0->GetParent();
            KinBodyPtr pbody1 = plink1->GetParent();
            if( pbody0->IsAttached(pbody1) ) {
                return false;
            }
            if( !IsActiveLink(pbody0,plink0->GetIndex()) || !IsActiveLink(pbody1,plink1->GetIndex()) ) {
                return false;
            }

            // want collisions only with _pbody
            return _pbody->IsAttached(pbody0) || _pbody->IsAttached(pbody1);
        }

protected:
        CollisionCheckerBasePtr _pchecker;
        CollisionReportPtr _report;
        KinBodyConstPtr _pbody;
private:
        bool _bActiveDOFs;
        mutable vector<uint8_t> _vactivelinks;     ///< active links for _pbody, only valid if _pbody is a robot
    };

    class KinBodyFilterCallback : public CollisionFilterCallback
    {
public:
        KinBodyFilterCallback(CollisionCheckerBasePtr pchecker, KinBodyConstPtr pbody, KinBodyConstPtr pbody1=KinBodyConstPtr()) : CollisionFilterCallback(pchecker,pbody), _pbody1(pbody1) {
        }

        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
            if( CollisionFilterCallback::CheckLinks(plink0,plink1) ) {
                if( !!_pbody1 ) {
                    // wants collisions only between _pbody0 and _pbody1
                    KinBodyPtr pbody0 = plink0->GetParent();
                    KinBodyPtr pbody1 = plink1->GetParent();
                    return (_pbody->IsAttached(pbody0) && _pbody1->IsAttached(pbody1)) || (_pbody->IsAttached(pbody1) && _pbody1->IsAttached(pbody0));
                }
                return true;
            }

            return false;
        }

protected:
        KinBodyConstPtr _pbody1;
    };

    class KinBodyFilterExCallback : public KinBodyFilterCallback
    {
public:
        KinBodyFilterExCallback(CollisionCheckerBasePtr pchecker, KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded) : KinBodyFilterCallback(pchecker,pbody), _vbodyexcluded(vbodyexcluded) {
        }

        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
            KinBodyPtr pbody0 = plink0->GetParent();
            KinBodyPtr pbody1 = plink1->GetParent();
            if( find(_vbodyexcluded.begin(),_vbodyexcluded.end(),pbody0) != _vbodyexcluded.end() || find(_vbodyexcluded.begin(),_vbodyexcluded.end(),pbody1) != _vbodyexcluded.end() ) {
                return false;
            }
            return KinBodyFilterCallback::CheckLinks(plink0,plink1);
        }

        const std::vector<KinBodyConstPtr>& _vbodyexcluded;
    };

    class LinkFilterCallback : public OpenRAVEFilterCallback
    {
public:
        LinkFilterCallback() : OpenRAVEFilterCallback() {
        }

        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1)  const
        {
            if( !!_pcollink1 ) {
                BOOST_ASSERT( !!_pcollink0 );
                // wants collisions only between specific links
                return (plink0 == _pcollink0 && plink1 == _pcollink1) || (plink0 == _pcollink1 && plink1 == _pcollink0);
            }
            else {
                if( plink0->GetParent()->IsAttached(plink1->GetParent()) ) {
                    return false;
                }
                return plink0 == _pcollink0 || plink1 == _pcollink0;
            }
        }

        KinBody::LinkConstPtr _pcollink0, _pcollink1;
    };

    class LinkAdjacentFilterCallback : public OpenRAVEFilterCallback
    {
public:
        LinkAdjacentFilterCallback(KinBodyConstPtr pparent, const std::set<int>& setadjacency) : OpenRAVEFilterCallback(), _pparent(pparent), _setadjacency(setadjacency) {
        }

        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
            if(  plink0->GetParent() != _pparent ||plink1->GetParent() != _pparent ) {
                return false;
            }
            // check if links are in adjacency list
            int index0 = plink0->GetIndex();
            int index1 = plink1->GetIndex();
            return _setadjacency.find(index0|(index1<<16)) != _setadjacency.end() || _setadjacency.find(index1|(index0<<16)) != _setadjacency.end();
        }

        KinBodyConstPtr _pparent;
        const std::set<int>& _setadjacency;
    };

    class KinBodyLinkFilterCallback : public OpenRAVEFilterCallback
    {
public:
        KinBodyLinkFilterCallback() : OpenRAVEFilterCallback() {
        }

        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
            BOOST_ASSERT( !!_pcollink && !!_pbody );
            KinBodyPtr pbody0 = plink0->GetParent();
            KinBodyPtr pbody1 = plink1->GetParent();
            if( pbody0->IsAttached(pbody1) ) {
                return false;
            }
            return (plink0 == _pcollink && _pbody->IsAttached(pbody1)) || (plink1 == _pcollink && _pbody->IsAttached(pbody0));
        }

        KinBody::LinkConstPtr _pcollink;
        KinBodyConstPtr _pbody;
    };

    class btOpenraveDispatcher : public btCollisionDispatcher
    {
public:
        btOpenraveDispatcher(BulletCollisionChecker* pchecker, btCollisionConfiguration* collisionConfiguration) : btCollisionDispatcher(collisionConfiguration), _pchecker(pchecker) {
        }

        // need special collision function
        virtual bool needsCollision(btCollisionObject* co0, btCollisionObject* co1)
        {
            if( btCollisionDispatcher::needsCollision(co0, co1) ) {
                KinBody::LinkPtr plink0 = *(KinBody::LinkPtr*)co0->getUserPointer();
                KinBody::LinkPtr plink1 = *(KinBody::LinkPtr*)co1->getUserPointer();
                OpenRAVEFilterCallback* popenravefilt = dynamic_cast<OpenRAVEFilterCallback*>(_poverlapfilt);
                if( !!popenravefilt && !popenravefilt->CheckLinks(plink0,plink1) ) {
                    return false;
                }
                // recheck the broadphase again
                return _poverlapfilt != NULL ? _poverlapfilt->needBroadphaseCollision(co0->getBroadphaseHandle(), co1->getBroadphaseHandle()) : true;
            }

            return false;
        }

        btOverlapFilterCallback* _poverlapfilt;
private:
        BulletCollisionChecker* _pchecker;
    };

    class SetFilterScope
    {
public:
        SetFilterScope(boost::shared_ptr<btOpenraveDispatcher> dispatcher, btOverlappingPairCache* paircallback, btOverlapFilterCallback* filter) : _dispatcher(dispatcher), _paircallback(paircallback) {
            _paircallback->setOverlapFilterCallback(filter);
            _dispatcher->_poverlapfilt = filter;
        }
        virtual ~SetFilterScope()
        {
            _paircallback->setOverlapFilterCallback(NULL);
            _dispatcher->_poverlapfilt = NULL;
        }

private:
        boost::shared_ptr<btOpenraveDispatcher> _dispatcher;
        btOverlappingPairCache* _paircallback;
    };

    class AllRayResultCallback : public btCollisionWorld::RayResultCallback    //btCollisionWorld::ClosestRayResultCallback
    {
public:
        AllRayResultCallback(const btVector3&   rayFromWorld,const btVector3&   rayToWorld, KinBodyConstPtr pbodyonly) : RayResultCallback(), m_rayFromWorld(rayFromWorld), m_rayToWorld(rayToWorld), _pbodyonly(pbodyonly) {
        }
        //: btCollisionWorld::ClosestRayResultCallback(rayFromWorld, rayToWorld), _pbodyonly(pbodyonly) {}

        virtual bool needsCollision (btBroadphaseProxy *proxy0) const {
            KinBody::LinkPtr plink = *(KinBody::LinkPtr*)static_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer();
            if( !!_pbodyonly &&( _pbodyonly != plink->GetParent()) ) {
                return false;
            }
            //RAVELOG_INFO("clink: %s: %d\n",plink->GetParent()->GetName().c_str(),plink->IsEnabled());
            return plink->IsEnabled();
        }

        btVector3 m_rayFromWorld;                  //used to calculate hitPointWorld from hitFraction
        btVector3 m_rayToWorld;

        btVector3 m_hitNormalWorld;
        btVector3 m_hitPointWorld;

        virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace) {
            //caller already does the filter on the m_closestHitFraction
            if(rayResult.m_hitFraction <= m_closestHitFraction) {
                KinBody::LinkPtr plink = *(KinBody::LinkPtr*)static_cast<btCollisionObject*>(rayResult.m_collisionObject)->getUserPointer();
                if( !plink->IsEnabled() || (!!_pbodyonly &&( _pbodyonly != plink->GetParent()) ) ) {
                    return m_closestHitFraction;
                }
                //RAVELOG_INFO("clink: %s=%s: %d\n",plink->GetParent()->GetName().c_str(),_pbodyonly->GetName().c_str(),plink->IsEnabled());

                m_closestHitFraction = rayResult.m_hitFraction;
                m_collisionObject = rayResult.m_collisionObject;

                //RAVELOG_INFO("ray link: %s:%s\n",plink->GetParent()->GetName().c_str(),plink->GetName().c_str());
                if (normalInWorldSpace) {
                    m_hitNormalWorld = rayResult.m_hitNormalLocal;
                }
                else {
                    ///need to transform normal into worldspace
                    m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
                }
                m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
            }
            return m_closestHitFraction;
        }

        KinBodyConstPtr _pbodyonly;
    };

    static BulletSpace::KinBodyInfoPtr GetCollisionInfo(KinBodyConstPtr pbody) {
        return boost::dynamic_pointer_cast<BulletSpace::KinBodyInfo>(pbody->GetCollisionData());
    }

    bool CheckCollisionP(btOverlapFilterCallback* poverlapfilt, CollisionReportPtr report)
    {
        if( !!report ) {
            report->Reset(_options);
        }
        bool bHasCallbacks = GetEnv()->HasRegisteredCollisionCallbacks();
        std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;

        SetFilterScope filter(_dispatcher, _world->getPairCache(), poverlapfilt);
        _world->performDiscreteCollisionDetection();

        // for some reason this is necessary, or else collisions will start disappearing
        _broadphase->calculateOverlappingPairs(_world->getDispatcher());

        int numManifolds = _world->getDispatcher()->getNumManifolds();

        for (int i=0; i<numManifolds; i++) {
            btPersistentManifold* contactManifold = _world->getDispatcher()->getManifoldByIndexInternal(i);
            int numContacts = contactManifold->getNumContacts();

            btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
            btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

            KinBody::LinkPtr plink0 = *(KinBody::LinkPtr*)obA->getUserPointer();
            KinBody::LinkPtr plink1 = *(KinBody::LinkPtr*)obB->getUserPointer();

            if( numContacts == 0 ) {
                continue;
            }

            if( bHasCallbacks && !report ) {
                report.reset(new CollisionReport());
                report->Reset(_options);
            }

            if( !!report ) {
                report->numCols = numContacts;
                report->minDistance = 0;
                report->plink1 = plink0;
                report->plink2 = plink1;

                if( _options & OpenRAVE::CO_Contacts ) {
                    report->contacts.reserve(numContacts);
                    for (int j=0; j<numContacts; j++) {
                        btManifoldPoint& pt = contactManifold->getContactPoint(j);
                        btVector3 btp = pt.getPositionWorldOnB();
                        btVector3 btn = pt.m_normalWorldOnB;
                        Vector p(btp[0],btp[1],btp[2]), n(btn[0],btn[1],btn[2]);
                        dReal distance = pt.m_distance1;
                        if( !!plink1 && plink1->ValidateContactNormal(p,n) ) {
                            distance = -distance;
                        }
                        report->contacts.push_back(CollisionReport::CONTACT(p, n, distance));
                    }
                }
            }

            contactManifold->clearManifold();

            if( bHasCallbacks ) {
                if( listcallbacks.size() == 0 ) {
                    GetEnv()->GetRegisteredCollisionCallbacks(listcallbacks);
                }
                bool bDefaultAction = true;
                FOREACHC(itfn, listcallbacks) {
                    OpenRAVE::CollisionAction action = (*itfn)(report,false);
                    if( action != OpenRAVE::CA_DefaultAction ) {
                        report->Reset();
                        bDefaultAction = false;
                        break;
                    }
                }

                if( !bDefaultAction ) {
                    continue;
                }
            }

            // collision, so clear the rest and return
            for (int j=i+1; j<numManifolds; j++) {
                _world->getDispatcher()->getManifoldByIndexInternal(j)->clearManifold();
            }
            return true;
        }

        return false;
    }

public:
    BulletCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput) : CollisionCheckerBase(penv), bulletspace(new BulletSpace(penv, GetCollisionInfo, false)), _options(0) {
        __description = ":Interface Author: Rosen Diankov\n\nCollision checker from the `Bullet Physics Package <http://bulletphysics.org>`";
    }
    virtual ~BulletCollisionChecker() {
    }

    virtual bool InitEnvironment()
    {
        // note: btAxisSweep3 is buggy
        _broadphase.reset(new btDbvtBroadphase());     // dynamic aabbs, supposedly good for changing scenes
        _collisionConfiguration.reset(new btDefaultCollisionConfiguration());
        _dispatcher.reset(new btOpenraveDispatcher(this, _collisionConfiguration.get()));
        _world.reset(new btCollisionWorld(_dispatcher.get(),_broadphase.get(),_collisionConfiguration.get()));

        if( !bulletspace->InitEnvironment(_world) )
            return false;

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies)
        InitKinBody(*itbody);

        return true;
    }

    virtual void DestroyEnvironment()
    {
        // go through all the KinBodies and destory their collision pointers
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            SetCollisionData(*itbody, UserDataPtr());
        }
        bulletspace->DestroyEnvironment();
        if( !!_world && _world->getNumCollisionObjects() )
            RAVELOG_WARN("world objects still left!\n");

        _world.reset();
        _dispatcher.reset();
        _collisionConfiguration.reset();
        _broadphase.reset();
    }

    virtual bool InitKinBody(KinBodyPtr pbody)
    {
        UserDataPtr pinfo = bulletspace->InitKinBody(pbody);
        SetCollisionData(pbody, pinfo);
        return !!pinfo;
    }

    virtual bool SetCollisionOptions(int options)
    {
        _options = options;
        if( options & CO_Distance ) {
            //RAVELOG_WARN("bullet checker doesn't support CO_Distance\n");
            return false;
        }

        if( options & CO_Contacts ) {
            //setCollisionFlags btCollisionObject::CF_NO_CONTACT_RESPONSE - don't generate
        }
        return true;
    }

    virtual int GetCollisionOptions() const {
        return _options;
    }

    virtual bool Enable(KinBodyConstPtr pbody, bool bEnable)
    {
        return bulletspace->Enable(pbody,bEnable);
    }
    virtual bool EnableLink(KinBody::LinkConstPtr plink, bool bEnable)
    {
        return bulletspace->EnableLink(plink,bEnable);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if(( pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
            RAVELOG_WARN(str(boost::format("body %s not valid\n")%pbody->GetName()));
            return false;
        }

        bulletspace->Synchronize();

        KinBodyFilterCallback kinbodycallback(shared_collisionchecker(),pbody);
        if( CheckCollisionP(&kinbodycallback, report) ) {
            return true;
        }
        return false;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report)
    {
        if(( pbody1->GetLinks().size() == 0) || !pbody1->IsEnabled() ) {
            RAVELOG_WARN(str(boost::format("body1 %s not valid\n")%pbody1->GetName()));
            return false;
        }
        if(( pbody2 == NULL) ||( pbody2->GetLinks().size() == 0) || !pbody2->IsEnabled() ) {
            RAVELOG_WARN(str(boost::format("body2 %s not valid\n")%pbody2->GetName()));
            return false;
        }

        if( pbody1->IsAttached(pbody2) )
            return false;

        bulletspace->Synchronize();

        KinBodyFilterCallback kinbodycallback(shared_collisionchecker(),pbody1,pbody2);
        return CheckCollisionP(&kinbodycallback, report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        if( !plink->IsEnabled() ) {
            RAVELOG_VERBOSE(str(boost::format("calling collision on disabled link %s\n")%plink->GetName()));
            return false;
        }

        bulletspace->Synchronize();

        _linkcallback._pcollink0 = plink;
        _linkcallback._pcollink1.reset();
        return CheckCollisionP(&_linkcallback, report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report)
    {
        if( !plink1->IsEnabled() ) {
            RAVELOG_VERBOSE(str(boost::format("calling collision on disabled link1 %s\n")%plink1->GetName()));
            return false;
        }
        if( !plink2->IsEnabled() ) {
            RAVELOG_VERBOSE(str(boost::format("calling collision on disabled link2 %s\n")%plink2->GetName()));
            return false;
        }

        bulletspace->Synchronize();
        _linkcallback._pcollink0 = plink1;
        _linkcallback._pcollink1 = plink2;
        return CheckCollisionP(&_linkcallback, report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if(( pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {    //
            RAVELOG_WARN(str(boost::format("body %s not valid\n")%pbody->GetName()));
            return false;
        }

        if( !plink->IsEnabled() ) {
            RAVELOG_VERBOSE(str(boost::format("calling collision on disabled link %s\n")%plink->GetName()));
            return false;
        }

        bulletspace->Synchronize();

        KinBodyLinkFilterCallback kinbodylinkcallback;
        kinbodylinkcallback._pcollink = plink;
        kinbodylinkcallback._pbody = pbody;
        return CheckCollisionP(&kinbodylinkcallback, report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        RAVELOG_FATAL("This type of collision checking is not yet implemented in the Bullet collision checker.\n");
        BOOST_ASSERT(0);
        bulletspace->Synchronize();
        return false;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        if(( vbodyexcluded.size() == 0) &&( vlinkexcluded.size() == 0) )
            return CheckCollision(pbody, report);

        if(( pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
            RAVELOG_WARN(str(boost::format("body %s not valid\n")%pbody->GetName()));
            return false;
        }

        bulletspace->Synchronize();

        KinBodyFilterExCallback kinbodyexcallback(shared_collisionchecker(),pbody,vbodyexcluded);
        return CheckCollisionP(&kinbodyexcallback, report);
    }

    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        if( !plink->IsEnabled() ) {
            RAVELOG_VERBOSE(str(boost::format("calling collision on disabled link %s\n")%plink->GetName()));
            return false;
        }

        if( !!report )
            report->Reset();

        bulletspace->Synchronize();
        _world->updateAabbs();

        if( fabsf(sqrtf(ray.dir.lengthsqr3())-1) < 1e-4 )
            RAVELOG_DEBUG("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");

        btVector3 from = BulletSpace::GetBtVector(ray.pos);
        btVector3 to = BulletSpace::GetBtVector(ray.pos+ray.dir);
        btTransform rayFromTrans, rayToTrans;
        rayFromTrans.setIdentity();
        rayFromTrans.setOrigin(from);
        rayToTrans.setIdentity();
        rayToTrans.setOrigin(to);
        btCollisionWorld::ClosestRayResultCallback rayCallback(from,to);
        BulletSpace::KinBodyInfoPtr pinfo = GetCollisionInfo(plink->GetParent());
        boost::shared_ptr<BulletSpace::KinBodyInfo::LINK> plinkinfo = pinfo->vlinks.at(plink->GetIndex());
        _world->rayTestSingle(rayFromTrans,rayToTrans,plinkinfo->obj.get(),plinkinfo->shape.get(),plinkinfo->obj->getWorldTransform(),rayCallback);

        bool bCollision = rayCallback.hasHit();
        if( bCollision ) {
            if( GetEnv()->HasRegisteredCollisionCallbacks() && !report ) {
                report.reset(new CollisionReport());
                report->Reset(_options);
            }

            if( !!report ) {
                report->numCols = 1;
                report->minDistance = (rayCallback.m_hitPointWorld-rayCallback.m_rayFromWorld).length();
                report->plink1 = *(KinBody::LinkPtr*)rayCallback.m_collisionObject->getUserPointer();

                Vector p(rayCallback.m_hitPointWorld[0], rayCallback.m_hitPointWorld[1], rayCallback.m_hitPointWorld[2]);
                Vector n(rayCallback.m_hitNormalWorld[0], rayCallback.m_hitNormalWorld[1], rayCallback.m_hitNormalWorld[2]);
                report->contacts.push_back(CollisionReport::CONTACT(p,n.normalize3(),report->minDistance));
            }

            if( GetEnv()->HasRegisteredCollisionCallbacks() ) {
                std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;
                GetEnv()->GetRegisteredCollisionCallbacks(listcallbacks);

                FOREACHC(itfn, listcallbacks) {
                    OpenRAVE::CollisionAction action = (*itfn)(report,false);
                    if( action != OpenRAVE::CA_DefaultAction ) {
                        report->Reset();
                        return false;
                    }
                }
            }
        }

        return bCollision;
    }
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if(( pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
            RAVELOG_WARN(str(boost::format("body %s not valid\n")%pbody->GetName()));
            return false;
        }

        CollisionFilterCallback filtercallback(shared_collisionchecker(),pbody);
        if( !!report ) {
            report->Reset();
        }

        bulletspace->Synchronize();
        _world->updateAabbs();

        if( fabsf(sqrtf(ray.dir.lengthsqr3())-1) < 1e-4 )
            RAVELOG_DEBUG("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");

        btVector3 from = BulletSpace::GetBtVector(ray.pos);
        btVector3 to = BulletSpace::GetBtVector(ray.pos+ray.dir);
        btTransform rayFromTrans, rayToTrans;
        rayFromTrans.setIdentity();
        rayFromTrans.setOrigin(from);
        rayToTrans.setIdentity();
        rayToTrans.setOrigin(to);

        AllRayResultCallback rayCallback(from,to,pbody);
        BulletSpace::KinBodyInfoPtr pinfo = GetCollisionInfo(pbody);
        BOOST_ASSERT(pinfo->pbody == pbody );
        FOREACH(itlink,pinfo->vlinks) {
            if( (*itlink)->plink->IsEnabled() && filtercallback.IsActiveLink(pbody,(*itlink)->plink->GetIndex()) ) {
                _world->rayTestSingle(rayFromTrans,rayToTrans,(*itlink)->obj.get(),(*itlink)->shape.get(),(*itlink)->obj->getWorldTransform(),rayCallback);
            }
        }
        //_world->rayTest(from,to, rayCallback);

        bool bCollision = rayCallback.hasHit();
        if( bCollision ) {
            if( GetEnv()->HasRegisteredCollisionCallbacks() && !report ) {
                report.reset(new CollisionReport());
                report->Reset(_options);
            }

            if( !!report ) {
                report->numCols = 1;
                report->minDistance = (rayCallback.m_hitPointWorld-rayCallback.m_rayFromWorld).length();
                report->plink1 = *(KinBody::LinkPtr*)rayCallback.m_collisionObject->getUserPointer();

                Vector p(rayCallback.m_hitPointWorld[0], rayCallback.m_hitPointWorld[1], rayCallback.m_hitPointWorld[2]);
                Vector n(rayCallback.m_hitNormalWorld[0], rayCallback.m_hitNormalWorld[1], rayCallback.m_hitNormalWorld[2]);
                report->contacts.push_back(CollisionReport::CONTACT(p,n.normalize3(),report->minDistance));
            }

            if( GetEnv()->HasRegisteredCollisionCallbacks() ) {
                std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;
                GetEnv()->GetRegisteredCollisionCallbacks(listcallbacks);

                FOREACHC(itfn, listcallbacks) {
                    OpenRAVE::CollisionAction action = (*itfn)(report,false);
                    if( action != OpenRAVE::CA_DefaultAction ) {
                        report->Reset();
                        return false;
                    }
                }
            }
        }

        return bCollision;
    }

    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report)
    {
        if( !!report ) {
            report->Reset();
        }
        bulletspace->Synchronize();
        _world->updateAabbs();

        if( fabsf(sqrtf(ray.dir.lengthsqr3())-1) < 1e-4 ) {
            RAVELOG_DEBUG("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
        }
        // unfortunately, the bullet ray checker cannot handle disabled bodies properly, so have to move all of them away
        list<boost::shared_ptr<KinBody::KinBodyStateSaver> > listsavers;
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        Vector vnormdir = ray.dir*(1/RaveSqrt(ray.dir.lengthsqr3()));
        FOREACH(it,vbodies) {
            if( !(*it)->IsEnabled() ) {
                listsavers.push_back(boost::shared_ptr<KinBody::KinBodyStateSaver>(new KinBody::KinBodyStateSaver(*it)));
                AABB ab = (*it)->ComputeAABB();
                Transform t; t.trans = ray.pos-vnormdir*4*RaveSqrt(ab.extents.lengthsqr3());
                (*it)->SetTransform(t);
            }
        }

        btVector3 from = BulletSpace::GetBtVector(ray.pos);
        btVector3 to = BulletSpace::GetBtVector(ray.pos+ray.dir);
        AllRayResultCallback rayCallback(from,to,KinBodyConstPtr());
        _world->rayTest(from,to, rayCallback);

        bool bCollision = rayCallback.hasHit();
        if( bCollision ) {
            if( GetEnv()->HasRegisteredCollisionCallbacks() && !report ) {
                report.reset(new CollisionReport());
                report->Reset(_options);
            }

            if( !!report ) {
                report->numCols = 1;
                report->minDistance = (rayCallback.m_hitPointWorld-rayCallback.m_rayFromWorld).length();
                report->plink1 = *(KinBody::LinkPtr*)rayCallback.m_collisionObject->getUserPointer();

                Vector p(rayCallback.m_hitPointWorld[0], rayCallback.m_hitPointWorld[1], rayCallback.m_hitPointWorld[2]);
                Vector n(rayCallback.m_hitNormalWorld[0], rayCallback.m_hitNormalWorld[1], rayCallback.m_hitNormalWorld[2]);
                report->contacts.push_back(CollisionReport::CONTACT(p,n.normalize3(),report->minDistance));
            }

            if( GetEnv()->HasRegisteredCollisionCallbacks() ) {
                std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;
                GetEnv()->GetRegisteredCollisionCallbacks(listcallbacks);

                FOREACHC(itfn, listcallbacks) {
                    OpenRAVE::CollisionAction action = (*itfn)(report,false);
                    if( action != OpenRAVE::CA_DefaultAction ) {
                        report->Reset();
                        return false;
                    }
                }
            }
        }

        return bCollision;
    }

    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if(( pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
            RAVELOG_WARN(str(boost::format("body %s not valid\n")%pbody->GetName()));
            return false;
        }


        int adjacentoptions = KinBody::AO_Enabled;
        if( (_options&OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
            adjacentoptions |= KinBody::AO_ActiveDOFs;
        }
        LinkAdjacentFilterCallback linkadjacent(pbody, pbody->GetNonAdjacentLinks(adjacentoptions));
        bulletspace->Synchronize(); // call after GetNonAdjacentLinks since it can modify the body, even though it is const!
        bool bCollision = CheckCollisionP(&linkadjacent, report);
        return bCollision;
    }

    virtual void SetTolerance(dReal tolerance) {
        RAVELOG_WARN("not implemented\n");
    }

private:
    boost::shared_ptr<BulletSpace> bulletspace;
    int _options;

    boost::shared_ptr<btBroadphaseInterface> _broadphase;
    boost::shared_ptr<btDefaultCollisionConfiguration> _collisionConfiguration;
    boost::shared_ptr<btOpenraveDispatcher> _dispatcher;
    boost::shared_ptr<btCollisionWorld> _world;

    LinkFilterCallback _linkcallback;
};

CollisionCheckerBasePtr CreateBulletCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput)
{
    return CollisionCheckerBasePtr(new BulletCollisionChecker(penv,sinput));
}
