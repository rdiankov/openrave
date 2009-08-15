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
#ifndef RAVE_COLLISION_BULLET
#define RAVE_COLLISION_BULLET

#include "bulletspace.h"

class BulletCollisionChecker : public CollisionCheckerBase
{
private:
    static void* GetCollisionInfo(const KinBody* pbody) { return pbody->GetCollisionData(); }

    struct KinBodyFilterCallback : public btOverlapFilterCallback
    {
        KinBodyFilterCallback() : btOverlapFilterCallback(), _pbody0(NULL), _pbody1(NULL) {}
        
        virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
        {
            assert( static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL );
            assert( static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL );

            KinBody::Link* plink0 = (KinBody::Link*)static_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer();
            KinBody::Link* plink1 = (KinBody::Link*)static_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer();
            assert( plink0 != NULL );
            assert( plink1 != NULL );

            if( !plink0->IsEnabled() || !plink1->IsEnabled() ) {
                //assert(0);
                return false;
            }
            
            KinBody* pbody0 = plink0->GetParent();
            KinBody* pbody1 = plink1->GetParent();

            //RAVELOG(L"filter: %S:%S, this: %S (%d)\n", plink0->GetParent()->GetName(), plink1->GetParent()->GetName(), _pbody0->GetName(), pbody0 != pbody1 && (pbody0 == _pbody0 || pbody1 == _pbody0));

            if( _pbody1 ) {
                // wants collisions only between _pbody0 and _pbody1
                assert( _pbody0 != 0 );
                return (pbody0 == _pbody0 && pbody1 == _pbody1) || (pbody0 == _pbody1 && pbody1 == _pbody0);
            }

            assert( _pbody0 != NULL );
            
            // want collisions only with _pbody0
            return pbody0 != pbody1 && (pbody0 == _pbody0 || pbody1 == _pbody0);
        }

        const KinBody* _pbody0, *_pbody1;
    };

    struct LinkFilterCallback : public btOverlapFilterCallback
    {
        LinkFilterCallback() : btOverlapFilterCallback(), _pcollink0(NULL), _pcollink1(NULL) {}
        
        virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
        {
            assert( static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL );
            assert( static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL );

            KinBody::Link* plink0 = (KinBody::Link*)static_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer();
            KinBody::Link* plink1 = (KinBody::Link*)static_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer();
            assert( plink0 != NULL );
            assert( plink1 != NULL );            

            if( _pcollink1 != NULL ) {
                assert( _pcollink0 != NULL );
                // wants collisions only between specific links
                return (plink0 == _pcollink0 && plink1 == _pcollink1) || (plink0 == _pcollink1 && plink1 == _pcollink0);
            }
            else {
                if( plink0->GetParent() == plink1->GetParent() )
                    return false;
                return plink0 == _pcollink0 || plink1 == _pcollink0;
            }
        }

        const KinBody::Link *_pcollink0, *_pcollink1;
    };

    struct LinkAdjacentFilterCallback : public btOverlapFilterCallback
    {
    LinkAdjacentFilterCallback(const KinBody* pparent, const std::set<int>& setadjacency) : btOverlapFilterCallback(), _pparent(pparent), _setadjacency(setadjacency) {
        }
        
        virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
        {
            assert( static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL );
            assert( static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL );

            KinBody::Link* plink0 = (KinBody::Link*)static_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer();
            KinBody::Link* plink1 = (KinBody::Link*)static_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer();
            assert( plink0 != NULL );
            assert( plink1 != NULL );

            if( plink0->GetParent() != _pparent || plink1->GetParent() != _pparent )
                return false;
            // check if links are in adjacency list
            int index0 = plink0->GetIndex();
            int index1 = plink1->GetIndex();
            return _setadjacency.find(index0|(index1<<16)) != _setadjacency.end() ||
                _setadjacency.find(index1|(index0<<16)) != _setadjacency.end();
        }

        const KinBody* _pparent;
        const std::set<int>& _setadjacency;
    };

    struct KinBodyLinkFilterCallback : public btOverlapFilterCallback
    {
        KinBodyLinkFilterCallback() : btOverlapFilterCallback(), _pcollink(NULL), _pbody(NULL) {}
        
        virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
        {
            assert( _pcollink != NULL && _pbody != NULL );
            assert( static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL );
            assert( static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL );

            KinBody::Link* plink0 = (KinBody::Link*)static_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer();
            KinBody::Link* plink1 = (KinBody::Link*)static_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer();

            assert( plink0 != NULL );
            assert( plink1 != NULL );

            if( !plink0->IsEnabled() || !plink1->IsEnabled() )
                return false;

            KinBody* pbody0 = plink0->GetParent();
            KinBody* pbody1 = plink1->GetParent();

            if( (plink0 == _pcollink && pbody1 == _pbody) || (plink1 == _pcollink && pbody0 == _pbody) )
                return true;

            return false;
        }

        const KinBody::Link *_pcollink;
        const KinBody* _pbody;
    };

    struct KinBodyFilterExCallback : public btOverlapFilterCallback
    {
        KinBodyFilterExCallback() : btOverlapFilterCallback(), _pbody(NULL), _pvexcluded(NULL) {}
        
        virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
        {
            assert( static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL );
            assert( static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL );

            KinBody::Link* plink0 = (KinBody::Link*)static_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer();
            KinBody::Link* plink1 = (KinBody::Link*)static_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer();

            assert( plink0 != NULL );
            assert( plink1 != NULL );
            assert( _pvexcluded != NULL );
            
            if( !plink0->IsEnabled() || !plink1->IsEnabled() )
                return false;

            KinBody* pbody0 = plink0->GetParent();
            KinBody* pbody1 = plink1->GetParent();

            if( pbody0 == pbody1 || (pbody0 == _pbody && _pvexcluded->find(pbody1) != _pvexcluded->end()) 
                || (pbody1 == _pbody && _pvexcluded->find(pbody0) != _pvexcluded->end()))
                return false;

            return true;
        }

        const KinBody* _pbody;
        const std::set<KinBody *>* _pvexcluded;
    };

    class btOpenraveDispatcher : public btCollisionDispatcher
    {
    public:
        btOpenraveDispatcher(BulletCollisionChecker* pchecker, btCollisionConfiguration* collisionConfiguration)
            : btCollisionDispatcher(collisionConfiguration), _pchecker(pchecker) {
        }

        // need special collision function
        virtual bool needsCollision(btCollisionObject* co0, btCollisionObject* co1)
        {
            if( btCollisionDispatcher::needsCollision(co0, co1) ) {
                KinBody::Link* plink0 = (KinBody::Link*)co0->getUserPointer();
                KinBody::Link* plink1 = (KinBody::Link*)co1->getUserPointer();
                assert( plink0 != NULL );
                assert( plink1 != NULL );

                if( !plink0->IsEnabled() || !plink1->IsEnabled() )
                    return false;

                if( plink0->GetParent()->IsAttached(plink1->GetParent()) )
                    return false;

                //RAVEPRINT(L"%S:%S (%d)\n", plink0->GetName(), plink1->GetName(), _poverlapfilt->needBroadphaseCollision(co0->getBroadphaseHandle(), co1->getBroadphaseHandle()));

                // recheck the broadphase again
                return _poverlapfilt != NULL ? _poverlapfilt->needBroadphaseCollision(co0->getBroadphaseHandle(), co1->getBroadphaseHandle()) : true;
            }

            return false;
        }

        btOverlapFilterCallback* _poverlapfilt;
    private:
        BulletCollisionChecker* _pchecker;
    };

    struct	AllRayResultCallback : public btCollisionWorld::ClosestRayResultCallback
	{
        AllRayResultCallback(const btVector3&	rayFromWorld,const btVector3&	rayToWorld, const KinBody* pbodyonly)
             : btCollisionWorld::ClosestRayResultCallback(rayFromWorld, rayToWorld), _pbodyonly(pbodyonly) {}

        virtual bool needsCollision (btBroadphaseProxy *proxy0) const {
            KinBody::Link* plink = (KinBody::Link*)((btCollisionObject*)proxy0->m_clientObject)->getUserPointer();
            if( _pbodyonly != NULL && _pbodyonly != plink->GetParent() )
                return false;
            return plink->IsEnabled();
        }

        const KinBody* _pbodyonly;
	};

    struct	AllRayResultCallbackLink : public btCollisionWorld::ClosestRayResultCallback
	{
        AllRayResultCallbackLink(const btVector3& rayFromWorld, const btVector3& rayToWorld, const KinBody::Link* plink)
             : btCollisionWorld::ClosestRayResultCallback(rayFromWorld, rayToWorld), _plink(plink) {}

        virtual bool needsCollision (btBroadphaseProxy *proxy0) const {
            KinBody::Link* pcollink = (KinBody::Link*)((btCollisionObject*)proxy0->m_clientObject)->getUserPointer();
            if( pcollink != _plink )
                return false;
            return pcollink->IsEnabled();
        }

        const KinBody::Link* _plink;
	};

    bool CheckCollisionP(btOverlapFilterCallback* poverlapfilt, COLLISIONREPORT* pReport)
    {
        if( pReport != NULL ) pReport->Reset(_options);

        _world->getPairCache()->setOverlapFilterCallback(poverlapfilt);
        _dispatcher->_poverlapfilt = poverlapfilt;

        _world->performDiscreteCollisionDetection(); 

        // for some reason this is necessary, or else collisions will start disappearing
        _broadphase->calculateOverlappingPairs(_world->getDispatcher());

        int numManifolds = _world->getDispatcher()->getNumManifolds();
        bool bCollision = false;

        for (int i=0;i<numManifolds;i++) {
            btPersistentManifold* contactManifold = _world->getDispatcher()->getManifoldByIndexInternal(i);
            int numContacts = contactManifold->getNumContacts();

            btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
            btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
	
            KinBody::Link* plink0 = (KinBody::Link*)obA->getUserPointer();
            KinBody::Link* plink1 = (KinBody::Link*)obB->getUserPointer();

            //RAVEPRINT(L"col %S:%S (%d)\n", plink0->GetParent()->GetName(), plink1->GetParent()->GetName(), numContacts);

            if( numContacts == 0 )
                continue;

            if( pReport != NULL ) {
                pReport->numCols = numContacts;
                pReport->minDistance = 0;
                pReport->plink1 = plink0;
                pReport->plink2 = plink1;
            
                if( _options & OpenRAVE::CO_Contacts ) {
                    pReport->contacts.reserve(numContacts);
                    for (int j=0;j<numContacts;j++) {
                        btManifoldPoint& pt = contactManifold->getContactPoint(j);
                        btVector3 p = pt.getPositionWorldOnB();
                        btVector3 n = pt.m_normalWorldOnB;
                        pReport->contacts.push_back(COLLISIONREPORT::CONTACT(Vector(p[0],p[1],p[2]), Vector(n[0],n[1],n[2]), pt.m_distance1));
                    }
                }
            }

            bCollision = true;
            contactManifold->clearManifold();
        }

        return bCollision;
    }

public:
    BulletCollisionChecker(EnvironmentBase* penv) : CollisionCheckerBase(penv), bulletspace(penv, GetCollisionInfo, false), _options(0) {
    }
    virtual ~BulletCollisionChecker() {}
    
    virtual bool InitEnvironment()
    {
        // note: btAxisSweep3 is buggy
        _broadphase.reset(new btDbvtBroadphase()); // dynamic aabbs, supposedly good for changing scenes
        _collisionConfiguration.reset(new btDefaultCollisionConfiguration());
        _dispatcher.reset(new btOpenraveDispatcher(this, _collisionConfiguration.get()));
        _world.reset(new btCollisionWorld(_dispatcher.get(),_broadphase.get(),_collisionConfiguration.get()));

        if( !bulletspace.InitEnvironment(_world) )
            return false;
        FOREACHC(itbody, GetEnv()->GetBodies())
            InitKinBody(*itbody);

        return true;
    }

    virtual void DestroyEnvironment()
    {
        // go through all the KinBodies and destory their collision pointers
        FOREACHC(itbody, GetEnv()->GetBodies()) {
            if( !bulletspace.DestroyKinBody(*itbody) )
                RAVELOG(L"bullet collision checker failed to destroy body\n");
            SetCollisionData(*itbody, NULL);
        }
        bulletspace.DestroyEnvironment();
        if( !!_world && _world->getNumCollisionObjects() )
            RAVELOG_WARNA("world objects still left!\n");
    
        _world.reset();
        _dispatcher.reset();
        _collisionConfiguration.reset();
        _broadphase.reset();
    }

    virtual bool InitKinBody(KinBody* pbody)
    {
        void* pinfo = bulletspace.InitKinBody(pbody);
        SetCollisionData(pbody, pinfo);
        return pinfo != NULL;
    }

    virtual bool DestroyKinBody(KinBody* pbody)
    {
        bool bSuccess = bulletspace.DestroyKinBody(pbody);
        SetCollisionData(pbody, NULL);
        return bSuccess;
    }

    virtual bool SetCollisionOptions(int options)
    {
        _options = options;
        if( options & CO_Distance ) {
            RAVELOG_WARNA("bullet checker doesn't support CO_Distance\n");
            return false;
        }

        if( options & CO_Contacts ) {
            //setCollisionFlags btCollisionObject::CF_NO_CONTACT_RESPONSE - don't generate
        }
    
        return true;
    }

    virtual int GetCollisionOptions() const { return _options; }

    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) { return false; }

    virtual bool Enable(const KinBody* pbody, bool bEnable)
    {
        return bulletspace.Enable(pbody,bEnable);
    }
    virtual bool EnableLink(const KinBody::Link* plink, bool bEnable)
    {
        return bulletspace.EnableLink(plink,bEnable);
    }

    virtual bool CheckCollision(const KinBody* pbody, COLLISIONREPORT* pReport)
    {
        if( pbody == NULL || pbody->GetLinks().size() == 0 || !pbody->IsEnabled() ) {
            RAVELOG_WARNA("body %S not valid\n", pbody != NULL ? pbody->GetName() : L"()");
            return false;
        }

        bulletspace.Synchronize();

        bulletspace.Synchronize(pbody);

        _kinbodycallback._pbody0 = pbody;
        _kinbodycallback._pbody1 = NULL;
        if( CheckCollisionP(&_kinbodycallback, pReport) )
            return true;

        // check attached objects
        FOREACHC(itbody, pbody->GetAttached()) {
            _kinbodycallback._pbody0 = *itbody;
            _kinbodycallback._pbody1 = NULL;
            if( CheckCollisionP(&_kinbodycallback, pReport) )
                return true;
        }

        return false;
    }

    virtual bool CheckCollision(const KinBody* pbody1, const KinBody* pbody2, COLLISIONREPORT* pReport)
    {
        if( pbody1 == NULL || pbody1->GetLinks().size() == 0 || !pbody1->IsEnabled()  ) {
            RAVELOG(L"body1 %S not valid\n", pbody1 != NULL ? pbody1->GetName() : L"()");
            return false;
        }
        if( pbody2 == NULL || pbody2->GetLinks().size() == 0 || !pbody2->IsEnabled()  ) {
            RAVELOG(L"body2 %S not valid\n", pbody2 != NULL ? pbody2->GetName() : L"()");
            return false;
        }

        if( pbody1->IsAttached(pbody2) )
            return false;

        bulletspace.Synchronize();

        _kinbodycallback._pbody0 = pbody1;
        _kinbodycallback._pbody1 = pbody2;
        return CheckCollisionP(&_kinbodycallback, pReport);
    }
    virtual bool CheckCollision(const KinBody::Link* plink, COLLISIONREPORT* pReport)
    {
        if( plink == NULL || !plink->IsEnabled() ) {
            RAVELOG(L"calling collision on disabled link %S\n", plink != NULL ? plink->GetName() : L"()");
            return false;
        }

        bulletspace.Synchronize();

        _linkcallback._pcollink0 = plink;
        _linkcallback._pcollink1 = NULL;
        return CheckCollisionP(&_linkcallback, pReport);
    }

    virtual bool CheckCollision(const KinBody::Link* plink1, const KinBody::Link* plink2, COLLISIONREPORT* pReport)
    {
        if( plink1 == NULL || !plink1->IsEnabled() ) {
            RAVELOG(L"calling collision on disabled link1 %S\n", plink1 != NULL ? plink1->GetName() : L"()");
            return false;
        }
        if( plink2 == NULL || !plink2->IsEnabled() ) {
            RAVELOG(L"calling collision on disabled link2 %S\n", plink2 != NULL ? plink2->GetName() : L"()");
            return false;
        }

//        if( plink1->GetParent()->IsAttached(plink2->GetParent()) )
//            return false;

        bulletspace.Synchronize();
        _linkcallback._pcollink0 = plink1;
        _linkcallback._pcollink1 = plink2;
        return CheckCollisionP(&_linkcallback, pReport);
    }

    virtual bool CheckCollision(const KinBody::Link* plink, const KinBody* pbody, COLLISIONREPORT* pReport)
    {
        if( pbody == NULL || pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) { // 
            RAVELOG(L"body %S not valid\n", pbody != NULL ? pbody->GetName() : L"()");
            return false;
        }

        if( plink == NULL || !plink->IsEnabled() ) {
            RAVELOG(L"calling collision on disabled link %S\n", plink != NULL ? plink->GetName() : L"()");
            return false;
        }

//        if( plink->GetParent()->IsAttached(pbody) )
//            return false;

        bulletspace.Synchronize();

        _kinbodylinkcallback._pcollink = plink;
        _kinbodylinkcallback._pbody = pbody;
        return CheckCollisionP(&_kinbodylinkcallback, pReport);
    }
    
    virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, COLLISIONREPORT* pReport)
    {
        RAVELOG_FATAL(L"This type of collision checking is not yet implemented in the Bullet collision checker.\n"); 
        assert(0);
        bulletspace.Synchronize();
        return false;
    }

    virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, COLLISIONREPORT* pReport)
    {
        if( vbodyexcluded.size() == 0 && vlinkexcluded.size() == 0 )
            return CheckCollision(pbody, pReport);

        if( pbody == NULL || pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
            RAVELOG(L"body %S not valid\n", pbody != NULL ? pbody->GetName() : L"()");
            return false;
        }

        assert( vlinkexcluded.size() == 0 );
        bulletspace.Synchronize();
    
        _kinbodyexcallback._pbody = pbody;
        _kinbodyexcallback._pvexcluded = &vbodyexcluded;
        return CheckCollisionP(&_kinbodyexcallback, pReport);
    }

    virtual bool CheckCollision(const RAY& ray, const KinBody::Link* plink, COLLISIONREPORT* pReport)
    {
        if( plink == NULL || !plink->IsEnabled() ) {
            RAVELOG(L"calling collision on disabled link %S\n", plink != NULL ? plink->GetName() : L"()");
            return false;
        }

        if( pReport != NULL ) pReport->Reset();

        bulletspace.Synchronize();
        _world->updateAabbs();
    
        if( fabsf(sqrtf(ray.dir.lengthsqr3())-1) < 1e-4 )
            RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");

        btVector3 from = BulletSpace::GetBtVector(ray.pos);
        btVector3 to = BulletSpace::GetBtVector(ray.pos+ray.dir);
        AllRayResultCallbackLink rayCallback(from,to,plink);
        _world->rayTest(from,to, rayCallback);

        bool bCollision = rayCallback.hasHit();
        if( bCollision && pReport != NULL ) {
            pReport->numCols = 1;
            pReport->minDistance = (rayCallback.m_hitPointWorld-rayCallback.m_rayFromWorld).length();
            pReport->plink1 = (KinBody::Link*)rayCallback.m_collisionObject->getUserPointer();

            Vector p(rayCallback.m_hitPointWorld[0], rayCallback.m_hitPointWorld[1], rayCallback.m_hitPointWorld[2]);
            Vector n(rayCallback.m_hitNormalWorld[0], rayCallback.m_hitNormalWorld[1], rayCallback.m_hitNormalWorld[2]);
            pReport->contacts.push_back(COLLISIONREPORT::CONTACT(p,n.normalize3(),pReport->minDistance));
        }

        return bCollision;
    }    
    virtual bool CheckCollision(const RAY& ray, const KinBody* pbody, COLLISIONREPORT* pReport)
    {
        if( pbody == NULL || pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
            RAVELOG(L"body %S not valid\n", pbody != NULL ? pbody->GetName() : L"()");
            return false;
        }

        if( pReport != NULL ) pReport->Reset();

        bulletspace.Synchronize();
        _world->updateAabbs();
    
        if( fabsf(sqrtf(ray.dir.lengthsqr3())-1) < 1e-4 )
            RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");

        btVector3 from = BulletSpace::GetBtVector(ray.pos);
        btVector3 to = BulletSpace::GetBtVector(ray.pos+ray.dir);
        AllRayResultCallback rayCallback(from,to,pbody);
        _world->rayTest(from,to, rayCallback);

        bool bCollision = rayCallback.hasHit();
        if( bCollision && pReport != NULL ) {
            pReport->numCols = 1;
            pReport->minDistance = (rayCallback.m_hitPointWorld-rayCallback.m_rayFromWorld).length();
            pReport->plink1 = (KinBody::Link*)rayCallback.m_collisionObject->getUserPointer();

            Vector p(rayCallback.m_hitPointWorld[0], rayCallback.m_hitPointWorld[1], rayCallback.m_hitPointWorld[2]);
            Vector n(rayCallback.m_hitNormalWorld[0], rayCallback.m_hitNormalWorld[1], rayCallback.m_hitNormalWorld[2]);
            pReport->contacts.push_back(COLLISIONREPORT::CONTACT(p,n.normalize3(),pReport->minDistance));
        }

        return bCollision;
    }
    virtual bool CheckCollision(const RAY& ray, COLLISIONREPORT* pReport)
    {
        if( pReport != NULL ) pReport->Reset();

        bulletspace.Synchronize();
        _world->updateAabbs();

        if( fabsf(sqrtf(ray.dir.lengthsqr3())-1) < 1e-4 )
            RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
    
        btVector3 from = BulletSpace::GetBtVector(ray.pos);
        btVector3 to = BulletSpace::GetBtVector(ray.pos+ray.dir);
        AllRayResultCallback rayCallback(from,to,NULL);
        _world->rayTest(from,to, rayCallback);

        bool bCollision = rayCallback.hasHit();
        if( bCollision ) {
            pReport->numCols = 1;
            pReport->minDistance = (rayCallback.m_hitPointWorld-rayCallback.m_rayFromWorld).length();
            pReport->plink1 = (KinBody::Link*)rayCallback.m_collisionObject->getUserPointer();

            Vector p(rayCallback.m_hitPointWorld[0], rayCallback.m_hitPointWorld[1], rayCallback.m_hitPointWorld[2]);
            Vector n(rayCallback.m_hitNormalWorld[0], rayCallback.m_hitNormalWorld[1], rayCallback.m_hitNormalWorld[2]);
            pReport->contacts.push_back(COLLISIONREPORT::CONTACT(p,n.normalize3(),pReport->minDistance));
        }

        return bCollision;
    }
	
    //tolerance check
    virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, dReal tolerance)
    {
        RAVELOG_FATAL(L"This type of collision checking is not yet implemented in the Bullet collision checker.\n"); 
        assert(0);
        bulletspace.Synchronize();
        return false;
    }

    virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, dReal tolerance)
    {
        RAVELOG_FATAL(L"This type of collision checking is not yet implemented in the Bullet collision checker.\n"); 
        assert(0);
        bulletspace.Synchronize();
        return false;
    }

    virtual bool CheckSelfCollision(const KinBody* pbody, COLLISIONREPORT* pReport = NULL)
    {
        if( pbody == NULL || pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
            RAVELOG(L"body %S not valid\n", pbody != NULL ? pbody->GetName() : L"()");
            return false;
        }

        bulletspace.Synchronize();
        LinkAdjacentFilterCallback linkadjacent(pbody, pbody->GetNonAdjacentLinks());
        return CheckCollisionP(&linkadjacent, pReport);
    }

private:
    BulletSpace bulletspace;
    int _options;

    boost::shared_ptr<btBroadphaseInterface> _broadphase;
    boost::shared_ptr<btDefaultCollisionConfiguration> _collisionConfiguration;
    boost::shared_ptr<btOpenraveDispatcher> _dispatcher;
    boost::shared_ptr<btCollisionWorld> _world;

    KinBodyFilterCallback _kinbodycallback;
    LinkFilterCallback _linkcallback;
    KinBodyLinkFilterCallback _kinbodylinkcallback;
    KinBodyFilterExCallback _kinbodyexcallback;
};

#endif
