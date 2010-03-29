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
    static boost::shared_ptr<void> GetCollisionInfo(KinBodyConstPtr pbody) { return pbody->GetCollisionData(); }

    struct OpenRAVEFilterCallback : public btOverlapFilterCallback
    {
        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const = 0;
        virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
        {
            BOOST_ASSERT( static_cast<btCollisionObject*>(proxy0->m_clientObject) != NULL );
            BOOST_ASSERT( static_cast<btCollisionObject*>(proxy1->m_clientObject) != NULL );

            KinBody::LinkPtr plink0 = *(KinBody::LinkPtr*)static_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer();
            KinBody::LinkPtr plink1 = *(KinBody::LinkPtr*)static_cast<btCollisionObject*>(proxy1->m_clientObject)->getUserPointer();
            if( !plink0->IsEnabled() || !plink1->IsEnabled() )
                return false;
            return CheckLinks(plink0,plink1);
        }
    };

    struct KinBodyFilterCallback : public OpenRAVEFilterCallback
    {
    KinBodyFilterCallback() : OpenRAVEFilterCallback() {}
        
        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
            KinBodyPtr pbody0 = plink0->GetParent();
            KinBodyPtr pbody1 = plink1->GetParent();
            if( pbody0->IsAttached(pbody1) )
                return false;

            if( !!_pbody1 ) {
                // wants collisions only between _pbody0 and _pbody1
                BOOST_ASSERT( !!_pbody0 );
                return (_pbody0->IsAttached(pbody0) && _pbody1->IsAttached(pbody1)) || (_pbody0->IsAttached(pbody1) && _pbody1->IsAttached(pbody0));
            }

            BOOST_ASSERT( !!_pbody0 );
            
            // want collisions only with _pbody0
            return _pbody0->IsAttached(pbody0) || _pbody0->IsAttached(pbody1);
        }

        KinBodyConstPtr _pbody0, _pbody1;
    };

    struct LinkFilterCallback : public OpenRAVEFilterCallback
    {
        LinkFilterCallback() : OpenRAVEFilterCallback() {}
        
        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1)  const
        {
            if( !!_pcollink1 ) {
                BOOST_ASSERT( !!_pcollink0 );
                // wants collisions only between specific links
                return (plink0 == _pcollink0 && plink1 == _pcollink1) || (plink0 == _pcollink1 && plink1 == _pcollink0);
            }
            else {
                if( plink0->GetParent() == plink1->GetParent() )
                    return false;
                return plink0 == _pcollink0 || plink1 == _pcollink0;
            }
        }

        KinBody::LinkConstPtr _pcollink0, _pcollink1;
    };

    struct LinkAdjacentFilterCallback : public OpenRAVEFilterCallback
    {
    LinkAdjacentFilterCallback(KinBodyConstPtr pparent, const std::set<int>& setadjacency) : OpenRAVEFilterCallback(), _pparent(pparent), _setadjacency(setadjacency) {
        }
        
        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
            if( plink0->GetParent() != _pparent || plink1->GetParent() != _pparent )
                return false;
            // check if links are in adjacency list
            int index0 = plink0->GetIndex();
            int index1 = plink1->GetIndex();
            return _setadjacency.find(index0|(index1<<16)) != _setadjacency.end() ||
                _setadjacency.find(index1|(index0<<16)) != _setadjacency.end();
        }
        
        KinBodyConstPtr _pparent;
        const std::set<int>& _setadjacency;
    };

    struct KinBodyLinkFilterCallback : public OpenRAVEFilterCallback
    {
        KinBodyLinkFilterCallback() : OpenRAVEFilterCallback() {}
        
        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
            BOOST_ASSERT( !!_pcollink && !!_pbody );
            KinBodyPtr pbody0 = plink0->GetParent();
            KinBodyPtr pbody1 = plink1->GetParent();

            if( pbody0->IsAttached(pbody1) )
                return false;
            
            return (plink0 == _pcollink && _pbody->IsAttached(pbody1)) || (plink1 == _pcollink && _pbody->IsAttached(pbody0));
        }

        KinBody::LinkConstPtr _pcollink;
        KinBodyConstPtr _pbody;
    };

    struct KinBodyFilterExCallback : public OpenRAVEFilterCallback
    {
        KinBodyFilterExCallback() : OpenRAVEFilterCallback(), _pvexcluded(NULL) {}
        
        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
            BOOST_ASSERT( _pvexcluded != NULL );
            KinBodyPtr pbody0 = plink0->GetParent();
            KinBodyPtr pbody1 = plink1->GetParent();

            if( find(_pvexcluded->begin(),_pvexcluded->end(),pbody0) != _pvexcluded->end() ||
                find(_pvexcluded->begin(),_pvexcluded->end(),pbody1) != _pvexcluded->end())
                return false;

            if( pbody0->IsAttached(pbody1) )
                return false;

            // want collisions only with _pbody0
            return _pbody->IsAttached(pbody0) || _pbody->IsAttached(pbody1);
        }

        KinBodyConstPtr _pbody;
        const std::vector<KinBodyConstPtr>* _pvexcluded;
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
                KinBody::LinkPtr plink0 = *(KinBody::LinkPtr*)co0->getUserPointer();
                KinBody::LinkPtr plink1 = *(KinBody::LinkPtr*)co1->getUserPointer();
                OpenRAVEFilterCallback* popenravefilt = dynamic_cast<OpenRAVEFilterCallback*>(_poverlapfilt);
                if( !!popenravefilt && !popenravefilt->CheckLinks(plink0,plink1) )
                    return false;
                // recheck the broadphase again
                return _poverlapfilt != NULL ? _poverlapfilt->needBroadphaseCollision(co0->getBroadphaseHandle(), co1->getBroadphaseHandle()) : true;
            }

            return false;
        }

        btOverlapFilterCallback* _poverlapfilt;
    private:
        BulletCollisionChecker* _pchecker;
    };

    struct	AllRayResultCallback : public btCollisionWorld::RayResultCallback//btCollisionWorld::ClosestRayResultCallback
	{
        AllRayResultCallback(const btVector3&	rayFromWorld,const btVector3&	rayToWorld, KinBodyConstPtr pbodyonly)
            : RayResultCallback(), m_rayFromWorld(rayFromWorld), m_rayToWorld(rayToWorld), _pbodyonly(pbodyonly) {}
        //: btCollisionWorld::ClosestRayResultCallback(rayFromWorld, rayToWorld), _pbodyonly(pbodyonly) {}

        virtual bool needsCollision (btBroadphaseProxy *proxy0) const {
            KinBody::LinkPtr plink = *(KinBody::LinkPtr*)static_cast<btCollisionObject*>(proxy0->m_clientObject)->getUserPointer();
            if( !!_pbodyonly && _pbodyonly != plink->GetParent() )
                return false;
            //RAVELOG_INFO("clink: %s=%s: %d\n",plink->GetParent()->GetName().c_str(),_pbodyonly->GetName().c_str(),plink->IsEnabled());
            return plink->IsEnabled();
        }

		btVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
		btVector3	m_rayToWorld;

		btVector3	m_hitNormalWorld;
		btVector3	m_hitPointWorld;

        virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace) {
            //caller already does the filter on the m_closestHitFraction
            BOOST_ASSERT(rayResult.m_hitFraction <= m_closestHitFraction);
            KinBody::LinkPtr plink = *(KinBody::LinkPtr*)static_cast<btCollisionObject*>(rayResult.m_collisionObject)->getUserPointer();            
            if( !plink->IsEnabled() || (!!_pbodyonly && _pbodyonly != plink->GetParent()) )
                return false;
            //RAVELOG_INFO("clink: %s=%s: %d\n",plink->GetParent()->GetName().c_str(),_pbodyonly->GetName().c_str(),plink->IsEnabled());
            
            m_closestHitFraction = rayResult.m_hitFraction;
            m_collisionObject = rayResult.m_collisionObject;

            //RAVELOG_INFO("ray link: %s:%s\n",plink->GetParent()->GetName().c_str(),plink->GetName().c_str());
            if (normalInWorldSpace)
            {
                m_hitNormalWorld = rayResult.m_hitNormalLocal;
            } else
            {
                ///need to transform normal into worldspace
                m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
            }
            m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
            return rayResult.m_hitFraction;
        }

        KinBodyConstPtr _pbodyonly;
	};

    bool CheckCollisionP(btOverlapFilterCallback* poverlapfilt, CollisionReportPtr report)
    {
        if( !!report )
            report->Reset(_options);
        bool bHasCallbacks = GetEnv()->HasRegisteredCollisionCallbacks();
        std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;

        _world->getPairCache()->setOverlapFilterCallback(poverlapfilt);
        _dispatcher->_poverlapfilt = poverlapfilt;

        _world->performDiscreteCollisionDetection(); 

        // for some reason this is necessary, or else collisions will start disappearing
        _broadphase->calculateOverlappingPairs(_world->getDispatcher());

        int numManifolds = _world->getDispatcher()->getNumManifolds();
        _dispatcher->_poverlapfilt = NULL;

        for (int i=0;i<numManifolds;i++) {
            btPersistentManifold* contactManifold = _world->getDispatcher()->getManifoldByIndexInternal(i);
            int numContacts = contactManifold->getNumContacts();

            btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
            btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
	
            KinBody::LinkPtr plink0 = *(KinBody::LinkPtr*)obA->getUserPointer();
            KinBody::LinkPtr plink1 = *(KinBody::LinkPtr*)obB->getUserPointer();

            if( numContacts == 0 )
                continue;

            if( bHasCallbacks && !report ) {
                report.reset(new COLLISIONREPORT());
                report->Reset(_options);
            }

            if( !!report ) {
                report->numCols = numContacts;
                report->minDistance = 0;
                report->plink1 = plink0;
                report->plink2 = plink1;
            
                if( _options & OpenRAVE::CO_Contacts ) {
                    report->contacts.reserve(numContacts);
                    for (int j=0;j<numContacts;j++) {
                        btManifoldPoint& pt = contactManifold->getContactPoint(j);
                        btVector3 p = pt.getPositionWorldOnB();
                        btVector3 n = pt.m_normalWorldOnB;
                        report->contacts.push_back(COLLISIONREPORT::CONTACT(Vector(p[0],p[1],p[2]), Vector(n[0],n[1],n[2]), pt.m_distance1));
                    }
                }
            }

            contactManifold->clearManifold();

            if( bHasCallbacks ) {
                if( listcallbacks.size() == 0 )
                    GetEnv()->GetRegisteredCollisionCallbacks(listcallbacks);
                bool bDefaultAction = true;
                FOREACHC(itfn, listcallbacks) {
                    OpenRAVE::CollisionAction action = (*itfn)(report,false);
                    if( action != OpenRAVE::CA_DefaultAction ) {
                        report->Reset();
                        bDefaultAction = false;
                        break;
                    }
                }

                if( !bDefaultAction )
                    continue;
            }
            
            // collision, so clear the rest and return
            for (int j=i+1;j<numManifolds;j++)
                _world->getDispatcher()->getManifoldByIndexInternal(j)->clearManifold();
            return true;
        }

        return false;
    }

public:
 BulletCollisionChecker(EnvironmentBasePtr penv) : CollisionCheckerBase(penv), bulletspace(new BulletSpace(penv, GetCollisionInfo, false)), _options(0) {
    }
    virtual ~BulletCollisionChecker() {}
    
    virtual bool InitEnvironment()
    {
        // note: btAxisSweep3 is buggy
        _broadphase.reset(new btDbvtBroadphase()); // dynamic aabbs, supposedly good for changing scenes
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
        _world->getPairCache()->setOverlapFilterCallback(NULL);
        _dispatcher->_poverlapfilt = NULL;

        // go through all the KinBodies and destory their collision pointers
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies)
            SetCollisionData(*itbody, boost::shared_ptr<void>());
        bulletspace->DestroyEnvironment();
        if( !!_world && _world->getNumCollisionObjects() )
            RAVELOG_WARNA("world objects still left!\n");
    
        _world.reset();
        _dispatcher.reset();
        _collisionConfiguration.reset();
        _broadphase.reset();
    }

    virtual bool InitKinBody(KinBodyPtr pbody)
    {
        boost::shared_ptr<void> pinfo = bulletspace->InitKinBody(pbody);
        SetCollisionData(pbody, pinfo);
        return !!pinfo;
    }

    virtual bool SetCollisionOptions(int options)
    {
        _options = options;
        if( options & CO_Distance ) {
            //RAVELOG_WARNA("bullet checker doesn't support CO_Distance\n");
            return false;
        }

        if( options & CO_Contacts ) {
            //setCollisionFlags btCollisionObject::CF_NO_CONTACT_RESPONSE - don't generate
        }
    
        return true;
    }

    virtual int GetCollisionOptions() const { return _options; }
    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) { return false; }

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
        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled() ) {
            RAVELOG_WARNA(str(boost::format("body %s not valid\n")%pbody->GetName()));
            return false;
        }

        bulletspace->Synchronize();

        _kinbodycallback._pbody0 = pbody;
        _kinbodycallback._pbody1.reset();
        if( CheckCollisionP(&_kinbodycallback, report) )
            return true;

        // check attached objects
        std::set<KinBodyPtr> vattached;
        pbody->GetAttached(vattached);
        FOREACHC(itbody, vattached) {
            _kinbodycallback._pbody0 = *itbody;
            _kinbodycallback._pbody1.reset();
            if( CheckCollisionP(&_kinbodycallback, report) )
                return true;
        }

        return false;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report)
    {
        if( pbody1->GetLinks().size() == 0 || !pbody1->IsEnabled()  ) {
            RAVELOG_WARNA(str(boost::format("body1 %s not valid\n")%pbody1->GetName()));
            return false;
        }
        if( pbody2 == NULL || pbody2->GetLinks().size() == 0 || !pbody2->IsEnabled()  ) {
            RAVELOG_WARNA(str(boost::format("body2 %s not valid\n")%pbody2->GetName()));
            return false;
        }

        if( pbody1->IsAttached(pbody2) )
            return false;

        bulletspace->Synchronize();

        _kinbodycallback._pbody0 = pbody1;
        _kinbodycallback._pbody1 = pbody2;
        return CheckCollisionP(&_kinbodycallback, report);
    }
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        if( !plink->IsEnabled() ) {
            RAVELOG_WARNA(str(boost::format("calling collision on disabled link %s\n")%plink->GetName()));
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
            RAVELOG_WARNA(str(boost::format("calling collision on disabled link1 %s\n")%plink1->GetName()));
            return false;
        }
        if( !plink2->IsEnabled() ) {
            RAVELOG_WARNA(str(boost::format("calling collision on disabled link2 %s\n")%plink2->GetName()));
            return false;
        }

        bulletspace->Synchronize();
        _linkcallback._pcollink0 = plink1;
        _linkcallback._pcollink1 = plink2;
        return CheckCollisionP(&_linkcallback, report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) { // 
            RAVELOG_WARNA(str(boost::format("body %s not valid\n")%pbody->GetName()));
            return false;
        }

        if( !plink->IsEnabled() ) {
            RAVELOG_WARNA(str(boost::format("calling collision on disabled link %s\n")%plink->GetName()));
            return false;
        }

        bulletspace->Synchronize();

        _kinbodylinkcallback._pcollink = plink;
        _kinbodylinkcallback._pbody = pbody;
        return CheckCollisionP(&_kinbodylinkcallback, report);
    }
    
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        RAVELOG_FATALA("This type of collision checking is not yet implemented in the Bullet collision checker.\n"); 
        BOOST_ASSERT(0);
        bulletspace->Synchronize();
        return false;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        if( vbodyexcluded.size() == 0 && vlinkexcluded.size() == 0 )
            return CheckCollision(pbody, report);

        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
            RAVELOG_WARNA(str(boost::format("body %s not valid\n")%pbody->GetName()));
            return false;
        }

        bulletspace->Synchronize();
    
        _kinbodyexcallback._pbody = pbody;
        _kinbodyexcallback._pvexcluded = &vbodyexcluded;
        return CheckCollisionP(&_kinbodyexcallback, report);
    }

    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        if( !plink->IsEnabled() ) {
            RAVELOG_WARNA(str(boost::format("calling collision on disabled link %s\n")%plink->GetName()));
            return false;
        }

        if( !!report )
            report->Reset();

        bulletspace->Synchronize();
        _world->updateAabbs();
    
        if( fabsf(sqrtf(ray.dir.lengthsqr3())-1) < 1e-4 )
            RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");

        btVector3 from = BulletSpace::GetBtVector(ray.pos);
        btVector3 to = BulletSpace::GetBtVector(ray.pos+ray.dir);
		btTransform rayFromTrans, rayToTrans;
        rayFromTrans.setIdentity();
		rayFromTrans.setOrigin(from);
		rayToTrans.setIdentity();
		rayToTrans.setOrigin(to);
        btCollisionWorld::ClosestRayResultCallback rayCallback(from,to);
        BulletSpace::KinBodyInfoPtr pinfo = boost::static_pointer_cast<BulletSpace::KinBodyInfo>(GetCollisionInfo(plink->GetParent()));
        boost::shared_ptr<BulletSpace::KinBodyInfo::LINK> plinkinfo = pinfo->vlinks.at(plink->GetIndex());
        _world->rayTestSingle(rayFromTrans,rayToTrans,plinkinfo->obj.get(),plinkinfo->shape.get(),plinkinfo->obj->getWorldTransform(),rayCallback);

        bool bCollision = rayCallback.hasHit();
        if( bCollision ) {
            if( GetEnv()->HasRegisteredCollisionCallbacks() && !report ) {
                report.reset(new COLLISIONREPORT());
                report->Reset(_options);
            }

            if( !!report ) {
                report->numCols = 1;
                report->minDistance = (rayCallback.m_hitPointWorld-rayCallback.m_rayFromWorld).length();
                report->plink1 = *(KinBody::LinkPtr*)rayCallback.m_collisionObject->getUserPointer();
                
                Vector p(rayCallback.m_hitPointWorld[0], rayCallback.m_hitPointWorld[1], rayCallback.m_hitPointWorld[2]);
                Vector n(rayCallback.m_hitNormalWorld[0], rayCallback.m_hitNormalWorld[1], rayCallback.m_hitNormalWorld[2]);
                report->contacts.push_back(COLLISIONREPORT::CONTACT(p,n.normalize3(),report->minDistance));
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
        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
            RAVELOG_WARNA(str(boost::format("body %s not valid\n")%pbody->GetName()));
            return false;
        }

        if( !!report )
            report->Reset();

        bulletspace->Synchronize();
        _world->updateAabbs();

        if( fabsf(sqrtf(ray.dir.lengthsqr3())-1) < 1e-4 )
            RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");

        btVector3 from = BulletSpace::GetBtVector(ray.pos);
        btVector3 to = BulletSpace::GetBtVector(ray.pos+ray.dir);
		btTransform rayFromTrans, rayToTrans;
        rayFromTrans.setIdentity();
		rayFromTrans.setOrigin(from);
		rayToTrans.setIdentity();
		rayToTrans.setOrigin(to);

        AllRayResultCallback rayCallback(from,to,pbody);
        BulletSpace::KinBodyInfoPtr pinfo = boost::static_pointer_cast<BulletSpace::KinBodyInfo>(GetCollisionInfo(pbody));
        BOOST_ASSERT(pinfo->pbody == pbody );
        FOREACH(itlink,pinfo->vlinks) {
            if( (*itlink)->plink->IsEnabled() ) {
                _world->rayTestSingle(rayFromTrans,rayToTrans,(*itlink)->obj.get(),(*itlink)->shape.get(),(*itlink)->obj->getWorldTransform(),rayCallback);
            }
        }
        //_world->rayTest(from,to, rayCallback);

        bool bCollision = rayCallback.hasHit();
        if( bCollision ) {
            if( GetEnv()->HasRegisteredCollisionCallbacks() && !report ) {
                report.reset(new COLLISIONREPORT());
                report->Reset(_options);
            }

            if( !!report ) {
                report->numCols = 1;
                report->minDistance = (rayCallback.m_hitPointWorld-rayCallback.m_rayFromWorld).length();
                report->plink1 = *(KinBody::LinkPtr*)rayCallback.m_collisionObject->getUserPointer();
                
                Vector p(rayCallback.m_hitPointWorld[0], rayCallback.m_hitPointWorld[1], rayCallback.m_hitPointWorld[2]);
                Vector n(rayCallback.m_hitNormalWorld[0], rayCallback.m_hitNormalWorld[1], rayCallback.m_hitNormalWorld[2]);
                report->contacts.push_back(COLLISIONREPORT::CONTACT(p,n.normalize3(),report->minDistance));
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
        if( !!report )
            report->Reset();

        bulletspace->Synchronize();
        _world->updateAabbs();

        if( fabsf(sqrtf(ray.dir.lengthsqr3())-1) < 1e-4 )
            RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");

        // unfortunately, the bullet ray checker cannot handle enabled bodies properly, so have to move all of them away
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
                report.reset(new COLLISIONREPORT());
                report->Reset(_options);
            }
            
            if( !!report ) {
                report->numCols = 1;
                report->minDistance = (rayCallback.m_hitPointWorld-rayCallback.m_rayFromWorld).length();
                report->plink1 = *(KinBody::LinkPtr*)rayCallback.m_collisionObject->getUserPointer();
                
                Vector p(rayCallback.m_hitPointWorld[0], rayCallback.m_hitPointWorld[1], rayCallback.m_hitPointWorld[2]);
                Vector n(rayCallback.m_hitNormalWorld[0], rayCallback.m_hitNormalWorld[1], rayCallback.m_hitNormalWorld[2]);
                report->contacts.push_back(COLLISIONREPORT::CONTACT(p,n.normalize3(),report->minDistance));
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
        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
            RAVELOG_WARNA(str(boost::format("body %s not valid\n")%pbody->GetName()));
            return false;
        }

        bulletspace->Synchronize();
        LinkAdjacentFilterCallback linkadjacent(pbody, pbody->GetNonAdjacentLinks());
        bool bCollision = CheckCollisionP(&linkadjacent, report);
        // things get cached and very messy if not released
        _world->getPairCache()->setOverlapFilterCallback(NULL);
        _dispatcher->_poverlapfilt = NULL;
        return bCollision;
    }

    virtual void SetTolerance(dReal tolerance) { RAVELOG_WARNA("not implemented\n"); }

private:
    boost::shared_ptr<BulletSpace> bulletspace;
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
