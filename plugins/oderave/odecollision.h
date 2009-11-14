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
#ifndef RAVE_COLLISION_ODE
#define RAVE_COLLISION_ODE

#include "odespace.h"

class ODECollisionChecker : public OpenRAVE::CollisionCheckerBase
{
    struct COLLISIONCALLBACK
    {
    COLLISIONCALLBACK(boost::shared_ptr<ODECollisionChecker> pchecker, CollisionReportPtr report, KinBodyConstPtr pbody, KinBody::LinkConstPtr plink) : _pchecker(pchecker), _report(report), _pbody(pbody), _plink(plink), _bCollision(false), _bOneCollision(false), fraymaxdist(0)
        {
            if( !!_report )
                _report->Reset(pchecker->GetCollisionOptions());
        }
        boost::shared_ptr<ODECollisionChecker> _pchecker;
        CollisionReportPtr _report;
        KinBodyConstPtr _pbody;
        KinBody::LinkConstPtr _plink;
        bool _bCollision;
        bool _bOneCollision;
        OpenRAVE::dReal fraymaxdist;
    };

    inline boost::shared_ptr<ODECollisionChecker> shared_checker() { return boost::static_pointer_cast<ODECollisionChecker>(shared_from_this()); }
    inline boost::shared_ptr<ODECollisionChecker const> shared_checker_const() const { return boost::static_pointer_cast<ODECollisionChecker const>(shared_from_this()); }

 public:
 ODECollisionChecker(EnvironmentBasePtr penv) : OpenRAVE::CollisionCheckerBase(penv), odespace(new ODESpace(penv,GetCollisionInfo,false)) {
        _options = 0;
        geomray = NULL;
    }
    ~ODECollisionChecker() {
        if( geomray != NULL )
            dGeomDestroy(geomray);
    }

    virtual void SetTolerance(OpenRAVE::dReal tolerance) {}
    
    virtual bool InitEnvironment()
    {
        if( !odespace->InitEnvironment() )
            return false;

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies)
            InitKinBody(*itbody);

        geomray = dCreateRay(0, 1000.0f); // 1000m (is this used?)
        //dGeomRaySetParams(geomray,0,0);
        return true;
    }

    virtual void DestroyEnvironment()
    {
        if( geomray != NULL ) {
            dGeomDestroy(geomray); geomray = NULL;
        }
    
        // go through all the KinBodies and destory their collision pointers
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies)
            SetCollisionData(*itbody, boost::shared_ptr<void>());
        odespace->DestroyEnvironment();
    }

    virtual bool InitKinBody(KinBodyPtr pbody)
    {
        boost::shared_ptr<void> pinfo = odespace->InitKinBody(pbody);
        SetCollisionData(pbody, pinfo);
        return !!pinfo;
    }

    virtual bool SetCollisionOptions(int collisionoptions)
    {
        _options = collisionoptions;
        if( _options & OpenRAVE::CO_Distance ) {
            //RAVELOG("ode checker doesn't support CO_Distance\n");
            return false;
        }
    
        return true;
    }

    virtual int GetCollisionOptions() const { return _options; }
    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput)
    {
        return false;
    }

    virtual bool Enable(KinBodyConstPtr pbody, bool bEnable)
    {
        return odespace->Enable(pbody,bEnable);
    }

    virtual bool EnableLink(KinBody::LinkConstPtr plink, bool bEnable)
    {
        return odespace->EnableLink(plink,bEnable);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,pbody,KinBody::LinkConstPtr());
        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled() ) {
            RAVELOG_WARNA("body %s not valid\n", pbody->GetName().c_str());
            return false;
        }

        odespace->Synchronize();
        dSpaceCollide(odespace->GetSpace(), &cb, KinBodyCollisionCallback);

        if( cb._bCollision )
            return true;

        // check attached objects
        std::vector<KinBodyPtr> vattached;
        pbody->GetAttached(vattached);
        FOREACHC(itbodies, vattached) {
            cb._pbody = *itbodies;
            dSpaceCollide(odespace->GetSpace(), &cb, KinBodyCollisionCallback);
            if( cb._bCollision )
                return true;
        }

        return false;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,KinBodyPtr(),KinBody::LinkConstPtr());

        if( pbody1->GetLinks().size() == 0 || !pbody1->IsEnabled()  ) {
            RAVELOG_WARNA("body1 %s not valid\n", pbody1->GetName().c_str());
            return false;
        }
        if( pbody2->GetLinks().size() == 0 || !pbody2->IsEnabled()  ) {
            RAVELOG_WARNA("body2 %s not valid\n", pbody2->GetName().c_str());
            return false;
        }

        if( pbody1->IsAttached(pbody2) || pbody2->IsAttached(pbody1) )
            return false;

        odespace->Synchronize();
        dSpaceCollide2((dGeomID)odespace->GetBodySpace(pbody1),(dGeomID)odespace->GetBodySpace(pbody2),&cb,KinBodyKinBodyCollisionCallback);

        return cb._bCollision;
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,KinBodyPtr(),plink);
        if( !plink->IsEnabled() ) {
            RAVELOG_WARNA("calling collision on disabled link %s\n", plink->GetName().c_str());
            return false;
        }

        odespace->Synchronize();
        dSpaceCollide(odespace->GetSpace(), &cb, LinkCollisionCallback);
        return cb._bCollision;
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report)
    {
        if( !!report )
            report->Reset(_options);

        if( !plink1->IsEnabled() ) {
            RAVELOG_WARNA("calling collision on disabled link1 %S\n", plink1->GetName().c_str());
            return false;
        }
        if( !plink2->IsEnabled() ) {
            RAVELOG_WARNA("calling collision on disabled link2 %s\n", plink2->GetName().c_str());
            return false;
        }

        odespace->Synchronize();

        dContact contact[16];
        dGeomID geom1 = odespace->GetLinkGeom(plink1);
        while(geom1 != NULL) {
            assert(dGeomIsEnabled(geom1));

            dGeomID geom2 = odespace->GetLinkGeom(plink2);

            while(geom2 != NULL) {

                assert(dGeomIsEnabled(geom2));

                int N = dCollide (geom1, geom2,16,&contact[0].geom,sizeof(dContact));
                if (N) {

                    if( N > 0 && !!report ) {
                        report->numCols = N;
                        report->plink1 = plink1;
                        report->plink2 = plink2;

                        if( report->options & OpenRAVE::CO_Contacts ) {
                            report->contacts.reserve(N);
                            dGeomID checkgeom1 = dGeomGetClass(geom1) == dGeomTransformClass ? dGeomTransformGetGeom(geom1) : geom1;
                            for(int i = 0; i < N; ++i) {
                                //assert(contact[i].geom.depth >= 0);
                                Vector vnorm(contact[i].geom.normal);
                                if( checkgeom1 != contact[i].geom.g1 )
                                    vnorm = -vnorm;
                                assert( checkgeom1 == contact[i].geom.g1 || checkgeom1 == contact[i].geom.g2 );
                                report->contacts.push_back(COLLISIONREPORT::CONTACT(contact[i].geom.pos, vnorm, contact[i].geom.depth));
                            }
                        }
                    }

                    return true;
                }

                geom2 = dGeomGetBodyNext(geom2);
            }

            geom1 = dGeomGetBodyNext(geom1);
        }

        return false;
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
            RAVELOG_WARNA("body %s not valid\n", pbody->GetName().c_str());
            return false;
        }

        if( !plink->IsEnabled() ) {
            RAVELOG_WARNA("calling collision on disabled link %s\n", plink->GetName().c_str());
            return false;
        }

        odespace->Synchronize();
        FOREACHC(itlink, pbody->GetLinks()) {
            if( CheckCollision(plink, KinBody::LinkConstPtr(*itlink), report) )
                return true;
        }
        return false;

        // doesn't work
        //    dSpaceCollide2((dGeomID)pbody->GetSpace(), plink1->GetGeom(), plink1, LinkCollisionCallback);
    }
    
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        if( vlinkexcluded.size() == 0 && vbodyexcluded.size() == 0 )
            return CheckCollision(plink,report);

        RAVELOG_ERRORA("This type of collision checking is not yet implemented in the ODE collision checker.\n");
        BOOST_ASSERT(0);
        odespace->Synchronize();
        return false;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        if( vbodyexcluded.size() == 0 && vlinkexcluded.size() == 0 )
            return CheckCollision(pbody, report);

        COLLISIONCALLBACK cb(shared_checker(),report,KinBodyPtr(),KinBody::LinkConstPtr());

        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
            RAVELOG_WARNA("body %s not valid\n", pbody->GetName().c_str());
            return false;
        }

        BOOST_ASSERT( vlinkexcluded.size() == 0 );
        odespace->Synchronize();

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            if( *itbody == pbody || find(vbodyexcluded.begin(),vbodyexcluded.end(),*itbody) != vbodyexcluded.end() )
                continue;

            if( pbody->IsAttached(*itbody) || (*itbody)->IsAttached(pbody) )
                continue;

            dSpaceCollide2((dGeomID)odespace->GetBodySpace(pbody),(dGeomID)odespace->GetBodySpace(*itbody),&cb,KinBodyKinBodyCollisionCallback);

            if( cb._bCollision )
                break;
        }

        if( cb._bCollision )
            return true;

        // check all attached bodies
        std::vector<KinBodyPtr> vattached;
        pbody->GetAttached(vattached);
        FOREACHC(itattached, vattached) {
            KinBodyPtr pattbody = *itattached;
            if( pattbody->GetLinks().size() == 0 || !pattbody->IsEnabled()  ) {
                RAVELOG_WARNA("body %s not valid\n", pattbody->GetName().c_str());
                return false;
            }
        
            FOREACHC(itbody, vbodies) {
                if( *itbody == pattbody || find(vbodyexcluded.begin(),vbodyexcluded.end(),*itbody) != vbodyexcluded.end() )
                    continue;

                if( pattbody->IsAttached(*itbody) || (*itbody)->IsAttached(pattbody) )
                    continue;

                dSpaceCollide2((dGeomID)odespace->GetBodySpace(pattbody),(dGeomID)odespace->GetBodySpace(*itbody),&cb,KinBodyKinBodyCollisionCallback);

                if( cb._bCollision )
                    break;
            }
        }

        return cb._bCollision;
    }
    
    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        if( !!report )
            report->Reset(_options);
    
        if( !plink->IsEnabled() ) {
            RAVELOG_WARNA("calling collision on disabled link %s\n", plink->GetName().c_str());
            return false;
        }

        odespace->Synchronize();
        OpenRAVE::dReal fmaxdist = OpenRAVE::RaveSqrt(ray.dir.lengthsqr3());
        if( fabsf(fmaxdist-1) < 1e-4 )
            RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
        
        Vector vnormdir = ray.dir*(1/fmaxdist);
        dGeomRaySet(geomray, ray.pos.x, ray.pos.y, ray.pos.z, vnormdir.x, vnormdir.y, vnormdir.z);
        dGeomRaySetClosestHit(geomray, !(_options&OpenRAVE::CO_RayAnyHit)); // only care about the closest points

        bool bCollision = false;
        dContact contact[16];
        dGeomID geom1 = odespace->GetLinkGeom(plink);
        while(geom1 != NULL) {
            assert(dGeomIsEnabled(geom1));

            int N = dCollide (geom1, geomray,16,&contact[0].geom,sizeof(dContact));
            if (N > 0) {
                int index = 0;
                for(;index < N; ++index) {
                    if( contact[index].geom.depth <= fmaxdist )
                        break;
                }

                if( index >= N ) {
                    geom1 = dGeomGetBodyNext(geom1);
                    continue;
                }

                if( !!report ) {

                    if( report->numCols ) {
                        // collided already, see if this point is closer
                        if( report->minDistance < contact[index].geom.depth ) {
                            geom1 = dGeomGetBodyNext(geom1);
                            continue;
                        }
                    }

                    report->numCols = 1;
                    report->minDistance = contact[index].geom.depth;
                    report->plink1 = plink;

                    // always return contacts since it isn't that much computation (openravepy expects this!)
                    //if( report->options & OpenRAVE::CO_Contacts) {
                    Vector vnorm(contact[index].geom.normal);
                    if( contact[index].geom.g1 != geomray )
                        vnorm = -vnorm;
                    if( report->contacts.size() == 0 ) {
                        report->contacts.push_back(COLLISIONREPORT::CONTACT(contact[index].geom.pos, vnorm, contact[index].geom.depth));
                    }
                    else
                        report->contacts.front() = COLLISIONREPORT::CONTACT(contact[index].geom.pos, vnorm, contact[index].geom.depth);
                    //}

                    if( report->options&OpenRAVE::CO_RayAnyHit ) {
                        bCollision = true;
                        break;
                    }
                }
                else {
                    bCollision = true;
                    break;
                }
            }

            geom1 = dGeomGetBodyNext(geom1);
        }

        return bCollision;
    }

    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,KinBodyPtr(),KinBody::LinkConstPtr());

        if( pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
            RAVELOG_WARNA("body %s not valid\n", pbody->GetName().c_str());
            return false;
        }

        odespace->Synchronize();
        cb.fraymaxdist = OpenRAVE::RaveSqrt(ray.dir.lengthsqr3());
        if( fabsf(cb.fraymaxdist-1) < 1e-4 )
            RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
        Vector vnormdir = ray.dir*(1/cb.fraymaxdist);
        dGeomRaySet(geomray, ray.pos.x, ray.pos.y, ray.pos.z, vnormdir.x, vnormdir.y, vnormdir.z);
        dGeomRaySetClosestHit(geomray, !(_options&OpenRAVE::CO_RayAnyHit)); // only care about the closest points

        //dSpaceAdd(pbody->GetSpace(), geomray);
        dSpaceCollide2((dGeomID)odespace->GetBodySpace(pbody), geomray, &cb, RayCollisionCallback);
        //dSpaceRemove(pbody->GetSpace(), geomray);

        return cb._bOneCollision;
    }

    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,KinBodyPtr(),KinBody::LinkConstPtr());

        cb.fraymaxdist = OpenRAVE::RaveSqrt(ray.dir.lengthsqr3());
        Vector vnormdir = ray.dir*(1/cb.fraymaxdist);
        if( fabsf(cb.fraymaxdist-1) < 1e-4 )
            RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
        dGeomRaySet(geomray, ray.pos.x, ray.pos.y, ray.pos.z, vnormdir.x, vnormdir.y, vnormdir.z);

        dGeomRaySetClosestHit(geomray, !(_options&OpenRAVE::CO_RayAnyHit)); // only care about the closest points
        dGeomRaySetParams(geomray,0,0);

        odespace->Synchronize();

        //dSpaceAdd(odespace->GetSpace(), geomray);
        dSpaceCollide2((dGeomID)odespace->GetSpace(), geomray, &cb, RayCollisionCallback);
        //dSpaceRemove(odespace->GetSpace(), geomray);

        return cb._bOneCollision;
    }

    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if( pbody->GetLinks().size() <= 1 )
            return false;

        // check collision, ignore adjacent bodies
        FOREACHC(itset, pbody->GetNonAdjacentLinks()) {
            assert( (*itset&0xffff) < (int)pbody->GetLinks().size() && (*itset>>16) < (int)pbody->GetLinks().size() );
            if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(pbody->GetLinks()[*itset&0xffff]), KinBody::LinkConstPtr(pbody->GetLinks()[*itset>>16]), report) ) {
                RAVELOG_VERBOSEA(str(boost::format("selfcol %s, Links %s %s are colliding\n")%pbody->GetName()%pbody->GetLinks()[*itset&0xffff]->GetName()%pbody->GetLinks()[*itset>>16]->GetName()));
                return true;
            }
        }

        return false;
    }

 private:
    static boost::shared_ptr<void> GetCollisionInfo(KinBodyConstPtr pbody) { return pbody->GetCollisionData(); }

    static void KinBodyCollisionCallback (void *data, dGeomID o1, dGeomID o2)
    {
        COLLISIONCALLBACK* pcb = (COLLISIONCALLBACK*)data;
        pcb->_pchecker->_KinBodyCollisionCallback(o1,o2,pcb);
    }


    void _KinBodyCollisionCallback (dGeomID o1, dGeomID o2, COLLISIONCALLBACK* pcb)
    {
        if( pcb->_bCollision )
            return; // don't test anymore

        KinBodyPtr* o1data = (KinBodyPtr*)dGeomGetData(o1);
        KinBodyPtr* o2data = (KinBodyPtr*)dGeomGetData(o2);

        // ASSUMPTION: every space is attached to a KinBody!
        if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) )
            return;
    
        // only recurse two spaces if one of them is _pbody
        if( dGeomIsSpace(o1) && dGeomIsSpace(o2) && !(o1data != NULL && *o1data == pcb->_pbody) && !(o2data != NULL && *o2data == pcb->_pbody) )
            return;

        // if certain
        if( dGeomIsSpace(o1) ) {
            BOOST_ASSERT(!!o1data);
            if( pcb->_pbody->IsAttached(*o1data) )
            return;
        }
        if( dGeomIsSpace(o2) ) {
            BOOST_ASSERT(!!o2data);
            if( pcb->_pbody->IsAttached(*o2data) )
                return;
        }

        if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {    
            dSpaceCollide2(o1,o2,pcb,KinBodyCollisionCallback);
            return;
        }

        // ignore static, static collisions
        dBodyID b1,b2;
        b1 = dGeomGetBody(o1);
        b2 = dGeomGetBody(o2);

        KinBody::LinkPtr pkb1,pkb2;
        if(!!b1 && dBodyGetData(b1))
            pkb1 = *(KinBody::LinkPtr*)dBodyGetData(b1);
        if(!!b2 && dBodyGetData(b1))
            pkb2 = *(KinBody::LinkPtr*)dBodyGetData(b2);

        if( !!pkb1 && !pkb1->IsEnabled() )
            return;
        if( !!pkb2 && !pkb2->IsEnabled() )
            return;

        dContact contact[16];
        int N = dCollide (o1,o2,16,&contact[0].geom,sizeof(dContact));
        if ( N ) {

            if( N > 0 && !!pcb->_report ) {
                pcb->_report->numCols = N;
                pcb->_report->minDistance = contact[0].geom.depth;
                pcb->_report->plink1 = pkb1;
                pcb->_report->plink2 = pkb2;

                if( pcb->_report->options & OpenRAVE::CO_Contacts ) {
                    pcb->_report->contacts.reserve(N);
                    dGeomID checkgeom1 = dGeomGetClass(o1) == dGeomTransformClass ? dGeomTransformGetGeom(o1) : o1;
                    for(int i = 0; i < N; ++i)
                        pcb->_report->contacts.push_back(COLLISIONREPORT::CONTACT(contact[i].geom.pos, checkgeom1 != contact[i].geom.g1 ? -Vector(contact[i].geom.normal) : Vector(contact[i].geom.normal), contact[i].geom.depth));
                }
            }

            pcb->_bCollision = true;
        }
    }

    static void KinBodyKinBodyCollisionCallback (void *data, dGeomID o1, dGeomID o2)
    {
        COLLISIONCALLBACK* pcb = (COLLISIONCALLBACK*)data;
        pcb->_pchecker->_KinBodyKinBodyCollisionCallback(o1,o2,pcb);
    }

    void _KinBodyKinBodyCollisionCallback (dGeomID o1, dGeomID o2, COLLISIONCALLBACK* pcb)
    {
        if( pcb->_bCollision )
            return; // don't test anymore

        if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) )
            return;

        if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {    
            dSpaceCollide2(o1,o2,pcb,KinBodyKinBodyCollisionCallback);
            return;
        }

        // ignore static, static collisions
        dBodyID b1,b2;
        b1 = dGeomGetBody(o1);
        b2 = dGeomGetBody(o2);

        KinBody::LinkPtr pkb1,pkb2;
        if(!!b1 && dBodyGetData(b1))
            pkb1 = *(KinBody::LinkPtr*)dBodyGetData(b1);
        if(!!b2 && dBodyGetData(b1))
            pkb2 = *(KinBody::LinkPtr*)dBodyGetData(b2);

        if( !!pkb1 && !pkb1->IsEnabled() )
            return;
        if( !!pkb2 && !pkb2->IsEnabled() )
            return;

        // only care if one of the bodies is the link
        dContact contact[16];
        int N = dCollide (o1,o2,16,&contact[0].geom,sizeof(dContact));
        if ( N ) {
            if( N > 0 && !!pcb->_report ) {
                pcb->_report->numCols = N;
                pcb->_report->plink1 = pkb1;
                pcb->_report->plink2 = pkb2;

                if( pcb->_report->options & OpenRAVE::CO_Contacts ) {
                    pcb->_report->contacts.reserve(N);
                    dGeomID checkgeom1 = dGeomGetClass(o1) == dGeomTransformClass ? dGeomTransformGetGeom(o1) : o1;
                    for(int i = 0; i < N; ++i)
                        pcb->_report->contacts.push_back(COLLISIONREPORT::CONTACT(contact[i].geom.pos, checkgeom1 != contact[i].geom.g1 ? -Vector(contact[i].geom.normal) : Vector(contact[i].geom.normal), contact[i].geom.depth));
                }
            }

            pcb->_bCollision = true;
        }
    }

    static void LinkCollisionCallback (void *data, dGeomID o1, dGeomID o2)
    {
        COLLISIONCALLBACK* pcb = (COLLISIONCALLBACK*)data;
        pcb->_pchecker->_LinkCollisionCallback(o1,o2,pcb);
    }

    void _LinkCollisionCallback (dGeomID o1, dGeomID o2, COLLISIONCALLBACK* pcb)
    {
        if( pcb->_bCollision )
            return; // don't test anymore

        KinBodyPtr pbody = pcb->_plink->GetParent();
    
        KinBodyPtr* o1data = (KinBodyPtr*)dGeomGetData(o1);
        KinBodyPtr* o2data = (KinBodyPtr*)dGeomGetData(o2);

        // ASSUMPTION: every space is attached to a KinBody!
        if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) )
            return;

        // only recurse two spaces if one of them is pbody
        if( dGeomIsSpace(o1) && dGeomIsSpace(o2) && !(o1data != NULL && *o1data == pbody) && !(o2data != NULL && *o2data == pbody) )
            return;

        if( dGeomIsSpace(o1) ) {
            BOOST_ASSERT(!!o1data);
            if( pbody->IsAttached(*o1data) )
            return;
        }
        if( dGeomIsSpace(o2) ) {
            BOOST_ASSERT(!!o2data);
            if( pbody->IsAttached(*o2data) )
                return;
        }

        if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {    
            dSpaceCollide2(o1,o2,pcb,LinkCollisionCallback);
            return;
        }

        // ignore static, static collisions
        dBodyID b1,b2;
        b1 = dGeomGetBody(o1);
        b2 = dGeomGetBody(o2);

        KinBody::LinkPtr pkb1,pkb2;
        if(!!b1 && dBodyGetData(b1))
            pkb1 = *(KinBody::LinkPtr*)dBodyGetData(b1);
        if(!!b2 && dBodyGetData(b1))
            pkb2 = *(KinBody::LinkPtr*)dBodyGetData(b2);

        if( !!pkb1 && !pkb1->IsEnabled() )
            return;
        if( !!pkb2 && !pkb2->IsEnabled() )
            return;

        // only care if one of the bodies is the link
        if( pkb1 == pcb->_plink || pkb2 == pcb->_plink ) {

            dContact contact[16];
            int N = dCollide (o1,o2,16,&contact[0].geom,sizeof(dContact));
            if (N) {

                if( N > 0 && !!pcb->_report ) {
                    pcb->_report->numCols = N;
                    pcb->_report->plink1 = pcb->_plink;
                    pcb->_report->plink2 = pkb1 != pcb->_plink ? pkb1 : pkb2;
                    dGeomID checkgeom1 = pkb1 == pcb->_plink ? o1 : o2;
                    checkgeom1 = dGeomGetClass(checkgeom1) == dGeomTransformClass ? dGeomTransformGetGeom(checkgeom1) : checkgeom1;

                    if( pcb->_report->options & OpenRAVE::CO_Contacts ) {
                        pcb->_report->contacts.reserve(N);
                        for(int i = 0; i < N; ++i) {
                            assert( checkgeom1 == contact[i].geom.g1 || checkgeom1 == contact[i].geom.g2 );
                            Vector vnorm(contact[i].geom.normal);
                            if( checkgeom1 != contact[i].geom.g1 )
                                vnorm = -vnorm;
                            pcb->_report->contacts.push_back(COLLISIONREPORT::CONTACT(contact[i].geom.pos,vnorm,contact[i].geom.depth));
                        }
                    }
                }

                pcb->_bCollision = true;
            }
        }
    }

    static void RayCollisionCallback (void *data, dGeomID o1, dGeomID o2)
    {
        COLLISIONCALLBACK* pcb = (COLLISIONCALLBACK*)data;
        pcb->_pchecker->_RayCollisionCallback(o1,o2,pcb);
    }

    void _RayCollisionCallback (dGeomID o1, dGeomID o2, COLLISIONCALLBACK* pcb)
    {
        if( pcb->_bCollision )
            return; // don't test anymore

        if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) )
            return;

        if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {    
            dSpaceCollide2(o1,o2,pcb,RayCollisionCallback);
            return;
        }

        // ignore if a ray isn't included
        if( dGeomGetClass(o1) != dRayClass && dGeomGetClass(o2) != dRayClass )
            return;

        dContact contact[2];
        int N = dCollide (o1,o2,2,&contact[0].geom,sizeof(dContact));
        if (N > 0) {

            int index = 0;
            for(;index < N; ++index) {
                if( contact[index].geom.depth <= pcb->fraymaxdist )
                    break;
            }

            if( index >= N )
                return;

            dGeomID geomray = o2;
            dBodyID b = dGeomGetBody(o1);
            if( b == NULL ) {
                assert( dGeomGetClass(o1) == dRayClass );
                geomray = o1;
                b = dGeomGetBody(o2);
            }
            BOOST_ASSERT( b != NULL );
            
            if( !!pcb->_report ) {
                if( pcb->_bOneCollision ) {
                    // collided already, see if this point is closer
                    if( pcb->_report->minDistance < contact[index].geom.depth )
                        return;
                }

                pcb->_report->numCols = 1;
                pcb->_report->minDistance = contact[index].geom.depth;
                if( dBodyGetData(b) )
                    pcb->_report->plink1 = *(KinBody::LinkPtr*)dBodyGetData(b);

                // always return contacts since it isn't that much computation (openravepy expects this!)
                //if( pcb->_report->options & OpenRAVE::CO_Contacts) {
                Vector vnorm(contact[index].geom.normal);
                if( contact[index].geom.g1 != geomray )
                    vnorm = -vnorm;
                if( pcb->_report->contacts.size() == 0 ) {
                    pcb->_report->contacts.push_back(COLLISIONREPORT::CONTACT(contact[index].geom.pos, vnorm, contact[index].geom.depth));
                }
                else
                    pcb->_report->contacts.front() = COLLISIONREPORT::CONTACT(contact[index].geom.pos, vnorm, contact[index].geom.depth);
                //}

                if( _options&OpenRAVE::CO_RayAnyHit )
                    pcb->_bCollision = true;
            }
            else {
                if( _options&OpenRAVE::CO_RayAnyHit )
                    pcb->_bCollision = true;
            }

            pcb->_bOneCollision = true;
        }
    }

    int _options;
    dGeomID geomray; // used for all ray tests
    boost::shared_ptr<ODESpace> odespace;
};

#endif
