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

#include "plugindefs.h"

#include "odecollision.h"

ODECollisionChecker::ODECollisionChecker(OpenRAVE::EnvironmentBase* penv) : OpenRAVE::CollisionCheckerBase(penv), odespace(penv, GetCollisionInfo, false)
{
    _options = 0;
    geomray = NULL;
    _bCollision = false;
    _pCurrentReport = NULL;
    _pbody = NULL;
}

ODECollisionChecker::~ODECollisionChecker()
{
    if( geomray != NULL )
        dGeomDestroy(geomray);
}

bool ODECollisionChecker::InitEnvironment()
{
    if( !odespace.InitEnvironment() )
        return false;
    FOREACHC(itbody, GetEnv()->GetBodies())
        InitKinBody(*itbody);

    geomray = dCreateRay(0, 1000.0f); // 1000m (is this used?)
    //dGeomRaySetParams(geomray,0,0);

    return true;
}

void ODECollisionChecker::DestroyEnvironment()
{
    if( geomray != NULL ) {
        dGeomDestroy(geomray); geomray = NULL;
    }
    
    // go through all the KinBodies and destory their collision pointers
    FOREACHC(itbody, GetEnv()->GetBodies()) {
        if( !odespace.DestroyKinBody(*itbody) )
            RAVELOG(L"ode collision checker failed to destroy body\n");
        SetCollisionData(*itbody, NULL);
    }
    odespace.DestroyEnvironment();
}

bool ODECollisionChecker::InitKinBody(KinBody* pbody)
{
    void* pinfo = odespace.InitKinBody(pbody);
    SetCollisionData(pbody, pinfo);
    return pinfo != NULL;
}

bool ODECollisionChecker::DestroyKinBody(KinBody* pbody)
{
    bool bSuccess = odespace.DestroyKinBody(pbody);
    SetCollisionData(pbody, NULL);
    return bSuccess;
}

bool ODECollisionChecker::SetCollisionOptions(int options)
{
    _options = options;
    if( options & OpenRAVE::CO_Distance ) {
        //RAVELOG(L"ode checker doesn't support CO_Distance\n");
        return false;
    }
    
    return true;
}

int ODECollisionChecker::GetCollisionOptions() const
{
    return _options;
}

bool ODECollisionChecker::SetCollisionOptions(std::ostream& sout, std::istream& sinput)
{
    return false;
}

bool ODECollisionChecker::Enable(const KinBody* pbody, bool bEnable)
{
    return odespace.Enable(pbody,bEnable);
}

bool ODECollisionChecker::EnableLink(const KinBody::Link* plink, bool bEnable)
{
    return odespace.EnableLink(plink,bEnable);
}

// only care about collision with kinbody
void ODECollisionChecker::KinBodyCollisionCallback (void *data, dGeomID o1, dGeomID o2)
{
    ((ODECollisionChecker*)data)->_KinBodyCollisionCallback(o1,o2);
}

void ODECollisionChecker::_KinBodyCollisionCallback (dGeomID o1, dGeomID o2)
{
    if( _bCollision )
        return; // don't test anymore

    void* o1data = dGeomGetData(o1);
    void* o2data = dGeomGetData(o2);

    // ASSUMPTION: every space is attached to a KinBody!
    if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) )
        return;
    
    // only recurse two spaces if one of them is _pbody
    if( dGeomIsSpace(o1) && dGeomIsSpace(o2) && o1data != _pbody && o2data != _pbody )
        return;

    // if certain
    if( dGeomIsSpace(o1) && _pbody->IsAttached((KinBody*)o1data) )
        return;
    if( dGeomIsSpace(o2) && _pbody->IsAttached((KinBody*)o2data) )
        return;

    if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {    
        dSpaceCollide2(o1,o2,this,KinBodyCollisionCallback);
        return;
    }

    // ignore static, static collisions
    dBodyID b1,b2;
    b1 = dGeomGetBody(o1);
    b2 = dGeomGetBody(o2);

    KinBody::Link* pkb1 = b1 != NULL ? (KinBody::Link*)dBodyGetData(b1) : NULL;
    KinBody::Link* pkb2 = b2 != NULL ? (KinBody::Link*)dBodyGetData(b2) : NULL;

    if( pkb1 != NULL && !pkb1->IsEnabled() )
        return;
    if( pkb2 != NULL && !pkb2->IsEnabled() )
        return;

    dContact contact[16];
    int N = dCollide (o1,o2,16,&contact[0].geom,sizeof(dContact));
    if ( N ) {

        if( N > 0 && _pCurrentReport != NULL ) {
            _pCurrentReport->numCols = N;
            _pCurrentReport->minDistance = contact[0].geom.depth;
            _pCurrentReport->plink1 = b1 != NULL ? (KinBody::Link*)dBodyGetData(b1) : NULL;
            _pCurrentReport->plink2 = b2 != NULL ? (KinBody::Link*)dBodyGetData(b2) : NULL;

            if( _pCurrentReport->options & OpenRAVE::CO_Contacts ) {
                _pCurrentReport->contacts.reserve(N);
                dGeomID checkgeom1 = dGeomGetClass(o1) == dGeomTransformClass ? dGeomTransformGetGeom(o1) : o1;
                for(int i = 0; i < N; ++i)
                    _pCurrentReport->contacts.push_back(COLLISIONREPORT::CONTACT(contact[i].geom.pos, checkgeom1 != contact[i].geom.g1 ? -Vector(contact[i].geom.normal) : Vector(contact[i].geom.normal), contact[i].geom.depth));
            }
        }

        _bCollision = true;
    }
}

bool ODECollisionChecker::CheckCollision(const KinBody* pbody, COLLISIONREPORT* pReport)
{
    _bCollision = false;
    _pCurrentReport = pReport;
    if( pReport != NULL ) pReport->Reset(_options);

    if( pbody == NULL || pbody->GetLinks().size() == 0 || !pbody->IsEnabled() ) {
        RAVELOG(L"body %S not valid\n", pbody != NULL ? pbody->GetName() : L"()");
        return false;
    }

    _pbody = pbody;
    odespace.Synchronize();
    dSpaceCollide(odespace.GetSpace(), this, KinBodyCollisionCallback);

    if( _bCollision )
        return true;

    // check attached objects
    FOREACHC(itbodies, pbody->GetAttached()) {
        _pbody = *itbodies;
        dSpaceCollide(odespace.GetSpace(), this, KinBodyCollisionCallback);
        if( _bCollision )
            return true;
    }

    return _bCollision;
}

// only care about collision with kinbody
void ODECollisionChecker::KinBodyKinBodyCollisionCallback (void *data, dGeomID o1, dGeomID o2)
{
    ((ODECollisionChecker*)data)->_KinBodyKinBodyCollisionCallback(o1,o2);
}

void ODECollisionChecker::_KinBodyKinBodyCollisionCallback (dGeomID o1, dGeomID o2)
{
    if( _bCollision )
        return; // don't test anymore

    if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) )
        return;

    if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {    
        dSpaceCollide2(o1,o2,this,KinBodyKinBodyCollisionCallback );
        return;
    }

    // ignore static, static collisions
    dBodyID b1,b2;
    b1 = dGeomGetBody(o1);
    b2 = dGeomGetBody(o2);

    KinBody::Link* pkb1 = (KinBody::Link*)dBodyGetData(b1);
    KinBody::Link* pkb2 = (KinBody::Link*)dBodyGetData(b2);
    if( pkb1 != NULL && !pkb1->IsEnabled() )
        return;
    if( pkb2 != NULL && !pkb2->IsEnabled() )
        return;

    // only care if one of the bodies is the link
    dContact contact[16];
    int N = dCollide (o1,o2,16,&contact[0].geom,sizeof(dContact));
    if ( N ) {

        if( N > 0 && _pCurrentReport != NULL ) {
            _pCurrentReport->numCols = N;
            _pCurrentReport->plink1 = pkb1;
            _pCurrentReport->plink2 = pkb2;

            if( _pCurrentReport->options & OpenRAVE::CO_Contacts ) {
                _pCurrentReport->contacts.reserve(N);
                dGeomID checkgeom1 = dGeomGetClass(o1) == dGeomTransformClass ? dGeomTransformGetGeom(o1) : o1;
                for(int i = 0; i < N; ++i)
                    _pCurrentReport->contacts.push_back(COLLISIONREPORT::CONTACT(contact[i].geom.pos, checkgeom1 != contact[i].geom.g1 ? -Vector(contact[i].geom.normal) : Vector(contact[i].geom.normal), contact[i].geom.depth));
            }
        }

        _bCollision = true;
    }
}

bool ODECollisionChecker::CheckCollision(const KinBody* pbody1, const KinBody* pbody2, COLLISIONREPORT* pReport)
{
    if( pbody1 == NULL || pbody1->GetLinks().size() == 0 || !pbody1->IsEnabled()  ) {
        RAVELOG(L"body1 %S not valid\n", pbody1 != NULL ? pbody1->GetName() : L"()");
        return false;
    }
    if( pbody2 == NULL || pbody2->GetLinks().size() == 0 || !pbody2->IsEnabled()  ) {
        RAVELOG(L"body2 %S not valid\n", pbody2 != NULL ? pbody2->GetName() : L"()");
        return false;
    }

    if( pbody1->IsAttached(pbody2) || pbody2->IsAttached(pbody1) )
        return false;

    _bCollision = false;
    _pCurrentReport = pReport;
    if( pReport != NULL ) pReport->Reset(_options);

    odespace.Synchronize();
    dSpaceCollide2((dGeomID)odespace.GetBodySpace(pbody1),(dGeomID)odespace.GetBodySpace(pbody2),this,KinBodyKinBodyCollisionCallback);

    return _bCollision;
}

void ODECollisionChecker::LinkCollisionCallback (void *data, dGeomID o1, dGeomID o2)
{
    ((ODECollisionChecker*)data)->_LinkCollisionCallback(o1,o2);
}

void ODECollisionChecker::_LinkCollisionCallback (dGeomID o1, dGeomID o2)
{
    if( _bCollision )
        return; // don't test anymore

    const KinBody::Link* plink = _plink;
    KinBody* pbody = plink->GetParent();
    
    void* o1data = dGeomGetData(o1);
    void* o2data = dGeomGetData(o2);

    // ASSUMPTION: every space is attached to a KinBody!
    if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) )
        return;

    // only recurse two spaces if one of them is pbody
    if( dGeomIsSpace(o1) && dGeomIsSpace(o2) && o1data != pbody && o2data != pbody )
        return;

    // if certain
    if( dGeomIsSpace(o1) && pbody->IsAttached((KinBody*)o1data) )
        return;
    if( dGeomIsSpace(o2) && pbody->IsAttached((KinBody*)o2data) )
        return;

    if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {    
        dSpaceCollide2(o1,o2,this,LinkCollisionCallback);
        return;
    }

    // ignore static, static collisions
    dBodyID b1,b2;
    b1 = dGeomGetBody(o1);
    b2 = dGeomGetBody(o2);

    KinBody::Link* pkb1 = (KinBody::Link*)dBodyGetData(b1);
    KinBody::Link* pkb2 = (KinBody::Link*)dBodyGetData(b2);
    if( pkb1 != NULL && !pkb1->IsEnabled() )
        return;
    if( pkb2 != NULL && !pkb2->IsEnabled() )
        return;

    // only care if one of the bodies is the link
    if( pkb1 == plink || pkb2 == plink ) {

        dContact contact[16];
        int N = dCollide (o1,o2,16,&contact[0].geom,sizeof(dContact));
        if (N) {

            if( N > 0 && _pCurrentReport != NULL ) {
                _pCurrentReport->numCols = N;
                _pCurrentReport->plink1 = (KinBody::Link*)plink;
                _pCurrentReport->plink2 = pkb1 != plink ? pkb1 : pkb2;
                dGeomID checkgeom1 = pkb1 == plink ? o1 : o2;
                checkgeom1 = dGeomGetClass(checkgeom1) == dGeomTransformClass ? dGeomTransformGetGeom(checkgeom1) : checkgeom1;

                if( _pCurrentReport->options & OpenRAVE::CO_Contacts ) {
                    _pCurrentReport->contacts.reserve(N);
                    for(int i = 0; i < N; ++i) {
                        assert( checkgeom1 == contact[i].geom.g1 || checkgeom1 == contact[i].geom.g2 );
                        Vector vnorm(contact[i].geom.normal);
                        if( checkgeom1 != contact[i].geom.g1 )
                            vnorm = -vnorm;
                        _pCurrentReport->contacts.push_back(COLLISIONREPORT::CONTACT(contact[i].geom.pos,vnorm,contact[i].geom.depth));
                    }
                }
            }

            _bCollision = true;
        }
    }
}

bool ODECollisionChecker::CheckCollision(const KinBody::Link* plink, COLLISIONREPORT* pReport)
{
    _bCollision = false;
    _pCurrentReport = pReport;
    if( pReport != NULL ) pReport->Reset(_options);

    if( plink == NULL || !plink->IsEnabled() ) {
        RAVELOG(L"calling collision on disabled link %S\n", plink != NULL ? plink->GetName() : L"()");
        return false;
    }

    _plink = plink;
    odespace.Synchronize();
    dSpaceCollide(odespace.GetSpace(), this, LinkCollisionCallback);

    if( _bCollision )
        return true;

    return _bCollision;
}

bool ODECollisionChecker::CheckCollision(const KinBody::Link* plink1, const KinBody::Link* plink2, COLLISIONREPORT* pReport)
{
    _pCurrentReport = pReport;
    if( pReport != NULL ) pReport->Reset(_options);

    if( plink1 == NULL || !plink1->IsEnabled() ) {
        RAVELOG(L"calling collision on disabled link1 %S\n", plink1 != NULL ? plink1->GetName() : L"()");
        return false;
    }
    if( plink2 == NULL || !plink2->IsEnabled() ) {
        RAVELOG(L"calling collision on disabled link2 %S\n", plink2 != NULL ? plink2->GetName() : L"()");
        return false;
    }

    odespace.Synchronize();

    dContact contact[16];
    dGeomID geom1 = odespace.GetLinkGeom(plink1);
    while(geom1 != NULL) {

        assert(dGeomIsEnabled(geom1));

        dGeomID geom2 = odespace.GetLinkGeom(plink2);

        while(geom2 != NULL) {

            assert(dGeomIsEnabled(geom2));

            int N = dCollide (geom1, geom2,16,&contact[0].geom,sizeof(dContact));
            if (N) {

                if( N > 0 && _pCurrentReport != NULL ) {
                    _pCurrentReport->numCols = N;
                    _pCurrentReport->plink1 = (KinBody::Link*)plink1;
                    _pCurrentReport->plink2 = (KinBody::Link*)plink2;

                    if( _pCurrentReport->options & OpenRAVE::CO_Contacts ) {
                        _pCurrentReport->contacts.reserve(N);
                        dGeomID checkgeom1 = dGeomGetClass(geom1) == dGeomTransformClass ? dGeomTransformGetGeom(geom1) : geom1;
                        for(int i = 0; i < N; ++i) {
                            //assert(contact[i].geom.depth >= 0);
                            Vector vnorm(contact[i].geom.normal);
                            if( checkgeom1 != contact[i].geom.g1 )
                                vnorm = -vnorm;
                            assert( checkgeom1 == contact[i].geom.g1 || checkgeom1 == contact[i].geom.g2 );
                            _pCurrentReport->contacts.push_back(COLLISIONREPORT::CONTACT(contact[i].geom.pos, vnorm, contact[i].geom.depth));
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

bool ODECollisionChecker::CheckCollision(const KinBody::Link* plink, const KinBody* pbody, COLLISIONREPORT* pReport)
{
    if( pbody == NULL || pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
        RAVELOG(L"body %S not valid\n", pbody != NULL ? pbody->GetName() : L"()");
        return false;
    }

    if( plink == NULL || !plink->IsEnabled() ) {
        RAVELOG(L"calling collision on disabled link %S\n", plink != NULL ? plink->GetName() : L"()");
        return false;
    }

    odespace.Synchronize();
    FOREACHC(itlink, pbody->GetLinks()) {
        if( CheckCollision(plink, *itlink, pReport) )
            return true;
    }
    return false;

    // doesn't work
//    _bCollision = false;
//    _pCurrentReport = pReport;
//    if( pReport != NULL ) pReport->Reset(_options);
//
//    if( !dBodyIsEnabled(plink1->GetBody()) ) {
//        RAVELOG(L"calling collision on disabled link\n");
//        return false;
//    }
//
//    dSpaceCollide2((dGeomID)pbody->GetSpace(), plink1->GetGeom(), plink1, LinkCollisionCallback);
//
//    if( _bCollision )
//        return true;
//
//    return _bCollision;
}

void ODECollisionChecker::RayCollisionCallback (void *data, dGeomID o1, dGeomID o2)
{
    void** pdata = (void**)data;
    ((ODECollisionChecker*)pdata[0])->_RayCollisionCallback(o1,o2,*(dReal*)pdata[1]);
}

void ODECollisionChecker::_RayCollisionCallback (dGeomID o1, dGeomID o2,dReal fmaxdist)
{
    if( _bCollision )
        return; // don't test anymore

    if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) )
        return;

    if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {    
        dSpaceCollide2(o1,o2,this,RayCollisionCallback);
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
            if( contact[index].geom.depth <= fmaxdist )
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
        assert( b != NULL );

        if( _pCurrentReport != NULL ) {

            if( _pCurrentReport->numCols ) {
                // collided already, see if this point is closer
                if( _pCurrentReport->minDistance < contact[index].geom.depth )
                    return;
            }

            _pCurrentReport->numCols = 1;
            _pCurrentReport->minDistance = contact[index].geom.depth;
            _pCurrentReport->plink1 = (KinBody::Link*)dBodyGetData(b);

            // always return contacts since it isn't that much computation (openravepy expects this!)
            //if( _pCurrentReport->options & OpenRAVE::CO_Contacts) {
                Vector vnorm(contact[index].geom.normal);
                if( contact[index].geom.g1 != geomray )
                    vnorm = -vnorm;
                if( _pCurrentReport->contacts.size() == 0 ) {
                    _pCurrentReport->contacts.push_back(COLLISIONREPORT::CONTACT(contact[index].geom.pos, vnorm, contact[index].geom.depth));
                }
                else
                    _pCurrentReport->contacts.front() = COLLISIONREPORT::CONTACT(contact[index].geom.pos, vnorm, contact[index].geom.depth);
            //}

            if( _pCurrentReport->options&OpenRAVE::CO_RayAnyHit )
                _bCollision = true;
        }
        else
            _bCollision = true;
    }
}

bool ODECollisionChecker::CheckCollision(const RAY& ray, const KinBody::Link* plink, COLLISIONREPORT* pReport)
{
    if( pReport != NULL ) pReport->Reset(_options);
    
    if( plink == NULL || !plink->IsEnabled() ) {
        RAVELOG(L"calling collision on disabled link %S\n", plink != NULL ? plink->GetName() : L"()");
        return false;
    }

    odespace.Synchronize();
    dReal fmaxdist = OpenRAVE::RaveSqrt(ray.dir.lengthsqr3());
    if( fabsf(fmaxdist-1) < 1e-4 )
        RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
        
    Vector vnormdir = ray.dir*(1/fmaxdist);
    dGeomRaySet(geomray, ray.pos.x, ray.pos.y, ray.pos.z, vnormdir.x, vnormdir.y, vnormdir.z);
    dGeomRaySetClosestHit(geomray, !(_options&OpenRAVE::CO_RayAnyHit)); // only care about the closest points

    bool bCollision = false;
    dContact contact[16];
    dGeomID geom1 = odespace.GetLinkGeom(plink);
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

            if( pReport != NULL ) {

                if( pReport->numCols ) {
                    // collided already, see if this point is closer
                    if( pReport->minDistance < contact[index].geom.depth ) {
                        geom1 = dGeomGetBodyNext(geom1);
                        continue;
                    }
                }

                pReport->numCols = 1;
                pReport->minDistance = contact[index].geom.depth;
                pReport->plink1 = (OpenRAVE::KinBody::Link*)plink;

                // always return contacts since it isn't that much computation (openravepy expects this!)
                //if( pReport->options & OpenRAVE::CO_Contacts) {
                Vector vnorm(contact[index].geom.normal);
                if( contact[index].geom.g1 != geomray )
                    vnorm = -vnorm;
                if( pReport->contacts.size() == 0 ) {
                    pReport->contacts.push_back(COLLISIONREPORT::CONTACT(contact[index].geom.pos, vnorm, contact[index].geom.depth));
                }
                else
                    pReport->contacts.front() = COLLISIONREPORT::CONTACT(contact[index].geom.pos, vnorm, contact[index].geom.depth);
                //}

                if( pReport->options&OpenRAVE::CO_RayAnyHit ) {
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

bool ODECollisionChecker::CheckCollision(const RAY& ray, const KinBody* pbody, COLLISIONREPORT* pReport)
{
    _bCollision = false;
    _pCurrentReport = pReport;
    if( pReport != NULL ) pReport->Reset(_options);
    

    if( pbody == NULL || pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
        RAVELOG(L"body %S not valid\n", pbody != NULL ? pbody->GetName() : L"()");
        return false;
    }

    odespace.Synchronize();
    dReal fmaxdist = OpenRAVE::RaveSqrt(ray.dir.lengthsqr3());
    if( fabsf(fmaxdist-1) < 1e-4 )
        RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
    Vector vnormdir = ray.dir*(1/fmaxdist);
    dGeomRaySet(geomray, ray.pos.x, ray.pos.y, ray.pos.z, vnormdir.x, vnormdir.y, vnormdir.z);
    dGeomRaySetClosestHit(geomray, !(_options&OpenRAVE::CO_RayAnyHit)); // only care about the closest points

    void* pdata[2] = {this,&fmaxdist};
    //dSpaceAdd(pbody->GetSpace(), geomray);
    dSpaceCollide2((dGeomID)odespace.GetBodySpace(pbody), geomray, pdata, RayCollisionCallback);
    //dSpaceRemove(pbody->GetSpace(), geomray);

    if( pReport != NULL )
        return pReport->numCols >  0;

    return _bCollision;
}

bool ODECollisionChecker::CheckCollision(const RAY& ray, COLLISIONREPORT* pReport)
{
    _bCollision = false;
    _pCurrentReport = pReport;
    if( pReport != NULL ) pReport->Reset(_options);

    dReal fmaxdist = OpenRAVE::RaveSqrt(ray.dir.lengthsqr3());
    Vector vnormdir = ray.dir*(1/fmaxdist);
    if( fabsf(fmaxdist-1) < 1e-4 )
        RAVELOG_DEBUGA("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
    dGeomRaySet(geomray, ray.pos.x, ray.pos.y, ray.pos.z, vnormdir.x, vnormdir.y, vnormdir.z);

    dGeomRaySetClosestHit(geomray, !(_options&OpenRAVE::CO_RayAnyHit)); // only care about the closest points
    dGeomRaySetParams(geomray,0,0);

    odespace.Synchronize();

    void* pdata[2] = {this,&fmaxdist};

    //dSpaceAdd(odespace.GetSpace(), geomray);
    dSpaceCollide2((dGeomID)odespace.GetSpace(), geomray, pdata, RayCollisionCallback);
    //dSpaceRemove(odespace.GetSpace(), geomray);

    if( pReport != NULL )
        return pReport->numCols >  0;

    return _bCollision;
}

bool ODECollisionChecker::CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* preport)
{
    if( vlinkexcluded.size() == 0 && vbodyexcluded.size() == 0 )
        return CheckCollision(plink,preport);

    RAVEPRINT(L"This type of collision checking is not yet implemented in the ODE collision checker.\n"); 
    assert(0);
    odespace.Synchronize();
    return false;
}

bool ODECollisionChecker::CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* pReport)
{
    if( vbodyexcluded.size() == 0 && vlinkexcluded.size() == 0 )
        return CheckCollision(pbody, pReport);

    _bCollision = false;
    _pCurrentReport = pReport;
    if( pReport != NULL ) pReport->Reset(_options);

    if( pbody == NULL || pbody->GetLinks().size() == 0 || !pbody->IsEnabled()  ) {
        RAVELOG(L"body %S not valid\n", pbody != NULL ? pbody->GetName() : L"()");
        return false;
    }

    assert( vlinkexcluded.size() == 0 );
    odespace.Synchronize();

    FOREACHC(itbody, GetEnv()->GetBodies()) {
        if( *itbody == pbody || vbodyexcluded.find(*itbody) != vbodyexcluded.end() )
            continue;

        if( pbody->IsAttached(*itbody) || (*itbody)->IsAttached(pbody) )
            continue;

        dSpaceCollide2((dGeomID)odespace.GetBodySpace(pbody),(dGeomID)odespace.GetBodySpace(*itbody),this,KinBodyKinBodyCollisionCallback);

        if( _bCollision )
            break;
    }

    if( _bCollision )
        return true;

    // check all attached bodies
    FOREACHC(itattached, pbody->GetAttached()) {

        KinBody* pattbody = *itattached;
        if( pattbody == NULL || pattbody->GetLinks().size() == 0 || !pattbody->IsEnabled()  ) {
            RAVELOG(L"body %S not valid\n", pattbody != NULL ? pattbody->GetName() : L"()");
            return false;
        }
        
        FOREACHC(itbody, GetEnv()->GetBodies()) {
            if( *itbody == pattbody || vbodyexcluded.find(*itbody) != vbodyexcluded.end() )
                continue;

            if( pattbody->IsAttached(*itbody) || (*itbody)->IsAttached(pattbody) )
                continue;

            dSpaceCollide2((dGeomID)odespace.GetBodySpace(pattbody),(dGeomID)odespace.GetBodySpace(*itbody),this,KinBodyKinBodyCollisionCallback);

            if( _bCollision )
                break;
        }
    }

    return _bCollision;
}

bool ODECollisionChecker::CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, OpenRAVE::dReal tolerance)
{
    RAVEPRINT(L"This type of collision checking is not yet implemented in the ODE collision checker.\n"); 
    assert(0);
    odespace.Synchronize();
    return false;
}
bool ODECollisionChecker::CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, OpenRAVE::dReal tolerance)
{
    RAVEPRINT(L"This type of collision checking is not yet implemented in the ODE collision checker.\n"); 
    assert(0);
    odespace.Synchronize();
    return false;
}
