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
#ifndef RAVE_COLLISION_ODE
#define RAVE_COLLISION_ODE

#include "odespace.h"

#ifndef ODE_USE_MULTITHREAD
static boost::mutex _mutexode;
static bool _bnotifiedmessage=false;
#endif

class ODECollisionChecker : public OpenRAVE::CollisionCheckerBase
{
    struct COLLISIONCALLBACK
    {
        COLLISIONCALLBACK(boost::shared_ptr<ODECollisionChecker> pchecker, CollisionReportPtr report, KinBodyConstPtr pbody, KinBody::LinkConstPtr plink) : _pchecker(pchecker), _report(report), _pbody(pbody), _plink(plink), _bCollision(false), _bOneCollision(false), fraymaxdist(0), pvbodyexcluded(NULL), pvlinkexcluded(NULL)
        {
            _bHasCallbacks = pchecker->GetEnv()->HasRegisteredCollisionCallbacks();
            if( _bHasCallbacks && !_report ) {
                _report.reset(new CollisionReport());
            }
            if( !!_report ) {
                _report->Reset(pchecker->GetCollisionOptions());
            }
            bActiveDOFs = !!(pchecker->GetCollisionOptions() & OpenRAVE::CO_ActiveDOFs);
        }

        const std::list<EnvironmentBase::CollisionCallbackFn>& GetCallbacks() {
            if( _bHasCallbacks &&( _listcallbacks.size() == 0) ) {
                _pchecker->GetEnv()->GetRegisteredCollisionCallbacks(_listcallbacks);
            }
            return _listcallbacks;
        }

        bool IsActiveLink(KinBodyPtr pbody, int linkindex)
        {
            if( !bActiveDOFs || !_pbody || !_pbody->IsRobot()) {
                return true;
            }
            RobotBaseConstPtr probot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(_pbody);
            if( pbody != _pbody ) {
                // pbody could be attached to a robot's link that is not active!
                KinBody::LinkPtr pgrabbinglink = probot->IsGrabbing(pbody);
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

        boost::shared_ptr<ODECollisionChecker> _pchecker;
        CollisionReportPtr _report;
        KinBodyConstPtr _pbody;
        KinBody::LinkConstPtr _plink;
        bool _bCollision;
        bool _bOneCollision;
        OpenRAVE::dReal fraymaxdist;
        const std::vector<KinBodyConstPtr>* pvbodyexcluded;
        const std::vector<KinBody::LinkConstPtr>* pvlinkexcluded;
private:
        vector<uint8_t> _vactivelinks;     ///< active links for _pbody, only valid if _pbody is a robot
        bool bActiveDOFs;
        bool _bHasCallbacks;
        std::list<EnvironmentBase::CollisionCallbackFn> _listcallbacks;
    };

    inline boost::shared_ptr<ODECollisionChecker> shared_checker() {
        return boost::dynamic_pointer_cast<ODECollisionChecker>(shared_from_this());
    }
    inline boost::shared_ptr<ODECollisionChecker const> shared_checker_const() const {
        return boost::dynamic_pointer_cast<ODECollisionChecker const>(shared_from_this());
    }

public:
    ODECollisionChecker(EnvironmentBasePtr penv) : OpenRAVE::CollisionCheckerBase(penv), odespace(new ODESpace(penv,"odecollision",false)) {
        _options = 0;
        geomray = NULL;
        _nMaxStartContacts = 32;
        _nMaxContacts = 255;     // this is a weird ODE threshold for the new tri-tri collision checker
        __description = ":Interface Author: Rosen Diankov\n\nOpen Dynamics Engine collision checker (fast, but inaccurate for triangle meshes)";
        RegisterCommand("SetMaxContacts",boost::bind(&ODECollisionChecker::_SetMaxContactsCommand, this,_1,_2),
                        str(boost::format("sets the maximum contacts that can be returned by the checker (limit is %d)")%_nMaxContacts));
#ifndef ODE_USE_MULTITHREAD
        if( !_bnotifiedmessage ) {
            RAVELOG_DEBUG("ode will be slow in multi-threaded environments\n");
            _bnotifiedmessage = false;
        }
#endif

        odespace->InitEnvironment();
        geomray = dCreateRay(0, 1000.0f);     // 1000m (is this used?)
    }
    ~ODECollisionChecker() {
        if( geomray != NULL ) {
            dGeomDestroy(geomray);
            geomray = NULL;
        }
        odespace->DestroyEnvironment();
    }

    bool _SetMaxContactsCommand(ostream& sout, istream& sinput)
    {
        sinput >> _nMaxContacts;
        return !!sinput;
    }

    virtual void SetTolerance(OpenRAVE::dReal tolerance) {
    }

    virtual bool InitEnvironment()
    {
//        if( !odespace->InitEnvironment() ) {
//            return false;
//        }
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            InitKinBody(*itbody);
        }
        //geomray = dCreateRay(0, 1000.0f);     // 1000m (is this used?)
        //dGeomRaySetParams(geomray,0,0);
        return true;
    }

    virtual void DestroyEnvironment()
    {
        // go through all the KinBodies and destory their collision pointers
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            (*itbody)->RemoveUserData("odecollision");
        }
    }

    virtual bool InitKinBody(KinBodyPtr pbody)
    {
        ODESpace::KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<ODESpace::KinBodyInfo>(pbody->GetUserData("odecollision"));
        // need the pbody check since kinbodies can be cloned and could have the wrong pointer
        if( !pinfo || pinfo->GetBody() != pbody ) {
            pinfo = odespace->InitKinBody(pbody);
            pbody->SetUserData("odecollision", pinfo);
        }
        return !!pinfo;
    }

    virtual void RemoveKinBody(KinBodyPtr pbody)
    {
        if( !!pbody ) {
            pbody->RemoveUserData("odecollision");
        }
    }

    virtual bool SetCollisionOptions(int collisionoptions)
    {
        _options = collisionoptions;
        if( _options & OpenRAVE::CO_Distance ) {
            return false;
        }
        return true;
    }

    virtual int GetCollisionOptions() const {
        return _options;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,pbody,KinBody::LinkConstPtr());
        if(( pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
            return false;
        }
        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("ode doesn't support CO_Distance\n");
            return false;
        }

#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif
        odespace->Synchronize();
        dSpaceCollide(odespace->GetSpace(), &cb, KinBodyCollisionCallback);
        return cb._bCollision;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,pbody1,KinBody::LinkConstPtr());
        if(( pbody1->GetLinks().size() == 0) || !pbody1->IsEnabled() ) {
            return false;
        }
        if(( pbody2->GetLinks().size() == 0) || !pbody2->IsEnabled() ) {
            return false;
        }
        if( pbody1->IsAttached(pbody2) ) {
            return false;
        }
        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("ode doesn't support CO_Distance\n");
            return false;
        }

#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif
        odespace->Synchronize();

        // have to go through all attached bodies manually (not sure if there's a fast way to set temporary groups in ode)
        std::set<KinBodyPtr> s1, s2;
        pbody1->GetAttached(s1);
        pbody2->GetAttached(s2);
        FOREACH(it1,s1) {
            FOREACH(it2,s2) {
                dSpaceCollide2((dGeomID)odespace->GetBodySpace(*it1),(dGeomID)odespace->GetBodySpace(*it2),&cb,KinBodyKinBodyCollisionCallback);
                if( cb._bCollision ) {
                    return true;
                }
            }
        }

        return cb._bCollision;
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,KinBodyPtr(),plink);
        if( !plink->IsEnabled() ) {
            RAVELOG_VERBOSE("calling collision on disabled link %s\n", plink->GetName().c_str());
            return false;
        }
        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("ode doesn't support CO_Distance\n");
            return false;
        }

#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif
        odespace->Synchronize();
        dSpaceCollide(odespace->GetSpace(), &cb, LinkCollisionCallback);
        return cb._bCollision;
    }

    int _GeomCollide(dGeomID geom1, dGeomID geom2, vector<dContact>& vcontacts, bool bComputeAllContacts)
    {
        vcontacts.resize(bComputeAllContacts ? _nMaxStartContacts : 1);
        int log2limit = (int)ceil(OpenRAVE::RaveLog(vcontacts.size())/OpenRAVE::RaveLog(2));
        while( 1 ) {
            int N = dCollide (geom1, geom2,vcontacts.size(),&vcontacts[0].geom,sizeof(vcontacts[0]));
            if(( N > 0) && !bComputeAllContacts ) {
                // not requesting contacts, so return
                vcontacts.resize(N);
                return N;
            }

            // this is weird, but it seems that ode returns approx log2(vcontacts.size()) less than the limit...
            if( N+log2limit < (int)vcontacts.size() ) {
                vcontacts.resize(N);
                return N;
            }
            if( vcontacts.size() >= _nMaxContacts ) {
                vcontacts.resize(N);
                break;
            }
            vcontacts.resize(min(_nMaxContacts,vcontacts.size()*2));
            log2limit += 1;
        }
        RAVELOG_WARN(str(boost::format("max contacts %d reached, but still more contacts left! If this is a problem, try increasing the limit with the SetMaxContracts command")%_nMaxContacts));
        return vcontacts.size();
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report)
    {
        if( !!report ) {
            report->Reset(_options);
        }
        if( !plink1->IsEnabled() ) {
            //RAVELOG_VERBOSE(str(boost::format("calling collision on disabled link1 %s\n")%plink1->GetName()));
            return false;
        }
        if( !plink2->IsEnabled() ) {
            //RAVELOG_VERBOSE(str(boost::format("calling collision on disabled link2 %s\n")%plink2->GetName()));
            return false;
        }
        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("ode doesn't support CO_Distance\n");
            return false;
        }

#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif

        odespace->Synchronize();
        return _CheckCollision(plink1,plink2,report);
    }

    bool _CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report)
    {
        bool bHasCallbacks = GetEnv()->HasRegisteredCollisionCallbacks();
        std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;

        vector<dContact> vcontacts;
        dGeomID geom1 = odespace->GetLinkGeom(plink1);
        int igeom1 = 0;
        while(geom1 != NULL) {
            BOOST_ASSERT(dGeomIsEnabled(geom1));
            dGeomID geom2 = odespace->GetLinkGeom(plink2);
            int igeom2 = 0;
            while(geom2 != NULL) {
                BOOST_ASSERT(dGeomIsEnabled(geom2));

                int N = _GeomCollide(geom1, geom2,vcontacts, !!report && !!(report->options & OpenRAVE::CO_Contacts));
                if (N) {
                    if( !report && bHasCallbacks ) {
                        report.reset(new CollisionReport());
                        report->Reset(_options);
                    }

                    if(( N > 0) && !!report ) {
                        report->numCols = N;
                        report->plink1 = plink1;
                        report->plink2 = plink2;

                        if( report->options & OpenRAVE::CO_Contacts ) {
                            report->contacts.reserve(N);
                            dGeomID checkgeom1 = dGeomGetClass(geom1) == dGeomTransformClass ? dGeomTransformGetGeom(geom1) : geom1;
                            for(int i = 0; i < N; ++i) {
                                //assert(contact[i].geom.depth >= 0);
                                Vector vnorm(vcontacts[i].geom.normal);
                                dReal distance = vcontacts[i].geom.depth;
                                if( checkgeom1 != vcontacts[i].geom.g1 ) {
                                    vnorm = -vnorm;
                                    distance = -distance;
                                }
                                BOOST_ASSERT( checkgeom1 == vcontacts[i].geom.g1 || checkgeom1 == vcontacts[i].geom.g2 );
                                if( !!report->plink2 && report->plink2->ValidateContactNormal(vcontacts[i].geom.pos,vnorm)) {
                                    distance = -distance;
                                }
                                report->contacts.push_back(CollisionReport::CONTACT(vcontacts[i].geom.pos, vnorm, distance));
                            }
                        }

                        if( bHasCallbacks ) {
                            if( listcallbacks.size() == 0 ) {
                                GetEnv()->GetRegisteredCollisionCallbacks(listcallbacks);
                            }
                            FOREACHC(itfn, listcallbacks) {
                                OpenRAVE::CollisionAction action = (*itfn)(report,false);
                                if( action != OpenRAVE::CA_DefaultAction ) {
                                    report->Reset();
                                    return false;
                                }
                            }
                        }
                    }

                    return true;
                }

                geom2 = dBodyGetNextGeom(geom2);
                ++igeom2;
            }

            geom1 = dBodyGetNextGeom(geom1);
            ++igeom1;
        }

        return false;
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if( !!report ) {
            report->Reset(_options);
        }
        if(( pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
            return false;
        }
        if( !plink->IsEnabled() ) {
            //RAVELOG_VERBOSE("calling collision on disabled link %s\n", plink->GetName().c_str());
            return false;
        }
        if( pbody->IsAttached(plink->GetParent()) ) {
            return false;
        }
        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("ode doesn't support CO_Distance\n");
            return false;
        }

#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif

        odespace->Synchronize();
        COLLISIONCALLBACK cb(shared_checker(),report,KinBodyPtr(),KinBody::LinkConstPtr());

        std::set<KinBodyPtr> setattached;
        pbody->GetAttached(setattached);
        FOREACH(itbody,setattached) {
            FOREACHC(itlink, (*itbody)->GetLinks()) {
                if( cb.IsActiveLink(*itbody,(*itlink)->GetIndex()) ) {
                    if( _CheckCollision(plink, KinBody::LinkConstPtr(*itlink), report) ) {
                        return true;
                    }
                }
            }
        }

        return false;

        // doesn't work, but why?
        //    dSpaceCollide2((dGeomID)pbody->GetSpace(), plink1->GetGeom(), plink1, LinkCollisionCallback);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        if(( vlinkexcluded.size() == 0) &&( vbodyexcluded.size() == 0) ) {
            return CheckCollision(plink,report);
        }
        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("ode doesn't support CO_Distance\n");
            return false;
        }
        throw openrave_exception("This type of collision checking is not yet implemented in the ODE collision checker.\n",OpenRAVE::ORE_NotImplemented);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,pbody,KinBody::LinkConstPtr());
        if(( pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
            return false;
        }
        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("ode doesn't support CO_Distance\n");
            return false;
        }
        if( vbodyexcluded.size() > 0 ) {
            cb.pvbodyexcluded = &vbodyexcluded;
        }
        if( vlinkexcluded.size() > 0 ) {
            cb.pvlinkexcluded = &vlinkexcluded;
        }
#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif

        odespace->Synchronize();
        dSpaceCollide(odespace->GetSpace(), &cb, KinBodyCollisionCallback);
        return cb._bCollision;
    }

    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        if( !!report ) {
            report->Reset(_options);
        }
        if( !plink->IsEnabled() ) {
            RAVELOG_VERBOSE("calling collision on disabled link %s\n", plink->GetName().c_str());
            return false;
        }

#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif

        odespace->Synchronize();
        OpenRAVE::dReal fmaxdist = OpenRAVE::RaveSqrt(ray.dir.lengthsqr3());
        if( RaveFabs(fmaxdist-1) < 1e-4 ) {
            RAVELOG_DEBUG("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
        }

        Vector vnormdir = ray.dir*(1/fmaxdist);
        dGeomRaySet(geomray, ray.pos.x, ray.pos.y, ray.pos.z, vnormdir.x, vnormdir.y, vnormdir.z);
        dGeomRaySetClosestHit(geomray, !(_options&OpenRAVE::CO_RayAnyHit));     // only care about the closest points
        dGeomRaySetLength(geomray,fmaxdist);
        dGeomRaySetParams(geomray,0,0);

        bool bHasCallbacks = GetEnv()->HasRegisteredCollisionCallbacks();
        std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;

        bool bCollision = false;
        vector<dContact> vcontacts;
        dGeomID geom1 = odespace->GetLinkGeom(plink);
        while(geom1 != NULL) {
            BOOST_ASSERT(dGeomIsEnabled(geom1));

            int N = _GeomCollide(geom1, geomray,vcontacts, !!report && !!(report->options & OpenRAVE::CO_Contacts));
            if (N > 0) {

                if( !report && bHasCallbacks ) {
                    report.reset(new CollisionReport());
                    report->Reset(_options);
                }

                int index = 0;
                for(; index < N; ++index) {
                    if( vcontacts[index].geom.depth <= fmaxdist )
                        break;
                }

                if( index >= N ) {
                    geom1 = dBodyGetNextGeom(geom1);
                    continue;
                }

                if( !!report ) {

                    if( report->numCols ) {
                        // collided already, see if this point is closer
                        if( report->minDistance < vcontacts[index].geom.depth ) {
                            geom1 = dBodyGetNextGeom(geom1);
                            continue;
                        }
                    }

                    report->numCols = 1;
                    report->minDistance = vcontacts[index].geom.depth;
                    report->plink1 = plink;

                    // always return contacts since it isn't that much computation (openravepy expects this!)
                    //if( report->options & OpenRAVE::CO_Contacts) {
                    Vector vnorm(vcontacts[index].geom.normal);
                    dReal distance = vcontacts[index].geom.depth;
                    if( vcontacts[index].geom.g1 != geomray ) {
                        vnorm = -vnorm;
                        distance = -distance;
                    }
                    if( !!report->plink1 && report->plink1->ValidateContactNormal(vcontacts[index].geom.pos,vnorm) ) {
                        distance = -distance;
                    }
                    if( report->contacts.size() == 0 ) {
                        report->contacts.push_back(CollisionReport::CONTACT(vcontacts[index].geom.pos, vnorm, distance));
                    }
                    else {
                        report->contacts.front() = CollisionReport::CONTACT(vcontacts[index].geom.pos, vnorm, distance);
                    }
                    if( listcallbacks.size() == 0 ) {
                        GetEnv()->GetRegisteredCollisionCallbacks(listcallbacks);
                    }
                    FOREACHC(itfn, listcallbacks) {
                        OpenRAVE::CollisionAction action = (*itfn)(report,false);
                        if( action != OpenRAVE::CA_DefaultAction ) {
                            report->Reset();
                            return false;
                        }
                    }

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

            geom1 = dBodyGetNextGeom(geom1);
        }

        return bCollision;
    }

    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,pbody,KinBody::LinkConstPtr());
        if(( pbody->GetLinks().size() == 0) || !pbody->IsEnabled() ) {
            return false;
        }

#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif

        odespace->Synchronize();
        cb.fraymaxdist = OpenRAVE::RaveSqrt(ray.dir.lengthsqr3());
        if( RaveFabs(cb.fraymaxdist-1) < 1e-4 ) {
            RAVELOG_DEBUG("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
        }
        Vector vnormdir = ray.dir*(1/cb.fraymaxdist);
        dGeomRaySet(geomray, ray.pos.x, ray.pos.y, ray.pos.z, vnormdir.x, vnormdir.y, vnormdir.z);
        dGeomRaySetClosestHit(geomray, !(_options&OpenRAVE::CO_RayAnyHit)); // only care about the closest points
        dGeomRaySetLength(geomray,cb.fraymaxdist);
        dGeomRaySetParams(geomray,0,0);
        //dSpaceAdd(pbody->GetSpace(), geomray);
        dSpaceCollide2((dGeomID)odespace->GetBodySpace(pbody), geomray, &cb, RayCollisionCallback);
        //dSpaceRemove(pbody->GetSpace(), geomray);
        return cb._bOneCollision;
    }

    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report)
    {
        COLLISIONCALLBACK cb(shared_checker(),report,KinBodyPtr(),KinBody::LinkConstPtr());
        cb.fraymaxdist = OpenRAVE::RaveSqrt(ray.dir.lengthsqr3());

        Vector vnormdir;
        if( cb.fraymaxdist > 0 ) {
            vnormdir = ray.dir*(1/cb.fraymaxdist);
        }
        else {
            vnormdir = ray.dir;
        }
        if( RaveFabs(cb.fraymaxdist-1) < 1e-4 ) {
            RAVELOG_DEBUG("CheckCollision: ray direction length is 1.0, note that only collisions within a distance of 1.0 will be checked\n");
        }

#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif
        dGeomRaySet(geomray, ray.pos.x, ray.pos.y, ray.pos.z, vnormdir.x, vnormdir.y, vnormdir.z);

        dGeomRaySetClosestHit(geomray, !(_options&OpenRAVE::CO_RayAnyHit));     // only care about the closest points
        dGeomRaySetLength(geomray,cb.fraymaxdist);
        dGeomRaySetParams(geomray,0,0);

        odespace->Synchronize();
        //dSpaceAdd(odespace->GetSpace(), geomray);
        dSpaceCollide2((dGeomID)odespace->GetSpace(), geomray, &cb, RayCollisionCallback);
        //dSpaceRemove(odespace->GetSpace(), geomray);
        return cb._bOneCollision;
    }

    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("ode doesn't support CO_Distance\n");
            return false;
        }
        if( !!report ) {
            report->Reset(_options);
        }
        if( pbody->GetLinks().size() <= 1 ) {
            return false;
        }

        int adjacentoptions = KinBody::AO_Enabled;
        if( (_options&OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
            adjacentoptions |= KinBody::AO_ActiveDOFs;
        }

        const std::set<int>& nonadjacent = pbody->GetNonAdjacentLinks(adjacentoptions);

#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif
        odespace->Synchronize(); // call after GetNonAdjacentLinks since it can modify the body, even though it is const!
        FOREACHC(itset, nonadjacent) {
            KinBody::LinkConstPtr plink1(pbody->GetLinks().at(*itset&0xffff)), plink2(pbody->GetLinks().at(*itset>>16));
            if( _CheckCollision(plink1,plink2, report) ) {
                if( IS_DEBUGLEVEL(OpenRAVE::Level_Verbose) ) {
                    RAVELOG_VERBOSE(str(boost::format("selfcol %s, Links %s %s are colliding\n")%pbody->GetName()%plink1->GetName()%plink2->GetName()));
                    std::vector<OpenRAVE::dReal> v;
                    pbody->GetDOFValues(v);
                    stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                    for(size_t i = 0; i < v.size(); ++i ) {
                        if( i > 0 ) {
                            ss << "," << v[i];
                        }
                        else {
                            ss << "colvalues=[" << v[i];
                        }
                    }
                    ss << "]";
                    RAVELOG_VERBOSE(ss.str());
                }
                return true;
            }
        }
        return false;
    }

    virtual bool CheckSelfCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        if( _options & OpenRAVE::CO_Distance ) {
            RAVELOG_WARN("ode doesn't support CO_Distance\n");
            return false;
        }
        if( !!report ) {
            report->Reset(_options);
        }
        KinBodyPtr pbody = plink->GetParent();
        if( pbody->GetLinks().size() <= 1 ) {
            return false;
        }

        int adjacentoptions = KinBody::AO_Enabled;
        if( (_options&OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
            adjacentoptions |= KinBody::AO_ActiveDOFs;
        }

        const std::set<int>& nonadjacent = pbody->GetNonAdjacentLinks(adjacentoptions);

#ifndef ODE_USE_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutexode);
#endif
        odespace->Synchronize(); // call after GetNonAdjacentLinks since it can modify the body, even though it is const!
        FOREACHC(itset, nonadjacent) {
            KinBody::LinkConstPtr plink1(pbody->GetLinks().at(*itset&0xffff)), plink2(pbody->GetLinks().at(*itset>>16));
            if( plink == plink1 || plink == plink2 ) {
                if( _CheckCollision(plink1,plink2, report) ) {
                    if( IS_DEBUGLEVEL(OpenRAVE::Level_Verbose) ) {
                        RAVELOG_VERBOSE(str(boost::format("selfcol %s, Links %s %s are colliding\n")%pbody->GetName()%plink1->GetName()%plink2->GetName()));
                        std::vector<OpenRAVE::dReal> v;
                        pbody->GetDOFValues(v);
                        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                        for(size_t i = 0; i < v.size(); ++i ) {
                            if( i > 0 ) {
                                ss << "," << v[i];
                            }
                            else {
                                ss << "colvalues=[" << v[i];
                            }
                        }
                        ss << "]";
                        RAVELOG_VERBOSE(ss.str());
                    }
                    return true;
                }
            }
        }
        return false;
    }

private:
    static void KinBodyCollisionCallback (void *data, dGeomID o1, dGeomID o2)
    {
        COLLISIONCALLBACK* pcb = (COLLISIONCALLBACK*)data;
        pcb->_pchecker->_KinBodyCollisionCallback(o1,o2,pcb);
    }

    void _KinBodyCollisionCallback (dGeomID o1, dGeomID o2, COLLISIONCALLBACK* pcb)
    {
        if( pcb->_bCollision ) {
            return;     // don't test anymore
        }
        // ASSUMPTION: every space is attached to a KinBody!
        if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) ) {
            return;
        }

        ODESpace::KinBodyInfo* o1data = (ODESpace::KinBodyInfo*)dGeomGetData(o1);
        ODESpace::KinBodyInfo* o2data = (ODESpace::KinBodyInfo*)dGeomGetData(o2);
        KinBodyConstPtr pbody1,pbody2;

        if( dGeomIsSpace(o1) &&( o1data != NULL) ) {
            pbody1 = KinBodyConstPtr(o1data->GetBody());
        }
        if( dGeomIsSpace(o2) &&( o2data != NULL) ) {
            pbody2 = KinBodyConstPtr(o2data->GetBody());
        }
        if( !!pcb->pvbodyexcluded ) {
            if( (!!pbody1 &&( std::find(pcb->pvbodyexcluded->begin(),pcb->pvbodyexcluded->end(),pbody1) != pcb->pvbodyexcluded->end()) ) || (!!pbody2 &&( std::find(pcb->pvbodyexcluded->begin(),pcb->pvbodyexcluded->end(),pbody2) != pcb->pvbodyexcluded->end()) ) ) {
                return;
            }
        }

        // only recurse two spaces if exactly one of them is attached to _pbody
        if( !!pbody1 && !!pbody2 &&( pcb->_pbody->IsAttached(pbody1) == pcb->_pbody->IsAttached(pbody2)) ) {
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
        if(!!b1 && dBodyGetData(b1)) {
            pkb1 = ((ODESpace::KinBodyInfo::LINK*)dBodyGetData(b1))->GetLink();
            if( !!pkb1 ) {
                if( !pkb1->IsEnabled() || !pcb->IsActiveLink(pkb1->GetParent(),pkb1->GetIndex()) ) {
                    return;
                }
            }
        }
        if(!!b2 && dBodyGetData(b1)) {
            pkb2 = ((ODESpace::KinBodyInfo::LINK*)dBodyGetData(b2))->GetLink();
            if( !!pkb2 ) {
                if( !pkb2->IsEnabled() || !pcb->IsActiveLink(pkb2->GetParent(),pkb2->GetIndex()) ) {
                    return;
                }
            }
        }

        // redundant but necessary for some versions of ODE
        if( !!pkb1 && !!pkb2 ) {
            if( pkb1->GetParent()->IsAttached(KinBodyConstPtr(pkb2->GetParent())) ) {
                return;
            }
            if( !!pcb->pvbodyexcluded ) {
                if(( std::find(pcb->pvbodyexcluded->begin(),pcb->pvbodyexcluded->end(),KinBodyConstPtr(pkb1->GetParent())) != pcb->pvbodyexcluded->end()) ||
                   ( std::find(pcb->pvbodyexcluded->begin(),pcb->pvbodyexcluded->end(),KinBodyConstPtr(pkb2->GetParent())) != pcb->pvbodyexcluded->end()) ) {
                    return;
                }
            }
            if( !!pcb->pvlinkexcluded ) {
                if(( std::find(pcb->pvlinkexcluded->begin(),pcb->pvlinkexcluded->end(),KinBody::LinkConstPtr(pkb1)) != pcb->pvlinkexcluded->end()) ||
                   ( std::find(pcb->pvlinkexcluded->begin(),pcb->pvlinkexcluded->end(),KinBody::LinkConstPtr(pkb2)) != pcb->pvlinkexcluded->end()) ) {
                    return;
                }
            }
        }

        vector<dContact> vcontacts;
        int N = _GeomCollide(o1,o2,vcontacts, !!pcb->_report && !!(pcb->_report->options & OpenRAVE::CO_Contacts));
        if ( N ) {
            if(( N > 0) && !!pcb->_report ) {
                pcb->_report->numCols = N;
                pcb->_report->minDistance = vcontacts[0].geom.depth;
                pcb->_report->plink1 = pkb1;
                pcb->_report->plink2 = pkb2;

                if( pcb->_report->options & OpenRAVE::CO_Contacts ) {
                    pcb->_report->contacts.reserve(N);
                    dGeomID checkgeom1 = dGeomGetClass(o1) == dGeomTransformClass ? dGeomTransformGetGeom(o1) : o1;
                    for(int i = 0; i < N; ++i) {
                        dReal distance = vcontacts[i].geom.depth;
                        Vector vnorm(vcontacts[i].geom.normal);
                        if( checkgeom1 != vcontacts[i].geom.g1 ) {
                            vnorm = -vnorm;
                            distance = -distance;
                        }
                        if( !!pcb->_report->plink2 && pcb->_report->plink2->ValidateContactNormal(vcontacts[i].geom.pos,vnorm) ) {
                            distance = -distance;
                        }
                        pcb->_report->contacts.push_back(CollisionReport::CONTACT(vcontacts[i].geom.pos, vnorm, distance));
                    }
                }

                FOREACHC(itfn, pcb->GetCallbacks()) {
                    OpenRAVE::CollisionAction action = (*itfn)(pcb->_report,false);
                    if( action != OpenRAVE::CA_DefaultAction ) {
                        pcb->_report->Reset();
                        return;
                    }
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
        if( pcb->_bCollision ) {
            return;     // don't test anymore
        }
        if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) ) {
            return;
        }
        if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
            dSpaceCollide2(o1,o2,pcb,KinBodyKinBodyCollisionCallback);
            return;
        }

        // ignore static, static collisions
        dBodyID b1,b2;
        b1 = dGeomGetBody(o1);
        b2 = dGeomGetBody(o2);

        KinBody::LinkPtr pkb1,pkb2;
        if(!!b1 && dBodyGetData(b1)) {
            pkb1 = ((ODESpace::KinBodyInfo::LINK*)dBodyGetData(b1))->GetLink();
        }
        if(!!b2 && dBodyGetData(b1)) {
            pkb2 = ((ODESpace::KinBodyInfo::LINK*)dBodyGetData(b2))->GetLink();
        }
        if( !!pkb1 ) {
            if( !pkb1->IsEnabled() || !pcb->IsActiveLink(pkb1->GetParent(),pkb1->GetIndex()) ) {
                return;
            }
        }
        if( !!pkb2 ) {
            if( !pkb2->IsEnabled() || !pcb->IsActiveLink(pkb2->GetParent(),pkb2->GetIndex()) ) {
                return;
            }
        }
        if( !!pkb1 && !!pkb2 && pkb1->GetParent()->IsAttached(KinBodyConstPtr(pkb2->GetParent())) ) {
            return;
        }

        // only care if one of the bodies is the link
        vector<dContact> vcontacts;
        int N = _GeomCollide(o1,o2,vcontacts, !!pcb->_report && !!(pcb->_report->options & OpenRAVE::CO_Contacts));
        if ( N ) {
            if(( N > 0) && !!pcb->_report ) {
                pcb->_report->numCols = N;
                pcb->_report->plink1 = pkb1;
                pcb->_report->plink2 = pkb2;

                if( pcb->_report->options & OpenRAVE::CO_Contacts ) {
                    pcb->_report->contacts.reserve(N);
                    dGeomID checkgeom1 = dGeomGetClass(o1) == dGeomTransformClass ? dGeomTransformGetGeom(o1) : o1;
                    for(int i = 0; i < N; ++i) {
                        dReal distance = vcontacts[i].geom.depth;
                        Vector vnorm(vcontacts[i].geom.normal);
                        if( checkgeom1 != vcontacts[i].geom.g1 ) {
                            vnorm = -vnorm;
                            distance = -distance;
                        }
                        if( !!pcb->_report->plink2 && pcb->_report->plink2->ValidateContactNormal(vcontacts[i].geom.pos,vnorm) ) {
                            distance = -distance;
                        }
                        pcb->_report->contacts.push_back(CollisionReport::CONTACT(vcontacts[i].geom.pos,vnorm,distance));
                    }
                }
            }

            FOREACHC(itfn, pcb->GetCallbacks()) {
                OpenRAVE::CollisionAction action = (*itfn)(pcb->_report,false);
                if( action != OpenRAVE::CA_DefaultAction ) {
                    pcb->_report->Reset();
                    return;
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
        if( pcb->_bCollision ) {
            return;     // don't test anymore
        }
        KinBodyPtr pbody = pcb->_plink->GetParent();

        ODESpace::KinBodyInfo* o1data = (ODESpace::KinBodyInfo*)dGeomGetData(o1);
        ODESpace::KinBodyInfo* o2data = (ODESpace::KinBodyInfo*)dGeomGetData(o2);

        // ASSUMPTION: every space is attached to a KinBody!
        if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) ) {
            return;
        }
        // only recurse two spaces if one of them is pbody
        if( dGeomIsSpace(o1) && dGeomIsSpace(o2) && !(( o1data != NULL) &&( o1data->GetBody() == pbody) ) && !(( o2data != NULL) &&( o2data->GetBody() == pbody) ) ) {
            return;
        }
        if( dGeomIsSpace(o1) ) {
            BOOST_ASSERT(!!o1data);
            if(( pbody != o1data->GetBody()) && pbody->IsAttached(KinBodyConstPtr(o1data->GetBody())) ) {
                return;
            }
        }
        if( dGeomIsSpace(o2) ) {
            BOOST_ASSERT(!!o2data);
            if(( pbody != o2data->GetBody()) && pbody->IsAttached(KinBodyConstPtr(o2data->GetBody())) ) {
                return;
            }
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
        if(!!b1 && dBodyGetData(b1)) {
            pkb1 = ((ODESpace::KinBodyInfo::LINK*)dBodyGetData(b1))->GetLink();
            if( !!pkb1 && !pkb1->IsEnabled() ) {
                return;
            }
        }
        if(!!b2 && dBodyGetData(b1)) {
            pkb2 = ((ODESpace::KinBodyInfo::LINK*)dBodyGetData(b2))->GetLink();
            if( !!pkb2 && !pkb2->IsEnabled() ) {
                return;
            }
        }

        if( !!pkb1 && !!pkb2 && pkb1->GetParent()->IsAttached(KinBodyConstPtr(pkb2->GetParent())) ) {
            return;
        }

        // only care if one of the bodies is the link
        if(( pkb1 == pcb->_plink) ||( pkb2 == pcb->_plink) ) {
            vector<dContact> vcontacts;
            int N = _GeomCollide(o1,o2,vcontacts, !!pcb->_report && !!(pcb->_report->options & OpenRAVE::CO_Contacts));
            if (N) {
                if(( N > 0) && !!pcb->_report ) {
                    pcb->_report->numCols = N;
                    pcb->_report->plink1 = pcb->_plink;
                    pcb->_report->plink2 = pkb1 != pcb->_plink ? pkb1 : pkb2;
                    dGeomID checkgeom1 = pkb1 == pcb->_plink ? o1 : o2;
                    checkgeom1 = dGeomGetClass(checkgeom1) == dGeomTransformClass ? dGeomTransformGetGeom(checkgeom1) : checkgeom1;

                    if( pcb->_report->options & OpenRAVE::CO_Contacts ) {
                        pcb->_report->contacts.reserve(N);
                        for(int i = 0; i < N; ++i) {
                            BOOST_ASSERT( checkgeom1 == vcontacts[i].geom.g1 || checkgeom1 == vcontacts[i].geom.g2 );
                            Vector vnorm(vcontacts[i].geom.normal);
                            dReal distance = vcontacts[i].geom.depth;
                            if( checkgeom1 != vcontacts[i].geom.g1 ) {
                                vnorm = -vnorm;
                                distance = -distance;
                            }
                            if( !!pcb->_report->plink2 && pcb->_report->plink2->ValidateContactNormal(vcontacts[i].geom.pos,vnorm) ) {
                                distance = -distance;
                            }
                            pcb->_report->contacts.push_back(CollisionReport::CONTACT(vcontacts[i].geom.pos,vnorm,distance));
                        }
                    }
                }

                FOREACHC(itfn, pcb->GetCallbacks()) {
                    OpenRAVE::CollisionAction action = (*itfn)(pcb->_report,false);
                    if( action != OpenRAVE::CA_DefaultAction ) {
                        pcb->_report->Reset();
                        return;
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
        if( pcb->_bCollision ) {
            return;     // don't test anymore
        }
        if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) ) {
            return;
        }
        if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
            dSpaceCollide2(o1,o2,pcb,RayCollisionCallback);
            return;
        }

        // ignore if a ray isn't included
        if(( dGeomGetClass(o1) != dRayClass) &&( dGeomGetClass(o2) != dRayClass) ) {
            return;
        }

        dGeomID geomray = o2;
        dBodyID b = dGeomGetBody(o1);
        if( b == NULL ) {
            BOOST_ASSERT( dGeomGetClass(o1) == dRayClass );
            geomray = o1;
            b = dGeomGetBody(o2);
        }
        BOOST_ASSERT( b != NULL );
        KinBody::LinkPtr plink;
        if( dBodyGetData(b) ) {
            plink = ((ODESpace::KinBodyInfo::LINK*)dBodyGetData(b))->GetLink();
            if( !!plink ) {
                if( !plink->IsEnabled() || !pcb->IsActiveLink(plink->GetParent(),plink->GetIndex()) ) {
                    return;
                }
            }
        }

        dContact contact[2];
        int N = dCollide (o1,o2,2,&contact[0].geom,sizeof(dContact));
        if (N > 0) {

            int index = 0;
            for(; index < N; ++index) {
                if( contact[index].geom.depth <= pcb->fraymaxdist ) {
                    break;
                }
            }
            if( index >= N ) {
                return;
            }

            if( !!pcb->_report ) {
                if( pcb->_bOneCollision ) {
                    // collided already, see if this point is closer
                    if( pcb->_report->minDistance < contact[index].geom.depth ) {
                        return;
                    }
                }

                pcb->_report->numCols = 1;
                pcb->_report->minDistance = contact[index].geom.depth;
                if( dBodyGetData(b) ) {
                    pcb->_report->plink1 = ((ODESpace::KinBodyInfo::LINK*)dBodyGetData(b))->GetLink();
                }
                else {
                    RAVELOG_WARN("ode body does not have a link attached\n");
                }
                // always return contacts since it isn't that much computation (openravepy expects this!)
                //if( pcb->_report->options & OpenRAVE::CO_Contacts) {
                Vector vnorm(contact[index].geom.normal);
                dReal distance = contact[index].geom.depth;
                if( contact[index].geom.g1 != geomray ) {
                    vnorm = -vnorm;
                    distance = -distance;
                }
                if( !!pcb->_report->plink1 && pcb->_report->plink1->ValidateContactNormal(contact[index].geom.pos,vnorm) ) {
                    distance = -distance;
                }
                if( pcb->_report->contacts.size() == 0 ) {
                    pcb->_report->contacts.push_back(CollisionReport::CONTACT(contact[index].geom.pos, vnorm, distance));
                }
                else {
                    pcb->_report->contacts.front() = CollisionReport::CONTACT(contact[index].geom.pos, vnorm, distance);
                }

                FOREACHC(itfn, pcb->GetCallbacks()) {
                    OpenRAVE::CollisionAction action = (*itfn)(pcb->_report,false);
                    if( action != OpenRAVE::CA_DefaultAction ) {
                        pcb->_report->Reset();
                        return;
                    }
                }

                if( _options&OpenRAVE::CO_RayAnyHit ) {
                    pcb->_bCollision = true;
                }
            }
            else {
                if( _options&OpenRAVE::CO_RayAnyHit ) {
                    pcb->_bCollision = true;
                }
            }

            pcb->_bOneCollision = true;
        }
    }

    int _options;
    dGeomID geomray;     // used for all ray tests
    boost::shared_ptr<ODESpace> odespace;
    size_t _nMaxStartContacts, _nMaxContacts;
};

#endif
