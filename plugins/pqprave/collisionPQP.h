// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>, Dmitry Berenson
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#ifndef  COLPQP_H
#define  COLPQP_H

#include "pqp/PQP.h"

//wrapper class for PQP, distance and tolerance checking is _off_ by default, collision checking is _on_ by default
class CollisionCheckerPQP : public CollisionCheckerBase
{
public:
    class KinBodyInfo : public OpenRAVE::UserData
    {
public:
        KinBodyInfo() : nLastStamp(0) {
        }
        virtual ~KinBodyInfo() {
        }
        KinBodyPtr GetBody() const {
            return _pbody.lock();
        }
        KinBodyWeakPtr _pbody;
        vector<boost::shared_ptr<PQP_Model> > vlinks;
        int nLastStamp;
    };
    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;

    CollisionCheckerPQP(EnvironmentBasePtr penv) : CollisionCheckerBase(penv)
    {
        __description = ":Interface Authors: Dmitry Berenson, Rosen Diankov\n\nPQP collision checker, slow but allows distance queries to objects.";
        _rel_err = 200.0;     //temporary change
        _abs_err = 0.001;       //temporary change
        _tolerance = 0.0;

        //enable or disable various features
        _benablecol = true;
        _benabledis = false;
        _benabletol = false;
    }

    virtual bool InitEnvironment()
    {
        RAVELOG_DEBUG("creating pqp collision\n");
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            if( !_InitKinBody(*itbody) ) {
                RAVELOG_WARN("failed to init kinbody\n");
            }
        }
        return true;
    }

    virtual void DestroyEnvironment()
    {
        RAVELOG_DEBUG("destroying pqp collision\n");
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            (*itbody)->RemoveUserData("pqpcollision");
        }
    }

    virtual bool InitKinBody(KinBodyPtr pbody)
    {
        return _InitKinBody(pbody);
    }

    virtual bool _InitKinBody(KinBodyConstPtr pbody)
    {
        KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<KinBodyInfo>(pbody->GetUserData("pqpcollision"));
        // need the pbody check since kinbodies can be cloned and could have the wrong pointer
        if( !!pinfo && pinfo->GetBody() == pbody ) {
            return true;
        }

        pinfo.reset(new KinBodyInfo());

        pinfo->_pbody = boost::const_pointer_cast<KinBody>(pbody);
        pbody->SetUserData("pqpcollision", pinfo);

        PQP_REAL p1[3], p2[3], p3[3];
        pinfo->vlinks.reserve(pbody->GetLinks().size());
        FOREACHC(itlink, pbody->GetLinks()) {
            const TriMesh& trimesh = (*itlink)->GetCollisionData();
            boost::shared_ptr<PQP_Model> pm;
            if( trimesh.indices.size() > 0 ) {
                pm.reset(new PQP_Model());
                pm->BeginModel(trimesh.indices.size()/3);
                for(int j = 0; j < (int)trimesh.indices.size(); j+=3) {
                    p1[0] = trimesh.vertices[trimesh.indices[j]].x;     p1[1] = trimesh.vertices[trimesh.indices[j]].y;     p1[2] = trimesh.vertices[trimesh.indices[j]].z;
                    p2[0] = trimesh.vertices[trimesh.indices[j+1]].x;   p2[1] = trimesh.vertices[trimesh.indices[j+1]].y;     p2[2] = trimesh.vertices[trimesh.indices[j+1]].z;
                    p3[0] = trimesh.vertices[trimesh.indices[j+2]].x;   p3[1] = trimesh.vertices[trimesh.indices[j+2]].y;     p3[2] = trimesh.vertices[trimesh.indices[j+2]].z;
                    pm->AddTri(p1, p2, p3, j/3);
                }
                pm->EndModel();
            }
            pinfo->vlinks.push_back(pm);
        }

        return true;
    }

    void Synchronize()
    {
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            _InitKinBody(*itbody);
        }
    }

    virtual void RemoveKinBody(KinBodyPtr pbody)
    {
        if( !!pbody ) {
            pbody->RemoveUserData("pqpcollision");
        }
    }

    void GetPQPTransformFromTransform(Transform T, PQP_REAL PQP_R[3][3], PQP_REAL PQP_T[3])
    {
        TransformMatrix Tfm1(T);
        PQP_R[0][0] = Tfm1.m[0];   PQP_R[0][1] = Tfm1.m[1];   PQP_R[0][2] = Tfm1.m[2];
        PQP_R[1][0] = Tfm1.m[4];   PQP_R[1][1] = Tfm1.m[5];   PQP_R[1][2] = Tfm1.m[6];
        PQP_R[2][0] = Tfm1.m[8];   PQP_R[2][1] = Tfm1.m[9];   PQP_R[2][2] = Tfm1.m[10];
        PQP_T[0] = Tfm1.trans.x;   PQP_T[1] = Tfm1.trans.y;   PQP_T[2] = Tfm1.trans.z;
    }

    virtual bool SetCollisionOptions(int options)
    {
        if(options & CO_Distance) {
            RAVELOG_VERBOSE("setting pqp distance computation\n");
            _benabledis = true;
        }
        else {
            _benabledis = false;
        }
        _benablecol = true;
        if(options & CO_UseTolerance) {
            _benabletol = true;
        }
        else {
            _benabletol = false;
        }
        _options = options;
        return true;
    }
    virtual int GetCollisionOptions() const {
        return _options;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report)
    {
        if(!!report) {
            report->Reset(_options);
        }
        _SetActiveBody(pbody1);
        std::vector<KinBodyConstPtr> vexcluded;
        vexcluded.push_back(pbody1);
        return CheckCollision(pbody1,vexcluded,std::vector<KinBody::LinkConstPtr>(),report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report)
    {
        if(!!report) {
            report->Reset(_options);
        }
        _SetActiveBody(pbody1);
        std::set<KinBodyPtr> s1, s2;
        pbody1->GetAttached(s1);
        pbody2->GetAttached(s2);
        FOREACH(it1,s1) {
            FOREACH(it2,s2) {
                if( CheckCollisionP(KinBodyConstPtr(*it1),KinBodyConstPtr(*it2),report) ) {
                    return true;
                }
            }
        }

        return false;
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        _pactiverobot.reset();
        return CheckCollision(plink, vector<KinBodyConstPtr>(), vector<KinBody::LinkConstPtr>(),report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report)
    {
        if(!!report) {
            report->Reset(_options);
        }
        _InitKinBody(plink1->GetParent());
        _InitKinBody(plink2->GetParent());
        _pactiverobot.reset();
        PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];
        GetPQPTransformFromTransform(plink1->GetTransform(),R1,T1);
        GetPQPTransformFromTransform(plink2->GetTransform(),R2,T2);
        return DoPQP(plink1,R1,T1,plink2,R2,T2,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if(!!report ) {
            report->Reset(_options);
        }
        if( pbody->IsAttached(plink->GetParent()) ) {
            return false;
        }
        _pactiverobot.reset();
        std::set<KinBodyPtr> setattached;
        pbody->GetAttached(setattached);
        FOREACH(itbody,setattached) {
            if( CheckCollisionP(plink,*itbody,report) ) {
                return true;
            }
        }
        return false;
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        if(!!report) {
            report->Reset(_options);
        }
        _pactiverobot.reset();
        int tmpnumcols = 0;
        int tmpnumwithintol = 0;
        bool retval;

        _InitKinBody(plink->GetParent());

        std::vector<KinBodyPtr> vecbodies;
        GetEnv()->GetBodies(vecbodies);

        PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];

        std::vector<Transform> vtrans1,vtrans2;
        plink->GetParent()->GetLinkTransformations(vtrans1);
        FOREACH(itbody,vecbodies) {
            if(!!report) {
                report->numWithinTol = 0;
                report->numCols = 0;
            }
            KinBodyPtr pbody2 = *itbody;

            if(plink->GetParent()->IsAttached(KinBodyConstPtr(pbody2)) ) {
                continue;
            }
            if( find(vbodyexcluded.begin(),vbodyexcluded.end(),pbody2) != vbodyexcluded.end() ) {
                continue;
            }
            _InitKinBody(pbody2);
            std::vector<KinBody::LinkPtr> veclinks2 = pbody2->GetLinks();
            pbody2->GetLinkTransformations(vtrans2);
            GetPQPTransformFromTransform(vtrans1[plink->GetIndex()],R1,T1);

            for(int j = 0; j < (int)vtrans2.size(); j++) {
                if(plink == veclinks2[j]) {
                    continue;
                }
                if( find(vlinkexcluded.begin(),vlinkexcluded.end(),veclinks2[j]) != vlinkexcluded.end() ) {
                    continue;
                }
                GetPQPTransformFromTransform(vtrans2[j],R2,T2);

                retval = DoPQP(plink,R1,T1,veclinks2[j],R2,T2,report);
                if(!report && _benablecol && !_benabledis && !_benabletol && retval) {
                    return true;
                }
                //return tolerance check result when it is the only thing enabled and there is no report
                if(!report && !_benablecol && !_benabledis && _benabletol && retval) {
                    return true;
                }
            }

            if(!!report) {
                if(report->numWithinTol > 0) {
                    tmpnumwithintol++;
                }
                if(report->numCols > 0) {
                    tmpnumcols++;
                }
            }
        }

        if(!!report) {
            report->numWithinTol = tmpnumwithintol;
            report->numCols = tmpnumcols;
        }

        return tmpnumcols>0;

    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        if(!!report ) {
            report->Reset(_options);
        }
        _SetActiveBody(pbody);
        std::set<KinBodyPtr> vattached;
        pbody->GetAttached(vattached);
        FOREACHC(itbody, vattached) {
            if( CheckCollisionP(*itbody, vbodyexcluded, vlinkexcluded, report) ) {
                return true;
            }
        }

        return false;
    }

    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr())
    {
        if(!!report ) {
            report->Reset(_options);
        }
        _pactiverobot.reset();
        throw openrave_exception("PQP collision checker does not support ray collision queries\n");
    }
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())
    {
        if(!!report ) {
            report->Reset(_options);
        }
        _SetActiveBody(pbody);
        throw openrave_exception("PQP collision checker does not support ray collision queries\n");
    }
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr())
    {
        if(!!report ) {
            report->Reset(_options);
        }
        _pactiverobot.reset();
        throw openrave_exception("PQP collision checker does not support ray collision queries\n");
    }

    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if( pbody->GetLinks().size() <= 1 ) {
            return false;
        }
        if(!!report ) {
            report->Reset(_options);
        }
        _InitKinBody(pbody);
        int adjacentoptions = KinBody::AO_Enabled;
        if( (_options&OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
            adjacentoptions |= KinBody::AO_ActiveDOFs;
        }
        const std::set<int>& nonadjacent = pbody->GetNonAdjacentLinks(adjacentoptions);
        FOREACHC(itset, nonadjacent) {
            if( CheckCollision(KinBody::LinkConstPtr(pbody->GetLinks().at(*itset&0xffff)), KinBody::LinkConstPtr(pbody->GetLinks().at(*itset>>16)), report) ) {
                RAVELOG_VERBOSE(str(boost::format("selfcol %s, Links %s %s are colliding\n")%pbody->GetName()%pbody->GetLinks().at(*itset&0xffff)->GetName()%pbody->GetLinks().at(*itset>>16)->GetName()));
                return true;
            }
        }

        return false;
    }

    virtual bool CheckSelfCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        KinBodyPtr pbody = plink->GetParent();
        if( pbody->GetLinks().size() <= 1 ) {
            return false;
        }
        if(!!report ) {
            report->Reset(_options);
        }
        _InitKinBody(pbody);
        int adjacentoptions = KinBody::AO_Enabled;
        if( (_options&OpenRAVE::CO_ActiveDOFs) && pbody->IsRobot() ) {
            adjacentoptions |= KinBody::AO_ActiveDOFs;
        }
        const std::set<int>& nonadjacent = pbody->GetNonAdjacentLinks(adjacentoptions);
        FOREACHC(itset, nonadjacent) {
            KinBody::LinkConstPtr plink1(pbody->GetLinks().at(*itset&0xffff)), plink2(pbody->GetLinks().at(*itset>>16));
            if( plink == plink1 || plink == plink2 ) {
                if( CheckCollision(plink1, plink2, report) ) {
                    RAVELOG_VERBOSE(str(boost::format("selfcol %s, Links %s %s are colliding\n")%pbody->GetName()%pbody->GetLinks().at(*itset&0xffff)->GetName()%pbody->GetLinks().at(*itset>>16)->GetName()));
                    return true;
                }
            }
        }

        return false;
    }

    boost::shared_ptr<PQP_Model> GetLinkModel(KinBody::LinkConstPtr plink)
    {
        KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<KinBodyInfo>(plink->GetParent()->GetUserData("pqpcollision"));
        BOOST_ASSERT( pinfo->GetBody() == plink->GetParent());
        return pinfo->vlinks.at(plink->GetIndex());
    }

    void SetTolerance(dReal tol){
        _benabletol = true; _tolerance = tol;
    }

private:
    // does not check attached
    bool CheckCollisionP(KinBodyConstPtr pbody1, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        int tmpnumcols = 0;
        int tmpnumwithintol = 0;
        bool retval;

        std::vector<KinBodyPtr> vecbodies;
        GetEnv()->GetBodies(vecbodies);

        PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];

        std::vector<Transform> vtrans1,vtrans2;
        pbody1->GetLinkTransformations(vtrans1);
        _InitKinBody(pbody1);

        std::vector<KinBody::LinkPtr> veclinks1 = pbody1->GetLinks();
        FOREACH(itbody,vecbodies) {
            if(!!report) {
                report->numWithinTol = 0;
                report->numCols = 0;
            }
            KinBodyPtr pbody2 = *itbody;

            if(pbody1->IsAttached(KinBodyConstPtr(pbody2)) ) {
                continue;
            }
            if( find(vbodyexcluded.begin(),vbodyexcluded.end(),pbody2) != vbodyexcluded.end() ) {
                continue;
            }

            _InitKinBody(pbody2);
            std::vector<KinBody::LinkPtr> veclinks2 = pbody2->GetLinks();
            pbody2->GetLinkTransformations(vtrans2);
            for(int i = 0; i < (int)vtrans1.size(); i++) {
                if(find(vlinkexcluded.begin(),vlinkexcluded.end(),veclinks1[i]) != vlinkexcluded.end()) {
                    continue;
                }
                GetPQPTransformFromTransform(vtrans1[i],R1,T1);

                for(int j = 0; j < (int)vtrans2.size(); j++) {
                    if(find(vlinkexcluded.begin(),vlinkexcluded.end(),veclinks2[j]) != vlinkexcluded.end()) {
                        continue;
                    }
                    GetPQPTransformFromTransform(vtrans2[j],R2,T2);
                    retval = DoPQP(veclinks1[i],R1,T1,veclinks2[j],R2,T2,report);
                    if(!report && _benablecol && !_benabledis && !_benabletol && retval) {
                        return true;
                    }
                    //return tolerance check result when it is the only thing enabled and there is no report
                    if(!report && !_benablecol && !_benabledis && _benabletol && retval) {
                        return true;
                    }
                }
            }
            if(!!report) {
                if(report->numWithinTol > 0) {
                    tmpnumwithintol++;
                }
                if(report->numCols > 0) {
                    tmpnumcols++;
                }
            }
        }
        if(!!report) {
            report->numWithinTol = tmpnumwithintol;
            report->numCols = tmpnumcols;
        }
        return tmpnumcols>0;
    }

    // does not check attached
    bool CheckCollisionP(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report)
    {
        _InitKinBody(pbody1);
        _InitKinBody(pbody2);
        PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];
        FOREACHC(itlink1,pbody1->GetLinks()) {
            GetPQPTransformFromTransform((*itlink1)->GetTransform(),R1,T1);
            FOREACHC(itlink2,pbody2->GetLinks()) {
                GetPQPTransformFromTransform((*itlink2)->GetTransform(),R2,T2);
                bool retval = DoPQP(*itlink1,R1,T1,*itlink2,R2,T2,report);
                if(!report && _benablecol && !_benabledis && !_benabletol && retval) {
                    return true;
                }
                //return tolerance check result when it is the only thing enabled and there is no report
                if(!report && !_benablecol && !_benabledis && _benabletol && retval) {
                    return true;
                }
            }
        }

        return false;
    }

    // does not check attached
    bool CheckCollisionP(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        _InitKinBody(plink->GetParent());
        _InitKinBody(pbody);
        bool success = false;
        PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];
        GetPQPTransformFromTransform(plink->GetTransform(),R1,T1);
        FOREACHC(itlink,pbody->GetLinks()) {
            GetPQPTransformFromTransform((*itlink)->GetTransform(),R2,T2);
            bool retval = DoPQP(plink,R1,T1,*itlink,R2,T2,report);
            success |= retval;
            if(!report && _benablecol && !_benabledis && !_benabletol && retval) {
                return true;
            }
            //return tolerance check result when it is the only thing enabled and there is no report
            if(!report && !_benablecol && !_benabledis && _benabletol && retval) {
                return true;
            }
        }

        return success;
    }

    Vector PQPRealToVector(const Vector& in, const PQP_REAL R[3][3], const PQP_REAL T[3])
    {
        return Vector(in.x*R[0][0]+in.y*R[0][1]+in.z*R[0][2]+T[0], in.x*R[1][0]+in.y*R[1][1]+in.z*R[1][2]+T[1], in.x*R[2][0]+in.y*R[2][1]+in.z*R[2][2]+T[2]);
    }

    bool DoPQP(KinBody::LinkConstPtr link1, PQP_REAL R1[3][3], PQP_REAL T1[3], KinBody::LinkConstPtr link2, PQP_REAL R2[3][3], PQP_REAL T2[3], CollisionReportPtr report)
    {
        if( !link1->IsEnabled() || !link2->IsEnabled() ) {
            return false;
        }
        if( !_IsActiveLink(link1->GetParent(),link1->GetIndex()) || !_IsActiveLink(link2->GetParent(),link2->GetIndex()) ) {
            return false;
        }
        boost::shared_ptr<PQP_Model> m1 = GetLinkModel(link1);
        boost::shared_ptr<PQP_Model> m2 = GetLinkModel(link2);
        bool bcollision = false;
        if( !m1 || !m2 ) {
            return false;
        }
        // collision
        if(_benablecol) {
            if( GetEnv()->HasRegisteredCollisionCallbacks() && !report ) {
                report.reset(new CollisionReport());
                report->Reset(_options);
            }

            if(!report) {
                PQP_CollideResult _colres;
                PQP_Collide(&_colres,R1,T1,m1.get(),R2,T2,m2.get());
                if(_colres.NumPairs() > 0)
                    bcollision = true;
            }
            else {
                PQP_Collide(&colres,R1,T1,m1.get(),R2,T2,m2.get());
                report->numCols += colres.Colliding();

                if(colres.NumPairs() > 0) {
                    report->plink1 = link1;
                    report->plink2 = link2;
                    bcollision = true;
                }

                for(int i = 0; i < colres.NumPairs(); i++) {
                    u1 = PQPRealToVector(link1->GetCollisionData().vertices[link1->GetCollisionData().indices[colres.Id1(i)*3]],R1,T1);
                    u2 = PQPRealToVector(link1->GetCollisionData().vertices[link1->GetCollisionData().indices[colres.Id1(i)*3+1]],R1,T1);
                    u3 = PQPRealToVector(link1->GetCollisionData().vertices[link1->GetCollisionData().indices[colres.Id1(i)*3+2]],R1,T1);

                    v1=PQPRealToVector(link2->GetCollisionData().vertices[link2->GetCollisionData().indices[colres.Id2(i)*3]],R2,T2);
                    v2=PQPRealToVector(link2->GetCollisionData().vertices[link2->GetCollisionData().indices[colres.Id2(i)*3+1]],R2,T2);
                    v3=PQPRealToVector(link2->GetCollisionData().vertices[link2->GetCollisionData().indices[colres.Id2(i)*3+2]],R2,T2);

                    if(TriTriCollision(u1,u2,u3,v1,v2,v3,contactpos,contactnorm)) {
                        report->contacts.push_back(CollisionReport::CONTACT(contactpos,contactnorm,0.));
                    }
                }

                if( GetEnv()->HasRegisteredCollisionCallbacks() ) {
                    std::list<EnvironmentBase::CollisionCallbackFn> listcallbacks;
                    GetEnv()->GetRegisteredCollisionCallbacks(listcallbacks);

                    FOREACHC(itfn, listcallbacks) {
                        OpenRAVE::CollisionAction action = (*itfn)(report,false);
                        if( action != OpenRAVE::CA_DefaultAction ) {
                            report->Reset(_options);
                            return false;
                        }
                    }
                }
            }
        }

        // distance
        if(_benabledis) {
            if(!report) {
                throw openrave_exception("CollisionCheckerPQP::DoPQP - ERROR: YOU MUST PASS IN A CollisionReport STRUCT TO MEASURE DISTANCE!\n");
            }
            //don't do a tolerance check, some users wants distance all the time
            //PQP_Tolerance(&tolres,R1,T1,m1,R2,T2,m2,_preport->minDistance);
            //if(tolres.CloserThanTolerance()) {

            PQP_Distance(&disres,R1,T1,m1.get(),R2,T2,m2.get(),_rel_err,_abs_err);
            if(report->minDistance > (dReal) disres.Distance()) {
                report->minDistance = (dReal) disres.Distance();
                report->contacts.resize(1);
                Vector p1(disres.P1()[0],disres.P1()[1],disres.P1()[2]), p2(disres.P2()[0],disres.P2()[1],disres.P2()[2]);
                p1 = PQPRealToVector(disres.P1(),R1,T1);
                p2 = PQPRealToVector(disres.P2(),R2,T2);
                dReal depth = RaveSqrt((p2-p1).lengthsqr3());
                report->contacts.at(0).pos = p1;
                report->contacts.at(0).depth = -depth;
                if( depth > 0 )
                    report->contacts.at(0).norm = (p2-p1)*(1/depth);
                else
                    report->contacts.at(0).norm = Vector(0,0,0);
                report->plink1 = link1;
                report->plink2 = link2;
            }
        }

        // tolerance
        if( _benabletol) {
            PQP_Tolerance(&tolres,R1,T1,m1.get(),R2,T2,m2.get(),_tolerance);
            if(!!report)
                report->numWithinTol +=tolres.CloserThanTolerance();
        }
        if(_benablecol)
            return bcollision;
        else if(_benabletol)
            return tolres.CloserThanTolerance()>0;
        else
            return false;
    }

    int _options;

    //pqp parameters
    PQP_REAL _tolerance;
    PQP_REAL _rel_err;
    PQP_REAL _abs_err;

    // collision
    bool _benablecol;
    PQP_CollideResult colres;

    // distance
    bool _benabledis;
    PQP_DistanceResult disres;

    // within tolerance
    bool _benabletol;
    PQP_ToleranceResult tolres;

    //for collision reporting
    Vector u1, u2, u3, v1, v2, v3;
    Vector contactpos, contactnorm;
    PQP_REAL tri1[3][3], tri2[3][3];
    TransformMatrix tmtemp;

    RobotBaseConstPtr _pactiverobot;     ///< set if ActiveDOFs option is enabled
    vector<uint8_t> _vactivelinks;

    void _SetActiveBody(KinBodyConstPtr pbody) {
        if( _options & CO_ActiveDOFs ) {
            _pactiverobot = OpenRAVE::RaveInterfaceConstCast<RobotBase>(pbody);
            _vactivelinks.resize(0);
        }
        else {
            _pactiverobot.reset();
        }
        // init just in case
        _InitKinBody(pbody);
    }

    bool _IsActiveLink(KinBodyConstPtr pbody, int linkindex)
    {
        if( !(_options & CO_ActiveDOFs) || !_pactiverobot ||( pbody != _pactiverobot) ) {
            return true;
        }
        if( _vactivelinks.size() == 0 ) {
            if( _pactiverobot->GetAffineDOF() ) {
                // enable everything
                _vactivelinks.resize(_pactiverobot->GetLinks().size(),1);
            }
            else {
                _vactivelinks.resize(_pactiverobot->GetLinks().size(),0);
                // only check links that can potentially move with respect to each other
                _vactivelinks.resize(_pactiverobot->GetLinks().size(),0);
                for(size_t i = 0; i < _pactiverobot->GetLinks().size(); ++i) {
                    FOREACHC(itindex, _pactiverobot->GetActiveDOFIndices()) {
                        if( _pactiverobot->DoesAffect(_pactiverobot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),i) ) {
                            _vactivelinks[i] = 1;
                            break;
                        }
                    }
                }
            }
        }
        return _vactivelinks.at(linkindex)>0;
    }
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(CollisionCheckerPQP)
BOOST_TYPEOF_REGISTER_TYPE(CollisionCheckerPQP::KinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(PQP_Model)
#endif

#endif   // COLPQP_H
