// Copyright (C) 2006-2009 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
    struct COL
    {
        COL(Tri tri1_in, Tri tri2_in){ tri1 = tri1_in; tri2 = tri2_in;};
        Tri tri1;
        Tri tri2;
        KinBodyPtr pbody1;
        KinBodyPtr pbody2;
    };

    class KinBodyInfo
    {
    public:
    KinBodyInfo() : nLastStamp(0) {}
        virtual ~KinBodyInfo() {}
        KinBodyPtr pbody;
        vector<boost::shared_ptr<PQP_Model> > vlinks;
        int nLastStamp;
    };
    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;
    
 CollisionCheckerPQP(EnvironmentBasePtr penv) : CollisionCheckerBase(penv)
    {
        _rel_err = 200.0; //temporary change
        _abs_err = 0.2;   //temporary change
        _tolerance = 0.0;
    
        //enable or disable various features
        _benablecol = true;
        _benabledis = false;
        _benabletol = false;
    }

    virtual bool InitEnvironment()
    {
        RAVELOG_DEBUGA("creating pqp collision\n");
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            if( !InitKinBody(*itbody) )
                RAVELOG_WARNA("failed to init kinbody\n");
        }
        return true;
    }

    virtual void DestroyEnvironment()
    {
        RAVELOG_DEBUGA("destroying pqp collision\n");
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies)
            SetCollisionData(*itbody, boost::shared_ptr<void>());
    }

    virtual bool InitKinBody(KinBodyPtr pbody)
    {
        KinBodyInfoPtr pinfo(new KinBodyInfo());

        pinfo->pbody = pbody;
        SetCollisionData(pbody, pinfo);

        PQP_REAL p1[3], p2[3], p3[3];
        pinfo->vlinks.reserve(pbody->GetLinks().size());
        FOREACHC(itlink, pbody->GetLinks()) {
            boost::shared_ptr<PQP_Model> pm(new PQP_Model());
        
            pm->BeginModel();
            const KinBody::Link::TRIMESH& trimesh = (*itlink)->GetCollisionData();
    
            for(int j = 0; j < (int)trimesh.indices.size(); j+=3) { 
                p1[0] = trimesh.vertices[trimesh.indices[j]].x;     p1[1] = trimesh.vertices[trimesh.indices[j]].y;     p1[2] = trimesh.vertices[trimesh.indices[j]].z;
                p2[0] = trimesh.vertices[trimesh.indices[j+1]].x;   p2[1] = trimesh.vertices[trimesh.indices[j+1]].y;     p2[2] = trimesh.vertices[trimesh.indices[j+1]].z;
                p3[0] = trimesh.vertices[trimesh.indices[j+2]].x;   p3[1] = trimesh.vertices[trimesh.indices[j+2]].y;     p3[2] = trimesh.vertices[trimesh.indices[j+2]].z;
                pm->AddTri(p1, p2, p3, j/3);
            }
            pm->EndModel();
            pinfo->vlinks.push_back(pm);
        }

        return true;
    }

    virtual bool Enable(KinBodyConstPtr pbody, bool bEnable)
    {
        return true;
    }

    virtual bool EnableLink(KinBody::LinkConstPtr plink, bool bEnable)
    {
        return true;
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
            RAVELOG_DEBUG("setting pqp distance computation\n");
            _benabledis = true;
        }
        else
            _benabledis = false;

        _benablecol = true;

        if(options & CO_UseTolerance) 
            _benabletol = true;
        else
            _benabletol = false;
    
        _options = options;

        return true;
    }
    virtual int GetCollisionOptions() const { return _options; }

    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) { return false; }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report)
    {
        std::vector<KinBodyConstPtr> vexcluded;
        vexcluded.push_back(pbody1);
        return CheckCollision(pbody1,vexcluded,std::vector<KinBody::LinkConstPtr>(),report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report)
    {
        std::vector<KinBodyConstPtr> vexcluded;
        std::vector<KinBodyPtr> vecbodies;
        GetEnv()->GetBodies(vecbodies);

        FOREACH(itbody,vecbodies) {
            if( *itbody != pbody2 )
                vexcluded.push_back(*itbody);
        }
        return CheckCollision(pbody1,vexcluded,std::vector<KinBody::LinkConstPtr>(),report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        return CheckCollision(plink, vector<KinBodyConstPtr>(), vector<KinBody::LinkConstPtr>(),report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report)
    {
        if(!!report)
            report->Reset();
    
        PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];
        GetPQPTransformFromTransform(plink1->GetTransform(),R1,T1);
        GetPQPTransformFromTransform(plink2->GetTransform(),R2,T2);
        return DoPQP(plink1,R1,T1,plink2,R2,T2,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        //does not check for self collision
        std::vector<KinBodyConstPtr> vbodyexcluded;
        std::vector<KinBodyPtr> vecbodies;
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;

        GetEnv()->GetBodies(vecbodies);
        FOREACH(itbody,vecbodies) {
            if(pbody != *itbody)
                vbodyexcluded.push_back(*itbody);
        }
    
        return CheckCollision(plink, vbodyexcluded, vlinkexcluded,report);
    }
    
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        if(!!report)
            report->Reset();
    
        int tmpnumcols = 0;
        int tmpnumwithintol = 0;
        bool retval;

        std::vector<KinBodyPtr> vecbodies;
        GetEnv()->GetBodies(vecbodies);

        PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];

        std::vector<Transform> vtrans1,vtrans2;
        plink->GetParent()->GetBodyTransformations(vtrans1);

        bool exclude_link2;
        FOREACH(itbody,vecbodies) {
            if(!!report) {
                report->numWithinTol = 0;
                report->numCols = 0;
            }
            KinBodyPtr pbody2 = *itbody;
        
            if(plink->GetParent() == pbody2 || plink->GetParent()->IsAttached(pbody2) )
                continue;

            if( find(vbodyexcluded.begin(),vbodyexcluded.end(),pbody2) != vbodyexcluded.end() )
                continue;

            std::vector<KinBody::LinkPtr> veclinks2 = pbody2->GetLinks();
            pbody2->GetBodyTransformations(vtrans2);
            GetPQPTransformFromTransform(vtrans1[plink->GetIndex()],R1,T1);

            exclude_link2 = false;
            for(int j = 0; j < (int)vtrans2.size(); j++) {

                if(plink == veclinks2[j])
                    continue;

                if( find(vlinkexcluded.begin(),vlinkexcluded.end(),veclinks2[j]) != vlinkexcluded.end() )
                    continue;

                GetPQPTransformFromTransform(vtrans2[j],R2,T2);

                retval = DoPQP(plink,R1,T1,veclinks2[j],R2,T2,report);
                if(!report && _benablecol && !_benabledis && !_benabletol && retval)
                    return true;
                //return tolerance check result when it is the only thing enabled and there is no report
                if(!report && !_benablecol && !_benabledis && _benabletol && retval)
                    return true;
            }
            
            if(!!report) {
                if(report->numWithinTol > 0)
                    tmpnumwithintol++;
             
                if(report->numCols > 0)
                    tmpnumcols++;
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
        if( CheckCollisionP(pbody, vbodyexcluded, vlinkexcluded, report) )
            return true;

        // check attached objects
        std::vector<KinBodyPtr> vattached;
        pbody->GetAttached(vattached);
        FOREACHC(itbody, vattached) {
            if( CheckCollisionP(*itbody, vbodyexcluded, vlinkexcluded, report) )
                return true;
        }

        return false;
    }
    
    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr())
    {
        throw openrave_exception("PQP collision checker does not support ray collision queries\n");
    }
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr())
    {
        throw openrave_exception("PQP collision checker does not support ray collision queries\n");
    }
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr())
    {
        throw openrave_exception("PQP collision checker does not support ray collision queries\n");
    }
    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        if( pbody->GetLinks().size() <= 1 )
            return false;

        // check collision, ignore adjacent bodies
        FOREACHC(itset, pbody->GetNonAdjacentLinks()) {
            BOOST_ASSERT( (*itset&0xffff) < (int)pbody->GetLinks().size() && (*itset>>16) < (int)pbody->GetLinks().size() );
            if( CheckCollision(KinBody::LinkConstPtr(pbody->GetLinks()[*itset&0xffff]), KinBody::LinkConstPtr(pbody->GetLinks()[*itset>>16]), report) ) {
                RAVELOG_VERBOSEA(str(boost::format("selfcol %s, Links %s %s are colliding\n")%pbody->GetName()%pbody->GetLinks()[*itset&0xffff]->GetName()%pbody->GetLinks()[*itset>>16]->GetName()));
                return true;
            }
        }

        return false;
    }

    boost::shared_ptr<PQP_Model> GetLinkModel(KinBody::LinkConstPtr plink)
    {
        KinBodyInfoPtr pinfo = boost::static_pointer_cast<KinBodyInfo>(plink->GetParent()->GetCollisionData());
        BOOST_ASSERT( pinfo->pbody == plink->GetParent());
        return pinfo->vlinks.at(plink->GetIndex());
    }


    void SetTolerance(dReal tol){_benabletol = true; _tolerance = tol;}

 private:
    // does not check attached
    bool CheckCollisionP(KinBodyConstPtr pbody1, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        if(!!report )
            report->Reset();

        int tmpnumcols = 0;
        int tmpnumwithintol = 0;
        bool retval;

        std::vector<KinBodyPtr> vecbodies;
        GetEnv()->GetBodies(vecbodies);

        PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];

        std::vector<Transform> vtrans1,vtrans2;
        pbody1->GetBodyTransformations(vtrans1);
    
        std::vector<KinBody::LinkPtr> veclinks1 = pbody1->GetLinks();

        bool exclude_link1, exclude_link2;
        FOREACH(itbody,vecbodies) {
            if(!!report) {
                report->numWithinTol = 0;
                report->numCols = 0;
            }
            KinBodyPtr pbody2 = *itbody;
        
            if(pbody1 == pbody2 || pbody1->IsAttached(pbody2) )
                continue;

            if( find(vbodyexcluded.begin(),vbodyexcluded.end(),pbody2) != vbodyexcluded.end() )
                continue;


            std::vector<KinBody::LinkPtr> veclinks2 = pbody2->GetLinks();
            pbody2->GetBodyTransformations(vtrans2);
        
            exclude_link1 = false;
            for(int i = 0; i < (int)vtrans1.size(); i++) {
                if(find(vlinkexcluded.begin(),vlinkexcluded.end(),veclinks1[i]) != vlinkexcluded.end())
                    continue;
            
                GetPQPTransformFromTransform(vtrans1[i],R1,T1);

                exclude_link2 = false;
                for(int j = 0; j < (int)vtrans2.size(); j++) {
                    if(find(vlinkexcluded.begin(),vlinkexcluded.end(),veclinks2[j]) != vlinkexcluded.end())
                        continue;

                    GetPQPTransformFromTransform(vtrans2[j],R2,T2);
                    retval = DoPQP(veclinks1[i],R1,T1,veclinks2[j],R2,T2,report);
                    if(!report && _benablecol && !_benabledis && !_benabletol && retval)
                        return true;
                    //return tolerance check result when it is the only thing enabled and there is no report
                    if(!report && !_benablecol && !_benabledis && _benabletol && retval)
                        return true;
                }
            }
            if(!!report) {
                if(report->numWithinTol > 0)
                    tmpnumwithintol++;
             
                if(report->numCols > 0)
                    tmpnumcols++;
            }
        }
        if(!!report) {
            report->numWithinTol = tmpnumwithintol;
            report->numCols = tmpnumcols;
        }
        return tmpnumcols>0;
    }

    void PQPRealToVector(const Vector& in, const PQP_REAL R[3][3], const PQP_REAL T[3], Vector& out)
    {
    
        tmtemp.m[0] = (dReal)R[0][0]; tmtemp.m[1] = (dReal)R[0][1]; tmtemp.m[2] = (dReal)R[0][2];
        tmtemp.m[4] = (dReal)R[1][0]; tmtemp.m[5] = (dReal)R[1][1]; tmtemp.m[6] = (dReal)R[1][2];
        tmtemp.m[8] = (dReal)R[2][0]; tmtemp.m[9] = (dReal)R[2][1]; tmtemp.m[10] = (dReal)R[2][2];
    
        out.x = in.x;
        out.y = in.y;
        out.z = in.z;
    
        out = tmtemp*out;
        out.x = out.x + (dReal)T[0];
        out.y = out.y + (dReal)T[1];
        out.z = out.z + (dReal)T[2];
    }

    bool DoPQP(KinBody::LinkConstPtr link1, PQP_REAL R1[3][3], PQP_REAL T1[3], KinBody::LinkConstPtr link2, PQP_REAL R2[3][3], PQP_REAL T2[3], CollisionReportPtr report)
    {
        if( !link1->IsEnabled() || !link2->IsEnabled() )
            return false;

        boost::shared_ptr<PQP_Model> m1 = GetLinkModel(link1);
        boost::shared_ptr<PQP_Model> m2 = GetLinkModel(link2);
        bool bcollision = false;
    
        // collision
        if(_benablecol) {
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
                
                    PQPRealToVector(link1->GetCollisionData().vertices[link1->GetCollisionData().indices[colres.Id1(i)*3]],R1,T1,u1);
                    PQPRealToVector(link1->GetCollisionData().vertices[link1->GetCollisionData().indices[colres.Id1(i)*3+1]],R1,T1,u2);
                    PQPRealToVector(link1->GetCollisionData().vertices[link1->GetCollisionData().indices[colres.Id1(i)*3+2]],R1,T1,u3);
                
                    PQPRealToVector(link2->GetCollisionData().vertices[link2->GetCollisionData().indices[colres.Id2(i)*3]],R2,T2,v1);
                    PQPRealToVector(link2->GetCollisionData().vertices[link2->GetCollisionData().indices[colres.Id2(i)*3+1]],R2,T2,v2);
                    PQPRealToVector(link2->GetCollisionData().vertices[link2->GetCollisionData().indices[colres.Id2(i)*3+2]],R2,T2,v3);
                
                    if(TriTriCollision(u1,u2,u3,v1,v2,v3,contactpos,contactnorm)) {
                        report->contacts.push_back(COLLISIONREPORT::CONTACT(contactpos,contactnorm,0.));
                    }            
                }
            }
        }
    
        // distance
        if(_benabledis) {
            if(!report) {
                RAVELOG_WARNA("CollisionCheckerPQP::DoPQP - ERROR: YOU MUST PASS IN A COLLISIONREPORT STRUCT TO MEASURE DISTANCE!\n");
                return false;
            }

            //don't do a tolerance check, some users wants distance all the time
            //PQP_Tolerance(&tolres,R1,T1,m1,R2,T2,m2,_preport->minDistance);
            //if(tolres.CloserThanTolerance()) {

            PQP_Distance(&disres,R1,T1,m1.get(),R2,T2,m2.get(),_rel_err,_abs_err);
            if(report->minDistance > (dReal) disres.Distance())
                report->minDistance = (dReal) disres.Distance();
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
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(CollisionCheckerPQP)
BOOST_TYPEOF_REGISTER_TYPE(CollisionCheckerPQP::COL)
BOOST_TYPEOF_REGISTER_TYPE(CollisionCheckerPQP::KinBodyInfo)
BOOST_TYPEOF_REGISTER_TYPE(PQP_Model)
#endif

#endif   // COLPQP_H
