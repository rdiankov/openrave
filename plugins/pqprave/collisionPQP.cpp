// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#include "plugindefs.h"

#include "collisionPQP.h"

CollisionCheckerPQP::KINBODYINFO::~KINBODYINFO()
{
    FOREACH(it, vlinks)
        delete *it;
}

CollisionCheckerPQP::CollisionCheckerPQP(EnvironmentBase* penv) : CollisionCheckerBase(penv)
{
    _rel_err = 200.0; //temporary change
    _abs_err = 0.2;   //temporary change
    _tolerance = 0.0;
    
    //enable or disable various features
    _benablecol = true;
    _benabledis = false;
    _benabletol = false;
}

CollisionCheckerPQP::~CollisionCheckerPQP()
{
}

bool CollisionCheckerPQP::InitEnvironment()
{
    RAVELOG_DEBUGA("creating pqp collision\n");
    FOREACHC(itbody, GetEnv()->GetBodies()) {
        if( !InitKinBody(*itbody) )
            RAVELOG(L"failed to init kinbody\n");
    }
    return true;
}

void CollisionCheckerPQP::DestroyEnvironment()
{
    RAVELOG_DEBUGA("destroying pqp collision\n");
    _preport = NULL;

    FOREACHC(itbody, GetEnv()->GetBodies()) {
        delete (KINBODYINFO*)(*itbody)->GetCollisionData();
        SetCollisionData(*itbody,NULL);
    }
}

bool CollisionCheckerPQP::InitKinBody(KinBody* pbody)
{
    KINBODYINFO* pinfo = new KINBODYINFO();

    pinfo->pbody = pbody;
    SetCollisionData(pbody, pinfo);

    PQP_REAL p1[3], p2[3], p3[3];
    pinfo->vlinks.reserve(pbody->GetLinks().size());
    FOREACHC(itlink, pbody->GetLinks()) {
        PQP_Model* pm = new PQP_Model();
        
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

bool CollisionCheckerPQP::DestroyKinBody(KinBody* pbody)
{
    delete (KINBODYINFO*)pbody->GetCollisionData();
    SetCollisionData(pbody,NULL);
    return true;
}

bool CollisionCheckerPQP::Enable(const KinBody* pbody, bool bEnable)
{
    return true;
}

bool CollisionCheckerPQP::EnableLink(const KinBody::Link* pbody, bool bEnable)
{
    return true;
}

PQP_Model* CollisionCheckerPQP::GetLinkModel(const KinBody::Link* plink)
{
    assert( plink != NULL && plink->GetParent() != NULL );
    KINBODYINFO* pinfo = (KINBODYINFO*)plink->GetParent()->GetCollisionData();
    assert( pinfo != NULL );
    assert( pinfo->pbody == plink->GetParent());
    assert( plink->GetIndex() < (int)pinfo->vlinks.size());
    return pinfo->vlinks[plink->GetIndex()];
}

bool CollisionCheckerPQP::DoPQP(const KinBody::Link* link1, PQP_REAL R1[3][3], PQP_REAL T1[3], const KinBody::Link* link2, PQP_REAL R2[3][3], PQP_REAL T2[3])
{
    if( !link1->IsEnabled() || !link2->IsEnabled() )
        return false;

    PQP_Model* m1 = GetLinkModel(link1);
    PQP_Model* m2 = GetLinkModel(link2);
    bool bcollision = false;
    
    // collision
    if(_benablecol) {
        
        if(_preport == NULL) {
            PQP_CollideResult _colres;
            PQP_Collide(&_colres,R1,T1,m1,R2,T2,m2);
            if(_colres.NumPairs() > 0)
                bcollision = true;                
        }
        else {
            PQP_Collide(&colres,R1,T1,m1,R2,T2,m2);
            _preport->numCols += colres.Colliding();

            if(colres.NumPairs() > 0) {
                _preport->plink1 = (KinBody::Link*)link1;
                _preport->plink2 = (KinBody::Link*)link2;
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
                    _preport->contacts.push_back(COLLISIONREPORT::CONTACT(contactpos,contactnorm,0.));
                }            
            }
        }
    }
    
    // distance
    if(_benabledis) {
        if(_preport == NULL) {
            RAVEPRINT(L"CollisionCheckerPQP::DoPQP - ERROR: YOU MUST PASS IN A COLLISIONREPORT STRUCT TO MEASURE DISTANCE!\n");
            return false;
        }

        //don't do a tolerance check, some users wants distance all the time
        //PQP_Tolerance(&tolres,R1,T1,m1,R2,T2,m2,_preport->minDistance);
        //if(tolres.CloserThanTolerance()) {

        PQP_Distance(&disres,R1,T1,m1,R2,T2,m2,_rel_err,_abs_err);
        if(_preport->minDistance > (dReal) disres.Distance())
            _preport->minDistance = (dReal) disres.Distance();
    }
   
    // tolerance
    if( _benabletol) {
        PQP_Tolerance(&tolres,R1,T1,m1,R2,T2,m2,_tolerance);
        if(_preport != NULL)
            _preport->numWithinTol +=tolres.CloserThanTolerance();
    }
	if(_benablecol)
		return bcollision;
	else if(_benabletol)
		return tolres.CloserThanTolerance()>0;
	else
		return false;
}

 //check collision between two KinBodys
bool CollisionCheckerPQP::CheckCollision(const KinBody* pbody1, const KinBody* pbody2, COLLISIONREPORT* preport)
{
    std::set<KinBody* > vexcluded;
    std::vector<KinBody *> vecbodies;
    GetEnv()->GetBodies(vecbodies);

    _preport = preport;

    for(int i = 0; i < (int)vecbodies.size(); i++)
        if(pbody2 != vecbodies[i])
            vexcluded.insert(vecbodies[i]);

    std::set<KinBody::Link* > vlinkexcluded;

    return CheckCollision(pbody1,vexcluded,vlinkexcluded,preport);
}

bool CollisionCheckerPQP::CheckCollision(const KinBody* pbody1, COLLISIONREPORT* preport)
{
    //_preport = preport;
    std::set<KinBody* > vexcluded;
    std::set<KinBody::Link* > vlinkexcluded;
    vexcluded.insert((KinBody*)pbody1);
    return CheckCollision(pbody1,vexcluded,vlinkexcluded,preport);
}

bool CollisionCheckerPQP::CheckCollision(const KinBody* pbody1, const std::set<KinBody* >& vexcluded,COLLISIONREPORT* preport)
{
    std::set<KinBody::Link* > dummy;
    return CheckCollision(pbody1,vexcluded,dummy,preport);
}


bool CollisionCheckerPQP::CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, dReal tolerance)
{
	int prevopts= GetCollisionOptions();
	dReal oldtol = _tolerance;
	_benabletol = true;
	_benablecol = false;
	_benabledis = false;
	_tolerance = tolerance;
	COLLISIONREPORT * preport = NULL;
	bool retval = CheckCollision(pbody,vbodyexcluded,vlinkexcluded,preport);
	SetCollisionOptions(prevopts);
	_tolerance = oldtol;
	return retval;
}

bool CollisionCheckerPQP::CheckCollision(const KinBody* pbody, const std::set<KinBody* >& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* preport)
{
    if( CheckCollisionP(pbody, vbodyexcluded, vlinkexcluded, preport) )
        return true;

    // check attached objects
    FOREACHC(itbody, pbody->GetAttached()) {
        if( CheckCollisionP(*itbody, vbodyexcluded, vlinkexcluded, preport) )
            return true;
    }

    return false;
}

bool CollisionCheckerPQP::CheckCollisionP(const KinBody* pbody1, const std::set<KinBody* >& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* preport)
{
    _preport = preport;
    if(_preport != NULL) {
        _preport->Reset();
        _preport->minDistance = 1e20f;
    }

    int tmpnumcols = 0;
    int tmpnumwithintol = 0;
    bool retval;

    std::vector<KinBody *> vecbodies;
    GetEnv()->GetBodies(vecbodies);

    PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];

    std::vector<Transform> vtrans1,vtrans2;
    pbody1->GetBodyTransformations(vtrans1);

    
    std::vector<KinBody::Link *> veclinks1 = pbody1->GetLinks();

    KinBody * pbody2;
    bool exclude_link1, exclude_link2;
    for(int k = 0; k < (int)vecbodies.size(); k++) {
        if(_preport != NULL) {
            _preport->numWithinTol = 0;
            _preport->numCols = 0;
        }
        pbody2 = vecbodies[k];
        
        if(pbody1 == pbody2 || pbody1->IsAttached(pbody2) )
            continue;

        if( vbodyexcluded.find(pbody2) != vbodyexcluded.end() )
            continue;


        std::vector<KinBody::Link *> veclinks2 = pbody2->GetLinks();
        
        pbody2->GetBodyTransformations(vtrans2);
        
        exclude_link1 = false;
        for(int i = 0; i < (int)vtrans1.size(); i++) {
            if(vlinkexcluded.find(veclinks1[i]) != vlinkexcluded.end())
                continue;
            
            GetPQPTransformFromTransform(vtrans1[i],R1,T1);

            exclude_link2 = false;
            for(int j = 0; j < (int)vtrans2.size(); j++) {
                if(vlinkexcluded.find(veclinks2[j]) != vlinkexcluded.end())
                    continue;

                GetPQPTransformFromTransform(vtrans2[j],R2,T2);
                retval = DoPQP(veclinks1[i],R1,T1,veclinks2[j],R2,T2);
                if(_preport == NULL && _benablecol == true && _benabledis == false && _benabletol == false && retval)
                    return true;
                //return tolerance check result when it is the only thing enabled and there is no report
                if(_preport == NULL && _benablecol == false && _benabledis == false && _benabletol == true && retval)
                    return true;
            }
            
        }
        if(_preport != NULL) {
            if(_preport->numWithinTol > 0)
                tmpnumwithintol++;
             
            if(_preport->numCols > 0)
                tmpnumcols++;
        }
    }
    if(_preport != NULL) {
        _preport->numWithinTol = tmpnumwithintol;
        _preport->numCols = tmpnumcols;
    }
    return tmpnumcols>0;
}

bool CollisionCheckerPQP::CheckCollision(const KinBody::Link* plink, COLLISIONREPORT* preport)
{
    std::set<KinBody*> vbodyexcluded;
    std::set<KinBody::Link *> vlinkexcluded;
    return CheckCollision(plink, vbodyexcluded, vlinkexcluded,preport);
}

bool CollisionCheckerPQP::SetCollisionOptions(int options)
{    
    if(options & CO_Distance) 
        _benabledis = true;
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

bool CollisionCheckerPQP::CheckCollision(const KinBody::Link* plink1, const KinBody::Link* plink2, COLLISIONREPORT* preport)
{
    //does not check for self collision
    std::set<KinBody*> vbodyexcluded;
    std::vector<KinBody *> vecbodies;
    GetEnv()->GetBodies(vecbodies);

    //exclude all kinbodies that aren't parents of plink2
    //parents of plink1 are excluded in the other CheckCollision function
    for(int i = 0; i < (int)vecbodies.size(); i++)
        if(plink2->GetParent() != vecbodies[i])
            vbodyexcluded.insert(vecbodies[i]);

    //exclude all links of plink2's parent that aren't plink2
    std::vector<KinBody::Link* > veclinks;
    set<KinBody::Link*> vlinkexcluded;
    veclinks = plink2->GetParent()->GetLinks();
    for(int i = 0; i < (int)veclinks.size(); i++)
        if(veclinks[i] != plink2)
            vlinkexcluded.insert(veclinks[i]);

    return CheckCollision(plink1, vbodyexcluded, vlinkexcluded,preport);
}
    
bool CollisionCheckerPQP::CheckCollision(const KinBody::Link* plink, const KinBody* pbody, COLLISIONREPORT* preport)
{
    //does not check for self collision
    std::set<KinBody*> vbodyexcluded;
    std::vector<KinBody *> vecbodies;
    std::set<KinBody::Link *> vlinkexcluded;

    GetEnv()->GetBodies(vecbodies);

    for(int i = 0; i < (int)vecbodies.size(); i++)
        if(pbody != vecbodies[i])
            vbodyexcluded.insert(vecbodies[i]);
    
    return CheckCollision(plink, vbodyexcluded, vlinkexcluded,preport);
}

bool CollisionCheckerPQP::CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, dReal tolerance)
{
	int prevopts= GetCollisionOptions();
	dReal oldtol = _tolerance;
	_benabletol = true;
	_benablecol = false;
	_benabledis = false;
	_tolerance = tolerance;
	COLLISIONREPORT * preport = NULL;
	bool retval = CollisionCheckerPQP::CheckCollision(plink,vbodyexcluded,vlinkexcluded,preport);
	SetCollisionOptions(prevopts);
	_tolerance = oldtol;
	return retval;
}

bool CollisionCheckerPQP::CheckCollision(const KinBody::Link* plink, const std::set<KinBody* >& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* preport)
{
    _preport = preport;
    if(_preport != NULL) {
        _preport->Reset();
        _preport->minDistance = 1e20f;
    }
    
    int tmpnumcols = 0;
    int tmpnumwithintol = 0;
    bool retval;

    std::vector<KinBody *> vecbodies;
    GetEnv()->GetBodies(vecbodies);

    PQP_REAL R1[3][3], R2[3][3], T1[3], T2[3];

    std::vector<Transform> vtrans1,vtrans2;
    plink->GetParent()->GetBodyTransformations(vtrans1);

    KinBody * pbody2;
    bool exclude_link2;
    for(int k = 0; k < (int)vecbodies.size(); k++) {
        if(_preport != NULL) {
            _preport->numWithinTol = 0;
            _preport->numCols = 0;
        }
        pbody2 = vecbodies[k];
        
        if(plink->GetParent() == pbody2 || plink->GetParent()->IsAttached(pbody2) )
            continue;

        if( vbodyexcluded.find(pbody2) != vbodyexcluded.end() )
            continue;

        std::vector<KinBody::Link *> veclinks2 = pbody2->GetLinks();
        
        pbody2->GetBodyTransformations(vtrans2);

           
        GetPQPTransformFromTransform(vtrans1[plink->GetIndex()],R1,T1);

        exclude_link2 = false;
        for(int j = 0; j < (int)vtrans2.size(); j++) {

            if(plink == veclinks2[j])
                continue;

            if( vlinkexcluded.find(veclinks2[j]) != vlinkexcluded.end() )
                continue;

            GetPQPTransformFromTransform(vtrans2[j],R2,T2);

            retval = DoPQP(plink,R1,T1,veclinks2[j],R2,T2);
            if(_preport == NULL && _benablecol == true && _benabledis == false && _benabletol == false && retval)
                return true;
            //return tolerance check result when it is the only thing enabled and there is no report
            if(_preport == NULL && _benablecol == false && _benabledis == false && _benabletol == true && retval)
                return true;
        }
            
        if(_preport != NULL) {
            if(_preport->numWithinTol > 0)
                tmpnumwithintol++;
             
            if(_preport->numCols > 0)
                tmpnumcols++;
        }
    }
    
    if(_preport != NULL) {
        _preport->numWithinTol = tmpnumwithintol;
        _preport->numCols = tmpnumcols;
    }
    
    return tmpnumcols>0;

}

bool CollisionCheckerPQP::CheckCollision(const RAY& ray, const KinBody::Link* plink, COLLISIONREPORT* pReport)
{
    RAVEPRINT(L"Ray collision is not implemented in PQP collision checker!\n");
    assert(0);
    return false;
}

bool CollisionCheckerPQP::CheckCollision(const RAY& ray, const KinBody* pbody, COLLISIONREPORT* pReport)
{
    RAVEPRINT(L"Ray collision is not implemented in PQP collision checker!\n");
    assert(0);
    return false;
}

bool CollisionCheckerPQP::CheckCollision(const RAY& ray, COLLISIONREPORT* pReport)
{
    RAVEPRINT(L"Ray collision is not implemented in PQP collision checker!\n");
    assert(0);
    return false;
}

void CollisionCheckerPQP::GetPQPTransformFromTransform(Transform T, PQP_REAL PQP_R[3][3], PQP_REAL PQP_T[3])
{
    
    TransformMatrix Tfm1(T);

    PQP_R[0][0] = Tfm1.m[0];   PQP_R[0][1] = Tfm1.m[1];   PQP_R[0][2] = Tfm1.m[2];
    PQP_R[1][0] = Tfm1.m[4];   PQP_R[1][1] = Tfm1.m[5];   PQP_R[1][2] = Tfm1.m[6];
    PQP_R[2][0] = Tfm1.m[8];   PQP_R[2][1] = Tfm1.m[9];   PQP_R[2][2] = Tfm1.m[10];
    PQP_T[0] = Tfm1.trans.x;   PQP_T[1] = Tfm1.trans.y;   PQP_T[2] = Tfm1.trans.z;
}

void CollisionCheckerPQP::PQPRealToVector(const Vector& in, const PQP_REAL R[3][3], const PQP_REAL T[3], Vector& out)
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
