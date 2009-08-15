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
        KinBody * pbody1;
        KinBody * pbody2;
    };

    struct KINBODYINFO
    {
        KINBODYINFO() : pbody(NULL), nLastStamp(0) {}
        ~KINBODYINFO();
        KinBody* pbody;
        vector<PQP_Model*> vlinks;
        int nLastStamp;
    };
    
    CollisionCheckerPQP(EnvironmentBase* penv);
    ~CollisionCheckerPQP();

    virtual bool InitEnvironment();
    virtual void DestroyEnvironment();
    virtual bool InitKinBody(KinBody* pbody);
    virtual bool DestroyKinBody(KinBody* pbody);

    virtual bool Enable(const KinBody* pbody, bool bEnable);
    virtual bool EnableLink(const KinBody::Link* pbody, bool bEnable);

    void GetPQPTransformFromTransform(Transform T, PQP_REAL PQP_R[3][3], PQP_REAL PQP_T[3]);

    virtual bool SetCollisionOptions(int collisionoptions);
    virtual int GetCollisionOptions() const { return _options; }

    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) { return false; }

    virtual bool CheckCollision(const KinBody* pbody1, COLLISIONREPORT* pReport);
    virtual bool CheckCollision(const KinBody* pbody1, const KinBody* pbody2, COLLISIONREPORT* pReport);
    bool CheckCollision(const KinBody* pbody1, const std::set<KinBody* >& vexcluded, COLLISIONREPORT* pReport);

    virtual bool CheckCollision(const KinBody::Link* plink, COLLISIONREPORT* pReport);
    virtual bool CheckCollision(const KinBody::Link* plink1, const KinBody::Link* plink2, COLLISIONREPORT* pReport);
    
    bool CheckCollision(const KinBody::Link* plink, const KinBody* pbody, COLLISIONREPORT* pReport);
  
    bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* pReport);
    bool CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* pReport);
    
    virtual bool CheckCollision(const RAY& ray, const KinBody::Link* plink, COLLISIONREPORT* pReport);
    virtual bool CheckCollision(const RAY& ray, const KinBody* pbody, COLLISIONREPORT* pReport);
    virtual bool CheckCollision(const RAY& ray, COLLISIONREPORT* pReport);
	
	virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, dReal tolerance);
	virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, dReal tolerance);
	
    PQP_Model* GetLinkModel(const KinBody::Link* plink);

    virtual bool CheckSelfCollision(const KinBody* pbody, COLLISIONREPORT* pReport = NULL)
    {
        if( pbody == NULL || pbody->GetLinks().size() <= 1 )
            return false;

        // check collision, ignore adjacent bodies
        FOREACHC(itset, pbody->GetNonAdjacentLinks()) {
            assert( (*itset&0xffff) < (int)pbody->GetLinks().size() && (*itset>>16) < (int)pbody->GetLinks().size() );
            if( GetEnv()->CheckCollision(pbody->GetLinks()[*itset&0xffff], pbody->GetLinks()[*itset>>16], pReport) ) {
                RAVELOG_VERBOSEA("selfcol %S, Links %S %S are colliding\n", pbody->GetName(), pbody->GetLinks()[*itset&0xffff]->GetName(), pbody->GetLinks()[*itset>>16]->GetName());
                return true;
            }
        }

        return false;
    }

private:
    // does not check attached
    bool CheckCollisionP(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* pReport);

    void SetOptions(bool CollisionCheckOnOff, bool DistanceCheckOnOff, bool ToleranceCheckOnOff){_benablecol = CollisionCheckOnOff; _benabledis = DistanceCheckOnOff; _benabletol = ToleranceCheckOnOff;}
    void SetTolerance(double tol){_benabletol = true; _tolerance = tol;}

    COLLISIONREPORT* _preport;

    void PQPRealToVector(const Vector& in, const PQP_REAL R[3][3], const PQP_REAL T[3], Vector& out);

    bool DoPQP(const KinBody::Link* link1, PQP_REAL R1[3][3], PQP_REAL T1[3], const KinBody::Link* link2, PQP_REAL R2[3][3], PQP_REAL T2[3]);
    
    int _options;

    //pqp parameters
    PQP_REAL _tolerance;
    PQP_REAL _rel_err;
    PQP_REAL _abs_err;
    
    //collision check results
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
BOOST_TYPEOF_REGISTER_TYPE(CollisionCheckerPQP::KINBODYINFO)
BOOST_TYPEOF_REGISTER_TYPE(PQP_Model)
#endif

#endif   // COLPQP_H
